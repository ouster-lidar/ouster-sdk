/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief ouster_pyclient
 *
 * Note: the type annotations in `client.pyi` need to be updated whenever this
 * file changes. See the mypy documentation for details.
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pyerrors.h>
#include <warnings.h>

#include <tuple>

#include "client_common.h"
#include "ouster/cloud_io.h"
#include "ouster/downsample.h"
#include "ouster/image_processing.h"
#include "ouster/lidar_scan.h"
#include "ouster/normals.h"
#include "ouster/packet.h"
#include "ouster/pose_conversion.h"
#include "ouster/pose_util.h"
#include "ouster/typedefs.h"
#include "ouster/types.h"

namespace py = pybind11;
using ouster::sdk::core::AutoExposure;
using ouster::sdk::core::BeamUniformityCorrector;
using ouster::sdk::core::cartesianT;
using ouster::sdk::core::img_t;
using ouster::sdk::core::LidarScan;
using ouster::sdk::core::Packet;
using ouster::sdk::core::PointsT;
using ouster::sdk::core::PosesT;
using ouster::sdk::core::ScanBatcher;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::XYZLut;
using ouster::sdk::core::XYZLutT;

// alias for non-casting row-major array arguments
template <typename T>
using pyimg_t = py::array_t<T, py::array::c_style>;

// factor out overloaded call operator for ae/buc
template <typename T, typename U>
void image_proc_call(T& self, pyimg_t<U> image, bool update_state) {
    if (image.ndim() != 2) {
        throw std::invalid_argument("Expected a 2d array");
    }
    self(Eigen::Map<img_t<U>>(image.mutable_data(), image.shape(0),
                              image.shape(1)),
         update_state);
}

std::pair<Eigen::Matrix<double, Eigen::Dynamic, 3>,
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
downsample_point_cloud(const py::array_t<double>& voxel_size,
                       const py::array_t<double>& pts,
                       const py::array_t<double>& attributes,
                       int min_points_per_voxel) {
    auto size_buf = voxel_size.request();
    auto size_ptr = static_cast<double*>(size_buf.ptr);
    Eigen::Matrix<double, 3, 1> eigen_voxel_size;
    if (voxel_size.size() == 1) {
        eigen_voxel_size(0, 0) = size_ptr[0];
        eigen_voxel_size(1, 0) = size_ptr[0];
        eigen_voxel_size(2, 0) = size_ptr[0];
    } else if (voxel_size.size() == 3) {
        eigen_voxel_size(0, 0) = size_ptr[0];
        eigen_voxel_size(1, 0) = size_ptr[1];
        eigen_voxel_size(2, 0) = size_ptr[2];
    } else {
        throw std::invalid_argument(
            "Expected a float/double or 3x1 array for voxel size.");
    }

    if (pts.ndim() != 2 || pts.shape(1) != 3) {
        throw std::invalid_argument("Points array must have a shape of Nx3");
    }

    py::array_t<double> c_style_points;
    py::array_t<double> c_style_attrs;

    // Create a C-style copy of arrays if it's neither C-style nor F-style
    const py::array_t<double>* points_ptr = &pts;
    if ((pts.flags() & py::array::c_style) == 0) {
        c_style_points = py::array_t<double, py::array::c_style>(pts);
        points_ptr = &c_style_points;  // Use the C-style array for processing
    }
    const py::array_t<double>* attr_ptr = &attributes;
    if ((attributes.flags() & py::array::c_style) == 0) {
        c_style_attrs = py::array_t<double, py::array::c_style>(attributes);
        attr_ptr = &c_style_attrs;  // Use the C-style array for processing
    }

    auto pts_buf = points_ptr->request();
    auto attr_buf = attr_ptr->request();

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>
        points_mat(static_cast<double*>(pts_buf.ptr), pts_buf.shape[0],
                   pts_buf.shape[1]);

    Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        attr_mat(static_cast<double*>(attr_buf.ptr), attr_buf.shape[0],
                 attr_buf.shape[1]);

    Eigen::Matrix<double, Eigen::Dynamic, 3> eigen_out_pts;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eigen_out_attr;
    ouster::sdk::core::voxel_downsample(eigen_voxel_size, points_mat, attr_mat,
                                        eigen_out_pts, eigen_out_attr,
                                        min_points_per_voxel);

    return {eigen_out_pts, eigen_out_attr};
}

/**
 * Applies a set of 4x4 pose transformations to a collection of 3D points,
 * reshapes the input into appropriate Eigen matrices, and invokes the C++
 * version of the `dewarp` function for performing the transformations.
 *
 * This function is designed to convert NumPy input arrays from Python into
 * Eigen matrices, which are suitable for efficient matrix operations in C++.
 * The 3D points are passed in the shape of (H, W, 3), and the 4x4 pose matrices
 * are passed in the shape of (W, 4, 4).
 *
 * @param[in] points A NumPy array of shape (H, W, 3) representing the 3D
 * points.
 * - H: Number of columns (groups of points)
 * - W: Number of points per column
 * - 3: 3D coordinates (x, y, z)
 *
 * @param[in] poses A NumPy array of shape (W, 4, 4) representing the 4x4 pose
 * matrices.
 * - W: Number of pose matrices
 * - 4x4: The transformation matrices
 *
 * @return A NumPy array of shape (H, W, 3) containing the dewarped 3D points
 * after applying the corresponding 4x4 transformation matrices to the points.
 *
 */

template <typename T>
py::array_t<T> dewarp(const py::array_t<T>& points,
                      const py::array_t<T>& poses) {
    auto poses_buf = poses.request();
    auto points_buf = points.request();

    // Validate poses dims: (W, 4, 4)
    if (poses_buf.ndim != 3 || poses_buf.shape[1] != 4 ||
        poses_buf.shape[2] != 4) {
        throw std::runtime_error("Invalid shape for poses, expected (W, 4, 4)");
    }

    // Validate points dims: (H, W, 3)
    if (points_buf.ndim != 3 || points_buf.shape[2] != 3) {
        throw std::runtime_error(
            "Invalid shape for points, expected (H, W, 3)");
    }

    const int num_poses = poses_buf.shape[0];            // W
    const int num_rows = points_buf.shape[0];            // H
    const int num_points_per_col = points_buf.shape[1];  // W
    const int point_dim = 3;

    if (num_points_per_col != num_poses) {
        throw std::runtime_error(
            "Number of points per set must match number of poses");
    }

    py::array_t<T> c_style_points{};
    py::array_t<T> c_style_poses{};

    // Create a C-style copy of arrays if it's neither C-style nor F-style
    const py::array_t<T>* points_ptr = &points;
    if ((points.flags() & py::array::c_style) == 0) {
        c_style_points = py::array_t<T, py::array::c_style>(points);
        points_ptr = &c_style_points;  // Use the C-style array for processing
    }
    const py::array_t<T>* poses_ptr = &poses;
    if ((poses.flags() & py::array::c_style) == 0) {
        c_style_poses = py::array_t<T, py::array::c_style>(poses);
        poses_ptr = &c_style_poses;  // Use the C-style array for processing
    }

    // Map the poses and points to Eigen matrices with zero-copy approach
    auto poses_buf_ptr = poses_ptr->request();

    // Always use zero-copy mapping with custom strides - no fallback copying
    // needed! Create a strided Eigen::Map that can handle any memory layout
    using StrideType = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
    StrideType pose_stride(
        poses_buf_ptr.strides[0] / sizeof(T),  // outer stride (between poses)
        poses_buf_ptr.strides[2] /
            sizeof(T)  // inner stride (between elements within 4x4)
    );

    // Map poses as a matrix where each row is a flattened 4x4 pose matrix
    // This handles both contiguous and non-contiguous layouts without copying
    Eigen::Map<
        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        0, StrideType>
        poses_strided_map{poses_ptr->data(), num_poses, 16, pose_stride};

    // Convert to the expected PosesT format (this is just a view, no copy)
    PosesT<T> poses_eigen = poses_strided_map;

    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>>
        points_map{points_ptr->data(), num_rows * num_points_per_col, 3};

    // Create C-style output array to ensure consistent memory layout
    std::vector<ssize_t> result_shape = {num_rows, num_poses, point_dim};
    auto result = py::array_t<T, py::array::c_style>(result_shape);
    auto result_buf = result.request();
    Eigen::Map<PointsT<T>> dewarped_points{static_cast<T*>(result_buf.ptr),
                                           num_rows * num_poses, point_dim};

    // Call your templated, in-place dewarp
    ouster::sdk::core::dewarp<T>(dewarped_points, points_map, poses_eigen);

    return result;
}

/**
 * Applies a single of 4x4 pose transformations to a collection of 3D points,
 * reshapes the input into appropriate Eigen matrices, and invokes the C++
 * version of the `transfrom` function for performing the transformations.
 *
 * This function is designed to convert NumPy input arrays from Python into
 * Eigen matrices, which are suitable for efficient matrix operations in C++.
 * The 3D points are passed in the shape of (H, W, 3), and the single 4x4 pose
 * matrics in the shape of (4, 4).
 *
 * @param[in] points A NumPy array of shape (H, W, 3), or (N, 3)
 * representing the 3D points.
 * - H: Number of columns (groups of points)
 * - W: Number of points per column
 * - 3: 3D coordinates (x, y, z)
 *
 * @param[in] poses A NumPy array of shape (4, 4) representing the 4x4 pose
 * matrices.
 * - 4x4: The transformation matrices
 *
 * @return A NumPy array of shape (H, W, 3) or (N, 3) containing the transformed
 * 3D points after applying the corresponding 4x4 transformation matrices to the
 * points.
 *
 */
template <typename T>
py::array_t<T> transform(const py::array_t<T>& points,
                         const py::array_t<T>& pose) {
    // Ensure the pose is a 4x4 matrix
    if (pose.ndim() != 2 || pose.shape(0) != 4 || pose.shape(1) != 4) {
        throw std::runtime_error("pose array must have shape (4, 4)");
    }

    py::array_t<T> c_style_points{};
    py::array_t<T> c_style_pose{};

    // Create a C-style copy of points if it's neither C-style nor F-style
    const py::array_t<T>* points_ptr = &points;
    if (!(points.flags() & py::array::c_style)) {
        c_style_points = py::array_t<T, py::array::c_style>(points);
        points_ptr = &c_style_points;  // Use the C-style array for processing
    }
    const py::array_t<T>* pose_ptr = &pose;
    if (!(pose.flags() & py::array::c_style)) {
        c_style_pose = py::array_t<T, py::array::c_style>(pose);
        pose_ptr = &c_style_pose;  // Use the C-style array for processing
    }

    // Convert pose to Eigen format
    Eigen::Map<const Eigen::Matrix<T, 1, 16, Eigen::RowMajor>> pose_eigen{
        pose_ptr->data()};

    // Handle case where points is a 2D array: (N, 3)
    if (points_ptr->ndim() == 2 && points_ptr->shape(1) == 3) {
        const int num_points = points_ptr->shape(0);

        // Define a matrix type for points using the template parameter
        using PointsMatrix =
            Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;
        Eigen::Map<const PointsMatrix> points_eigen{points_ptr->data(),
                                                    num_points, 3};

        auto result = py::array_t<T>({num_points, 3});
        auto result_buf = result.request();
        Eigen::Map<PointsMatrix> transformed{static_cast<T*>(result_buf.ptr),
                                             num_points, 3};

        ouster::sdk::core::transform<T>(transformed, points_eigen, pose_eigen);
        return result;
    }

    // Handle case where points is a 3D array: (H, W, 3)
    else if (points_ptr->ndim() == 3 && points_ptr->shape(2) == 3) {
        const int h = points_ptr->shape(0);
        const int w = points_ptr->shape(1);

        // Define a matrix type for points using the template parameter
        using PointsMatrix =
            Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;
        Eigen::Map<const PointsMatrix> points_eigen{points_ptr->data(), h * w,
                                                    3};

        auto result = py::array_t<T>({h, w, 3});
        auto result_buf = result.request();
        Eigen::Map<PointsMatrix> transformed{static_cast<T*>(result_buf.ptr),
                                             h * w, 3};

        ouster::sdk::core::transform<T>(transformed, points_eigen, pose_eigen);
        return result;
    } else {
        throw std::invalid_argument(
            "points array must have shape (n, 3) or (h, w, 3)");
    }
}

/**
 * Interpolates 4x4 pose matrices at given x-coordinate values.
 *
 * @param[in] x_interp NumPy array of shape (N, 1) or (N,) representing
 * interpolation x-coordinate values.
 * @param[in] x_known NumPy array of shape (M, 1) or (M,) representing known
 * x-coordinate values.
 * @param[in] poses_known NumPy array of shape (M, 4, 4) representing known pose
 * matrices.
 *
 * @return NumPy array of shape (N, 4, 4) containing the interpolated 4x4 pose
 * matrices.
 */

// Templated interpolate_pose for pybind11, matching pose_util.h: TX for x,
// TPOSE for pose
template <typename TX, typename TPOSE>
py::array_t<TPOSE> interp_pose(const py::array_t<TX>& x_interp,
                               const py::array_t<TX>& x_known,
                               const py::array_t<TPOSE>& poses_known) {
    // Ensure C-style for poses_known
    const py::array_t<TPOSE>* poses_known_ptr = &poses_known;
    py::array_t<TPOSE> c_style_poses{};
    if ((poses_known.flags() & py::array::c_style) == 0) {
        c_style_poses = py::array_t<TPOSE, py::array::c_style>(poses_known);
        poses_known_ptr = &c_style_poses;
    }

    py::buffer_info x_interp_buf = x_interp.request();
    py::buffer_info x_known_buf = x_known.request();
    py::buffer_info poses_known_buf = poses_known_ptr->request();

    // Check input dimensions
    if (x_interp_buf.ndim != 1 &&
        (x_interp_buf.ndim != 2 || x_interp_buf.shape[1] != 1)) {
        throw std::runtime_error("x_interp must have shape (N,) or (N,1)");
    }
    if (x_known_buf.ndim != 1 &&
        (x_known_buf.ndim != 2 || x_known_buf.shape[1] != 1)) {
        throw std::runtime_error("x_known must have shape (N,) or (N,1)");
    }
    if (poses_known_buf.ndim != 3 || poses_known_buf.shape[1] != 4 ||
        poses_known_buf.shape[2] != 4) {
        throw std::runtime_error(
            "poses_known must be a 3D array with shape (M, 4, 4)");
    }

    size_t n_dim = static_cast<size_t>(x_interp_buf.shape[0]);
    size_t m_dim = static_cast<size_t>(x_known_buf.shape[0]);
    size_t poses_dim = static_cast<size_t>(poses_known_buf.shape[0]);
    if (m_dim != poses_dim) {
        throw std::runtime_error(
            "The number of poses in poses_known must match the number of "
            "values in x_known");
    }

    // Map input data directly using Eigen::Map for x_interp and x_known
    TX* x_interp_ptr = static_cast<TX*>(x_interp_buf.ptr);
    TX* x_known_ptr = static_cast<TX*>(x_known_buf.ptr);

    const Eigen::Index n_dim_idx = static_cast<Eigen::Index>(n_dim);
    const Eigen::Index m_dim_idx = static_cast<Eigen::Index>(m_dim);
    Eigen::Map<const Eigen::Matrix<TX, Eigen::Dynamic, 1>> x_interp_map{
        x_interp_ptr, n_dim_idx};
    Eigen::Map<const Eigen::Matrix<TX, Eigen::Dynamic, 1>> x_known_map{
        x_known_ptr, m_dim_idx};

    // Map poses_known to PosesT format (flattened 4x4 matrices)
    TPOSE* poses_ptr = static_cast<TPOSE*>(poses_known_buf.ptr);
    Eigen::Map<const Eigen::Matrix<TPOSE, Eigen::Dynamic, 16, Eigen::RowMajor>>
        poses_map{poses_ptr, m_dim_idx, 16};

    // Call templated C++ core::interp_pose
    auto result_poses = ouster::sdk::core::interp_pose<TX, TPOSE>(
        x_interp_map, x_known_map, poses_map);

    // Create C-style output array with shape (N, 4, 4) to ensure consistent
    // memory layout
    std::vector<ssize_t> shape = {static_cast<ssize_t>(result_poses.rows()), 4,
                                  4};
    auto out = py::array_t<TPOSE, py::array::c_style>(shape);
    auto out_buf = out.request();

    // Map output array as N×16 row-major matrix and assign directly from
    // result_poses
    Eigen::Map<Eigen::Matrix<TPOSE, Eigen::Dynamic, 16, Eigen::RowMajor>>
        out_map{static_cast<TPOSE*>(out_buf.ptr), result_poses.rows(), 16};
    out_map = result_poses;

    return out;
}

Eigen::Map<const ouster::sdk::core::MatrixX3dR> to_matrixx3d(
    const py::array& arr) {
    if (arr.ndim() != 2 || arr.shape(1) != 3) {
        throw std::invalid_argument("Expected a 2D array with shape (N, 3)");
    }
    const Eigen::Index num_points = static_cast<Eigen::Index>(arr.shape(0));
    py::array_t<double, py::array::c_style | py::array::forcecast> c_style_arr{
        arr};
    return Eigen::Map<const ouster::sdk::core::MatrixX3dR>(
        static_cast<const double*>(c_style_arr.data()), num_points, 3);
}

Eigen::Map<const ouster::sdk::core::PointCloudXYZd> to_pointcloudxyzd(
    const py::array& arr) {
    if (arr.ndim() != 3 || arr.shape(2) != 3) {
        throw std::invalid_argument("Expected a 3D array with shape (H, W, 3)");
    }
    const Eigen::Index num_points = static_cast<Eigen::Index>(arr.shape(0)) *
                                    static_cast<Eigen::Index>(arr.shape(1));
    py::array_t<double, py::array::c_style | py::array::forcecast> c_style_arr{
        arr};
    return Eigen::Map<const ouster::sdk::core::PointCloudXYZd>(
        static_cast<const double*>(c_style_arr.data()), num_points, 3);
}

Eigen::Map<const ouster::sdk::core::img_t<uint32_t>> to_imgt_uint32(
    const py::array& arr) {
    if (arr.ndim() != 2) {
        throw std::invalid_argument("Expected a 2D array");
    }
    py::array_t<uint32_t, py::array::c_style | py::array::forcecast>
        c_style_arr{arr};
    return Eigen::Map<const ouster::sdk::core::img_t<uint32_t>>(
        static_cast<const uint32_t*>(c_style_arr.data()), arr.shape(0),
        arr.shape(1));
}

void init_client_processing(py::module& module, py::module& /*unused*/) {
    // Destagger overloads for most numpy scalar types
    module.def("destagger_bool", &ouster::sdk::core::destagger<bool>);
    module.def("destagger_int8", &ouster::sdk::core::destagger<int8_t>);
    module.def("destagger_int16", &ouster::sdk::core::destagger<int16_t>);
    module.def("destagger_int32", &ouster::sdk::core::destagger<int32_t>);
    module.def("destagger_int64", &ouster::sdk::core::destagger<int64_t>);
    module.def("destagger_uint8", &ouster::sdk::core::destagger<uint8_t>);
    module.def("destagger_uint16", &ouster::sdk::core::destagger<uint16_t>);
    module.def("destagger_uint32", &ouster::sdk::core::destagger<uint32_t>);
    module.def("destagger_uint64", &ouster::sdk::core::destagger<uint64_t>);
    module.def("destagger_float", &ouster::sdk::core::destagger<float>);
    module.def("destagger_double", &ouster::sdk::core::destagger<double>);

    module.def(
        "normals",
        [](const py::array& xyz_arr, const py::array& range_arr,
           const py::array& sensor_origins_obj, int pixel_search_range,
           double min_angle_of_incidence_rad, double target_distance_m) {
            int h = static_cast<int>(xyz_arr.shape(0));
            int w = static_cast<int>(xyz_arr.shape(1));
            int num_points = h * w;

            auto normals_matrix = ouster::sdk::core::normals(
                to_pointcloudxyzd(xyz_arr), to_imgt_uint32(range_arr),
                to_matrixx3d(sensor_origins_obj), pixel_search_range,
                min_angle_of_incidence_rad, target_distance_m);

            // Create output array with correct shape (H, W, 3)
            auto result = py::array_t<double>({h, w, 3});
            auto result_buf = result.request();

            // Map the output array to Eigen matrix and copy the data
            Eigen::Map<ouster::sdk::core::PointCloudXYZd> normals_output_map{
                static_cast<double*>(result_buf.ptr), num_points, 3};
            normals_output_map = normals_matrix;

            return result;
        },
        R"doc(
Compute normals from destaggered XYZ/range arrays.

Args:
    xyz: destaggered XYZ coordinates for the first return (H, W, 3)
    range: destaggered range image for the first return (H, W)
    sensor_origins_xyz: per-column sensor origins in the same frame as xyz/range,
        shape (W, 3). For world-frame xyz, use
        (scan.pose @ scan.sensor_info.extrinsic)[:, :3, 3].
        For sensor-frame xyz, pass zeros with shape (W, 3) (e.g. np.zeros((w, 3))).
    pixel_search_range: axial search radius (in pixels) when gathering neighbours
    min_angle_of_incidence_rad: minimum allowable incidence angle between a beam and
        surface (radians) (default: 1 deg, ~0.01745 rad)
    target_distance_m: target neighbour distance used when selecting candidate points

Returns:
    A destaggered normal array of shape (H, W, 3) for the provided return.
)doc",
        py::arg("xyz"), py::arg("range"), py::arg("sensor_origins_xyz"),
        py::arg("pixel_search_range") = 1,
        py::arg("min_angle_of_incidence_rad") =
            ouster::sdk::core::DEFAULT_MIN_ANGLE_INCIDENCE_RAD,
        py::arg("target_distance_m") =
            ouster::sdk::core::DEFAULT_TARGET_DISTANCE_METER);

    module.def(
        "normals",
        [](const py::array& xyz_arr, const py::array& range_arr,
           const py::array& xyz2_arr, const py::array& range2_arr,
           const py::array& sensor_origins_obj, int pixel_search_range,
           double min_angle_of_incidence_rad, double target_distance_m) {
            py::array_t<uint32_t, py::array::c_style | py::array::forcecast>
                range(range_arr);
            py::array_t<uint32_t, py::array::c_style | py::array::forcecast>
                range2(range2_arr);

            int h = static_cast<int>(xyz_arr.shape(0));
            int w = static_cast<int>(xyz_arr.shape(1));
            int num_points = h * w;

            auto normals_pair = ouster::sdk::core::normals(
                to_pointcloudxyzd(xyz_arr), to_imgt_uint32(range_arr),
                to_pointcloudxyzd(xyz2_arr), to_imgt_uint32(range2_arr),
                to_matrixx3d(sensor_origins_obj), pixel_search_range,
                min_angle_of_incidence_rad, target_distance_m);

            // Create output arrays with correct shape (H, W, 3)
            auto first = py::array_t<double>({h, w, 3});
            auto second = py::array_t<double>({h, w, 3});
            auto first_buf = first.request();
            auto second_buf = second.request();

            // Map the output arrays to Eigen matrices and copy the data
            Eigen::Map<ouster::sdk::core::PointCloudXYZd> first_output_map{
                static_cast<double*>(first_buf.ptr), num_points, 3};
            Eigen::Map<ouster::sdk::core::PointCloudXYZd> second_output_map{
                static_cast<double*>(second_buf.ptr), num_points, 3};

            first_output_map = normals_pair.first;
            second_output_map = normals_pair.second;

            return std::make_tuple(first, second);
        },
        R"doc(
Compute normals for both first and second returns from destaggered XYZ/range arrays.

Args:
    xyz: destaggered XYZ coordinates for the first return (H, W, 3)
    range: destaggered range image for the first return (H, W)
    xyz2: destaggered XYZ coordinates for the second return (H, W, 3)
    range2: destaggered range image for the second return (H, W)
    sensor_origins_xyz: per-column sensor origins in the same frame as xyz/range,
        shape (W, 3). For world-frame xyz, use
        (scan.pose @ scan.sensor_info.extrinsic)[:, :3, 3].
        For sensor-frame xyz, pass zeros.
    pixel_search_range: axial search radius (in pixels) when gathering neighbours
    min_angle_of_incidence_rad: minimum allowable incidence angle between a beam and
        surface (radians) (default: 1 deg, ~0.01745 rad)
    target_distance_m: target neighbour distance used when selecting candidate points

Returns:
    A tuple of destaggered normal arrays (first_return_normals, second_return_normals).
)doc",
        py::arg("xyz"), py::arg("range"), py::arg("xyz2"), py::arg("range2"),
        py::arg("sensor_origins_xyz"), py::arg("pixel_search_range") = 1,
        py::arg("min_angle_of_incidence_rad") =
            ouster::sdk::core::DEFAULT_MIN_ANGLE_INCIDENCE_RAD,
        py::arg("target_distance_m") =
            ouster::sdk::core::DEFAULT_TARGET_DISTANCE_METER);

    py::class_<ScanBatcher>(module, "ScanBatcher")
        .def(py::init<std::shared_ptr<SensorInfo>>())
        .def("reset", &ScanBatcher::reset)
        .def("batched_packets", &ScanBatcher::batched_packets)
        .def("__call__",
             [](ScanBatcher& self, Packet& packet, LidarScan& lidar_scan) {
                 return self(packet, lidar_scan);
             });

    // XYZ Projection
    py::class_<XYZLutT<float>>(module, "XYZLutFloat")
        .def(py::init([](const SensorInfo& sensor, bool use_extrinsics) {
                 XYZLutT<float> lut = XYZLutT<float>(sensor, use_extrinsics);
                 return lut;
             }),
             py::arg("info"), py::arg("use_extrinsics"))
        .def("__call__",
             [](const XYZLutT<float>& self,
                const Eigen::Ref<const img_t<uint32_t>>& range) {
                 return cartesianT<float>(range, self.direction, self.offset);
             })
        .def("__call__",
             [](const XYZLutT<float>& self, const LidarScan& scan) {
                 return cartesianT<float>(scan, self.direction, self.offset);
             })
        .def_property_readonly(
            "direction", [](const XYZLut& self) { return self.direction; })
        .def_property_readonly("offset",
                               [](const XYZLut& self) { return self.offset; });

    py::class_<XYZLutT<double>>(module, "XYZLut")
        .def(py::init([](const SensorInfo& sensor, bool use_extrinsics) {
                 return ouster::sdk::core::make_xyz_lut(sensor, use_extrinsics);
             }),
             py::arg("info"), py::arg("use_extrinsics"))
        .def("__call__",
             [](const XYZLutT<double>& self,
                const Eigen::Ref<const img_t<uint32_t>>& range) {
                 return cartesianT<double>(range, self.direction, self.offset);
             })
        .def("__call__",
             [](const XYZLutT<double>& self, const LidarScan& scan) {
                 return cartesianT<double>(scan, self.direction, self.offset);
             });

    // Image processing
    py::class_<AutoExposure>(module, "AutoExposure")
        .def(py::init<>())
        .def(py::init<int>(), py::arg("update_every"))
        .def(py::init<double, double, int>(), py::arg("lo_percentile"),
             py::arg("hi_percentile"), py::arg("update_every"))
        .def("__call__", &image_proc_call<AutoExposure, float>,
             py::arg("image"), py::arg("update_state") = true)
        .def("__call__", &image_proc_call<AutoExposure, double>,
             py::arg("image"), py::arg("update_state") = true);

    py::class_<BeamUniformityCorrector>(module, "BeamUniformityCorrector")
        .def(py::init<>())
        .def("__call__", &image_proc_call<BeamUniformityCorrector, float>,
             py::arg("image"), py::arg("update_state") = true)
        .def("__call__", &image_proc_call<BeamUniformityCorrector, double>,
             py::arg("image"), py::arg("update_state") = true);

    module.def("dewarp",
               py::overload_cast<const py::array_t<double>&,
                                 const py::array_t<double>&>(&dewarp<double>),
               R"(
Applies a set of 4x4 pose transformations to a collection of 3D points.
Args:
    points: A NumPy array of shape (H, W, 3) representing the 3D points.
    poses: A NumPy array of shape (W, 4, 4) representing the 4x4 pose

Return:
    A NumPy array of shape (H, W, 3) containing the dewarped 3D points
    )",
               py::arg("points"), py::arg("poses"));

    module.def(
        "dewarp",
        py::overload_cast<const py::array_t<float>&, const py::array_t<float>&>(
            &dewarp<float>),
        R"(
Applies a set of 4x4 pose transformations to a collection of 3D points (float precision).
Args:
    points: A NumPy array of shape (H, W, 3) representing the 3D points (float32).
    poses: A NumPy array of shape (W, 4, 4) representing the 4x4 pose (float32)

Return:
    A NumPy array of shape (H, W, 3) containing the dewarped 3D points (float32)
    )",
        py::arg("points"), py::arg("poses"));

    module.def("transform", &transform<double>,
               R"(
    Applies a single of 4x4 pose transformations to a collection of 3D points.
    Args:
    points: A NumPy array of shape (H, W, 3), or (N, 3)
    pose: A NumPy array of shape (4, 4) representing the 4x4 pose

    Return:
    A NumPy array of shape (H, W, 3) or (N, 3) containing the transformed 3D points
    after applying the corresponding 4x4 transformation matrices to the points
    )",
               py::arg("points"), py::arg("pose"));
    module.def("transform", &transform<float>,
               R"(
    Applies a single of 4x4 pose transformations to a collection of 3D points (float precision).
    Args:
    points: A NumPy array of shape (H, W, 3), or (N, 3) (float32)
    pose: A NumPy array of shape (4, 4) representing the 4x4 pose (float32)

    Return:
    A NumPy array of shape (H, W, 3) or (N, 3) containing the transformed 3D points (float32)
    after applying the corresponding 4x4 transformation matrices to the points
    )",
               py::arg("points"), py::arg("pose"));

    module.def("euler_pose_to_matrix", &ouster::sdk::core::euler_pose_to_matrix,
               R"(
        Convert a pose given in Euler angles and translation to a 4x4 transformation matrix.

        The pose vector should contain the following elements in order:
            [roll, pitch, yaw, x, y, z]
        where roll, pitch, and yaw are in radians.

        Returns:
            A 4x4 homogeneous transformation matrix.
        )");

    module.def("quaternion_pose_to_matrix",
               &ouster::sdk::core::quaternion_pose_to_matrix,
               R"(
        Convert a pose given as a quaternion and translation to a 4x4 transformation matrix.

        The pose vector should contain the following elements in order:
            [qw, qx, qy, qz, x, y, z]

        Returns:
            A 4x4 homogeneous transformation matrix.
        )");

    module.def("voxel_downsample", &downsample_point_cloud,
               py::arg("voxel_size"), py::arg("pts"), py::arg("attributes"),
               py::arg("min_points_per_voxel") = 1,
               R"(
        [BETA] Downsample a pointcloud using a voxel grid of the requested resolution.

        Args:
            voxel_size: The size of the voxel grid.
            pts: Nx3 matrix of points to downsample.
            attributes: A dictionary of attributes to downsample.
            min_points_per_voxel: Minimum number of points per voxel to keep.

        Returns:
            A tuple containing the downsampled points and attributes.

        Note:
            This is a beta feature and its API may change in future releases.
        )");

    module.def("read_pointcloud", &ouster::sdk::core::read_pointcloud,
               R"(
        [BETA] Loads the 3D X Y and Z points from a PCD or PLY file and returns
        them as Nx3 matrix.

        Args:
            filename: filename to load

        Returns:
            Nx3 matrix of the resulting points.

        Note:
            This is a beta feature and its API may change in future releases.
        )");

    module.def("interp_pose",
               py::overload_cast<const py::array_t<double>&,
                                 const py::array_t<double>&,
                                 const py::array_t<double>&>(
                   &interp_pose<double, double>),
               py::arg("x_interp"), py::arg("x_known"), py::arg("poses_known"),
               R"(
        Interpolate 4x4 pose matrices at given x-coordinate values (double precision).
        Args:
            x_interp: (N,) or (N,1) array of interpolation x values (float64)
            x_known: (M,) or (M,1) array of known x values (float64)
            poses_known: (M, 4, 4) array of known pose matrices (float64)
        Returns:
            (N, 4, 4) array of interpolated pose matrices (float64)
        )");

    module.def("interp_pose_float",
               py::overload_cast<const py::array_t<double>&,
                                 const py::array_t<double>&,
                                 const py::array_t<float>&>(
                   &interp_pose<double, float>),
               py::arg("x_interp"), py::arg("x_known"), py::arg("poses_known"),
               R"(
        Interpolate 4x4 pose matrices at given x-coordinate values (float precision for poses).
        Args:
            x_interp: (N,) or (N,1) array of interpolation x values (float64)
            x_known: (M,) or (M,1) array of known x values (float64)
            poses_known: (M, 4, 4) array of known pose matrices (float32)
        Returns:
            (N, 4, 4) array of interpolated pose matrices (float32)
        )");

    auto mesh_cls = py::class_<ouster::sdk::core::Mesh>(module, "Mesh");
    mesh_cls.def(py::init<>())
        .def(py::init<std::vector<ouster::sdk::core::Triangle>>())
        .def("load_from_stl", &ouster::sdk::core::Mesh::load_from_stl)
        .def_property_readonly("triangles",
                               &ouster::sdk::core::Mesh::triangles);

    auto coord_cls = py::class_<ouster::sdk::core::Coord>(module, "Coord");
    (void)coord_cls;

    auto tri_cls = py::class_<ouster::sdk::core::Triangle>(module, "Triangle");
    tri_cls
        .def(py::init<const ouster::sdk::core::Coord&,
                      const ouster::sdk::core::Coord&,
                      const ouster::sdk::core::Coord&>())
        .def_readwrite("coords", &ouster::sdk::core::Triangle::coords)
        .def_readwrite("edges", &ouster::sdk::core::Triangle::edges)
        .def_readwrite("normal", &ouster::sdk::core::Triangle::normal);
}

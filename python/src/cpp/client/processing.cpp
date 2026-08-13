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

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <pyerrors.h>
#include <warnings.h>

#include <stdexcept>
#include <tuple>

#include "client_common.h"
#include "eigen_dense.h"
#include "ndarray_helpers.h"
#include "ouster/core/cloud_io.h"
#include "ouster/core/image_processing.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/object_util.h"
#include "ouster/core/packet.h"
#include "ouster/core/pose_conversion.h"
#include "ouster/core/pose_util.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/types.h"
#include "ouster/core/voxel_hash_map.h"

using ouster::sdk::core::FrameBatcher;
using ouster::sdk::core::img_t;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::MatrixX16R;
using ouster::sdk::core::Object;
using ouster::sdk::core::Packet;
using ouster::sdk::core::PointCloudXYZ;
using ouster::sdk::core::rgb_img_t;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::XYZLut;
using ouster::sdk::core::XYZLutT;
using ouster::sdk::core::image::AutoExposure;
using ouster::sdk::core::image::BeamUniformityCorrector;
using ouster::sdk::core::image::LocalToneMapper;
using ouster::sdk::core::impl::cartesianT;
using ouster::sdk::python::ensure_c_contig;
using ouster::sdk::python::ensure_c_contig_floating;

namespace {
// alias for non-casting row-major array arguments
template <typename T>
using pyimg_t = py::ndarray<T, py::ndim<2>, py::c_contig>;

template <typename T, typename U>
void image_proc_call(T& self, pyimg_t<U> image, bool update_state) {
    self.update(Eigen::Map<img_t<U>>(image.data(), image.shape(0), image.shape(1)), update_state);
}

template <typename T>
void auto_exposure_update(AutoExposure& self, py::ndarray<T, py::ndim<3>, py::c_contig> image,
                          bool update_state) {
    if (image.ndim() != 3 || image.shape(2) != 3) {
        throw std::invalid_argument("Expected an H x W x 3 array");
    }
    const auto h = static_cast<Eigen::Index>(image.shape(0));
    const auto w = static_cast<Eigen::Index>(image.shape(1));

    // rgb_img_t is RowMajor, matching numpy C-contiguous layout
    Eigen::TensorMap<rgb_img_t<T>> tensor_map(image.data(), h, w, 3);

    self.update(tensor_map, update_state);
}

template <typename T>
py::capsule make_capsule(T* data) {
    return py::capsule(data, [](void* pointer) noexcept { delete[] static_cast<T*>(pointer); });
}

template <typename T>
py::ndarray<py::numpy, T, py::c_contig> make_array(const std::vector<size_t>& shape) {
    size_t size = 0;
    for (const auto& dim : shape) {
        if (size == 0) {
            size = dim;
            continue;
        } else {
            size *= dim;
        }
    }
    T* data = new T[size];

    py::ndarray<py::numpy, T, py::c_contig> result(data, shape.size(), shape.data(),
                                                   make_capsule(data));

    return result;
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
py::ndarray<py::numpy, T, py::c_contig> dewarp(
    const py::ndarray<const T, py::c_contig, py::shape<-1, -1, 3>>& points,
    const py::ndarray<const T, py::c_contig, py::shape<-1, 4, 4>>& poses) {
    const int num_poses = poses.shape(0);            // W
    const int num_rows = points.shape(0);            // H
    const int num_points_per_col = points.shape(1);  // W
    const int point_dim = 3;

    if (num_points_per_col != num_poses) {
        throw std::runtime_error("Number of points per set must match number of poses");
    }

    // Map poses as a matrix where each row is a flattened 4x4 pose matrix
    // This handles both contiguous and non-contiguous layouts without copying
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        poses_strided_map{poses.data(), num_poses, 16};

    // Convert to the expected PosesT format (this is just a view, no copy)
    MatrixX16R<T> poses_eigen = poses_strided_map;

    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>> points_map{
        points.data(), num_rows * num_points_per_col, 3};

    py::ndarray<py::numpy, T, py::c_contig> result =
        make_array<T>({static_cast<size_t>(num_rows), static_cast<size_t>(num_poses),
                       static_cast<size_t>(point_dim)});

    Eigen::Map<PointCloudXYZ<T>> dewarped_points(result.data(), num_rows * num_poses, point_dim);

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
py::ndarray<py::numpy, T, py::c_contig> transform(
    const py::ndarray<const T, py::c_contig>& points,
    const py::ndarray<const T, py::c_contig, py::shape<4, 4>>& pose) {
    // Convert pose to Eigen format
    Eigen::Map<const Eigen::Matrix<T, 1, 16, Eigen::RowMajor>> pose_eigen{pose.data()};

    // Handle case where points is a 2D array: (N, 3)
    Eigen::Index num_points = 0;
    py::ndarray<py::numpy, T, py::c_contig> result;
    if (points.ndim() == 2 && points.shape(1) == 3) {
        num_points = points.shape(0);

        result = make_array<T>({static_cast<size_t>(num_points), 3});
    }
    // Handle case where points is a 3D array: (H, W, 3)
    else if (points.ndim() == 3 && points.shape(2) == 3) {
        const Eigen::Index h = points.shape(0);
        const Eigen::Index w = points.shape(1);
        num_points = h * w;

        result = make_array<T>({static_cast<size_t>(h), static_cast<size_t>(w), 3});
    } else {
        throw std::invalid_argument("points array must have shape (n, 3) or (h, w, 3)");
    }

    // Define a matrix type for points using the template parameter
    using PointsMatrix = Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;
    Eigen::Map<const PointsMatrix> points_eigen{points.data(), num_points, 3};

    Eigen::Map<PointsMatrix> transformed{static_cast<T*>(result.data()), num_points, 3};

    ouster::sdk::core::transform<T>(transformed, points_eigen, pose_eigen);
    return result;
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
template <typename TX, typename TPOSE>
py::ndarray<py::numpy, TPOSE, py::c_contig> interp_pose(
    const py::ndarray<const TX, py::c_contig>& x_interp,
    const py::ndarray<const TX, py::c_contig>& x_known,
    const py::ndarray<const TPOSE, py::c_contig, py::shape<-1, 4, 4>>& poses_known) {
    // Check input dimensions
    if (x_interp.ndim() != 1 && (x_interp.ndim() != 2 || x_interp.shape(1) != 1)) {
        throw std::runtime_error("x_interp must have shape (N,) or (N,1)");
    }
    if (x_known.ndim() != 1 && (x_known.ndim() != 2 || x_known.shape(1) != 1)) {
        throw std::runtime_error("x_known must have shape (N,) or (N,1)");
    }

    size_t n_dim = static_cast<size_t>(x_interp.shape(0));
    size_t m_dim = static_cast<size_t>(x_known.shape(0));
    size_t poses_dim = static_cast<size_t>(poses_known.shape(0));
    if (m_dim != poses_dim) {
        throw std::runtime_error(
            "The number of poses in poses_known must match the number of "
            "values in x_known");
    }

    // Map input data directly using Eigen::Map for x_interp and x_known
    const TX* x_interp_ptr = static_cast<const TX*>(x_interp.data());
    const TX* x_known_ptr = static_cast<const TX*>(x_known.data());

    const Eigen::Index n_dim_idx = static_cast<Eigen::Index>(n_dim);
    const Eigen::Index m_dim_idx = static_cast<Eigen::Index>(m_dim);
    Eigen::Map<const Eigen::Matrix<TX, Eigen::Dynamic, 1>> x_interp_map{x_interp_ptr, n_dim_idx};
    Eigen::Map<const Eigen::Matrix<TX, Eigen::Dynamic, 1>> x_known_map{x_known_ptr, m_dim_idx};

    // Map poses_known to PosesT format (flattened 4x4 matrices)
    const TPOSE* poses_ptr = static_cast<const TPOSE*>(poses_known.data());
    Eigen::Map<const Eigen::Matrix<TPOSE, Eigen::Dynamic, 16, Eigen::RowMajor>> poses_map{
        poses_ptr, m_dim_idx, 16};

    // Call templated C++ core::interp_pose
    auto result_poses =
        ouster::sdk::core::interp_pose<TX, TPOSE>(x_interp_map, x_known_map, poses_map);

    py::ndarray<py::numpy, TPOSE, py::c_contig> result =
        make_array<TPOSE>({static_cast<size_t>(result_poses.rows()), 4, 4});

    // Map output array as N×16 row-major matrix and assign directly from
    // result_poses
    Eigen::Map<Eigen::Matrix<TPOSE, Eigen::Dynamic, 16, Eigen::RowMajor>> out_map{
        static_cast<TPOSE*>(result.data()), result_poses.rows(), 16};
    out_map = result_poses;

    return result;
}

template <typename T>
py::ndarray<py::numpy, T, py::c_contig> dewarp_any(
    const py::ndarray<py::ro, py::shape<-1, -1, 3>>& points,
    const py::ndarray<py::ro, py::shape<-1, 4, 4>>& poses) {
    return dewarp<T>(ensure_c_contig_floating<T, py::shape<-1, -1, 3>>(points),
                     ensure_c_contig_floating<T, py::shape<-1, 4, 4>>(poses));
}

py::object dewarp_dispatch(const py::ndarray<py::ro, py::shape<-1, -1, 3>>& points,
                           const py::ndarray<py::ro, py::shape<-1, 4, 4>>& poses) {
    if (!ouster::sdk::python::detail::is_floating_dtype(points.dtype()) ||
        !ouster::sdk::python::detail::is_floating_dtype(poses.dtype())) {
        throw py::type_error("points and poses must be floating-point arrays");
    }
    if (points.dtype() == py::dtype<float>()) {
        return py::cast(dewarp_any<float>(points, poses));
    }
    return py::cast(dewarp_any<double>(points, poses));
}

template <typename T>
py::ndarray<py::numpy, T, py::c_contig> transform_any(
    const py::ndarray<py::ro>& points, const py::ndarray<py::ro, py::shape<4, 4>>& pose) {
    return transform<T>(ensure_c_contig_floating<T>(points),
                        ensure_c_contig_floating<T, py::shape<4, 4>>(pose));
}

py::object transform_dispatch(const py::ndarray<py::ro>& points,
                              const py::ndarray<py::ro, py::shape<4, 4>>& pose) {
    if (!ouster::sdk::python::detail::is_floating_dtype(points.dtype()) ||
        !ouster::sdk::python::detail::is_floating_dtype(pose.dtype())) {
        throw py::type_error("points and pose must be floating-point arrays");
    }
    if (points.dtype() == py::dtype<float>()) {
        return py::cast(transform_any<float>(points, pose));
    }
    return py::cast(transform_any<double>(points, pose));
}

template <typename TX, typename TPOSE>
py::ndarray<py::numpy, TPOSE, py::c_contig> interp_pose_any(
    const py::ndarray<py::ro>& x_interp, const py::ndarray<py::ro>& x_known,
    const py::ndarray<py::ro, py::shape<-1, 4, 4>>& poses_known) {
    return interp_pose<TX, TPOSE>(
        ensure_c_contig_floating<TX>(x_interp), ensure_c_contig_floating<TX>(x_known),
        ensure_c_contig_floating<TPOSE, py::shape<-1, 4, 4>>(poses_known));
}

template <typename T>
py::ndarray<py::numpy, T, py::c_contig> xyzlut_call_any(
    const XYZLutT<T>& self, const py::ndarray<py::ro, py::shape<-1, -1>>& range) {
    const auto range_c = ensure_c_contig<uint32_t, py::shape<-1, -1>>(range);
    if (static_cast<size_t>(range_c.shape(0)) != self.h ||
        static_cast<size_t>(range_c.shape(1)) != self.w) {
        throw std::invalid_argument("Image dimensions do not match lut.");
    }

    auto result = make_array<T>({self.h, self.w, 3});
    Eigen::Map<ouster::sdk::core::PointCloudXYZ<T>> pts{
        static_cast<T*>(result.data()), static_cast<Eigen::Index>(self.h * self.w), 3};
    Eigen::Map<const img_t<uint32_t>> range_map{static_cast<const uint32_t*>(range_c.data()),
                                                static_cast<Eigen::Index>(self.h),
                                                static_cast<Eigen::Index>(self.w)};
    cartesianT<T>(pts, range_map, self.direction, self.offset);
    return result;
}

template <typename T>
py::ndarray<uint32_t, py::c_contig> restore_instance_ids(
    const Object& object, const py::ndarray<const T, py::c_contig>& points,
    py::ndarray<uint32_t, py::c_contig>& instance_ids) {
    // Validate points dims: (H, W, 3)
    if (points.ndim() != 3 || points.shape(2) != 3) {
        throw std::runtime_error("Invalid shape for points, expected (H, W, 3)");
    }

    const size_t num_rows = points.shape(0);            // H
    const size_t num_points_per_col = points.shape(1);  // W
    const size_t point_dim = 3;

    // Validate instance_ids dims: (H, W)
    if (instance_ids.ndim() != 2 || instance_ids.shape(0) != num_rows ||
        instance_ids.shape(1) != num_points_per_col) {
        throw std::runtime_error("Invalid shape for instance ids, expected (H, W)");
    }

    Eigen::Map<const PointCloudXYZ<T>> points_map(points.data(), num_rows * num_points_per_col,
                                                  point_dim);

    Eigen::Map<img_t<uint32_t>> instance_ids_map(instance_ids.data(), num_rows, num_points_per_col);

    ouster::sdk::core::restore_instance_ids<T>(object, points_map, instance_ids_map);

    return instance_ids;
}

// VoxelHashMap binding helpers ------------------------------------------
//
// Templated so we can register the four MapT specializations from a single
// implementation. Returns a Python ValueError-style exception (via
// std::invalid_argument) for wrong-shape inputs instead of letting Eigen
// abort or hit UB.

template <typename MapT>
void bind_voxel_hash_map_common(py::class_<MapT>& cls) {
    using PointType = typename MapT::point_type;
    constexpr bool FIXED_SIZE_POINT = PointType::SizeAtCompileTime != Eigen::Dynamic;
    constexpr std::size_t BASE_COL_COUNT = 3;

    auto map_point =
        [FIXED_SIZE_POINT, BASE_COL_COUNT](
            const py::ndarray<py::numpy, const double, py::ndim<1>, py::c_contig>& point) {
            const auto got = static_cast<std::size_t>(point.shape(0));
            if (FIXED_SIZE_POINT) {
                if (got != BASE_COL_COUNT) {
                    throw std::invalid_argument("VoxelHashMap method expects a 3-element point");
                }
            } else if (got < BASE_COL_COUNT) {
                throw std::invalid_argument(
                    "VoxelHashMap method expects a (3+attributes)-element point");
            }
            return Eigen::Map<const PointType>(point.data(), static_cast<Eigen::Index>(got));
        };

    auto to_numpy = [](const ouster::sdk::core::ArrayXXdR& arr) {
        const std::size_t rows = static_cast<std::size_t>(arr.rows());
        const std::size_t cols = static_cast<std::size_t>(arr.cols());
        auto result = make_array<double>({rows, cols});
        std::copy(arr.data(), arr.data() + arr.size(), static_cast<double*>(result.data()));
        return result;
    };

    cls.def_prop_ro("empty", &MapT::empty)
        .def("clear", &MapT::clear)
        .def(
            "add_points",
            [FIXED_SIZE_POINT, BASE_COL_COUNT](
                MapT& self,
                const py::ndarray<py::numpy, const double, py::ndim<2>, py::c_contig>& points) {
                const std::size_t cols = static_cast<std::size_t>(points.shape(1));
                if (FIXED_SIZE_POINT) {
                    if (cols != BASE_COL_COUNT) {
                        throw std::invalid_argument("add_points expects an Nx3 array");
                    }
                } else if (cols < BASE_COL_COUNT) {
                    throw std::invalid_argument(
                        "add_points expects at least Nx(3+num_attributes) "
                        "columns");
                }
                const Eigen::Map<const ouster::sdk::core::ArrayXXdR> mapped(points.data(),
                                                                            points.shape(0), cols);
                self.add_points(mapped);
            },
            py::arg("points"))
        .def("point_cloud", [&to_numpy](const MapT& self) { return to_numpy(self.pointcloud()); })
        .def(
            "remove_voxels_far_from_location",
            [map_point](
                MapT& self,
                const py::ndarray<py::numpy, const double, py::ndim<1>, py::c_contig>& point) {
                self.remove_voxels_far_from_location(map_point(point));
            },
            py::arg("point"))
        .def(
            "extract_voxels_far_from_location",
            [map_point, to_numpy](
                MapT& self,
                const py::ndarray<py::numpy, const double, py::ndim<1>, py::c_contig>& point) {
                return to_numpy(self.extract_voxels_far_from_location(map_point(point)));
            },
            py::arg("point"))
        .def(
            "get_closest_neighbor",
            [map_point](
                const MapT& self,
                const py::ndarray<py::numpy, const double, py::ndim<1>, py::c_contig>& point,
                double max_distance_sq) {
                return self.get_closest_neighbor(map_point(point), max_distance_sq);
            },
            py::arg("point"), py::arg("max_distance_sq") = std::numeric_limits<double>::max());
}

template <typename MapT>
void bind_voxel_hash_map_dynamic(py::class_<MapT>& cls) {
    using PointType = typename MapT::point_type;
    constexpr bool FIXED_SIZE_POINT = PointType::SizeAtCompileTime != Eigen::Dynamic;
    constexpr std::size_t BASE_COL_COUNT = 3;

    auto map_point =
        [FIXED_SIZE_POINT, BASE_COL_COUNT](
            const py::ndarray<py::numpy, const double, py::ndim<1>, py::c_contig>& point) {
            const auto got = static_cast<std::size_t>(point.shape(0));
            if (FIXED_SIZE_POINT) {
                if (got != BASE_COL_COUNT) {
                    throw std::invalid_argument("VoxelHashMap method expects a 3-element point");
                }
            } else if (got < BASE_COL_COUNT) {
                throw std::invalid_argument(
                    "VoxelHashMap method expects a (3+attributes)-element point");
            }
            return Eigen::Map<const PointType>(point.data(), static_cast<Eigen::Index>(got));
        };
}

ouster::sdk::core::ArrayXXdR voxel_downsample_xd_wrapper(
    py::ndarray<const double, py::c_contig>& frame, double voxel_size,
    std::size_t max_points_per_voxel, std::size_t min_pts_threshold,
    ouster::sdk::core::VoxelDownsampleStrategy strategy) {
    if (frame.ndim() != 2 || frame.shape(1) < 3)
        throw std::invalid_argument(
            "voxel_downsample_xd: frame must be Nx>=3 (x,y,z + optional attributes)");

    const Eigen::Index rows = static_cast<Eigen::Index>(frame.shape(0));
    const Eigen::Index cols = static_cast<Eigen::Index>(frame.shape(1));
    // C-contiguous numpy matches RowMajor Eigen — map directly, no copy
    Eigen::Map<const ouster::sdk::core::ArrayXXdR> mat(frame.data(), rows, cols);
    return ouster::sdk::core::voxel_downsample_xd(mat, voxel_size, max_points_per_voxel,
                                                  min_pts_threshold, strategy);
}

ouster::sdk::core::ArrayX3dR voxel_downsample_3d_wrapper(
    py::ndarray<const double, py::c_contig>& frame, double voxel_size,
    std::size_t max_points_per_voxel, std::size_t min_pts_threshold,
    ouster::sdk::core::VoxelDownsampleStrategy strategy) {
    if (frame.ndim() != 2 || frame.shape(1) != 3)
        throw std::invalid_argument("voxel_downsample_3d: frame must be Nx3");

    const Eigen::Index rows = static_cast<Eigen::Index>(frame.shape(0));
    Eigen::Map<const ouster::sdk::core::ArrayX3dR> mat(frame.data(), rows, 3);
    return ouster::sdk::core::voxel_downsample_3d(mat, voxel_size, max_points_per_voxel,
                                                  min_pts_threshold, strategy);
}

}  // namespace

template <typename T>
inline py::ndarray<py::numpy, T, py::c_contig> destagger_impl(
    const std::vector<size_t>& shape, const void* data, const std::vector<int>& pixel_shift_by_row,
    bool inverse) {
    int dim3 = 1;
    for (size_t i = 2; i < shape.size(); i++) {
        dim3 *= shape[i];
    }

    const T* real_data = reinterpret_cast<const T*>(data);
    auto result = make_array<T>(shape);

    Eigen::TensorMap<const Eigen::Tensor<T, 3, Eigen::RowMajor>> eigen_img(real_data, shape[0],
                                                                           shape[1], dim3);

    Eigen::TensorMap<Eigen::Tensor<T, 3, Eigen::RowMajor>> destaggered(result.data(), shape[0],
                                                                       shape[1], dim3);
    ouster::sdk::core::destagger_into<T, 3>(eigen_img, pixel_shift_by_row, inverse, destaggered);

    return result;
}

inline py::object destagger2(const py::ndarray<py::numpy, py::ro, py::c_contig>& img,
                             const std::vector<int>& pixel_shift_by_row, bool inverse = false) {
    // make sure the image is at least 2d
    if (img.ndim() < 2) {
        throw std::invalid_argument("Must have at least 2 dimensions.");
    }

    if (img.shape(0) == 0 || img.shape(1) == 0) {
        throw std::invalid_argument("Cannot have any dimensions of zero.");
    }

    std::vector<size_t> shape;
    for (size_t i = 0; i < img.ndim(); i++) {
        shape.push_back(img.shape(i));
    }

    if (img.dtype() == py::dtype<bool>()) {
        return destagger_impl<bool>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<uint8_t>()) {
        return destagger_impl<uint8_t>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<uint16_t>()) {
        return destagger_impl<uint16_t>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<uint32_t>()) {
        return destagger_impl<uint32_t>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<uint64_t>()) {
        return destagger_impl<uint64_t>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<float>()) {
        return destagger_impl<float>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<double>()) {
        return destagger_impl<double>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<ouster::sdk::core::float16_t>()) {
        return destagger_impl<ouster::sdk::core::float16_t>(shape, img.data(), pixel_shift_by_row,
                                                            inverse)
            .cast();
    } else if (img.dtype() == py::dtype<int8_t>()) {
        return destagger_impl<int8_t>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<int16_t>()) {
        return destagger_impl<int16_t>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<int32_t>()) {
        return destagger_impl<int32_t>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else if (img.dtype() == py::dtype<int64_t>()) {
        return destagger_impl<int64_t>(shape, img.data(), pixel_shift_by_row, inverse).cast();
    } else {
        throw std::invalid_argument("Destagger called with unsupported dtype.");
    }
}

inline py::object destagger3(const SensorInfo& sensor_info,
                             const py::ndarray<py::numpy, py::ro, py::c_contig>& img,
                             bool inverse = false) {
    if (img.ndim() < 2) {
        throw std::invalid_argument("Must have at least 2 dimensions.");
    }

    if (img.shape(0) != sensor_info.format.pixels_per_column ||
        img.shape(1) != sensor_info.format.columns_per_frame ||
        img.shape(0) != sensor_info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument("Image resolution must match SensorInfo.");
    }
    return destagger2(img, sensor_info.format.pixel_shift_by_row, inverse);
}

void init_client_processing(py::module_& module, py::module_& /*unused*/) {
    module.def("destagger", &destagger2, py::arg("img"), py::arg("pixel_shift_by_row"),
               py::arg("inverse") = false,
               py::sig("def destagger(img: Annotated[NDArray, dict(order='C', writable=False)], "
                       "pixel_shift_by_row: Sequence[int], inverse: bool = False) -> "
                       "Annotated[NDArray, dict(order='C')]"));
    module.def(
        "destagger", &destagger3, py::arg("sensor_info"), py::arg("img"),
        py::arg("inverse") = false,
        py::sig("def destagger(sensor_info: SensorInfo, img: Annotated[NDArray, dict(order='C', "
                "writable=False)], inverse: bool = False) -> Annotated[NDArray, dict(order='C')]"));

    py::class_<FrameBatcher>(module, "FrameBatcher")
        .def(py::init<std::shared_ptr<SensorInfo>>())
        .def("reset", &FrameBatcher::reset)
        .def("batched_packets", &FrameBatcher::batched_packets)
        .def("dropped_packets", &FrameBatcher::dropped_packets)
        .def("set_max_cache_size", &FrameBatcher::set_max_cache_size)
        .def("get_max_cache_size", &FrameBatcher::get_max_cache_size)
        .def("__call__",
             [](FrameBatcher& self, Packet& packet, LidarFrame& lidar_frame) {
                 PyErr_WarnEx(PyExc_FutureWarning,
                              "FrameBatcher.__call__() is deprecated, use "
                              "FrameBatcher.batch() instead",
                              1);
                 return self.batch(packet, lidar_frame);
             })
        .def("batch", &FrameBatcher::batch);

    py::class_<XYZLutT<float>>(module, "XYZLutFloat")
        .def(
            "__init__",
            [](XYZLutT<float>* self, const SensorInfo& sensor, bool use_extrinsics) {
                new (self) XYZLutT<float>(sensor, use_extrinsics);
            },
            py::arg("info"), py::arg("use_extrinsics") = true, R"doc(
Return a function that can project frames into Cartesian coordinates.

If called with a numpy array representing a range image, the range image
must be in "staggered" form, where each column corresponds to a single
measurement block. LidarFrame fields are always staggered.

Internally, this will pre-compute a lookup table using the supplied
intrinsic parameters. XYZ points are returned as a H x W x 3 array of
doubles, where H is the number of beams and W is the horizontal resolution
of the frame.

The coordinates are reported in meters in the *sensor frame* (when
``use_extrinsics`` is False, default True) as defined in the sensor documentation.

However, the result is returned in the "extrinsics frame" if
``use_extrinsics`` is True, which makes additional transform from
"sensor frame" to "extrinsics frame" using the homogeneous 4x4 transform
matrix from ``info.sensor_to_body`` property.

Args:
    info: sensor metadata
    use_extrinsics: if True, applies the ``info.sensor_to_body`` transform to the
                    resulting "sensor frame" coordinates and returns the
                    result in "extrinsics frame".

Returns:
    A function that computes a point cloud given a range image)doc")
        .def("__call__", &xyzlut_call_any<float>, py::arg("range"))
        .def(
            "__call__",
            [](const XYZLutT<float>& self, const LidarFrame& frame) {
                if (frame.w != self.w || frame.h != self.h) {
                    throw std::invalid_argument("Frame dimensions do not match lut.");
                }

                // Create output array with correct shape (H, W, 3)
                auto result = make_array<float>({self.h, self.w, 3});

                // Map the output array to Eigen matrix and copy the data
                Eigen::Map<ouster::sdk::core::PointCloudXYZ<float>> pts{
                    static_cast<float*>(result.data()), static_cast<Eigen::Index>(self.h * self.w),
                    3};

                cartesianT<float>(pts, frame.field(ouster::sdk::core::ChanField::RANGE),
                                  self.direction, self.offset);
                return result;
            },
            py::arg("frame"))
        .def_prop_ro("h", [](const XYZLutT<float>& self) { return self.h; })
        .def_prop_ro("w", [](const XYZLutT<float>& self) { return self.w; })
        .def_prop_ro(
            "direction",
            [](const XYZLutT<float>& self) {
                return py::ndarray<py::numpy, const float, py::c_contig>(self.direction.data(),
                                                                         {self.h * self.w, 3});
            },
            py::rv_policy::reference_internal)
        .def_prop_ro(
            "offset",
            [](const XYZLutT<float>& self) {
                return py::ndarray<py::numpy, const float, py::c_contig>(self.offset.data(),
                                                                         {self.h * self.w, 3});
            },
            py::rv_policy::reference_internal);

    py::class_<XYZLutT<double>>(module, "XYZLut")
        .def(
            "__init__",
            [](XYZLutT<double>* self, const SensorInfo& sensor, bool use_extrinsics) {
                new (self) XYZLutT<double>(sensor, use_extrinsics);
            },
            py::arg("info"), py::arg("use_extrinsics") = false, R"doc(
Return a function that can project frames into Cartesian coordinates.

If called with a numpy array representing a range image, the range image
must be in "staggered" form, where each column corresponds to a single
measurement block. LidarFrame fields are always staggered.

Internally, this will pre-compute a lookup table using the supplied
intrinsic parameters. XYZ points are returned as a H x W x 3 array of
doubles, where H is the number of beams and W is the horizontal resolution
of the frame.

The coordinates are reported in meters in the *sensor frame* (when
``use_extrinsics`` is False, default) as defined in the sensor documentation.

However, the result is returned in the "extrinsics frame" if
``use_extrinsics`` is True, which makes additional transform from
"sensor frame" to "extrinsics frame" using the homogeneous 4x4 transform
matrix from ``info.sensor_to_body`` property.

Args:
    info: sensor metadata
    use_extrinsics: if True, applies the ``info.sensor_to_body`` transform to the
                    resulting "sensor frame" coordinates and returns the
                    result in "extrinsics frame".

Returns:
    A function that computes a point cloud given a range image)doc")
        .def("__call__", &xyzlut_call_any<double>, py::arg("range"))
        .def(
            "__call__",
            [](const XYZLutT<double>& self, const LidarFrame& frame) {
                if (frame.w != self.w || frame.h != self.h) {
                    throw std::invalid_argument("Frame dimensions do not match lut.");
                }

                // Create output array with correct shape (H, W, 3)
                auto result = make_array<double>({self.h, self.w, 3});

                // Map the output array to Eigen matrix and copy the data
                Eigen::Map<ouster::sdk::core::PointCloudXYZ<double>> pts{
                    static_cast<double*>(result.data()), static_cast<Eigen::Index>(self.h * self.w),
                    3};

                cartesianT<double>(pts, frame.field(ouster::sdk::core::ChanField::RANGE),
                                   self.direction, self.offset);
                return result;
            },
            py::arg("frame"))
        .def_prop_ro("h", [](const XYZLutT<double>& self) { return self.h; })
        .def_prop_ro("w", [](const XYZLutT<double>& self) { return self.w; })
        .def_prop_ro(
            "direction",
            [](const XYZLutT<double>& self) {
                return py::ndarray<py::numpy, const double, py::c_contig>(self.direction.data(),
                                                                          {self.h * self.w, 3});
            },
            py::rv_policy::reference_internal)
        .def_prop_ro(
            "offset",
            [](const XYZLutT<double>& self) {
                return py::ndarray<py::numpy, const double, py::c_contig>(self.offset.data(),
                                                                          {self.h * self.w, 3});
            },
            py::rv_policy::reference_internal);

    // Image processing
    py::class_<AutoExposure>(module, "AutoExposure")
        .def(py::init<>())
        .def(
            "__init__",
            [](AutoExposure* self, int update_every) { new (self) AutoExposure(update_every); },
            py::arg("update_every"))
        .def(
            "__init__",
            [](AutoExposure* self, double low, double high, int update_every, double damping) {
                new (self) AutoExposure(low, high, update_every, damping);
            },
            py::arg("lo_percentile"), py::arg("hi_percentile"), py::arg("update_every"),
            py::arg("damping") = 0.9)
        .def("update", &auto_exposure_update<float>, py::arg("image").noconvert(),
             py::arg("update_state") = true)
        .def("update", &auto_exposure_update<double>, py::arg("image").noconvert(),
             py::arg("update_state") = true)
        .def(
            "update",
            [](AutoExposure& self,
               const py::ndarray<ouster::sdk::core::float16_t, py::ndim<3>, py::c_contig>& image,
               bool update_state) {
                if (image.ndim() != 3 || image.shape(2) != 3) {
                    throw std::invalid_argument("Expected an H x W x 3 array");
                }
                const auto h = static_cast<Eigen::Index>(image.shape(0));
                const auto w = static_cast<Eigen::Index>(image.shape(1));

                auto result = make_array<float>({image.shape(0), image.shape(1), 3});

                Eigen::TensorMap<const rgb_img_t<ouster::sdk::core::float16_t>> in_map(image.data(),
                                                                                       h, w, 3);
                Eigen::TensorMap<rgb_img_t<float>> out_map(result.data(), h, w, 3);

                self.update(in_map, out_map, update_state);

                return result;
            },
            py::arg("image").noconvert(), py::arg("update_state") = true)
        .def("update", &image_proc_call<AutoExposure, float>, py::arg("image").noconvert(),
             py::arg("update_state") = true)
        .def("update", &image_proc_call<AutoExposure, double>, py::arg("image").noconvert(),
             py::arg("update_state") = true);

    py::class_<BeamUniformityCorrector>(module, "BeamUniformityCorrector")
        .def(py::init<>())
        .def("update", &image_proc_call<BeamUniformityCorrector, float>,
             py::arg("image").noconvert(), py::arg("update_state") = true)
        .def("update", &image_proc_call<BeamUniformityCorrector, double>,
             py::arg("image").noconvert(), py::arg("update_state") = true);

    py::class_<LocalToneMapper>(module, "LocalToneMapper")
        .def(py::init<>())
        .def(
            "__init__",
            [](LocalToneMapper* self, int update_every) {
                new (self) LocalToneMapper(update_every);
            },
            py::arg("update_every"))
        .def(
            "__init__",
            [](LocalToneMapper* self, double low, double high, int update_every, double damping,
               double compress_dr_max_lum, bool color_correct) {
                new (self) LocalToneMapper(low, high, update_every, damping, compress_dr_max_lum,
                                           color_correct);
            },
            py::arg("lo_percentile"), py::arg("hi_percentile"), py::arg("update_every"),
            py::arg("damping"), py::arg("compress_dr_max_lum"), py::arg("color_correct"))
        .def(
            "__init__",
            [](LocalToneMapper* self, double low, double high, int update_every, double damping,
               bool compress_dr, bool color_correct) {
                new (self)
                    LocalToneMapper(low, high, update_every, damping, compress_dr, color_correct);
            },
            py::arg("lo_percentile"), py::arg("hi_percentile"), py::arg("update_every"),
            py::arg("damping"), py::arg("compress_dr"), py::arg("color_correct"))
        .def(
            "update",
            [](LocalToneMapper& self,
               py::ndarray<ouster::sdk::core::float16_t, py::ndim<3>, py::c_contig> image,
               bool update_state) {
                if (image.ndim() != 3 || image.shape(2) != 3) {
                    throw std::invalid_argument("Expected an H x W x 3 array");
                }
                const auto h = static_cast<Eigen::Index>(image.shape(0));
                const auto w = static_cast<Eigen::Index>(image.shape(1));

                auto result = make_array<float>({image.shape(0), image.shape(1), 3});

                Eigen::TensorMap<const rgb_img_t<ouster::sdk::core::float16_t>> in_map(image.data(),
                                                                                       h, w, 3);
                Eigen::TensorMap<rgb_img_t<float>> out_map(result.data(), h, w, 3);

                self.update(in_map, out_map, update_state);

                return result;
            },
            py::arg("image").noconvert(), py::arg("update_state") = true);

    module.def("dewarp", &dewarp_dispatch,
               R"(
Applies a set of 4x4 pose transformations to a collection of 3D points.
Args:
    points: A NumPy array of shape (H, W, 3) representing the 3D points.
    poses: A NumPy array of shape (W, 4, 4) representing the 4x4 pose

Return:
    A NumPy array of shape (H, W, 3) containing the dewarped 3D points
    )",
               py::arg("points"), py::arg("poses"));

    module.def("transform", &transform_dispatch,
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

    module.def("euler_pose_to_matrix", &ouster::sdk::core::euler_pose_to_matrix,
               R"(
        Convert a pose given in Euler angles and translation to a 4x4 transformation matrix.

        The pose vector should contain the following elements in order:
            [roll, pitch, yaw, x, y, z]
        where roll, pitch, and yaw are in radians.

        Returns:
            A 4x4 homogeneous transformation matrix.
        )");

    module.def("matrix_to_euler", &ouster::sdk::core::matrix_to_euler,
               R"(
        Extract ZYX Euler angles (roll, pitch, yaw) from a 3x3 rotation matrix.

        Args:
            matrix: A 3x3 rotation matrix (numpy array, float64).

        Returns:
            A length-3 array [roll, pitch, yaw] in radians.
        )");

    module.def("quaternion_pose_to_matrix", &ouster::sdk::core::quaternion_pose_to_matrix,
               R"(
        Convert a pose given as a quaternion and translation to a 4x4 transformation matrix.

        The pose vector should contain the following elements in order:
            [qw, qx, qy, qz, x, y, z]

        Returns:
            A 4x4 homogeneous transformation matrix.
        )");

    module.def("get_rot_matrix_to_align_to_gravity",
               &ouster::sdk::core::get_rot_matrix_to_align_to_gravity, py::arg("accel_x"),
               py::arg("accel_y"), py::arg("accel_z"), py::arg("fix_yaw") = true,
               R"(
        Computes a 3x3 rotation matrix that aligns acceleration to gravity
        [0, 0, 1].

        Args:
            accel_x: x-component of acceleration.
            accel_y: y-component of acceleration.
            accel_z: z-component of acceleration.
            fix_yaw: if true (default), neutralize yaw; if false, keep the
                shortest gravity-alignment rotation without forced yaw.
        )");

    py::enum_<ouster::sdk::core::VoxelDownsampleStrategy>(module, "VoxelDownsampleStrategy")
        .value("FIRST_N_POINT", ouster::sdk::core::VoxelDownsampleStrategy::FIRST_N_POINT)
        .value("AVERAGE_POINT", ouster::sdk::core::VoxelDownsampleStrategy::AVERAGE_POINT)
        .value("RANDOM", ouster::sdk::core::VoxelDownsampleStrategy::RANDOM);

    module.def("voxel_downsample_3d", &voxel_downsample_3d_wrapper, py::arg("frame"),
               py::arg("voxel_size"), py::arg("max_points_per_voxel") = std::size_t{1},
               py::arg("min_pts_threshold") = std::size_t{1},
               py::arg("strategy") = ouster::sdk::core::VoxelDownsampleStrategy::RANDOM,
               R"(
        [BETA] Downsample an Nx3 pointcloud using a voxel hash map.

        Args:
            frame: Nx3 array of points (x,y,z).
            voxel_size: Edge length of each cubic voxel.
            max_points_per_voxel: Maximum points kept per voxel (FIRST_N_POINT, RANDOM).
            min_pts_threshold: Minimum raw insertions required before a voxel
                contributes to the output (AVERAGE_POINT).
            strategy: VoxelDownsampleStrategy selecting the per-voxel reduction.

        Returns:
            Mx3 array of downsampled points (M <= N).
        )");

    module.def("voxel_downsample_xd", &voxel_downsample_xd_wrapper, py::arg("frame"),
               py::arg("voxel_size"), py::arg("max_points_per_voxel") = std::size_t{1},
               py::arg("min_pts_threshold") = std::size_t{1},
               py::arg("strategy") = ouster::sdk::core::VoxelDownsampleStrategy::RANDOM,
               R"(
        [BETA] Downsample an NxD pointcloud using a voxel hash map.

        Args:
            frame: NxD array of points (D >= 3; first 3 columns are x,y,z).
            voxel_size: Edge length of each cubic voxel.
            max_points_per_voxel: Maximum points kept per voxel (FIRST_N_POINT, RANDOM).
            min_pts_threshold: Minimum raw insertions required before a voxel
                contributes to the output (AVERAGE_POINT).
            strategy: VoxelDownsampleStrategy selecting the per-voxel reduction.

        Returns:
            MxD array of downsampled points (M <= N, same column layout as input).
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

    module.def("interp_pose", &interp_pose_any<double, double>, py::arg("x_interp"),
               py::arg("x_known"), py::arg("poses_known"),
               R"(
        Interpolate 4x4 pose matrices at given x-coordinate values (double precision).
        Args:
            x_interp: (N,) or (N,1) array of interpolation x values (float64)
            x_known: (M,) or (M,1) array of known x values (float64)
            poses_known: (M, 4, 4) array of known pose matrices (float64)
        Returns:
            (N, 4, 4) array of interpolated pose matrices (float64)
        )");

    module.def("interp_pose_float", &interp_pose_any<double, float>, py::arg("x_interp"),
               py::arg("x_known"), py::arg("poses_known"),
               R"(
        Interpolate 4x4 pose matrices at given x-coordinate values (float precision for poses).
        Args:
            x_interp: (N,) or (N,1) array of interpolation x values (float64)
            x_known: (M,) or (M,1) array of known x values (float64)
            poses_known: (M, 4, 4) array of known pose matrices (float32)
        Returns:
            (N, 4, 4) array of interpolated pose matrices (float32)
        )");

    // 100m is a sensible default for SLAM-scale data and avoids the
    // previous default of 0.0 (which always threw at construction time).
    constexpr double DEFAULT_VOXEL_SIZE = 0.1;
    constexpr double DEFAULT_MAX_DISTANCE = 100.0;
    constexpr std::size_t DEFAULT_MAX_POINTS_PER_VOXEL = 20;
    constexpr std::size_t DEFAULT_MIN_PTS_THRESHOLD = 1;

    {
        auto cls = py::class_<ouster::sdk::core::VoxelHashMap3d>(module, "VoxelHashMap3d")
                       .def(py::init<double, double, std::size_t, std::size_t>(),
                            py::arg("voxel_size") = DEFAULT_VOXEL_SIZE,
                            py::arg("max_distance") = DEFAULT_MAX_DISTANCE,
                            py::arg("max_points_per_voxel") = DEFAULT_MAX_POINTS_PER_VOXEL,
                            py::arg("min_pts_threshold") = DEFAULT_MIN_PTS_THRESHOLD);
        bind_voxel_hash_map_common(cls);
    }

    {
        auto cls = py::class_<ouster::sdk::core::VoxelHashMapXd>(module, "VoxelHashMapXd")
                       .def(py::init<double, double, std::size_t, std::size_t, std::size_t>(),
                            py::arg("voxel_size") = DEFAULT_VOXEL_SIZE,
                            py::arg("max_distance") = DEFAULT_MAX_DISTANCE,
                            py::arg("max_points_per_voxel") = DEFAULT_MAX_POINTS_PER_VOXEL,
                            py::arg("min_pts_threshold") = DEFAULT_MIN_PTS_THRESHOLD,
                            py::arg("num_attributes") = 0);
        bind_voxel_hash_map_common(cls);
    }

    auto mesh_cls = py::class_<ouster::sdk::core::Mesh>(module, "Mesh");
    mesh_cls.def(py::init<>())
        .def(py::init<std::vector<ouster::sdk::core::Triangle>>())
        .def("load_from_stl", &ouster::sdk::core::Mesh::load_from_stl)
        .def("save_stl_binary", py::overload_cast<const std::string&>(
                                    &ouster::sdk::core::Mesh::save_stl_binary, py::const_))
        .def_prop_ro("triangles", &ouster::sdk::core::Mesh::triangles)
        .def("__eq__", [](const ouster::sdk::core::Mesh& left, const py::object& right) {
            if (!py::isinstance<ouster::sdk::core::Mesh>(right)) {
                return false;
            }
            return left == py::cast<const ouster::sdk::core::Mesh&>(right);
        });

    auto tri_cls = py::class_<ouster::sdk::core::Triangle>(module, "Triangle");
    tri_cls
        .def(py::init<const ouster::sdk::core::Coord&, const ouster::sdk::core::Coord&,
                      const ouster::sdk::core::Coord&>())
        .def_rw("coords", &ouster::sdk::core::Triangle::coords)
        .def_rw("edges", &ouster::sdk::core::Triangle::edges)
        .def_rw("normal", &ouster::sdk::core::Triangle::normal);

    module.def("restore_instance_ids", &restore_instance_ids<double>,
               R"(
Fills instance_ids pixel field based on object's oriented bounding box.
Args:
    object: an object, such as one produced by detection engine
    points: A NumPy array of shape (H, W, 3) representing the 3D points.
    instance_ids: A NumPy array of shape (H, W) representing the instance ids
Return:
    (H, W) updated instance_ids field
               )",
               py::arg("object"), py::arg("points").noconvert(), py::arg("poses").noconvert());

    module.def("restore_instance_ids", &restore_instance_ids<float>,
               R"(
Fills instance_ids pixel field based on object's oriented bounding box.
Args:
    object: an object, such as one produced by detection engine
    points: A NumPy array of shape (H, W, 3) representing the 3D points.
    instance_ids: A NumPy array of shape (H, W) representing the instance ids
Return:
    (H, W) updated instance_ids field
               )",
               py::arg("object"), py::arg("points").noconvert(), py::arg("poses").noconvert());
}

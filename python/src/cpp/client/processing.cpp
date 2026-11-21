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
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <pyerrors.h>
#include <warnings.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>

#include "client_common.h"
#include "eigen_dense.h"
#include "ndarray_helpers.h"
#include "ouster/core/attribute_layout.h"
#include "ouster/core/chanfield.h"
#include "ouster/core/cloud_io.h"
#include "ouster/core/frame.h"
#include "ouster/core/image_processing.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/object.h"
#include "ouster/core/object_util.h"
#include "ouster/core/packet.h"
#include "ouster/core/pose_conversion.h"
#include "ouster/core/pose_util.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/voxel_hash_map.h"

using ouster::sdk::core::Frame;
using ouster::sdk::core::FrameBatcher;
using ouster::sdk::core::img_t;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::LidarInfo;
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
 * @param[in] poses A NumPy array of shape (W, 4, 4) or (1, 4, 4) representing
 * the 4x4 pose matrices.
 * - W: Number of pose matrices (one per image column for lidar rolling shutter)
 * - 1: A single rigid pose broadcast to every column (e.g. CameraFrame)
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
    const int num_poses = poses.shape(0);            // W or 1
    const int num_rows = points.shape(0);            // H
    const int num_points_per_col = points.shape(1);  // W
    const int point_dim = 3;

    MatrixX16R<T> poses_eigen;
    if (num_poses == 1 && num_points_per_col > 1) {
        // Single-pose frame (e.g. CameraFrame): broadcast the rigid pose to
        // every image column instead of requiring one pose per column.
        poses_eigen.resize(num_points_per_col, 16);
        Eigen::Map<const Eigen::Matrix<T, 1, 16, Eigen::RowMajor>> single_pose(poses.data());
        for (int col = 0; col < num_points_per_col; ++col) {
            poses_eigen.row(col) = single_pose;
        }
    } else if (num_points_per_col != num_poses) {
        throw std::runtime_error("Number of points per set must match number of poses");
    } else {
        // Map poses as a matrix where each row is a flattened 4x4 pose matrix.
        // This handles both contiguous and non-contiguous layouts without copying.
        Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            poses_strided_map{poses.data(), num_poses, 16};
        poses_eigen = poses_strided_map;
    }

    const int num_cols = static_cast<int>(poses_eigen.rows());

    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>> points_map{
        points.data(), num_rows * num_points_per_col, 3};

    py::ndarray<py::numpy, T, py::c_contig> result =
        make_array<T>({static_cast<size_t>(num_rows), static_cast<size_t>(num_cols),
                       static_cast<size_t>(point_dim)});

    Eigen::Map<PointCloudXYZ<T>> dewarped_points(result.data(), num_rows * num_cols, point_dim);

    ouster::sdk::core::dewarp<T>(dewarped_points, points_map, poses_eigen);

    return result;
}

/**
 * Applies a single of 4x4 pose transformations to a collection of 3D points,
 * reshapes the input into appropriate Eigen matrices, and invokes the C++
 * version of the `transform` function for performing the transformations.
 *
 * This function is designed to convert NumPy input arrays from Python into
 * Eigen matrices, which are suitable for efficient matrix operations in C++.
 * The 3D points are passed in the shape of (H, W, 3), and the single 4x4 pose
 * matrices in the shape of (4, 4).
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

py::ndarray<py::numpy, py::c_contig> dewarp_dispatch(
    const py::ndarray<py::ro, py::shape<-1, -1, 3>>& points,
    const py::ndarray<py::ro, py::shape<-1, 4, 4>>& poses) {
    if (!ouster::sdk::python::detail::is_floating_dtype(points.dtype()) ||
        !ouster::sdk::python::detail::is_floating_dtype(poses.dtype())) {
        throw py::type_error("points and poses must be floating-point arrays");
    }
    if (points.dtype() == py::dtype<float>()) {
        return py::ndarray<py::numpy, py::c_contig>(dewarp_any<float>(points, poses));
    }
    return py::ndarray<py::numpy, py::c_contig>(dewarp_any<double>(points, poses));
}

template <typename T>
py::ndarray<py::numpy, T, py::c_contig> transform_any(
    const py::ndarray<py::ro>& points, const py::ndarray<py::ro, py::shape<4, 4>>& pose) {
    return transform<T>(ensure_c_contig_floating<T>(points),
                        ensure_c_contig_floating<T, py::shape<4, 4>>(pose));
}

py::ndarray<py::numpy, py::c_contig> transform_dispatch(
    const py::ndarray<py::ro>& points, const py::ndarray<py::ro, py::shape<4, 4>>& pose) {
    if (!ouster::sdk::python::detail::is_floating_dtype(points.dtype()) ||
        !ouster::sdk::python::detail::is_floating_dtype(pose.dtype())) {
        throw py::type_error("points and pose must be floating-point arrays");
    }
    if (points.dtype() == py::dtype<float>()) {
        return py::ndarray<py::numpy, py::c_contig>(transform_any<float>(points, pose));
    }
    return py::ndarray<py::numpy, py::c_contig>(transform_any<double>(points, pose));
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
py::ndarray<py::numpy, T, py::c_contig> xyzlut_call_frame(const XYZLutT<T>& self,
                                                          const Frame& frame) {
    if (frame.w != self.w || frame.h != self.h) {
        throw std::invalid_argument("Frame dimensions do not match lut.");
    }
    if (!frame.has_field(ouster::sdk::core::ChanField::RANGE)) {
        throw std::invalid_argument("Frame must contain a RANGE field.");
    }

    auto result = make_array<T>({self.h, self.w, 3});
    Eigen::Map<ouster::sdk::core::PointCloudXYZ<T>> pts{
        static_cast<T*>(result.data()), static_cast<Eigen::Index>(self.h * self.w), 3};
    cartesianT<T>(pts, frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE), self.direction,
                  self.offset);
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

// VoxelHashMap binding helpers -------------------------------------
//
// Return a Python ValueError-style exception (via std::invalid_argument) for
// wrong-shape inputs instead of letting Eigen abort or hit UB.

using ouster::sdk::core::VoxelHashMap;

// A 1-D position/point argument: xyz first, then (for a full point) the
// attributes as doubles.
using PointArray = py::ndarray<py::numpy, const double, py::ndim<1>, py::c_contig>;

Eigen::Map<const Eigen::VectorXd> map_point(const PointArray& point) {
    const auto got = static_cast<Eigen::Index>(point.shape(0));
    if (got < 3) {
        throw std::invalid_argument("VoxelHashMap method expects a (3+attributes)-element point");
    }
    return Eigen::Map<const Eigen::VectorXd>(point.data(), got);
}

// The double-valued API: points as rows of point_cols() doubles.
void bind_voxel_hash_map_common(py::class_<VoxelHashMap>& cls) {
    cls.def_prop_ro("empty", &VoxelHashMap::empty)
        .def("clear", &VoxelHashMap::clear)
        .def(
            "add_points",
            [](VoxelHashMap& self,
               const py::ndarray<py::numpy, const double, py::ndim<2>, py::c_contig>& points) {
                const Eigen::Map<const ouster::sdk::core::ArrayXXdR> mapped(
                    points.data(), static_cast<Eigen::Index>(points.shape(0)),
                    static_cast<Eigen::Index>(points.shape(1)));
                self.add_points(mapped);
            },
            py::arg("points"),
            R"pbdoc(
                Insert points, as rows of ``point_cols()`` values each (xyz,
                then any attributes as doubles).

                To keep attributes in their native types instead, call with
                ``(xyz, attributes)``; see the other overload.
            )pbdoc")
        .def(
            "update",
            [](VoxelHashMap& self,
               const py::ndarray<py::numpy, const double, py::ndim<2>, py::c_contig>& points,
               const PointArray& position) {
                // Binds straight to the numpy array's buffer; self.update()
                // checks the column count and inserts from it directly, with
                // no intermediate per-point copy of the batch.
                const Eigen::Map<const ouster::sdk::core::ArrayXXdR> mapped(
                    points.data(), static_cast<Eigen::Index>(points.shape(0)),
                    static_cast<Eigen::Index>(points.shape(1)));
                self.update(mapped, map_point(position));
            },
            py::arg("points"), py::arg("position"),
            R"pbdoc(
                Insert ``points`` and then remove voxels far from ``position``.

                Every point is checked against the map's ``point_cols()`` before any
                is inserted, so one bad point leaves the map unchanged.
            )pbdoc")
        .def(
            "remove_voxels_far_from_location",
            [](VoxelHashMap& self, const PointArray& point) {
                self.remove_voxels_far_from_location(map_point(point));
            },
            py::arg("point"))
        .def(
            "get_closest_neighbor",
            [](const VoxelHashMap& self, const PointArray& point, double max_distance_sq) {
                return self.get_closest_neighbor(map_point(point), max_distance_sq);
            },
            py::arg("point"), py::arg("max_distance_sq") = std::numeric_limits<double>::max(),
            R"pbdoc(
                Find the closest point in the map to ``point``.

                Only the query's voxel and its 26 neighbors are searched, within
                ``min(max_distance_sq, 8 * voxel_size**2)``; a point farther away is
                never returned, even if it is the closest in the map.

                Returns:
                    (closest_point, squared_distance). If nothing is found,
                    ``squared_distance >= max_distance_sq``.
            )pbdoc")
        .def("storage_mode", &VoxelHashMap::storage_mode);
}

// Maps carrying an AttributeLayout additionally accept and return the packed
// attribute records verbatim, so a uint32 attribute stays a uint32 from the
// caller's numpy array all the way to the exported file. Python builds and
// reads those records with a packed structured dtype matching the layout, so
// the binding only ever sees an (N, stride) uint8 buffer.
// Any C-contiguous numpy array; the attribute helpers below check each one's
// dtype against the map's AttributeLayout themselves.
using AnyArray = py::ndarray<py::numpy, py::c_contig>;

// The numpy dtype an attribute of this type is handed to, and handed back in.
py::dlpack::dtype attribute_dtype(ouster::sdk::core::AttributeType type) {
    return ouster::sdk::core::visit_attribute_type(
        type, [](auto sample) { return py::dtype<decltype(sample)>(); });
}

// The numpy dtype object for `type`, e.g. numpy.dtype('uint32') for
// AttributeType::U32. Built from attribute_dtype() above, so the C++ scalar
// type visit_attribute_type() names stays the only source of the mapping.
py::object attribute_numpy_dtype(ouster::sdk::core::AttributeType type) {
    const py::dlpack::dtype dtype = attribute_dtype(type);
    char kind = 0;
    switch (static_cast<py::dlpack::dtype_code>(dtype.code)) {
        case py::dlpack::dtype_code::Int:
            kind = 'i';
            break;
        case py::dlpack::dtype_code::UInt:
            kind = 'u';
            break;
        case py::dlpack::dtype_code::Float:
            kind = 'f';
            break;
        default:
            throw std::logic_error("attribute type has no numpy dtype");
    }
    return py::module_::import_("numpy").attr("dtype")(std::string(1, kind) +
                                                       std::to_string(dtype.bits / 8));
}

// numpy's name for `type`'s dtype, e.g. "uint32"; used in error messages.
std::string attribute_numpy_dtype_name(ouster::sdk::core::AttributeType type) {
    return py::cast<std::string>(attribute_numpy_dtype(type).attr("name"));
}

// The AttributeType whose attribute_numpy_dtype() equals `dtype_like`, which
// may be any spelling numpy.dtype(...) accepts: a scalar type (np.uint32), a
// dtype instance, or a type string ("uint32").
ouster::sdk::core::AttributeType attribute_type_from_dtype(const py::handle& dtype_like) {
    const py::object dtype = py::module_::import_("numpy").attr("dtype")(dtype_like);
    std::string supported;
    for (const ouster::sdk::core::AttributeType type : ouster::sdk::core::ALL_ATTRIBUTE_TYPES) {
        if (dtype.equal(attribute_numpy_dtype(type))) {
            return type;
        }
        if (!supported.empty()) {
            supported += ", ";
        }
        supported += attribute_numpy_dtype_name(type);
    }
    throw std::invalid_argument("unsupported attribute dtype '" +
                                py::cast<std::string>(py::str(dtype)) +
                                "'; expected one of: " + supported);
}

// One attribute of a layout built from Python: either an AttributeSpec
// already, or a (dtype, count) pair naming a numpy scalar type and how many
// of it, e.g. (np.uint32, 1) for a scalar RANGE value, resolved through
// attribute_type_from_dtype() above.
ouster::sdk::core::AttributeSpec attribute_spec_from_python(const py::object& item) {
    if (py::isinstance<ouster::sdk::core::AttributeSpec>(item)) {
        return py::cast<const ouster::sdk::core::AttributeSpec&>(item);
    }
    if (!py::isinstance<py::sequence>(item) || py::len(item) != 2) {
        throw std::invalid_argument(
            "each attribute must be an AttributeSpec or a (dtype, count) pair");
    }

    return ouster::sdk::core::AttributeSpec{attribute_type_from_dtype(item[0]),
                                            py::cast<std::uint32_t>(item[1])};
}

// Builds an AttributeLayout from a Python object that is either already an
// AttributeLayout, or a list of attributes as accepted by
// attribute_spec_from_python() above, e.g. [(np.uint32, 1), (np.uint8, 1)].
// Lets VoxelHashMap.create()'s attribute_layout argument speak in numpy
// dtypes directly, with no need to spell out AttributeSpec/AttributeType.
ouster::sdk::core::AttributeLayout attribute_layout_from_python(const py::object& obj) {
    if (py::isinstance<ouster::sdk::core::AttributeLayout>(obj)) {
        return py::cast<const ouster::sdk::core::AttributeLayout&>(obj);
    }
    if (!py::isinstance<py::sequence>(obj)) {
        throw std::invalid_argument(
            "attribute_layout must be an AttributeLayout or a list of "
            "AttributeSpec/(dtype, count) attributes");
    }

    py::sequence seq = py::cast<py::sequence>(obj);
    std::vector<ouster::sdk::core::AttributeSpec> specs;
    specs.reserve(py::len(seq));
    for (py::handle item : seq) {
        specs.push_back(attribute_spec_from_python(py::borrow<py::object>(item)));
    }
    return ouster::sdk::core::AttributeLayout(std::move(specs));
}

// Values per point in `array`: the product of every dimension past the first.
std::size_t attribute_elements(const AnyArray& array) {
    std::size_t elements = 1;
    for (std::size_t dim = 1; dim < array.ndim(); ++dim) {
        elements *= static_cast<std::size_t>(array.shape(dim));
    }
    return elements;
}

// Checks that `attributes` holds one numpy array per attribute of `layout`,
// each already in the type its AttributeSpec declares and with one row of
// spec.count values per point. Nothing is converted on the way into the map,
// so a uint32 RANGE stays a uint32 from the caller's array into the voxel.
void check_attribute_arrays(const ouster::sdk::core::AttributeLayout& layout,
                            const std::vector<AnyArray>& attributes, std::size_t rows) {
    const auto& specs = layout.specs();
    if (attributes.size() != specs.size()) {
        throw std::invalid_argument(
            "expected " + std::to_string(specs.size()) +
            " attribute array(s) to match the map's attribute layout, got " +
            std::to_string(attributes.size()));
    }

    for (std::size_t i = 0; i < specs.size(); ++i) {
        const AnyArray& array = attributes[i];
        const std::string where = "attribute " + std::to_string(i);
        if (array.dtype() != attribute_dtype(specs[i].type)) {
            throw std::invalid_argument(where + " must be a numpy array of dtype " +
                                        attribute_numpy_dtype_name(specs[i].type) +
                                        ", matching the type its AttributeSpec declares");
        }
        if (array.ndim() == 0 || static_cast<std::size_t>(array.shape(0)) != rows) {
            throw std::invalid_argument(where + " must have one row per point (" +
                                        std::to_string(rows) + ")");
        }
        if (attribute_elements(array) != specs[i].count) {
            throw std::invalid_argument(where + " must have " + std::to_string(specs[i].count) +
                                        " value(s) per point");
        }
    }
}

// Packs rows [first, first + rows) of each attribute array, already validated
// by check_attribute_arrays() above, into the leading `rows` rows of `records`.
void pack_attribute_rows(const ouster::sdk::core::AttributeLayout& layout,
                         const std::vector<AnyArray>& attributes, std::size_t first,
                         std::size_t rows, ouster::sdk::core::AttributeRecords& records) {
    const auto& specs = layout.specs();
    for (std::size_t i = 0; i < attributes.size(); ++i) {
        ouster::sdk::core::visit_attribute_type(specs[i].type, [&](auto sample) {
            using T = decltype(sample);
            // Rows [first, first + rows) of the (C-contiguous) array.
            const std::size_t count = specs[i].count;
            layout.pack(i, static_cast<const T*>(attributes[i].data()) + first * count,
                        rows * count, records.topRows(static_cast<Eigen::Index>(rows)));
        });
    }
}

// An AttributedPointCloud as Python sees it: the Nx3 positions, and one numpy
// array per attribute in the type its layout declares. Bound as
// ouster.sdk.core.AttributedPointCloud; it also unpacks like an
// (xyz, attributes) pair.
struct PyAttributedPointCloud {
    py::ndarray<py::numpy, double, py::c_contig> xyz;
    py::list attributes;
};

// The inverse of pack_attribute_rows: each attribute shaped (rows,) for a
// scalar attribute and (rows, count) for a multi-element one.
PyAttributedPointCloud attributed_cloud_to_python(
    const ouster::sdk::core::AttributedPointCloud& cloud) {
    const std::size_t rows = cloud.size();
    auto xyz = make_array<double>({rows, 3});
    std::copy(cloud.xyz.data(), cloud.xyz.data() + cloud.xyz.size(),
              static_cast<double*>(xyz.data()));

    py::list attributes;
    const auto& specs = cloud.layout.specs();
    for (std::size_t i = 0; i < specs.size(); ++i) {
        const std::size_t count = static_cast<std::size_t>(specs[i].count);
        attributes.append(ouster::sdk::core::visit_attribute_type(specs[i].type, [&](auto sample) {
            using T = decltype(sample);
            auto array = count == 1 ? make_array<T>({rows}) : make_array<T>({rows, count});
            // Decoded straight into the new array's buffer.
            cloud.attribute(i, static_cast<T*>(array.data()), array.size());
            return py::cast(array);
        }));
    }
    return PyAttributedPointCloud{std::move(xyz), std::move(attributes)};
}

// The AttributeLayout `attributes` already describe: one spec per array, typed
// by the array's dtype and as wide as its values per row, so e.g. a uint32
// (N,) RANGE array and a float32 (N, 3) RGB array become [(U32, 1), (F32, 3)].
ouster::sdk::core::AttributeLayout attribute_layout_from_arrays(
    const std::vector<AnyArray>& attributes) {
    std::vector<ouster::sdk::core::AttributeSpec> specs;
    specs.reserve(attributes.size());
    for (std::size_t i = 0; i < attributes.size(); ++i) {
        const AnyArray& array = attributes[i];
        const auto* type = std::find_if(std::begin(ouster::sdk::core::ALL_ATTRIBUTE_TYPES),
                                        std::end(ouster::sdk::core::ALL_ATTRIBUTE_TYPES),
                                        [&](ouster::sdk::core::AttributeType candidate) {
                                            return array.dtype() == attribute_dtype(candidate);
                                        });
        if (type == std::end(ouster::sdk::core::ALL_ATTRIBUTE_TYPES)) {
            std::string supported;
            for (const auto candidate : ouster::sdk::core::ALL_ATTRIBUTE_TYPES) {
                if (!supported.empty()) {
                    supported += ", ";
                }
                supported += attribute_numpy_dtype_name(candidate);
            }
            throw std::invalid_argument("attribute " + std::to_string(i) +
                                        " has an unsupported dtype; expected one of: " + supported);
        }
        specs.push_back({*type, static_cast<std::uint32_t>(attribute_elements(array))});
    }
    return ouster::sdk::core::AttributeLayout(std::move(specs));
}

// Maps carrying an AttributeLayout additionally take and return their
// attributes as one numpy array per attribute, each in the type the layout
// declares, so callers never have to lay out the packed records themselves.
void bind_voxel_hash_map_attributed(py::class_<VoxelHashMap>& cls) {
    cls.def_prop_ro("attribute_layout",
                    [](const VoxelHashMap& self) { return self.attribute_layout(); })
        .def(
            "add_points",
            [](VoxelHashMap& self,
               const py::ndarray<py::numpy, const double, py::ndim<2>, py::c_contig>& xyz,
               const std::vector<AnyArray>& attributes) {
                if (xyz.shape(1) != 3) {
                    throw std::invalid_argument(
                        "add_points(xyz, attributes) expects xyz as an Nx3 array");
                }
                const std::size_t rows = static_cast<std::size_t>(xyz.shape(0));
                const ouster::sdk::core::AttributeLayout& layout = self.attribute_layout();
                // Validate everything up front so a bad array never leaves the
                // map with only some of the points inserted.
                check_attribute_arrays(layout, attributes, rows);
                const Eigen::Map<const ouster::sdk::core::ArrayX3dR> mapped(
                    xyz.data(), static_cast<Eigen::Index>(rows), 3);
                // Records are packed a block of rows at a time into one reused
                // buffer, so staging costs block_rows * stride bytes rather
                // than a full copy of every point's attributes.
                constexpr std::size_t block_rows = 4096;
                ouster::sdk::core::AttributeRecords records(
                    static_cast<Eigen::Index>(std::min(rows, block_rows)),
                    static_cast<Eigen::Index>(layout.stride()));
                for (std::size_t first = 0; first < rows; first += block_rows) {
                    const std::size_t count = std::min(block_rows, rows - first);
                    pack_attribute_rows(layout, attributes, first, count, records);
                    // topRows() of a row-major matrix is contiguous, so it binds
                    // to the map's Eigen::Ref without a copy.
                    self.add_points(mapped.middleRows(static_cast<Eigen::Index>(first),
                                                      static_cast<Eigen::Index>(count)),
                                    records.topRows(static_cast<Eigen::Index>(count)));
                }
            },
            py::arg("points"), py::arg("attributes"),
            R"pbdoc(
                Insert points with attributes kept in their native types.

                ``points`` is an Nx3 array of positions; ``attributes`` is one
                array per attribute of ``attribute_layout``, in its order, each
                already in the dtype its AttributeSpec declares -- so a uint32
                RANGE stays a uint32 rather than being widened to double.
            )pbdoc")
        .def(
            "pointcloud",
            [](const VoxelHashMap& self) { return attributed_cloud_to_python(self.pointcloud()); },
            R"pbdoc(
                Get every point in the map as an ``AttributedPointCloud``.

                ``.xyz`` is an Nx3 float64 array of positions -- all you need
                when the attributes don't matter. ``.attributes`` is one array
                per attribute of ``attribute_layout``, in its order and in its
                own dtype (shape ``(N,)`` for a scalar attribute, ``(N, count)``
                otherwise), empty for a map without an ``attribute_layout``.
                It unpacks like ``(xyz, attributes)``, the shape of data
                ``add_points(xyz, attributes)`` takes, so
                ``other.add_points(*voxel_map.pointcloud())`` copies a map's
                points.
            )pbdoc")
        .def(
            "extract_voxels_far_from_location",
            [](VoxelHashMap& self, const PointArray& point) {
                return attributed_cloud_to_python(
                    self.extract_voxels_far_from_location(map_point(point)));
            },
            py::arg("point"),
            R"pbdoc(
                Remove voxels far from ``point`` and return their points.

                The same as ``remove_voxels_far_from_location``, but returns what
                it removed as an ``AttributedPointCloud``, like ``pointcloud()``.
            )pbdoc");
}

// Positions plus, optionally, one numpy array per attribute in, the same shape
// of data out (as VoxelHashMap.pointcloud() returns it), each
// attribute keeping its own dtype. core::voxel_downsample() picks the
// positions-only or attributed implementation.
auto voxel_downsample_wrapper(
    const py::ndarray<py::numpy, const double, py::ndim<2>, py::c_contig>& points,
    double voxel_size, std::size_t max_points_per_voxel, std::size_t min_points_per_voxel,
    ouster::sdk::core::VoxelDownsampleStrategy strategy,
    const std::optional<std::vector<AnyArray>>& attributes_arg,
    ouster::sdk::core::VHMInternalStorageMode internal_storage_mode, std::size_t attribute_index) {
    if (points.shape(1) != 3) {
        throw std::invalid_argument("voxel_downsample: points must be Nx3");
    }
    const std::vector<AnyArray> attributes = attributes_arg.value_or(std::vector<AnyArray>{});
    const std::size_t rows = static_cast<std::size_t>(points.shape(0));
    const ouster::sdk::core::AttributeLayout layout = attribute_layout_from_arrays(attributes);
    check_attribute_arrays(layout, attributes, rows);

    // Downsampling consumes every point at once, so the records are packed in
    // one go rather than a block at a time.
    ouster::sdk::core::AttributeRecords records(static_cast<Eigen::Index>(rows),
                                                static_cast<Eigen::Index>(layout.stride()));
    pack_attribute_rows(layout, attributes, 0, rows, records);
    const Eigen::Map<const ouster::sdk::core::ArrayX3dR> xyz(points.data(),
                                                             static_cast<Eigen::Index>(rows), 3);
    return attributed_cloud_to_python(ouster::sdk::core::voxel_downsample(
        xyz, voxel_size, max_points_per_voxel, min_points_per_voxel, strategy, layout, records,
        internal_storage_mode, attribute_index));
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

namespace {
void check_destagger_image(const py::ndarray<py::numpy, py::ro, py::c_contig>& img) {
    if (img.ndim() < 2) {
        throw std::invalid_argument("Must have at least 2 dimensions.");
    }

    if (img.shape(0) == 0 || img.shape(1) == 0) {
        throw std::invalid_argument("Cannot have any dimensions of zero.");
    }
}

py::object destagger_lidar_info(const LidarInfo& lidar_info,
                                const py::ndarray<py::numpy, py::ro, py::c_contig>& img,
                                bool inverse = false) {
    check_destagger_image(img);
    if (img.shape(0) != lidar_info.format.pixels_per_column ||
        img.shape(1) != lidar_info.format.columns_per_frame ||
        img.shape(0) != lidar_info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument("Image resolution must match LidarInfo.");
    }
    return destagger2(img, lidar_info.format.pixel_shift_by_row, inverse);
}

py::object destagger_sensor_info(const SensorInfo& sensor_info,
                                 const py::ndarray<py::numpy, py::ro, py::c_contig>& img,
                                 bool inverse = false) {
    if (const auto* lidar = dynamic_cast<const LidarInfo*>(&sensor_info)) {
        return destagger_lidar_info(*lidar, img, inverse);
    }

    check_destagger_image(img);
    if (img.shape(0) != sensor_info.h() || img.shape(1) != sensor_info.w()) {
        throw std::invalid_argument("Image resolution must match SensorInfo.");
    }

    // Non-lidar sensors (e.g. cameras) are not staggered: destagger/stagger is a
    // no-op implemented via zero per-row pixel shifts.
    std::vector<int> pixel_shift_by_row(img.shape(0), 0);
    return destagger2(img, pixel_shift_by_row, inverse);
}
}  // namespace

// NOLINTNEXTLINE(misc-use-internal-linkage)
void init_client_processing(py::module_& module, py::module_& /*unused*/) {
    module.def(
        "destagger", &destagger_lidar_info, py::arg("sensor_info"), py::arg("img"),
        py::arg("inverse") = false,
        py::sig("def destagger(sensor_info: LidarInfo, img: Annotated[NDArray, dict(order='C', "
                "writable=False)], inverse: bool = False) -> Annotated[NDArray, dict(order='C')]"));
    module.def("destagger", &destagger2, py::arg("img"), py::arg("pixel_shift_by_row"),
               py::arg("inverse") = false,
               py::sig("def destagger(img: Annotated[NDArray, dict(order='C', writable=False)], "
                       "pixel_shift_by_row: Sequence[int], inverse: bool = False) -> "
                       "Annotated[NDArray, dict(order='C')]"));
    module.def(
        "destagger", &destagger_sensor_info, py::arg("sensor_info"), py::arg("img"),
        py::arg("inverse") = false,
        py::sig("def destagger(sensor_info: SensorInfo, img: Annotated[NDArray, dict(order='C', "
                "writable=False)], inverse: bool = False) -> Annotated[NDArray, dict(order='C')]"));

    py::class_<FrameBatcher>(module, "FrameBatcher")
        .def(py::init<std::shared_ptr<LidarInfo>>())
        .def("reset", &FrameBatcher::reset)
        .def("batched_packets", &FrameBatcher::batched_packets)
        .def("dropped_packets", &FrameBatcher::dropped_packets)
        .def("set_max_cache_size", &FrameBatcher::set_max_cache_size)
        .def("get_max_cache_size", &FrameBatcher::get_max_cache_size)
        .def("finalize_frame", &FrameBatcher::finalize_frame)
        .def("batch", &FrameBatcher::batch);

    // Constructed with py::new_ so that each LUT is owned by a std::shared_ptr.
    // Through enable_shared_from_this, every later conversion to a shared_ptr
    // then shares that ownership, so APIs that key on LUT identity, like
    // viz::Cloud, recognize the same Python LUT across calls.
    py::class_<XYZLutT<float>>(module, "XYZLutFloat")
        .def(py::new_(
                 [](const SensorInfo& sensor) { return std::make_shared<XYZLutT<float>>(sensor); }),
             py::arg("info"), R"doc(
Return a function that can project frames into Cartesian coordinates.

If called with a numpy array representing a range image, the range image
must be in "staggered" form, where each column corresponds to a single
measurement block. LidarFrame fields are always staggered.

Internally, this will pre-compute a lookup table using the supplied
intrinsic parameters. XYZ points are returned as a H x W x 3 array of
doubles, where H is the number of beams and W is the horizontal resolution
of the frame.

The result is returned in the "extrinsics frame" by applying the homogeneous 4x4 transform
matrix from ``info.sensor_to_body`` property.

Args:
    info: sensor metadata

Returns:
    A function that computes a point cloud given a range image)doc")
        .def("__call__", &xyzlut_call_any<float>, py::arg("range"))
        .def("__call__", &xyzlut_call_frame<float>, py::arg("frame"))
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
        .def(py::new_([](const SensorInfo& sensor) {
                 return std::make_shared<XYZLutT<double>>(sensor);
             }),
             py::arg("info"), R"doc(
Return a function that can project frames into Cartesian coordinates.

If called with a numpy array representing a range image, the range image
must be in "staggered" form, where each column corresponds to a single
measurement block. LidarFrame fields are always staggered.

Internally, this will pre-compute a lookup table using the supplied
intrinsic parameters. XYZ points are returned as a H x W x 3 array of
doubles, where H is the number of beams and W is the horizontal resolution
of the frame.

The result is returned in the "extrinsics frame" by applying the homogeneous 4x4 transform
matrix from ``info.sensor_to_body`` property.

Args:
    info: sensor metadata

Returns:
    A function that computes a point cloud given a range image)doc")
        .def("__call__", &xyzlut_call_any<double>, py::arg("range"))
        .def("__call__", &xyzlut_call_frame<double>, py::arg("frame"))
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
               double compress_dr_max_lum, bool color_correct, bool x_wrap) {
                new (self) LocalToneMapper(low, high, update_every, damping, compress_dr_max_lum,
                                           color_correct, x_wrap);
            },
            py::arg("lo_percentile"), py::arg("hi_percentile"), py::arg("update_every"),
            py::arg("damping"), py::arg("compress_dr_max_lum"), py::arg("color_correct"),
            py::arg("x_wrap") = true)
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
    poses: A NumPy array of shape (W, 4, 4) with one pose per image column
        (lidar rolling shutter), or (1, 4, 4) with a single rigid pose
        broadcast to every column (e.g. CameraFrame).

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

    // Registered ahead of every def() using it as a default argument: nanobind
    // converts defaults when the function is defined.
    py::enum_<ouster::sdk::core::VHMInternalStorageMode>(module, "VHMInternalStorageMode")
        .value("AUTO", ouster::sdk::core::VHMInternalStorageMode::AUTO)
        .value("ABS", ouster::sdk::core::VHMInternalStorageMode::ABS)
        .value("FX16", ouster::sdk::core::VHMInternalStorageMode::FX16)
        .value("FX8", ouster::sdk::core::VHMInternalStorageMode::FX8);

    py::enum_<ouster::sdk::core::VoxelDownsampleStrategy>(module, "VoxelDownsampleStrategy")
        .value("FIRST_N_POINT", ouster::sdk::core::VoxelDownsampleStrategy::FIRST_N_POINT)
        .value("LAST_N_POINT", ouster::sdk::core::VoxelDownsampleStrategy::LAST_N_POINT)
        .value("AVERAGE_POINT", ouster::sdk::core::VoxelDownsampleStrategy::AVERAGE_POINT)
        .value("RANDOM", ouster::sdk::core::VoxelDownsampleStrategy::RANDOM)
        .value("SMALLEST", ouster::sdk::core::VoxelDownsampleStrategy::SMALLEST)
        .value("LARGEST", ouster::sdk::core::VoxelDownsampleStrategy::LARGEST);

    module.def(
        "voxel_downsample", &voxel_downsample_wrapper, py::arg("points"), py::arg("voxel_size"),
        py::arg("max_points_per_voxel") = VoxelHashMap::DEFAULT_MAX_POINTS_PER_VOXEL,
        py::arg("min_points_per_voxel") = VoxelHashMap::DEFAULT_MIN_POINTS_PER_VOXEL,
        py::arg("strategy") = VoxelHashMap::DEFAULT_STRATEGY, py::arg("attributes") = py::none(),
        py::arg("internal_storage_mode") = VoxelHashMap::DEFAULT_INTERNAL_STORAGE_MODE,
        py::arg("attribute_index") = std::size_t{0},
        R"(
        Downsample a pointcloud, optionally with attributes that keep their own types.

        Attributes travel as one numpy array per attribute, exactly as
        :meth:`VoxelHashMap.add_points` (the ``(xyz, attributes)`` overload) takes them and
        :meth:`VoxelHashMap.pointcloud` returns them, so a uint32
        RANGE stays a uint32 rather than being widened to double. Each array's
        dtype and width (its values per row) determine the attribute layout.
        Leave ``attributes`` out to downsample positions only.

        The parameters follow :meth:`VoxelHashMap.create`, in the same order and
        with the same defaults; ``attributes`` takes the place of its
        ``attribute_layout``, and there is no ``max_distance``, since
        downsampling never trims.

        For positions only::

            xyz, _ = voxel_downsample(points, voxel_size)

        To migrate from the 1.0 form taking one Nx(3+K) float64 array, pass the
        attribute columns as one array and rebuild the old array from the
        result::

            xyz, attrs = voxel_downsample(frame[:, :3], voxel_size,
                                          attributes=[frame[:, 3:]])
            out = np.column_stack([xyz, *attrs])

        See the "Migrating from 1.0.0 to 2.0.0" guide.

        Args:
            points: Nx3 float64 array of positions (x,y,z).
            voxel_size: Edge length of each cubic voxel.
            max_points_per_voxel: Maximum points kept per voxel (FIRST_N_POINT, LAST_N_POINT, RANDOM).
            min_points_per_voxel: Minimum insertions before a voxel contributes.
            strategy: The VoxelDownsampleStrategy selecting the per-voxel
                reduction. With AVERAGE_POINT an integer attribute's per-voxel
                mean is rounded to the nearest representable value. With
                SMALLEST or LARGEST, the points kept per voxel are the ones
                with the smallest, respectively largest, value of the scalar
                attribute named by attribute_index; attributes must not be
                empty.
            attributes: One array per attribute, each with N rows: shape (N,) or
                (N, 1) for a scalar attribute, (N, K) for a K-element one, in any
                dtype listed by ``AttributeType``. None (the default) for
                positions only.
            internal_storage_mode: How points are stored inside the voxels. AUTO
                picks fixed point for small voxels, which returns kept positions
                within a small fraction of the voxel size of the input; ABS
                returns them exactly.
            attribute_index: Index into ``attributes`` of the scalar attribute to
                rank by; defaults to the first attribute (0). Ignored unless
                strategy is SMALLEST or LARGEST, where it must name a scalar
                attribute.

        Returns:
            ``(xyz, attributes)``: the Mx3 kept positions (M <= N), and one array
            per attribute in its input dtype, shaped (M,) for a scalar attribute
            and (M, K) otherwise; an empty list without attributes.
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

    py::enum_<ouster::sdk::core::AttributeType>(module, "AttributeType")
        .value("U8", ouster::sdk::core::AttributeType::U8)
        .value("U16", ouster::sdk::core::AttributeType::U16)
        .value("U32", ouster::sdk::core::AttributeType::U32)
        .value("U64", ouster::sdk::core::AttributeType::U64)
        .value("I8", ouster::sdk::core::AttributeType::I8)
        .value("I16", ouster::sdk::core::AttributeType::I16)
        .value("I32", ouster::sdk::core::AttributeType::I32)
        .value("I64", ouster::sdk::core::AttributeType::I64)
        .value("F16", ouster::sdk::core::AttributeType::F16)
        .value("F32", ouster::sdk::core::AttributeType::F32)
        .value("F64", ouster::sdk::core::AttributeType::F64)
        .def_prop_ro("dtype", &attribute_numpy_dtype,
                     "The numpy dtype attributes of this type are passed and returned in.")
        .def_static(
            "from_dtype", [](const py::object& dtype) { return attribute_type_from_dtype(dtype); },
            py::arg("dtype"),
            "The AttributeType for a numpy dtype (np.uint32, np.dtype('uint32'), "
            "'uint32', ...); raises ValueError for dtypes a map cannot store.");

    py::class_<ouster::sdk::core::AttributeSpec>(module, "AttributeSpec")
        .def(
            "__init__",
            [](ouster::sdk::core::AttributeSpec* self, ouster::sdk::core::AttributeType type,
               std::uint32_t count) {
                new (self) ouster::sdk::core::AttributeSpec{type, count};
            },
            py::arg("type"), py::arg("count") = 1)
        .def_rw("type", &ouster::sdk::core::AttributeSpec::type)
        .def_rw("count", &ouster::sdk::core::AttributeSpec::count)
        .def("__repr__", [](const ouster::sdk::core::AttributeSpec& self) {
            return std::string("AttributeSpec(") +
                   ouster::sdk::core::attribute_type_name(self.type) + ", " +
                   std::to_string(self.count) + ")";
        });

    py::class_<PyAttributedPointCloud>(module, "AttributedPointCloud", R"pbdoc(
        Points returned by ``VoxelHashMap.pointcloud()``,
        ``VoxelHashMap.extract_voxels_far_from_location()`` and ``voxel_downsample()``.

        ``xyz`` is an Nx3 float64 array of positions; ``attributes`` is one array
        per attribute of the layout, in its order and own dtype. Use ``.xyz``
        when only the positions matter. It also unpacks like an
        ``(xyz, attributes)`` pair, so ``xyz, attributes = voxel_map.pointcloud()``
        and ``other.add_points(*voxel_map.pointcloud())`` work too. ``len()`` is
        the number of points.
    )pbdoc")
        // By value: the arrays already own their buffers, so the default
        // reference_internal policy does not apply to them.
        .def_prop_ro(
            "xyz", [](const PyAttributedPointCloud& self) { return self.xyz; },
            py::rv_policy::automatic, "Nx3 float64 array of positions.")
        .def_prop_ro(
            "attributes", [](const PyAttributedPointCloud& self) { return self.attributes; },
            py::rv_policy::automatic, "One array per attribute, in layout order and its own dtype.")
        // The number of points, as len() of the 1.0 Nx(3+K) array returned. This
        // is not the number of items __iter__ yields: unpacking always gives
        // exactly (xyz, attributes).
        .def("__len__",
             [](const PyAttributedPointCloud& self) {
                 return static_cast<std::size_t>(self.xyz.shape(0));
             })
        .def("__iter__",
             [](const PyAttributedPointCloud& self) {
                 return py::iter(py::make_tuple(self.xyz, self.attributes));
             })
        .def("__getitem__",
             [](const PyAttributedPointCloud& self, Py_ssize_t index) -> py::object {
                 if (index == 0 || index == -2) {
                     return py::cast(self.xyz);
                 }
                 if (index == 1 || index == -1) {
                     return self.attributes;
                 }
                 throw py::index_error("AttributedPointCloud index out of range");
             })
        .def("__repr__", [](const PyAttributedPointCloud& self) {
            return "AttributedPointCloud(" + std::to_string(self.xyz.shape(0)) + " points, " +
                   std::to_string(self.attributes.size()) + " attribute(s))";
        });

    py::class_<ouster::sdk::core::AttributeLayout>(module, "AttributeLayout")
        .def(py::init<>())
        // Same spellings VoxelHashMap.create()'s attribute_layout argument accepts:
        // AttributeSpecs and/or (dtype, count) pairs.
        .def(
            "__init__",
            [](ouster::sdk::core::AttributeLayout* self, const py::object& specs) {
                new (self) ouster::sdk::core::AttributeLayout(attribute_layout_from_python(specs));
            },
            py::arg("specs"))
        .def_prop_ro("specs",
                     [](const ouster::sdk::core::AttributeLayout& self) {
                         return std::vector<ouster::sdk::core::AttributeSpec>(self.specs());
                     })
        .def_prop_ro("stride", &ouster::sdk::core::AttributeLayout::stride)
        .def_prop_ro("element_count", &ouster::sdk::core::AttributeLayout::element_count)
        .def("byte_offset", &ouster::sdk::core::AttributeLayout::byte_offset, py::arg("spec_index"))
        .def("__len__",
             [](const ouster::sdk::core::AttributeLayout& self) { return self.specs().size(); })
        .def("__eq__",
             [](const ouster::sdk::core::AttributeLayout& self, const py::object& other) {
                 if (!py::isinstance<ouster::sdk::core::AttributeLayout>(other)) {
                     return false;
                 }
                 return self == py::cast<const ouster::sdk::core::AttributeLayout&>(other);
             })
        // Consistent with __eq__: equal layouts (same specs in the same order)
        // hash equal.
        .def("__hash__", [](const ouster::sdk::core::AttributeLayout& self) {
            std::size_t hash = std::hash<std::size_t>{}(self.specs().size());
            for (const ouster::sdk::core::AttributeSpec& spec : self.specs()) {
                const std::size_t spec_hash =
                    (static_cast<std::size_t>(spec.type) << 32) ^ spec.count;
                hash ^= std::hash<std::size_t>{}(spec_hash) + 0x9e3779b97f4a7c15ULL + (hash << 6) +
                        (hash >> 2);
            }
            return hash;
        });

    {
        auto cls = py::class_<VoxelHashMap>(module, "VoxelHashMap", R"(
        A spatial hash map organizing 3D points, and optionally per-point
        attributes, into cubic voxels.

        Build one with :meth:`create`, which picks the implementation of the
        requested :class:`VoxelDownsampleStrategy`: a positions-only one when
        no ``attribute_layout`` is given, an attributed one otherwise.
        )");
        cls.def_static(
               "create",
               [](double voxel_size, std::size_t max_points_per_voxel,
                  std::size_t min_points_per_voxel,
                  ouster::sdk::core::VoxelDownsampleStrategy strategy,
                  const py::object& attribute_layout, double max_distance,
                  ouster::sdk::core::VHMInternalStorageMode internal_storage_mode,
                  std::size_t attribute_index) {
                   return VoxelHashMap::create(
                       voxel_size, max_points_per_voxel, min_points_per_voxel, strategy,
                       attribute_layout.is_none() ? ouster::sdk::core::AttributeLayout()
                                                  : attribute_layout_from_python(attribute_layout),
                       max_distance, internal_storage_mode, attribute_index);
               },
               py::arg("voxel_size"),
               py::arg("max_points_per_voxel") = VoxelHashMap::DEFAULT_MAX_POINTS_PER_VOXEL,
               py::arg("min_points_per_voxel") = VoxelHashMap::DEFAULT_MIN_POINTS_PER_VOXEL,
               py::arg("strategy") = VoxelHashMap::DEFAULT_STRATEGY,
               py::arg("attribute_layout") = py::none(),
               py::arg("max_distance") = VoxelHashMap::DEFAULT_MAX_DISTANCE,
               py::arg("internal_storage_mode") = VoxelHashMap::DEFAULT_INTERNAL_STORAGE_MODE,
               py::arg("attribute_index") = std::size_t{0},
               R"(
        Build the voxel map implementing ``strategy``.

        The parameters follow :func:`voxel_downsample`, in the same order and
        with the same defaults; ``attribute_layout`` takes the place of its
        ``attributes``, and ``max_distance`` has no downsampling counterpart.

        Args:
            voxel_size: Edge length of each cubic voxel.
            max_points_per_voxel: Maximum points kept per voxel (FIRST_N_POINT, LAST_N_POINT, RANDOM).
            min_points_per_voxel: Minimum insertions before a voxel contributes.
            strategy: The VoxelDownsampleStrategy selecting the per-voxel
                reduction. With SMALLEST or LARGEST, the points kept per voxel
                are the ones with the smallest, respectively largest, value of
                the scalar attribute named by attribute_index;
                attribute_layout must not be empty.
            attribute_layout: Types and widths of the per-point attributes, as an
                AttributeLayout or a list of AttributeSpec/(dtype, count) pairs,
                e.g. ``[(np.uint32, 1), (np.uint8, 1)]``. None (the default) for
                positions only.
            max_distance: Maximum distance to retain points from a reference.
            internal_storage_mode: How points are stored inside the voxels. AUTO
                picks fixed point for small voxels, which returns kept positions
                within a small fraction of the voxel size of the input; ABS
                returns them exactly.
            attribute_index: Index into ``attribute_layout`` of the scalar
                attribute to rank by; defaults to the first attribute (0).
                Ignored unless strategy is SMALLEST or LARGEST, where it must
                name a scalar attribute.

        Returns:
            The proxy, owning the selected map.
        )")
            .def_prop_ro("strategy", &VoxelHashMap::strategy)
            .def_prop_ro("voxel_size", &VoxelHashMap::voxel_size)
            .def_prop_ro("max_distance", &VoxelHashMap::max_distance)
            .def_prop_ro("max_points_per_voxel", &VoxelHashMap::max_points_per_voxel)
            .def_prop_ro("min_points_per_voxel", &VoxelHashMap::min_points_per_voxel)
            .def_prop_ro("point_cols", &VoxelHashMap::point_cols);
        bind_voxel_hash_map_common(cls);
        bind_voxel_hash_map_attributed(cls);
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

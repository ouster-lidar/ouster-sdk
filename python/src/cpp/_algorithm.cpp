/*
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * Python bindings for shared Ouster algorithms.
 */

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "common.h"
#include "ndarray_helpers.h"
#include "nonstd/optional.hpp"
#include "ouster/algorithm/align_clouds.h"
#include "ouster/algorithm/ground_seg.h"
#include "ouster/algorithm/normals.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/lidar_frame.h"

using namespace ouster::sdk::algorithm;
using namespace ouster::sdk::core;
using ouster::sdk::python::ensure_c_contig;
using ouster::sdk::python::ensure_c_contig_floating;

using Mat4Input = py::ndarray<py::numpy, const double, py::shape<4, 4>, py::c_contig>;
using ArrayX3Input = py::ndarray<py::numpy, const double, py::shape<-1, 3>, py::c_contig>;

template <typename T, int Ndim = 1>
py::ndarray<py::numpy, T, py::ndim<Ndim>, py::c_contig> make_algorithm_array(
    const std::vector<size_t>& shape) {
    size_t size = 0;
    for (const auto dim : shape) {
        size = (size == 0) ? dim : size * dim;
    }
    T* data = new T[size];
    auto capsule =
        py::capsule(data, [](void* pointer) noexcept { delete[] static_cast<T*>(pointer); });
    return {data, shape.size(), shape.data(), capsule};
}

namespace {

void fill_initial_guess(Matrix4dR& out, const py::object& initial_guess) {
    out = Matrix4dR::Identity();
    if (initial_guess.is_none()) {
        return;
    }
    auto arr = py::cast<Mat4Input>(initial_guess);
    auto view = arr.view();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            out(row, col) = view(row, col);
        }
    }
}

py::object pose_delta_result(const Matrix4dR& pose_delta, bool compute_confidence,
                             double confidence) {
    auto pose_array = make_algorithm_array<double, 2>({4, 4});
    auto pose_view = pose_array.view();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            pose_view(row, col) = pose_delta(row, col);
        }
    }
    py::object pose_obj = py::cast(pose_array);
    if (!compute_confidence) {
        return pose_obj;
    }
    return py::cast(std::make_tuple(pose_obj, confidence));
}

Eigen::Map<const ArrayX3dR> ndarray_to_arrayx3(const ArrayX3Input& points) {
    return Eigen::Map<const ArrayX3dR>(points.data(), static_cast<Eigen::Index>(points.shape(0)),
                                       3);
}

py::object matrix4_vector_to_numpy_view(const std::vector<Matrix4dR>& transforms) {
    auto out = make_algorithm_array<double, 3>({transforms.size(), 4, 4});
    auto view = out.view();
    for (size_t i = 0; i < transforms.size(); ++i) {
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                view(i, row, col) = transforms[i](row, col);
            }
        }
    }
    return py::cast(out);
}

py::object align_clouds_frames_py(const LidarFrame& source_frame, const LidarFrame& target_frame,
                                  const py::object& initial_guess, bool compute_confidence) {
    Matrix4dR init;
    fill_initial_guess(init, initial_guess);
    if (!compute_confidence) {
        return pose_delta_result(align_clouds(source_frame, target_frame, init), false, -1.0);
    }
    double confidence = -1.0;
    const Matrix4dR pose_delta = align_clouds(source_frame, target_frame, init, confidence);
    return pose_delta_result(pose_delta, true, confidence);
}

py::object align_clouds_frame_set_py(const FrameSet& frame_set) {
    return matrix4_vector_to_numpy_view(align_clouds(frame_set));
}

py::object align_clouds_points_py(const ArrayX3Input& source_points,
                                  const ArrayX3Input& target_points,
                                  const py::object& initial_guess, bool compute_confidence) {
    const auto source_points_map = ndarray_to_arrayx3(source_points);
    const auto target_points_map = ndarray_to_arrayx3(target_points);
    Matrix4dR init;
    fill_initial_guess(init, initial_guess);
    if (!compute_confidence) {
        return pose_delta_result(align_clouds(source_points_map, target_points_map, init), false,
                                 -1.0);
    }
    double confidence = -1.0;
    const Matrix4dR pose_delta =
        align_clouds(source_points_map, target_points_map, init, confidence);
    return pose_delta_result(pose_delta, true, confidence);
}

py::object align_clouds_points_with_normals_py(const ArrayX3Input& source_points,
                                               const ArrayX3Input& source_normals,
                                               const ArrayX3Input& target_points,
                                               const ArrayX3Input& target_normals,
                                               const py::object& initial_guess,
                                               bool compute_confidence) {
    const auto source_points_map = ndarray_to_arrayx3(source_points);
    const auto source_normals_map = ndarray_to_arrayx3(source_normals);
    const auto target_points_map = ndarray_to_arrayx3(target_points);
    const auto target_normals_map = ndarray_to_arrayx3(target_normals);
    Matrix4dR init;
    fill_initial_guess(init, initial_guess);
    if (!compute_confidence) {
        return pose_delta_result(align_clouds(source_points_map, source_normals_map,
                                              target_points_map, target_normals_map, init),
                                 false, -1.0);
    }
    double confidence = -1.0;
    const Matrix4dR pose_delta =
        align_clouds(source_points_map, source_normals_map, target_points_map, target_normals_map,
                     init, confidence);
    return pose_delta_result(pose_delta, true, confidence);
}

py::object point_to_point_align_py(const ArrayX3Input& source_points,
                                   const ArrayX3Input& target_points,
                                   const py::object& initial_guess, double max_corr_dist) {
    Matrix4dR init;
    fill_initial_guess(init, initial_guess);
    return pose_delta_result(
        point_to_point_align(ndarray_to_arrayx3(source_points), ndarray_to_arrayx3(target_points),
                             init, max_corr_dist),
        false, -1.0);
}

py::object point_to_plane_align_py(const ArrayX3Input& source_points,
                                   const ArrayX3Input& target_points,
                                   const ArrayX3Input& source_normals,
                                   const ArrayX3Input& target_normals,
                                   const py::object& initial_guess, double max_corr_dist,
                                   double max_normal_angle_deg) {
    Matrix4dR init;
    fill_initial_guess(init, initial_guess);
    return pose_delta_result(
        point_to_plane_align(ndarray_to_arrayx3(source_points), ndarray_to_arrayx3(target_points),
                             ndarray_to_arrayx3(source_normals), ndarray_to_arrayx3(target_normals),
                             init, max_corr_dist, max_normal_angle_deg),
        false, -1.0);
}

}  // namespace

void init_algorithm_align_clouds(py::module_& module) {
    module.def("point_to_point_align", &point_to_point_align_py, py::arg("source_points"),
               py::arg("target_points"), py::arg("initial_guess") = py::none(),
               py::arg("max_corr_dist") = 0.25,
               R"pbdoc(
            Align two point clouds using point-to-point ICP.

            This helper returns ``source_to_target_transform``: ``source_points``
            are transformed by the returned pose and ``target_points`` stay
            fixed as the reference cloud.

            Args:
                source_points: ``(N, 3)`` source points transformed by the
                    result.
                target_points: ``(M, 3)`` reference target points.
                initial_guess: Optional initial ``source_to_target_transform``.
                max_corr_dist: Maximum correspondence distance in meters.

            Returns:
                A ``(4, 4)`` numpy array containing
                ``source_to_target_transform``. The initial guess is returned when
                fewer than 20 usable points or correspondences are available.

            Non-finite points are ignored. Correspondences are weighted with a
            MAD-scaled Huber loss.
        )pbdoc");

    module.def("point_to_plane_align", &point_to_plane_align_py, py::arg("source_points"),
               py::arg("target_points"), py::arg("source_normals"), py::arg("target_normals"),
               py::arg("initial_guess") = py::none(), py::arg("max_corr_dist") = 0.25,
               py::arg("max_normal_angle_deg") = 20.0,
               R"pbdoc(
            Align two point clouds using point-to-plane ICP.

            This helper returns ``source_to_target_transform``: ``source_points``
            are transformed by the returned pose and ``target_points`` stay
            fixed as the reference cloud.

            Args:
                source_points: ``(N, 3)`` source points transformed by the
                    result.
                target_points: ``(M, 3)`` reference target points.
                source_normals: ``(N, 3)`` normals aligned with the source
                    points.
                target_normals: ``(M, 3)`` normals aligned with the reference
                    target points.
                initial_guess: Optional initial ``source_to_target_transform``.
                max_corr_dist: Maximum correspondence distance in meters.
                max_normal_angle_deg: Maximum normal-angle difference in
                    degrees.

            Returns:
                A ``(4, 4)`` numpy array containing
                ``source_to_target_transform``. The initial guess is returned when
                fewer than 20 usable points or correspondences are available.

            Non-finite points and normals are ignored, normals are normalized
            internally, and correspondences are weighted with a MAD-scaled
            Huber loss.
        )pbdoc");

    module.def("align_clouds", &align_clouds_frame_set_py, py::arg("frames"),
               R"pbdoc(
            Align every valid frame in a `FrameSet` to the first frame.

            The frame set must contain a valid frame at index 0, which is used as
            the anchor. Each frame's `sensor_info.sensor_to_body` should already hold
            the gravity-aligned plumb extrinsic; the full matrix is used as
            the initial extrinsic prior. The function returns a `(N, 4, 4)`
            numpy array with one updated extrinsic matrix per input frame: the
            first matrix is frame 0's input extrinsic. Each later matrix is the
            final extrinsic equal to that frame's input extrinsic left-multiplied
            by the estimated correction, following the
            ``source_to_target_transform`` convention.
            Two valid frames use the pairwise alignment path directly; three or
            more valid frames use the multi-pass spanning-tree matcher.
        )pbdoc");

    module.def("align_clouds", &align_clouds_frames_py, py::arg("source_frame"),
               py::arg("target_frame"), py::arg("initial_guess") = py::none(),
               py::arg("compute_confidence") = false,
               R"pbdoc(
            Estimate `source_to_target_transform` from two `LidarFrame` inputs.

            This overload performs frame preprocessing internally: it uses
            RANGE, dewarps with per-column poses, and uses frame normals when
            available (otherwise it estimates normals). Each frame's
            `sensor_info.sensor_to_body` is applied while preparing features:
            its rotation should be gravity-aligned, and the full matrix,
            including translation, is used as the initial extrinsic prior. The
            caller is responsible for setting a gravity-aligned extrinsic
            (e.g. computed from IMU data) on each frame before calling.

            Ground segmentation is applied to filter ground points from the XY
            features used for initial yaw/translation matching. If the frame
            already carries a ``"GROUND"`` field (uint8, written by
            ``GroundSegEngine``), it is reused directly; otherwise ground
            segmentation is computed on the fly.

            Args:
                source_frame: Source frame transformed by the returned pose.
                target_frame: Reference target frame.
                initial_guess: Optional initial `source_to_target_transform` (defaults to identity).
                compute_confidence: If True, also returns confidence.

            Returns:
                - `compute_confidence=False`: `np.ndarray` of shape `(4, 4)`
                  containing `source_to_target_transform`.
                - `compute_confidence=True`: `(pose_delta, confidence)` where
                  `pose_delta` is `(4, 4)` and `confidence` is clamped to
                  `[0.0, 1.0]`.

            Confidence:
                Symmetric overlap fraction: points from each cloud are checked
                for a nearby point in the other cloud after applying the
                estimated pose (`<= 0.5 m` in XY). If normals are available on
                both sides, a normal-angle gate (`<= 5 deg`) is also required.
        )pbdoc");

    module.def("align_clouds", &align_clouds_points_py, py::arg("source_points"),
               py::arg("target_points"), py::arg("initial_guess") = py::none(),
               py::arg("compute_confidence") = false,
               R"pbdoc(
            Estimate `source_to_target_transform` from two point clouds.

            Inputs are expected to be `(N, 3)` and `(M, 3)` already in a
            gravity-aligned common frame. The caller is responsible for
            applying any IMU-based gravity rotation and dewarping before
            calling.

            Args:
                source_points: `(N, 3)` points transformed by the result.
                target_points: `(M, 3)` reference target points.
                initial_guess: Optional initial `source_to_target_transform` (defaults to identity).
                compute_confidence: If True, also returns confidence.

            Returns:
                - `compute_confidence=False`: `np.ndarray` of shape `(4, 4)`
                  containing `source_to_target_transform`.
                - `compute_confidence=True`: `(pose_delta, confidence)` where
                  `pose_delta` is `(4, 4)` and `confidence` is clamped to
                  `[0.0, 1.0]`.

            Confidence:
                Symmetric overlap fraction: points from each cloud are checked
                for a nearby point in the other cloud after applying the
                estimated pose (`<= 0.5 m` in XY).
        )pbdoc");

    module.def("align_clouds", &align_clouds_points_with_normals_py, py::arg("source_points"),
               py::arg("source_normals"), py::arg("target_points"), py::arg("target_normals"),
               py::arg("initial_guess") = py::none(), py::arg("compute_confidence") = false,
               R"pbdoc(
            Estimate `source_to_target_transform` from point clouds with normals.

            This overload expects:

            - `source_points.shape == source_normals.shape == (N, 3)`
            - `target_points.shape == target_normals.shape == (M, 3)`

            Inputs must already be in a gravity-aligned common frame. The
            caller is responsible for applying any IMU-based gravity rotation
            and dewarping (including rotating the normals) before calling.
            Non-finite points/normals are ignored and normals are normalized
            internally.

            Args:
                source_points: `(N, 3)` points transformed by the result.
                source_normals: `(N, 3)` normals aligned to `source_points`.
                target_points: `(M, 3)` reference target points.
                target_normals: `(M, 3)` normals aligned to `target_points`.
                initial_guess: Optional initial `source_to_target_transform` (defaults to identity).
                compute_confidence: If True, also returns confidence.

            Returns:
                - `compute_confidence=False`: `np.ndarray` of shape `(4, 4)`
                  containing `source_to_target_transform`.
                - `compute_confidence=True`: `(pose_delta, confidence)` where
                  `pose_delta` is `(4, 4)` and `confidence` is clamped to
                  `[0.0, 1.0]`.

            Confidence:
                Symmetric overlap fraction: points from each cloud are checked
                for a nearby point in the other cloud after applying the
                estimated pose (`<= 0.5 m` in XY), plus a normal-angle gate
                (`<= 5 deg`).
        )pbdoc");
}

namespace {
std::vector<py::ndarray<py::numpy, double, py::c_contig>> process_normals(
    const py::ndarray<const double, py::c_contig, py::shape<-1, -1, 3>>& xyz,
    const py::ndarray<const uint32_t, py::c_contig, py::shape<-1, -1>>& range,
    const py::ndarray<const double, py::c_contig, py::shape<-1, 3>>& sensor_origins,
    int pixel_search_range, double max_angle_of_incidence_rad, double target_distance_m,
    const nonstd::optional<py::ndarray<const double, py::c_contig, py::shape<-1, -1, 3>>>& xyz2 =
        nonstd::nullopt,
    const nonstd::optional<py::ndarray<const uint32_t, py::c_contig, py::shape<-1, -1>>>& range2 =
        nonstd::nullopt) {
    if (xyz2.has_value() != range2.has_value()) {
        throw std::runtime_error("Either both xyz2 and range2 must be provided, or neither.");
    }
    if (pixel_search_range < 0) {
        throw std::invalid_argument("pixel_search_range must be non-negative");
    }

    const int h = static_cast<int>(xyz.shape(0));
    const int w = static_cast<int>(xyz.shape(1));
    const Eigen::Index num_points = static_cast<Eigen::Index>(h) * static_cast<Eigen::Index>(w);

    Eigen::Map<const ouster::sdk::core::MatrixX3dR> sensor_origins_map{
        static_cast<const double*>(sensor_origins.data()),
        static_cast<Eigen::Index>(sensor_origins.shape(0)),
        static_cast<Eigen::Index>(sensor_origins.shape(1))};

    Eigen::Map<const ouster::sdk::core::PointCloudXYZd> first_xyz_map{
        static_cast<const double*>(xyz.data()), num_points, 3};
    Eigen::Map<const ouster::sdk::core::img_t<uint32_t>> first_range_map{
        static_cast<const uint32_t*>(range.data()), static_cast<Eigen::Index>(range.shape(0)),
        static_cast<Eigen::Index>(range.shape(1))};

    std::vector<ouster::sdk::core::MatrixX3dR> normals_vec;
    if (xyz2.has_value()) {
        Eigen::Map<const ouster::sdk::core::PointCloudXYZd> second_xyz_map{
            static_cast<const double*>(xyz2->data()),
            static_cast<Eigen::Index>(xyz2->shape(0) * xyz2->shape(1)), 3};
        Eigen::Map<const ouster::sdk::core::img_t<uint32_t>> second_range_map{
            static_cast<const uint32_t*>(range2->data()),
            static_cast<Eigen::Index>(range2->shape(0)),
            static_cast<Eigen::Index>(range2->shape(1))};
        auto normals_pair = ouster::sdk::algorithm::normals(
            first_xyz_map, first_range_map, second_xyz_map, second_range_map, sensor_origins_map,
            pixel_search_range, max_angle_of_incidence_rad, target_distance_m);
        normals_vec.push_back(std::move(normals_pair.first));
        normals_vec.push_back(std::move(normals_pair.second));
    } else {
        normals_vec.push_back(ouster::sdk::algorithm::normals(
            first_xyz_map, first_range_map, sensor_origins_map, pixel_search_range,
            max_angle_of_incidence_rad, target_distance_m));
    }

    std::vector<py::ndarray<py::numpy, double, py::c_contig>> results;
    for (auto& it : normals_vec) {
        std::vector<size_t> shape = {static_cast<size_t>(h), static_cast<size_t>(w), 3};

        // avoid the copy by moving
        auto data = new ouster::sdk::core::MatrixX3dR(std::move(it));

        auto capsule = py::capsule(data, [](void* pointer) noexcept {
            delete static_cast<ouster::sdk::core::MatrixX3dR*>(pointer);
        });

        py::ndarray<py::numpy, double, py::c_contig> temp(data->data(), shape.size(), shape.data(),
                                                          capsule);
        results.push_back(temp);
    }

    return results;
}

std::vector<py::ndarray<py::numpy, double, py::c_contig>> process_normals_flexible(
    const py::ndarray<py::ro, py::shape<-1, -1, 3>>& xyz,
    const py::ndarray<py::ro, py::shape<-1, -1>>& range,
    const py::ndarray<py::ro, py::shape<-1, 3>>& sensor_origins, int pixel_search_range,
    double max_angle_of_incidence_rad, double target_distance_m,
    const nonstd::optional<py::ndarray<py::ro, py::shape<-1, -1, 3>>>& xyz2 = nonstd::nullopt,
    const nonstd::optional<py::ndarray<py::ro, py::shape<-1, -1>>>& range2 = nonstd::nullopt) {
    nonstd::optional<py::ndarray<const double, py::c_contig, py::shape<-1, -1, 3>>> opt_xyz2;
    nonstd::optional<py::ndarray<const uint32_t, py::c_contig, py::shape<-1, -1>>> opt_range2;
    if (xyz2.has_value()) {
        opt_xyz2.emplace(ensure_c_contig_floating<double, py::shape<-1, -1, 3>>(*xyz2));
    }
    if (range2.has_value()) {
        opt_range2.emplace(ensure_c_contig<uint32_t, py::shape<-1, -1>>(*range2));
    }
    return process_normals(ensure_c_contig_floating<double, py::shape<-1, -1, 3>>(xyz),
                           ensure_c_contig<uint32_t, py::shape<-1, -1>>(range),
                           ensure_c_contig_floating<double, py::shape<-1, 3>>(sensor_origins),
                           pixel_search_range, max_angle_of_incidence_rad, target_distance_m,
                           opt_xyz2, opt_range2);
}

}  // namespace

void init_algorithm_normals(py::module_& module) {
    module.def(
        "normals",
        [](const py::ndarray<py::ro, py::shape<-1, -1, 3>>& xyz,
           const py::ndarray<py::ro, py::shape<-1, -1>>& range,
           const py::ndarray<py::ro, py::shape<-1, 3>>& sensor_origins, int pixel_search_range,
           double min_angle_of_incidence_rad, double target_distance_m) {
            return process_normals_flexible(xyz, range, sensor_origins, pixel_search_range,
                                            min_angle_of_incidence_rad, target_distance_m)[0];
        },
        R"doc(
Compute normals from destaggered XYZ/range arrays.

Args:
    xyz: destaggered XYZ coordinates for the first return (H, W, 3)
    range: destaggered range image for the first return (H, W)
    sensor_origins_xyz: per-column sensor origins in the same frame as xyz/range,
        shape (W, 3). For world-frame xyz, use
        (frame.body_to_world @ frame.sensor_info.sensor_to_body)[:, :3, 3].
        For sensor-frame xyz, pass zeros with shape (W, 3) (e.g. np.zeros((w, 3))).
    pixel_search_range: non-negative axial search radius (in pixels) when gathering
        neighbours
    min_angle_of_incidence_rad: minimum allowable incidence angle between a beam and
        surface (radians) (default: 1 deg, ~0.01745 rad)
    target_distance_m: target neighbour distance used when selecting candidate points

Returns:
    A destaggered normal array of shape (H, W, 3) for the provided return.
)doc",
        py::arg("xyz"), py::arg("range"), py::arg("sensor_origins_xyz"),
        py::arg("pixel_search_range") = 1,
        py::arg("min_angle_of_incidence_rad") =
            ouster::sdk::algorithm::DEFAULT_MIN_ANGLE_INCIDENCE_RAD,
        py::arg("target_distance_m") = ouster::sdk::algorithm::DEFAULT_TARGET_DISTANCE_METER);

    module.def(
        "normals",
        [](const py::ndarray<py::ro, py::shape<-1, -1, 3>>& xyz,
           const py::ndarray<py::ro, py::shape<-1, -1>>& range,
           const py::ndarray<py::ro, py::shape<-1, -1, 3>>& xyz2,
           const py::ndarray<py::ro, py::shape<-1, -1>>& range2,
           const py::ndarray<py::ro, py::shape<-1, 3>>& sensor_origins, int pixel_search_range,
           double min_angle_of_incidence_rad, double target_distance_m) {
            auto result = process_normals_flexible(xyz, range, sensor_origins, pixel_search_range,
                                                   min_angle_of_incidence_rad, target_distance_m,
                                                   xyz2, range2);

            return std::make_tuple(result[0], result[1]);
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
        (frame.body_to_world @ frame.sensor_info.sensor_to_body)[:, :3, 3].
        For sensor-frame xyz, pass zeros.
    pixel_search_range: non-negative axial search radius (in pixels) when gathering
        neighbours
    min_angle_of_incidence_rad: minimum allowable incidence angle between a beam and
        surface (radians) (default: 1 deg, ~0.01745 rad)
    target_distance_m: target neighbour distance used when selecting candidate points

Returns:
    A tuple of destaggered normal arrays (first_return_normals, second_return_normals).
)doc",
        py::arg("xyz"), py::arg("range"), py::arg("xyz2"), py::arg("range2"),
        py::arg("sensor_origins_xyz"), py::arg("pixel_search_range") = 1,
        py::arg("min_angle_of_incidence_rad") =
            ouster::sdk::algorithm::DEFAULT_MIN_ANGLE_INCIDENCE_RAD,
        py::arg("target_distance_m") = ouster::sdk::algorithm::DEFAULT_TARGET_DISTANCE_METER);
}

void init_ground_seg(py::module_& module) {
    py::class_<GroundSegConfig>(module, "GroundSegConfig",
                                R"pbdoc(
        Configuration for the ground segmentation engine.
    )pbdoc")
        .def(py::init())
        .def_rw("grid_size", &GroundSegConfig::grid_size,
                R"pbdoc(
                Grid cell size in meters for the 2.5-D height map.
                Default: 0.5
            )pbdoc");

    py::class_<GroundSegEngine>(module, "GroundSegEngine",
                                R"pbdoc(
        Ground segmentation engine for classifying ground points.

        Coordinate frame selection:
            If any valid column pose in a frame is non-identity, the engine
            treats those poses as SLAM poses and segments in that global frame.
            Otherwise, it uses sensor extrinsics and frame-local column poses.

        The engine does not compute IMU gravity alignment internally. If
        gravity alignment is desired, plumb the frame set source first and write the
        resulting transform into sensor extrinsics before calling update().
    )pbdoc")
        .def_static(
            "create",
            [](const GroundSegConfig& config) -> std::unique_ptr<GroundSegEngine> {
                return GroundSegEngine::create(config);
            },
            py::arg("config") = GroundSegConfig{},
            R"pbdoc(
                Create a GroundSegEngine with the given configuration.

                Args:
                    config: GroundSegConfig with parameters (default: grid_size=0.5).

                Returns:
                    A GroundSegEngine instance.
            )pbdoc")
        .def(
            "update",
            [](GroundSegEngine& self, FrameSet& frame_set) -> FrameSet& {
                self.update(frame_set);
                return frame_set;
            },
            py::arg("frames"),
            R"pbdoc(
                Run ground segmentation on a FrameSet.

                Annotates each frame with a "GROUND" field (uint8: 1=ground,
                0=non-ground) for the first return, and a "GROUND2" field
                for the second return when RANGE2 is present. Returns the
                same FrameSet after in-place annotation.
            )pbdoc");
}

void init_algorithm(py::module_& module, py::module_& /*root_module*/) {
    module.doc() = "Python bindings for shared Ouster perception and mapping algorithms.";
    init_algorithm_normals(module);
    init_ground_seg(module);
    init_algorithm_align_clouds(module);
}

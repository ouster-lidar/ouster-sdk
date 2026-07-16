/*
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * Bindings for frame-to-map ICP registration and adaptive thresholding.
 */

#include <vector>

#include "_mapping.h"
#include "ouster/core/voxel_hash_map.h"
#include "ouster/mapping/adaptive_threshold.h"
#include "ouster/mapping/icp_registration.h"

using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::VoxelHashMap3d;
using ouster::sdk::core::VoxelHashMapXd;
using ouster::sdk::mapping::AdaptiveThreshold;
using ouster::sdk::mapping::ICPRegistration;

namespace {

std::vector<Eigen::Vector3d> ndarray_to_vector3d(const ArrayX3Input& points) {
    const size_t num_points = static_cast<size_t>(points.shape(0));
    std::vector<Eigen::Vector3d> out(num_points);
    auto view = points.view();
    for (size_t i = 0; i < num_points; ++i) {
        out[i] = Eigen::Vector3d(view(i, 0), view(i, 1), view(i, 2));
    }
    return out;
}

Matrix4dR ndarray_to_matrix4(const Mat4Input& matrix) {
    Matrix4dR out;
    auto view = matrix.view();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            out(row, col) = view(row, col);
        }
    }
    return out;
}

}  // namespace

void init_mapping_registration(py::module_& module) {
    py::class_<ICPRegistration>(module, "ICPRegistration", R"pbdoc(
        Iterative Closest Point (ICP) registration for aligning a point cloud to a voxel map.

        This class implements frame-to-map registration used by the LIO SLAM and localization
        pipelines. Given a LidarFrame (source points) and a pre-built
        :class:`ouster.sdk.core.VoxelHashMap3d` or :class:`ouster.sdk.core.VoxelHashMapXd` map,
        it iteratively estimates the SE(3) transform that best aligns the LidarFrame to the map
        using point-to-point correspondences and a Geman-McClure robust kernel.
    )pbdoc")
        .def(py::init<int, double, int>(), py::arg("max_num_iterations") = 50,
             py::arg("convergence_criterion") = 0.0001, py::arg("max_num_threads") = 0,
             R"pbdoc(
                 Create a ICPRegistration instance.

                 Args:
                     max_num_iterations (int): Maximum number of ICP iterations per call.
                         Defaults to 50.
                     convergence_criterion (float): Stop when the squared norm of the
                         incremental pose update falls below this threshold.
                         Defaults to 1e-4.
                     max_num_threads (int): Maximum worker threads. When 0 (default),
                         uses the TBB default concurrency.
             )pbdoc")
        .def_prop_rw(
            "max_num_iterations",
            [](const ICPRegistration& self) { return self.max_num_iterations_; },
            [](ICPRegistration& self, int value) { self.max_num_iterations_ = value; },
            R"pbdoc(
                Maximum number of ICP iterations performed by :meth:`align_points_to_map`.
            )pbdoc")
        .def_prop_rw(
            "convergence_criterion",
            [](const ICPRegistration& self) { return self.convergence_criterion_; },
            [](ICPRegistration& self, double value) { self.convergence_criterion_ = value; },
            R"pbdoc(
                Convergence threshold on the squared norm of the incremental pose update.
            )pbdoc")
        .def_prop_rw(
            "max_num_threads", [](const ICPRegistration& self) { return self.max_num_threads_; },
            [](ICPRegistration& self, int value) { self.max_num_threads_ = value; },
            R"pbdoc(
                Maximum number of TBB threads used during correspondence search and linear
                system assembly.
            )pbdoc")
        .def(
            "align_points_to_map",
            [](const ICPRegistration& self, const ArrayX3Input& frame,
               const VoxelHashMap3d& voxel_map, double max_distance, double kernel_scale) {
                return py::cast(self.align_points_to_map(ndarray_to_vector3d(frame), voxel_map,
                                                         max_distance, kernel_scale));
            },
            py::arg("frame"), py::arg("voxel_map"), py::arg("max_distance"),
            py::arg("kernel_scale"),
            R"pbdoc(
                Align a point cloud to a voxel hash map using ICP.

                The returned 4×4 homogeneous matrix transforms points from the input frame
                coordinate system into the map coordinate system.

                Args:
                    frame (numpy.ndarray): Source points as an ``(N, 3)`` array.
                    voxel_map (VoxelHashMap3d): Target map built from prior scans or a saved map.
                    max_distance (float): Maximum correspondence distance in meters. Points
                        without a map neighbor within this radius are ignored.
                    kernel_scale (float): Scale parameter for the Geman-McClure robust kernel
                        used to down-weight outlier correspondences.

                Returns:
                    numpy.ndarray: ``(4, 4)`` float64 transform matrix. Returns identity when
                    ``voxel_map`` is empty.
            )pbdoc")
        .def(
            "align_points_to_map",
            [](const ICPRegistration& self, const ArrayX3Input& frame,
               const VoxelHashMapXd& voxel_map, double max_distance, double kernel_scale) {
                return py::cast(self.align_points_to_map(ndarray_to_vector3d(frame), voxel_map,
                                                         max_distance, kernel_scale));
            },
            py::arg("frame"), py::arg("voxel_map"), py::arg("max_distance"),
            py::arg("kernel_scale"),
            R"pbdoc(
                Align a point cloud to a dynamic-size voxel hash map using ICP.

                This overload supports :class:`ouster.sdk.core.VoxelHashMapXd` maps that store
                per-point attributes alongside xyz coordinates. Correspondences use the xyz
                components of the closest map point returned by the voxel map lookup.

                Args:
                    frame (numpy.ndarray): Source points as an ``(N, 3)`` array.
                    voxel_map (VoxelHashMapXd): Target map with optional per-point attributes.
                    max_distance (float): Maximum correspondence distance in meters.
                    kernel_scale (float): Geman-McClure robust kernel scale parameter.

                Returns:
                    numpy.ndarray: ``(4, 4)`` float64 transform matrix. Returns identity when
                    ``voxel_map`` is empty.
            )pbdoc");

    py::class_<AdaptiveThreshold>(module, "AdaptiveThreshold", R"pbdoc(
        KISS-ICP adaptive correspondence threshold estimator.

        Tracks the deviation between predicted and registered motion to adapt the maximum
        correspondence distance used during frame-to-map ICP. Used internally by the LIO SLAM
        and localization backends.
    )pbdoc")
        .def(py::init<double, double, double>(), py::arg("max_range"),
             py::arg("initial_threshold") = 2.0, py::arg("min_motion_threshold") = 0.01,
             R"pbdoc(
                 Create an AdaptiveThreshold instance.

                 Args:
                     max_range (float): Sensor maximum range in meters, used to scale rotational
                         deviation into a distance-like error term.
                     initial_threshold (float): Initial correspondence threshold in meters.
                         Defaults to 2.0.
                     min_motion_threshold (float): Minimum motion magnitude in meters required
                         before a deviation sample is incorporated. Defaults to 0.01.
             )pbdoc")
        .def_prop_rw(
            "min_motion_threshold",
            [](const AdaptiveThreshold& self) { return self.min_motion_threshold_; },
            [](AdaptiveThreshold& self, double value) { self.min_motion_threshold_ = value; },
            R"pbdoc(
                Minimum motion magnitude in meters before a deviation sample is recorded.
            )pbdoc")
        .def_prop_rw(
            "max_range", [](const AdaptiveThreshold& self) { return self.max_range_; },
            [](AdaptiveThreshold& self, double value) { self.max_range_ = value; },
            R"pbdoc(
                Sensor maximum range in meters used when converting rotational error to distance.
            )pbdoc")
        .def(
            "update_model_deviation",
            [](AdaptiveThreshold& self, const Mat4Input& current_deviation) {
                self.update_model_deviation(ndarray_to_matrix4(current_deviation));
            },
            py::arg("current_deviation"),
            R"pbdoc(
                Incorporate a registration correction into the running deviation model.

                Args:
                    current_deviation (numpy.ndarray): ``(4, 4)`` homogeneous transform
                        representing the ICP correction for the current frame.
            )pbdoc")
        .def("compute_threshold", &AdaptiveThreshold::compute_threshold,
             R"pbdoc(
                 Return the current adaptive correspondence threshold in meters.

                 Returns:
                     float: Square root of the running mean squared deviation.
             )pbdoc");
}

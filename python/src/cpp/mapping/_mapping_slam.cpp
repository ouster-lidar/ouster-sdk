/*
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * Bindings for SlamEngine, LocalizationEngine, DeskewMethod variants, and
 * ActiveTimeCorrection.  Extracted from _mapping.cpp to allow parallel
 * compilation.
 */

#include <nanobind/stl/unique_ptr.h>
#include <nanobind/trampoline.h>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include "_mapping.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/mapping/active_time_correction.h"
#include "ouster/mapping/deskew_method.h"
#include "ouster/mapping/localization_engine.h"
#include "ouster/mapping/slam_engine.h"

using namespace ouster::sdk::mapping;
using namespace ouster::sdk::core;
namespace {

class PyDeskewMethod : public DeskewMethod {
   public:
    NB_TRAMPOLINE(ouster::sdk::mapping::DeskewMethod, 2);

    void update(FrameSet& lidar_frame_set) override {
        NB_OVERRIDE_PURE(update, lidar_frame_set);
    }

    void set_last_pose(int64_t timestamp, const Matrix4dR& pose) override {
        NB_OVERRIDE_PURE(set_last_pose, timestamp, pose);
    }
};

}  // namespace

void init_mapping_slam(py::module_& module) {
    py::class_<SlamConfig>(module, "SlamConfig", R"pbdoc(
        Configuration options for the SLAM engine.
    )pbdoc")
        .def_rw("min_range", &SlamConfig::min_range)
        .def_rw("max_range", &SlamConfig::max_range)
        .def_rw("voxel_size", &SlamConfig::voxel_size)
        .def_rw("max_iterations", &SlamConfig::max_iterations)
        .def_rw("initial_pose", &SlamConfig::initial_pose)
        .def_rw("deskew_method", &SlamConfig::deskew_method)
        .def_static(
            "create",
            [](const std::string& kind) {
                if (kind == "lio") {
                    return std::make_shared<LIOSlamConfig>();
                } else {
                    throw std::invalid_argument("Unsupported SlamConfig type: " + kind);
                }
            },
            py::arg("kind") = "lio",
            R"pbdoc(
            Factory method to create SlamConfig instances based on the specified type.
            Args:
                kind (str): The type of SlamConfig to create. Supported values are "lio".
                    Defaults to "lio".
            Raises:
                ValueError: If kind is not supported.
            Returns:
                SlamConfig: A new instance of the specified SlamConfig type.
         )pbdoc");

    py::class_<LIOSlamConfig, SlamConfig>(module, "LIOSlamConfig", R"pbdoc(
        Configuration options for the SLAM engine.

        This class extends SlamConfig with additional configuration options specific to the SLAM engine.
    )pbdoc")
        .def(py::init());

    py::class_<SlamEngine>(module, "SlamEngine", R"pbdoc(
        The SLAM engine for processing LiDAR frames and computing body_to_world transforms.
    )pbdoc")
        .def_static(
            "create",
            [](const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
               const LIOSlamConfig& config) -> std::unique_ptr<SlamEngine> {
                return SlamEngine::create(sensor_infos, config);
            },
            py::arg("sensor_infos"), py::arg("config") = LIOSlamConfig{},
            R"pbdoc(
                SlamEngine constructor.

                Args:
                    sensor_infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
                    config (LIOSlamConfig): Options for the SLAM engine. Defaults to
                        LIOSlamConfig with all default values.
             )pbdoc")
        .def_static(
            "create",
            [](const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
               const LIOSlamConfig& config) -> std::unique_ptr<SlamEngine> {
                return SlamEngine::create(sensor_infos, config);
            },
            py::arg("infos"), py::arg("config") = LIOSlamConfig{},
            R"pbdoc(
                SlamEngine constructor (backwards-compatible alias with ``infos`` arg name).

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
                    config (LIOSlamConfig): Options for the SLAM engine. Defaults to
                        LIOSlamConfig with all default values.
             )pbdoc")
        .def(
            "update",
            [](SlamEngine& self, FrameSet& frame_set) -> FrameSet& {
                self.update(frame_set);
                return frame_set;
            },
            py::arg("frames"),
            R"pbdoc(
                Update the pose (per_column_global_pose) variable in frame and return

                Args:
                    frame_set (FrameSet): List of frames to update with the latest pose.

                Returns:
                    FrameSet: The updated frames with per_column_global_pose set.
            )pbdoc")
        .def("get_point_cloud", &SlamEngine::get_point_cloud,
             R"pbdoc(
                Get the current point cloud from the SLAM engine.

                Returns:
                    Nx3: The point cloud generated by the SLAM engine.
            )pbdoc");

    py::class_<LocalizationConfig>(module, "LocalizationConfig",
                                   R"pbdoc(
        Configuration options for the Localization engine.
    )pbdoc")
        .def_rw("min_range", &LocalizationConfig::min_range)
        .def_rw("max_range", &LocalizationConfig::max_range)
        .def_rw("voxel_size", &LocalizationConfig::voxel_size)
        .def_rw("max_iterations", &LocalizationConfig::max_iterations)
        .def_rw("initial_pose", &LocalizationConfig::initial_pose)
        .def_rw("deskew_method", &LocalizationConfig::deskew_method)
        .def_static(
            "create",
            [](const std::string& kind) {
                if (kind == "lio") {
                    return std::make_shared<LIOLocalizationConfig>();
                } else {
                    throw std::invalid_argument("Unsupported LocalizationConfig type: " + kind);
                }
            },
            py::arg("kind") = "lio",
            R"pbdoc(
            Factory method to create LocalizationConfig instances based on the specified type.
            Args:
                kind (str): The type of LocalizationConfig to create. Supported values are "lio".
                    Defaults to "lio".
            Raises:
                ValueError: If kind is not supported.
            Returns:
                LocalizationConfig: A new instance of the specified LocalizationConfig type.
         )pbdoc");

    py::class_<LIOLocalizationConfig, LocalizationConfig>(module, "LIOLocalizationConfig", R"pbdoc(
        Configuration options for the ICP-based Localization engine.
    )pbdoc")
        .def(py::init<>(), "...");

    py::class_<LocalizationEngine>(module, "LocalizationEngine", R"pbdoc(
        The Localization engine for processing LiDAR frames and computing body_to_world
        transforms based on prebuilt pointcloud map.
    )pbdoc")
        .def_static(
            "create",
            [](const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
               const std::string& map_path,
               const LIOLocalizationConfig& config) -> std::unique_ptr<LocalizationEngine> {
                return LocalizationEngine::create(sensor_infos, map_path, config);
            },
            py::arg("sensor_infos"), py::arg("map_path"),
            py::arg("config") = LIOLocalizationConfig{},
            R"pbdoc(
                Instantiate a LocalizationEngine with a map file path and optional config.

                Args:
                    sensor_infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
                    map_path (str): Path to the point cloud map file.
                    config (LIOLocalizationConfig): Configuration options. Defaults to
                        LIOLocalizationConfig with all default values.
             )pbdoc")
        .def_static(
            "create",
            [](const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
               const Eigen::Ref<const PointCloudXYZf> map,
               const LIOLocalizationConfig& config) -> std::unique_ptr<LocalizationEngine> {
                return LocalizationEngine::create(sensor_infos, map, config);
            },
            py::arg("sensor_infos"), py::arg("map"), py::arg("config") = LIOLocalizationConfig{},
            R"pbdoc(
                Instantiate a LocalizationEngine with a preloaded point cloud map and optional config.

                Args:
                    sensor_infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
                    map (numpy.ndarray): Preloaded point cloud map, shape (N, 3).
                    config (LIOLocalizationConfig): Configuration options. Defaults to
                        LIOLocalizationConfig with all default values.
             )pbdoc")
        .def(
            "update",
            [](LocalizationEngine& self, FrameSet& frame_set) -> FrameSet& {
                self.update(frame_set);
                return frame_set;
            },
            py::arg("frames"),
            R"pbdoc(
                Update the body_to_world transform (per_column_global_pose) variable
                in frame and return

                Args:
                    frame_set (FrameSet): List of frames to update with the latest
                        body_to_world transform.

                Returns:
                    FrameSet: The updated frames with per_column_global_pose set
            )pbdoc");

    py::class_<DeskewMethod, PyDeskewMethod>(module, "DeskewMethod", R"pbdoc(
        Base class for all deskewing methods.

        This is the abstract base class for all deskewing methods. Use specific
        derived classes like ConstantVelocityDeskewMethod instead of using this
        class directly.
    )pbdoc")
        .def(py::init<const std::vector<std::shared_ptr<SensorInfo>>&>(), py::arg("infos"),
             R"pbdoc(
                ConstantVelocityDeskewMethod constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             )pbdoc")
        .def(
            "update",
            [](DeskewMethod& self, FrameSet& frame_set) -> FrameSet& {
                self.update(frame_set);
                return frame_set;
            },
            py::arg("frames"),
            R"pbdoc(
                Update the pose (per_column_global_pose) variable in frame.

                Args:
                    frame_set (FrameSet): a FrameSet.

                Returns:
                    FrameSet: The updated lidar frames with per_column_global_pose set
            )pbdoc");

    py::class_<ConstantVelocityDeskewMethod, DeskewMethod>(module, "ConstantVelocityDeskewMethod",
                                                           R"pbdoc(
        Deskew method that assumes constant velocity motion between poses.
    )pbdoc")
        .def(py::init<const std::vector<std::shared_ptr<SensorInfo>>&>(), py::arg("infos"),
             R"pbdoc(
                ConstantVelocityDeskewMethod constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             )pbdoc")
        .def(
            "set_last_pose",
            [](ConstantVelocityDeskewMethod& self, uint64_t timestamp,
               const py::ndarray<double, py::ndim<2>, py::c_contig>& pose_arr) {
                if (pose_arr.ndim() != 2 || pose_arr.shape(0) != 4 || pose_arr.shape(1) != 4) {
                    throw std::runtime_error(
                        "pose must be a (4,4) array representing a "
                        "transformation matrix");
                }
                auto buf = pose_arr.view();
                Matrix4dR pose;
                for (int row = 0; row < 4; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        pose(row, col) = buf(row, col);
                    }
                }
                self.set_last_pose(static_cast<int64_t>(timestamp), pose);
            },
            py::arg("ts"), py::arg("pose"),
            R"pbdoc(
                Set the current pose to use for deskewing.

                This method allows setting the current pose that will be used
                as a reference for deskewing incoming frames. The current pose
                should represent the sensor's pose at the time of the most recent
                frame.

                Args:
                    ts (int): The timestamp (nanoseconds) associated with the pose.
                    pose (numpy.ndarray): A 4x4 transformation matrix representing
                        the current pose.
            )pbdoc");

    py::class_<DeskewMethodFactory>(module, "DeskewMethodFactory", R"pbdoc(
            Factory class for creating DeskewMethod instances.
        )pbdoc")
        .def_static(
            "create",
            [&](const std::string& method_name,
                const std::vector<std::shared_ptr<SensorInfo>>& infos)
                -> std::shared_ptr<DeskewMethod> {
                return DeskewMethodFactory::create(method_name, infos);
            },
            py::arg("method_name"), py::arg("infos"),
            R"pbdoc(
                    Create a DeskewMethod instance based on the specified method name.

                    Args:
                        method_name (str): The name of the deskew method to create.
                        infos (List[SensorInfo]): List of sensor info objects for each
                            sensor in the system.

                    Returns:
                        DeskewMethod: A new instance of the specified deskew method.
                 )pbdoc");

    py::class_<ActiveTimeCorrection>(module, "ActiveTimeCorrection", R"pbdoc(
        Class for correcting timestamps of LiDAR frames based on active time correction.
    )pbdoc")
        .def(py::init<const std::vector<std::shared_ptr<SensorInfo>>&>(), py::arg("infos"),
             R"pbdoc(
                ActiveTimeCorrection constructor.
                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             )pbdoc")
        .def(
            "update",
            [](ActiveTimeCorrection& self, FrameSet& frame_set) -> FrameSet& {
                self.update(frame_set);
                return frame_set;
            },
            py::arg("frames"),
            R"pbdoc(
                Update the timestamps in the provided FrameSet using active time correction.
            )pbdoc")
        .def(
            "reset",
            [](ActiveTimeCorrection& self, FrameSet& frame_set) -> FrameSet& {
                self.reset(frame_set);
                return frame_set;
            },
            py::arg("frames"),
            R"pbdoc(
                Restore the timestamps in the provided FrameSet to their original values before active time correction was applied.
            )pbdoc");
}

/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * Note: the type annotations in `perception.pyi` need to be updated whenever
 * this file changes. See the mypy documentation for details.
 */
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <ouster/perception/detection_engine.h>

namespace py = nanobind;

using ouster::sdk::core::FrameSet;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::perception::ClassicDetectionConfig;
using ouster::sdk::perception::DetectionConfig;
using ouster::sdk::perception::DetectionEngine;

// NOLINTNEXTLINE(misc-use-internal-linkage)
void init_perception(py::module_& module, py::module_& /*unused*/) {
    py::class_<DetectionConfig>(module, "DetectionConfig")
        .def(py::init())
        .def_static("create",
                    [](const std::string& /*kind = "classic"*/,
                       const py::kwargs& /*kwargs*/) -> DetectionConfig {
                        /**
                         * The reason to introduce a non-implemented method here
                         * is to have nanobind generate the type stub for us
                         */
                        throw std::logic_error(
                            "Implementation is overriden in python. "
                            "Are you importing DetectionConfig correctly?");
                    })
        .def_rw("save_instance_id_fields", &DetectionConfig::save_instance_id_fields);

    py::class_<ClassicDetectionConfig, DetectionConfig>(module, "ClassicDetectionConfig")
        .def(py::init())
        .def_rw("cluster_filter_min_side_length",
                &ClassicDetectionConfig::cluster_filter_min_side_length)
        .def_rw("cluster_filter_min_vertical_size",
                &ClassicDetectionConfig::cluster_filter_min_vertical_size)
        .def_rw("cluster_filter_max_volume", &ClassicDetectionConfig::cluster_filter_max_volume)
        .def_rw("cluster_filter_max_side_length",
                &ClassicDetectionConfig::cluster_filter_max_side_length);

    py::class_<DetectionEngine>(module, "DetectionEngine")
        .def("update", [](DetectionEngine& self, LidarFrame& frame) { self.update(frame); })
        .def("update", [](DetectionEngine& self, FrameSet& frame_set) { self.update(frame_set); })
        .def_static(
            "create",
            [](const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
               const ClassicDetectionConfig& config = {}) -> std::unique_ptr<DetectionEngine> {
                return DetectionEngine::create(sensor_infos, config);
            },
            R"(
               Create a detection engine.

               Args:
                   sensor_infos: a list of SensorInfos
                   config: optional ClassicDetectionConfig; defaults to
                           ClassicDetectionConfig with all default field values
               )",
            py::arg("sensor_infos"), py::arg("config") = ClassicDetectionConfig{})
        .def_static(
            "create",
            [](const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
               const std::string& kind) -> std::unique_ptr<DetectionEngine> {
                return DetectionEngine::create(sensor_infos, kind);
            },
            R"(
               Create a detection engine by kind string.

               Args:
                   sensor_infos: a list of SensorInfos
                   kind: backend name, e.g. "classic"
               )",
            py::arg("sensor_infos"), py::arg("kind"));
}

/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief ouster_pyclient - CameraFrame / CameraInfo / CameraModel
 *
 * Note: the type annotations in `client.pyi` need to be updated whenever this
 * file changes. See the mypy documentation for details.
 */

#include "ouster/core/camera_frame.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstring>
#include <memory>
#include <nonstd/optional.hpp>
#include <sstream>
#include <string>
#include <utility>

#include "common.h"  // NOLINT(misc-include-cleaner): provides the `py = nanobind` alias
#include "ouster/core/camera_info.h"
#include "ouster/core/camera_model.h"
#include "ouster/core/frame.h"
#include "ouster/core/sensor_info.h"

using ouster::sdk::core::CameraDataFormat;
using ouster::sdk::core::CameraFrame;
using ouster::sdk::core::CameraInfo;
using ouster::sdk::core::CameraModel;
using ouster::sdk::core::Frame;
using ouster::sdk::core::SensorInfo;

// External linkage is intentional: declared in client/main.cpp and invoked
// from the module init there.
// NOLINTNEXTLINE(misc-use-internal-linkage)
void init_client_camera_frame(py::module_& module, py::module_& /*unused*/) {
    py::class_<CameraDataFormat>(module, "CameraDataFormat", R"(
    Data format information for a camera.
    )")
        .def_rw("pixels_per_column", &CameraDataFormat::pixels_per_column,
                "Number of pixels per column.")
        .def_rw("columns_per_frame", &CameraDataFormat::columns_per_frame,
                "Number of columns per frame.")
        .def_rw("fps", &CameraDataFormat::fps, "Frames per second.");

    py::class_<CameraModel>(module, "CameraModel", R"(
    Rectified pinhole intrinsics for a camera.

    There are deliberately no distortion coefficients: the SDK never
    undistorts, so a CameraModel asserts that its imagery is already
    rectified.
    )")
        .def(py::init<float, float, float, float>(), py::arg("fx"), py::arg("fy"), py::arg("cx"),
             py::arg("cy"),
             "Construct a CameraModel. Raises ValueError if fx or fy is not "
             "greater than zero.")
        .def_rw("fx", &CameraModel::fx, "Focal length in pixels (x).")
        .def_rw("fy", &CameraModel::fy, "Focal length in pixels (y).")
        .def_rw("cx", &CameraModel::cx, "Optical center in pixels (x).")
        .def_rw("cy", &CameraModel::cy, "Optical center in pixels (y).")
        .def(
            "__eq__",
            [](const CameraModel& self, const py::object& other) {
                if (!py::isinstance<CameraModel>(other)) {
                    return false;
                }
                return self == py::cast<const CameraModel&>(other);
            },
            py::arg("other").none())
        .def("__repr__", [](const CameraModel& self) {
            std::stringstream sstr;
            sstr << "<ouster.sdk.client._client.CameraModel fx=" << self.fx << " fy=" << self.fy
                 << " cx=" << self.cx << " cy=" << self.cy << ">";
            return sstr.str();
        });

    py::class_<CameraInfo, SensorInfo>(module, "CameraInfo", R"(
    Metadata for a camera sensor.

    The optional `model` holds the intrinsics used to project between pixel
    and camera coordinates. Without it the imagery can be visualized but not
    projected into a point cloud.
    )")
        // Constructed via py::new_ to match SensorInfo's factory __new__; see
        // the SensorInfo binding.
        .def(py::new_([]() { return std::make_shared<CameraInfo>(); }),
             "Default constructor creates an invalid CameraInfo.")
        .def(py::new_([](const std::string& json_string) {
                 return std::make_shared<CameraInfo>(json_string);
             }),
             py::arg("json_string"), "Construct a CameraInfo by parsing a JSON metadata string.")
        .def_rw("sensor_to_body", &CameraInfo::sensor_to_body,
                "Homogeneous transformation matrix from the camera frame to the "
                "body frame. Defaults to the identity.")
        .def_rw("format", &CameraInfo::format, "Data format for this camera.")
        .def_prop_rw(
            "model", [](const CameraInfo& self) { return self.model; },
            [](CameraInfo& self, const nonstd::optional<CameraModel>& model) {
                self.model = model;
            },
            py::arg().none(),
            // The getter hands back a temporary optional, so the property's
            // default reference_internal policy would alias freed memory.
            py::rv_policy::copy,
            "Rectified pinhole intrinsics, or None when the camera is "
            "uncalibrated. Without a model the imagery can be visualized "
            "but not projected into a point cloud.")
        .def_prop_ro(
            "fps", [](const CameraInfo& self) { return self.fps(); },
            "Rate at which this camera produces frames, in frames per second.")
        .def(
            "__eq__",
            [](const CameraInfo& self, const py::object& other) {
                if (!py::isinstance<CameraInfo>(other)) {
                    return false;
                }
                return self == py::cast<const CameraInfo&>(other);
            },
            py::arg("other").none())
        .def("__copy__", [](const CameraInfo& self) { return CameraInfo{self}; })
        .def("__deepcopy__",
             [](const CameraInfo& self, const py::dict&) { return CameraInfo{self}; })
        .def("__repr__", [](const CameraInfo& self) {
            std::stringstream sstr;
            sstr << "<ouster.sdk.client._client.CameraInfo " << self.w() << "x" << self.h()
                 << (self.model ? " calibrated" : " uncalibrated") << ">";
            return sstr.str();
        });

    py::class_<CameraFrame, Frame>(module, "CameraFrame", R"(
    Represents a single frame of camera data.

    Analogous to LidarFrame, CameraFrame is a container for the pixel/column
    fields captured by a single camera exposure. Camera frames share the
    common Frame base with LidarFrame and can be stored alongside lidar
    frames inside a FrameSet.
    )")
        .def(py::init<>(),
             "Default constructor creates an invalid 0 x 0 frame with a single "
             "timestamp and identity body-to-world pose.")
        .def(py::init<std::shared_ptr<CameraInfo>>(), py::arg("camera_info"),
             "Initialize a CameraFrame configured for the provided CameraInfo.")
        // Unlike LidarFrame (whose w/h are derived from sensor_info and thus
        // read-only), a CameraFrame has no constructor that sizes it, so w/h
        // are writable here, shadowing the read-only base Frame properties.
        .def_rw("w", &CameraFrame::w, "Width or horizontal resolution of the frame.")
        .def_rw("h", &CameraFrame::h, "Height or vertical resolution of the frame.")
        // Shadows the read-only base Frame.sensor_info with a writable property
        // whose setter accepts only a CameraInfo, mirroring the C++
        // TypedFrame<CameraInfo>::sensor_info() / set_sensor_info() pair.
        .def_prop_rw(
            "sensor_info", [](const CameraFrame& self) { return self.sensor_info(); },
            [](CameraFrame& self, std::shared_ptr<CameraInfo> info) {
                self.set_sensor_info(std::move(info));
            },
            py::arg().none(), "The CameraInfo associated with this camera frame, if any.")
        // __eq__ is inherited from the Frame base, which dispatches to the
        // virtual Frame::equals().
        .def("__copy__", [](const CameraFrame& self) { return CameraFrame{self}; })
        .def("__deepcopy__",
             [](const CameraFrame& self, const py::dict& /*unused*/) { return CameraFrame{self}; })
        .def("__repr__",
             [](const CameraFrame& self) {
                 std::stringstream sstr;
                 sstr << "<ouster.sdk.client._client.CameraFrame @"
                      << static_cast<const void*>(&self) << " " << self.w << "x" << self.h << ">";
                 return sstr.str();
             })
        .def("__str__", [](const CameraFrame& self) { return to_string(self); });
}

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

#include "ouster/core/object.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_map.h>
#include <nanobind/stl/string.h>

#include "ouster/core/object_util.h"

namespace py = nanobind;

using ouster::sdk::core::Object;
using ouster::sdk::core::pose_at_timestamp;

// NOLINTNEXTLINE(use-internal-linkage)  Tidy just doesn't get it.
void init_client_object(py::module_& module, py::module_& /*unused*/) {
    py::bind_map<std::unordered_map<std::string, std::string>>(module, "DictStrStr");

    py::class_<Object>(module, "Object", R"(
    Data holder class for perception annotations.
    )")
        .def(py::init())
        .def_rw("id", &Object::id,
                "Object ID.\n\n"
                "If tracked, objects maintain their IDs between frames.")
        .def_rw("creation_ts", &Object::creation_ts,
                "Timestamp of the first detection, in nanoseconds. Lidar time.")
        .def_rw("timestamp", &Object::timestamp,
                "Timestamp of the current detection, in nanoseconds. Lidar "
                "time.")
        .def_rw("class_id", &Object::class_id,
                "Object classification.\n\n"
                "DetectionEngine implementations determine the meaning behind "
                "class_id numbers; see ClassMap for corresponding class name "
                "strings.")
        .def_rw("class_confidence", &Object::class_confidence,
                "Classification confidence.\n\n"
                "Value between 0 and 1, where 1 is absolute confidence.")
        .def_rw("object_to_body", &Object::object_to_body,
                "Transform from the object frame to the body frame.")
        .def_rw("body_to_world", &Object::body_to_world,
                "Transform from the body frame to the world frame.")
        .def_rw("velocity", &Object::velocity, "Velocity vector (inside worldframe).")
        .def_rw("dimensions", &Object::dimensions, "Full extents of the object's bounding box.")
        .def_rw("properties", &Object::properties,
                "Properties dictionary.\n\n"
                "Allows to extend object information with arbitrary data.")
        .def("__eq__",
             [](const Object& left, const py::object& right) {
                 if (!py::isinstance<Object>(right)) {
                     return false;
                 }
                 return left == py::cast<Object>(right);
             })
        .def("__copy__", [](const Object& self) { return Object{self}; })
        .def("__deepcopy__", [](const Object& self, const py::object&) { return Object{self}; });

    module.def("pose_at_timestamp", &pose_at_timestamp, py::arg("frame"), py::arg("lidar_ts"),
               R"(
            Interpolate the body-to-world pose at a lidar timestamp.

            Args:
                frame: Lidar frame providing column timestamps, status, and poses
                lidar_ts: Lidar timestamp in nanoseconds

            Returns:
                Interpolated pose at the requested lidar timestamp

            Raises:
                ValueError: if the timestamp cannot be bracketed by valid columns
        )");
}

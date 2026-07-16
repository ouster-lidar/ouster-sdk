/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Python bindings for ouster::sdk::core::Pose
 */

#include "ouster/core/pose.h"

#include <stdexcept>
#include <string>

#include "common.h"

namespace py = nanobind;

using ouster::sdk::core::Matrix3dR;
using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::Pose;

namespace {

Eigen::Vector4d rotation_to_wxyz(const Eigen::Quaterniond& quat) {
    return {quat.w(), quat.x(), quat.y(), quat.z()};
}

void set_rotation_wxyz(Pose& pose, const Eigen::Vector4d& quat) {
    pose.set_rotation(Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)));
}

template <int N>
Eigen::Matrix<double, N, 1> vector_from_python(const py::object& obj) {
    try {
        return py::cast<Eigen::Matrix<double, N, 1>>(obj);
    } catch (const py::cast_error&) {
    }

    if (!py::isinstance<py::sequence>(obj)) {
        throw std::invalid_argument("expected a sequence or array");
    }
    const size_t size = py::len(obj);
    // clang-tidy hallucinates nonexistent C cast here
    // NOLINTBEGIN
    if (size != static_cast<size_t>(N)) {
        throw std::invalid_argument("expected a sequence of length " + std::to_string(N) +
                                    ", got " + std::to_string(size));
    }
    // NOLINTEND
    Eigen::Matrix<double, N, 1> out;
    for (int i = 0; i < N; ++i) {
        out(i) = py::cast<double>(obj[i]);
    }
    return out;
}

void set_position(Pose& pose, const py::object& position) {
    pose.set_position(vector_from_python<3>(position));
}

void set_rotation(Pose& pose, const py::object& rotation) {
    set_rotation_wxyz(pose, vector_from_python<4>(rotation));
}

}  // namespace

void init_client_pose(py::module_& module, py::module_& /*unused*/) {
    py::class_<Pose>(module, "Pose", R"(
    A rigid-body pose represented by translation and rotation.
    )")
        .def(py::init<>())
        .def(py::init<const Matrix4dR&>())
        .def(py::init<const Eigen::Vector3d&, const Eigen::Quaterniond&>(), py::arg("position"),
             py::arg("rotation"), "Construct from translation and a unit quaternion.")
        .def_prop_rw(
            "position", [](const Pose& pose) { return pose.position(); }, &set_position,
            "Translation vector (x, y, z) in meters. Accepts a length-3 array "
            "or sequence.")
        .def_prop_rw(
            "rotation", [](const Pose& pose) { return rotation_to_wxyz(pose.rotation()); },
            &set_rotation,
            "Orientation as a unit quaternion [w, x, y, z]. Accepts a length-4 "
            "array or sequence.")
        .def("to_matrix", &Pose::to_matrix,
             "Return a 4x4 row-major homogeneous transformation matrix.")
        .def("euler_angles", &Pose::euler_angles,
             "Return fixed-axis Euler angles [roll, pitch, yaw] in radians.")
        .def("set_rotation", py::overload_cast<const Matrix3dR&>(&Pose::set_rotation),
             py::arg("rotation_matrix"), "Set rotation from a 3x3 rotation matrix.")
        .def("set_rotation", py::overload_cast<const Eigen::Vector3d&>(&Pose::set_rotation),
             py::arg("euler_angles"),
             "Set rotation from fixed-axis Euler angles [roll, pitch, yaw] in "
             "radians.")
        .def("inverse", &Pose::inverse, "Return the inverse of this pose.")
        .def("__mul__", py::overload_cast<const Pose&>(&Pose::operator*, py::const_),
             py::arg("other"), "Compose this pose with another pose.")
        .def("__mul__", py::overload_cast<const Eigen::Vector3d&>(&Pose::operator*, py::const_),
             py::arg("point"), "Transform a 3D point by this pose.")
        .def("__eq__", [](const Pose& left, const py::object& right) {
            if (!py::isinstance<Pose>(right)) {
                return false;
            }
            return left == py::cast<Pose>(right);
        });
}

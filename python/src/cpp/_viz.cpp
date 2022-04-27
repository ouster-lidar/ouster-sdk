/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief pybind wrappers for the ouster simple viz library
 *
 * PoC for exposing the opengl visualizer in Python.
 */
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

#include <atomic>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/point_viz.h"
#include "ouster/types.h"

namespace py = pybind11;
using namespace ouster;

template <typename T, int F>
static void check_array(const py::array_t<T, F>& array, size_t size = 0,
                        size_t dims = 0, char storage = 'X') {
    if (size && static_cast<size_t>(array.size()) != size)
        throw std::invalid_argument("Expected array of size: " +
                                    std::to_string(size));

    if (dims && static_cast<size_t>(array.ndim()) != dims)
        throw std::invalid_argument("Expected an array of dimension: " +
                                    std::to_string(dims));

    if (storage == 'F' && !(array.flags() & py::array::f_style))
        throw std::invalid_argument("Expected a F_CONTIGUOUS array");

    if (storage == 'C' && !(array.flags() & py::array::c_style))
        throw std::invalid_argument("Expected a C_CONTIGUOUS array");
}

template <size_t N>
static void tuple_to_float_array(std::array<float, N>& dst,
                                 const py::tuple& tuple) {
    if (tuple.size() > N)
        throw std::invalid_argument("Expected a tuple of size <= " +
                                    std::to_string(N));
    try {
        for (size_t i = 0; i < tuple.size(); i++) {
            dst[i] = tuple[i].cast<float>();
        }
    } catch (const py::cast_error&) {
        throw py::type_error("Expected a tuple of floats");
    }
}

// pybind11 size type changed since v2.0
using pysize = decltype(py::array{}.size());

// matrices passed to eigen must be converted to col-major
using pymatrixd =
    py::array_t<double, py::array::f_style | py::array::forcecast>;

PYBIND11_PLUGIN(_viz) {
    py::module m("_viz", R"(
    LidarScanViz bindings generated by pybind11.

    This module is generated from the C++ code and not meant to be used directly.
    )");

    // turn off signatures in docstrings: mypy stubs provide better types
    py::options options;
    options.disable_function_signatures();

    py::class_<viz::PointViz>(m, "PointViz")
        .def(py::init<const std::string&, bool, int, int>(), py::arg("name"),
             py::arg("fix_aspect") = false, py::arg("window_width") = 800,
             py::arg("window_height") = 600)

        .def(
            "run",
            [](viz::PointViz& self) {
                // acquire gil every n frames to check for signals
                const int check_every = 10;
                self.running(true);
                self.visible(true);
                while (self.running()) {
                    if (PyErr_CheckSignals() != 0)
                        throw py::error_already_set();
                    py::gil_scoped_release release;
                    for (int i = 0; i < check_every; i++) self.run_once();
                }
                self.visible(false);
            },
            R"(
             Run the visualizer rendering loop.

             Must be called from the main thread. Will return when ``running(False)`` is
             called from another thread or when the visualizer window is closed.
        )")

        .def("running", py::overload_cast<>(&viz::PointViz::running),
             "Check if the rendering loop is running.")

        .def("running", py::overload_cast<bool>(&viz::PointViz::running),
             "Shut down the visualizer and break out of the rendering loop.")

        .def("update", &viz::PointViz::update,
             "Show updated data in the next rendered frame.")

        // misc
        .def(
            "push_key_handler",
            [](viz::PointViz& self,
               std::function<bool(const viz::WindowCtx&, int, int)> f) {
                // pybind11 doesn't seem to deal with the rvalue ref arg
                // pybind11 already handles acquiring the GIL in the callback
                self.push_key_handler(std::move(f));
            },
            "Add a callback for handling keyboard input.")

        // control scene
        .def_property_readonly("camera", &viz::PointViz::camera,
                               py::return_value_policy::reference_internal,
                               "Get a reference to the camera controls.")

        .def_property_readonly("target_display", &viz::PointViz::target_display,
                               py::return_value_policy::reference_internal,
                               "Get a reference to the target display.")

        .def("add",
             py::overload_cast<const std::shared_ptr<viz::Cloud>&>(
                 &viz::PointViz::add),
             R"(
             Add an object to the scene.

             Args:
                 obj: A cloud, label, image or cuboid.)")
        .def("add", py::overload_cast<const std::shared_ptr<viz::Cuboid>&>(
                        &viz::PointViz::add))
        .def("add", py::overload_cast<const std::shared_ptr<viz::Label>&>(
                        &viz::PointViz::add))
        .def("add", py::overload_cast<const std::shared_ptr<viz::Image>&>(
                        &viz::PointViz::add))
        .def("remove",
             py::overload_cast<const std::shared_ptr<viz::Cloud>&>(
                 &viz::PointViz::remove),
             R"(
             Remove an object from the scene.

             Args:
                 obj: A cloud, label, image or cuboid.

             Returns:
                 True if the object was in the scene and was removed.
             )")
        .def("remove", py::overload_cast<const std::shared_ptr<viz::Cuboid>&>(
                           &viz::PointViz::remove))
        .def("remove", py::overload_cast<const std::shared_ptr<viz::Label>&>(
                           &viz::PointViz::remove))
        .def("remove", py::overload_cast<const std::shared_ptr<viz::Image>&>(
                           &viz::PointViz::remove));

    m.def(
        "add_default_controls",
        [](viz::PointViz& viz) { viz::add_default_controls(viz); },
        "Add default keyboard and mouse bindings to a visualizer instance.");

    py::class_<viz::WindowCtx>(m, "WindowCtx")
        .def_readonly("lbutton_down", &viz::WindowCtx::lbutton_down)
        .def_readonly("mbutton_down", &viz::WindowCtx::mbutton_down)
        .def_readonly("mouse_x", &viz::WindowCtx::mouse_x)
        .def_readonly("mouse_y", &viz::WindowCtx::mouse_y)
        .def_readonly("viewport_width", &viz::WindowCtx::viewport_width)
        .def_readonly("viewport_height", &viz::WindowCtx::viewport_height);

    py::class_<viz::Camera>(m, "Camera")
        .def("reset", &viz::Camera::reset)
        .def("yaw", &viz::Camera::yaw)
        .def("pitch", &viz::Camera::pitch)
        .def("dolly", &viz::Camera::dolly)
        .def("dolly_xy", &viz::Camera::dolly_xy)
        .def("set_fov", &viz::Camera::set_fov)
        .def("set_orthographic", &viz::Camera::set_orthographic)
        .def("set_proj_offset", &viz::Camera::set_proj_offset);

    py::class_<viz::TargetDisplay>(m, "TargetDisplay")
        .def("enable_rings", &viz::TargetDisplay::enable_rings)
        .def("set_ring_size", &viz::TargetDisplay::set_ring_size);

    py::class_<viz::Cloud, std::shared_ptr<viz::Cloud>>(m, "Cloud")
        .def("__init__",
             [](viz::Cloud& self, size_t n) { new (&self) viz::Cloud{n}; })
        .def("__init__",
             [](viz::Cloud& self, const sensor::sensor_info& info) {
                 const auto xyz_lut = make_xyz_lut(info);

                 // make_xyz_lut still outputs doubles
                 Eigen::Array<float, Eigen::Dynamic, 3> direction =
                     xyz_lut.direction.cast<float>();
                 Eigen::Array<float, Eigen::Dynamic, 3> offset =
                     xyz_lut.direction.cast<float>();

                 viz::mat4d extrinsica;
                 std::copy(info.extrinsic.data(), info.extrinsic.data() + 16,
                           extrinsica.data());

                 new (&self)
                     viz::Cloud{info.format.columns_per_frame,
                                info.format.pixels_per_column, direction.data(),
                                offset.data(), extrinsica};
             })
        .def("set_range",
             [](viz::Cloud& self, py::array_t<uint32_t> range) {
                 check_array(range, self.get_size(), 2, 'C');
                 self.set_range(range.data());
             })
        .def("set_key",
             [](viz::Cloud& self, py::array_t<float> key) {
                 check_array(key, self.get_size(), 0, 'C');
                 self.set_key(key.data());
             })
        .def("set_mask",
             [](viz::Cloud& self, py::array_t<float> mask) {
                 check_array(mask, self.get_size() * 4, 3, 'C');
                 self.set_mask(mask.data());
             })
        .def("set_xyz",
             [](viz::Cloud& self, py::array_t<float> xyz) {
                 check_array(xyz, self.get_size() * 3, 0);
                 self.set_xyz(xyz.data());
             })
        .def("set_pose",
             [](viz::Cloud& self, pymatrixd pose) {
                 check_array(pose, 16, 2, 'F');
                 viz::mat4d posea;
                 std::copy(pose.data(), pose.data() + 16, posea.data());
                 self.set_pose(posea);
             })
        .def("set_point_size", &viz::Cloud::set_point_size)
        .def("set_palette", [](viz::Cloud& self, py::array_t<float> buf) {
            check_array(buf, 0, 2, 'C');
            if (buf.shape(1) != 3)
                throw std::invalid_argument("Expected a N x 3 array");
            self.set_palette(buf.data(), buf.shape(0));
        });

    py::class_<viz::Image, std::shared_ptr<viz::Image>>(m, "Image")
        .def(py::init<>())
        .def("set_image",
             [](viz::Image& self, py::array_t<float> image) {
                 check_array(image, 0, 2, 'C');
                 self.set_image(image.shape(1), image.shape(0), image.data());
             })
        .def("set_mask",
             [](viz::Image& self, py::array_t<float> buf) {
                 check_array(buf, 0, 3, 'C');
                 if (buf.shape(2) != 4)
                     throw std::invalid_argument("Expected a M x N x 4 array");
                 self.set_mask(buf.shape(1), buf.shape(0), buf.data());
             })
        .def("set_position", &viz::Image::set_position);

    py::class_<viz::Cuboid, std::shared_ptr<viz::Cuboid>>(m, "Cuboid")
        .def("__init__",
             [](viz::Cuboid& self, pymatrixd pose, py::tuple rgba) {
                 check_array(pose, 16, 2, 'F');
                 viz::mat4d posea;
                 std::copy(pose.data(), pose.data() + 16, posea.data());

                 viz::vec4f ar{0.0, 0.0, 0.0, 1.0};
                 tuple_to_float_array(ar, rgba);

                 new (&self) viz::Cuboid{posea, ar};
             })
        .def("set_transform",
             [](viz::Cuboid& self, pymatrixd pose) {
                 check_array(pose, 16, 2, 'F');
                 viz::mat4d posea;
                 std::copy(pose.data(), pose.data() + 16, posea.data());
                 self.set_transform(posea);
             })
        .def("set_rgba", [](viz::Cuboid& self, py::tuple rgba) {
            viz::vec4f ar{0.0, 0.0, 0.0, 1.0};
            tuple_to_float_array(ar, rgba);
            self.set_rgba(ar);
        });

    py::class_<viz::Label, std::shared_ptr<viz::Label>>(m, "Label")
        .def("__init__",
             [](viz::Label& self, const std::string& text, double x, double y,
                double z) {
                 new (&self) viz::Label{text, {x, y, z}};
             })
        .def(py::init<const std::string&, float, float, bool>(),
             py::arg("text"), py::arg("x"), py::arg("y"),
             py::arg("align_right") = false)
        .def("set_text", &viz::Label::set_text)
        .def("set_position",
             [](viz::Label& self, double x, double y, double z) {
                 self.set_position({x, y, z});
             })
        .def("set_position",
             py::overload_cast<float, float, bool>(&viz::Label::set_position),
             py::arg("x"), py::arg("y"), py::arg("align_right") = false)
        .def("set_scale", &viz::Label::set_scale);

    m.attr("spezia_palette") = py::array_t<float>{
        {static_cast<pysize>(viz::spezia_n), static_cast<pysize>(3)},
        &viz::spezia_palette[0][0]};
    m.attr("calref_palette") = py::array_t<float>{
        {static_cast<pysize>(viz::calref_n), static_cast<pysize>(3)},
        &viz::calref_palette[0][0]};

    m.attr("__version__") = VERSION_INFO;

    return m.ptr();
}

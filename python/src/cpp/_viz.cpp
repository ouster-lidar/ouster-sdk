/*
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief nanobind wrappers for the ouster simple viz library
 *
 * PoC for exposing the opengl visualizer in Python.
 */
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ndarray_helpers.h"
#include "ouster/core/impl/build.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"
#include "ouster/viz/impl/zone_monitor_voxel_mesh.h"
#include "ouster/viz/point_viz.h"

namespace py = nanobind;
namespace viz = ouster::sdk::viz;
using ouster::sdk::python::ensure_c_contig;
using ouster::sdk::python::ensure_c_contig_floating;
using ouster::sdk::viz::EventModifierKeys;
using ouster::sdk::viz::MouseButton;
using ouster::sdk::viz::MouseButtonEvent;
using ouster::sdk::viz::MouseEventType;
using ouster::sdk::viz::PointVizNotRunningError;
namespace {

// Helper to check array properties at runtime
template <typename ArrayType>
void check_array(const ArrayType& array, size_t size = 0, size_t dims = 0) {
    if (size && array.size() != size) {
        throw std::invalid_argument("Expected array of size: " + std::to_string(size) +
                                    ", but got: " + std::to_string(array.size()));
    }

    if (dims && array.ndim() != dims) {
        throw std::invalid_argument("Expected an array of dimension: " + std::to_string(dims) +
                                    ", but got: " + std::to_string(array.ndim()));
    }
}

template <size_t N>
void tuple_to_float_array(std::array<float, N>& dst, const py::tuple& tuple) {
    if (tuple.size() > N) {
        // NOLINTNEXTLINE(google-readability-casting) -- false positive on throw
        throw std::invalid_argument("Expected a tuple of size <= " + std::to_string(N));
    }
    try {
        for (size_t i = 0; i < tuple.size(); i++) {
            dst.at(i) = py::cast<float>(tuple[i]);
        }
    } catch (const py::cast_error&) {
        throw py::type_error("Expected a tuple of floats");
    }
}

// Thin wrapper of PointViz to keep track of some python references
class PyViz : public viz::PointViz {
   public:
    using viz::PointViz::PointViz;

    // lists of all cb handles for cycle tracking
    std::vector<py::handle> mb_cbs;
    std::vector<py::handle> key_cbs;
    std::vector<py::handle> fb_cbs, mp_cbs, scroll_cbs;
};

// Store Python callbacks outside the Cloud to avoid threading issues in their
// refcounting occuring in the viz as it copies them in a background thread
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
std::mutex select_cb_mutex;
uint64_t select_cb_id = 0;
std::map<uint64_t, std::function<void(py::object)>> select_cbs;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

class PyCloud : public ouster::sdk::viz::Cloud {
   public:
    using ouster::sdk::viz::Cloud::Cloud;

    py::handle select_cb;
    uint64_t select_id = 0;

    ~PyCloud() override {
        if (select_id > 0) {
            std::lock_guard<std::mutex> lock(select_cb_mutex);
            select_cbs.erase(select_id);
        }
    }
};

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class PyCuboid : public ouster::sdk::viz::Cuboid {
   public:
    using ouster::sdk::viz::Cuboid::Cuboid;

    py::handle select_cb;
    uint64_t select_id = 0;

    PyCuboid copy_without_select() const {
        return PyCuboid{get_transform(), get_rgba()};
    }

    ~PyCuboid() override {
        if (select_id > 0) {
            std::lock_guard<std::mutex> lock(select_cb_mutex);
            select_cbs.erase(select_id);
        }
    }
};

template <typename T>
void cloud_set_key_impl(PyCloud& self, const py::ndarray<const T, py::c_contig>& key) {
    if (key.size() == self.get_size()) {
        if (key.ndim() == 1 || key.ndim() == 2 || key.ndim() == 3) {
            self.set_key(key.data());
            return;
        }
    }
    if (key.ndim() == 2) {
        if (key.shape(1) == 3) {
            check_array(key, self.get_size() * 3);
            self.set_key_rgb(key.data());
            return;
        }
        if (key.shape(1) == 4) {
            check_array(key, self.get_size() * 4);
            self.set_key_rgba(key.data());
            return;
        }
        throw std::invalid_argument("Expected array with size of 2nd dimension: 1, 3 or 4");
    }
    check_array(key, 0, 3);
    if (key.shape(2) == 3) {
        check_array(key, self.get_size() * 3);
        self.set_key_rgb(key.data());
        return;
    }
    if (key.shape(2) == 4) {
        check_array(key, self.get_size() * 4);
        self.set_key_rgba(key.data());
        return;
    }
    throw std::invalid_argument("Expected array with size of 3rd dimension: 1, 3 or 4");
}

template <typename T>
void cloud_set_mask_impl(PyCloud& self, const py::ndarray<const T, py::c_contig>& mask) {
    check_array(mask, self.get_size() * 4, 0);
    if (mask.ndim() != 2 && mask.ndim() != 3) {
        throw std::invalid_argument("Expected an array of dimensions: 2 or 3");
    }
    self.set_mask(mask.data());
}

void cloud_set_xyz_impl(PyCloud& self, const py::ndarray<const float, py::c_contig>& xyz) {
    if (xyz.ndim() == 2) {
        if (xyz.shape(1) != 3) {
            throw std::invalid_argument("Expected array with size of 2nd dimension: 3");
        }
        check_array(xyz, self.get_size() * 3, 2);
        self.set_xyz(xyz.data());
        return;
    }
    if (xyz.ndim() == 3) {
        check_array(xyz, self.get_size() * 3, 3);
        self.set_xyz(xyz.data());
        return;
    }
    check_array(xyz, self.get_size() * 3, 1);
    self.set_xyz(xyz.data());
}

template <typename T>
void image_set_image_impl(viz::Image& self, const py::ndarray<const T, py::c_contig>& image) {
    if (image.ndim() == 2 || (image.ndim() == 3 && image.shape(2) == 1)) {
        self.set_image(image.shape(1), image.shape(0), image.data());
        return;
    }
    check_array(image, 0, 3);
    if (image.shape(2) == 3) {
        self.set_image_rgb(image.shape(1), image.shape(0), image.data());
        return;
    }
    if (image.shape(2) == 4) {
        self.set_image_rgba(image.shape(1), image.shape(0), image.data());
        return;
    }
    throw std::invalid_argument("Expected array with size of 3rd dimension: 1, 3 or 4");
}

void lines_set_points_impl(viz::Lines& self, const py::ndarray<const float, py::c_contig>& points) {
    if (points.ndim() == 2) {
        if (points.shape(1) != 3) {
            throw std::invalid_argument("Expected a Nx3 or Nx1 array.");
        }
        self.set_points(points.shape(0), points.data());
        return;
    }
    if (points.ndim() != 1) {
        throw std::invalid_argument("Expected a Nx3 or Nx1 array.");
    }
    self.set_points(points.shape(0) / 3, points.data());
}

int wrapper_tp_traverse(PyObject* self, visitproc visit, void* arg) {
    // We must traverse the implicit dependency of an object on its
    // associated type object.
    Py_VISIT(Py_TYPE(self));

    // The tp_traverse method may be called after __new__ but before or during
    // __init__, before the C++ constructor has been completed. We must not
    // inspect the C++ state if the constructor has not yet completed.
    if (!py::inst_ready(self)) {
        return 0;
    }

    // Get the C++ object associated with 'self' (this always succeeds)
    auto w = py::inst_ptr<PyViz>(self);

    // Visit all callbacks which can hold references to things (including
    // ourselves)
    for (const auto& handle : w->mb_cbs) {
        Py_VISIT(handle.ptr());
    }
    for (const auto& handle : w->key_cbs) {
        Py_VISIT(handle.ptr());
    }
    for (const auto& handle : w->fb_cbs) {
        Py_VISIT(handle.ptr());
    }
    for (const auto& handle : w->mp_cbs) {
        Py_VISIT(handle.ptr());
    }
    for (const auto& handle : w->scroll_cbs) {
        Py_VISIT(handle.ptr());
    }

    // todo should visit clouds but run into issues with them not being
    // all python objects

    return 0;
}

int wrapper_tp_clear(PyObject* self) {
    // Get the C++ object associated with 'self' (this always succeeds)
    auto w = py::inst_ptr<PyViz>(self);

    // clear all callbacks to break any reference cycles
    for (size_t i = 0; i < w->mb_cbs.size(); i++) {
        w->pop_mouse_button_handler();
    }
    w->mb_cbs.clear();

    for (size_t i = 0; i < w->key_cbs.size(); i++) {
        w->pop_key_handler();
    }
    w->key_cbs.clear();

    for (size_t i = 0; i < w->fb_cbs.size(); i++) {
        w->pop_frame_buffer_resize_handler();
    }
    w->fb_cbs.clear();

    for (size_t i = 0; i < w->mp_cbs.size(); i++) {
        w->pop_mouse_pos_handler();
    }
    w->mp_cbs.clear();

    for (size_t i = 0; i < w->scroll_cbs.size(); i++) {
        w->pop_scroll_handler();
    }
    w->scroll_cbs.clear();

    // todo what do I clear about clouds? we copy them but they can hold
    // callbacks which makes things awkward

    return 0;
}

// Table of custom type slots we want to install
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
PyType_Slot wrapper_slots[] = {{Py_tp_traverse, reinterpret_cast<void*>(wrapper_tp_traverse)},
                               {Py_tp_clear, reinterpret_cast<void*>(wrapper_tp_clear)},
                               {0, nullptr}};

int cloud_tp_traverse(PyObject* self, visitproc visit, void* arg) {
    // We must traverse the implicit dependency of an object on its
    // associated type object.
    Py_VISIT(Py_TYPE(self));

    // The tp_traverse method may be called after __new__ but before or during
    // __init__, before the C++ constructor has been completed. We must not
    // inspect the C++ state if the constructor has not yet completed.
    if (!py::inst_ready(self)) {
        return 0;
    }

    // Get the C++ object associated with 'self' (this always succeeds)
    auto w = py::inst_ptr<PyCloud>(self);

    // Visit all callbacks which can hold references to things (including
    // ourselves)
    Py_VISIT(w->select_cb.ptr());

    return 0;
}

int cloud_tp_clear(PyObject* self) {
    // Get the C++ object associated with 'self' (this always succeeds)
    auto w = py::inst_ptr<PyCloud>(self);

    w->clear_on_select();
    w->select_cb = {};

    if (w->select_id > 0) {
        select_cb_mutex.lock();
        select_cbs.erase(w->select_id);
        select_cb_mutex.unlock();
        w->select_id = 0;
    }

    return 0;
}

// Table of custom type slots we want to install
const PyType_Slot CLOUD_SLOTS[] = {{Py_tp_traverse, reinterpret_cast<void*>(cloud_tp_traverse)},
                                   {Py_tp_clear, reinterpret_cast<void*>(cloud_tp_clear)},
                                   {0, nullptr}};

int cuboid_tp_traverse(PyObject* self, visitproc visit, void* arg) {
    // We must traverse the implicit dependency of an object on its
    // associated type object.
    Py_VISIT(Py_TYPE(self));

    // The tp_traverse method may be called after __new__ but before or during
    // __init__, before the C++ constructor has been completed. We must not
    // inspect the C++ state if the constructor has not yet completed.
    if (!py::inst_ready(self)) {
        return 0;
    }

    // Get the C++ object associated with 'self' (this always succeeds)
    auto w = py::inst_ptr<PyCuboid>(self);

    // Visit all callbacks which can hold references to things (including
    // ourselves)
    Py_VISIT(w->select_cb.ptr());

    return 0;
}

int cuboid_tp_clear(PyObject* self) {
    // Get the C++ object associated with 'self' (this always succeeds)
    auto w = py::inst_ptr<PyCuboid>(self);

    w->clear_on_select();
    w->select_cb = {};

    if (w->select_id > 0) {
        select_cb_mutex.lock();
        select_cbs.erase(static_cast<int>(w->select_id));
        select_cb_mutex.unlock();
        w->select_id = 0;
    }

    return 0;
}

// Table of custom type slots we want to install
const PyType_Slot CUBOID_SLOTS[] = {{Py_tp_traverse, reinterpret_cast<void*>(cuboid_tp_traverse)},
                                    {Py_tp_clear, reinterpret_cast<void*>(cuboid_tp_clear)},
                                    {0, nullptr}};

// Signal handler for SIGINT (Ctrl+C)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::atomic<bool> signaled(false);
void handle_sigint(int signal) {
    if (signal == SIGINT) {
        PyErr_SetInterrupt();
        signaled = true;
    }
}
}  // namespace

using pysize = size_t;

// Matrix inputs are expected to be row-major (C-style), matching mat4d.
using pymatrixd = py::ndarray<const double, py::c_contig, py::ndim<2>, py::device::cpu>;

void init_viz(py::module_& module, py::module_& /*unused*/) {
    module.doc() = R"(
    PointViz bindings generated by nanobind.

    This module is generated from the C++ code and not meant to be used directly.
    )";

    py::enum_<viz::MouseButton>(module, "MouseButton", py::is_arithmetic())
        .value("MOUSE_BUTTON_1", MouseButton::MOUSE_BUTTON_1)
        .value("MOUSE_BUTTON_2", MouseButton::MOUSE_BUTTON_2)
        .value("MOUSE_BUTTON_3", MouseButton::MOUSE_BUTTON_3)
        .value("MOUSE_BUTTON_4", MouseButton::MOUSE_BUTTON_4)
        .value("MOUSE_BUTTON_5", MouseButton::MOUSE_BUTTON_5)
        .value("MOUSE_BUTTON_6", MouseButton::MOUSE_BUTTON_6)
        .value("MOUSE_BUTTON_7", MouseButton::MOUSE_BUTTON_7)
        .value("MOUSE_BUTTON_8", MouseButton::MOUSE_BUTTON_8)
        .value("MOUSE_BUTTON_LAST", MouseButton::MOUSE_BUTTON_LAST)
        .value("MOUSE_BUTTON_LEFT", MouseButton::MOUSE_BUTTON_LEFT)
        .value("MOUSE_BUTTON_RIGHT", MouseButton::MOUSE_BUTTON_RIGHT)
        .value("MOUSE_BUTTON_MIDDLE", MouseButton::MOUSE_BUTTON_MIDDLE)
        .export_values();

    py::enum_<viz::MouseButtonEvent>(module, "MouseButtonEvent", py::is_arithmetic())
        .value("MOUSE_BUTTON_RELEASED", MouseButtonEvent::MOUSE_BUTTON_RELEASED)
        .value("MOUSE_BUTTON_PRESSED", MouseButtonEvent::MOUSE_BUTTON_PRESSED)
        .export_values();

    py::enum_<viz::MouseEventType>(module, "MouseEventType", py::is_arithmetic())
        .value("LEFT", MouseEventType::LEFT)
        .value("RIGHT", MouseEventType::RIGHT)
        .value("MIDDLE", MouseEventType::MIDDLE)
        .value("ANY", MouseEventType::ANY)
        .export_values();

    py::enum_<viz::EventModifierKeys>(module, "EventModifierKeys", py::is_arithmetic(),
                                      py::is_flag())
        .value("MOD_NONE", EventModifierKeys::MOD_NONE)
        .value("MOD_SHIFT", EventModifierKeys::MOD_SHIFT)
        .value("MOD_CONTROL", EventModifierKeys::MOD_CONTROL)
        .value("MOD_ALT", EventModifierKeys::MOD_ALT)
        .value("MOD_SUPER", EventModifierKeys::MOD_SUPER)
        .value("MOD_CAPS_LOCK", EventModifierKeys::MOD_CAPS_LOCK)
        .value("MOD_NUM_LOCK", EventModifierKeys::MOD_NUM_LOCK)
        .export_values();

    // clang-tidy produces spurious error here
    // NOLINTBEGIN
    py::exception<PointVizNotRunningError>(module, "PointVizNotRunningError", PyExc_RuntimeError);
    // NOLINTEND

    py::class_<PyViz>(module, "PointViz", py::is_weak_referenceable(),
                      py::type_slots(wrapper_slots))
        .def(py::init<const std::string&, bool, int, int, bool, bool, bool>(), py::arg("name"),
             py::arg("fix_aspect") = false, py::arg("window_width") = 800,
             py::arg("window_height") = 600, py::arg("maximized") = false,
             py::arg("fullscreen") = false, py::arg("borderless") = false)
        .def(
            "run",
            [](PyViz& self) {
                // install a replacement signal handler to detect SIGINT and forward to python
                auto previous_handler = std::signal(SIGINT, handle_sigint);
                if (previous_handler == SIG_ERR) {
                    throw std::runtime_error("Failed to set signal handler for viz.");
                }
                self.running(true);
                self.visible(true);
                py::gil_scoped_release release;
                while (self.running()) {
                    if (signaled.exchange(false)) {
                        // only acquire the gil if we were signaled about SIGINT
                        py::gil_scoped_acquire acquire;
                        if (PyErr_CheckSignals() != 0) {
                            // restore old sigint handler
                            if (std::signal(SIGINT, previous_handler) == SIG_ERR) {
                                throw std::runtime_error("Failed to restore signal handler.");
                            }
                            throw py::python_error();
                        }
                    }
                    self.run_once();
                }
                self.visible(false);
                // restore old sigint handler
                if (std::signal(SIGINT, previous_handler) == SIG_ERR) {
                    throw std::runtime_error("Failed to restore signal handler.");
                }
            },
            R"(
             Run the visualizer rendering loop.

             Must be called from the main thread. Will return when ``running(False)`` is
             called from another thread or when the visualizer window is closed.
        )")
        .def("run_once", &PyViz::run_once,
             "Run one iteration of the main loop for rendering and input "
             "handling.")
        .def(
            "running", [](PyViz& self) { return self.running(); },
            "Check if the rendering loop is running.")
        .def(
            "running", [](PyViz& self, bool state) { self.running(state); },
            "Shut down the visualizer and break out of the rendering loop.")
        .def("update", &PyViz::update, "Show updated data in the next rendered frame.")
        .def(
            "pick",
            [](PyViz& self, int x_1, int y_1, int x_2, int y_2) {
                py::gil_scoped_release release;
                self.pick(x_1, y_1, x_2, y_2);
            },
            "Pick viz elements in the specified area.")
        .def("visible", &PyViz::visible, "Toggle if the PointViz window is visible")
        .def("cursor_visible", &PyViz::cursor_visible,
             "Toggle if the cursor is visible when over this window")
        .def(
            "push_key_handler",
            [](PyViz& self, const py::object& handler) {
                auto cb = py::cast<std::function<bool(const viz::WindowCtx&, int, int)>>(handler);
                self.key_cbs.push_back(handler);
                self.push_key_handler(std::move(cb));
            },
            py::sig("def push_key_handler(self, handler: "
                    "typing.Callable[[WindowCtx, int, int], bool]) -> None"),
            "Add a callback for handling keyboard input.")
        .def(
            "pop_key_handler",
            [](PyViz& self) {
                self.pop_key_handler();
                self.key_cbs.pop_back();
            },
            "Remove the last added callback for handling keyboard input.")
        .def(
            "get_screenshot",
            [](PyViz& self, size_t width, size_t height) {
                py::gil_scoped_release release;
                auto pixels = self.get_screenshot(width, height);
                {
                    py::gil_scoped_acquire acquire;
                    size_t shape[3] = {height, width, 3};
                    uint8_t* data = new uint8_t[height * width * 3];
                    py::capsule owner(
                        data, [](void* ptr) noexcept { delete[] static_cast<uint8_t*>(ptr); });
                    py::ndarray<uint8_t, py::numpy, py::c_contig> result(data, 3, shape, owner);
                    std::memcpy(result.data(), pixels.data(), pixels.size());
                    return result;
                }
            },
            py::arg("width"), py::arg("height"),
            "Gets a screenshot with an explicit width and height. Returns the "
            "pixels.")
        .def(
            "get_screenshot",
            [](PyViz& self, double scale_factor) {
                py::gil_scoped_release release;
                auto pixels = self.get_screenshot(scale_factor);
                auto size = self.get_scaled_viewport_size(scale_factor);
                {
                    py::gil_scoped_acquire acquire;
                    size_t shape[3] = {static_cast<size_t>(size.second),
                                       static_cast<size_t>(size.first), 3};
                    uint8_t* data = new uint8_t[static_cast<size_t>(size.second) * size.first * 3];
                    py::capsule owner(
                        data, [](void* ptr) noexcept { delete[] static_cast<uint8_t*>(ptr); });
                    py::ndarray<uint8_t, py::numpy, py::c_contig> result(data, 3, shape, owner);
                    std::memcpy(result.data(), pixels.data(), pixels.size());
                    return result;
                }
            },
            py::arg("scale_factor") = 1.0,
            "Gets a screenshot using a scale factor, which is a multiplier "
            "over the window width and height. Returns the "
            "pixels.")
        .def(
            "save_screenshot",
            [](PyViz& self, const std::string& path, double scale_factor) {
                py::gil_scoped_release release;
                return self.save_screenshot(path, scale_factor);
            },
            py::arg("path"), py::arg("scale_factor") = 1.0,
            "Saves a screenshot using a scale factor, which is a multiplier "
            "over the window width and height. Returns the resulting file "
            "name.")
        .def(
            "save_screenshot",
            [](PyViz& self, const std::string& path, int width, int height) {
                py::gil_scoped_release release;
                return self.save_screenshot(path, width, height);
            },
            py::arg("path"), py::arg("width"), py::arg("height"),
            "Saves a screenshot with an explicit width and height. Returns the "
            "resulting file name.")
        .def(
            "toggle_screen_recording",
            [](PyViz& self, double scale_factor) {
                return self.toggle_screen_recording(scale_factor);
            },
            py::arg("scale_factor") = 1.0,
            "Toggle screen recording. Returns true if started, false if "
            "stopped")
        .def(
            "toggle_screen_recording",
            [](PyViz& self, int width, int height) {
                return self.toggle_screen_recording(width, height);
            },
            py::arg("width"), py::arg("height"),
            "Toggle screen recording with explicit width and height. "
            "Returns true if started, false if stopped.")
        .def(
            "push_mouse_button_handler",
            [](PyViz& self, const py::object& handler, MouseEventType mode) {
                auto cb = py::cast<
                    std::function<bool(const viz::WindowCtx& context, viz::MouseButton button,
                                       viz::MouseButtonEvent, viz::EventModifierKeys mods)>>(
                    handler);
                self.mb_cbs.push_back(handler);
                self.push_mouse_button_handler(std::move(cb), mode);
            },
            py::arg("handler"), py::arg("mode") = MouseEventType::ANY,
            py::sig("def push_mouse_button_handler(self, handler: "
                    "typing.Callable[[WindowCtx, MouseButton, "
                    "MouseButtonEvent, EventModifierKeys], bool], mode: MouseEventType = "
                    "ouster.sdk._bindings.viz.MouseEventType.ANY) -> None"),
            "Add a callback for handling mouse button events.")
        .def(
            "pop_mouse_button_handler",
            [](PyViz& self) {
                self.pop_mouse_button_handler();
                self.mb_cbs.pop_back();
            },
            "Remove the last added callback for mouse button events.")
        .def(
            "push_scroll_handler",
            [](PyViz& self, const py::object& handler) {
                auto cb = py::cast<
                    std::function<bool(const viz::WindowCtx& context, double x, double y)>>(
                    handler);
                self.scroll_cbs.push_back(handler);
                self.push_scroll_handler(std::move(cb));
            },
            py::sig("def push_scroll_handler(self, handler: "
                    "typing.Callable[[WindowCtx, float, float], bool]) -> None"),
            "Add a callback for handling mouse scroll events.")
        .def(
            "pop_scroll_handler",
            [](PyViz& self) {
                self.pop_scroll_handler();
                self.scroll_cbs.pop_back();
            },
            "Remove the last added callback for mouse scroll events.")
        .def(
            "push_mouse_pos_handler",
            [](PyViz& self, const py::object& handler, MouseEventType mode) {
                auto cb = py::cast<
                    std::function<bool(const viz::WindowCtx& context, double x, double y)>>(
                    handler);
                self.mp_cbs.push_back(handler);
                self.push_mouse_pos_handler(std::move(cb), mode);
            },
            py::arg("handler"), py::arg("mode") = MouseEventType::ANY,
            py::sig("def push_mouse_pos_handler(self, handler: "
                    "typing.Callable[[WindowCtx, float, float], bool], mode: MouseEventType = "
                    "ouster.sdk._bindings.viz.MouseEventType.ANY) -> None"),
            "Add a callback for handling mouse position events.")
        .def(
            "pop_mouse_pos_handler", [](PyViz& self) { self.pop_mouse_pos_handler(); },
            "Remove the last added callback for mouse position events.")
        .def(
            "push_frame_buffer_resize_handler",
            [](PyViz& self, const py::object& handler) {
                auto cb = py::cast<std::function<bool(const viz::WindowCtx& context)>>(handler);
                self.fb_cbs.push_back(handler);
                self.push_frame_buffer_resize_handler(std::move(cb));
            },
            py::sig("def push_frame_buffer_resize_handler(self, handler: "
                    "typing.Callable[[WindowCtx], bool]) -> None"),
            "Add a callback for handling window resize events.")
        .def(
            "pop_frame_buffer_resize_handler",
            [](PyViz& self) {
                self.fb_cbs.pop_back();
                self.pop_frame_buffer_resize_handler();
            },
            "Remove the last added callback for window resize events.")
        .def_prop_ro("camera", &PyViz::camera, py::rv_policy::reference_internal,
                     "Get a reference to the camera controls.")
        .def_prop_ro("target_display", &PyViz::target_display, py::rv_policy::reference_internal,
                     "Get a reference to the target display.")
        .def_prop_ro("viewport_width", &PyViz::viewport_width, "Current viewport width in pixels")
        .def_prop_ro("viewport_height", &PyViz::viewport_height,
                     "Current viewport height in pixels")
        .def_prop_ro("window_width", &PyViz::window_width,
                     "Current window width in screen coordinates")
        .def_prop_ro("window_height", &PyViz::window_height,
                     "Current window height in screen coordinates")
        .def(
            "add", [](PyViz& self, const std::shared_ptr<PyCloud>& obj) { self.add(obj); },
            R"(
             Add an object to the scene.

             Args:
                 obj: A cloud, label, image or cuboid.)")
        .def("add", [](PyViz& self, const std::shared_ptr<viz::Mesh>& obj) { self.add(obj); })
        .def("add", [](PyViz& self, const std::shared_ptr<PyCuboid>& obj) { self.add(obj); })
        .def("add", [](PyViz& self, const std::shared_ptr<viz::Label>& obj) { self.add(obj); })
        .def("add", [](PyViz& self, const std::shared_ptr<viz::Image>& obj) { self.add(obj); })
        .def("add", [](PyViz& self, const std::shared_ptr<viz::Lines>& obj) { self.add(obj); })
        .def("add",
             [](PyViz& self, const std::shared_ptr<viz::ObjectOverlay>& obj) { self.add(obj); })
        .def(
            "remove",
            [](PyViz& self, const std::shared_ptr<PyCloud>& obj) { return self.remove(obj); },
            R"(
             Remove an object from the scene.

             Args:
                 obj: A cloud, label, image or cuboid.

             Returns:
                 True if the object was in the scene and was removed.
             )")
        .def("remove",
             [](PyViz& self, const std::shared_ptr<viz::Mesh>& obj) { return self.remove(obj); })
        .def("remove",
             [](PyViz& self, const std::shared_ptr<PyCuboid>& obj) { return self.remove(obj); })
        .def("remove",
             [](PyViz& self, const std::shared_ptr<viz::Label>& obj) { return self.remove(obj); })
        .def("remove",
             [](PyViz& self, const std::shared_ptr<viz::Image>& obj) { return self.remove(obj); })
        .def("remove",
             [](PyViz& self, const std::shared_ptr<viz::Lines>& obj) { return self.remove(obj); })
        .def("remove",
             [](PyViz& self, const std::shared_ptr<viz::ObjectOverlay>& obj) {
                 return self.remove(obj);
             })
        .def_prop_ro("fps", &PyViz::fps,
                     "Frames per second, updated every second in the draw() func")
        .def(
            "set_background_color",
            [](PyViz& self, const py::tuple& rgba) {
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                self.set_background_color(rgba_vec);
            },
            py::arg("rgba"),
            R"(
            Set the background color of the viz.

            Args:
                rgba: 4 value tuple of RGBA color
        )")
        .def("set_notification", &PyViz::set_notification, py::arg("text"),
             py::arg("duration") = 2.0,
             R"(
             Set a notification text to be displayed in the top-right corner of
             the window.

             Args:
                 text: Text to display. Empty string to clear.
                 duration: Duration in seconds to display the text (default: 2.0 seconds).
                           0 means indefinitely.
         )")
        .def_rw("notifications_enabled", &PyViz::notifications_enabled,
                "Enable or disable notifications.")
        .def(
            "__copy__", [](PyViz& self) { return &self; }, py::rv_policy::reference)
        .def(
            "__deepcopy__", [](PyViz& self, const py::object&) { return &self; },
            py::rv_policy::reference);

    module.def(
        "add_default_controls",
        [](PyViz& viz) {
            viz::add_default_controls(viz);
            // make sure we keep track of these pushes
            viz.key_cbs.emplace_back();
            viz.mp_cbs.emplace_back();
            viz.scroll_cbs.emplace_back();
        },
        "Add default keyboard and mouse bindings to a visualizer instance.");

    py::class_<viz::WindowCtx>(module, "WindowCtx", "Context for input callbacks.")
        .def(py::init<>())
        .def_ro("lbutton_down", &viz::WindowCtx::lbutton_down,
                "True if the left mouse button is held")
        .def_ro("mbutton_down", &viz::WindowCtx::mbutton_down,
                "True if the middle mouse button is held")
        .def_ro("rbutton_down", &viz::WindowCtx::rbutton_down,
                "True if the right mouse button is held")
        .def_ro("mouse_x", &viz::WindowCtx::mouse_x, "Current mouse x position")
        .def_ro("mouse_y", &viz::WindowCtx::mouse_y, "Current mouse y position")
        .def_ro("viewport_width", &viz::WindowCtx::viewport_width,
                "Current viewport width in pixels")
        .def_ro("viewport_height", &viz::WindowCtx::viewport_height,
                "Current viewport height in pixels")
        .def_ro("window_width", &viz::WindowCtx::window_width,
                "Current window width in screen coordinates")
        .def_ro("window_height", &viz::WindowCtx::window_height,
                "Current window height in screen coordinates")
        .def("aspect_ratio", &viz::WindowCtx::aspect_ratio,
             "Return the aspect ratio of the viewport.")
        .def("normalized_coordinates", &viz::WindowCtx::normalized_coordinates,
             "Return 2d normalized viewport coordinates given window "
             "coordinates.")
        .def("__copy__", [](const viz::WindowCtx& self) { return viz::WindowCtx{self}; })
        .def("__deepcopy__",
             [](const viz::WindowCtx& self, const py::object&) { return viz::WindowCtx{self}; });

    py::class_<viz::Camera>(module, "Camera", "Controls the camera view and projection.")
        .def("reset", &viz::Camera::reset, "Reset the camera view and fov.")
        .def("yaw", &viz::Camera::yaw, py::arg("degrees"),
             "Orbit the camera left or right about the camera target.")
        .def("set_yaw", &viz::Camera::set_yaw, py::arg("degrees"), "Set yaw in degrees.")
        .def("get_yaw", &viz::Camera::get_yaw, "Get yaw in degrees.")
        .def("roll", &viz::Camera::roll, py::arg("degrees"),
             "Roll the camera left or right about the camera target.")
        .def("set_roll", &viz::Camera::set_roll, py::arg("degrees"), "Set roll in degrees.")
        .def("get_roll", &viz::Camera::get_roll, "Get roll in degrees.")
        .def("pitch", &viz::Camera::pitch, py::arg("degrees"), "Pitch the camera up or down.")
        .def("set_pitch", &viz::Camera::set_pitch, py::arg("degrees"), "Set pitch in degrees.")
        .def("get_pitch", &viz::Camera::get_pitch, "Get pitch in degrees.")
        .def("dolly", &viz::Camera::dolly, py::arg("amount"),
             "Move the camera towards or away from the target.")
        .def("set_dolly", &viz::Camera::set_dolly, py::arg("log_distance"),
             "Set the dolly (i.e. log distance) of the camera from the target.")
        .def("get_dolly", &viz::Camera::get_dolly,
             "Get the dolly (i.e. log distance) of the camera from the target.")
        .def("dolly_xy", &viz::Camera::dolly_xy, py::arg("x"), py::arg("y"),
             R"(
             Move the camera in the XY plane of the camera view.

             Args:
                 x: horizontal offset
                 y: vertical offset
             )")
        .def("set_view_offset", &viz::Camera::set_view_offset, py::arg("view_offset"),
             "Set view offset of a camera")
        .def("get_view_offset", &viz::Camera::get_view_offset, "Get view offset of a camera")
        .def("set_fov", &viz::Camera::set_fov, py::arg("degrees"),
             "Set the diagonal field of view.")
        .def("get_fov", &viz::Camera::get_fov, "Get the diagonal field of view in degrees.")
        .def("set_orthographic", &viz::Camera::set_orthographic, py::arg("state"),
             "Use an orthographic or perspective projection.")
        .def("is_orthographic", &viz::Camera::is_orthographic, "Get the orthographic state.")
        .def("set_proj_offset", &viz::Camera::set_proj_offset, py::arg("x"), py::arg("y"),
             R"(
             Set the 2d position of camera target in the viewport.

             Args:
                 x: horizontal position in in normalized coordinates [-1, 1]
                 y: vertical position in in normalized coordinates [-1, 1]
             )")
        .def("get_proj_offset", &viz::Camera::get_proj_offset,
             "Get the 2d position of a camera target in the viewport.")
        .def(
            "set_target",
            [](viz::Camera& self, const pymatrixd& pose) {
                check_array(pose, 16, 2);
                viz::mat4d posea;
                std::copy(pose.data(), pose.data() + 16, posea.data());
                self.set_target(posea);
            },
            py::arg("pose"),
            R"(
                 Set the camera target pose (inverted pose).

                 Args:
                    pose: 4x4 homogeneous transformation matrix (row-major/C-order)
             )")
        .def("get_target", &viz::Camera::get_target, "Get a pose of the camera target.")
        .def("__copy__", [](const viz::Camera& self) { return viz::Camera{self}; })
        .def("__deepcopy__",
             [](const viz::Camera& self, const py::object&) { return viz::Camera{self}; });

    py::class_<viz::TargetDisplay>(module, "TargetDisplay",
                                   "Manages the state of the camera target display.")
        .def("enable_rings", &viz::TargetDisplay::enable_rings, py::arg("state"),
             "Enable or disable distance ring display.")
        .def("get_ring_size_m", &viz::TargetDisplay::get_ring_size_m,
             "Get the distance between rings in meters.")
        .def("set_ring_size", &viz::TargetDisplay::set_ring_size, py::arg("n"),
             "Set the distance between rings.")
        .def("set_ring_line_width", &viz::TargetDisplay::set_ring_line_width, py::arg("line_width"),
             "Set the line width of the rings.")
        .def(
            "__copy__", [](viz::TargetDisplay& self) { return &self; }, py::rv_policy::reference)
        .def(
            "__deepcopy__", [](viz::TargetDisplay& self, const py::object&) { return &self; },
            py::rv_policy::reference);

    py::class_<PyCloud>(module, "Cloud",
                        R"(
             Manages the state of a point cloud.

             Each point cloud consists of n points with w poses. The ith point will be
             transformed by the (i % w)th pose. For example for 2048 x 64 Ouster lidar
             point cloud, we may have w = 2048 poses and n = 2048 * 64 = 131072 points.

             We also keep track of a per-cloud pose to efficiently transform the
             whole point cloud without having to update all ~2048 poses.
             )",
                        py::type_slots(CLOUD_SLOTS))
        // Points only (extrinsics defaulted to Identity in C++)
        .def(
            "__init__",
            [](PyCloud* self, size_t num_points) {
                new (self) PyCloud(num_points, viz::IDENTITY4D);
            },
            py::arg("num_points"),
            R"(
                 ``def __init__(self, n_points: int, extrinsics: np.ndarray) -> None:``

                 Unstructured point cloud for visualization.

                 Call set_xyz() to update

                 Args:
                    num_points: number of points
             )")
        // Points + Extrinsics
        .def(
            "__init__",
            [](PyCloud* self, size_t num_points, const pymatrixd& extrinsics) {
                check_array(extrinsics, 16, 0);
                viz::mat4d extrinsica;
                std::copy(extrinsics.data(), extrinsics.data() + 16, extrinsica.data());
                new (self) PyCloud(num_points, extrinsica);
            },
            py::arg("num_points"), py::arg("extrinsics"),
            R"(
                 ``def __init__(self, n_points: int, extrinsics: np.ndarray) -> None:``

                 Unstructured point cloud for visualization.

                 Call set_xyz() to update

                 Args:
                    num_points: number of points
                    extrinsics: sensor extrinsic calibration. 4x4 row-major
                                homogeneous transformation matrix.
             )")
        .def(
            "__init__",
            [](PyCloud* self, const ouster::sdk::core::SensorInfo& info) {
                new (self) PyCloud{info};
            },
            py::arg("metadata"),
            R"(
                 ``def __init__(self, si: SensorInfo) -> None:``

                 Structured point cloud for visualization.

                 Call set_range() to update

                 Args:
                    info: sensor metadata
             )")
        .def(
            "__init__",
            [](PyCloud* self, const std::shared_ptr<ouster::sdk::core::XYZLut>& xyz_lut) {
                new (self) PyCloud{xyz_lut};
            },
            py::arg("xyzlut"),
            R"(
                 ``def __init__(self, xyzlut: XYZLut) -> None:``

                 Structured point cloud for visualization.

                 Call set_range() to update

                 Args:
                    xyzlut: sensor XYZLut
             )")
        .def(
            "set_range",
            [](PyCloud& self, const py::ndarray<py::ro, py::shape<-1, -1>>& range) {
                const auto range_c = ensure_c_contig<uint32_t, py::shape<-1, -1>>(range);
                self.set_range(range_c.data());
            },
            py::arg("range"),
            R"(
                Set the range values.

                Args:
                  range: array of at least as many elements as there are points,
                         representing the range of the points
              )")
        .def(
            "set_key",
            [](PyCloud& self, const py::ndarray<py::ro>& key) {
                if (key.dtype() == py::dtype<uint8_t>()) {
                    cloud_set_key_impl(self, ensure_c_contig<uint8_t>(key));
                } else if (ouster::sdk::python::detail::is_floating_dtype(key.dtype())) {
                    cloud_set_key_impl(self, ensure_c_contig_floating<float>(key));
                } else {
                    throw py::type_error("key must be uint8 or floating-point array");
                }
            },
            py::arg("key"),
            R"(
                 Set the key values, used for colouring.

                 Number of elements defines the type of Cloud coloration:
                 - num elements == cloud.get_size(): MONO with palette
                 - 3 dimensions with the last dimesion: 3 - RGB, 4 - RGBA,
                   no palette used

                 Args:
                    key: uint8 or float32, or float64 array of at least as many elements as there are
                         points, preferably normalized between 0 and 1
                            (float) or between 0 and 255 (uint8)
             )")
        .def(
            "set_mask",
            [](PyCloud& self, const py::ndarray<py::ro>& mask) {
                if (mask.dtype() == py::dtype<uint8_t>()) {
                    cloud_set_mask_impl(self, ensure_c_contig<uint8_t>(mask));
                } else if (ouster::sdk::python::detail::is_floating_dtype(mask.dtype())) {
                    cloud_set_mask_impl(self, ensure_c_contig_floating<float>(mask));
                } else {
                    throw py::type_error("mask must be uint8 or floating-point array");
                }
            },
            py::arg("mask"),
            R"(
                 Set the RGBA mask values, used as an overlay on top of the key.

                 Args:
                    mask: uint8, float32, or float64 array of at least 4x as many elements as there
                          are points, preferably normalized between 0 and 1
                          (float) or between 0 and 255 (uint8)
             )")
        .def(
            "set_xyz",
            [](PyCloud& self, const py::ndarray<py::ro>& xyz) {
                cloud_set_xyz_impl(self, ensure_c_contig_floating<float>(xyz));
            },
            py::arg("xyz"),
            R"(
                 Set the XYZ values.

                 :param xyz:  Supports 3 formats:
                              * array of exactly 3n where n is the number of
                                points, so that the xyz position of the ith
                                point is ``i``, ``i+n``, ``i+2n``.
                              * array of (N, 3) where N is the number of points
                              * array of (H, W, 3) where H*W is the number of
                                points
                 :type xyz: array of np.float32
             )")
        .def(
            "set_column_poses",
            [](PyCloud& self, const py::ndarray<py::ro>& column_poses) {
                const auto poses = ensure_c_contig_floating<float>(column_poses);
                check_array(poses, self.get_cols() * 16, 0);
                self.set_column_poses(poses.data());
            },
            py::arg("column_poses"),
            R"(
                 Set frame poses (per every column).

                 Args:
                    column_poses: array of poses (Wx4x4) per every column.
             )")
        .def(
            "set_pose",
            [](PyCloud& self, const pymatrixd& pose) {
                check_array(pose, 16, 2);
                viz::mat4d posea;
                std::copy(pose.data(), pose.data() + 16, posea.data());
                self.set_pose(posea);
            },
            py::arg("pose"), py::sig("def set_pose(self, pose: numpy.ndarray) -> None"),
            R"(
                 Set the ith point cloud pose.

                 Args:
                    pose: 4x4 homogeneous transformation matrix (row-major/C-order)
             )")
        .def("set_point_size", &PyCloud::set_point_size, py::arg("size"),
             R"(
            Set point size.

            Args:
                size: point size
        )")
        .def(
            "set_palette",
            [](PyCloud& self, const py::ndarray<py::ro, py::shape<-1, 3>>& buf) {
                const auto palette = ensure_c_contig_floating<float, py::shape<-1, 3>>(buf);
                self.set_palette(palette.data(), palette.shape(0));
            },
            py::arg("palette"),
            R"(
            Set the point cloud color palette.

            Args:
                palette: the new palette to use, must have size 3*palette_size
        )")
        .def(
            "clear_on_select",
            [](PyCloud& self) {
                self.clear_on_select();
                self.select_cb = {};
                if (self.select_id != 0) {
                    select_cb_mutex.lock();
                    select_cbs.erase(self.select_id);
                    select_cb_mutex.unlock();
                }
                self.select_id = 0;
            },
            "Clears the on select callback and makes this cloud no longer "
            "selectable.")
        .def_prop_ro("selectable", &PyCloud::selectable, "If the cloud is selectable or not.")
        .def(
            "set_on_select",
            [](PyCloud& self, const py::object& cb_obj) {
                self.select_cb = cb_obj;
                auto callback = py::cast<std::function<void(py::object)>>(cb_obj);

                // store the callback outside the cloud object so it cant get
                // copied
                select_cb_mutex.lock();
                self.select_id = ++select_cb_id;
                uint64_t id = self.select_id;
                select_cbs[id] = callback;
                select_cb_mutex.unlock();

                self.set_on_select([id](const std::vector<uint32_t>& selection) {
                    py::gil_scoped_acquire acquire;
                    std::vector<size_t> shape = {selection.size()};
                    size_t size = selection.size();
                    auto data = new uint32_t[size];
                    auto capsule = py::capsule(data, [](void* pointer) noexcept {
                        delete[] static_cast<uint32_t*>(pointer);
                    });

                    py::ndarray<py::numpy, uint32_t, py::c_contig, py::shape<-1>> res(
                        data, shape.size(), shape.data(), capsule);
                    auto view = res.view();
                    for (size_t i = 0; i < selection.size(); i++) {
                        view(i) = selection[i];
                    }

                    // call the callback from the callback storage
                    std::unique_lock<std::mutex> lock(select_cb_mutex);
                    auto cb_iter = select_cbs.find(id);
                    if (cb_iter == select_cbs.end()) {
                        // callback was cleared, do nothing
                        return;
                    }
                    auto cb = cb_iter->second;
                    lock.unlock();
                    cb(res.cast());
                });
            },
            "Makes the cloud selectable and adds a callback called on "
            "selection.")
        .def_prop_ro("size", &PyCloud::get_size, "Number of points in a cloud")
        .def_prop_ro("cols", &PyCloud::get_cols,
                     "Number of columns in a cloud (1 if point cloud is unstructured")
        .def("__copy__", [](const PyCloud& self) { return PyCloud{self}; })
        .def("__deepcopy__", [](const PyCloud& self, const py::object&) { return PyCloud{self}; })
        .def("__repr__", [](const PyCloud& self) {
            std::stringstream stream;
            stream << "<ouster.sdk.viz.Cloud " << &self << ", pts = " << self.get_size()
                   << ", cols = " << self.get_cols() << ">";
            return stream.str();
        });

    py::class_<viz::Image>(module, "Image", "Manages the state of an image.")
        .def(py::init<>())
        .def(
            "set_image",
            [](viz::Image& self, const py::ndarray<py::ro>& image) {
                if (image.dtype() == py::dtype<uint8_t>()) {
                    image_set_image_impl(self, ensure_c_contig<uint8_t>(image));
                } else if (ouster::sdk::python::detail::is_floating_dtype(image.dtype())) {
                    image_set_image_impl(self, ensure_c_contig_floating<float>(image));
                } else {
                    throw py::type_error("image must be uint8 or floating-point array");
                }
            },
            py::arg("image"), py::sig("def set_image(self, image: numpy.ndarray) -> None"), R"(
                 Set the image data, MONO or RGB/RGBA depending on dimensions.

                 Color palette is applied for MONO mode if set_palette() was
                 used to set the palette, otherwise MONO mode makes the
                 monochrome image.

                 Args:
                    image: uint8, float32, or float64 2D array for a monochrome image or 3D array
                           with RGB or RGBA components for color image.
             )")
        .def(
            "set_mask",
            [](viz::Image& self, const py::ndarray<py::ro, py::shape<-1, -1, 4>>& buf) {
                if (buf.dtype() == py::dtype<uint8_t>()) {
                    const auto mask = ensure_c_contig<uint8_t, py::shape<-1, -1, 4>>(buf);
                    self.set_mask(mask.shape(1), mask.shape(0), mask.data());
                } else if (ouster::sdk::python::detail::is_floating_dtype(buf.dtype())) {
                    const auto mask = ensure_c_contig_floating<float, py::shape<-1, -1, 4>>(buf);
                    self.set_mask(mask.shape(1), mask.shape(0), mask.data());
                } else {
                    throw py::type_error("mask must be uint8 or floating-point array");
                }
            },
            py::arg("mask"),
            R"(
                 Set the RGBA mask.

                 Args:
                    mask: uint8, float32, or float64 M x N x 4 array with RGBA mask
             )")
        .def("set_position", &viz::Image::set_position, py::arg("x_min"), py::arg("x_max"),
             py::arg("y_min"), py::arg("y_max"),
             R"(
            Set the display position of the image.

            Coordinates are {x_min, x_max, y_max, y_min} in sort-of normalized
            screen coordinates: y is in [-1, 1], and x uses the same scale
            (i.e. window width is ignored). This is currently just the same
            method the previous hard-coded 'image_frac' logic was using; I
            believe it was done this way to allow scaling with the window
            while maintaining the aspect ratio.

            Args:
                x_min: min x
                x_max: max x
                y_min: min y
                y_max: max y
        )")
        .def("set_hshift", &viz::Image::set_hshift, py::arg("hshift"),
             R"(
            Set horizontal shift in normalized viewport screen width coordinate.

            This may be used to "snap" images to the left/right screen edges.

            Some example values:
              ``0`` - default, image is centered horizontally on the screen
              ``-0.5`` - image moved to the left for the 1/4 of a horizontal viewport
              ``-1`` - image moved to the left for the 1/2 of a horizontal viewport
              ``+1`` - image moved to the right for the 1/2 of a horizontal viewport
              ``+0.5`` - image moved to the right for the 1/4 of a horizontal viewport
        )")
        .def(
            "set_palette",
            [](viz::Image& self, const py::ndarray<py::ro, py::shape<-1, 3>>& buf) {
                const auto palette = ensure_c_contig_floating<float, py::shape<-1, 3>>(buf);
                self.set_palette(palette.data(), palette.shape(0));
            },
            py::arg("palette"),
            R"(
            Set the image color palette.

            Args:
                palette: the new palette to use, must have size 3*palette_size
        )")
        .def("clear_palette", &viz::Image::clear_palette,
             "Removes the image palette and use keys as grey color in MONO")
        .def(
            "viewport_coordinates_to_image_pixel",
            [](viz::Image& self, const viz::WindowCtx& ctx, double x,
               double y) -> std::pair<int, int> {
                auto res = self.viewport_coordinates_to_image_pixel(ctx, x, y);
                return {res.second, res.first};
            },
            R"(Returns the image pixel as a (row, col) tuple given window coordinates,
             or None if the given window coordinate is not within the image.)")
        .def(
            "image_pixel_to_viewport_coordinates",
            [](viz::Image& self, const viz::WindowCtx& ctx,
               std::pair<int, int> pixel) -> std::pair<double, double> {
                return self.image_pixel_to_viewport_coordinates(ctx, pixel.second, pixel.first);
            },
            R"(Returns the window pixel (x, y) given an image (row, col) pixel.)")
        .def("pixel_size", &viz::Image::pixel_size,
             "Returns the pixel size (w, h) in window pixels.")
        .def("__copy__", [](const viz::Image& self) { return viz::Image{self}; })
        .def("__deepcopy__",
             [](const viz::Image& self, const py::object&) { return viz::Image{self}; });

    py::class_<viz::Vertex3f>(module, "Vertex3f",
                              "A vertex consisting of a position and a normal vector.")
        .def(py::init<Eigen::Vector3f, Eigen::Vector3f>())
        .def_rw("position", &viz::Vertex3f::position, "vertex position")
        .def_rw("normal", &viz::Vertex3f::normal, "vertex normal")
        .def("__copy__", [](const viz::Vertex3f& self) { return viz::Vertex3f{self}; })
        .def("__deepcopy__",
             [](const viz::Vertex3f& self, const py::object&) { return viz::Vertex3f{self}; });

    py::class_<viz::Mesh>(module, "Mesh", "Manages the state of a single mesh.")
        .def(
            "__init__",
            [](viz::Mesh* self, const py::list& vertices,
               const py::ndarray<const unsigned int>& face_indices,
               const py::ndarray<const unsigned int>& edge_indices, const py::tuple& face_rgba,
               const py::tuple& edge_rgba) {
                std::vector<viz::Vertex3f> vertex_vector;
                for (py::handle obj : vertices) {
                    vertex_vector.push_back(py::cast<viz::Vertex3f>(obj));
                }
                viz::vec4f face_rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(face_rgba_vec, face_rgba);
                viz::vec4f edge_rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(edge_rgba_vec, edge_rgba);

                new (self) viz::Mesh(
                    vertex_vector,
                    std::vector<unsigned int>(
                        face_indices.data(),
                        face_indices.data() + face_indices.nbytes() / edge_indices.itemsize()),
                    std::vector<unsigned int>(
                        edge_indices.data(),
                        edge_indices.data() + edge_indices.nbytes() / face_indices.itemsize()),
                    face_rgba_vec, edge_rgba_vec);
            },
            py::arg("vertices"), py::arg("face_indices"), py::arg("edge_indices"),
            py::arg("face_rgba"), py::arg("edge_rgba"))
        .def_static("from_simple_mesh", &viz::Mesh::from_simple_mesh)
        .def(
            "set_face_rgba",
            [](viz::Mesh& self, const py::tuple& rgba) {
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                self.set_face_rgba(rgba_vec);
            },
            py::arg("rgba"),
            R"(
        Set the face color of the mesh.

        Args:
            rgba: 4 value tuple of RGBA color
    )")
        .def(
            "set_edge_rgba",
            [](viz::Mesh& self, const py::tuple& rgba) {
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                self.set_edge_rgba(rgba_vec);
            },
            py::arg("rgba"),
            R"(
        Set the edge color of the mesh.

        Args:
            rgba: 4 value tuple of RGBA color
    )")
        .def(
            "set_transform",
            [](viz::Mesh& self, const pymatrixd& pose) {
                check_array(pose, 16, 2);
                viz::mat4d posea;
                std::copy(pose.data(), pose.data() + 16, posea.data());
                self.set_transform(posea);
            },
            py::arg("pose"),
            R"(
             Set the transform defining the mesh.

             Args:
                pose: 4x4 pose matrix
         )")
        .def("__copy__", [](const viz::Mesh& self) { return viz::Mesh{self}; })
        .def("__deepcopy__",
             [](const viz::Mesh& self, const py::object&) { return viz::Mesh{self}; });

    py::class_<PyCuboid>(module, "Cuboid", py::is_weak_referenceable(),
                         py::type_slots(CUBOID_SLOTS), "Manages the state of a single cuboid.")
        .def(
            "__init__",
            [](PyCuboid* self, const pymatrixd& pose, const py::tuple& rgba) {
                check_array(pose, 16, 2);
                viz::mat4d posea;
                std::copy(pose.data(), pose.data() + 16, posea.data());
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                new (self) PyCuboid{posea, rgba_vec};
            },
            py::arg("pose"), py::arg("rgba"),
            R"(
                 Creates cuboid.

                 Args:
                    pose: 4x4 pose matrix
                    rgba: 4 value tuple of RGBA color
             )")
        .def_static(
            "from_object",
            [](const ouster::sdk::core::Object& object, const py::tuple& rgba) {
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                auto cuboid = viz::Cuboid::from_object(object, rgba_vec);
                return std::make_shared<PyCuboid>(cuboid.get_transform(), cuboid.get_rgba());
            },
            py::arg("object"), py::arg("rgba"),
            R"(Creates a cuboid from an object with position, dimensions and rotation attributes, and an RGBA color.)")

        .def(
            "set_transform",
            [](PyCuboid& self, const pymatrixd& pose) {
                check_array(pose, 16, 2);
                viz::mat4d posea;
                std::copy(pose.data(), pose.data() + 16, posea.data());
                self.set_transform(posea);
            },
            py::arg("pose"),
            R"(
                 Set the transform defining the cuboid.

                 Applied to a unit cube centered at the origin.

                 Args:
                    pose: 4x4 pose matrix
             )")
        .def(
            "set_rgba",
            [](PyCuboid& self, const py::tuple& rgba) {
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                self.set_rgba(rgba_vec);
            },
            py::arg("rgba"),
            R"(
            Set the color of the cuboid.

            Args:
                rgba: 4 value tuple of RGBA color
        )")
        .def(
            "clear_on_select",
            [](PyCuboid& self) {
                self.clear_on_select();
                self.select_cb = {};
                if (self.select_id != 0) {
                    select_cb_mutex.lock();
                    select_cbs.erase(static_cast<int>(self.select_id));
                    select_cb_mutex.unlock();
                }
                self.select_id = 0;
            },
            "Clears the on select callback and makes this cuboid no longer "
            "selectable.")
        .def_prop_ro("selectable", &PyCuboid::selectable, "If the cuboid is selectable or not.")
        .def(
            "set_on_select",
            [](PyCuboid& self, const py::object& cb_obj) {
                self.select_cb = cb_obj;
                auto callback = py::cast<std::function<void(py::object)>>(cb_obj);

                // store the callback outside the cuboid object so it cant get
                // copied
                select_cb_mutex.lock();
                self.select_id = ++select_cb_id;
                uint64_t id = self.select_id;
                select_cbs[id] = callback;
                select_cb_mutex.unlock();

                self.set_on_select([id](const std::vector<uint32_t>& selection) {
                    py::gil_scoped_acquire acquire;
                    std::vector<size_t> shape = {selection.size()};
                    size_t size = selection.size();
                    auto data = new uint32_t[size];
                    auto capsule = py::capsule(data, [](void* pointer) noexcept {
                        delete[] static_cast<uint32_t*>(pointer);
                    });

                    py::ndarray<py::numpy, uint32_t, py::c_contig, py::shape<-1>> res(
                        data, shape.size(), shape.data(), capsule);
                    auto view = res.view();
                    for (size_t i = 0; i < selection.size(); i++) {
                        view(i) = selection[i];
                    }

                    // call the callback from the callback storage
                    std::unique_lock<std::mutex> lock(select_cb_mutex);
                    auto cb_iter = select_cbs.find(id);
                    if (cb_iter == select_cbs.end()) {
                        // callback was cleared, do nothing
                        return;
                    }
                    auto cb = cb_iter->second;
                    lock.unlock();
                    cb(res.cast());
                });
            },
            "Makes the cuboid selectable and adds a callback called on "
            "selection.")
        .def("__copy__", [](const PyCuboid& self) { return self.copy_without_select(); })
        .def("__deepcopy__",
             [](const PyCuboid& self, const py::object&) { return self.copy_without_select(); });

    py::class_<viz::Lines>(module, "Lines", "Manages the state of line segments.")
        .def(
            "__init__",
            [](viz::Lines* self, const pymatrixd& pose, const py::tuple& rgba) {
                check_array(pose, 16, 2);
                viz::mat4d posea;
                std::copy(pose.data(), pose.data() + 16, posea.data());
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                new (self) viz::Lines{posea, rgba_vec};
            },
            py::arg("pose"), py::arg("rgba"),
            R"(
                 Creates lines.

                 Args:
                    pose: 4x4 pose matrix
                    rgba: 4 value tuple of RGBA color
             )")
        .def(
            "set_points",
            [](viz::Lines& self, const py::ndarray<py::ro>& points) {
                lines_set_points_impl(self, ensure_c_contig_floating<float>(points));
            },
            py::arg("points"),
            R"(
                 Set the line points.

                 Args:
                    points: array of floats
             )")
        .def(
            "set_transform",
            [](viz::Lines& self, const pymatrixd& pose) {
                check_array(pose, 16, 2);
                viz::mat4d posea;
                std::copy(pose.data(), pose.data() + 16, posea.data());
                self.set_transform(posea);
            },
            py::arg("pose"),
            R"(
                 Set the transform for the lines.

                 Args:
                    pose: 4x4 pose matrix
             )")
        .def(
            "set_rgba",
            [](viz::Lines& self, const py::tuple& rgba) {
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                self.set_rgba(rgba_vec);
            },
            py::arg("rgba"),
            R"(
            Set the color of the lines.

            Args:
                rgba: 4 value tuple of RGBA color
        )")
        .def("__copy__", [](const viz::Lines& self) { return viz::Lines{self}; })
        .def("__deepcopy__",
             [](const viz::Lines& self, const py::object&) { return viz::Lines{self}; });

    py::class_<viz::Label>(module, "Label", "Manages the state of a text label.")
        .def(
            "__init__",
            [](viz::Label* self, const std::string& text, double x, double y, double z) {
                new (self) viz::Label{text, {x, y, z}};
            },
            py::arg("text"), py::arg("x"), py::arg("y"), py::arg("z"),
            R"(
                 ``def __init__(self, text: str, x: float, y: float, z: float) -> None:``

                 Creates 3D Label.

                 Args:
                    text: label text
                    x,y,z: label location
             )")
        .def(
            "__init__",
            [](viz::Label* self, const std::string& text, float x, float y, bool align_right,
               bool align_top) { new (self) viz::Label(text, x, y, align_right, align_top); },
            py::arg("text"), py::arg("x"), py::arg("y"), py::arg("align_right") = false,
            py::arg("align_top") = false,
            R"(
                 ``def __init__(self, text: str, x: float, y: float, align_right: bool = ..., align_top: bool = ...) -> None:``

                 Creates 2D Label.

                 Args:
                    text: label text
                    x,y: label 2D location in screen coords ``[0..1]``, corresponding to top left corner of label
                    align_right: if ``True`` - anchor point of the label is the right side
                    align_top: if ``True`` - anchor point of the label is the top side
             )")
        .def("set_text", &viz::Label::set_text, py::arg("text"),
             R"(
            Update label text.

            Args:
                text: new text to display
        )")
        .def(
            "set_position",
            [](viz::Label& self, double x, double y, double z) {
                self.set_position({x, y, z});
            },
            py::arg("x"), py::arg("y"), py::arg("z"),
            R"(
                ``def set_position(self, x: float, y: float, z: float) -> None:``

                 Set label position. Position correspnods to top left (viewer's left) of label.

                 Args:
                    x,y,z: label position in 3D
            )")
        .def(
            "set_position",
            [](viz::Label& self, float x, float y, bool align_right, bool align_top) {
                self.set_position(x, y, align_right, align_top);
            },
            py::arg("x"), py::arg("y"), py::arg("align_right") = false,
            py::arg("align_top") = false,
            R"(
                 ``def set_position(self, x: float, y: float, align_right: bool = ...) -> None:``

                 Set position of the 2D label.

                 Args:
                    x,y: label 2D position in screen coords ``[0..1]``
                    align_right: if ``True`` - anchor point of the label is the right side
                    align_top: if ``True`` - anchor point of the label is the top side
             )")
        .def("set_scale", &viz::Label::set_scale, py::arg("scale"),
             R"(
             Set scaling factor of the label.

             Args:
                scale: text scale factor
         )")
        .def(
            "set_rgba",
            [](viz::Label& self, const py::tuple& rgba) {
                viz::vec4f rgba_vec{0.0, 0.0, 0.0, 1.0};
                tuple_to_float_array(rgba_vec, rgba);
                self.set_rgba(rgba_vec);
            },
            py::arg("rgba"),
            R"(
            Set the color of the label.

            Args:
                rgba: 4 value tuple of RGBA color
        )")
        .def_prop_ro("text_height", &viz::Label::get_text_height,
                     R"(
                               Get the height of the label text.
                               )")
        .def("__copy__", [](const viz::Label& self) { return viz::Label{self}; })
        .def("__deepcopy__",
             [](const viz::Label& self, const py::object&) { return viz::Label{self}; });

    py::class_<viz::ObjectOverlay>(module, "ObjectOverlay",
                                   "Manages the state of an object overlay.")
        .def(py::init<>())
        .def(
            "set_cuboids",
            [](viz::ObjectOverlay& self, const std::vector<std::shared_ptr<PyCuboid>>& py_cuboids) {
                std::vector<viz::Cuboid> cuboids;
                cuboids.reserve(py_cuboids.size());
                for (const auto& py_cuboid : py_cuboids) {
                    if (py_cuboid) {
                        cuboids.push_back(*py_cuboid);
                    }
                }
                self.set_cuboids(cuboids);
            },
            py::arg("cuboids"), "Set the cuboids to be displayed.")
        .def("set_sensor_info", &viz::ObjectOverlay::set_sensor_info, py::arg("info"),
             "Set the sensor information for projection.")
        .def(
            "set_view_matrix",
            [](viz::ObjectOverlay& self, const pymatrixd& mat) {
                check_array(mat, 16, 2);
                viz::mat4d mata;
                std::copy(mat.data(), mat.data() + 16, mata.data());
                self.set_view_matrix(mata);
            },
            py::arg("mat"), "Set the view matrix for panorama projection.")
        .def("set_position", &viz::ObjectOverlay::set_position, py::arg("x_min"), py::arg("x_max"),
             py::arg("y_min"), py::arg("y_max"), "Set the display position of the overlay.")
        .def("set_hshift", &viz::ObjectOverlay::set_hshift, py::arg("hshift"),
             "Set horizontal shift in normalized viewport screen width "
             "coordinate.")
        .def("__copy__", [](const viz::ObjectOverlay& self) { return viz::ObjectOverlay{self}; })
        .def("__deepcopy__", [](const viz::ObjectOverlay& self, const py::object&) {
            return viz::ObjectOverlay{self};
        });

    module.attr("distinct_palette") =
        py::ndarray<const float, py::c_contig, py::device::cpu, py::ndim<2>, py::numpy>(
            &viz::DISTINCT_PALETTE[0][0], {viz::DISTINCT_N, 3}, py::none());
    module.attr("spezia_palette") =
        py::ndarray<const float, py::c_contig, py::device::cpu, py::ndim<2>, py::numpy>(
            &viz::SPEZIA_PALETTE[0][0], {viz::SPEZIA_N, 3}, py::none());
    module.attr("spezia_cal_ref_palette") =
        py::ndarray<const float, py::c_contig, py::device::cpu, py::ndim<2>, py::numpy>(
            &viz::SPEZIA_CAL_REF_PALETTE[0][0], {viz::SPEZIA_CAL_REF_N, 3}, py::none());

    module.attr("calref_palette") =
        py::ndarray<const float, py::c_contig, py::ndim<2>, py::device::cpu, py::numpy>(
            &viz::CALREF_PALETTE[0][0], {viz::CALREF_N, 3}, py::none());

    module.attr("grey_palette") =
        py::ndarray<const float, py::c_contig, py::device::cpu, py::ndim<2>, py::numpy>(
            &viz::GREY_PALETTE[0][0], {viz::GREY_N, 3}, py::none());
    module.attr("grey_cal_ref_palette") =
        py::ndarray<const float, py::c_contig, py::ndim<2>, py::device::cpu, py::numpy>(
            &viz::GREY_CAL_REF_PALETTE[0][0], {viz::GREY_CAL_REF_N, 3}, py::none());

    module.attr("viridis_palette") =
        py::ndarray<const float, py::c_contig, py::ndim<2>, py::device::cpu, py::numpy>(
            &viz::VIRIDIS_PALETTE[0][0], {viz::VIRIDIS_N, 3}, py::none());
    module.attr("viridis_cal_ref_palette") =
        py::ndarray<const float, py::c_contig, py::ndim<2>, py::device::cpu, py::numpy>(
            &viz::VIRIDIS_CAL_REF_PALETTE[0][0], {viz::VIRIDIS_CAL_REF_N, 3}, py::none());

    module.attr("magma_palette") =
        py::ndarray<const float, py::c_contig, py::ndim<2>, py::device::cpu, py::numpy>(
            &viz::MAGMA_PALETTE[0][0], {viz::MAGMA_N, 3}, py::none());
    module.attr("magma_cal_ref_palette") =
        py::ndarray<const float, py::c_contig, py::ndim<2>, py::device::cpu, py::numpy>(
            &viz::MAGMA_CAL_REF_PALETTE[0][0], {viz::MAGMA_CAL_REF_N, 3}, py::none());

    module.def("voxel_style_mesh_from_zone_image_pair", &viz::voxel_style_mesh_from_zone_image_pair,
               py::arg("zone_image_pair"), py::arg("voxel_vertex_data"), py::arg("sensor_info"));

    py::class_<viz::VoxelVertexData>(module, "VoxelVertexData")
        .def(py::init<>())
        .def_rw("direction", &viz::VoxelVertexData::direction,
                "Direction vector (Eigen::Vector3d -> np.ndarray[float64[3,1]])")
        .def_rw("offset", &viz::VoxelVertexData::offset,
                "Offset vector (Eigen::Vector3d -> np.ndarray[float64[3,1]])")
        .def("__copy__",
             [](const viz::VoxelVertexData& self) { return viz::VoxelVertexData{self}; })
        .def("__deepcopy__", [](const viz::VoxelVertexData& self, const py::object&) {
            return viz::VoxelVertexData{self};
        });

    module.def("precompute_voxel_vertices", &viz::precompute_voxel_vertices, py::arg("metadata"),
               R"(
              Pre-computes direction and offset vectors for every vertex in the pixel grid.

              This is an expensive, one-time calculation that generates a lookup table
              which can be reused for creating meshes from any number of image pairs of
              the same resolution from the same sensor.

              Args:
                  metadata (ouster.sdk.sensor.SensorInfo): The sensor metadata.

              Returns:
                  list[list[VoxelVertexData]]: A 2D lookup table of geometric data
                  for the grid corners.
    )");

    module.attr("__version__") = ouster::sdk::SDK_VERSION;
}

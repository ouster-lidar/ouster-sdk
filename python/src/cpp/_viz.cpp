/**
 * @file
 * @brief pybind wrappers for the ouster simple viz library
 *
 * This is purely a PoC for exposing the opengl visualizer in Python that should
 * help us figure out how to update the C++ API. Currently doesn't shut down
 * cleanly at all.
 */
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <vector>

#include "ouster/image_processing.h"
#include "ouster/lidar_scan.h"
#include "ouster/lidar_scan_viz.h"
#include "ouster/point_viz.h"
#include "ouster/types.h"

namespace py = pybind11;
using namespace ouster;

/*
 * LidarScanViz needs to be wrapped since it keeps a reference to point_viz and
 * doesn't provide an easy way to initialize itself.
 *
 * TODO: fix the C++ API
 */
struct PyViz {
    std::unique_ptr<viz::PointViz> point_viz;
    std::unique_ptr<viz::LidarScanViz> scan_viz;

    explicit PyViz(const sensor::sensor_info& info) {
        const uint32_t H = info.format.pixels_per_column;
        const uint32_t W = info.format.columns_per_frame;

        auto packet_format = sensor::get_format(info);
        const auto xyz_lut = make_xyz_lut(info);

        point_viz = std::make_unique<viz::PointViz>(
            std::vector<viz::CloudSetup>{
                {xyz_lut.direction.data(), xyz_lut.offset.data(), H * W, W,
                 info.extrinsic.data()},
                {xyz_lut.direction.data(), xyz_lut.offset.data(), H * W, W,
                 info.extrinsic.data()}},
            "Ouster Viz (Python)", false);

        scan_viz = std::make_unique<viz::LidarScanViz>(info, *point_viz.get());
    }

    void draw(const LidarScan& scan, int cloud_ind) {
        this->scan_viz->draw(scan, cloud_ind);
    }

    // TODO: should be safe, but maybe there's a less ugly way to do this
    void loop() {
        py::gil_scoped_release release;

        // set signal handler to exit current viz for the duration of drawLoop()
        thread_local std::atomic_bool& quit = this->point_viz->quit;
        auto sig = std::signal(SIGINT, [](int) { quit = true; });

        this->point_viz->drawLoop();

        // restore python signal handler
        std::signal(SIGINT, sig);
    }

    bool is_quit() { return this->point_viz->quit; }

    void quit() { this->point_viz->quit = true; }
};

PYBIND11_PLUGIN(_viz) {
    py::module m("_viz",
                 R"(Ouster viz bindings generated by pybind11.

This module is generated from the C++ code and not meant to be used directly.
)");

    // turn off signatures in docstrings: mypy stubs provide better types
    py::options options;
    options.disable_function_signatures();

    py::class_<PyViz>(m, "PyViz")
        .def(py::init<const sensor::sensor_info&>())
        .def("draw", &PyViz::draw, py::arg("scan"), py::arg("cloud_ind") = 0)
        .def("loop", &PyViz::loop)
        .def("is_quit", &PyViz::is_quit)
        .def("quit", &PyViz::quit);

    m.attr("__version__") = VERSION_INFO;

    return m.ptr();
}

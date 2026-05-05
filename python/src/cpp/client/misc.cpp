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

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pyerrors.h>
#include <warnings.h>

#include "client_common.h"
#include "ouster/client.h"
#include "ouster/error_handler.h"
#include "ouster/impl/build.h"
#include "ouster/impl/logging.h"
#include "ouster/io_type.h"
#include "ouster/types.h"

namespace py = pybind11;

void init_client_misc(py::module& module, py::module& /*unused*/) {
    // turn off signatures in docstrings: mypy stubs provide better types
    py::options options;
    options.disable_function_signatures();

    module.def(
        "init_logger",
        [](const std::string& log_level, const std::string& log_file_path,
           bool rotating, int max_size_in_bytes, int max_files) {
            return ouster::sdk::sensor::init_logger(log_level, log_file_path,
                                                    rotating, max_size_in_bytes,
                                                    max_files);
        },
        R"(
    Initializes and configures ouster_client logs. This method should be invoked
    only once before calling any other method from the library if the user wants
    to direct the library log statements to a different medium (other than
    console which is the default).

    Args:
        log_level Control the level of log messages outputed by the client.
            Valid options are (case-sensitive): "trace", "debug", "info", "warning",
            "error", "critical" and "off".
        log_file_path (str): Path to location where log files are stored. The
            path must be in a location that the process has write access to. If an empty
            string is provided then the logs will be directed to the console. When
            an empty string is passed then the rest of parameters are ignored.
        rotating (bool): Configure the log file with rotation, rotation rules are
            specified through the two following parameters max_size_in_bytes and
            max_files. If rotating is set to false the following parameters are ignored
        max_size_in_bytes (int): Maximum number of bytes to write to a rotating log
            file before starting a new file. ignored if rotating is set to False.
        max_files (int): Maximum number of rotating files to accumlate before
            re-using the first file. ignored if rotating is set to False.

    Returns:
        returns True on success, False otherwise.
    )",
        py::arg("log_level"), py::arg("log_file_path") = "",
        py::arg("rotating") = false, py::arg("max_size_in_bytes") = 0,
        py::arg("max_files") = 0);

    module.attr("__version__") = ouster::sdk::SDK_VERSION;

    py::enum_<ouster::sdk::core::Severity>(module, "Severity")
        .value("OUSTER_WARNING", ouster::sdk::core::Severity::OUSTER_WARNING)
        .value("OUSTER_ERROR", ouster::sdk::core::Severity::OUSTER_ERROR);

    py::enum_<ouster::sdk::core::IoType>(module, "IoType", py::arithmetic(),
                                         R"(
    Enumeration of input/output types.

    This enum defines the various types of input/output formats supported by the SDK.
    )")
        .value("OSF", ouster::sdk::core::IoType::OSF)
        .value("PCAP", ouster::sdk::core::IoType::PCAP)
        .value("SENSOR", ouster::sdk::core::IoType::SENSOR)
        .value("BAG", ouster::sdk::core::IoType::BAG)
        .value("CSV", ouster::sdk::core::IoType::CSV)
        .value("PLY", ouster::sdk::core::IoType::PLY)
        .value("PCD", ouster::sdk::core::IoType::PCD)
        .value("LAS", ouster::sdk::core::IoType::LAS)
        .value("MCAP", ouster::sdk::core::IoType::MCAP)
        .value("STL", ouster::sdk::core::IoType::STL)
        .value("PNG", ouster::sdk::core::IoType::PNG);

    module.def("io_type", &ouster::sdk::core::io_type, py::arg("uri"),
               R"(
    Determine the input/output type for a given URI.
    )");

    module.def("io_type_from_extension",
               &ouster::sdk::core::io_type_from_extension, py::arg("filename"),
               R"(
        Determine the input/output type based on a file extension.
        )");

    module.def("extension_from_io_type",
               &ouster::sdk::core::extension_from_io_type, py::arg("io_type"),
               R"(
        Get the file extension for a given input/output type.
        )");
}


#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ouster/open_source.h"

namespace py = pybind11;

void init_client(py::module&, py::module&);
void init_pcap(py::module&, py::module&);
void init_osf(py::module&, py::module&);
void init_viz(py::module&, py::module&);
void init_mapping(py::module&, py::module&);

void parse_packet_source_options(const py::kwargs& args,
                                 ouster::PacketSourceOptions& options) {
    for (const auto& item : args) {
        // this check is probably unnecessary
        if (!py::isinstance<py::str>(item.first)) {
            throw std::invalid_argument("Incorrect key type for kwargs.");
        }

        auto key = py::cast<std::string>(item.first);
        if (key == "lidar_port") {
            if (py::isinstance<py::none>(item.second)) {
                continue;
            }
            options.lidar_port =
                nonstd::optional<uint16_t>(py::cast<uint16_t>(item.second));
        } else if (key == "imu_port") {
            if (py::isinstance<py::none>(item.second)) {
                continue;
            }
            options.imu_port =
                nonstd::optional<uint16_t>(py::cast<uint16_t>(item.second));
        } else if (key == "no_auto_udp_dest") {
            options.no_auto_udp_dest = py::cast<bool>(item.second);
        } else if (key == "do_not_reinitialize") {
            options.do_not_reinitialize = py::cast<bool>(item.second);
        } else if (key == "timeout") {
            options.timeout = py::cast<float>(item.second);
        } else if (key == "sensor_info") {
            options.sensor_info =
                py::cast<std::vector<ouster::sensor::sensor_info>>(item.second);
        } else if (key == "sensor_config") {
            options.sensor_config =
                py::cast<std::vector<ouster::sensor::sensor_config>>(
                    item.second);
        } else if (key == "extrinsics") {
            auto extrinsics =
                py::cast<std::vector<py::array_t<double>>>(item.second);
            std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> exts;
            for (auto& pose : extrinsics) {
                // Ensure the pose is a 4x4 matrix
                if (pose.ndim() != 2 || pose.shape(0) != 4 ||
                    pose.shape(1) != 4) {
                    throw std::invalid_argument(
                        "Extrinsic matrix must have shape (4, 4)");
                }

                // Create a C-style copy of pose if it's neither C-style nor
                // F-style const
                const py::array_t<double>* pose_ptr = &pose;
                py::array_t<double> c_style_pose;
                if (!(pose.flags() & py::array::c_style)) {
                    c_style_pose =
                        py::array_t<double, py::array::c_style>(pose);
                    pose_ptr =
                        &c_style_pose;  // Use the C-style array for processing
                }

                // Convert pose to Eigen format
                Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
                    pose_eigen(pose_ptr->data());
                exts.push_back(pose_eigen);
            }
            options.extrinsics = exts;
        } else if (key == "extrinsics_file") {
            options.extrinsics_file = py::cast<std::string>(item.second);
        } else if (key == "index") {
            options.index = py::cast<bool>(item.second);
        } else if (key == "soft_id_check") {
            options.soft_id_check = py::cast<bool>(item.second);
        } else if (key == "config_timeout") {
            options.config_timeout = py::cast<float>(item.second);
        } else if (key == "buffer_time_sec") {
            options.buffer_time_sec = py::cast<float>(item.second);
        } else if (key == "meta") {
            options.meta = py::cast<std::vector<std::string>>(item.second);
        } else {
            throw std::invalid_argument("Unknown parameter '" + key +
                                        "' for source.");
        }
    }
}

void parse_scan_source_options(const py::kwargs& args,
                               ouster::ScanSourceOptions& options) {
    for (const auto& item : args) {
        // this check is probably unnecessary
        if (!py::isinstance<py::str>(item.first)) {
            throw std::invalid_argument("Incorrect key type for kwargs.");
        }

        auto key = py::cast<std::string>(item.first);
        try {
            if (key == "error_handler") {
                if (py::isinstance<py::none>(item.second)) {
                    continue;
                }
                options.error_handler =
                    py::cast<ouster::core::error_handler_t>(item.second);
            } else if (key == "lidar_port") {
                if (py::isinstance<py::none>(item.second)) {
                    continue;
                }
                options.lidar_port =
                    nonstd::optional<uint16_t>(py::cast<uint16_t>(item.second));
            } else if (key == "imu_port") {
                if (py::isinstance<py::none>(item.second)) {
                    continue;
                }
                options.imu_port =
                    nonstd::optional<uint16_t>(py::cast<uint16_t>(item.second));
            } else if (key == "field_names") {
                if (py::isinstance<py::none>(item.second)) {
                    continue;
                }
                options.field_names =
                    py::cast<std::vector<std::string>>(item.second);
            } else if (key == "no_auto_udp_dest") {
                options.no_auto_udp_dest = py::cast<bool>(item.second);
            } else if (key == "do_not_reinitialize") {
                options.do_not_reinitialize = py::cast<bool>(item.second);
            } else if (key == "timeout") {
                options.timeout = py::cast<float>(item.second);
            } else if (key == "sensor_info") {
                options.sensor_info =
                    py::cast<std::vector<ouster::sensor::sensor_info>>(
                        item.second);
            } else if (key == "sensor_config") {
                options.sensor_config =
                    py::cast<std::vector<ouster::sensor::sensor_config>>(
                        item.second);
            } else if (key == "extrinsics") {
                auto extrinsics =
                    py::cast<std::vector<py::array_t<double>>>(item.second);
                std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> exts;
                for (auto& pose : extrinsics) {
                    // Ensure the pose is a 4x4 matrix
                    if (pose.ndim() != 2 || pose.shape(0) != 4 ||
                        pose.shape(1) != 4) {
                        throw std::invalid_argument(
                            "Extrinsic matrix must have shape (4, 4)");
                    }

                    // Create a C-style copy of pose if it's neither C-style nor
                    // F-style const
                    const py::array_t<double>* pose_ptr = &pose;
                    py::array_t<double> c_style_pose;
                    if (!(pose.flags() & py::array::c_style)) {
                        c_style_pose =
                            py::array_t<double, py::array::c_style>(pose);
                        pose_ptr = &c_style_pose;  // Use the C-style array for
                                                   // processing
                    }

                    // Convert pose to Eigen format
                    Eigen::Map<
                        const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
                        pose_eigen(pose_ptr->data());
                    exts.push_back(pose_eigen);
                }
                options.extrinsics = exts;
            } else if (key == "extrinsics_file") {
                options.extrinsics_file = py::cast<std::string>(item.second);
            } else if (key == "index") {
                options.index = py::cast<bool>(item.second);
            } else if (key == "soft_id_check") {
                options.soft_id_check = py::cast<bool>(item.second);
            } else if (key == "config_timeout") {
                options.config_timeout = py::cast<float>(item.second);
            } else if (key == "queue_size") {
                options.queue_size = py::cast<unsigned int>(item.second);
            } else if (key == "raw_headers") {
                options.raw_headers = py::cast<bool>(item.second);
            } else if (key == "raw_fields") {
                options.raw_fields = py::cast<bool>(item.second);
            } else if (key == "meta") {
                options.meta = py::cast<std::vector<std::string>>(item.second);
            } else {
                // throw specific errors for deprecated/removed parameters
                if (key == "cycle") {
                    throw std::invalid_argument(
                        "Parameter 'cycle' is no longer supported. Please "
                        "provide "
                        "on_eof='loop' to SimpleViz constructor or manually "
                        "loop "
                        "instead.");
                } else if (key == "complete") {
                    throw std::invalid_argument(
                        "Parameter 'complete' is no longer supported. Please "
                        "check "
                        "for LidarScan.complete() explicitly instead.");
                }
                throw std::invalid_argument("Unknown parameter '" + key +
                                            "' for source.");
            }
        } catch (std::runtime_error& error) {
            throw std::invalid_argument("Invalid type for parameter '" + key +
                                        "'.");
        }
    }
}

PYBIND11_MODULE(_bindings, m) {
    m.doc() = R"(
    SDK bindings generated by pybind11.

    This module is generated directly from the C++ code and not meant to be used
    directly.
    )";

    auto client = m.def_submodule("client");
    init_client(client, m);
    auto pcap = m.def_submodule("pcap");
    init_pcap(pcap, m);
    auto osf = m.def_submodule("osf");
    init_osf(osf, m);
    auto viz = m.def_submodule("viz");
    init_viz(viz, m);
    auto mapping = m.def_submodule("mapping");
    init_mapping(mapping, m);
}

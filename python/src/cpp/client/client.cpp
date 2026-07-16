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

#include "ouster/sensor/client.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <pyerrors.h>
#include <warnings.h>

#include "client_common.h"
#include "common.h"
#include "eigen_dense.h"
#include "ouster/core/defaults.h"
#include "ouster/core/packet.h"
#include "ouster/core/profile_extension.h"
#include "ouster/core/types.h"
#include "ouster/sensor/sensor_http.h"
#include "ouster/sensor/sensor_packet_source.h"

using ouster::sdk::core::ImuPacket;
using ouster::sdk::core::LidarPacket;
using ouster::sdk::core::PacketType;
using ouster::sdk::core::SensorConfig;
using ouster::sdk::sensor::ClientEvent;
using ouster::sdk::sensor::Sensor;
using ouster::sdk::sensor::SensorHttp;

void init_client_client(py::module_& module, py::module_& /*unused*/) {
    py::exception<ouster::sdk::sensor::ClientError> client_error(module, "ClientError");

    // Derived exceptions. passing the base exception object as scope/parent
    py::exception<ouster::sdk::sensor::ClientTimeout>(module, "ClientTimeout", client_error);
    py::exception<ouster::sdk::sensor::ClientOverflow>(module, "ClientOverflow", client_error);

    // New Client
    py::enum_<ClientEvent::EventType>(module, "ClientEventType", py::is_arithmetic())
        .value("Error", ClientEvent::ERR)
        .value("Exit", ClientEvent::EXIT)
        .value("PollTimeout", ClientEvent::POLL_TIMEOUT)
        .value("Packet", ClientEvent::PACKET);

    py::class_<ClientEvent>(module, "ClientEvent")
        .def(py::init<>())
        .def_rw("source", &ClientEvent::source)
        .def_rw("type", &ClientEvent::type)
        .def("packet", [](ClientEvent& self) {
            if (self.type == ClientEvent::PACKET) {
                if (self.packet().type() == PacketType::Lidar) {
                    return py::cast(static_cast<LidarPacket&>(self.packet()));
                } else if (self.packet().type() == PacketType::Imu) {
                    return py::cast(static_cast<ImuPacket&>(self.packet()));
                }
            }
            throw std::runtime_error("No packet available in event.");
        });

    py::class_<Sensor>(module, "Sensor")
        .def(py::init<const std::string&, const SensorConfig&>(), py::arg("hostname"),
             py::arg("desired_config") = SensorConfig())
        .def(
            "fetch_metadata",
            [](Sensor& self, int timeout) { return self.fetch_metadata(timeout); },
            py::arg("timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("http_client", &Sensor::http_client)
        .def("desired_config", &Sensor::desired_config)
        .def("hostname", &Sensor::hostname);

    py::class_<SensorHttp>(module, "SensorHttp")
        .def("metadata", &SensorHttp::metadata,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("sensor_info", &SensorHttp::sensor_info,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("get_config_params", &SensorHttp::get_config_params, py::arg("active"),
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("set_config_param", &SensorHttp::set_config_param, py::arg("key"), py::arg("value"),
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("active_config_params", &SensorHttp::active_config_params,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("staged_config_params", &SensorHttp::staged_config_params,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("set_udp_dest_auto", &SensorHttp::set_udp_dest_auto,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("beam_intrinsics", &SensorHttp::beam_intrinsics,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("imu_intrinsics", &SensorHttp::imu_intrinsics,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("lidar_intrinsics", &SensorHttp::lidar_intrinsics,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("lidar_data_format", &SensorHttp::lidar_data_format,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("reinitialize", &SensorHttp::reinitialize,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("save_config_params", &SensorHttp::save_config_params,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("get_user_data", &SensorHttp::get_user_data,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        // TODO: get_user_data_and_policy is hard to bind, bind later if
        // needed
        .def("set_user_data", &SensorHttp::set_user_data, py::arg("data"),
             py::arg("keep_on_config_delete") = true,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("delete_user_data", &SensorHttp::delete_user_data,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("network", &SensorHttp::network,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("restart", &SensorHttp::restart,
             py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def(
            "set_static_ip",
            [](SensorHttp& self, const std::string& ip_address, int timeout_sec) {
                return self.set_static_ip(ip_address, timeout_sec);
            },
            py::arg("ip_address"), py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def(
            "set_static_ip",
            [](SensorHttp& self, const std::string& ip_address, const std::string& gateway_address,
               int timeout_sec) {
                return self.set_static_ip(ip_address, gateway_address, timeout_sec);
            },
            py::arg("ip_address"), py::arg("gateway_address"),
            py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("delete_static_ip", &SensorHttp::delete_static_ip,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("auto_detected_udp_dest", &SensorHttp::auto_detected_udp_dest,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("original_destination") = py::none())
        .def(
            "diagnostics_dump",
            [](SensorHttp& self, int timeout_sec) {
                auto vec = self.diagnostics_dump(timeout_sec);
                return py::bytes(reinterpret_cast<const char*>(vec.data()), vec.size());
            },
            py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def(
            "get_zone_monitor_config_zip",
            [](SensorHttp& self, bool staged, int timeout_sec) {
                auto vec = self.get_zone_monitor_config_zip(staged, timeout_sec);
                return py::bytes(reinterpret_cast<const char*>(vec.data()), vec.size());
            },
            py::arg("staged") = false, py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def(
            "set_zone_monitor_config_zip",
            // NOTE: zip_bytes has to be string for py::bytes conversion,
            // which turns into an ugly string -> vector<uint8>
            // conversion in this context but that happens once per
            // configuration and we get cleaner API -- Tim T.
            [](SensorHttp& self, const py::bytes& zip_bytes, int timeout_sec) {
                auto data = reinterpret_cast<const uint8_t*>(zip_bytes.c_str());
                size_t len = zip_bytes.size();

                std::vector<uint8_t> vec{data, data + len};
                self.set_zone_monitor_config_zip(vec, timeout_sec);
            },
            py::arg("zip_bytes"), py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("delete_zone_monitor_staged_config", &SensorHttp::delete_zone_monitor_staged_config,
             py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("apply_zone_monitor_staged_config_to_active",
             &SensorHttp::apply_zone_monitor_staged_config_to_active,
             py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("get_zone_monitor_live_ids", &SensorHttp::get_zone_monitor_live_ids,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("set_zone_monitor_live_ids", &SensorHttp::set_zone_monitor_live_ids,
             py::arg("zone_ids"), py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("firmware_version", [](SensorHttp& self) { return self.firmware_version(); })
        .def_static("get_firmware_version", &SensorHttp::get_firmware_version, py::arg("hostname"),
                    py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def_static("get_firmware_version_string", &SensorHttp::get_firmware_version_string,
                    py::arg("hostname"),
                    py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("hostname", &SensorHttp::hostname)
        .def_static(
            "create",
            [](const std::string& hostname, int timeout_sec) {
                return SensorHttp::create(hostname, timeout_sec);
            },
            py::arg("hostname"), py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS);

    module.def(
        "set_config",
        [](const std::string& hostname, const SensorConfig& config, bool persist,
           bool udp_dest_auto, bool force_reinit) {
            uint8_t config_flags = 0;
            if (persist) {
                config_flags |= ouster::sdk::sensor::CONFIG_PERSIST;
            }
            if (udp_dest_auto) {
                config_flags |= ouster::sdk::sensor::CONFIG_UDP_DEST_AUTO;
            }
            if (force_reinit) {
                config_flags |= ouster::sdk::sensor::CONFIG_FORCE_REINIT;
            }
            ouster::sdk::sensor::set_config(hostname, config, config_flags);
        },
        R"(
    Set sensor config parameters on sensor.

    Args:
        hostname (str): hostname of the sensor
        config (SensorConfig): config to set sensor parameters to
        persist (bool): persist parameters after sensor disconnection (default = False)
        udp_dest_auto (bool): automatically determine sender's IP at the time command was sent
            and set it as destination of UDP traffic. Function will error out if config has
            udp_dest member. (default = False)
        force_reinit (bool): forces the sensor to re-init during set_config even when config
            params have not changed. (default = False)
    )",
        py::arg("hostname"), py::arg("config"), py::arg("persist") = false,
        py::arg("udp_dest_auto") = false, py::arg("force_reinit") = false);

    module.def(
        "get_config",
        [](const std::string& hostname, bool active) {
            return ouster::sdk::sensor::get_config(hostname, active);
        },
        R"(
    Returns sensor config parameters as SensorConfig.

    Args:
        hostname: hostname of the sensor
        active: return active or staged sensor configuration
    )",
        py::arg("hostname"), py::arg("active") = true);

    module.def("set_http_api_headers", &ouster::sdk::sensor::set_http_api_headers,
               py::arg("headers"));

    module.def("set_http_api_prefix", &ouster::sdk::sensor::set_http_api_prefix, py::arg("prefix"));

    module.def("in_multicast", &ouster::sdk::sensor::in_multicast);

    module.attr("SHORT_HTTP_REQUEST_TIMEOUT_SECONDS") =
        py::int_(SHORT_HTTP_REQUEST_TIMEOUT_SECONDS);
    module.attr("LONG_HTTP_REQUEST_TIMEOUT_SECONDS") = py::int_(LONG_HTTP_REQUEST_TIMEOUT_SECONDS);
}

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
#include "ouster/packet.h"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pyerrors.h>
#include <warnings.h>

#include "client_common.h"
#include "ouster/impl/packet_writer.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster/zone_monitor.h"

namespace py = pybind11;
using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::DataFormat;
using ouster::sdk::core::ImuPacket;
using ouster::sdk::core::LidarPacket;
using ouster::sdk::core::LidarScan;
using ouster::sdk::core::Packet;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::PacketType;
using ouster::sdk::core::PacketValidationFailure;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::ZonePacket;
using ouster::sdk::core::impl::PacketWriter;

void init_client_packet(py::module& module, py::module& /*unused*/) {
    // clang-format off
    // Packet Format
    py::class_<PacketFormat, std::shared_ptr<PacketFormat>>(module, "PacketFormat")
        .def(py::init([](const SensorInfo& info) {
            return new PacketFormat(info);
        }))
        .def_static("from_metadata",
                    [](const SensorInfo& info) -> const PacketFormat& {
                        return ouster::sdk::core::get_format(info);
                    }, py::return_value_policy::reference)
        .def_static("from_info",
                    [](const SensorInfo& info) -> const PacketFormat& {
                        return ouster::sdk::core::get_format(info);
                    }, py::return_value_policy::reference)
        .def_static("from_data_format",
                    [](const DataFormat& format) -> const PacketFormat& {
                        return ouster::sdk::core::get_format(format);
                    }, py::return_value_policy::reference)
        .def_readonly("lidar_packet_size", &PacketFormat::lidar_packet_size)
        .def_readonly("imu_packet_size", &PacketFormat::imu_packet_size)
        .def_readonly("zone_packet_size", &PacketFormat::zone_packet_size)
        .def_readonly("udp_profile_lidar", &PacketFormat::udp_profile_lidar)
        .def_readonly("columns_per_packet", &PacketFormat::columns_per_packet)
        .def_readonly("pixels_per_column", &PacketFormat::pixels_per_column)
        .def_readonly("imu_measurements_per_packet",
                      &PacketFormat::imu_measurements_per_packet)
        .def_readonly("imu_packets_per_frame",
                      &PacketFormat::imu_packets_per_frame)
        .def_readonly("packet_header_size", &PacketFormat::packet_header_size)
        .def_readonly("col_header_size", &PacketFormat::col_header_size)
        .def_readonly("col_footer_size", &PacketFormat::col_footer_size)
        .def_readonly("col_size", &PacketFormat::col_size)
        .def_readonly("packet_footer_size", &PacketFormat::packet_footer_size)
        .def_readonly("max_frame_id", &PacketFormat::max_frame_id)

        .def("field_value_mask", &PacketFormat::field_value_mask)
        .def("field_bitness", &PacketFormat::field_bitness)

        .def("crc", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.crc(getptr(0, buf), buf.request().size);
        })

        .def("calculate_crc", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.calculate_crc(getptr(0, buf), buf.request().size);
        })

        .def("packet_type", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.packet_type(getptr(0, buf));
        })

        .def("frame_id", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.frame_id(getptr(0, buf));
        })

        .def("prod_sn", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.prod_sn(getptr(0, buf));
        })

        .def("init_id", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.init_id(getptr(0, buf));
        })

        .def("alert_flags", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.alert_flags(getptr(0, buf));
        })

        .def("countdown_thermal_shutdown", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.countdown_thermal_shutdown(getptr(0, buf));
        })

        .def("countdown_shot_limiting", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.countdown_shot_limiting(getptr(0, buf));
        })

        .def("thermal_shutdown", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.thermal_shutdown(getptr(0, buf));
        })

        .def("shot_limiting", [](PacketFormat& packet_format, py::buffer buf) {
            return packet_format.shot_limiting(getptr(0, buf));
        })

        // NOTE: keep_alive seems to be ignored without cpp_function wrapper
        .def_property_readonly("fields", py::cpp_function([](const PacketFormat& self) {
            return py::make_key_iterator(self.begin(), self.end());
        }, py::keep_alive<0, 1>()),
            "Return an iterator of available channel fields.")

        .def("packet_field", [](PacketFormat& packet_format, const std::string& field_name, py::buffer buf) -> py::array {
            auto buf_ptr = getptr(packet_format.lidar_packet_size, buf);

            auto packet_field = [&](auto& res) -> void {
                for (int icol = 0; icol < packet_format.columns_per_packet; icol++) {
                    auto col = packet_format.nth_col(icol, buf_ptr);
                    auto dst_col = res.mutable_data(0, icol);
                    packet_format.col_field(col, field_name, dst_col, packet_format.columns_per_packet);
                }
            };

            std::vector<size_t> dims{static_cast<size_t>(packet_format.pixels_per_column),
                                     static_cast<size_t>(packet_format.columns_per_packet)};

            switch (packet_format.field_type(field_name)) {
            case ChanFieldType::UINT8: {
                py::array_t<uint8_t> res(dims);
                packet_field(res);
                return std::move(res);
            }
            case ChanFieldType::UINT16: {
                py::array_t<uint16_t> res(dims);
                packet_field(res);
                return std::move(res);
            }
            case ChanFieldType::UINT32: {
                py::array_t<uint32_t> res(dims);
                packet_field(res);
                return std::move(res);
            }
            case ChanFieldType::UINT64: {
                py::array_t<uint64_t> res(dims);
                packet_field(res);
                return std::move(res);
            }
            default:
                throw py::key_error("Invalid type for PacketFormat");
            }
        })

        .def("packet_header", [](PacketFormat& packet_format, py::object header_index_obj, py::buffer buf) {

            auto packet_header = [&](auto&& header_func) -> py::array {
                using T = std::result_of_t<decltype(header_func)(const uint8_t*)>;

                auto packet_ptr = getptr(packet_format.lidar_packet_size, buf);
                auto res = py::array_t<T>(packet_format.columns_per_packet);

                for (int icol = 0; icol < packet_format.columns_per_packet; icol++) {
                    res.mutable_at(icol) = header_func(packet_format.nth_col(icol, packet_ptr));
                }

                return std::move(res);
            };

            auto ind = py::int_(header_index_obj).cast<int>();
            switch (ind) {
            case 0: return packet_header([&](auto col) { return packet_format.col_timestamp(col); });
            case 1: return packet_header([&](auto col) { return packet_format.col_encoder(col); });
            case 2: return packet_header([&](auto col) { return packet_format.col_measurement_id(col); });
            case 3: return packet_header([&](auto col) { return packet_format.col_status(col); });
            case 4: return packet_header([&](auto col) { return packet_format.col_frame_id(col); });
            default: throw py::key_error("Invalid header index for PacketFormat");
            }
        })

        // IMU packet accessors
        .def("imu_sys_ts", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_sys_ts(getptr(packet_format.imu_packet_size, buf)); })
        .def("imu_accel_ts", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_accel_ts(getptr(packet_format.imu_packet_size, buf)); })
        .def("imu_gyro_ts", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_gyro_ts(getptr(packet_format.imu_packet_size, buf)); })
        .def("imu_av_x", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_av_x(getptr(packet_format.imu_packet_size, buf)); })
        .def("imu_av_y", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_av_y(getptr(packet_format.imu_packet_size, buf)); })
        .def("imu_av_z", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_av_z(getptr(packet_format.imu_packet_size, buf)); })
        .def("imu_la_x", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_la_x(getptr(packet_format.imu_packet_size, buf)); })
        .def("imu_la_y", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_la_y(getptr(packet_format.imu_packet_size, buf)); })
        .def("imu_la_z", [](PacketFormat& packet_format, py::buffer buf) { return packet_format.imu_la_z(getptr(packet_format.imu_packet_size, buf)); });
    // TODO: new imu accessors && zone accessors

    // PacketWriter
    py::class_<PacketWriter, PacketFormat, std::shared_ptr<PacketWriter>>(module, "PacketWriter")
        // this is safe so long as PacketWriter does not carry any extra state
        .def_static("from_info",
                    [](const SensorInfo& info) -> const PacketWriter& {
                        const auto& packet_format = ouster::sdk::core::get_format(info);
                        return static_cast<const PacketWriter&>(packet_format);
                    }, py::return_value_policy::reference)
        .def_static("from_data_format",
                    [](const DataFormat& format) -> const PacketWriter& {
                        const auto& packet_format = ouster::sdk::core::get_format(format);
                        return static_cast<const PacketWriter&>(packet_format);
                    }, py::return_value_policy::reference)
        .def("set_col_status",
             [](const PacketWriter& self, LidarPacket& packet, int col_idx,
                uint32_t status) {
                 if (col_idx >= self.columns_per_packet) {
                     throw std::invalid_argument("col_idx out of bounds");
                 }
                 uint8_t* col_buf = self.nth_col(col_idx, packet.buf.data());
                 self.set_col_status(col_buf, status);
             })
        .def("set_col_timestamp",
             [](const PacketWriter& self, LidarPacket& packet, int col_idx,
                uint64_t timestamp) {
                 if (col_idx >= self.columns_per_packet) {
                     throw std::invalid_argument("col_idx out of bounds");
                 }
                 uint8_t* col_buf = self.nth_col(col_idx, packet.buf.data());
                 self.set_col_timestamp(col_buf, timestamp);
             })
        .def("set_col_measurement_id",
             [](const PacketWriter& self, LidarPacket& packet, int col_idx,
                uint16_t m_id) {
                 if (col_idx >= self.columns_per_packet) {
                     throw std::invalid_argument("col_idx out of bounds");
                 }
                 uint8_t* col_buf = self.nth_col(col_idx, packet.buf.data());
                 self.set_col_measurement_id(col_buf, m_id);
             })
        .def("set_frame_id",
             [](const PacketWriter& self, LidarPacket& packet, uint32_t frame_id) {
                 self.set_frame_id(packet.buf.data(), frame_id);
             })
        .def("set_alert_flags", [](const PacketWriter& self, LidarPacket& packet, uint8_t alert_flags) {
            self.set_alert_flags(packet.buf.data(), alert_flags);
        })
        .def("set_shutdown_countdown", [](const PacketWriter& self, LidarPacket& packet, uint8_t shutdown_countdown) {
            self.set_shutdown_countdown(packet.buf.data(), shutdown_countdown);
        })
        .def("set_shot_limiting_countdown", [](const PacketWriter& self, LidarPacket& packet, uint8_t shot_limiting_countdown) {
            self.set_shot_limiting_countdown(packet.buf.data(), shot_limiting_countdown);
        })
        .def("set_field", SetField<uint8_t>{})
        .def("set_field", SetField<uint16_t>{})
        .def("set_field", SetField<uint32_t>{})
        .def("set_field", SetField<uint64_t>{})
        .def("set_field", SetField<int8_t>{})
        .def("set_field", SetField<int16_t>{})
        .def("set_field", SetField<int32_t>{})
        .def("set_field", SetField<int64_t>{})
        .def("set_field", SetField<float>{})
        .def("set_field", SetField<double>{});

    module.def("scan_to_packets",
               [](const LidarScan& lidar_scan, const PacketWriter& packet_writer_inst, uint32_t init_id, uint64_t prod_sn) {
                   std::vector<py::object> packets;

                   auto append_pypacket = [&](Packet& packet) {
                       auto copy = new Packet(std::move(packet));
                       if (copy->type() == PacketType::Lidar) {
                           std::shared_ptr<LidarPacket> shared((LidarPacket*)copy);
                           packets.push_back(py::cast(shared));
                       } else if (copy->type() == PacketType::Zone) {
                           std::shared_ptr<ZonePacket> shared((ZonePacket*)copy);
                           packets.push_back(py::cast(shared));
                       } else {
                           std::shared_ptr<ImuPacket> shared((ImuPacket*)copy);
                           packets.push_back(py::cast(shared));
                       }
                   };

                   auto iter = make_lambda_iter(append_pypacket);
                   ouster::sdk::core::impl::scan_to_packets(lidar_scan, packet_writer_inst, iter, init_id, prod_sn);

                   return packets;
               });

    py::class_<Packet, std::shared_ptr<Packet>>(module, "Packet")
        // direct access to timestamp field
        .def_readwrite("host_timestamp", &Packet::host_timestamp)
        .def_readwrite("format", &Packet::format)
        .def("packet_type", &Packet::packet_type)
        .def("frame_id", &Packet::frame_id)
        .def("init_id", &Packet::init_id)
        .def("prod_sn", &Packet::prod_sn)
        .def("alert_flags", &Packet::alert_flags)
        .def("countdown_thermal_shutdown",
             &Packet::countdown_thermal_shutdown)
        .def("countdown_shot_limiting", &Packet::countdown_shot_limiting)
        .def("thermal_shutdown", &Packet::thermal_shutdown)
        .def("shot_limiting", &Packet::shot_limiting)
        .def("crc", &Packet::crc)
        .def("calculate_crc", &Packet::calculate_crc)
        .def_property_readonly("type",
                               [](const Packet& self) { return self.type(); })
        .def("__copy__", [](const Packet& self) { return Packet(self); })
        .def("__deepcopy__",
             [](const Packet& self, py::dict) { return Packet(self); })
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "buf",
            // we have to use cpp_function here because py::keep_alive
            // does not work with pybind11 def_property methods due to a
            // bug: https://github.com/pybind/pybind11/issues/4236
            py::cpp_function(
                [](Packet& self) {
                    return py::array(py::dtype::of<uint8_t>(), self.buf.size(),
                                     self.buf.data(), py::cast(self));
                },
                py::keep_alive<0, 1>()))
        .def("validate",
             [](const Packet& self, const SensorInfo& info,
                const PacketFormat& format) {
                 return self.validate(info, format);
             })
        .def("validate", [](const Packet& self, const SensorInfo& info) {
            return self.validate(info);
        });

    py::enum_<PacketType>(module, "PacketType", py::arithmetic())
        .value("Unknown", PacketType::Unknown)
        .value("Lidar", PacketType::Lidar)
        .value("Imu", PacketType::Imu)
        .value("Zone", PacketType::Zone);

    py::enum_<PacketValidationFailure>(module, "PacketValidationFailure",
                                               py::arithmetic())
        .value("NONE", PacketValidationFailure::NONE)
        .value("PACKET_SIZE", PacketValidationFailure::PACKET_SIZE)
        .value("ID", PacketValidationFailure::ID);

    py::class_<LidarPacket, Packet, std::shared_ptr<LidarPacket>>(module,
                                                                  "LidarPacket")
        .def(py::init<int>(), py::arg("size") = 65536)
        .def("__copy__",
             [](const LidarPacket& self) { return LidarPacket(self); })
        .def("__deepcopy__", [](const LidarPacket& self,
                                py::dict) { return LidarPacket(self); });

    py::class_<ImuPacket, Packet, std::shared_ptr<ImuPacket>>(module, "ImuPacket")
        .def(py::init<int>(), py::arg("size") = 65536)
        .def("__copy__", [](const ImuPacket& self) { return ImuPacket(self); })
        .def("__deepcopy__",
             [](const ImuPacket& self, py::dict) { return ImuPacket(self); })
        .def("sys_ts", &ImuPacket::sys_ts)
        .def("accel_ts", &ImuPacket::accel_ts)
        .def("gyro_ts", &ImuPacket::gyro_ts)
        .def("nmea_sentence", &ImuPacket::nmea_sentence)
        .def("nmea_ts", &ImuPacket::nmea_ts)
        .def("accel", &ImuPacket::accel)
        .def("gyro", &ImuPacket::gyro)
        .def("status", &ImuPacket::status)
        .def("timestamp", &ImuPacket::timestamp)
        .def("measurement_id", &ImuPacket::measurement_id)
        .def("la_x",
             [](const ImuPacket& self) {
                 PyErr_WarnEx(PyExc_DeprecationWarning,
                              "la_x is deprecated, use accel() instead", 2);
                 return self.la_x();
             })
        .def("la_y",
             [](const ImuPacket& self) {
                 PyErr_WarnEx(PyExc_DeprecationWarning,
                              "la_y is deprecated, use accel() instead", 2);
                 return self.la_y();
             })
        .def("la_z",
             [](const ImuPacket& self) {
                 PyErr_WarnEx(PyExc_DeprecationWarning,
                              "la_z is deprecated, use accel() instead", 2);
                 return self.la_z();
             })
        .def("av_x",
             [](const ImuPacket& self) {
                 PyErr_WarnEx(PyExc_DeprecationWarning,
                              "av_x is deprecated, use accel() instead", 2);
                 return self.av_x();
             })
        .def("av_y",
             [](const ImuPacket& self) {
                 PyErr_WarnEx(PyExc_DeprecationWarning,
                              "av_y is deprecated, use accel() instead", 2);
                 return self.av_y();
             })
        .def("av_z",
             [](const ImuPacket& self) {
                 PyErr_WarnEx(PyExc_DeprecationWarning,
                              "av_z is deprecated, use accel() instead", 2);
                 return self.av_z();
             });
}

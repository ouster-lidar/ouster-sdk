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
#include "ouster/core/packet.h"

#include <nanobind/make_iterator.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_map.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <pyerrors.h>
#include <warnings.h>

#include "client_common.h"
#include "common.h"  // NOLINT(unused-includes)
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/core/zone_monitor.h"

using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::DataFormat;
using ouster::sdk::core::ImuPacket;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::LidarPacket;
using ouster::sdk::core::Packet;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::PacketType;
using ouster::sdk::core::PacketValidationFailure;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::ZonePacket;

void init_client_packet(py::module_& module, py::module_& /*unused*/) {
    // Packet Format
    py::class_<PacketFormat>(module, "PacketFormat")
        .def(py::init<const SensorInfo&>())
        .def(py::init<const DataFormat&>())
        .def_static(
            "from_metadata",
            [](const SensorInfo& info) -> const PacketFormat& {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "from_metadata is deprecated, use from_info instead", 1);
                return ouster::sdk::core::get_format(info);
            },
            py::rv_policy::reference)
        .def_static(
            "from_info",
            [](const SensorInfo& info) -> const PacketFormat& {
                return ouster::sdk::core::get_format(info);
            },
            py::rv_policy::reference)
        .def_static(
            "from_data_format",
            [](const DataFormat& format) -> const PacketFormat& {
                return ouster::sdk::core::get_format(format);
            },
            py::rv_policy::reference)
        .def_ro("lidar_packet_size", &PacketFormat::lidar_packet_size)
        .def_ro("imu_packet_size", &PacketFormat::imu_packet_size)
        .def_ro("zone_packet_size", &PacketFormat::zone_packet_size)
        .def_ro("udp_profile_lidar", &PacketFormat::udp_profile_lidar)
        .def_ro("columns_per_packet", &PacketFormat::columns_per_packet)
        .def_ro("pixels_per_column", &PacketFormat::pixels_per_column)
        .def_ro("imu_measurements_per_packet", &PacketFormat::imu_measurements_per_packet)
        .def_ro("imu_packets_per_frame", &PacketFormat::imu_packets_per_frame)
        .def_ro("packet_header_size", &PacketFormat::packet_header_size)
        .def_ro("col_header_size", &PacketFormat::col_header_size)
        .def_ro("col_footer_size", &PacketFormat::col_footer_size)
        .def_ro("col_size", &PacketFormat::col_size)
        .def_ro("packet_footer_size", &PacketFormat::packet_footer_size)
        .def_ro("max_frame_id", &PacketFormat::max_frame_id)

        .def("field_value_mask", &PacketFormat::field_value_mask)
        .def("field_bitness", &PacketFormat::field_bitness)

        .def(
            "crc",
            [](PacketFormat& packet_format, const py::object& buf) {
                auto arr = py::cast<py::ndarray<const uint8_t, py::ndim<1>, py::c_contig>>(buf);
                return packet_format.crc(getptr(0, buf), arr.size());
            },
            py::sig("def crc(self, buf: BufferT) -> typing.Optional[int]"))

        .def(
            "calculate_crc",
            [](PacketFormat& packet_format, const py::object& buf) {
                auto arr = py::cast<py::ndarray<const uint8_t, py::ndim<1>, py::c_contig>>(buf);
                return packet_format.calculate_crc(getptr(0, buf), arr.size());
            },
            py::sig("def calculate_crc(self, buf: BufferT) -> int"))

        .def(
            "packet_type",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.packet_type(getptr(0, buf));
            },
            py::sig("def packet_type(self, buf: BufferT) -> int"))

        .def(
            "frame_id",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.frame_id(getptr(0, buf));
            },
            py::sig("def frame_id(self, buf: BufferT) -> int"))

        .def(
            "prod_sn",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.prod_sn(getptr(0, buf));
            },
            py::sig("def prod_sn(self, buf: BufferT) -> int"))

        .def(
            "init_id",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.init_id(getptr(0, buf));
            },
            py::sig("def init_id(self, buf: BufferT) -> int"))

        .def(
            "alert_flags",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.alert_flags(getptr(0, buf));
            },
            py::sig("def alert_flags(self, buf: BufferT) -> int"))

        .def(
            "countdown_thermal_shutdown",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.countdown_thermal_shutdown(getptr(0, buf));
            },
            py::sig("def countdown_thermal_shutdown(self, buf: BufferT) -> int"))

        .def(
            "countdown_shot_limiting",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.countdown_shot_limiting(getptr(0, buf));
            },
            py::sig("def countdown_shot_limiting(self, buf: BufferT) -> int"))

        .def(
            "thermal_shutdown",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.thermal_shutdown(getptr(0, buf));
            },
            py::sig("def thermal_shutdown(self, buf: BufferT) -> int"))

        .def(
            "shot_limiting",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.shot_limiting(getptr(0, buf));
            },
            py::sig("def shot_limiting(self, buf: BufferT) -> int"))

        // NOTE: keep_alive seems to be ignored without cpp_function wrapper
        .def_prop_ro(
            "fields",
            [](const PacketFormat& self) {
                return py::make_key_iterator(py::type<PacketFormat>(), "iterator", self.begin(),
                                             self.end());
            },
            py::keep_alive<0, 1>(), "Return an iterator of available channel fields.")

        .def(
            "packet_field",
            [](PacketFormat& packet_format, const std::string& field_name, const py::object& buf) {
                auto buf_ptr = getptr(packet_format.lidar_packet_size, buf);

                auto make_packet_field = [&](auto dummy_type) -> py::object {
                    using T = decltype(dummy_type);
                    size_t shape[2] = {static_cast<size_t>(packet_format.pixels_per_column),
                                       static_cast<size_t>(packet_format.columns_per_packet)};

                    T* data = new T[shape[0] * shape[1]];
                    py::capsule owner(data,
                                      [](void* ptr) noexcept { delete[] static_cast<T*>(ptr); });
                    py::ndarray<T, py::numpy, py::shape<-1, -1>> res(data, 2, shape, owner);

                    for (uint32_t icol = 0; icol < packet_format.columns_per_packet; icol++) {
                        auto col = packet_format.nth_col(icol, buf_ptr);
                        T* dst_col = res.data() + icol;
                        packet_format.col_field(col, field_name, dst_col,
                                                packet_format.columns_per_packet);
                    }
                    return py::cast(res);
                };

                switch (packet_format.field_type(field_name)) {
                    case ChanFieldType::UINT8:
                        return make_packet_field(uint8_t{});
                    case ChanFieldType::UINT16:
                        return make_packet_field(uint16_t{});
                    case ChanFieldType::UINT32:
                        return make_packet_field(uint32_t{});
                    case ChanFieldType::UINT64:
                        return make_packet_field(uint64_t{});
                    default:
                        throw py::key_error("Invalid type for PacketFormat");
                }
            },
            py::sig("def packet_field(self, name: str, buf: NDArray, /) -> "
                    "NDArray"))

        .def(
            "packet_header",
            [](PacketFormat& packet_format, const py::object& header_index_obj,
               const py::object& buf) {
                auto packet_header = [&](auto&& header_func) -> py::object {
                    using T = std::result_of_t<decltype(header_func)(const uint8_t*)>;

                    auto packet_ptr = getptr(packet_format.lidar_packet_size, buf);

                    size_t shape[1] = {static_cast<size_t>(packet_format.columns_per_packet)};
                    T* data = new T[shape[0]];
                    py::capsule owner(data,
                                      [](void* ptr) noexcept { delete[] static_cast<T*>(ptr); });
                    py::ndarray<T, py::numpy, py::shape<-1>> res(data, 1, shape, owner);

                    for (uint32_t icol = 0; icol < packet_format.columns_per_packet; icol++) {
                        res.data()[icol] = header_func(packet_format.nth_col(icol, packet_ptr));
                    }
                    return py::cast(res);
                };

                auto ind = py::cast<int>(header_index_obj);
                switch (ind) {
                    case 0:
                        return packet_header(
                            [&](auto col) { return packet_format.col_timestamp(col); });
                    case 1:
                        return packet_header(
                            [&](auto col) { return packet_format.col_encoder(col); });
                    case 2:
                        return packet_header(
                            [&](auto col) { return packet_format.col_measurement_id(col); });
                    case 3:
                        return packet_header(
                            [&](auto col) { return packet_format.col_status(col); });
                    case 4:
                        return packet_header(
                            [&](auto col) { return packet_format.col_frame_id(col); });
                    default:
                        throw py::key_error("Invalid header index for PacketFormat");
                }
            },
            py::sig("def packet_header(self, header_type: object, buf: "
                    "NDArray, /) -> NDArray"))

        // IMU packet accessors
        .def(
            "imu_sys_ts",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_sys_ts(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_sys_ts(self, buf: BufferT) -> int"))
        .def(
            "imu_accel_ts",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_accel_ts(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_accel_ts(self, buf: BufferT) -> int"))
        .def(
            "imu_gyro_ts",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_gyro_ts(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_gyro_ts(self, buf: BufferT) -> int"))
        .def(
            "imu_av_x",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_av_x(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_av_x(self, buf: BufferT) -> float"))
        .def(
            "imu_av_y",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_av_y(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_av_y(self, buf: BufferT) -> float"))
        .def(
            "imu_av_z",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_av_z(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_av_z(self, buf: BufferT) -> float"))
        .def(
            "imu_la_x",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_la_x(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_la_x(self, buf: BufferT) -> float"))
        .def(
            "imu_la_y",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_la_y(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_la_y(self, buf: BufferT) -> float"))
        .def(
            "imu_la_z",
            [](PacketFormat& packet_format, const py::object& buf) {
                return packet_format.imu_la_z(getptr(packet_format.imu_packet_size, buf));
            },
            py::sig("def imu_la_z(self, buf: BufferT) -> float"))
        .def("set_col_status",
             [](const PacketFormat& self, LidarPacket& packet, size_t col_idx, uint32_t status) {
                 if (col_idx >= self.columns_per_packet) {
                     throw std::invalid_argument("col_idx out of bounds");
                 }
                 uint8_t* col_buf = self.nth_col(col_idx, packet.buf.data());
                 self.set_col_status(col_buf, status);
             })
        .def("set_col_timestamp",
             [](const PacketFormat& self, LidarPacket& packet, size_t col_idx, uint64_t timestamp) {
                 if (col_idx >= self.columns_per_packet) {
                     throw std::invalid_argument("col_idx out of bounds");
                 }
                 uint8_t* col_buf = self.nth_col(col_idx, packet.buf.data());
                 self.set_col_timestamp(col_buf, timestamp);
             })
        .def("set_col_measurement_id",
             [](const PacketFormat& self, LidarPacket& packet, size_t col_idx, uint16_t m_id) {
                 if (col_idx >= self.columns_per_packet) {
                     throw std::invalid_argument("col_idx out of bounds");
                 }
                 uint8_t* col_buf = self.nth_col(col_idx, packet.buf.data());
                 self.set_col_measurement_id(col_buf, m_id);
             })
        .def("set_frame_id",
             [](const PacketFormat& self, LidarPacket& packet, uint32_t frame_id) {
                 self.set_frame_id(packet.buf.data(), frame_id);
             })
        .def("set_frame_id",
             [](const PacketFormat& self, ImuPacket& packet, uint32_t frame_id) {
                 self.set_frame_id(packet.buf.data(), frame_id);
             })
        .def("set_alert_flags",
             [](const PacketFormat& self, LidarPacket& packet, uint8_t alert_flags) {
                 self.set_alert_flags(packet.buf.data(), alert_flags);
             })
        .def("set_shutdown_countdown",
             [](const PacketFormat& self, LidarPacket& packet, uint8_t shutdown_countdown) {
                 self.set_shutdown_countdown(packet.buf.data(), shutdown_countdown);
             })
        .def("set_shot_limiting_countdown",
             [](const PacketFormat& self, LidarPacket& packet, uint8_t shot_limiting_countdown) {
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

    module.def(
        "frame_to_packets",
        [](const LidarFrame& lidar_frame, const std::shared_ptr<PacketFormat>& packet_format,
           uint32_t init_id, uint64_t prod_sn) {
            std::vector<py::object> packets;

            auto append_pypacket = [&](Packet& packet) {
                auto copy = new Packet(std::move(packet));
                if (copy->type() == PacketType::Lidar) {
                    std::shared_ptr<LidarPacket> shared(
                        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
                        static_cast<LidarPacket*>(copy));
                    packets.push_back(py::cast(shared));
                } else if (copy->type() == PacketType::Zone) {
                    std::shared_ptr<ZonePacket> shared(
                        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
                        static_cast<ZonePacket*>(copy));
                    packets.push_back(py::cast(shared));
                } else {
                    std::shared_ptr<ImuPacket> shared(
                        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
                        static_cast<ImuPacket*>(copy));
                    packets.push_back(py::cast(shared));
                }
            };

            auto iter = make_lambda_iter(append_pypacket);
            ouster::sdk::core::impl::frame_to_packets(lidar_frame, packet_format, iter, init_id,
                                                      prod_sn);

            return packets;
        },
        py::sig("def frame_to_packets(lidar_frame: LidarFrame, packet_format: "
                "PacketFormat, init_id: int, prod_sn: int) -> "
                "List[Union[LidarPacket, ImuPacket, ZonePacket]]"));

    py::class_<Packet>(module, "Packet")
        // direct access to timestamp field
        .def_rw("host_timestamp", &Packet::host_timestamp)
        .def_rw("format", &Packet::format)
        .def("packet_type", &Packet::packet_type)
        .def("frame_id", &Packet::frame_id)
        .def("init_id", &Packet::init_id)
        .def("prod_sn", &Packet::prod_sn)
        .def("alert_flags", &Packet::alert_flags)
        .def("countdown_thermal_shutdown", &Packet::countdown_thermal_shutdown)
        .def("countdown_shot_limiting", &Packet::countdown_shot_limiting)
        .def("thermal_shutdown", &Packet::thermal_shutdown)
        .def("shot_limiting", &Packet::shot_limiting)
        .def("crc", &Packet::crc)
        .def("calculate_crc", &Packet::calculate_crc)
        .def_prop_ro("type", [](const Packet& self) { return self.type(); })
        .def("__copy__", [](const Packet& self) { return Packet(self); })
        .def("__deepcopy__", [](const Packet& self, const py::dict&) { return Packet(self); })
        // NOTE: returned array is writeable, but not reassignable
        .def_prop_ro(
            "buf",
            [](Packet& self) {
                // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays)
                size_t shape[1] = {self.buf.size()};
                return py::ndarray<uint8_t, py::numpy, py::shape<-1>>(self.buf.data(), 1, shape,
                                                                      py::handle());
            },
            py::keep_alive<0, 1>())
        .def("validate", [](const Packet& self, const SensorInfo& info,
                            const PacketFormat& format) { return self.validate(info, format); })
        .def("validate",
             [](const Packet& self, const SensorInfo& info) { return self.validate(info); });

    py::enum_<PacketType>(module, "PacketType", py::is_arithmetic())
        .value("Unknown", PacketType::Unknown)
        .value("Lidar", PacketType::Lidar)
        .value("Imu", PacketType::Imu)
        .value("Zone", PacketType::Zone);

    py::enum_<PacketValidationFailure>(module, "PacketValidationFailure", py::is_arithmetic())
        .value("NONE", PacketValidationFailure::NONE)
        .value("PACKET_SIZE", PacketValidationFailure::PACKET_SIZE)
        .value("ID", PacketValidationFailure::ID);

    py::class_<LidarPacket, Packet>(module, "LidarPacket")
        .def(
            "__init__",
            [](LidarPacket* self, int size = 65536) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "LidarPacket(size: int) is deprecated, use "
                             "LidarPacket(fmt: PacketFormat) instead",
                             1);
                new (self) LidarPacket(size);
            },
            py::arg("size") = 65536)
        .def(py::init<std::shared_ptr<PacketFormat>>())
        .def("__copy__", [](const LidarPacket& self) { return LidarPacket(self); })
        .def("__deepcopy__",
             [](const LidarPacket& self, const py::dict&) { return LidarPacket(self); });

    py::class_<ImuPacket, Packet>(module, "ImuPacket")
        .def(
            "__init__",
            [](ImuPacket* self, int size = 65536) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "ImuPacket(size: int) is deprecated, use "
                             "ImuPacket(fmt: PacketFormat) instead",
                             1);
                new (self) ImuPacket(size);
            },
            py::arg("size") = 65536)
        .def(py::init<std::shared_ptr<PacketFormat>>())
        .def("__copy__", [](const ImuPacket& self) { return ImuPacket(self); })
        .def("__deepcopy__",
             [](const ImuPacket& self, const py::dict& /*unused*/) { return ImuPacket(self); })
        .def("sys_ts", &ImuPacket::sys_ts)
        .def("accel_ts", &ImuPacket::accel_ts)
        .def("gyro_ts", &ImuPacket::gyro_ts)
        .def("nmea_sentence", &ImuPacket::nmea_sentence)
        .def("nmea_ts", &ImuPacket::nmea_ts)
        .def("accel", &ImuPacket::accel)
        .def("gyro", &ImuPacket::gyro)
        .def("status", &ImuPacket::status)
        .def("timestamp", &ImuPacket::timestamp)
        .def("measurement_id", &ImuPacket::measurement_id);
}

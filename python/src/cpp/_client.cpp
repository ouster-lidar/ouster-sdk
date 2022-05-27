/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief ouster_pyclient python module
 *
 * Note: the type annotations in `client.pyi` need to be updated whenever this
 * file changes. See the mypy documentation for details.
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include "ouster/buffered_udp_source.h"
#include "ouster/client.h"
#include "ouster/image_processing.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

namespace py = pybind11;
namespace chrono = std::chrono;

using ouster::sensor::data_format;
using ouster::sensor::packet_format;
using ouster::sensor::sensor_config;
using ouster::sensor::sensor_info;
using ouster::sensor::impl::BufferedUDPSource;
using namespace ouster;

namespace pybind11 {
namespace detail {
template <typename T>
struct type_caster<nonstd::optional<T>> : optional_caster<nonstd::optional<T>> {
};
}  // namespace detail
}  // namespace pybind11

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

namespace ouster {
namespace sensor {
namespace impl {

extern const Table<lidar_mode, const char*, 6> lidar_mode_strings;
extern const Table<timestamp_mode, const char*, 4> timestamp_mode_strings;
extern const Table<OperatingMode, const char*, 2> operating_mode_strings;
extern const Table<MultipurposeIOMode, const char*, 6>
    multipurpose_io_mode_strings;
extern const Table<Polarity, const char*, 2> polarity_strings;
extern const Table<NMEABaudRate, const char*, 2> nmea_baud_rate_strings;
extern Table<ChanField, const char*, 13> chanfield_strings;
extern Table<UDPProfileLidar, const char*, 4> udp_profile_lidar_strings;
extern Table<UDPProfileIMU, const char*, 1> udp_profile_imu_strings;

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

// alias for non-casting row-major array arguments
template <typename T>
using pyimg_t = py::array_t<T, py::array::c_style>;

// factor out overloaded call operator for ae/buc
template <typename T, typename U>
void image_proc_call(T& self, pyimg_t<U> image, bool update_state) {
    if (image.ndim() != 2) throw std::invalid_argument("Expected a 2d array");
    self(Eigen::Map<img_t<U>>(image.mutable_data(), image.shape(0),
                              image.shape(1)),
         update_state);
}

/*
 * Define an enum from a table of strings, along with some properties to make
 * the class behave more like a Python enum
 */
template <typename C, typename E, size_t N>
void def_enum(C& Enum, const Table<E, const char*, N>& strings_table,
              const std::string& enum_prefix = "") {
    // in pybind11 2.0, calling enum.value(const char* name, val) doesn't make a
    // copy of the name argument. When value names aren't statically allocated,
    // we have to keep them alive. Use deque for stability of c_str() pointers
    static std::deque<std::string> enumerator_names;

    // module imports
    py::object MappingProxy =
        py::module::import("types").attr("MappingProxyType");

    // declare enumerators
    for (const auto& p : strings_table) {
        enumerator_names.push_back(enum_prefix + p.second);
        Enum.value(enumerator_names.back().c_str(), p.first);
    }

    // use immutable MappingProxy to return members dict
    std::map<std::string, E> members;
    for (const auto& p : strings_table) members[p.second] = p.first;
    py::object py_members = MappingProxy(members);
    Enum.def_property_readonly_static(
        "__members__", [=](py::object) { return py_members; },
        "Returns a mapping of member name->value.");

    // can't make the class iterable itself easily
    Enum.def_property_readonly_static(
        "values",
        [&](py::object) {
            return py::make_key_iterator(strings_table.begin(),
                                         strings_table.end());
        },
        "Returns an iterator of all enum members.");

    // support name / value properties like regular enums
    Enum.def_property_readonly(
        "value", [](const E& self) { return static_cast<int>(self); },
        "The value of the Enum member.");
    Enum.def_property_readonly(
        "name", [](const E& self) { return to_string(self); },
        "The name of the Enum member.");
    Enum.attr("__str__") =
        py::cpp_function([](const E& u) { return to_string(u); },
                         py::name("__str__"), py::is_method(Enum));

    Enum.def_static(
        "from_string",
        [=](const std::string& s) {
            return members.count(s) > 0 ? py::cast(members.at(s)) : py::none();
        },
        "Create enum value from string.");
}

/*
 * Check that buffer is a 1-d byte array of size > bound and return an internal
 * pointer to the data for writing. Check is strictly greater to account for the
 * extra byte required to determine if a datagram is bigger than expected.
 */
inline uint8_t* getptr(size_t bound, py::buffer& buf) {
    auto info = buf.request();
    if (info.format != py::format_descriptor<uint8_t>::format() ||
        // type of info.size has changed from size_t to ssize_t
        info.ndim != 1 || info.size < static_cast<decltype(info.size)>(bound)) {
        throw std::invalid_argument(
            "Incompatible argument: expected a bytearray of size >= " +
            std::to_string(bound));
    }
    return (uint8_t*)info.ptr;
}

/*
 * Map a dtype to a channel field type
 */
static sensor::ChanFieldType field_type_of_dtype(const py::dtype& dt) {
    if (dt == py::dtype::of<uint8_t>())
        return sensor::ChanFieldType::UINT8;
    else if (dt == py::dtype::of<uint16_t>())
        return sensor::ChanFieldType::UINT16;
    else if (dt == py::dtype::of<uint32_t>())
        return sensor::ChanFieldType::UINT32;
    else if (dt == py::dtype::of<uint64_t>())
        return sensor::ChanFieldType::UINT64;
    else
        throw std::invalid_argument("Invalid dtype for a channel field");
}

PYBIND11_PLUGIN(_client) {
    py::module m("_client", R"(
    Sensor client bindings generated by pybind11.

    This module is generated directly from the C++ code and not meant to be used
    directly.
    )");

    // turn off signatures in docstrings: mypy stubs provide better types
    py::options options;
    options.disable_function_signatures();

    // clang-format off

    // Packet Format
    py::class_<packet_format>(m, "PacketFormat")
        .def_static("from_info", [](const sensor_info& info) -> const packet_format& {
            return sensor::get_format(info);
        }, py::return_value_policy::reference)
        .def_readonly("lidar_packet_size", &packet_format::lidar_packet_size)
        .def_readonly("imu_packet_size", &packet_format::imu_packet_size)
        .def_readonly("columns_per_packet", &packet_format::columns_per_packet)
        .def_readonly("pixels_per_column", &packet_format::pixels_per_column)

        .def("packet_type", [](packet_format& pf, py::buffer buf) {
            return pf.packet_type(getptr(pf.lidar_packet_size, buf));
        })

        .def("frame_id", [](packet_format& pf, py::buffer buf) {
            return pf.frame_id(getptr(pf.lidar_packet_size, buf));
        })

        .def("prod_sn", [](packet_format& pf, py::buffer buf) {
            return pf.prod_sn(getptr(pf.lidar_packet_size, buf));
        })

        .def("init_id", [](packet_format& pf, py::buffer buf) {
            return pf.init_id(getptr(pf.lidar_packet_size, buf));
        })

        // NOTE: keep_alive seems to be ignored without cpp_function wrapper
        .def_property_readonly("fields", py::cpp_function([](const packet_format& self) {
                return py::make_key_iterator(self.begin(), self.end());
        }, py::keep_alive<0, 1>()),
        "Return an iterator of available channel fields.")

        .def("packet_field", [](packet_format& pf, sensor::ChanField f, py::buffer buf) -> py::array {
            auto buf_ptr = getptr(pf.lidar_packet_size, buf);

            auto packet_field = [&](auto& res) -> void {
                for (int icol = 0; icol < pf.columns_per_packet; icol++) {
                    auto col = pf.nth_col(icol, buf_ptr);
                    auto dst_col = res.mutable_data(0, icol);
                    pf.col_field(col, f, dst_col, pf.columns_per_packet);
                }
            };

            std::vector<size_t> dims{static_cast<size_t>(pf.pixels_per_column),
                                     static_cast<size_t>(pf.columns_per_packet)};

            switch (pf.field_type(f)) {
                case sensor::ChanFieldType::UINT8: {
                    py::array_t<uint8_t> res(dims);
                    packet_field(res);
                    return std::move(res);
                }
                case sensor::ChanFieldType::UINT16: {
                    py::array_t<uint16_t> res(dims);
                    packet_field(res);
                    return std::move(res);
                }
                case sensor::ChanFieldType::UINT32: {
                    py::array_t<uint32_t> res(dims);
                    packet_field(res);
                    return std::move(res);
                }
                case sensor::ChanFieldType::UINT64: {
                    py::array_t<uint64_t> res(dims);
                    packet_field(res);
                    return std::move(res);
                }
                default:
                    throw py::key_error("Invalid field for PacketFormat");
            }
        })

        .def("packet_header", [](packet_format& pf, py::object o, py::buffer buf) {

            auto packet_header = [&](auto&& f) -> py::array {
                using T = typename std::result_of<decltype(f)(const uint8_t*)>::type;

                auto p = getptr(pf.lidar_packet_size, buf);
                auto res = py::array_t<T>(pf.columns_per_packet);

                for (int icol = 0; icol < pf.columns_per_packet; icol++)
                    res.mutable_at(icol) = f(pf.nth_col(icol, p));

                return std::move(res);
            };

            auto ind = py::int_(o).cast<int>();
            switch (ind) {
                case 0: return packet_header([&](auto col) { return pf.col_timestamp(col); });
                case 1: return packet_header([&](auto col) { return pf.col_encoder(col); });
                case 2: return packet_header([&](auto col) { return pf.col_measurement_id(col); });
                case 3: return packet_header([&](auto col) { return pf.col_status(col); });
                case 4: return packet_header([&](auto col) { return pf.col_frame_id(col); });
                default: throw py::key_error("Invalid header index for PacketFormat");
            }
        })

        // IMU packet accessors
        .def("imu_sys_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_sys_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_accel_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_accel_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_gyro_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_gyro_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_x", [](packet_format& pf, py::buffer buf) { return pf.imu_av_x(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_y", [](packet_format& pf, py::buffer buf) { return pf.imu_av_y(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_z", [](packet_format& pf, py::buffer buf) { return pf.imu_av_z(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_x", [](packet_format& pf, py::buffer buf) { return pf.imu_la_x(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_y", [](packet_format& pf, py::buffer buf) { return pf.imu_la_y(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_z", [](packet_format& pf, py::buffer buf) { return pf.imu_la_z(getptr(pf.imu_packet_size, buf)); });

    // Data Format
    py::class_<data_format>(m, "DataFormat")
        .def(py::init<>())
        .def_readwrite("pixels_per_column", &data_format::pixels_per_column)
        .def_readwrite("columns_per_packet", &data_format::columns_per_packet)
        .def_readwrite("columns_per_frame", &data_format::columns_per_frame)
        .def_readwrite("pixel_shift_by_row", &data_format::pixel_shift_by_row)
        .def_readwrite("column_window", &data_format::column_window)
        .def_readwrite("udp_profile_lidar", &data_format::udp_profile_lidar)
        .def_readwrite("udp_profile_imu", &data_format::udp_profile_imu);

    // Sensor Info
    py::class_<sensor_info>(m, "SensorInfo", R"(
        Sensor Info required to interpret UDP data streams.

        See the sensor documentation for the meaning of each property.
        )")
        .def(py::init<>(), R"(
        Args:
            raw (str): json string to parse
        )")
        .def("__init__", [](sensor_info& self, const std::string& s) {
            new (&self) sensor_info{};
            self = sensor::parse_metadata(s);
        })
        .def_readwrite("hostname", &sensor_info::name, "Sensor hostname.")
        .def_readwrite("sn", &sensor_info::sn, "Sensor serial number.")
        .def_readwrite("fw_rev", &sensor_info::fw_rev, "Sensor firmware revision.")
        .def_readwrite("mode", &sensor_info::mode, "Lidar mode, e.g., 1024x10.")
        .def_readwrite("prod_line", &sensor_info::prod_line, "Product line, e.g., 'OS-1-128'.")
        .def_readwrite("format", &sensor_info::format,  "Describes the structure of a lidar packet.")
        .def_readwrite("beam_azimuth_angles", &sensor_info::beam_azimuth_angles, "Beam azimuth angles, useful for XYZ projection.")
        .def_readwrite("beam_altitude_angles", &sensor_info::beam_altitude_angles, "Beam altitude angles, useful for XYZ projection.")
        .def_readwrite("imu_to_sensor_transform", &sensor_info::imu_to_sensor_transform, "Homogenous transformation matrix representing IMU offset to Sensor Coordinate Frame.")
        .def_readwrite("lidar_to_sensor_transform", &sensor_info::lidar_to_sensor_transform, "Homogeneous transformation matrix from Lidar Coordinate Frame to Sensor Coordinate Frame.")
        .def_readwrite("lidar_origin_to_beam_origin_mm", &sensor_info::lidar_origin_to_beam_origin_mm, "Distance between lidar origin and beam origin in millimeters.")
        .def_readwrite("extrinsic", &sensor_info::extrinsic, "Extrinsic Matrix.")
        .def_readwrite("init_id", &sensor_info::init_id, "Initialization id.")
        .def_readwrite("udp_port_lidar", &sensor_info::udp_port_lidar, "Configured port for lidar data.")
        .def_readwrite("udp_port_imu", &sensor_info::udp_port_imu, "Configured port for imu data.")
        .def_static("from_default", &sensor::default_sensor_info, R"(
        Create gen-1 OS-1-64 SensorInfo populated with design values.
        )")
        .def("__eq__", [](const sensor_info& i, const sensor_info& j) { return i == j; })
        .def("__repr__", [](const sensor_info& self) {
            return "<ouster.client.SensorInfo " + self.prod_line + " " +
                self.sn + " " + self.fw_rev + " " + to_string(self.mode) + ">";
        })
        .def("__copy__", [](const sensor_info& self) { return sensor_info{self}; })
        .def("__deepcopy__", [](const sensor_info& self, py::dict) { return sensor_info{self}; });


    // Enums
    auto lidar_mode = py::enum_<sensor::lidar_mode>(m, "LidarMode", R"(
        Possible Lidar Modes of sensor.

        Determines to horizontal and vertical resolution rates of sensor. See
        sensor documentation for details.)", py::metaclass());
    def_enum(lidar_mode, sensor::impl::lidar_mode_strings, "MODE_");
    lidar_mode.def_property_readonly("cols", [](const sensor::lidar_mode& self) {
        return sensor::n_cols_of_lidar_mode(self); },
        "Returns columns of lidar mode, e.g., 1024 for LidarMode 1024x10.");
    lidar_mode.def_property_readonly("frequency", [](const sensor::lidar_mode& self) {
        return sensor::frequency_of_lidar_mode(self); },
        "Returns frequency of lidar mode, e.g. 10 for LidarMode 512x10.");
    // TODO: this is wacky
    lidar_mode.value("MODE_UNSPEC", sensor::lidar_mode::MODE_UNSPEC);
    lidar_mode.attr("from_string") = py::cpp_function([](const std::string& s) {
        return sensor::lidar_mode_of_string(s); }, py::name("from_string"), "Create LidarMode from string.");

    auto timestamp_mode = py::enum_<sensor::timestamp_mode>(m, "TimestampMode", R"(
        Possible Timestamp modes of sensor.

        See sensor documentation for details.)", py::metaclass());
    def_enum(timestamp_mode, sensor::impl::timestamp_mode_strings);
    timestamp_mode.value("TIME_FROM_UNSPEC", sensor::timestamp_mode::TIME_FROM_UNSPEC);
    timestamp_mode.attr("from_string") = py::cpp_function([](const std::string& s) {
        return sensor::timestamp_mode_of_string(s); }, py::name("from_string"), "Create TimestampMode from string.");

    auto OperatingMode = py::enum_<sensor::OperatingMode>(m, "OperatingMode", R"(
        Possible Operating modes of sensor.

        See sensor documentation for details.)", py::metaclass());
    def_enum(OperatingMode, sensor::impl::operating_mode_strings, "OPERATING_");

    auto MultipurposeIOMode = py::enum_<sensor::MultipurposeIOMode>(m, "MultipurposeIOMode", R"(
        Mode of MULTIPURPOSE_IO pin.

        See sensor documentation for details.)", py::metaclass());
    def_enum(MultipurposeIOMode, sensor::impl::multipurpose_io_mode_strings, "MULTIPURPOSE_");

    auto Polarity = py::enum_<sensor::Polarity>(m, "Polarity", R"(
        Pulse Polarity.

        Applicable to several Polarity settings on sensor.)", py::metaclass());
    def_enum(Polarity, sensor::impl::polarity_strings, "POLARITY_");

    auto NMEABaudRate = py::enum_<sensor::NMEABaudRate>(m, "NMEABaudRate", R"(
        Expected baud rate sensor attempts to decode for NMEA UART input $GPRMC messages.)", py::metaclass());
    def_enum(NMEABaudRate, sensor::impl::nmea_baud_rate_strings);

    auto ChanField = py::enum_<sensor::ChanField>(m, "ChanField", "Channel data block fields.", py::metaclass());
    def_enum(ChanField, sensor::impl::chanfield_strings);

    auto UDPProfileLidar = py::enum_<sensor::UDPProfileLidar>(m, "UDPProfileLidar", "UDP lidar profile.", py::metaclass());
    def_enum(UDPProfileLidar, sensor::impl::udp_profile_lidar_strings, "PROFILE_LIDAR_");

    auto UDPProfileIMU = py::enum_<sensor::UDPProfileIMU>(m, "UDPProfileIMU", "UDP imu profile.", py::metaclass());
    def_enum(UDPProfileIMU, sensor::impl::udp_profile_imu_strings, "PROFILE_IMU_");

    // Sensor Config
    py::class_<sensor_config>(m, "SensorConfig", R"(
        Corresponds to sensor config parameters. Please see sensor documentation for the meaning of each property.
        )")
        .def(py::init<>(), R"(
        Args:
            raw (str): json string to parse
        )")
        .def("__init__", [](sensor_config& self, const std::string &s) {
            new (&self) sensor_config{};
            self = sensor::parse_config(s);
        })
        .def_readwrite("udp_dest", &sensor_config::udp_dest, "Destination to which sensor sends UDP traffic.")
        .def_readwrite("udp_port_lidar", &sensor_config::udp_port_lidar, "Port on UDP destination to which lidar data will be sent.")
        .def_readwrite("udp_port_imu", &sensor_config::udp_port_imu, "Port on UDP destination to which IMU data will be sent.")
        .def_readwrite("timestamp_mode", &sensor_config::ts_mode, "Timestamp mode of sensor. See class TimestampMode.")
        .def_readwrite("lidar_mode", &sensor_config::ld_mode, "Horizontal and Vertical Resolution rate of sensor as mode, e.g., 1024x10. See class LidarMode.")
        .def_readwrite("operating_mode", &sensor_config::operating_mode, "Operating Mode of sensor. See class OperatingMode.")
        .def_readwrite("multipurpose_io_mode", &sensor_config::multipurpose_io_mode, "Mode of MULTIPURPOSE_IO pin. See class MultipurposeIOMode.")
        .def_readwrite("azimuth_window", &sensor_config::azimuth_window, "Tuple representing the visible region of interest of the sensor in millidegrees, .e.g., (0, 360000) for full visibility.")
        .def_readwrite("signal_multiplier", &sensor_config::signal_multiplier, "Multiplier for signal strength of sensor, corresponding to maximum allowable azimuth_window. Gen 2 Only.")
        .def_readwrite("sync_pulse_out_angle", &sensor_config::sync_pulse_out_angle, "Polarity of SYNC_PULSE_OUT output. See sensor documentation for details." )
        .def_readwrite("sync_pulse_out_pulse_width", &sensor_config::sync_pulse_out_pulse_width, "SYNC_PULSE_OUT pulse width in ms. See sensor documentation for details.")
        .def_readwrite("nmea_in_polarity", &sensor_config::nmea_in_polarity, "Polarity of NMEA UART input $GPRMC messages. See sensor documentation for details.")
        .def_readwrite("nmea_baud_rate", &sensor_config::nmea_baud_rate, "Expected baud rate sensor attempts to decode for NMEA UART input $GPRMC messages.")
        .def_readwrite("nmea_ignore_valid_char", &sensor_config::nmea_ignore_valid_char, "NMEA Ignore Valid Char. True corresponds to 1 and False to 0 for property; see sensor documentation for details.")
        .def_readwrite("nmea_leap_seconds", &sensor_config::nmea_leap_seconds, "Integer number of leap seconds that will be added to the UDP timetsamp when calculating seconds since Unix Epoch time. See sensor documentation for details.")
        .def_readwrite("sync_pulse_in_polarity", &sensor_config::sync_pulse_in_polarity, "Polarity of SYNC_PULSE_IN pin. See sensor documentation for details.")
        .def_readwrite("sync_pulse_out_polarity", &sensor_config::sync_pulse_out_polarity, "Polarity of SYNC_PULSE_OUT output. See sensor documentation for details." )
        .def_readwrite("sync_pulse_out_frequency", &sensor_config::sync_pulse_out_frequency, "SYNC_PULSE_OUT rate. See sensor documentation for details.")
        .def_readwrite("phase_lock_enable", &sensor_config::phase_lock_enable, "Enable phase lock. See sensor documentation for more details.")
        .def_readwrite("phase_lock_offset", &sensor_config::phase_lock_offset, "Angle in Lidar Coordinate Frame that sensors are locked to, in millidegrees. See sensor documentation for details.")
        .def_readwrite("columns_per_packet", &sensor_config::columns_per_packet, "Measurement blocks per UDP packet. See sensor documentation for details.")
        .def_readwrite("udp_profile_lidar", &sensor_config::udp_profile_lidar, "UDP packet format for lidar data. See sensor documentation for details.")
        .def_readwrite("udp_profile_imu", &sensor_config::udp_profile_imu, "UDP packet format for imu data. See sensor documentation for details.")
        .def("__str__", [](const sensor_config& i) { return to_string(i); })
        .def("__eq__", [](const sensor_config& i, const sensor_config& j) { return i == j; })
        .def("__copy__", [](const sensor_config& self) { return sensor_config{self}; })
        .def("__deepcopy__", [](const sensor_config& self, py::dict) { return sensor_config{self}; });

    m.def("set_config", [] (const std::string& hostname, const sensor_config& config, bool persist,  bool udp_dest_auto) {
        uint8_t config_flags = 0;
        if (persist) config_flags |= ouster::sensor::CONFIG_PERSIST;
        if (udp_dest_auto) config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
        if (!sensor::set_config(hostname, config, config_flags)) {
            throw std::runtime_error("Error setting sensor config.");
        }
    }, R"(
        Set sensor config parameters on sensor.

        Args:
            hostname (str): hostname of the sensor
            config (SensorConfig): config to set sensor parameters to
            persist (bool): persist parameters after sensor disconnection (default = False)
            udp_dest_auto: automatically determine sender's IP at the time command was sent
                and set it as destination of UDP traffic. Function will error out if config has
                udp_dest member. (default = False)
        )", py::arg("hostname"), py::arg("config"), py::arg("persist") = false, py::arg("udp_dest_auto") = false);

    m.def("get_config", [](const std::string& hostname, bool active) {
        sensor::sensor_config config;
        if (!sensor::get_config(hostname, config, active)) {
            throw std::runtime_error("Error getting sensor config.");
        }
        return config;
    }, R"(
        Returns sensor config parameters as SensorConfig.

        Args:
            hostname (str): hostname of the sensor
            active (bool): return active or staged sensor configuration
        )", py::arg("hostname"), py::arg("active") = true);

    // Version Info
    py::class_<util::version>(m, "Version")
        .def(py::init<>())
        .def("__eq__", [](const util::version& u, const util::version& v) { return u == v; })
        .def("__lt__", [](const util::version& u, const util::version& v) { return u < v; })
        .def("__le__", [](const util::version& u, const util::version& v) { return u <= v; })
        .def("__str__", [](const util::version& u) { return to_string(u); })
        .def_readwrite("major", &util::version::major)
        .def_readwrite("minor", &util::version::minor)
        .def_readwrite("patch", &util::version::patch)
        .def_static("from_string", &util::version_of_string);

    m.attr("invalid_version") = util::invalid_version;

    m.attr("min_version") = sensor::min_version;

    // clang-format on

    // Client Handle
    py::enum_<sensor::client_state>(m, "ClientState", py::arithmetic())
        .value("TIMEOUT", sensor::client_state::TIMEOUT)
        .value("ERROR", sensor::client_state::CLIENT_ERROR)
        .value("LIDAR_DATA", sensor::client_state::LIDAR_DATA)
        .value("IMU_DATA", sensor::client_state::IMU_DATA)
        .value("EXIT", sensor::client_state::EXIT)
        // TODO: revisit including in C++ API
        .value("OVERFLOW",
               sensor::client_state(BufferedUDPSource::CLIENT_OVERFLOW));

    py::class_<BufferedUDPSource>(m, "Client")
        .def(py::init<std::string, int, int, size_t>(),
             py::arg("hostname") = "", py::arg("lidar_port") = 7502,
             py::arg("imu_port") = 7503, py::arg("capacity") = 128)
        .def(py::init<std::string, std::string, sensor::lidar_mode,
                      sensor::timestamp_mode, int, int, int, size_t>(),
             py::arg("hostname"), py::arg("udp_dest_host"),
             py::arg("mode") = sensor::lidar_mode::MODE_1024x10,
             py::arg("timestamp_mode") =
                 sensor::timestamp_mode::TIME_FROM_INTERNAL_OSC,
             py::arg("lidar_port") = 0, py::arg("imu_port") = 0,
             py::arg("timeout_sec") = 30, py::arg("capacity") = 128)
        .def("get_metadata", &BufferedUDPSource::get_metadata,
             py::arg("timeout_sec") = 60, py::arg("legacy") = true)
        .def("shutdown", &BufferedUDPSource::shutdown)
        .def("consume",
             [](BufferedUDPSource& self, py::buffer buf, float timeout_sec) {
                 using fsec = chrono::duration<float>;

                 auto info = buf.request();

                 // timeout_sec == 0 means nonblocking, < 0 means forever
                 auto timeout_time =
                     timeout_sec >= 0
                         ? chrono::steady_clock::now() + fsec{timeout_sec}
                         : chrono::steady_clock::time_point::max();

                 // consume() with 0 timeout means return if no queued
                 // packets
                 float poll_interval = timeout_sec ? 0.1 : 0.0;

                 // allow interrupting timeout from Python by polling
                 sensor::client_state res = sensor::client_state::TIMEOUT;
                 do {
                     res = self.consume(static_cast<uint8_t*>(info.ptr),
                                        info.size, poll_interval);
                     if (res != sensor::client_state::TIMEOUT) break;

                     if (PyErr_CheckSignals() != 0)
                         throw py::error_already_set();
                     // allow other python threads to run
                     py::gil_scoped_release release;
                 } while (chrono::steady_clock::now() < timeout_time);
                 return res;
             })
        .def("produce",
             [](BufferedUDPSource& self, const packet_format& pf) {
                 py::gil_scoped_release release;
                 self.produce(pf);
             })
        .def("flush", &BufferedUDPSource::flush, py::arg("n_packets") = 0)
        .def_property_readonly("capacity", &BufferedUDPSource::capacity)
        .def_property_readonly("size", &BufferedUDPSource::size)
        .def_property_readonly("lidar_port", &BufferedUDPSource::get_lidar_port)
        .def_property_readonly("imu_port", &BufferedUDPSource::get_imu_port);

    // Scans
    py::class_<LidarScan>(m, "LidarScan", py::metaclass(), R"(
        Represents a single "scan" or "frame" of lidar data.

        This is a "struct of arrays" representation of lidar data. Column headers are
        stored as contiguous W element arrays, while fields are WxH arrays. Channel
        fields are staggered, so the ith column header value corresponds to the ith
        column of data in each field.
        )")
        .def_readonly_static("N_FIELDS", &LidarScan::N_FIELDS, "Deprecated.")
        // TODO: Python and C++ API differ in h/w order for some reason
        .def("__init__", [](LidarScan& self, size_t h,
                            size_t w) { new (&self) LidarScan(w, h); })
        .def("__init__",
             [](LidarScan& self, size_t h, size_t w,
                sensor::UDPProfileLidar profile) {
                 new (&self) LidarScan(w, h, profile);
             })
        .def("__init__",
             [](LidarScan& self, size_t h, size_t w,
                const std::map<sensor::ChanField, py::object>& field_types) {
                 std::map<sensor::ChanField, sensor::ChanFieldType> ft;
                 for (const auto& kv : field_types) {
                     auto dt = py::dtype::from_args(kv.second);
                     ft[kv.first] = field_type_of_dtype(dt);
                 }
                 new (&self) LidarScan(w, h, ft.begin(), ft.end());
             })
        .def_readonly("w", &LidarScan::w,
                      "Width or horizontal resolution of the scan.")
        .def_readonly("h", &LidarScan::h,
                      "Height or vertical resolution of the scan.")
        .def_readwrite(
            "frame_id", &LidarScan::frame_id,
            "Corresponds to the frame id header in the packet format.")
        .def(
            "_complete",
            [](const LidarScan& self,
               nonstd::optional<sensor::AzimuthWindow> window) {
                if (!window) window = {0, self.w - 1};

                const auto& status = self.status();
                auto start = window.value().first;
                auto end = window.value().second;

                if (start <= end)
                    return status.segment(start, end - start + 1)
                        .unaryExpr([](uint32_t s) { return s & 0x01; })
                        .isConstant(0x01);
                else
                    return status.segment(0, end)
                               .unaryExpr([](uint32_t s) { return s & 0x01; })
                               .isConstant(0x01) &&
                           status.segment(start, self.w - start)
                               .unaryExpr([](uint32_t s) { return s & 0x01; })
                               .isConstant(0x01);
            },
            py::arg("window") =
                static_cast<nonstd::optional<sensor::AzimuthWindow>>(
                    nonstd::nullopt))
        .def(
            "field",
            [](LidarScan& self, sensor::ChanField f) {
                std::vector<size_t> dims{static_cast<size_t>(self.h),
                                         static_cast<size_t>(self.w)};
                py::array res;
                impl::visit_field(self, f, [&](auto field) {
                    auto dtype =
                        py::dtype::of<typename decltype(field)::Scalar>();
                    res = py::array(dtype, dims, field.data(), py::cast(self));
                });
                return res;
            },
            R"(
        Return a view of the specified channel field.

        Args:
            field: The channel field to return

        Returns:
            The specified field as a numpy array
        )")
        .def(
            "header",
            [](LidarScan& self, py::object& o) {
                // the argument should be a ColHeader enum defined in
                // data.py
                auto ind = py::int_(o).cast<int>();
                switch (ind) {
                    case 0:
                        return py::array(py::dtype::of<uint64_t>(),
                                         static_cast<size_t>(self.w),
                                         self.timestamp().data(),
                                         py::cast(self));
                    case 1:
                        // encoder values are deprecated and not included in
                        // the updated C++ LidarScan API. Access old values
                        // instead
                        return py::array(py::dtype::of<uint32_t>(),
                                         {static_cast<size_t>(self.w)},
                                         {sizeof(LidarScan::BlockHeader)},
                                         &self.headers.at(0).encoder,
                                         py::cast(self));
                    case 2:
                        return py::array(py::dtype::of<uint16_t>(),
                                         static_cast<size_t>(self.w),
                                         self.measurement_id().data(),
                                         py::cast(self));
                    case 3:
                        return py::array(py::dtype::of<uint32_t>(),
                                         static_cast<size_t>(self.w),
                                         self.status().data(), py::cast(self));
                    default:
                        throw std::invalid_argument(
                            "Unexpected index for LidarScan.header()");
                }
            },
            R"(
        Return the specified column header as a numpy array.

        This function is deprecated. Use the ``measurment_id``, ``timestamp`` or
        ``status`` properties instead.

        Args:
            header: The column header to return

        Returns:
            The specified column header as a numpy array
        )")
        .def_property_readonly(
            "timestamp",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint64_t>(), self.w,
                                 self.timestamp().data(), py::cast(self));
            },
            "The measurement timestamp header as a W-element numpy array.")
        .def_property_readonly(
            "measurement_id",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint16_t>(), self.w,
                                 self.measurement_id().data(), py::cast(self));
            },
            "The measurement id header as a W-element numpy array.")
        .def_property_readonly(
            "status",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint32_t>(), self.w,
                                 self.status().data(), py::cast(self));
            },
            "The measurement status header as a W-element numpy array.")
        .def_property_readonly(
            "fields",
            // NOTE: keep_alive seems to be ignored without cpp_function wrapper
            py::cpp_function(
                [](LidarScan& self) {
                    return py::make_key_iterator(self.begin(), self.end());
                },
                py::keep_alive<0, 1>()),
            "Return an iterator of available channel fields.")
        .def("__eq__",
             [](const LidarScan& l, const LidarScan& r) { return l == r; })
        .def("__copy__", [](const LidarScan& self) { return LidarScan{self}; })
        .def("__deepcopy__",
             [](const LidarScan& self, py::dict) { return LidarScan{self}; })
        // for backwards compatibility: previously converted between Python
        // / native representations, now a noop
        .def("to_native", [](py::object& self) { return self; })
        .def_static("from_native", [](py::object& scan) { return scan; });

    // Destagger overloads for most numpy scalar types
    m.def("destagger_int8", &ouster::destagger<int8_t>);
    m.def("destagger_int16", &ouster::destagger<int16_t>);
    m.def("destagger_int32", &ouster::destagger<int32_t>);
    m.def("destagger_int64", &ouster::destagger<int64_t>);
    m.def("destagger_uint8", &ouster::destagger<uint8_t>);
    m.def("destagger_uint16", &ouster::destagger<uint16_t>);
    m.def("destagger_uint32", &ouster::destagger<uint32_t>);
    m.def("destagger_uint64", &ouster::destagger<uint64_t>);
    m.def("destagger_float", &ouster::destagger<float>);
    m.def("destagger_double", &ouster::destagger<double>);

    py::class_<ScanBatcher>(m, "ScanBatcher")
        .def(py::init<int, packet_format>())
        .def(py::init<sensor_info>())
        .def("__call__", [](ScanBatcher& self, py::buffer& buf, LidarScan& ls) {
            uint8_t* ptr = getptr(self.pf.lidar_packet_size, buf);
            return self(ptr, ls);
        });

    // XYZ Projection
    py::class_<XYZLut>(m, "XYZLut")
        .def("__init__",
             [](XYZLut& self, const sensor_info& sensor) {
                 new (&self) XYZLut{};
                 self = make_xyz_lut(sensor);
             })
        .def("__call__",
             [](const XYZLut& self, Eigen::Ref<img_t<uint32_t>>& range) {
                 return cartesian(range, self);
             })
        .def("__call__", [](const XYZLut& self, const LidarScan& scan) {
            return cartesian(scan, self);
        });

    m.attr("__version__") = VERSION_INFO;

    // Image processing
    py::class_<viz::AutoExposure>(m, "AutoExposure")
        .def(py::init<>())
        .def(py::init<int>(), py::arg("update_every"))
        .def(py::init<double, double, int>(), py::arg("lo_percentile"),
             py::arg("hi_percentile"), py::arg("update_every"))
        .def("__call__", &image_proc_call<viz::AutoExposure, float>,
             py::arg("image"), py::arg("update_state") = true)
        .def("__call__", &image_proc_call<viz::AutoExposure, double>,
             py::arg("image"), py::arg("update_state") = true);

    py::class_<viz::BeamUniformityCorrector>(m, "BeamUniformityCorrector")
        .def(py::init<>())
        .def("__call__", &image_proc_call<viz::BeamUniformityCorrector, float>,
             py::arg("image"), py::arg("update_state") = true)
        .def("__call__", &image_proc_call<viz::BeamUniformityCorrector, double>,
             py::arg("image"), py::arg("update_state") = true);

    return m.ptr();
}

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

// spdlog includes
// TODO: revise once ubuntu 18.04 is deprecated to drop dependence on
// spdlog with version < 1
#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>
#if (SPDLOG_VER_MAJOR < 1)
#include <spdlog/formatter.h>
#elif ((SPDLOG_VER_MAJOR == 1) && (SPDLOG_VER_MINOR <= 5))
#include <spdlog/details/pattern_formatter.h>
#else
#include <spdlog/pattern_formatter.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "ouster/buffered_udp_source.h"
#include "ouster/client.h"
#include "ouster/image_processing.h"
#include "ouster/impl/build.h"
#include "ouster/impl/packet_writer.h"
#include "ouster/impl/profile_extension.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

namespace py = pybind11;
namespace chrono = std::chrono;
using spdlog::sinks::base_sink;

using ouster::sensor::calibration_status;
using ouster::sensor::ChanField;
using ouster::sensor::data_format;
using ouster::sensor::ImuPacket;
using ouster::sensor::LidarPacket;
using ouster::sensor::packet_format;
using ouster::sensor::sensor_config;
using ouster::sensor::sensor_info;
using ouster::sensor::impl::BufferedUDPSource;
using ouster::sensor::impl::packet_writer;
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

extern spdlog::logger& logger();

namespace impl {

extern const Table<lidar_mode, const char*, 7> lidar_mode_strings;
extern const Table<timestamp_mode, const char*, 4> timestamp_mode_strings;
extern const Table<OperatingMode, const char*, 2> operating_mode_strings;
extern const Table<MultipurposeIOMode, const char*, 6>
    multipurpose_io_mode_strings;
extern const Table<Polarity, const char*, 2> polarity_strings;
extern const Table<NMEABaudRate, const char*, 2> nmea_baud_rate_strings;
extern Table<ChanField, const char*, 29> chanfield_strings;
extern Table<UDPProfileLidar, const char*, MAX_NUM_PROFILES>
    udp_profile_lidar_strings;
extern Table<UDPProfileIMU, const char*, 1> udp_profile_imu_strings;
extern Table<ShotLimitingStatus, const char*, 10> shot_limiting_status_strings;
extern Table<ThermalShutdownStatus, const char*, 2>
    thermal_shutdown_status_strings;

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
    // weed out empty profiles
    auto end = std::find_if(
        strings_table.begin(), strings_table.end(),
        [](const std::pair<E, const char*>& p) { return p.second == nullptr; });

    // in pybind11 2.0, calling enum.value(const char* name, val) doesn't make a
    // copy of the name argument. When value names aren't statically allocated,
    // we have to keep them alive. Use deque for stability of c_str() pointers
    static std::deque<std::string> enumerator_names;

    // module imports
    py::object MappingProxy =
        py::module::import("types").attr("MappingProxyType");

    // declare enumerators
    for (auto it = strings_table.begin(); it != end; ++it) {
        enumerator_names.push_back(enum_prefix + it->second);
        Enum.value(enumerator_names.back().c_str(), it->first);
    }

    // use immutable MappingProxy to return members dict
    std::map<std::string, E> members;
    for (auto it = strings_table.begin(); it != end; ++it)
        members[it->second] = it->first;
    py::object py_members = MappingProxy(members);
    Enum.def_property_readonly_static(
        "__members__", [=](py::object) { return py_members; },
        "Returns a mapping of member name->value.");

    // can't make the class iterable itself easily
    Enum.def_property_readonly_static(
        "values",
        [&, end](py::object) {
            return py::make_key_iterator(strings_table.begin(), end);
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
    if (dt.is(py::dtype::of<uint8_t>()))
        return sensor::ChanFieldType::UINT8;
    else if (dt.is(py::dtype::of<uint16_t>()))
        return sensor::ChanFieldType::UINT16;
    else if (dt.is(py::dtype::of<uint32_t>()))
        return sensor::ChanFieldType::UINT32;
    else if (dt.is(py::dtype::of<uint64_t>()))
        return sensor::ChanFieldType::UINT64;
    else
        throw std::invalid_argument("Invalid dtype for a channel field");
}

/*
 * Map a channel field type to a dtype
 */
static py::dtype dtype_of_field_type(const sensor::ChanFieldType& ftype) {
    switch (ftype) {
        case sensor::ChanFieldType::UINT8:
            return py::dtype::of<uint8_t>();
        case sensor::ChanFieldType::UINT16:
            return py::dtype::of<uint16_t>();
        case sensor::ChanFieldType::UINT32:
            return py::dtype::of<uint32_t>();
        case sensor::ChanFieldType::UINT64:
            return py::dtype::of<uint64_t>();
        default:
            throw std::invalid_argument(
                "Invalid field_type for convertion to dtype");
    }
    return py::dtype();  // unreachable ...
}

template <typename Fn>
struct lambda_iter {
    Fn lambda;

    // generative output iterators return themselves
    lambda_iter& operator*() { return *this; }
    // prefix
    lambda_iter& operator++() { return *this; }
    // postfix
    lambda_iter& operator++(int) { return *this; }

    template <typename T>
    lambda_iter& operator=(T&& item) {
        lambda(item);
        return *this;
    }
};

template <typename Fn>
lambda_iter<Fn> make_lambda_iter(Fn&& f) {
    return lambda_iter<Fn>{f};
}

template <typename T>
struct set_field {
    using Field = py::array_t<T, py::array::c_style | py::array::forcecast>;

    void operator()(const packet_writer& self, LidarPacket& p, ChanField i,
                    Field field) {
        if (field.ndim() != 2 || field.shape(0) != self.pixels_per_column ||
            field.shape(1) != self.columns_per_packet)
            throw std::invalid_argument("field dimension mismatch");

        /**
         * It is a bit weird to be setting these back and forth to work around
         * packet_writer::set_block logic that is intended for lidarscan usage
         * but I do think it is better than keeping two versions of the same.
         *
         * This is intended for python users, which will expect it to work out
         * of the box without any extra fiddling.
         */
        std::vector<uint16_t> m_ids(self.columns_per_packet);
        std::vector<uint32_t> statuses(self.columns_per_packet);

        for (int icol = 0; icol < self.columns_per_packet; ++icol) {
            uint8_t* col_buf = self.nth_col(icol, p.buf.data());
            // store for later reassignment
            m_ids[icol] = self.col_measurement_id(col_buf);
            statuses[icol] = self.col_status(col_buf);
            // overwrite with 0..columns_per_packet
            self.set_col_measurement_id(col_buf, icol);
            self.set_col_status(col_buf, 0x1);
        }

        Eigen::Map<const img_t<T>> field_map(field.data(), field.shape(0),
                                             field.shape(1));
        // eigen is trash; this extra step is extra annoying
        Eigen::Ref<const img_t<T>> ref = field_map;
        self.set_block(ref, i, p.buf.data());

        // restore m_ids and statuses
        for (int icol = 0; icol < self.columns_per_packet; ++icol) {
            uint8_t* col_buf = self.nth_col(icol, p.buf.data());
            self.set_col_measurement_id(col_buf, m_ids[icol]);
            self.set_col_status(col_buf, statuses[icol]);
        }
    }
};

#if (SPDLOG_VER_MAJOR >= 1)  // don't include for spdlog < 1.x.x

/*
 * Forward spdlog to python logging module
 */
class PySink : public base_sink<std::mutex> {
   protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        // https://spdlog.docsforge.com/v1.x/4.sinks/#implementing-your-own-sink
        // Probably need #if here for spdlog 0.x on Ubuntu 18.04
        // fmt::memory_buffer formatted;
        spdlog::memory_buf_t formatted;
        base_sink<std::mutex>::formatter_->format(msg, formatted);

        py::gil_scoped_acquire acquire;
        // map spdlog levels to python logging levels
        int py_level = logger_.attr("NOTSET").cast<int>();
        switch (msg.level) {
            case spdlog::level::trace:
            case spdlog::level::debug:
                py_level = logger_.attr("DEBUG").cast<int>();
                break;
            case spdlog::level::info:
                py_level = logger_.attr("INFO").cast<int>();
                break;
            case spdlog::level::warn:
                py_level = logger_.attr("WARNING").cast<int>();
                break;
            case spdlog::level::err:
                py_level = logger_.attr("ERROR").cast<int>();
                break;
            case spdlog::level::critical:
                py_level = logger_.attr("CRITICAL").cast<int>();
                break;
            default:
                break;
        }
        logger_.attr("log")(py_level, fmt::to_string(formatted));
    }

    // noop for python logger
    void flush_() override {}

    py::object logger_;

   public:
    PySink(py::object logger) : logger_{logger} {}
};

#endif  // (SPDLOG_VER_MAJOR >= 1)

PYBIND11_MODULE(_client, m) {
    m.doc() = R"(
    Sensor client bindings generated by pybind11.

    This module is generated directly from the C++ code and not meant to be used
    directly.
    )";

    // turn off signatures in docstrings: mypy stubs provide better types
    py::options options;
    options.disable_function_signatures();

    // TODO (1/4): Enable automatically directing ouster sensor logs to the
    // python::logging in the future. This is disabled for now so the user will
    // have to use init_logger API to practice any control over the logs
    // ouster::sensor::logger().sinks() = {
    //     std::make_shared<PySink>(py::module::import("logging"))
    // };

    // TODO (2/4): parse python::logging level and set spdlog logger to the same
    // level rather than forwarding everything to Python as set here:
    // ouster::sensor::logger().set_level(spdlog::level::trace);

    // TODO (3/4): configure the formatter.
    // spdlog adds newline by default, not needed when forwarding to python. Use
    // a custom formatter to avoid this (check the "") argument
    // https://github.com/gabime/spdlog/issues/579
    // ouster::sensor::logger().set_formatter(
    //     std::make_unique<spdlog::pattern_formatter>(
    //         "%v", spdlog::pattern_time_type::local, ""));

    // TODO (4/4): Remove the PySink.
    // since the static logger sink keeps a reference to Python objects, they
    // must be cleaned up before the Python runtime exits
    // m.add_object("_cleanup", py::capsule([]() {
    //                  ouster::sensor::logger().sinks().clear();
    //              }));

    // clang-format off

    // Packet Format
    py::class_<packet_format>(m, "PacketFormat")
        .def_static("from_info",
                    [](const sensor_info& info) -> const packet_format& {
                        return sensor::get_format(info);
                    }, py::return_value_policy::reference)
        .def_static("from_profile",
                    [](sensor::UDPProfileLidar profile,
                       size_t pixels_per_column,
                       size_t columns_per_packet) -> const packet_format& {
                        return sensor::get_format(profile, pixels_per_column,
                                                  columns_per_packet);
                    }, py::return_value_policy::reference)
        .def_readonly("lidar_packet_size", &packet_format::lidar_packet_size)
        .def_readonly("imu_packet_size", &packet_format::imu_packet_size)
        .def_readonly("udp_profile_lidar", &packet_format::udp_profile_lidar)
        .def_readonly("columns_per_packet", &packet_format::columns_per_packet)
        .def_readonly("pixels_per_column", &packet_format::pixels_per_column)
        .def_readonly("packet_header_size", &packet_format::packet_header_size)
        .def_readonly("col_header_size", &packet_format::col_header_size)
        .def_readonly("col_footer_size", &packet_format::col_footer_size)
        .def_readonly("col_size", &packet_format::col_size)
        .def_readonly("packet_footer_size", &packet_format::packet_footer_size)

        .def("field_value_mask", &packet_format::field_value_mask)
        .def("field_bitness", &packet_format::field_bitness)

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

        .def("countdown_thermal_shutdown", [](packet_format& pf, py::buffer buf) {
            return pf.countdown_thermal_shutdown(getptr(pf.lidar_packet_size, buf));
        })

        .def("countdown_shot_limiting", [](packet_format& pf, py::buffer buf) {
            return pf.countdown_shot_limiting(getptr(pf.lidar_packet_size, buf));
        })

        .def("thermal_shutdown", [](packet_format& pf, py::buffer buf) {
            return pf.thermal_shutdown(getptr(pf.lidar_packet_size, buf));
        })

        .def("shot_limiting", [](packet_format& pf, py::buffer buf) {
            return pf.shot_limiting(getptr(pf.lidar_packet_size, buf));
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

    // PacketWriter
    py::class_<packet_writer, packet_format>(m, "PacketWriter")
        // this is safe so long as packet_writer does not carry any extra state
        .def_static("from_info",
                    [](const sensor_info& info) -> const packet_writer& {
                        const auto& pf = sensor::get_format(info);
                        return static_cast<const packet_writer&>(pf);
                    }, py::return_value_policy::reference)
        .def_static("from_profile",
                    [](sensor::UDPProfileLidar profile,
                       size_t pixels_per_column,
                       size_t columns_per_packet) -> const packet_writer& {
                        const auto& pf = sensor::get_format(profile,
                                                            pixels_per_column,
                                                            columns_per_packet);
                        return static_cast<const packet_writer&>(pf);
                    }, py::return_value_policy::reference)
        .def("set_col_status",
              [](const packet_writer& self, LidarPacket& p, int col_idx,
                 uint32_t status) {
                  if (col_idx >= self.columns_per_packet)
                      throw std::invalid_argument("col_idx out of bounds");
                  uint8_t* col_buf = self.nth_col(col_idx, p.buf.data());
                  self.set_col_status(col_buf, status);
              })
        .def("set_col_timestamp",
              [](const packet_writer& self, LidarPacket& p, int col_idx,
                 uint64_t ts) {
                  if (col_idx >= self.columns_per_packet)
                      throw std::invalid_argument("col_idx out of bounds");
                  uint8_t* col_buf = self.nth_col(col_idx, p.buf.data());
                  self.set_col_timestamp(col_buf, ts);
              })
        .def("set_col_measurement_id",
              [](const packet_writer& self, LidarPacket& p, int col_idx,
                 uint16_t m_id) {
                  if (col_idx >= self.columns_per_packet)
                      throw std::invalid_argument("col_idx out of bounds");
                  uint8_t* col_buf = self.nth_col(col_idx, p.buf.data());
                  self.set_col_measurement_id(col_buf, m_id);
              })
        .def("set_frame_id",
              [](const packet_writer& self, LidarPacket& p, uint32_t frame_id) {
                  self.set_frame_id(p.buf.data(), frame_id);
              })
        .def("set_field", set_field<uint8_t>{})
        .def("set_field", set_field<uint16_t>{})
        .def("set_field", set_field<uint32_t>{})
        .def("set_field", set_field<uint64_t>{});

    m.def("scan_to_packets",
          [](const LidarScan& ls, const packet_writer& pw) -> py::list {
              py::list packets;
              py::object class_type =
                  py::module::import("ouster.client").attr("LidarPacket");

              auto append_pypacket = [&](const LidarPacket& packet) {
                  py::object pypacket = class_type(py::arg("packet_format") = pw);
                  // next couple lines should not fail unless someone messes with
                  // ouster.client.LidarPacket implementation
                  LidarPacket* p_ptr = pypacket.cast<LidarPacket*>();
                  if (p_ptr->buf.size() != packet.buf.size())
                      throw std::invalid_argument("packet sizes don't match");
                  p_ptr->host_timestamp = packet.host_timestamp;
                  std::memcpy(p_ptr->buf.data(), packet.buf.data(), packet.buf.size());
                  packets.append(pypacket);
              };

              auto iter = make_lambda_iter(append_pypacket);
              impl::scan_to_packets(ls, pw, iter);
              return packets;
          });

    // Data Format
    py::class_<data_format>(m, "DataFormat")
        .def(py::init<>(), R"(
            Data Format of a packet coming from sensor

            See sensor documentation for the meaning of each property
        )")
        .def_readwrite("pixels_per_column", &data_format::pixels_per_column, R"(

        Pixels per column in the lidar packet, synonymous with the number of beams of the sensor

        :type: int
        )")
        .def_readwrite("columns_per_packet", &data_format::columns_per_packet, R"(
        Columns per packet in the lidar packet, typically 16

        :type: int
        )")
        .def_readwrite("columns_per_frame", &data_format::columns_per_frame, R"(
        Columns per frame in the lidar packet, corresponding with lidar_mode

        :type: int
        )")
        .def_readwrite("pixel_shift_by_row", &data_format::pixel_shift_by_row, R"(
        Shift of pixels by row to create natural images

        :type: List[int]

        )")
        .def_readwrite("column_window", &data_format::column_window, R"(
        Firing window of sensor, set by config.azimuth_window

        :type: Tuple[int, int]
        )")
        .def_readwrite("udp_profile_lidar", &data_format::udp_profile_lidar, R"(
        Lidar packet profile
        )")
        .def_readwrite("udp_profile_imu", &data_format::udp_profile_imu, R"(
        IMU packet profile
        )")
        .def_readwrite("fps", &data_format::fps, R"(
        Frames per second, e.g., 10 for lidar_mode 1024x10

        :type: int
        )");

    py::class_<calibration_status>(m, "SensorCalibration")
        .def(py::init<>(), R"(
           Sensor Calibration in a Sensor Metadata, covering reflectivity calibration and more"
        )")
        .def_readwrite("reflectivity_status", &calibration_status::reflectivity_status, "Reflectivity calibration status")
        .def_readwrite("reflectivity_timestamp", &calibration_status::reflectivity_timestamp, "Reflectivity timestamp")
        .def("__str__", [](const calibration_status& i) { return to_string(i); })
        .def("__eq__", [](const calibration_status& i, const calibration_status& j) { return i == j; });

    // Sensor Info
    py::class_<sensor_info>(m, "SensorInfo", R"(
        Sensor Info required to interpret UDP data streams.

        See the sensor documentation for the meaning of each property.
        )")
        .def(py::init<>(), R"(
        Args:
            raw (str): json string to parse
        )")
        .def("__init__", [](sensor_info& self, const std::string& s, bool skip_beam_validation) {
            new (&self) sensor_info{};
            self = sensor::parse_metadata(s, skip_beam_validation);
        }, py::arg("s"), py::arg("skip_beam_validation") = false)
        .def_readwrite("hostname", &sensor_info::name, "Sensor hostname.")
        .def_readwrite("sn", &sensor_info::sn, "Sensor serial number.")
        .def_readwrite("fw_rev", &sensor_info::fw_rev, "Sensor firmware revision.")
        .def_readwrite("mode", &sensor_info::mode, "Lidar mode, e.g., 1024x10.")
        .def_readwrite("prod_line", &sensor_info::prod_line, "Product line, e.g., 'OS-1-128'.")
        .def_readwrite("format", &sensor_info::format,  "Describes the structure of a lidar packet. See class DataFormat.")
        .def_readwrite("beam_azimuth_angles", &sensor_info::beam_azimuth_angles, "Beam azimuth angles, useful for XYZ projection.")
        .def_readwrite("beam_altitude_angles", &sensor_info::beam_altitude_angles, "Beam altitude angles, useful for XYZ projection.")
        .def_readwrite("imu_to_sensor_transform", &sensor_info::imu_to_sensor_transform, "Homogenous transformation matrix representing IMU offset to Sensor Coordinate Frame.")
        .def_readwrite("lidar_to_sensor_transform", &sensor_info::lidar_to_sensor_transform, "Homogeneous transformation matrix from Lidar Coordinate Frame to Sensor Coordinate Frame.")
        .def_readwrite("lidar_origin_to_beam_origin_mm", &sensor_info::lidar_origin_to_beam_origin_mm, "Distance between lidar origin and beam origin in millimeters.")
        .def_readwrite("beam_to_lidar_transform", &sensor_info::beam_to_lidar_transform, "Homogenous transformation matrix reprsenting Beam to Lidar Transform")
        .def_readwrite("extrinsic", &sensor_info::extrinsic, "Extrinsic Matrix.")
        .def_readwrite("init_id", &sensor_info::init_id, "Initialization id.")
        .def_readwrite("udp_port_lidar", &sensor_info::udp_port_lidar, "Configured port for lidar data.")
        .def_readwrite("udp_port_imu", &sensor_info::udp_port_imu, "Configured port for imu data.")
        .def_readwrite("build_date", &sensor_info::build_date, "Build date")
        .def_readwrite("image_rev", &sensor_info::image_rev, "Image rev")
        .def_readwrite("prod_pn", &sensor_info::prod_pn, "Prod pn")
        .def_readwrite("status", &sensor_info::status, "sensor status")
        .def_readwrite("cal", &sensor_info::cal, "sensor calibration")
        .def_readwrite("config", &sensor_info::config, "sensor config")
        .def_static("from_default", &sensor::default_sensor_info, R"(
        Create gen-1 OS-1-64 SensorInfo populated with design values.
        )")
        .def("original_string", &sensor_info::original_string, R"( Return original string that initialized sensor_info"
        )")
        .def("updated_metadata_string", &sensor_info::updated_metadata_string, R"( Return metadata string made from updated entries"
        )")
        .def("has_fields_equal", &sensor_info::has_fields_equal, R"(Compare public fields")")
        // only uncomment for debugging purposes!! story for general use and output is not filled
        //.def("__str__", [](const sensor_info& i) { return to_string(i); })
        .def("__eq__", [](const sensor_info& i, const sensor_info& j) { return i == j; })
        .def("__repr__", [](const sensor_info& self) {
            const auto mode = self.mode ? to_string(self.mode) : std::to_string(self.format.fps) + "fps";
            return "<ouster.client.SensorInfo " + self.prod_line + " " +
                self.sn + " " + self.fw_rev + " " + mode + ">";
        })
        .def("__copy__", [](const sensor_info& self) { return sensor_info{self}; })
        .def("__deepcopy__", [](const sensor_info& self, py::dict) { return sensor_info{self}; });


    // Enums
    auto lidar_mode = py::enum_<sensor::lidar_mode>(m, "LidarMode", R"(
        Possible Lidar Modes of sensor.

        Determines to horizontal and vertical resolution rates of sensor. See
        sensor documentation for details.)");
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

        See sensor documentation for details.)");
    def_enum(timestamp_mode, sensor::impl::timestamp_mode_strings);
    timestamp_mode.value("TIME_FROM_UNSPEC", sensor::timestamp_mode::TIME_FROM_UNSPEC);
    timestamp_mode.attr("from_string") = py::cpp_function([](const std::string& s) {
        return sensor::timestamp_mode_of_string(s); }, py::name("from_string"), "Create TimestampMode from string.");

    auto OperatingMode = py::enum_<sensor::OperatingMode>(m, "OperatingMode", R"(
        Possible Operating modes of sensor.

        See sensor documentation for details.)");
    def_enum(OperatingMode, sensor::impl::operating_mode_strings, "OPERATING_");

    auto MultipurposeIOMode = py::enum_<sensor::MultipurposeIOMode>(m, "MultipurposeIOMode", R"(
        Mode of MULTIPURPOSE_IO pin.

        See sensor documentation for details.)");
    def_enum(MultipurposeIOMode, sensor::impl::multipurpose_io_mode_strings, "MULTIPURPOSE_");

    auto Polarity = py::enum_<sensor::Polarity>(m, "Polarity", R"(
        Pulse Polarity.

        Applicable to several Polarity settings on sensor.)");
    def_enum(Polarity, sensor::impl::polarity_strings, "POLARITY_");

    auto NMEABaudRate = py::enum_<sensor::NMEABaudRate>(m, "NMEABaudRate", R"(
        Expected baud rate sensor attempts to decode for NMEA UART input $GPRMC messages.)");
    def_enum(NMEABaudRate, sensor::impl::nmea_baud_rate_strings);

    auto ChanField = py::enum_<sensor::ChanField>(m, "ChanField", R"(
    Channel data block fields
    )");
    def_enum(ChanField, sensor::impl::chanfield_strings);

    auto UDPProfileLidar = py::enum_<sensor::UDPProfileLidar>(m, "UDPProfileLidar", "UDP lidar profile.");
    def_enum(UDPProfileLidar, sensor::impl::udp_profile_lidar_strings, "PROFILE_LIDAR_");
    UDPProfileLidar.attr("from_string") = py::cpp_function(
        [](const std::string& s) {
            return sensor::udp_profile_lidar_of_string(s);
        },
        py::name("from_string"), "Create UDPProfileLidar from string.");
    UDPProfileLidar.def_property_readonly_static(
        "values",
        [](py::object) {
            return py::make_key_iterator(
                sensor::impl::udp_profile_lidar_strings.begin(),
                std::find_if(
                    sensor::impl::udp_profile_lidar_strings.begin(),
                    sensor::impl::udp_profile_lidar_strings.end(),
                    [](const auto& p) { return p.second == nullptr; }));
        },
        "Returns an iterator of all UDPProfileLidar enum members.");

    auto UDPProfileIMU = py::enum_<sensor::UDPProfileIMU>(m, "UDPProfileIMU", "UDP imu profile.");
    def_enum(UDPProfileIMU, sensor::impl::udp_profile_imu_strings, "PROFILE_IMU_");

    auto ShotLimitingStatus = py::enum_<sensor::ShotLimitingStatus>(m, "ShotLimitingStatus", "Shot Limiting Status.");
    def_enum(ShotLimitingStatus, sensor::impl::shot_limiting_status_strings);

    auto ThermalShutdownStatus = py::enum_<sensor::ThermalShutdownStatus>(m, "ThermalShutdownStatus", "Thermal Shutdown Status.");
    def_enum(ThermalShutdownStatus, sensor::impl::thermal_shutdown_status_strings);

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

    m.def("init_logger",
        [](const std::string& log_level, const std::string& log_file_path,
           bool rotating, int max_size_in_bytes, int max_files) {
            return sensor::init_logger(log_level, log_file_path, rotating, max_size_in_bytes,
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
        py::arg("rotating") = false, py::arg("max_size_in_bytes") = 0, py::arg("max_files") = 0);

    m.def("set_config", [] (const std::string& hostname, const sensor_config& config, bool persist,  bool udp_dest_auto, bool force_reinit) {
        uint8_t config_flags = 0;
        if (persist) config_flags |= ouster::sensor::CONFIG_PERSIST;
        if (udp_dest_auto) config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
        if (force_reinit) config_flags |= ouster::sensor::CONFIG_FORCE_REINIT;
        if (!sensor::set_config(hostname, config, config_flags)) {
            throw std::runtime_error("Error setting sensor config.");
        }
    }, R"(
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
        )", py::arg("hostname"), py::arg("config"), py::arg("persist") = false, py::arg("udp_dest_auto") = false,
            py::arg("force_reinit") = false);

    m.def("convert_to_legacy", &ouster::sensor::convert_to_legacy , R"(
        Convert a non-legacy metadata in string format to legacy

        Args:
            metadata: non-legacy metadata string

        Returns:
            returns legacy string representation of metadata.
            )", py::arg("metadata"));

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
        .def(py::init<std::string, int, int, size_t>(), py::arg("hostname"),
             py::arg("lidar_port"), py::arg("imu_port"),
             py::arg("capacity") = 128)
        .def(py::init<std::string, std::string, sensor::lidar_mode,
                      sensor::timestamp_mode, int, int, int, size_t>(),
             py::arg("hostname"), py::arg("udp_dest_host"),
             py::arg("mode") = sensor::lidar_mode::MODE_1024x10,
             py::arg("timestamp_mode") =
                 sensor::timestamp_mode::TIME_FROM_INTERNAL_OSC,
             py::arg("lidar_port") = 0, py::arg("imu_port") = 0,
             py::arg("timeout_sec") = 10, py::arg("capacity") = 128)
        .def("get_metadata", &BufferedUDPSource::get_metadata,
             py::arg("timeout_sec") = 10, py::arg("legacy") = true)
        .def("shutdown", &BufferedUDPSource::shutdown)
        .def("consume",
             [](BufferedUDPSource& self, LidarPacket& lp, ImuPacket& ip,
                float timeout_sec) {
                 using fsec = chrono::duration<float>;

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
                     res = self.consume(lp, ip, poll_interval);
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
    py::class_<LidarScan>(m, "LidarScan", R"(
        Represents a single "scan" or "frame" of lidar data.

        This is a "struct of arrays" representation of lidar data. Column headers are
        stored as contiguous W element arrays, while fields are WxH arrays. Channel
        fields are staggered, so the ith column header value corresponds to the ith
        column of data in each field.
        )")
        // TODO: Python and C++ API differ in h/w order for some reason
        .def(
            "__init__",
            [](LidarScan& self, size_t h, size_t w) {
                new (&self) LidarScan(w, h);
            },
            R"(

        Default constructor creates a 0 x 0 scan

        Args:
            height: height of scan
            width: width of scan

        Returns:
            New LidarScan of 0x0 expecting fields of the LEGACY profile

        )",
            py::arg("h"), py::arg("w"))
        .def(
            "__init__",
            [](LidarScan& self, size_t h, size_t w,
               sensor::UDPProfileLidar profile, size_t columns_per_packet) {
                new (&self) LidarScan(w, h, profile, columns_per_packet);
            },
            R"(

        Initialize a scan with the default fields for a particular udp profile

        Args:
            height: height of LidarScan, i.e., number of channels
            width: width of LidarScan
            profile: udp profile

        Returns:
            New LidarScan of specified dimensions expecting fields of specified profile

         )",
            py::arg("w"), py::arg("h"), py::arg("profile"),
            py::arg("columns_per_packet") = DEFAULT_COLUMNS_PER_PACKET)
        .def(
            "__init__",
            [](LidarScan& self, size_t h, size_t w,
               const std::map<sensor::ChanField, py::object>& field_types,
               size_t columns_per_packet) {
                std::map<sensor::ChanField, sensor::ChanFieldType> ft;
                for (const auto& kv : field_types) {
                    auto dt = py::dtype::from_args(kv.second);
                    ft[kv.first] = field_type_of_dtype(dt);
                }
                new (&self)
                    LidarScan(w, h, ft.begin(), ft.end(), columns_per_packet);
            },
            R"(
        Initialize a scan with a custom set of fields

        Args:
            height: height of LidarScan, i.e., number of channels
            width: width of LidarScan
            fields_dict: dict where keys are ChanFields and values are type, e.g., {client.ChanField.SIGNAL: np.uint32}

        Returns:
            New LidarScan of specified dimensions expecting fields specified by dict

         )",
            py::arg("w"), py::arg("h"), py::arg("field_types"),
            py::arg("columns_per_packet") = DEFAULT_COLUMNS_PER_PACKET)
        .def_readonly("w", &LidarScan::w,
                      "Width or horizontal resolution of the scan.")
        .def_readonly("h", &LidarScan::h,
                      "Height or vertical resolution of the scan.")
        .def_readwrite(
            "frame_id", &LidarScan::frame_id,
            "Corresponds to the frame id header in the packet format.")
        .def_readwrite(
            "frame_status", &LidarScan::frame_status,
            "Information from the packet header which corresponds to a frame.")
        .def(
            "complete",
            [](const LidarScan& self,
               nonstd::optional<sensor::ColumnWindow> window) {
                if (!window) {
                    window = {0, static_cast<int>(self.w) - 1};
                }
                return self.complete(window.value());
            },
            py::arg("window") =
                static_cast<nonstd::optional<sensor::ColumnWindow>>(
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
        .def("shot_limiting", &LidarScan::shot_limiting,
             "The frame shot limiting status.")
        .def("thermal_shutdown", &LidarScan::thermal_shutdown,
             "The frame thermal shutdown status.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "timestamp",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint64_t>(), self.w,
                                 self.timestamp().data(), py::cast(self));
            },
            "The measurement timestamp header as a W-element numpy array.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "packet_timestamp",
            [](LidarScan& self) {
                return py::array(
                    py::dtype::of<uint64_t>(), self.packet_timestamp().rows(),
                    self.packet_timestamp().data(), py::cast(self));
            },
            "The host timestamp header as a numpy array with "
            "W/columns-per-packet entries.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "measurement_id",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint16_t>(), self.w,
                                 self.measurement_id().data(), py::cast(self));
            },
            "The measurement id header as a W-element numpy array.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "status",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint32_t>(), self.w,
                                 self.status().data(), py::cast(self));
            },
            "The measurement status header as a W-element numpy array.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "pose",
            [](LidarScan& self) {
                return py::array(
                    py::dtype::of<double>(),
                    std::vector<size_t>{
                        static_cast<size_t>(self.timestamp().size()), 4, 4},
                    self.pose().data()->data(), py::cast(self));
            },
            "The pose vector of 4x4 homogeneous matrices (per each timestamp).")
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
        .def("__repr__",
             [](const LidarScan& self) {
                 std::stringstream ss;
                 ss << "<ouster.client._client.LidarScan @" << (void*)&self
                    << ">";
                 return ss.str();
             })
        .def("__str__", [](const LidarScan& self) { return to_string(self); })
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
        .def("__call__",
             [](ScanBatcher& self, py::buffer& buf, LidarScan& ls) {
                 uint8_t* ptr = getptr(self.pf.lidar_packet_size, buf);
                 return self(ptr, 0, ls);
             })
        .def(
            "__call__",
            [](ScanBatcher& self, py::buffer& buf, uint64_t ts, LidarScan& ls) {
                uint8_t* ptr = getptr(self.pf.lidar_packet_size, buf);
                return self(ptr, ts, ls);
            })
        .def("__call__", [](ScanBatcher& self, LidarPacket& packet,
                            LidarScan& ls) { return self(packet, ls); });

    // XYZ Projection
    py::class_<XYZLut>(m, "XYZLut")
        .def(
            "__init__",
            [](XYZLut& self, const sensor_info& sensor, bool use_extrinsics) {
                new (&self) XYZLut{};
                if (use_extrinsics) {
                    // apply extrinsics after lidar_to_sensor_transform so the
                    // resulting LUT will produce the coordinates in
                    // "extrinsics frame" instead of "sensor frame"
                    mat4d ext_transform = sensor.extrinsic;
                    ext_transform(0, 3) /= sensor::range_unit;
                    ext_transform(1, 3) /= sensor::range_unit;
                    ext_transform(2, 3) /= sensor::range_unit;
                    ext_transform =
                        ext_transform * sensor.lidar_to_sensor_transform;
                    self = make_xyz_lut(
                        sensor.format.columns_per_frame,
                        sensor.format.pixels_per_column, sensor::range_unit,
                        sensor.beam_to_lidar_transform, ext_transform,
                        sensor.beam_azimuth_angles,
                        sensor.beam_altitude_angles);
                } else {
                    self = make_xyz_lut(sensor);
                }
            },
            py::arg("info"), py::arg("use_extrinsics") = false)
        .def("__call__",
             [](const XYZLut& self, Eigen::Ref<img_t<uint32_t>>& range) {
                 return cartesian(range, self);
             })
        .def("__call__", [](const XYZLut& self, const LidarScan& scan) {
            return cartesian(scan, self);
        });

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

    m.def(
        "get_field_types",
        [](const sensor::sensor_info& info) {
            auto field_types = ouster::get_field_types(info);
            std::map<sensor::ChanField, py::dtype> field_types_res{};
            for (const auto& f : field_types) {
                auto dtype = dtype_of_field_type(f.second);
                field_types_res.emplace(f.first, dtype);
            }
            return field_types_res;
        },
        R"(
        Extracts LidarScan fields with types for a given SensorInfo

        Args:
            info: sensor metadata for which to find fields types

        Returns:
            returns field types
            )",
        py::arg("info"));

    m.def(
        "get_field_types",
        [](const LidarScan& ls) {
            auto field_types = ouster::get_field_types(ls);
            std::map<sensor::ChanField, py::dtype> field_types_res{};
            for (const auto& f : field_types) {
                auto dtype = dtype_of_field_type(f.second);
                field_types_res.emplace(f.first, dtype);
            }
            return field_types_res;
        },
        R"(
        Extracts LidarScan fields with types for a given lidar scan ``ls``

        Args:
            ls: lidar scan from which to get field types

        Returns:
            returns field types
            )",
        py::arg("lidar_scan"));

    m.def(
        "get_field_types",
        [](sensor::UDPProfileLidar profile) {
            auto field_types = ouster::get_field_types(profile);
            std::map<sensor::ChanField, py::dtype> field_types_res{};
            for (const auto& f : field_types) {
                auto dtype = dtype_of_field_type(f.second);
                field_types_res.emplace(f.first, dtype);
            }
            return field_types_res;
        },
        R"(
        Extracts LidarScan fields with types for a given ``udp_profile_lidar``

        Args:
            udp_profile_lidar: lidar profile from which to get field types

        Returns:
            returns field types
            )",
        py::arg("udp_profile_lidar"));

    using ouster::sensor::Packet;
    py::class_<Packet>(m, "_Packet")
        .def(py::init<int>(), py::arg("size") = 65536)
        // direct access to timestamp field
        .def_readwrite("_host_timestamp", &Packet::host_timestamp)
        // access via seconds
        .def_property(
            "capture_timestamp",
            [](Packet& self) -> nonstd::optional<double> {
                if (self.host_timestamp) {
                    return self.host_timestamp / 1e9;
                } else {
                    return nonstd::nullopt;
                }
            },
            [](Packet& self, nonstd::optional<double> v) {
                if (v) {
                    self.host_timestamp = static_cast<uint64_t>(*v * 1e9);
                } else {
                    self.host_timestamp = 0;
                }
            })
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "_data",
            // we have to use cpp_function here because py::keep_alive
            // does not work with pybind11 def_property methods due to a bug:
            // https://github.com/pybind/pybind11/issues/4236
            py::cpp_function(
                [](Packet& self) {
                    return py::array(py::dtype::of<uint8_t>(), self.buf.size(),
                                     self.buf.data(), py::cast(self));
                },
                py::keep_alive<0, 1>()));

    py::class_<LidarPacket, Packet>(m, "_LidarPacket")
        .def(py::init<int>(), py::arg("size") = 65536);

    py::class_<ImuPacket, Packet>(m, "_ImuPacket")
        .def(py::init<int>(), py::arg("size") = 65536);

    using ouster::sensor::impl::FieldInfo;
    py::class_<FieldInfo>(m, "FieldInfo")
        .def("__init__",
             [](FieldInfo& self, py::object dt, size_t offset, uint64_t mask,
                int shift) {
                 new (&self)
                     FieldInfo{field_type_of_dtype(py::dtype::from_args(dt)),
                               offset, mask, shift};
             })
        .def_property_readonly("ty_tag",
                               [](const FieldInfo& self) {
                                   return dtype_of_field_type(self.ty_tag);
                               })
        .def_readwrite("offset", &FieldInfo::offset)
        .def_readwrite("mask", &FieldInfo::mask)
        .def_readwrite("shift", &FieldInfo::shift);

    m.def("add_custom_profile", &ouster::sensor::add_custom_profile);

    m.attr("__version__") = ouster::SDK_VERSION;
}

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
#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>
#if ((SPDLOG_VER_MAJOR == 1) && (SPDLOG_VER_MINOR <= 5))
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

#include "common.h"
#include "ouster/client.h"
#include "ouster/image_processing.h"
#include "ouster/impl/build.h"
#include "ouster/impl/packet_writer.h"
#include "ouster/impl/profile_extension.h"
#include "ouster/lidar_scan.h"
#include "ouster/metadata.h"
#include "ouster/sensor_client.h"
#include "ouster/sensor_http.h"
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"

namespace py = pybind11;
namespace chrono = std::chrono;
using spdlog::sinks::base_sink;

using ouster::sensor::calibration_status;
using ouster::sensor::data_format;
using ouster::sensor::ImuPacket;
using ouster::sensor::LidarPacket;
using ouster::sensor::Packet;
using ouster::sensor::packet_format;
using ouster::sensor::product_info;
using ouster::sensor::sensor_config;
using ouster::sensor::sensor_info;
using ouster::sensor::impl::packet_writer;
using ouster::sensor::util::SensorHttp;
using namespace ouster;

using client_shared_ptr = std::shared_ptr<sensor::client>;
PYBIND11_MAKE_OPAQUE(client_shared_ptr);

inline py::object from_json(const Json::Value& j) {
    switch (j.type()) {
        case Json::nullValue:
            return py::none();
        case Json::intValue:
        case Json::uintValue:
            return py::int_(j.asInt64());
        case Json::realValue:
            return py::float_(j.asDouble());
        case Json::stringValue:
            return py::str(j.asString());
        case Json::booleanValue:
            return py::bool_(j.asBool());
        case Json::arrayValue: {
            py::list obj(j.size());
            for (std::size_t i = 0; i < j.size(); i++) {
                obj[i] = from_json(j[(int)i]);
            }
            return obj;
        }
        case Json::objectValue: {
            py::dict obj;
            for (const auto& member : j.getMemberNames()) {
                obj[py::str(member)] = from_json(j[member]);
            }
            return obj;
        }
    }
    throw std::runtime_error("Unhandled Json Type");
}

namespace pybind11 {
namespace detail {
template <typename T>
struct type_caster<nonstd::optional<T>> : optional_caster<nonstd::optional<T>> {
};
template <>
struct type_caster<Json::Value> {
   public:
    PYBIND11_TYPE_CASTER(Json::Value, _("Value"));

    bool load(handle /*src*/, bool) {
        throw std::runtime_error("Unimplemented");
    }

    static handle cast(Json::Value src, return_value_policy /* policy */,
                       handle /* parent */) {
        object obj = from_json(src);
        return obj.release();
    }
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
extern Table<UDPProfileLidar, const char*, MAX_NUM_PROFILES>
    udp_profile_lidar_strings;
extern Table<UDPProfileIMU, const char*, 1> udp_profile_imu_strings;
extern Table<ShotLimitingStatus, const char*, 10> shot_limiting_status_strings;
extern Table<ThermalShutdownStatus, const char*, 2>
    thermal_shutdown_status_strings;
extern Table<FullScaleRange, const char*, 2> full_scale_range_strings;
extern Table<ReturnOrder, const char*, 5> return_order_strings;
extern const Table<FieldClass, const char*, 4> field_class_strings;

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
    else if (dt.is(py::dtype::of<int8_t>()))
        return sensor::ChanFieldType::INT8;
    else if (dt.is(py::dtype::of<int16_t>()))
        return sensor::ChanFieldType::INT16;
    else if (dt.is(py::dtype::of<int32_t>()))
        return sensor::ChanFieldType::INT32;
    else if (dt.is(py::dtype::of<int64_t>()))
        return sensor::ChanFieldType::INT64;
    else if (dt.is(py::dtype::of<float>()))
        return sensor::ChanFieldType::FLOAT32;
    else if (dt.is(py::dtype::of<double>()))
        return sensor::ChanFieldType::FLOAT64;
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
        case sensor::ChanFieldType::INT8:
            return py::dtype::of<int8_t>();
        case sensor::ChanFieldType::INT16:
            return py::dtype::of<int16_t>();
        case sensor::ChanFieldType::INT32:
            return py::dtype::of<int32_t>();
        case sensor::ChanFieldType::INT64:
            return py::dtype::of<int64_t>();
        case sensor::ChanFieldType::FLOAT32:
            return py::dtype::of<float>();
        case sensor::ChanFieldType::FLOAT64:
            return py::dtype::of<double>();
        default:
            throw std::invalid_argument(
                "Invalid field_type for conversion to dtype");
    }
    return py::dtype();  // unreachable ...
}

/**
 * Retrieves numpy shape from field descriptor
 */
py::array::ShapeContainer shape_of_descriptor(const FieldDescriptor& rd) {
    /**
     * Throws if shape is absent i.e. field is not declared as array; we are
     * currently not handling those cases, change if/when the usecase arises
     */
    if (rd.shape.size() == 0) {
        throw std::invalid_argument("Field is not an array");
    }
    return py::array::ShapeContainer(rd.shape.begin(), rd.shape.end());
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

    void operator()(const packet_writer& self, LidarPacket& p,
                    const std::string& i, Field field) {
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

/**
 * Applies a set of 4x4 pose transformations to a collection of 3D points,
 * reshapes the input into appropriate Eigen matrices, and invokes the C++
 * version of the `dewarp` function for performing the transformations.
 *
 * This function is designed to convert NumPy input arrays from Python into
 * Eigen matrices, which are suitable for efficient matrix operations in C++.
 * The 3D points are passed in the shape of (H, W, 3), and the 4x4 pose matrices
 * are passed in the shape of (W, 4, 4).
 *
 * @param[in] points A NumPy array of shape (H, W, 3) representing the 3D
 * points.
 *               - H: Number of columns (groups of points)
 *               - W: Number of points per column
 *               - 3: 3D coordinates (x, y, z)
 *
 * @param[in] poses A NumPy array of shape (W, 4, 4) representing the 4x4 pose
 * matrices.
 *              - W: Number of pose matrices
 *              - 4x4: The transformation matrices
 *
 * @return A NumPy array of shape (H, W, 3) containing the dewarped 3D points
 * after applying the corresponding 4x4 transformation matrices to the points.
 *
 */

py::array_t<double> dewarp(const py::array_t<double>& points,
                           const py::array_t<double>& poses) {
    auto poses_buf = poses.request();
    auto points_buf = points.request();

    // Validate input dimensions for poses
    if (poses_buf.ndim != 3 || poses_buf.shape[1] != 4 ||
        poses_buf.shape[2] != 4) {
        throw std::runtime_error("Invalid shape for poses, expected (W, 4, 4)");
    }

    // Validate input dimensions for points
    if (points_buf.ndim != 3 || points_buf.shape[2] != 3) {
        throw std::runtime_error(
            "Invalid shape for points, expected (H, W, 3)");
    }

    const int num_poses = poses_buf.shape[0];            // W: 1024, 2048 etc
    const int num_rows = points_buf.shape[0];            // H: 64, 128 etc
    const int num_points_per_col = points_buf.shape[1];  // W
    const int point_dim = 3;

    if (num_points_per_col != num_poses) {
        throw std::runtime_error(
            "Number of points per set must match the number of poses");
    }

    // poses reshape into (W, 16)
    Eigen::Map<pose_util::Poses> poses_mat(static_cast<double*>(poses_buf.ptr),
                                           num_poses, 16);

    auto result = py::array_t<double>({num_rows, num_poses, point_dim});
    auto result_buf = result.request();
    Eigen::Map<pose_util::Points> dewarped_points(
        static_cast<double*>(result_buf.ptr), num_rows * num_poses, point_dim);

    if (points.flags() & py::array_t<double>::c_style) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>
            points_mat(static_cast<double*>(points_buf.ptr),
                       num_rows * num_poses, point_dim);
        pose_util::dewarp(dewarped_points, points_mat, poses_mat);
    } else {
        // TODO[UN]: Optimize for the ColMajor case
        // needs tests in place
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::ColMajor>>
            points_mat_col_major(static_cast<double*>(points_buf.ptr),
                                 num_rows * num_poses, point_dim);
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> points_mat =
            points_mat_col_major;
        pose_util::dewarp(dewarped_points, points_mat, poses_mat);
    }

    return result;
}

/**
 * Applies a single of 4x4 pose transformations to a collection of 3D points,
 * reshapes the input into appropriate Eigen matrices, and invokes the C++
 * version of the `transfrom` function for performing the transformations.
 *
 * This function is designed to convert NumPy input arrays from Python into
 * Eigen matrices, which are suitable for efficient matrix operations in C++.
 * The 3D points are passed in the shape of (H, W, 3), and the single 4x4 pose
 * matrics in the shape of (4, 4).
 *
 * @param[in] points A NumPy array of shape (H, W, 3), or (N, 3)
 * representing the 3D points.
 *               - H: Number of columns (groups of points)
 *               - W: Number of points per column
 *               - 3: 3D coordinates (x, y, z)
 *
 * @param[in] poses A NumPy array of shape (4, 4) representing the 4x4 pose
 * matrices.
 *              - 4x4: The transformation matrices
 *
 * @return A NumPy array of shape (H, W, 3) or (N, 3) containing the transformed
 * 3D points after applying the corresponding 4x4 transformation matrices to the
 * points.
 *
 */
py::array_t<double> transform(const py::array_t<double>& points,
                              const py::array_t<double>& pose) {
    // Ensure the pose is a 4x4 matrix
    if (pose.ndim() != 2 || pose.shape(0) != 4 || pose.shape(1) != 4) {
        throw std::runtime_error("pose array must have shape (4, 4)");
    }

    Eigen::Map<const pose_util::Pose> pose_eigen(pose.data());

    // Handle case where points is a 2D array: (N, 3)
    if (points.ndim() == 2 && points.shape(1) == 3) {
        const int n = points.shape(0);

        Eigen::Map<const pose_util::Points> points_eigen(points.data(), n, 3);

        auto result = py::array_t<double>({n, 3});
        auto result_buf = result.request();
        Eigen::Map<pose_util::Points> transformed(
            static_cast<double*>(result_buf.ptr), n, 3);

        pose_util::transform(transformed, points_eigen, pose_eigen);
        return result;
    }

    // Handle case where points is a 3D array: (H, W, 3)
    else if (points.ndim() == 3 && points.shape(2) == 3) {
        const int h = points.shape(0);
        const int w = points.shape(1);

        Eigen::Map<const pose_util::Points> points_eigen(points.data(), h * w,
                                                         3);

        auto result = py::array_t<double>({h, w, 3});
        auto result_buf = result.request();
        Eigen::Map<pose_util::Points> transformed(
            static_cast<double*>(result_buf.ptr), h * w, 3);

        pose_util::transform(transformed, points_eigen, pose_eigen);
        return result;
    } else {
        throw std::invalid_argument(
            "points array must have shape (n, 3) or (h, w, 3)");
    }
}

void init_client(py::module& m, py::module&) {
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
        .def(py::init([](const sensor_info& info) {
            return new packet_format(info);
        }))
        .def(py::init([](sensor::UDPProfileLidar profile,
                         size_t pixels_per_column,
                         size_t columns_per_packet) {
                        return new packet_format(profile, pixels_per_column,
                                                 columns_per_packet);
                    }))
        .def_static("from_metadata",
                    [](const sensor_info& info) -> const packet_format& {
                        return sensor::get_format(info);
                    }, py::return_value_policy::reference)
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
        .def_readonly("max_frame_id", &packet_format::max_frame_id)

        .def("field_value_mask", &packet_format::field_value_mask)
        .def("field_bitness", &packet_format::field_bitness)

        .def("crc", [](packet_format& pf, py::buffer buf) {
            return pf.crc(getptr(pf.lidar_packet_size, buf));
        })

        .def("calculate_crc", [](packet_format& pf, py::buffer buf) {
            return pf.calculate_crc(getptr(pf.lidar_packet_size, buf));
        })

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

        .def("alert_flags", [](packet_format& pf, py::buffer buf) {
            return pf.alert_flags(getptr(pf.lidar_packet_size, buf));
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

        .def("packet_field", [](packet_format& pf, const std::string& f, py::buffer buf) -> py::array {
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
                    throw py::key_error("Invalid type for PacketFormat");
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
        .def("set_alert_flags", [](const packet_writer& self, LidarPacket& p, uint8_t alert_flags) {
            self.set_alert_flags(p.buf.data(), alert_flags);
        })
        .def("set_shutdown_countdown", [](const packet_writer& self, LidarPacket& p, uint8_t shutdown_countdown) {
            self.set_shutdown_countdown(p.buf.data(), shutdown_countdown);
        })
        .def("set_shot_limiting_countdown", [](const packet_writer& self, LidarPacket& p, uint8_t shot_limiting_countdown) {
            self.set_shot_limiting_countdown(p.buf.data(), shot_limiting_countdown);
        })
        .def("set_field", set_field<uint8_t>{})
        .def("set_field", set_field<uint16_t>{})
        .def("set_field", set_field<uint32_t>{})
        .def("set_field", set_field<uint64_t>{})
        .def("set_field", set_field<int8_t>{})
        .def("set_field", set_field<int16_t>{})
        .def("set_field", set_field<int32_t>{})
        .def("set_field", set_field<int64_t>{})
        .def("set_field", set_field<float>{})
        .def("set_field", set_field<double>{});

    m.def("scan_to_packets",
          [](const LidarScan& ls, const packet_writer& pw, uint32_t init_id, uint64_t prod_sn) {
              std::vector<LidarPacket> packets;

              auto append_pypacket = [&](const LidarPacket& packet) {
                  packets.push_back(packet);
              };

              auto iter = make_lambda_iter(append_pypacket);
              impl::scan_to_packets(ls, pw, iter, init_id, prod_sn);
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

    // Product Line
    py::class_<product_info>(m, "ProductInfo")
        .def_readonly("full_product_info", &product_info::full_product_info, R"(
        The original full product info string.

        :type: string
        )")
        .def_readonly("form_factor", &product_info::form_factor, R"(
        The form factor of the product.

        :type: string
        )")
        .def_readonly("short_range", &product_info::short_range, R"(
        If the product is a short range make.

        :type: bool
        )")
        .def_readonly("beam_config", &product_info::beam_config, R"(
        The beam configuration of the product..

        :type: string
        )")
        .def_readonly("beam_count", &product_info::beam_count, R"(
        Number of beams

        :type: int
        )")
        .def("__eq__", [](const product_info& i, const product_info& j) { return i == j; })
        .def("__str__", [](const product_info& i) {return to_string(i);});

    // Sensor Info
    py::class_<sensor_info>(m, "SensorInfo", R"(
        Sensor Info required to interpret UDP data streams.

        See the sensor documentation for the meaning of each property.
        )")
        .def(py::init<>(), R"(
            Construct an empty metadata.
        )")
        .def(py::init([](const std::string& s, bool skip_beam_validation) {
            return new sensor_info(s, skip_beam_validation);
        }), py::arg("s"), py::arg("skip_beam_validation") = false,  R"(
        Args:
            s (str): json string to parse
            skip_beam_validation (boolean): if true, skip validating beam angles
        )")
        .def_readwrite("sn", &sensor_info::sn, "Sensor serial number.")
        .def_readwrite("fw_rev", &sensor_info::fw_rev, "Sensor firmware revision.")
        .def_property("mode", [](const sensor_info& self) {
            ouster::sensor::logger().warn("sensor_info.mode is deprecated, use sensor_info.config.lidar_mode instead.");
            return self.config.lidar_mode;
        },
        [](sensor_info& self, ouster::sensor::lidar_mode mode) {
            ouster::sensor::logger().warn("sensor_info.mode is deprecated, use sensor_info.config.lidar_mode instead.");
            self.config.lidar_mode = mode;
        })
        //.def_readwrite("mode", &sensor_info::config::mode, "Lidar mode, e.g., 1024x10.")
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
        .def_property("udp_port_lidar", [](const sensor_info& self) {
            ouster::sensor::logger().warn("sensor_info.udp_port_lidar is deprecated, use sensor_info.config.udp_port_lidar instead.");
            return self.config.udp_port_lidar;
        },
        [](sensor_info& self, int port) {
            ouster::sensor::logger().warn("sensor_info.udp_port_lidar is deprecated, use sensor_info.config.udp_port_lidar instead.");
            self.config.udp_port_lidar = port;
        })
        .def_property("udp_port_imu", [](const sensor_info& self) {
            ouster::sensor::logger().warn("sensor_info.udp_port_imu is deprecated, use sensor_info.config.udp_port_imu instead.");
            return self.config.udp_port_imu;
        },
        [](sensor_info& self, int port) {
            ouster::sensor::logger().warn("sensor_info.udp_port_imu is deprecated, use sensor_info.config.udp_port_imu instead.");
            self.config.udp_port_imu = port;
        })
        .def_readwrite("build_date", &sensor_info::build_date, "Build date")
        .def_readwrite("image_rev", &sensor_info::image_rev, "Image rev")
        .def_readwrite("prod_pn", &sensor_info::prod_pn, "Prod pn")
        .def_readwrite("status", &sensor_info::status, "sensor status")
        .def_readwrite("cal", &sensor_info::cal, "sensor calibration")
        .def_readwrite("config", &sensor_info::config, "sensor config")
        .def_readwrite("user_data", &sensor_info::user_data, "sensor user data")
        .def_static("from_default", &sensor::default_sensor_info, R"(
        Create gen-1 OS-1-64 SensorInfo populated with design values.
        )")
        .def("to_json_string", &sensor_info::to_json_string, R"( Return metadata string made from current entries"
        )")
        .def("has_fields_equal", &sensor_info::has_fields_equal, R"(Compare public fields")")
        // only uncomment for debugging purposes!! story for general use and output is not filled
        //.def("__str__", [](const sensor_info& i) { return to_string(i); })
        .def_property_readonly("w", [](const sensor_info& self) { return self.w(); }, R"(returns the width of a frame (equivalent to format.columns_per_frame))")
        .def_property_readonly("h", [](const sensor_info& self) { return self.h(); }, R"(returns the height of a frame (equivalent to format.pixels_per_column))")
        .def("__eq__", [](const sensor_info& i, const sensor_info& j) { return i == j; })
        .def("__repr__", [](const sensor_info& self) {
            const auto mode = self.config.lidar_mode ? to_string(self.config.lidar_mode.value()) : std::to_string(self.format.fps) + "fps";
            return "<ouster.sdk.client.SensorInfo " + self.prod_line + " " +
                self.sn + " " + self.fw_rev + " " + mode + ">";
        })
        .def("get_version", [](const sensor_info& self) { return self.get_version(); }, R"(Get parsed sensor version)")
        .def("get_product_info", [](const sensor_info& self) { return self.get_product_info(); }, R"(Get parsed product info)")
        .def("__copy__", [](const sensor_info& self) { return sensor_info{self}; })
        .def("__deepcopy__", [](const sensor_info& self, py::dict) { return sensor_info{self}; });


    // Enums
    auto lidar_mode = py::enum_<sensor::lidar_mode>(m, "LidarMode", R"(
        Possible Lidar Modes of sensor.

        Determines to horizontal and vertical resolution rates of sensor. See
        sensor documentation for details.)");
    def_enum(lidar_mode, sensor::impl::lidar_mode_strings, "MODE_");
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

    auto ReturnOrder = py::enum_<sensor::ReturnOrder>(m, "ReturnOrder", R"(
        Sensor return order.

        See sensor documentation for details.)");
    def_enum(ReturnOrder, sensor::impl::return_order_strings, "ORDER_");

    auto FullScaleRange = py::enum_<sensor::FullScaleRange>(m, "FullScaleRange", R"(
        IMU output scale range.

        See sensor documentation for details.)");
    def_enum(FullScaleRange, sensor::impl::full_scale_range_strings, "FSR_");

    auto NMEABaudRate = py::enum_<sensor::NMEABaudRate>(m, "NMEABaudRate", R"(
        Expected baud rate sensor attempts to decode for NMEA UART input $GPRMC messages.)");
    def_enum(NMEABaudRate, sensor::impl::nmea_baud_rate_strings);

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

    auto FieldClass = py::enum_<ouster::FieldClass>(m, "FieldClass", "LidarScan field classes", py::arithmetic());
    def_enum(FieldClass, sensor::impl::field_class_strings);

    // Sensor Config
    py::class_<sensor_config>(m, "SensorConfig", R"(
        Corresponds to sensor config parameters. Please see sensor documentation for the meaning of each property.
        )")
        .def(py::init<>(), "Construct an empty SensorConfig.")
        .def(py::init([](const std::string& s) {
            auto config = new sensor_config{};
            *config = sensor::parse_config(s);
            return config;
        }), py::arg("s"), R"(Construct a SensorConfig from a json string.
        Args:
            s (str): json string to parse
        )")
        .def_readwrite("udp_dest", &sensor_config::udp_dest, "Destination to which sensor sends UDP traffic.")
        .def_readwrite("udp_port_lidar", &sensor_config::udp_port_lidar, "Port on UDP destination to which lidar data will be sent.")
        .def_readwrite("udp_port_imu", &sensor_config::udp_port_imu, "Port on UDP destination to which IMU data will be sent.")
        .def_readwrite("timestamp_mode", &sensor_config::timestamp_mode, "Timestamp mode of sensor. See class TimestampMode.")
        .def_readwrite("lidar_mode", &sensor_config::lidar_mode, "Horizontal and Vertical Resolution rate of sensor as mode, e.g., 1024x10. See class LidarMode.")
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
        .def_readwrite("gyro_fsr", &sensor_config::gyro_fsr, "The gyro full scale measurement range to use. See sensor documentation for details.")
        .def_readwrite("accel_fsr", &sensor_config::accel_fsr, "The accelerometer full scale measurement range to use. See sensor documentation for details.")
        .def_readwrite("return_order", &sensor_config::return_order, "The priority of sensor returns to output. See sensor documentation for details.")
        .def_readwrite("min_range_threshold_cm", &sensor_config::min_range_threshold_cm, "The minimum detection range of the sensor in cm. See sensor documentation for details.")
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
        .def_readwrite("major", &util::version::major)
        .def_readwrite("minor", &util::version::minor)
        .def_readwrite("patch", &util::version::patch)
        .def_readwrite("stage", &util::version::stage)
        .def_readwrite("machine", &util::version::machine)
        .def_readwrite("prerelease", &util::version::prerelease)
        .def_readwrite("build", &util::version::build)
        .def_static("from_string", &util::version_from_string);

    m.attr("invalid_version") = util::invalid_version;

    m.attr("min_version") = sensor::min_version;

    // clang-format on

    // client
    py::class_<client_shared_ptr>(m, "SensorConnection")
        .def(py::init([](std::string hostname, int lidar_port,
                         int imu_port) -> client_shared_ptr {
                 auto cli = sensor::init_client(hostname, lidar_port, imu_port);
                 if (!cli)
                     throw std::runtime_error(
                         "Failed initializing sensor connection");
                 return cli;
             }),
             py::arg("hostname"), py::arg("lidar_port") = 7502,
             py::arg("imu_port") = 7503)
        .def(py::init([](std::string hostname, std::string udp_dest_host,
                         sensor::lidar_mode lp_mode,
                         sensor::timestamp_mode ts_mode, int lidar_port,
                         int imu_port, int timeout_sec,
                         bool persist_config) -> client_shared_ptr {
                 auto cli = sensor::init_client(
                     hostname, udp_dest_host, lp_mode, ts_mode, lidar_port,
                     imu_port, timeout_sec, persist_config);
                 if (!cli)
                     throw std::runtime_error(
                         "Failed initializing sensor connection");
                 return cli;
             }),
             py::arg("hostname"), py::arg("udp_dest_host"),
             py::arg("mode") = sensor::lidar_mode::MODE_1024x10,
             py::arg("timestamp_mode") =
                 sensor::timestamp_mode::TIME_FROM_INTERNAL_OSC,
             py::arg("lidar_port") = 0, py::arg("imu_port") = 0,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("persist_config") = false)
        .def(
            "poll",
            [](const client_shared_ptr& self,
               int timeout_sec) -> sensor::client_state {
                return sensor::poll_client(*self, timeout_sec);
            },
            py::arg("timeout_sec") = 1)
        .def("read_lidar_packet",
             [](const client_shared_ptr& self, LidarPacket& packet) -> bool {
                 return sensor::read_lidar_packet(*self, packet);
             })
        .def("read_imu_packet",
             [](const client_shared_ptr& self, ImuPacket& packet) -> bool {
                 return sensor::read_imu_packet(*self, packet);
             })
        .def_property_readonly("lidar_port",
                               [](const client_shared_ptr& self) -> int {
                                   return sensor::get_lidar_port(*self);
                               })
        .def_property_readonly("imu_port",
                               [](const client_shared_ptr& self) -> int {
                                   return sensor::get_imu_port(*self);
                               })
        .def(
            "get_metadata",
            [](client_shared_ptr& self, int timeout_sec) -> std::string {
                return sensor::get_metadata(*self, timeout_sec);
            },
            py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("shutdown", [](client_shared_ptr& self) { self.reset(); });

    // New Client
    py::enum_<sensor::ClientEvent::EventType>(m, "ClientEventType",
                                              py::arithmetic())
        .value("Error", sensor::ClientEvent::Error)
        .value("Exit", sensor::ClientEvent::Exit)
        .value("PollTimeout", sensor::ClientEvent::PollTimeout)
        .value("ImuPacket", sensor::ClientEvent::ImuPacket)
        .value("LidarPacket", sensor::ClientEvent::LidarPacket);

    py::class_<sensor::ClientEvent>(m, "ClientEvent")
        .def(py::init())
        .def_readwrite("source", &sensor::ClientEvent::source)
        .def_readwrite("type", &sensor::ClientEvent::type);

    py::class_<sensor::Sensor>(m, "Sensor")
        .def(py::init<const std::string&, const sensor_config&>(),
             py::arg("hostname"), py::arg("desired_config") = sensor_config())
        .def(
            "fetch_metadata",
            [](sensor::Sensor& self, int timeout) {
                return self.fetch_metadata(timeout);
            },
            py::arg("timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("http_client", &sensor::Sensor::http_client)
        .def("desired_config", &sensor::Sensor::desired_config)
        .def("hostname", &sensor::Sensor::hostname);

    py::class_<sensor::SensorClient>(m, "SensorClient")
        .def(py::init([](std::vector<sensor::Sensor> sensors,
                         double config_timeout,
                         double buffer_time) -> sensor::SensorClient* {
                 return new sensor::SensorClient(sensors, config_timeout,
                                                 buffer_time);
             }),
             py::arg("sensors"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("buffer_time") = 0)
        .def(py::init([](std::vector<sensor::Sensor> sensors,
                         std::vector<sensor::sensor_info> metadata,
                         double config_timeout,
                         double buffer_size) -> sensor::SensorClient* {
                 return new sensor::SensorClient(sensors, metadata,
                                                 config_timeout, buffer_size);
             }),
             py::arg("sensors"), py::arg("metadata"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("buffer_time") = 0)
        .def("get_sensor_info",
             [](sensor::SensorClient& self) { return self.get_sensor_info(); })
        .def("flush", &sensor::SensorClient::flush)
        .def("close", &sensor::SensorClient::close)
        .def("buffer_size", &sensor::SensorClient::buffer_size)
        .def(
            "get_packet",
            [](sensor::SensorClient& self, LidarPacket& lp, ImuPacket& ip,
               double timeout) {
                py::gil_scoped_release release;
                auto packet = self.get_packet(lp, ip, timeout);
                return packet;
            },
            py::arg("lidar_packet"), py::arg("imu_packet"),
            py::arg("timeout") = 0.1);

    py::class_<sensor::SensorScanSource>(m, "SensorScanSource")
        .def(py::init([](std::vector<sensor::Sensor> sensors, double timeout,
                         unsigned int queue_size,
                         bool soft_id_check) -> sensor::SensorScanSource* {
                 return new sensor::SensorScanSource(sensors, timeout,
                                                     queue_size, soft_id_check);
             }),
             py::arg("sensors"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def(py::init([](std::vector<sensor::Sensor> sensors,
                         std::vector<sensor::sensor_info> metadata,
                         double timeout, unsigned int queue_size,
                         bool soft_id_check) -> sensor::SensorScanSource* {
                 return new sensor::SensorScanSource(sensors, metadata, timeout,
                                                     queue_size, soft_id_check);
             }),
             py::arg("sensors"), py::arg("metadata"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def(py::init([](std::vector<sensor::Sensor> sensors,
                         std::vector<sensor::sensor_info> metadata,
                         const std::vector<std::vector<FieldType>>& field_types,
                         double timeout, unsigned int queue_size,
                         bool soft_id_check) -> sensor::SensorScanSource* {
                 return new sensor::SensorScanSource(sensors, metadata,
                                                     field_types, timeout,
                                                     queue_size, soft_id_check);
             }),
             py::arg("sensors"), py::arg("metadata"), py::arg("fields"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def("get_sensor_info",
             [](sensor::SensorScanSource& self) {
                 return self.get_sensor_info();
             })
        .def("flush", &sensor::SensorScanSource::flush)
        .def("close", &sensor::SensorScanSource::close)
        .def("dropped_scans", &sensor::SensorScanSource::dropped_scans)
        .def("id_error_count", &sensor::SensorScanSource::id_error_count)
        .def(
            "get_scan",
            [](sensor::SensorScanSource& self, double timeout) {
                py::gil_scoped_release release;
                auto packet = self.get_scan(timeout);
                return packet;
            },
            py::arg("timeout") = 0.1);

    // Client Handle
    py::enum_<sensor::client_state>(m, "ClientState", py::arithmetic())
        .value("TIMEOUT", sensor::client_state::TIMEOUT)
        .value("ERROR", sensor::client_state::CLIENT_ERROR)
        .value("LIDAR_DATA", sensor::client_state::LIDAR_DATA)
        .value("IMU_DATA", sensor::client_state::IMU_DATA)
        .value("EXIT", sensor::client_state::EXIT);

    py::class_<FieldType>(m, "FieldType", R"(
        Describes a field.
    )")
        .def(py::init(
                 [](const std::string& name, py::type dtype,
                    py::tuple extra_dims,
                    size_t flags = (size_t)ouster::FieldClass::PIXEL_FIELD) {
                     return new FieldType(
                         name, field_type_of_dtype(py::dtype::from_args(dtype)),
                         extra_dims.cast<std::vector<size_t>>(),
                         static_cast<ouster::FieldClass>(flags));
                 }),
             R"(
        Construct a FieldType.

        Args:
            name:   name of the field
            dt:     data type of the field, e.g. np.uint8.
            extra_dims: a tuple representing the number of elements.
                in the dimensions beyond width and height,
                for fields with three or more dimensions.
            field_class: indicates whether the field has an entry
                per-packet, per-column, per-scan or per-pixel.
        )",
             py::arg("name"), py::arg("dtype"),
             py::arg("extra_dims") = py::tuple(),
             py::arg("field_class") = ouster::FieldClass::PIXEL_FIELD)
        .def_readwrite("name", &FieldType::name, "The name of the field.")
        .def_readwrite(
            "field_class", &FieldType::field_class,
            R"("field_class - an enum that indicates whether the field has an entry
                per-packet, per-column, per-scan or per-pixel.)")
        .def_property(
            "extra_dims",
            [](FieldType& self) -> py::tuple {
                py::tuple extra_dims_tuple(self.extra_dims.size());
                for (size_t i = 0; i < self.extra_dims.size(); i++) {
                    extra_dims_tuple[i] = self.extra_dims[i];
                }
                return extra_dims_tuple;
            },
            [](FieldType& self, py::tuple extra_dims) {
                self.extra_dims = extra_dims.cast<std::vector<size_t>>();
            },
            R"(A tuple representing the size of extra dimensions
            (if the field is greater than 2 dimensions.))")
        .def_property(
            "element_type",
            [](FieldType& self) -> py::dtype {
                return dtype_of_field_type(self.element_type);
            },
            [](FieldType& self, py::dtype dtype) {
                self.element_type = field_type_of_dtype(dtype);
            },
            "The data type (as a numpy dtype) of the field.")
        .def("__lt__",
             [](const FieldType& u, const FieldType& v) { return u < v; })
        .def("__repr__",
             [](const FieldType& self) {
                 std::stringstream ss;
                 ss << "<ouster.sdk.client.FieldType " << to_string(self)
                    << ">";
                 return ss.str();
             })
        .def("__eq__",
             [](const FieldType& l, const FieldType& r) { return l == r; })
        .def("__str__", [](const FieldType& self) { return to_string(self); });

    // Scans
    py::class_<LidarScan>(m, "LidarScan", R"(
        Represents a single "scan" or "frame" of lidar data.

        This is a "struct of arrays" representation of lidar data. Column headers are
        stored as contiguous W element arrays, while fields are WxH arrays. Channel
        fields are staggered, so the ith column header value corresponds to the ith
        column of data in each field.
        )")
        // TODO: Python and C++ API differ in h/w order for some reason
        .def(py::init([](size_t h, size_t w) { return new LidarScan(w, h); }),
             R"(

        Default constructor creates a 0 x 0 scan

        Args:
            height: height of scan
            width: width of scan

        Returns:
            New LidarScan of 0x0 expecting fields of the LEGACY profile

        )",
             py::arg("h"), py::arg("w"))
        .def(py::init([](size_t h, size_t w, sensor::UDPProfileLidar profile,
                         size_t columns_per_packet) {
                 return new LidarScan(w, h, profile, columns_per_packet);
             }),
             R"(

        Initialize a scan with the default fields for a particular udp profile

        Args:
            height: height of LidarScan, i.e., number of channels
            width: width of LidarScan
            profile: udp profile

        Returns:
            New LidarScan of specified dimensions expecting fields of specified profile

         )",
             py::arg("h"), py::arg("w"), py::arg("profile"),
             py::arg("columns_per_packet") = DEFAULT_COLUMNS_PER_PACKET)
        .def(py::init([](size_t h, size_t w,
                         const std::vector<ouster::FieldType>& field_types,
                         size_t columns_per_packet) {
                 return new LidarScan(w, h, field_types.begin(),
                                      field_types.end(), columns_per_packet);
             }),
             R"(
        Initialize a scan with a custom set of fields

        Args:
            height: height of LidarScan, i.e., number of channels
            width: width of LidarScan
            field_types: list of FieldType that specifies which fields should be present in the scan

        Returns:
            New LidarScan of specified dimensions expecting fields specified by dict

         )",
             py::arg("w"), py::arg("h"), py::arg("field_types"),
             py::arg("columns_per_packet") = DEFAULT_COLUMNS_PER_PACKET)
        .def(py::init([](const sensor_info& sensor_info) {
                 return new LidarScan(sensor_info);
             }),
             R"(
        Initialize a scan with defaults fields and size for a given sensor_info

        Args:
            sensor_info: SensorInfo to construct a scan for

        Returns:
            New LidarScan approprate for the sensor_info

         )",
             py::arg("sensor_info"))
        .def(py::init([](const LidarScan& source,
                         const std::vector<FieldType>& field_types) {
                 return new LidarScan(source, field_types);
             }),
             R"(
        Initialize a lidar scan from another with only the indicated fields.
        Casts, zero pads or removes fields from the original scan if necessary.

        Args:
            source: LidarScan to copy data from
            field_types: list of fields to have in the new scan where keys are ChanFields
                         and values are type, e.g., FieldType(client.ChanField.SIGNAL, np.uint32)

        Returns:
            New LidarScan with selected data copied over or zero padded

         )",
             py::arg("source"), py::arg("field_types"))
        .def(py::init(
                 [](const LidarScan& source) { return new LidarScan(source); }),
             R"(
        Initialize a lidar scan with a copy of the data from another.

        Args:
            source: LidarScan to copy

        Returns:
            New LidarScan with data copied over from provided scan.

         )",
             py::arg("source"))
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
        .def_readwrite("shutdown_countdown", &LidarScan::shutdown_countdown,
                       "Thermal shutdown countdown. Please refer to the "
                       "firmware documentation for more information.")
        .def_readwrite("shot_limiting_countdown",
                       &LidarScan::shot_limiting_countdown,
                       "Shot-limiting countdown. Please refer to the firmware "
                       "documentation for more information.")
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
        .def_property_readonly(
            "packet_count", &LidarScan::packet_count,
            "The number of packets used to produce a full scan given the width "
            "in pixels and the number of columns per "
            "packet.")
        .def(
            "field",
            [](LidarScan& self, const std::string& name) {
                auto&& field = self.field(name);
                auto dt = dtype_of_field_type(field.tag());
                auto shape = shape_of_descriptor(field.desc());
                return py::array(dt, shape, field.get(), py::cast(self));
            },
            R"(
        Return a view of the specified channel field.

        Args:
            name: name of the field to return

        Returns:
            The specified field as a numpy array
        )")
        .def(
            "add_field",
            [](LidarScan& self, const std::string& name, py::type dtype,
               py::tuple shape, size_t field_class) {
                auto ft = field_type_of_dtype(py::dtype::from_args(dtype));
                Field& ptr = self.add_field(
                    FieldType(name, ft, shape.cast<std::vector<size_t>>(),
                              static_cast<ouster::FieldClass>(field_class)));

                auto dt = dtype_of_field_type(ptr.tag());
                auto sh = shape_of_descriptor(ptr.desc());
                return py::array(dt, sh, ptr.get(), py::cast(self));
            },
            R"(
        Adds a new field under specified name

        Args:
            name: name of the field to add
            shape: tuple of ints, shape of the field to add
            dtype: dtype of field to add, e.g. np.uint32
            field_class: class of the field to add, see field_class

        Returns:
            The field as a numpy array
        )",
            py::arg("name"), py::arg("dtype"), py::arg("shape") = py::tuple(),
            py::arg("field_class") = ouster::FieldClass::PIXEL_FIELD)
        .def(
            "add_field",
            [](LidarScan& self, const FieldType& type) {
                Field& ptr = self.add_field(type);

                auto dt = dtype_of_field_type(ptr.tag());
                auto sh = shape_of_descriptor(ptr.desc());
                return py::array(dt, sh, ptr.get(), py::cast(self));
            },
            R"(
        Adds a new field under specified name and type

        Args:
            type: FieldType of the field to add

        Returns:
            The field as a numpy array
        )",
            py::arg("type"))
        .def(
            "add_field",
            [](LidarScan& self, const std::string& name, const py::array& data,
               size_t field_class) -> py::array {
                if (!(data.flags() & py::array::c_style)) {
                    throw std::invalid_argument(
                        "add_field: submitted ndarray is not c-contiguous");
                }

                auto dt = data.dtype();
                std::vector<size_t> shape;
                for (int i = 0; i < data.ndim(); i++) {
                    shape.push_back(data.shape(i));
                }
                auto ft = field_type_of_dtype(dt);
                Field& ptr = self.add_field(
                    name, FieldDescriptor::array(ft, shape),
                    static_cast<ouster::FieldClass>(field_class));
                auto sh = shape_of_descriptor(ptr.desc());
                py::buffer_info info = data.request();
                memcpy(ptr.get(), info.ptr, ptr.bytes());
                return py::array(dt, sh, ptr.get(), py::cast(self));
            },
            R"(
        Adds a new field under the specified name, with the given contents.
        IMPORTANT: this will deep copy the supplied data.

        Args:
            name: the name of the new field
            data: the contents of the new field
            field_class: class of the field to add, see field_class

        Returns:
            The field as a numpy array.
            )",
            py::arg("name"), py::arg("data"),
            py::arg("field_class") = ouster::FieldClass::PIXEL_FIELD)
        .def(
            "del_field",
            [](LidarScan& self, const std::string& name) -> py::array {
                // need a heap allocated Field to pass to python
                Field* field = new Field(std::move(self.del_field(name)));
                auto dt = dtype_of_field_type(field->tag());
                auto shape = shape_of_descriptor(field->desc());

                py::capsule cleanup{field, [](void* p) {
                                        Field* ptr =
                                            reinterpret_cast<Field*>(p);
                                        delete ptr;
                                    }};
                return py::array(dt, shape, field->get(), cleanup);
            },
            R"(
        Release a field from the LidarScan and return it to the user

        Args:
            name: name of the field to drop

        Returns:
            The specified field as a numpy array
        )")
        .def("has_field", &LidarScan::has_field,
             R"(
        Returns true if the LidarScan has a field with the given name

        Args:
            name: name of the field to check for

        Returns:
            True if the field exists in the scan, false otherwise
        )",
             py::arg("name"))
        .def(
            "field_class",
            [](LidarScan& self, const std::string& name) -> ouster::FieldClass {
                return self.field(name).field_class();
            },
            R"(
        Retrieve FieldClass of field

        Args:
            name: name of the field

        Returns:
            FieldClass of the field
        )")
        .def(
            "has_field",
            [](LidarScan& self, const std::string& name) -> bool {
                return self.has_field(name);
            },
            R"(
        Check if a field with a given name exists in the LidarScan.

        Args:
            name: name of the field

        Returns:
            True if the field is present in the LidarScan.
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
            "alert_flags",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint8_t>(),
                                 self.alert_flags().rows(),
                                 self.alert_flags().data(), py::cast(self));
            },
            "The alert flags header as a numpy array with "
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
                auto&& field = self.pose();
                return py::array(py::dtype::of<double>(),
                                 shape_of_descriptor(field.desc()), field.get(),
                                 py::cast(self));
            },
            "The pose vector of 4x4 homogeneous matrices (per each timestamp).")
        .def_property_readonly(
            "fields",
            [](const LidarScan& self) {
                std::vector<std::string> keys;
                for (const auto& f : self.fields()) {
                    keys.push_back(f.first);
                }
                std::sort(keys.begin(), keys.end());
                return keys;
            },
            "Return an list of available fields.")
        .def_property_readonly(
            "field_types",
            [](const LidarScan& self) { return self.field_types(); },
            "Return an list of available fields.")
        .def(
            "get_first_valid_packet_timestamp",
            [](const LidarScan& self) {
                return self.get_first_valid_packet_timestamp();
            },
            "Return first valid packet timestamp in the scan.")
        .def(
            "get_first_valid_column_timestamp",
            [](const LidarScan& self) {
                return self.get_first_valid_column_timestamp();
            },
            "Return first valid column timestamp in the scan.")
        .def("__eq__",
             [](const LidarScan& l, const LidarScan& r) { return l == r; })
        .def("__copy__", [](const LidarScan& self) { return LidarScan{self}; })
        .def("__deepcopy__",
             [](const LidarScan& self, py::dict) { return LidarScan{self}; })
        .def("__repr__",
             [](const LidarScan& self) {
                 std::stringstream ss;
                 ss << "<ouster.sdk.client._client.LidarScan @" << (void*)&self
                    << ">";
                 return ss.str();
             })
        .def("__str__", [](const LidarScan& self) { return to_string(self); });

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

    py::class_<SensorHttp>(m, "SensorHttp")
        .def("metadata", &SensorHttp::metadata,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("sensor_info", &SensorHttp::sensor_info,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("get_config_params", &SensorHttp::get_config_params,
             py::arg("active"),
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("set_config_param", &SensorHttp::set_config_param, py::arg("key"),
             py::arg("value"),
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
        // TODO: get_user_data_and_policy is hard to bind, bind later if needed
        .def("set_user_data", &SensorHttp::set_user_data, py::arg("data"),
             py::arg("keep_on_config_delete") = true,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("delete_user_data", &SensorHttp::delete_user_data,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("network", &SensorHttp::network,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("firmware_version",
             [](SensorHttp& self) { return self.firmware_version(); })
        .def("hostname", &SensorHttp::hostname)
        .def_static(
            "create",
            [](const std::string& hostname, int timeout_sec) {
                return SensorHttp::create(hostname, timeout_sec);
            },
            py::arg("hostname"),
            py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS);

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
        .def(py::init([](const sensor_info& sensor, bool use_extrinsics) {
                 auto self = new XYZLut{};
                 *self = make_xyz_lut(sensor, use_extrinsics);
                 return self;
             }),
             py::arg("info"), py::arg("use_extrinsics"))
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
            return ouster::get_field_types(info);
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
        [](sensor::UDPProfileLidar profile) {
            return ouster::get_field_types(profile);
        },
        R"(
        Extracts LidarScan fields with types for a given ``udp_profile_lidar``

        Args:
            udp_profile_lidar: lidar profile from which to get field types

        Returns:
            returns field types
            )",
        py::arg("udp_profile_lidar"));

    py::class_<ouster::ValidatorIssues>(m, "ValidatorIssues")
        .def_property_readonly(
            "critical",
            [](ouster::ValidatorIssues& self) { return self.critical; },
            "Critical validator issues.")
        .def_property_readonly(
            "warning",
            [](ouster::ValidatorIssues& self) { return self.warning; },
            "Warning validator issues.")
        .def_property_readonly(
            "information",
            [](ouster::ValidatorIssues& self) { return self.information; },
            "Information validator issues");

    py::class_<ouster::ValidatorIssues::ValidatorEntry>(m, "ValidatorEntry")
        .def("__str__", &ouster::ValidatorIssues::ValidatorEntry::to_string,
             R"(
        Get the string representation of a ValidatorEntry

        Returns:
            returns the string representation of a ValidatorEntry
        )")
        .def("__repr__", &ouster::ValidatorIssues::ValidatorEntry::to_string,
             R"(
        Get the string representation of a ValidatorEntry

        Returns:
            returns the string representation of a ValidatorEntry
        )")
        .def("get_path", &ouster::ValidatorIssues::ValidatorEntry::get_path,
             R"(
        Get the entry path to the issue.

        Returns:
            returns the entry path to the issue.
        )")
        .def("get_msg", &ouster::ValidatorIssues::ValidatorEntry::get_msg,
             R"(
        Get the message of the ValidatorEntry

        Returns:
            returns the message of the ValidatorEntry
        )");

    m.def(
        "parse_and_validate_metadata",
        [](const std::string& metadata)
            -> std::tuple<nonstd::optional<ouster::sensor::sensor_info>,
                          ouster::ValidatorIssues> {
            nonstd::optional<ouster::sensor::sensor_info> sensor_info;
            ouster::ValidatorIssues issues;

            ouster::parse_and_validate_metadata(metadata, sensor_info, issues);

            return std::make_pair(sensor_info, issues);
        },
        R"(
        Parse and validate sensor metadata

        Args:
            metadata (str): The metadata json to parse and validate.

        Returns:
            returns (ValidatorIssues, SensorInfo): The list of issues that were encountered 
                                                  and the parsed SensorInfo
        )");

    py::class_<Packet>(m, "Packet")
        .def(py::init<int>(), py::arg("size") = 65536)
        // direct access to timestamp field
        .def_readwrite("host_timestamp", &Packet::host_timestamp)
        // access via seconds (deprecated)
        .def_property(
            "capture_timestamp",
            [](Packet& self) -> nonstd::optional<double> {
                ouster::sensor::logger().warn(
                    "Packet.capture_timestamp is deprecated, use "
                    "Packet.host_timestamp instead.");
                if (self.host_timestamp) {
                    return self.host_timestamp / 1e9;
                } else {
                    return nonstd::nullopt;
                }
            },
            [](Packet& self, nonstd::optional<double> v) {
                ouster::sensor::logger().warn(
                    "Packet.capture_timestamp is deprecated, use "
                    "Packet.host_timestamp instead.");
                if (v) {
                    self.host_timestamp = static_cast<uint64_t>(*v * 1e9);
                } else {
                    self.host_timestamp = 0;
                }
            })
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "buf",
            // we have to use cpp_function here because py::keep_alive
            // does not work with pybind11 def_property methods due to a bug:
            // https://github.com/pybind/pybind11/issues/4236
            py::cpp_function(
                [](Packet& self) {
                    return py::array(py::dtype::of<uint8_t>(), self.buf.size(),
                                     self.buf.data(), py::cast(self));
                },
                py::keep_alive<0, 1>()));

    py::enum_<sensor::PacketValidationFailure>(m, "PacketValidationFailure",
                                               py::arithmetic())
        .value("NONE", sensor::PacketValidationFailure::NONE)
        .value("PACKET_SIZE", sensor::PacketValidationFailure::PACKET_SIZE)
        .value("ID", sensor::PacketValidationFailure::ID);

    py::class_<LidarPacket, Packet>(m, "LidarPacket")
        .def(py::init<int>(), py::arg("size") = 65536)
        .def("__copy__",
             [](const LidarPacket& self) { return LidarPacket(self); })
        .def("__deepcopy__", [](const LidarPacket& self,
                                py::dict) { return LidarPacket(self); })
        .def("validate", &LidarPacket::validate);

    py::class_<ImuPacket, Packet>(m, "ImuPacket")
        .def(py::init<int>(), py::arg("size") = 65536)
        .def("__copy__", [](const ImuPacket& self) { return ImuPacket(self); })
        .def("__deepcopy__",
             [](const ImuPacket& self, py::dict) { return ImuPacket(self); })
        .def("validate", &ImuPacket::validate);

    using ouster::sensor::impl::FieldInfo;
    py::class_<FieldInfo>(m, "FieldInfo")
        .def(py::init([](py::object dt, size_t offset, uint64_t mask,
                         int shift) {
            return new FieldInfo{field_type_of_dtype(py::dtype::from_args(dt)),
                                 offset, mask, shift};
        }))
        .def_property_readonly("ty_tag",
                               [](const FieldInfo& self) {
                                   return dtype_of_field_type(self.ty_tag);
                               })
        .def_readwrite("offset", &FieldInfo::offset)
        .def_readwrite("mask", &FieldInfo::mask)
        .def_readwrite("shift", &FieldInfo::shift);

    m.def("add_custom_profile", &ouster::sensor::add_custom_profile);

    m.def("in_multicast", &ouster::sensor::in_multicast);

    m.def("dewarp",
          py::overload_cast<const py::array_t<double>&,
                            const py::array_t<double>&>(&dewarp),
          R"(
	Applies a set of 4x4 pose transformations to a collection of 3D points.
	Args:
	  points: A NumPy array of shape (H, W, 3) representing the 3D points.
	  poses: A NumPy array of shape (W, 4, 4) representing the 4x4 pose

	Return:
	  A NumPy array of shape (H, W, 3) containing the dewarped 3D points
	  )",
          py::arg("points"), py::arg("poses"));

    m.def("transform",
          py::overload_cast<const py::array_t<double>&,
                            const py::array_t<double>&>(&transform),
          R"(
	Applies a single of 4x4 pose transformations to a collection of 3D points.
	Args:
	  points: A NumPy array of shape (H, W, 3), or (N, 3)
	  pose: A NumPy array of shape (4, 4) representing the 4x4 pose

	Return:
	  A NumPy array of shape (H, W, 3) or (N, 3) containing the transformed 3D points
	  after applying the corresponding 4x4 transformation matrices to the points
	  )",
          py::arg("points"), py::arg("pose"));

    m.attr("__version__") = ouster::SDK_VERSION;

    m.attr("SHORT_HTTP_REQUEST_TIMEOUT_SECONDS") =
        py::int_(SHORT_HTTP_REQUEST_TIMEOUT_SECONDS);
    m.attr("LONG_HTTP_REQUEST_TIMEOUT_SECONDS") =
        py::int_(LONG_HTTP_REQUEST_TIMEOUT_SECONDS);
}

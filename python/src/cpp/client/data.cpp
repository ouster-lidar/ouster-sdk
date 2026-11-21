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

#include <nanobind/make_iterator.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <pyerrors.h>
#include <warnings.h>

#include <memory>
#include <nonstd/optional.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include "client_common.h"
#include "common.h"
#include "eigen_dense.h"
#include "ouster/core/json_tools.h"
#include "ouster/core/lidar_config.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/metadata.h"
#include "ouster/core/open_source.h"
#include "ouster/core/profile_extension.h"
#include "ouster/core/sensor_config.h"
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"
#include "ouster/sensor/client.h"

using ouster::sdk::core::CalibrationStatus;
using ouster::sdk::core::DataFormat;
using ouster::sdk::core::LidarConfig;
using ouster::sdk::core::LidarInfo;
using ouster::sdk::core::LidarMode;
using ouster::sdk::core::ProductInfo;
using ouster::sdk::core::SensorConfig;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::TimestampMode;
using ouster::sdk::core::ValidatorIssues;
using ouster::sdk::core::Version;

namespace ouster {
namespace sdk {
namespace core {

namespace impl {

extern OUSTER_API_VAR const Table<TimestampMode, const char*, 3> TIMESTAMP_MODE_STRINGS;
extern OUSTER_API_VAR const Table<ouster::sdk::core::OperatingMode, const char*, 2>
    OPERATING_MODE_STRINGS;
extern OUSTER_API_VAR const Table<ouster::sdk::core::MultipurposeIOMode, const char*, 8>
    MULTIPURPOSE_IO_MODE_STRINGS;
extern OUSTER_API_VAR const Table<ouster::sdk::core::Polarity, const char*, 2> POLARITY_STRINGS;
extern OUSTER_API_VAR const Table<ouster::sdk::core::NMEABaudRate, const char*, 2>
    NMEA_BAUD_RATE_STRINGS;
extern OUSTER_API_VAR Table<ouster::sdk::core::UDPProfileIMU, const char*, 3>
    udp_profile_imu_strings;
extern OUSTER_API_VAR Table<ouster::sdk::core::HeaderType, const char*, 2> udp_profile_type_strings;
extern OUSTER_API_VAR Table<ouster::sdk::core::BloomReductionOptimization, const char*, 2>
    bloom_reduction_optimization_strings;
extern OUSTER_API_VAR Table<ouster::sdk::core::ShotLimitingStatus, const char*, 10>
    shot_limiting_status_strings;
extern OUSTER_API_VAR Table<ouster::sdk::core::ThermalShutdownStatus, const char*, 2>
    thermal_shutdown_status_strings;
extern OUSTER_API_VAR Table<ouster::sdk::core::FullScaleRange, const char*, 2>
    full_scale_range_strings;
extern OUSTER_API_VAR Table<ouster::sdk::core::ReturnOrder, const char*, 5> return_order_strings;

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster

void init_client_data(py::module_& module, py::module_& /*unused*/) {
    py::class_<DataFormat>(module, "DataFormat")
        .def(py::init<>(), R"(
        Data Format of a packet coming from sensor

        See sensor documentation for the meaning of each property
    )")
        .def_rw("pixels_per_column", &DataFormat::pixels_per_column, R"(

    Pixels per column in the lidar packet, synonymous with the number of beams of the sensor

    :type: int
    )")
        .def_rw("columns_per_packet", &DataFormat::columns_per_packet,
                R"(
    Columns per packet in the lidar packet, typically 16

    :type: int
    )")
        .def_rw("columns_per_frame", &DataFormat::columns_per_frame, R"(
    Columns per frame in the lidar packet, corresponding with LidarMode

    :type: int
    )")
        .def_rw("imu_measurements_per_packet", &DataFormat::imu_measurements_per_packet, R"(
    Number of IMU measurements per IMU packet

    :type: int
    )")
        .def_rw("imu_packets_per_frame", &DataFormat::imu_packets_per_frame, R"(
    Number of IMU packets per frame

    :type: int
    )")
        .def_prop_rw(
            "pixel_shift_by_row",
            [](DataFormat& self) {
                return py::ndarray<py::numpy, int, py::c_contig>(self.pixel_shift_by_row.data(),
                                                                 {self.pixel_shift_by_row.size()});
            },
            [](DataFormat& self, const std::vector<int>& data) { self.pixel_shift_by_row = data; },
            py::rv_policy::reference_internal,
            R"(
    Shift of pixels by row to create natural images

    :type: List[int]

    )")

        .def_rw("column_window", &DataFormat::column_window, R"(
    Firing window of sensor, set by config.azimuth_window

    :type: Tuple[int, int]
    )")
        .def_rw("udp_profile_lidar", &DataFormat::udp_profile_lidar, R"(
    Lidar packet profile
    )")
        .def_rw("udp_profile_imu", &DataFormat::udp_profile_imu, R"(
    IMU packet profile
    )")
        .def_rw("header_type", &DataFormat::header_type, R"(
    Lidar/IMU packet header profile
    )")
        // TODO: imu measurements per packet
        .def_rw("fps", &DataFormat::fps, R"(
    Frames per second, e.g., 10 for LidarMode 1024x10

    :type: int
    )")
        .def_rw("imu_enabled", &DataFormat::imu_enabled,
                R"(
    True if the IMU stream is enabled, otherwise False.

    :type: bool
    )")
        .def_rw("zone_monitoring_enabled", &DataFormat::zone_monitoring_enabled,
                R"(
    True if zone monitoring is enabled, otherwise False.

    :type: bool
    )")
        .def("valid_columns_per_frame", &DataFormat::valid_columns_per_frame,
             R"(
    Return the number of valid columns per complete frame of data with the column_window applied.
    )")
        .def("lidar_packets_per_frame", &DataFormat::lidar_packets_per_frame,
             R"(
    Return the number of valid packets actually sent per frame of data with the column_window applied.
    )")
        .def("__eq__", [](const DataFormat& left, const py::object& right) {
            if (!py::isinstance<DataFormat>(right)) {
                return false;
            }
            return left == py::cast<const DataFormat&>(right);
        });

    auto calibration_status =
        py::class_<CalibrationStatus>(module, "CalibrationStatus")
            .def(py::init<>(), R"(
        Sensor Calibration in a Sensor Metadata, covering reflectivity calibration and more"
    )")
            .def("__str__",
                 [](const CalibrationStatus& cal_status) { return to_string(cal_status); })
            .def("__eq__", [](const CalibrationStatus& left, const py::object& right) {
                if (!py::isinstance<CalibrationStatus>(right)) {
                    return false;
                }
                return left == py::cast<const CalibrationStatus&>(right);
            });

    def_opt(calibration_status, CalibrationStatus, "reflectivity_status", reflectivity_status,
            "Reflectivity calibration status.");

    def_opt(calibration_status, CalibrationStatus, "reflectivity_timestamp", reflectivity_timestamp,
            "Reflectivity timestamp.");

    // Product Line
    py::class_<ProductInfo>(module, "ProductInfo")
        .def_ro("full_product_info", &ProductInfo::full_product_info, R"(
    The original full product info string.

    :type: string
    )")
        .def_ro("form_factor", &ProductInfo::form_factor, R"(
    The form factor of the product.

    :type: string
    )")
        .def_ro("short_range", &ProductInfo::short_range, R"(
    If the product is a short range make.

    :type: bool
    )")
        .def_ro("rgb", &ProductInfo::rgb, R"(
        If the product is a rgb make.

        :type: bool
    )")
        .def_ro("beam_config", &ProductInfo::beam_config, R"(
    The beam configuration of the product..

    :type: string
    )")
        .def_ro("beam_count", &ProductInfo::beam_count, R"(
    Number of beams

    :type: int
    )")
        .def("__eq__",
             [](const ProductInfo& left, const py::object& right) {
                 if (!py::isinstance<ProductInfo>(right)) {
                     return false;
                 }
                 return left == py::cast<const ProductInfo&>(right);
             })
        .def("__str__", [](const ProductInfo& product_info) { return to_string(product_info); });

    // NOLINTNEXTLINE(bugprone-unused-raii)
    py::class_<SensorConfig>(module, "SensorConfig", R"(
    Base class for sensor configurations.

    Currently has no attributes. APIs that accept a configuration take this
    base type and downcast to the concrete type they require (e.g. LidarConfig).
    )");

    auto sensor_config =
        py::class_<LidarConfig, SensorConfig>(module, "LidarConfig", R"(
    Corresponds to sensor config parameters.

    Please see sensor documentation for the meaning of each property.
    )")
            .def(py::init<>(), "Construct an empty LidarConfig.")
            .def(
                "__init__",
                [](LidarConfig* self, const std::string& config_string) {
                    new (self) LidarConfig{};
                    ValidatorIssues issues;
                    if (!ouster::sdk::core::parse_and_validate_config(config_string, *self,
                                                                      issues)) {
                        self->~LidarConfig();
                        throw std::runtime_error(to_string(issues.critical));
                    }
                },
                py::arg("config_string"), R"(
    Construct a LidarConfig from a json string.
    Args:
        config_string (str): json string to parse
    )")
            .def("__str__",
                 [](const LidarConfig& sensor_config) { return to_string(sensor_config); })
            .def("__eq__",
                 [](const LidarConfig& left, const py::object& right) {
                     if (!py::isinstance<LidarConfig>(right)) {
                         return false;
                     }
                     return left == py::cast<const LidarConfig&>(right);
                 })
            .def("__copy__", [](const LidarConfig& self) { return LidarConfig{self}; })
            .def("__deepcopy__",
                 [](const LidarConfig& self, const py::dict&) { return LidarConfig{self}; });

    def_opt(sensor_config, LidarConfig, "udp_dest", udp_dest,
            "Destination to which sensor sends UDP traffic.");
    def_opt(sensor_config, LidarConfig, "udp_dest_zm", udp_dest_zm,
            "Destination to which sensor sends ZM UDP traffic.");
    def_opt(sensor_config, LidarConfig, "udp_multicast_ttl", udp_multicast_ttl,
            "Multicast TTL for IMU and lidar UDP traffic.");
    def_opt(sensor_config, LidarConfig, "udp_multicast_ttl_zm", udp_multicast_ttl_zm,
            "Multicast TTL for ZM UDP traffic.");
    def_opt(sensor_config, LidarConfig, "udp_port_lidar", udp_port_lidar,
            "Port on UDP destination to which lidar data will be sent.");
    def_opt(sensor_config, LidarConfig, "udp_port_imu", udp_port_imu,
            "Port on UDP destination to which IMU data will be sent.");
    def_opt(sensor_config, LidarConfig, "udp_port_zm", udp_port_zm,
            "Port on UDP destination to which ZM data will be sent.");
    def_opt_noconvert(sensor_config, LidarConfig, "timestamp_mode", timestamp_mode,
                      "Timestamp mode of sensor. See class TimestampMode.");
    def_opt(sensor_config, LidarConfig, "lidar_mode", lidar_mode,
            "Horizontal and Vertical Resolution rate of sensor as mode, e.g., "
            "1024x10. See class LidarMode.",
            py::rv_policy::copy);
    def_opt_noconvert(sensor_config, LidarConfig, "operating_mode", operating_mode,
                      "Operating Mode of sensor. See class OperatingMode.");
    def_opt_noconvert(sensor_config, LidarConfig, "multipurpose_io_mode", multipurpose_io_mode,
                      "Mode of MULTIPURPOSE_IO pin. See class MultipurposeIOMode.");
    def_opt(sensor_config, LidarConfig, "lidar_frame_azimuth_offset", lidar_frame_azimuth_offset,
            "Origin angle for the sensor in millidegrees.");
    def_opt(sensor_config, LidarConfig, "azimuth_window", azimuth_window,
            "Tuple representing the visible region of interest of the sensor in "
            "millidegrees, .e.g., (0, 360000) for full visibility.");
    def_opt(sensor_config, LidarConfig, "signal_multiplier", signal_multiplier,
            "Multiplier for signal strength of sensor, corresponding to maximum "
            "allowable azimuth_window. Gen 2 Only.");
    def_opt(sensor_config, LidarConfig, "sync_pulse_out_angle", sync_pulse_out_angle,
            "Polarity of SYNC_PULSE_OUT output. See sensor documentation for "
            "details.");
    def_opt(sensor_config, LidarConfig, "sync_pulse_out_pulse_width", sync_pulse_out_pulse_width,
            "SYNC_PULSE_OUT pulse width in ms. See sensor documentation for "
            "details.");
    def_opt_noconvert(sensor_config, LidarConfig, "nmea_in_polarity", nmea_in_polarity,
                      "Polarity of NMEA UART input $GPRMC messages. See sensor "
                      "documentation for details.");
    def_opt_noconvert(sensor_config, LidarConfig, "nmea_baud_rate", nmea_baud_rate,
                      "Expected baud rate sensor attempts to decode for NMEA UART input "
                      "$GPRMC messages.");
    def_opt(sensor_config, LidarConfig, "nmea_ignore_valid_char", nmea_ignore_valid_char,
            "NMEA Ignore Valid Char. True corresponds to 1 and False to 0 for "
            "property; see sensor documentation for details.");
    def_opt(sensor_config, LidarConfig, "nmea_leap_seconds", nmea_leap_seconds,
            "Integer number of leap seconds that will be added to the UDP "
            "timestamp when calculating seconds since Unix Epoch time. See "
            "sensor documentation for details.");
    def_opt_noconvert(sensor_config, LidarConfig, "sync_pulse_in_polarity", sync_pulse_in_polarity,
                      "Polarity of SYNC_PULSE_IN pin. See sensor documentation for "
                      "details.");
    def_opt_noconvert(sensor_config, LidarConfig, "sync_pulse_out_polarity",
                      sync_pulse_out_polarity,
                      "Polarity of SYNC_PULSE_OUT output. See sensor documentation for "
                      "details.");
    def_opt(sensor_config, LidarConfig, "sync_pulse_out_frequency", sync_pulse_out_frequency,
            "SYNC_PULSE_OUT rate. See sensor documentation for details.");
    def_opt(sensor_config, LidarConfig, "phase_lock_enable", phase_lock_enable,
            "Enable phase lock. See sensor documentation for more details.");
    def_opt(sensor_config, LidarConfig, "phase_lock_offset", phase_lock_offset,
            "Angle in Lidar Coordinate Frame that sensors are locked to, in "
            "millidegrees. See sensor documentation for details.");
    def_opt(sensor_config, LidarConfig, "columns_per_packet", columns_per_packet,
            "Measurement blocks per UDP packet. See sensor documentation for "
            "details.");
    def_opt(sensor_config, LidarConfig, "udp_profile_lidar", udp_profile_lidar,
            "UDP packet format for lidar data. See sensor documentation for "
            "details.");
    def_opt(sensor_config, LidarConfig, "udp_profile_imu", udp_profile_imu,
            "UDP packet format for imu data. See sensor documentation for "
            "details.");
    def_opt(sensor_config, LidarConfig, "gyro_fsr", gyro_fsr,
            "The gyro full scale measurement range to use. See sensor "
            "documentation for details.");
    def_opt(sensor_config, LidarConfig, "accel_fsr", accel_fsr,
            "The accelerometer full scale measurement range to use. See sensor "
            "documentation for details.");
    def_opt(sensor_config, LidarConfig, "return_order", return_order,
            "The priority of sensor returns to output. See sensor documentation "
            "for details.");
    def_opt(sensor_config, LidarConfig, "min_range_threshold_cm", min_range_threshold_cm,
            "The minimum detection range of the sensor in cm. See sensor "
            "documentation for details.");
    def_opt(sensor_config, LidarConfig, "imu_packets_per_frame", imu_packets_per_frame,
            "Number of IMU packets per lidar frame.");
    def_opt_noconvert(sensor_config, LidarConfig, "header_type", header_type,
                      "Type of UDP packet header to use.");
    def_opt_noconvert(sensor_config, LidarConfig, "bloom_reduction_optimization",
                      bloom_reduction_optimization,
                      "The type of bloom reduction optimization to use.");
    def_opt(sensor_config, LidarConfig, "extra_options", extra_options,
            "Extra configuration options on the sensor. Each value should be "
            "stringized json.");

    module.def("parse_and_validate_sensor_config",
               [](const std::string& metadata) -> std::tuple<LidarConfig, ValidatorIssues> {
                   LidarConfig config;
                   ValidatorIssues issues;
                   ouster::sdk::core::parse_and_validate_config(metadata, config, issues);
                   return std::make_pair(config, issues);
               });

    // Version Info
    py::class_<Version>(module, "Version")
        .def(py::init<>())
        .def(
            "__init__",
            [](Version* self, int major, int minor, int patch) {
                new (self)
                    Version(major, static_cast<uint16_t>(minor), static_cast<uint16_t>(patch));
            },
            py::arg("major"), py::arg("minor"), py::arg("patch"))
        .def("__eq__",
             [](const Version& left, const py::object& right) {
                 if (!py::isinstance<Version>(right)) {
                     return false;
                 }
                 return left == py::cast<const Version&>(right);
             })
        .def("__lt__", [](const Version& left, const Version& right) { return left < right; })
        .def("__le__", [](const Version& left, const Version& right) { return left <= right; })
        .def_rw("major", &Version::major)
        .def_rw("minor", &Version::minor)
        .def_rw("patch", &Version::patch)
        .def_rw("stage", &Version::stage)
        .def_rw("machine", &Version::machine)
        .def_rw("prerelease", &Version::prerelease)
        .def_rw("build", &Version::build)
        .def("__str__", [](const Version& version) { return version.to_string(); })
        .def_static("from_string", &ouster::sdk::core::version_from_string);

    module.attr("INVALID_VERSION") = ouster::sdk::core::INVALID_VERSION;

    module.attr("MIN_VERSION") = ouster::sdk::sensor::MIN_VERSION;

    py::class_<ValidatorIssues>(module, "ValidatorIssues")
        .def_prop_ro(
            "critical", [](ValidatorIssues& self) { return self.critical; },
            "Critical validator issues.")
        .def_prop_ro(
            "warning", [](ValidatorIssues& self) { return self.warning; },
            "Warning validator issues.")
        .def_prop_ro(
            "information", [](ValidatorIssues& self) { return self.information; },
            "Information validator issues");

    py::class_<ValidatorIssues::ValidatorEntry>(module, "ValidatorEntry")
        .def("__str__", &ValidatorIssues::ValidatorEntry::to_string,
             R"(
    Get the string representation of a ValidatorEntry

    Returns:
        returns the string representation of a ValidatorEntry
    )")
        .def("__repr__", &ValidatorIssues::ValidatorEntry::to_string,
             R"(
    Get the string representation of a ValidatorEntry

    Returns:
        returns the string representation of a ValidatorEntry
    )")
        .def("get_path", &ValidatorIssues::ValidatorEntry::get_path,
             R"(
    Get the entry path to the issue.

    Returns:
        returns the entry path to the issue.
    )")
        .def("get_msg", &ValidatorIssues::ValidatorEntry::get_msg,
             R"(
    Get the message of the ValidatorEntry

    Returns:
        returns the message of the ValidatorEntry
    )");

    module.def(
        "parse_and_validate_metadata",
        [](const std::string& metadata)
            -> std::tuple<nonstd::optional<LidarInfo>, ValidatorIssues> {
            nonstd::optional<LidarInfo> sensor_info;
            ValidatorIssues issues;

            ouster::sdk::core::parse_and_validate_metadata(metadata, sensor_info, issues);

            return std::make_pair(sensor_info, issues);
        },
        R"(
    Parse and validate sensor metadata

    Args:
        metadata (str): The metadata json to parse and validate.

    Returns:
        returns (ValidatorIssues, LidarInfo): The list of issues that were encountered
                                                and the parsed LidarInfo
    )");

    module.def(
        "populate_extrinsics",
        [](const std::string& extrinsics_file,
           std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> extrinsics,
           std::vector<std::shared_ptr<LidarInfo>>& sensor_infos) {
            // Nanobind with <nanobind/eigen/dense.h> handles the conversion
            // from list-of-numpy-arrays to std::vector<Eigen::Matrix>
            // automatically.

            ouster::sdk::core::populate_extrinsics(extrinsics_file, std::move(extrinsics),
                                                   sensor_infos);
        },
        R"(
    Populate extrinsics for a set of sensors.

    This function reads extrinsics from a file and applies them to the provided
    sensor metadata.

    Args:
        extrinsics_file (str): Path to the file containing extrinsics data.
        extrinsics (List[np.ndarray]): A list of 4x4 pose matrices.
        sensor_infos (List[LidarInfo]): A list of sensor metadata objects to update.

    Returns:
        None
    )");

    auto lidar_mode = py::class_<LidarMode>(module, "LidarMode",
                                            R"(Possible Lidar Modes of sensor.

    Determines to horizontal and vertical resolution rates of sensor. See
    sensor documentation for details.)")
                          .def(py::init<unsigned int, unsigned int>(), R"(
        Construct a mode with a given column count and fps.
    )",
                               py::arg("columns"), py::arg("fps"))
                          .def(
                              "__init__",
                              [](LidarMode* self, const std::string& mode_string) {
                                  new (self) LidarMode(mode_string);
                              },
                              py::arg("mode_string"), R"(
    Args:
        mode_string (str): mode string to parse
    )")
                          .def_ro("columns", &LidarMode::columns, "Number of columns per frame.")
                          .def_ro("fps", &LidarMode::fps, "Framerate of frames.")
                          .def_ro_static("_512x10", &LidarMode::_512x10)
                          .def_ro_static("_512x20", &LidarMode::_512x20)
                          .def_ro_static("_1024x10", &LidarMode::_1024x10)
                          .def_ro_static("_1024x20", &LidarMode::_1024x20)
                          .def_ro_static("_2048x10", &LidarMode::_2048x10)
                          .def_ro_static("_4096x5", &LidarMode::_4096x5)
                          .def("__str__", [](const LidarMode& self) { return to_string(self); })
                          .def("__eq__", [](const LidarMode& left, const py::object& right) {
                              if (!py::isinstance<LidarMode>(right)) {
                                  return false;
                              }
                              return left == py::cast<const LidarMode&>(right);
                          });

    auto timestamp_mode = py::enum_<TimestampMode>(module, "TimestampMode", R"(
    Possible Timestamp modes of sensor.See sensor documentation for details.)");
    def_enum(timestamp_mode, ouster::sdk::core::impl::TIMESTAMP_MODE_STRINGS, "TimestampMode");

    auto operating_mode = py::enum_<ouster::sdk::core::OperatingMode>(module, "OperatingMode", R"(
    Possible Operating modes of sensor.

    See sensor documentation for details.)");
    def_enum(operating_mode, ouster::sdk::core::impl::OPERATING_MODE_STRINGS, "OperatingMode");

    auto multipurpose_io_mode =
        py::enum_<ouster::sdk::core::MultipurposeIOMode>(module, "MultipurposeIOMode", R"(
    Mode of MULTIPURPOSE_IO pin.

    See sensor documentation for details.)");
    def_enum(multipurpose_io_mode, ouster::sdk::core::impl::MULTIPURPOSE_IO_MODE_STRINGS,
             "MultipurposeIOMode");

    auto polarity = py::enum_<ouster::sdk::core::Polarity>(module, "Polarity", R"(
    Pulse Polarity.

    Applicable to several Polarity settings on sensor.)");
    def_enum(polarity, ouster::sdk::core::impl::POLARITY_STRINGS, "Polarity");

    auto return_order = py::enum_<ouster::sdk::core::ReturnOrder>(module, "ReturnOrder", R"(
    Sensor return order.

    See sensor documentation for details.)");
    def_enum(return_order, ouster::sdk::core::impl::return_order_strings, "ReturnOrder");

    auto full_scale_range =
        py::enum_<ouster::sdk::core::FullScaleRange>(module, "FullScaleRange", R"(
    IMU output scale range.

    See sensor documentation for details.)");
    def_enum(full_scale_range, ouster::sdk::core::impl::full_scale_range_strings, "FullScaleRange");

    auto nmea_baud_rate = py::enum_<ouster::sdk::core::NMEABaudRate>(module, "NMEABaudRate", R"(
    Expected baud rate sensor attempts to decode for NMEA UART input $GPRMC messages.)");
    // NOTE[UN]: NMEA Baud Rates are the only exception among other enums here
    // where the prefix is embedded in the string values themselves
    // (understandably), so no need to pass a prefix to def_enum
    def_enum(nmea_baud_rate, ouster::sdk::core::impl::NMEA_BAUD_RATE_STRINGS, "NMEABaudRate");

    module.def(
        "add_custom_profile",
        [&](const std::string& name,
            const std::vector<std::pair<std::string, ouster::sdk::core::FieldDecodeInfo>>& fields,
            size_t chan_data_size) {
            return ouster::sdk::core::add_custom_profile(name, fields, chan_data_size);
        });

    auto udp_profile_imu =
        py::enum_<ouster::sdk::core::UDPProfileIMU>(module, "UDPProfileIMU", "UDP imu profile.");
    def_enum(udp_profile_imu, ouster::sdk::core::impl::udp_profile_imu_strings, "UDPProfileIMU");

    auto header_type = py::enum_<ouster::sdk::core::HeaderType>(module, "HeaderType",
                                                                "UDP header format profile.");
    def_enum(header_type, ouster::sdk::core::impl::udp_profile_type_strings, "HeaderType");

    auto bloom_reduction_optimization = py::enum_<ouster::sdk::core::BloomReductionOptimization>(
        module, "BloomReductionOptimization", "Bloom Reduction Optimization.");
    def_enum(bloom_reduction_optimization,
             ouster::sdk::core::impl::bloom_reduction_optimization_strings,
             "BloomReductionOptimization");

    auto shot_limiting_status = py::enum_<ouster::sdk::core::ShotLimitingStatus>(
        module, "ShotLimitingStatus", "Shot Limiting Status.");
    def_enum(shot_limiting_status, ouster::sdk::core::impl::shot_limiting_status_strings,
             "ShotLimitingStatus");

    auto thermal_shutdown_status = py::enum_<ouster::sdk::core::ThermalShutdownStatus>(
        module, "ThermalShutdownStatus", "Thermal Shutdown Status.");
    def_enum(thermal_shutdown_status, ouster::sdk::core::impl::thermal_shutdown_status_strings,
             "ThermalShutdownStatus");

    // Sensor Info
    auto sensor_info =
        py::class_<LidarInfo, SensorInfo>(module, "LidarInfo",
                                          R"(
    Sensor Info required to interpret UDP data streams.

    See the sensor documentation for the meaning of each property.
    )")
            // Constructed via py::new_ to match SensorInfo's factory __new__;
            // see the SensorInfo binding.
            .def(py::new_([]() { return std::make_shared<LidarInfo>(); }), R"(
        Construct an empty metadata.
    )")
            .def(py::new_([](const std::string& json_string) {
                     return std::make_shared<LidarInfo>(json_string);
                 }),
                 py::arg("json_string"), R"(
    Args:
        json_string (str): json string to parse
    )")
            .def_rw("fw_rev", &LidarInfo::fw_rev, "Sensor firmware revision.")
            .def_rw("prod_line", &LidarInfo::prod_line, "Product line, e.g., 'OS-1-128'.")
            .def_rw("format", &LidarInfo::format,
                    "Describes the structure of a lidar packet. See class "
                    "DataFormat.")
            .def_prop_ro(
                "fps", [](const LidarInfo& self) { return self.fps(); },
                "Frames per second, e.g., 10 for LidarMode 1024x10")
            .def_prop_rw(
                "beam_azimuth_angles",
                [](LidarInfo& self) {
                    return py::ndarray<py::numpy, double, py::c_contig>(
                        self.beam_azimuth_angles.data(), {self.beam_azimuth_angles.size()});
                },
                [](LidarInfo& self, const std::vector<double>& data) {
                    self.beam_azimuth_angles = data;
                },
                py::rv_policy::reference_internal,
                "Beam azimuth angles, useful for XYZ projection.")
            .def_prop_rw(
                "beam_altitude_angles",
                [](LidarInfo& self) {
                    return py::ndarray<py::numpy, double, py::c_contig>(
                        self.beam_altitude_angles.data(), {self.beam_altitude_angles.size()});
                },
                [](LidarInfo& self, const std::vector<double>& data) {
                    self.beam_altitude_angles = data;
                },
                py::rv_policy::reference_internal,
                "Beam altitude angles, useful for XYZ projection.")
            .def_rw("imu_to_sensor_transform", &LidarInfo::imu_to_sensor_transform,
                    "Homogeneous transformation matrix representing IMU "
                    "offset to Sensor Coordinate Frame.")
            .def_rw("lidar_to_sensor_transform", &LidarInfo::lidar_to_sensor_transform,
                    "Homogeneous transformation matrix from Lidar "
                    "Coordinate Frame to Sensor Coordinate Frame.")
            .def_rw("lidar_origin_to_beam_origin_mm", &LidarInfo::lidar_origin_to_beam_origin_mm,
                    "Distance between lidar origin and beam origin in millimeters.")
            .def_rw("beam_to_lidar_transform", &LidarInfo::beam_to_lidar_transform,
                    "Homogeneous transformation matrix representing Beam to "
                    "Lidar Transform")
            .def_rw("sensor_to_body", &LidarInfo::sensor_to_body,
                    "Homogeneous transformation matrix from sensor frame to "
                    "body frame. Defaults to the identity.")
            .def_rw("init_id", &LidarInfo::init_id, "Initialization id.")
            .def_rw("build_date", &LidarInfo::build_date, "Build date")
            .def_rw("image_rev", &LidarInfo::image_rev, "Image rev")
            .def_rw("prod_pn", &LidarInfo::prod_pn, "Prod pn")
            .def_rw("status", &LidarInfo::status, "sensor status")
            .def_rw("cal", &LidarInfo::cal, "sensor calibration")
            .def_rw("config", &LidarInfo::config, "sensor config",
                    py::rv_policy::reference_internal)
            .def_rw("user_data", &LidarInfo::user_data, "sensor user data")
            .def_rw("client_version", &LidarInfo::client_version,
                    "version of library that wrote metadata")
            .def_static("from_default", &ouster::sdk::core::LidarInfo::from_default,
                        R"(
    Create gen-1 OS-1-64 LidarInfo populated with design values.
    )")
            .def("has_fields_equal", &LidarInfo::has_fields_equal, R"(Compare public fields)")
            // only uncomment for debugging purposes!!
            //.def("__str__", [](const LidarInfo& i) { return to_string(i); })
            .def("__eq__",
                 [](const LidarInfo& left, const py::object& right) {
                     if (!py::isinstance<LidarInfo>(right)) {
                         return false;
                     }
                     return left == py::cast<const LidarInfo&>(right);
                 })
            .def("__repr__",
                 [](const LidarInfo& self) {
                     const auto mode = self.config.lidar_mode
                                           ? to_string(self.config.lidar_mode.value())
                                           : std::to_string(self.format.fps) + "fps";
                     return "<ouster.sdk.client.LidarInfo " + self.prod_line + " " +
                            std::to_string(self.sn) + " " + self.fw_rev + " " + mode + ">";
                 })
            .def(
                "get_version", [](const LidarInfo& self) { return self.get_version(); },
                R"(Get parsed sensor version)")
            .def(
                "get_product_info", [](const LidarInfo& self) { return self.get_product_info(); },
                R"(Get parsed product info)")
            .def_prop_ro("num_returns", &LidarInfo::num_returns)
            .def("__copy__", [](const LidarInfo& self) { return LidarInfo{self}; })
            .def("__deepcopy__", [](const LidarInfo& sensor_info, const py::dict&) {
                return LidarInfo{sensor_info};
            });
    sensor_info.def_prop_rw(
        "zone_set",
        [](const LidarInfo* obj) {
            if (obj->zone_set.has_value()) {
                return py::cast(obj->zone_set.value(), py::rv_policy::reference_internal,
                                py::cast(obj));
            }
            return py::none();
        },
        [](LidarInfo& obj, const decltype(LidarInfo::zone_set)& val) { obj.zone_set = val; },
        py::arg().none(), py::for_getter(py::sig("def zone_set(self, /) -> Optional[ZoneSet]")),
        "zone monitor configuration", py::rv_policy::reference_internal);
}

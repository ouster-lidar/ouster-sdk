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

#include <iostream>

#include "client_common.h"
#include "ouster/client.h"
#include "ouster/impl/profile_extension.h"
#include "ouster/lidar_scan.h"
#include "ouster/metadata.h"
#include "ouster/open_source.h"
#include "ouster/types.h"

namespace py = pybind11;
using ouster::sdk::core::CalibrationStatus;
using ouster::sdk::core::DataFormat;
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

extern const Table<LidarMode, const char*, 7> LIDAR_MODE_STRINGS;
extern const Table<TimestampMode, const char*, 4> TIMESTAMP_MODE_STRINGS;
extern const Table<ouster::sdk::core::OperatingMode, const char*, 2>
    OPERATING_MODE_STRINGS;
extern const Table<ouster::sdk::core::MultipurposeIOMode, const char*, 6>
    MULTIPURPOSE_IO_MODE_STRINGS;
extern const Table<ouster::sdk::core::Polarity, const char*, 2>
    POLARITY_STRINGS;
extern const Table<ouster::sdk::core::NMEABaudRate, const char*, 2>
    NMEA_BAUD_RATE_STRINGS;
extern Table<ouster::sdk::core::UDPProfileLidar, const char*,
             ouster::sdk::core::impl::MAX_NUM_PROFILES>
    udp_profile_lidar_strings;
extern Table<ouster::sdk::core::UDPProfileIMU, const char*, 3>
    udp_profile_imu_strings;
extern Table<ouster::sdk::core::HeaderType, const char*, 2>
    udp_profile_type_strings;
extern Table<ouster::sdk::core::BloomReductionOptimization, const char*, 2>
    bloom_reduction_optimization_strings;
extern Table<ouster::sdk::core::ShotLimitingStatus, const char*, 10>
    shot_limiting_status_strings;
extern Table<ouster::sdk::core::ThermalShutdownStatus, const char*, 2>
    thermal_shutdown_status_strings;
extern Table<ouster::sdk::core::FullScaleRange, const char*, 2>
    full_scale_range_strings;
extern Table<ouster::sdk::core::ReturnOrder, const char*, 5>
    return_order_strings;

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster

void init_client_data(py::module& module, py::module& /*unused*/) {
    py::class_<DataFormat>(module, "DataFormat")
        .def(py::init<>(), R"(
        Data Format of a packet coming from sensor

        See sensor documentation for the meaning of each property
    )")
        .def_readwrite("pixels_per_column", &DataFormat::pixels_per_column, R"(

    Pixels per column in the lidar packet, synonymous with the number of beams of the sensor

    :type: int
    )")
        .def_readwrite("columns_per_packet", &DataFormat::columns_per_packet,
                       R"(
    Columns per packet in the lidar packet, typically 16

    :type: int
    )")
        .def_readwrite("columns_per_frame", &DataFormat::columns_per_frame, R"(
    Columns per frame in the lidar packet, corresponding with LidarMode

    :type: int
    )")
        .def_readwrite("imu_measurements_per_packet",
                       &DataFormat::imu_measurements_per_packet, R"(
    Number of IMU measurements per IMU packet

    :type: int
    )")
        .def_readwrite("imu_packets_per_frame",
                       &DataFormat::imu_packets_per_frame, R"(
    Number of IMU packets per frame

    :type: int
    )")
        .def_readwrite("pixel_shift_by_row", &DataFormat::pixel_shift_by_row,
                       R"(
    Shift of pixels by row to create natural images

    :type: List[int]

    )")
        .def_readwrite("column_window", &DataFormat::column_window, R"(
    Firing window of sensor, set by config.azimuth_window

    :type: Tuple[int, int]
    )")
        .def_readwrite("udp_profile_lidar", &DataFormat::udp_profile_lidar, R"(
    Lidar packet profile
    )")
        .def_readwrite("udp_profile_imu", &DataFormat::udp_profile_imu, R"(
    IMU packet profile
    )")
        .def_readwrite("header_type", &DataFormat::header_type, R"(
    Lidar/IMU packet header profile
    )")
        // TODO: imu measurements per packet
        .def_readwrite("fps", &DataFormat::fps, R"(
    Frames per second, e.g., 10 for LidarMode 1024x10

    :type: int
    )")
        .def_readwrite("zone_monitoring_enabled",
                       &DataFormat::zone_monitoring_enabled, R"(
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
        .def("__eq__", [](const DataFormat& left, const DataFormat& right) {
            return left == right;
        });

    py::class_<CalibrationStatus>(module, "CalibrationStatus")
        .def(py::init<>(), R"(
        Sensor Calibration in a Sensor Metadata, covering reflectivity calibration and more"
    )")
        .def_readwrite("reflectivity_status",
                       &CalibrationStatus::reflectivity_status,
                       "Reflectivity calibration status")
        .def_readwrite("reflectivity_timestamp",
                       &CalibrationStatus::reflectivity_timestamp,
                       "Reflectivity timestamp")
        .def("__str__",
             [](const CalibrationStatus& cal_status) {
                 return to_string(cal_status);
             })
        .def("__eq__",
             [](const CalibrationStatus& left, const CalibrationStatus& right) {
                 return left == right;
             });

    // Product Line
    py::class_<ProductInfo>(module, "ProductInfo")
        .def_readonly("full_product_info", &ProductInfo::full_product_info, R"(
    The original full product info string.

    :type: string
    )")
        .def_readonly("form_factor", &ProductInfo::form_factor, R"(
    The form factor of the product.

    :type: string
    )")
        .def_readonly("short_range", &ProductInfo::short_range, R"(
    If the product is a short range make.

    :type: bool
    )")
        .def_readonly("beam_config", &ProductInfo::beam_config, R"(
    The beam configuration of the product..

    :type: string
    )")
        .def_readonly("beam_count", &ProductInfo::beam_count, R"(
    Number of beams

    :type: int
    )")
        .def("__eq__", [](const ProductInfo& left,
                          const ProductInfo& right) { return left == right; })
        .def("__str__", [](const ProductInfo& product_info) {
            return to_string(product_info);
        });

    py::class_<SensorConfig>(module, "SensorConfig", R"(
    Corresponds to sensor config parameters.

    Please see sensor documentation for the meaning of each property.
    )")
        .def(py::init<>(), "Construct an empty SensorConfig.")
        .def(py::init([](const std::string& config_string) {
                 auto config = new SensorConfig{};
                 ValidatorIssues issues;
                 if (!ouster::sdk::core::parse_and_validate_config(
                         config_string, *config, issues)) {
                     throw std::runtime_error(to_string(issues.critical));
                 }
                 return config;
             }),
             py::arg("config_string"), R"(
    Construct a SensorConfig from a json string.
    Args:
        config_string (str): json string to parse
    )")
        .def_readwrite(
            "udp_dest", &SensorConfig::udp_dest,
            "Destination to which sensor sends lidar and IMU UDP traffic.")
        .def_readwrite("udp_dest_zm", &SensorConfig::udp_dest_zm,
                       "Destination to which sensor sends ZM UDP traffic.")
        .def_readwrite("udp_multicast_ttl", &SensorConfig::udp_multicast_ttl,
                       "Multicast TTL for IMU and lidar UDP traffic.")
        .def_readwrite("udp_multicast_ttl_zm",
                       &SensorConfig::udp_multicast_ttl_zm,
                       "Multicast TTL for ZM UDP traffic.")
        .def_readwrite(
            "udp_port_lidar", &SensorConfig::udp_port_lidar,
            "Port on UDP destination to which lidar data will be sent.")
        .def_readwrite(
            "udp_port_imu", &SensorConfig::udp_port_imu,
            "Port on UDP destination to which IMU data will be sent.")
        .def_readwrite("udp_port_zm", &SensorConfig::udp_port_zm,
                       "Port on UDP destination to which ZM data will be sent.")
        .def_readwrite("timestamp_mode", &SensorConfig::timestamp_mode,
                       "Timestamp mode of sensor. See class TimestampMode.")
        .def_readwrite("lidar_mode", &SensorConfig::lidar_mode,
                       "Horizontal and Vertical Resolution rate of sensor as "
                       "mode, e.g., 1024x10. See class LidarMode.")
        .def_readwrite("operating_mode", &SensorConfig::operating_mode,
                       "Operating Mode of sensor. See class OperatingMode.")
        .def_readwrite(
            "multipurpose_io_mode", &SensorConfig::multipurpose_io_mode,
            "Mode of MULTIPURPOSE_IO pin. See class MultipurposeIOMode.")
        .def_readwrite("lidar_frame_azimuth_offset",
                       &SensorConfig::lidar_frame_azimuth_offset,
                       "Origin angle for the sensor in millidegrees.")
        .def_readwrite(
            "azimuth_window", &SensorConfig::azimuth_window,
            "Tuple representing the visible region of interest of the sensor "
            "in millidegrees, .e.g., (0, 360000) for full visibility.")
        .def_readwrite(
            "signal_multiplier", &SensorConfig::signal_multiplier,
            "Multiplier for signal strength of sensor, corresponding to "
            "maximum allowable azimuth_window. Gen 2 Only.")
        .def_readwrite("sync_pulse_out_angle",
                       &SensorConfig::sync_pulse_out_angle,
                       "Polarity of SYNC_PULSE_OUT output. See sensor "
                       "documentation for details.")
        .def_readwrite("sync_pulse_out_pulse_width",
                       &SensorConfig::sync_pulse_out_pulse_width,
                       "SYNC_PULSE_OUT pulse width in ms. See sensor "
                       "documentation for details.")
        .def_readwrite("nmea_in_polarity", &SensorConfig::nmea_in_polarity,
                       "Polarity of NMEA UART input $GPRMC messages. See "
                       "sensor documentation for details.")
        .def_readwrite("nmea_baud_rate", &SensorConfig::nmea_baud_rate,
                       "Expected baud rate sensor attempts to decode for NMEA "
                       "UART input $GPRMC messages.")
        .def_readwrite(
            "nmea_ignore_valid_char", &SensorConfig::nmea_ignore_valid_char,
            "NMEA Ignore Valid Char. True corresponds to 1 and False to 0 for "
            "property; see sensor documentation for details.")
        .def_readwrite("nmea_leap_seconds", &SensorConfig::nmea_leap_seconds,
                       "Integer number of leap seconds that will be added to "
                       "the UDP timetsamp when calculating seconds since Unix "
                       "Epoch time. See sensor documentation for details.")
        .def_readwrite("sync_pulse_in_polarity",
                       &SensorConfig::sync_pulse_in_polarity,
                       "Polarity of SYNC_PULSE_IN pin. See sensor "
                       "documentation for details.")
        .def_readwrite("sync_pulse_out_polarity",
                       &SensorConfig::sync_pulse_out_polarity,
                       "Polarity of SYNC_PULSE_OUT output. See sensor "
                       "documentation for details.")
        .def_readwrite(
            "sync_pulse_out_frequency", &SensorConfig::sync_pulse_out_frequency,
            "SYNC_PULSE_OUT rate. See sensor documentation for details.")
        .def_readwrite(
            "phase_lock_enable", &SensorConfig::phase_lock_enable,
            "Enable phase lock. See sensor documentation for more details.")
        .def_readwrite(
            "phase_lock_offset", &SensorConfig::phase_lock_offset,
            "Angle in Lidar Coordinate Frame that sensors are locked to, in "
            "millidegrees. See sensor documentation for details.")
        .def_readwrite("columns_per_packet", &SensorConfig::columns_per_packet,
                       "Measurement blocks per UDP packet. See sensor "
                       "documentation for details.")
        .def_readwrite("udp_profile_lidar", &SensorConfig::udp_profile_lidar,
                       "UDP packet format for lidar data. See sensor "
                       "documentation for details.")
        .def_readwrite("udp_profile_imu", &SensorConfig::udp_profile_imu,
                       "UDP packet format for imu data. See sensor "
                       "documentation for details.")
        .def_readwrite("gyro_fsr", &SensorConfig::gyro_fsr,
                       "The gyro full scale measurement range to use. See "
                       "sensor documentation for details.")
        .def_readwrite("accel_fsr", &SensorConfig::accel_fsr,
                       "The accelerometer full scale measurement range to use. "
                       "See sensor documentation for details.")
        .def_readwrite("return_order", &SensorConfig::return_order,
                       "The priority of sensor returns to output. See sensor "
                       "documentation for details.")
        .def_readwrite("min_range_threshold_cm",
                       &SensorConfig::min_range_threshold_cm,
                       "The minimum detection range of the sensor in cm. See "
                       "sensor documentation for details.")
        .def_readwrite("imu_packets_per_frame",
                       &SensorConfig::imu_packets_per_frame,
                       "Number of IMU packets per lidar frame.")
        .def_readwrite("header_type", &SensorConfig::header_type,
                       "Type of UDP packet header to use.")
        .def_readwrite("bloom_reduction_optimization",
                       &SensorConfig::bloom_reduction_optimization,
                       "The type of bloom reduction optimization to use.")
        .def_readwrite("extra_options", &SensorConfig::extra_options,
                       "Extra configuration options on the sensor. Each value "
                       "should be stringized json.")
        .def("__str__",
             [](const SensorConfig& sensor_config) {
                 return to_string(sensor_config);
             })
        .def("__eq__", [](const SensorConfig& left,
                          const SensorConfig& right) { return left == right; })
        .def("__copy__",
             [](const SensorConfig& self) { return SensorConfig{self}; })
        .def("__deepcopy__", [](const SensorConfig& self, py::dict) {
            return SensorConfig{self};
        });

    module.def("parse_and_validate_sensor_config",
               [](const std::string& metadata)
                   -> std::tuple<SensorConfig, ValidatorIssues> {
                   SensorConfig config;
                   ValidatorIssues issues;
                   ouster::sdk::core::parse_and_validate_config(metadata,
                                                                config, issues);
                   return std::make_pair(config, issues);
               });

    // Version Info
    py::class_<Version>(module, "Version")
        .def(py::init<>())
        .def(py::init([](int major, uint16_t minor, uint16_t patch) {
                 return Version(major, minor, patch);
             }),
             py::arg("major"), py::arg("minor"), py::arg("patch"))
        .def("__eq__", [](const Version& left,
                          const Version& right) { return left == right; })
        .def("__lt__", [](const Version& left,
                          const Version& right) { return left < right; })
        .def("__le__", [](const Version& left,
                          const Version& right) { return left <= right; })
        .def_readwrite("major", &Version::major)
        .def_readwrite("minor", &Version::minor)
        .def_readwrite("patch", &Version::patch)
        .def_readwrite("stage", &Version::stage)
        .def_readwrite("machine", &Version::machine)
        .def_readwrite("prerelease", &Version::prerelease)
        .def_readwrite("build", &Version::build)
        .def_static("from_string", &ouster::sdk::core::version_from_string);

    module.attr("INVALID_VERSION") = ouster::sdk::core::INVALID_VERSION;

    module.attr("MIN_VERSION") = ouster::sdk::sensor::MIN_VERSION;

    py::class_<ValidatorIssues>(module, "ValidatorIssues")
        .def_property_readonly(
            "critical", [](ValidatorIssues& self) { return self.critical; },
            "Critical validator issues.")
        .def_property_readonly(
            "warning", [](ValidatorIssues& self) { return self.warning; },
            "Warning validator issues.")
        .def_property_readonly(
            "information",
            [](ValidatorIssues& self) { return self.information; },
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
            -> std::tuple<nonstd::optional<SensorInfo>, ValidatorIssues> {
            nonstd::optional<SensorInfo> sensor_info;
            ValidatorIssues issues;

            ouster::sdk::core::parse_and_validate_metadata(metadata,
                                                           sensor_info, issues);

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

    module.def(
        "populate_extrinsics",
        [](const std::string& extrinsics_file,
           std::vector<py::array_t<double>> extrinsics,
           std::vector<std::shared_ptr<SensorInfo>>& sensor_infos) {
            std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> exts;
            for (auto& pose : extrinsics) {
                // Ensure the pose is a 4x4 matrix
                if (pose.ndim() != 2 || pose.shape(0) != 4 ||
                    pose.shape(1) != 4) {
                    throw std::runtime_error(
                        "pose array must have shape (4, 4)");
                }

                py::array_t<double> c_style_pose;

                // Create a C-style copy of pose if it's neither C-style nor
                // F-style const
                const py::array_t<double>* pose_ptr = &pose;
                if (!(pose.flags() & py::array::c_style)) {
                    c_style_pose =
                        py::array_t<double, py::array::c_style>(pose);
                    pose_ptr = &c_style_pose;  // Use the C-style array for
                                               // processing
                }

                // Convert pose to Eigen format
                Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
                    pose_eigen(pose_ptr->data());
                exts.emplace_back(pose_eigen);
            }
            ouster::sdk::core::populate_extrinsics(extrinsics_file, exts,
                                                   sensor_infos);
        },
        R"(
    Populate extrinsics for a set of sensors.

    This function reads extrinsics from a file and applies them to the provided
    sensor metadata.

    Args:
        extrinsics_file (str): Path to the file containing extrinsics data.
        extrinsics (List[np.ndarray]): A list of 4x4 pose matrices.
        sensor_infos (List[SensorInfo]): A list of sensor metadata objects to update.

    Returns:
        None
    )");

    auto lidar_mode = py::enum_<LidarMode>(module, "LidarMode", R"(
    Possible Lidar Modes of sensor.

    Determines to horizontal and vertical resolution rates of sensor. See
    sensor documentation for details.)");
    def_enum(lidar_mode, ouster::sdk::core::impl::LIDAR_MODE_STRINGS, "MODE_");
    // Part of DEPRECATION: retainining these for backward compatibility
    lidar_mode.value("MODE_UNSPEC", LidarMode::UNSPECIFIED);
    lidar_mode.value("UNSPECIFIED", LidarMode::UNSPECIFIED);
    lidar_mode.attr("from_string") = py::cpp_function(
        [](const std::string& s) {
            return ouster::sdk::core::lidar_mode_of_string(s);
        },
        py::name("from_string"), "Create LidarMode from string.");

    auto timestamp_mode = py::enum_<TimestampMode>(module, "TimestampMode", R"(
    Possible Timestamp modes of sensor.See sensor documentation for details.)");
    def_enum(timestamp_mode, ouster::sdk::core::impl::TIMESTAMP_MODE_STRINGS);
    // Part of DEPRECATION: retainining these for backward compatibility
    timestamp_mode.value("UNSPECIFIED",
                         ouster::sdk::core::TimestampMode::UNSPECIFIED);
    timestamp_mode.value("TIME_FROM_UNSPEC",
                         ouster::sdk::core::TimestampMode::UNSPECIFIED);
    timestamp_mode.attr("from_string") = py::cpp_function(
        [](const std::string& s) {
            return ouster::sdk::core::timestamp_mode_of_string(s);
        },
        py::name("from_string"), "Create TimestampMode from string.");

    auto operating_mode =
        py::enum_<ouster::sdk::core::OperatingMode>(module, "OperatingMode", R"(
    Possible Operating modes of sensor.

    See sensor documentation for details.)");
    def_enum(operating_mode, ouster::sdk::core::impl::OPERATING_MODE_STRINGS,
             "OPERATING_");
    // Part of DEPRECATION: retainining these for backward compatibility
    operating_mode.value("OPERATING_UNSPEC",
                         ouster::sdk::core::OperatingMode::UNSPECIFIED);

    auto multipurpose_io_mode =
        py::enum_<ouster::sdk::core::MultipurposeIOMode>(
            module, "MultipurposeIOMode", R"(
    Mode of MULTIPURPOSE_IO pin.

    See sensor documentation for details.)");
    def_enum(multipurpose_io_mode,
             ouster::sdk::core::impl::MULTIPURPOSE_IO_MODE_STRINGS,
             "MULTIPURPOSE_");

    auto polarity =
        py::enum_<ouster::sdk::core::Polarity>(module, "Polarity", R"(
    Pulse Polarity.

    Applicable to several Polarity settings on sensor.)");
    def_enum(polarity, ouster::sdk::core::impl::POLARITY_STRINGS, "POLARITY_");

    auto return_order =
        py::enum_<ouster::sdk::core::ReturnOrder>(module, "ReturnOrder", R"(
    Sensor return order.

    See sensor documentation for details.)");
    def_enum(return_order, ouster::sdk::core::impl::return_order_strings,
             "ORDER_");

    auto full_scale_range = py::enum_<ouster::sdk::core::FullScaleRange>(
        module, "FullScaleRange", R"(
    IMU output scale range.

    See sensor documentation for details.)");
    def_enum(full_scale_range,
             ouster::sdk::core::impl::full_scale_range_strings, "FSR_");

    auto nmea_baud_rate =
        py::enum_<ouster::sdk::core::NMEABaudRate>(module, "NMEABaudRate", R"(
    Expected baud rate sensor attempts to decode for NMEA UART input $GPRMC messages.)");
    // NOTE[UN]: NMEA Baud Rates are the only exception among other enums here
    // where the prefix is embedded in the string values themselves
    // (understandably), so no need to pass a prefix to def_enum
    def_enum(nmea_baud_rate, ouster::sdk::core::impl::NMEA_BAUD_RATE_STRINGS);

    auto udp_profile_lidar = py::enum_<ouster::sdk::core::UDPProfileLidar>(
        module, "UDPProfileLidar", "UDP lidar profile.");
    def_enum(udp_profile_lidar,
             ouster::sdk::core::impl::udp_profile_lidar_strings,
             "PROFILE_LIDAR_");
    udp_profile_lidar.attr("from_string") = py::cpp_function(
        [](const std::string& profile_string) {
            return ouster::sdk::core::udp_profile_lidar_of_string(
                profile_string);
        },
        py::name("from_string"), "Create UDPProfileLidar from string.");
    udp_profile_lidar.def_property_readonly_static(
        "values",
        [](py::object) {
            return py::make_key_iterator(
                ouster::sdk::core::impl::udp_profile_lidar_strings.begin(),
                std::find_if(
                    ouster::sdk::core::impl::udp_profile_lidar_strings.begin(),
                    ouster::sdk::core::impl::udp_profile_lidar_strings.end(),
                    [](const auto& pair) { return pair.second == nullptr; }));
        },
        "Returns an iterator of all UDPProfileLidar enum members.");

    auto udp_profile_imu = py::enum_<ouster::sdk::core::UDPProfileIMU>(
        module, "UDPProfileIMU", "UDP imu profile.");
    def_enum(udp_profile_imu, ouster::sdk::core::impl::udp_profile_imu_strings,
             "PROFILE_IMU_");

    auto header_type = py::enum_<ouster::sdk::core::HeaderType>(
        module, "HeaderType", "UDP header format profile.");
    def_enum(header_type, ouster::sdk::core::impl::udp_profile_type_strings,
             "HEADER_TYPE_");

    auto bloom_reduction_optimization =
        py::enum_<ouster::sdk::core::BloomReductionOptimization>(
            module, "BloomReductionOptimization",
            "Bloom Reduction Optimization.");
    def_enum(bloom_reduction_optimization,
             ouster::sdk::core::impl::bloom_reduction_optimization_strings);

    auto shot_limiting_status =
        py::enum_<ouster::sdk::core::ShotLimitingStatus>(
            module, "ShotLimitingStatus", "Shot Limiting Status.");
    def_enum(shot_limiting_status,
             ouster::sdk::core::impl::shot_limiting_status_strings,
             "SHOT_LIMITING_");

    auto thermal_shutdown_status =
        py::enum_<ouster::sdk::core::ThermalShutdownStatus>(
            module, "ThermalShutdownStatus", "Thermal Shutdown Status.");
    def_enum(thermal_shutdown_status,
             ouster::sdk::core::impl::thermal_shutdown_status_strings,
             "THERMAL_SHUTDOWN_");

    // Sensor Info
    py::class_<SensorInfo, std::shared_ptr<SensorInfo>>(module, "SensorInfo",
                                                        R"(
    Sensor Info required to interpret UDP data streams.

    See the sensor documentation for the meaning of each property.
    )")
        .def(py::init<>(), R"(
        Construct an empty metadata.
    )")
        .def(py::init([](const std::string& json_string) {
                 return new SensorInfo(json_string);
             }),
             py::arg("json_string"), R"(
    Args:
        json_string (str): json string to parse
    )")
        .def_readwrite("sn", &SensorInfo::sn, "Sensor serial number.")
        .def_readwrite("fw_rev", &SensorInfo::fw_rev,
                       "Sensor firmware revision.")
        .def_readwrite("prod_line", &SensorInfo::prod_line,
                       "Product line, e.g., 'OS-1-128'.")
        .def_readwrite(
            "format", &SensorInfo::format,
            "Describes the structure of a lidar packet. See class DataFormat.")
        .def_readwrite("beam_azimuth_angles", &SensorInfo::beam_azimuth_angles,
                       "Beam azimuth angles, useful for XYZ projection.")
        .def_readwrite("beam_altitude_angles",
                       &SensorInfo::beam_altitude_angles,
                       "Beam altitude angles, useful for XYZ projection.")
        .def_readwrite("imu_to_sensor_transform",
                       &SensorInfo::imu_to_sensor_transform,
                       "Homogenous transformation matrix representing IMU "
                       "offset to Sensor Coordinate Frame.")
        .def_readwrite("lidar_to_sensor_transform",
                       &SensorInfo::lidar_to_sensor_transform,
                       "Homogeneous transformation matrix from Lidar "
                       "Coordinate Frame to Sensor Coordinate Frame.")
        .def_readwrite(
            "lidar_origin_to_beam_origin_mm",
            &SensorInfo::lidar_origin_to_beam_origin_mm,
            "Distance between lidar origin and beam origin in millimeters.")
        .def_readwrite("beam_to_lidar_transform",
                       &SensorInfo::beam_to_lidar_transform,
                       "Homogenous transformation matrix reprsenting Beam to "
                       "Lidar Transform")
        .def_property(
            "extrinsic",
            [](SensorInfo& self)
                -> Eigen::Ref<decltype(SensorInfo::extrinsic)> {
                return self.extrinsic;
            },
            [](SensorInfo& self,
               const decltype(SensorInfo::extrinsic)& extrinsic) {
                self.extrinsic = extrinsic;
            },
            "Extrinsic Matrix.")
        .def_readwrite("init_id", &SensorInfo::init_id, "Initialization id.")
        .def_readwrite("build_date", &SensorInfo::build_date, "Build date")
        .def_readwrite("image_rev", &SensorInfo::image_rev, "Image rev")
        .def_readwrite("prod_pn", &SensorInfo::prod_pn, "Prod pn")
        .def_readwrite("status", &SensorInfo::status, "sensor status")
        .def_readwrite("cal", &SensorInfo::cal, "sensor calibration")
        .def_readwrite("config", &SensorInfo::config, "sensor config")
        .def_readwrite("user_data", &SensorInfo::user_data, "sensor user data")
        .def_readwrite("zone_set", &SensorInfo::zone_set,
                       "zone monitor configuration")
        .def_static("from_default", &ouster::sdk::core::default_sensor_info, R"(
    Create gen-1 OS-1-64 SensorInfo populated with design values.
    )")
        .def("to_json_string", &SensorInfo::to_json_string,
             R"( Return metadata string made from current entries"
    )")
        .def("has_fields_equal", &SensorInfo::has_fields_equal,
             R"(Compare public fields")")
        // only uncomment for debugging purposes!! story for general use and
        // output is not filled
        //.def("__str__", [](const SensorInfo& i) { return to_string(i); })
        .def_property_readonly(
            "w", [](const SensorInfo& sensor_info) { return sensor_info.w(); },
            R"(returns the width of a frame (equivalent to format.columns_per_frame))")
        .def_property_readonly(
            "h", [](const SensorInfo& self) { return self.h(); },
            R"(returns the height of a frame (equivalent to format.pixels_per_column))")
        .def("__eq__", [](const SensorInfo& left,
                          const SensorInfo& right) { return left == right; })
        .def("__repr__",
             [](const SensorInfo& self) {
                 const auto mode =
                     self.config.lidar_mode
                         ? to_string(self.config.lidar_mode.value())
                         : std::to_string(self.format.fps) + "fps";
                 return "<ouster.sdk.client.SensorInfo " + self.prod_line +
                        " " + std::to_string(self.sn) + " " + self.fw_rev +
                        " " + mode + ">";
             })
        .def(
            "get_version",
            [](const SensorInfo& self) { return self.get_version(); },
            R"(Get parsed sensor version)")
        .def(
            "get_product_info",
            [](const SensorInfo& self) { return self.get_product_info(); },
            R"(Get parsed product info)")
        .def("__copy__",
             [](const SensorInfo& self) { return SensorInfo{self}; })
        .def("__deepcopy__", [](const SensorInfo& sensor_info, py::dict) {
            return SensorInfo{sensor_info};
        });
}

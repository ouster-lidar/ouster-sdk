/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/sensor_config.h"

#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ouster/core/impl/table.h"
#include "ouster/core/json_tools.h"
#include "ouster/core/metadata.h"
#include "ouster/core/visibility.h"

using nonstd::optional;
using ouster::impl::lookup;
using ouster::impl::rlookup;
using ouster::impl::Table;

namespace ouster {
namespace sdk {
namespace core {

namespace impl {

extern OUSTER_API_VAR const Table<TimestampMode, const char*, 3> TIMESTAMP_MODE_STRINGS{
    {{TimestampMode::TIME_FROM_INTERNAL_OSC, "TIME_FROM_INTERNAL_OSC"},
     {TimestampMode::TIME_FROM_SYNC_PULSE_IN, "TIME_FROM_SYNC_PULSE_IN"},
     {TimestampMode::TIME_FROM_PTP_1588, "TIME_FROM_PTP_1588"}}};

extern OUSTER_API_VAR const Table<OperatingMode, const char*, 2> OPERATING_MODE_STRINGS{
    {{OperatingMode::NORMAL, "NORMAL"}, {OperatingMode::STANDBY, "STANDBY"}}};

extern OUSTER_API_VAR const Table<MultipurposeIOMode, const char*, 6> MULTIPURPOSE_IO_MODE_STRINGS{
    {{MultipurposeIOMode::OFF, "OFF"},
     {MultipurposeIOMode::INPUT_NMEA_UART, "INPUT_NMEA_UART"},
     {MultipurposeIOMode::OUTPUT_FROM_INTERNAL_OSC, "OUTPUT_FROM_INTERNAL_OSC"},
     {MultipurposeIOMode::OUTPUT_FROM_SYNC_PULSE_IN, "OUTPUT_FROM_SYNC_PULSE_IN"},
     {MultipurposeIOMode::OUTPUT_FROM_PTP_1588, "OUTPUT_FROM_PTP_1588"},
     {MultipurposeIOMode::OUTPUT_FROM_ENCODER_ANGLE, "OUTPUT_FROM_ENCODER_ANGLE"}}};

extern OUSTER_API_VAR const Table<Polarity, const char*, 2> POLARITY_STRINGS{
    {{Polarity::ACTIVE_LOW, "ACTIVE_LOW"}, {Polarity::ACTIVE_HIGH, "ACTIVE_HIGH"}}};

#if defined(BAUD_9600)
#undef BAUD_9600
#endif
#if defined(BAUD_115200)
#undef BAUD_115200
#endif

extern OUSTER_API_VAR const Table<NMEABaudRate, const char*, 2> NMEA_BAUD_RATE_STRINGS{
    {{NMEABaudRate::BAUD_9600, "BAUD_9600"}, {NMEABaudRate::BAUD_115200, "BAUD_115200"}}};

OUSTER_API_VAR Table<FullScaleRange, const char*, 2> full_scale_range_strings{{
    {FullScaleRange::NORMAL, "NORMAL"},
    {FullScaleRange::EXTENDED, "EXTENDED"},
}};

OUSTER_API_VAR Table<ReturnOrder, const char*, 5> return_order_strings{{
    {ReturnOrder::STRONGEST_TO_WEAKEST, "STRONGEST_TO_WEAKEST"},
    {ReturnOrder::FARTHEST_TO_NEAREST, "FARTHEST_TO_NEAREST"},
    {ReturnOrder::NEAREST_TO_FARTHEST, "NEAREST_TO_FARTHEST"},
    {ReturnOrder::DEPRECATED_STRONGEST_RETURN_FIRST, "STRONGEST_RETURN_FIRST"},
    {ReturnOrder::DEPRECATED_LAST_RETURN_FIRST, "LAST_RETURN_FIRST"},
}};

OUSTER_API_VAR Table<BloomReductionOptimization, const char*, 2>
    bloom_reduction_optimization_strings{{
        {BloomReductionOptimization::BALANCED, "BALANCED"},
        {BloomReductionOptimization::MINIMIZE_FALSE_POSITIVES, "MINIMIZE_FALSE_POSITIVES"},
    }};

}  // namespace impl

bool operator==(const LidarMode& lhs, const LidarMode& rhs) {
    return (lhs.fps == rhs.fps && lhs.columns == rhs.columns);
}

bool operator!=(const LidarMode& lhs, const LidarMode& rhs) {
    return !(lhs == rhs);
}

LidarMode::LidarMode(const std::string& mode) {
    auto split = mode.find('x');
    try {
        if (split == std::string::npos) {
            throw std::invalid_argument("");
        }
        auto str_cols = mode.substr(0, split);
        auto str_fps = mode.substr(split + 1);
        int cols_int = std::stoi(str_cols);
        int fps_int = std::stoi(str_fps);
        if (cols_int < 0 || fps_int < 0) {
            throw std::invalid_argument("");
        }
        columns = cols_int;
        fps = fps_int;
    } catch (std::invalid_argument&) {
        throw std::invalid_argument("Invalid lidar mode string \"" + mode + "\".");
    }
}

LidarMode::LidarMode(unsigned int cols, unsigned int framerate) : columns(cols), fps(framerate) {}

OUSTER_API_VAR const LidarMode LidarMode::_512x10 = {512, 10};
OUSTER_API_VAR const LidarMode LidarMode::_512x20 = {512, 20};
OUSTER_API_VAR const LidarMode LidarMode::_1024x10 = {1024, 10};
OUSTER_API_VAR const LidarMode LidarMode::_1024x20 = {1024, 20};
OUSTER_API_VAR const LidarMode LidarMode::_2048x10 = {2048, 10};
OUSTER_API_VAR const LidarMode LidarMode::_4096x5 = {4096, 5};

bool operator==(const SensorConfig& lhs, const SensorConfig& rhs) {
    return (lhs.udp_dest == rhs.udp_dest && lhs.udp_dest_zm == rhs.udp_dest_zm &&
            lhs.udp_port_lidar == rhs.udp_port_lidar && lhs.udp_port_imu == rhs.udp_port_imu &&
            lhs.udp_port_zm == rhs.udp_port_zm && lhs.udp_multicast_ttl == rhs.udp_multicast_ttl &&
            lhs.udp_multicast_ttl_zm == rhs.udp_multicast_ttl_zm &&
            lhs.timestamp_mode == rhs.timestamp_mode && lhs.lidar_mode == rhs.lidar_mode &&
            lhs.operating_mode == rhs.operating_mode &&
            lhs.multipurpose_io_mode == rhs.multipurpose_io_mode &&
            lhs.lidar_frame_azimuth_offset == rhs.lidar_frame_azimuth_offset &&
            lhs.azimuth_window == rhs.azimuth_window &&
            lhs.signal_multiplier == rhs.signal_multiplier &&
            lhs.nmea_in_polarity == rhs.nmea_in_polarity &&
            lhs.nmea_ignore_valid_char == rhs.nmea_ignore_valid_char &&
            lhs.nmea_baud_rate == rhs.nmea_baud_rate &&
            lhs.nmea_leap_seconds == rhs.nmea_leap_seconds &&
            lhs.sync_pulse_in_polarity == rhs.sync_pulse_in_polarity &&
            lhs.sync_pulse_out_polarity == rhs.sync_pulse_out_polarity &&
            lhs.sync_pulse_out_angle == rhs.sync_pulse_out_angle &&
            lhs.sync_pulse_out_pulse_width == rhs.sync_pulse_out_pulse_width &&
            lhs.sync_pulse_out_frequency == rhs.sync_pulse_out_frequency &&
            lhs.phase_lock_enable == rhs.phase_lock_enable &&
            lhs.phase_lock_offset == rhs.phase_lock_offset &&
            lhs.columns_per_packet == rhs.columns_per_packet &&
            lhs.udp_profile_lidar == rhs.udp_profile_lidar &&
            lhs.udp_profile_imu == rhs.udp_profile_imu && lhs.gyro_fsr == rhs.gyro_fsr &&
            lhs.accel_fsr == rhs.accel_fsr && lhs.return_order == rhs.return_order &&
            lhs.min_range_threshold_cm == rhs.min_range_threshold_cm &&
            lhs.extra_options == rhs.extra_options && lhs.header_type == rhs.header_type &&
            lhs.imu_packets_per_frame == rhs.imu_packets_per_frame &&
            lhs.bloom_reduction_optimization == rhs.bloom_reduction_optimization);
}

bool operator!=(const SensorConfig& lhs, const SensorConfig& rhs) {
    return !(lhs == rhs);
}

std::string to_string(LidarMode mode) {
    return std::to_string(mode.columns) + "x" + std::to_string(mode.fps);
}

optional<LidarMode> lidar_mode_of_string(const std::string& str) {
    try {
        return LidarMode(str);
    } catch (const std::invalid_argument&) {
        return {};
    }
}

std::string to_string(TimestampMode mode) {
    auto res = lookup(impl::TIMESTAMP_MODE_STRINGS, mode);
    return res ? res.value() : "UNKNOWN";
}

optional<TimestampMode> timestamp_mode_of_string(const std::string& str) {
    return rlookup(impl::TIMESTAMP_MODE_STRINGS, str.c_str());
}

std::string to_string(OperatingMode mode) {
    auto res = lookup(impl::OPERATING_MODE_STRINGS, mode);
    return res ? res.value() : "UNKNOWN";
}

optional<OperatingMode> operating_mode_of_string(const std::string& str) {
    return rlookup(impl::OPERATING_MODE_STRINGS, str.c_str());
}

std::string to_string(MultipurposeIOMode mode) {
    auto res = lookup(impl::MULTIPURPOSE_IO_MODE_STRINGS, mode);
    return res ? res.value() : "UNKNOWN";
}

optional<MultipurposeIOMode> multipurpose_io_mode_of_string(const std::string& str) {
    return rlookup(impl::MULTIPURPOSE_IO_MODE_STRINGS, str.c_str());
}

std::string to_string(Polarity polarity) {
    auto res = lookup(impl::POLARITY_STRINGS, polarity);
    return res ? res.value() : "UNKNOWN";
}

optional<Polarity> polarity_of_string(const std::string& str) {
    return rlookup(impl::POLARITY_STRINGS, str.c_str());
}

std::string to_string(NMEABaudRate rate) {
    auto res = lookup(impl::NMEA_BAUD_RATE_STRINGS, rate);
    return res ? res.value() : "UNKNOWN";
}

optional<NMEABaudRate> nmea_baud_rate_of_string(const std::string& str) {
    return rlookup(impl::NMEA_BAUD_RATE_STRINGS, str.c_str());
}

std::string to_string(AzimuthWindow azimuth_window) {
    std::stringstream string_stream;
    string_stream << "[" << azimuth_window.first << ", " << azimuth_window.second << "]";
    return string_stream.str();
}

optional<FullScaleRange> full_scale_range_of_string(const std::string& str) {
    return rlookup(impl::full_scale_range_strings, str.c_str());
}

optional<ReturnOrder> return_order_of_string(const std::string& str) {
    return rlookup(impl::return_order_strings, str.c_str());
}

std::string to_string(ReturnOrder return_order) {
    auto res = lookup(impl::return_order_strings, return_order);
    return res ? res.value() : "UNKNOWN";
}

std::string to_string(FullScaleRange full_scale_range) {
    auto res = lookup(impl::full_scale_range_strings, full_scale_range);
    return res ? res.value() : "UNKNOWN";
}

std::string to_string(BloomReductionOptimization bloom_reduction_optimization) {
    auto res = lookup(impl::bloom_reduction_optimization_strings, bloom_reduction_optimization);
    return res ? res.value() : "UNKNOWN";
}

optional<BloomReductionOptimization> bloom_reduction_optimization_of_string(
    const std::string& str_val) {
    return rlookup(impl::bloom_reduction_optimization_strings, str_val.c_str());
}

void check_signal_multiplier(const double signal_multiplier) {
    std::string signal_multiplier_error =
        "Provided signal multiplier is invalid: " + std::to_string(signal_multiplier) +
        " cannot be converted to one of [0.25, 0.5, 1, 2, 3]";

    std::set<double> valid_values = {0.25, 0.5, 1, 2, 3};
    if (valid_values.count(signal_multiplier) == 0u) {
        throw std::runtime_error(signal_multiplier_error);
    }
}

jsoncons::json config_to_json(const SensorConfig& config) {
    jsoncons::json root;

    if (config.udp_dest) {
        root["udp_dest"] = config.udp_dest.value();
    }

    if (config.udp_dest_zm) {
        root["udp_dest_zm"] = config.udp_dest_zm.value();
    }

    if (config.udp_port_lidar) {
        root["udp_port_lidar"] = config.udp_port_lidar.value();
    }

    if (config.udp_port_imu) {
        root["udp_port_imu"] = config.udp_port_imu.value();
    }

    if (config.udp_port_zm) {
        root["udp_port_zm"] = config.udp_port_zm.value();
    }

    if (config.udp_multicast_ttl) {
        root["udp_multicast_ttl"] = config.udp_multicast_ttl.value();
    }

    if (config.udp_multicast_ttl_zm) {
        root["udp_multicast_ttl_zm"] = config.udp_multicast_ttl_zm.value();
    }

    if (config.timestamp_mode) {
        root["timestamp_mode"] = to_string(config.timestamp_mode.value());
    }

    if (config.lidar_mode) {
        root["lidar_mode"] = to_string(config.lidar_mode.value());
    }

    if (config.operating_mode) {
        auto mode = config.operating_mode.value();
        root["operating_mode"] = to_string(mode);
    }

    if (config.multipurpose_io_mode) {
        root["multipurpose_io_mode"] = to_string(config.multipurpose_io_mode.value());
    }

    if (config.azimuth_window) {
        jsoncons::json azimuth_window(jsoncons::json_array_arg);
        azimuth_window.emplace_back(config.azimuth_window.value().first);
        azimuth_window.emplace_back(config.azimuth_window.value().second);
        root["azimuth_window"] = azimuth_window;
    }

    if (config.lidar_frame_azimuth_offset) {
        root["lidar_frame_azimuth_offset"] = config.lidar_frame_azimuth_offset.value();
    }

    if (config.signal_multiplier) {
        check_signal_multiplier(config.signal_multiplier.value());
        if ((config.signal_multiplier == 0.25) || (config.signal_multiplier == 0.5)) {
            root["signal_multiplier"] = config.signal_multiplier.value();
        } else {
            int signal_multiplier_int = static_cast<int>(config.signal_multiplier.value());
            root["signal_multiplier"] = signal_multiplier_int;
        }
    }

    if (config.sync_pulse_out_angle) {
        root["sync_pulse_out_angle"] = config.sync_pulse_out_angle.value();
    }

    if (config.sync_pulse_out_pulse_width) {
        root["sync_pulse_out_pulse_width"] = config.sync_pulse_out_pulse_width.value();
    }

    if (config.nmea_in_polarity) {
        root["nmea_in_polarity"] = to_string(config.nmea_in_polarity.value());
    }

    if (config.nmea_baud_rate) {
        root["nmea_baud_rate"] = to_string(config.nmea_baud_rate.value());
    }

    if (config.nmea_ignore_valid_char) {
        root["nmea_ignore_valid_char"] = config.nmea_ignore_valid_char.value() ? 1 : 0;
    }

    if (config.nmea_leap_seconds) {
        root["nmea_leap_seconds"] = config.nmea_leap_seconds.value();
    }

    if (config.sync_pulse_in_polarity) {
        root["sync_pulse_in_polarity"] = to_string(config.sync_pulse_in_polarity.value());
    }

    if (config.sync_pulse_out_polarity) {
        root["sync_pulse_out_polarity"] = to_string(config.sync_pulse_out_polarity.value());
    }

    if (config.sync_pulse_out_frequency) {
        root["sync_pulse_out_frequency"] = config.sync_pulse_out_frequency.value();
    }

    if (config.phase_lock_enable) {
        root["phase_lock_enable"] = config.phase_lock_enable.value();
    }

    if (config.phase_lock_offset) {
        root["phase_lock_offset"] = config.phase_lock_offset.value();
    }

    if (config.columns_per_packet) {
        root["columns_per_packet"] = config.columns_per_packet.value();
    }

    if (config.udp_profile_lidar) {
        root["udp_profile_lidar"] = to_string(config.udp_profile_lidar.value());
    }

    if (config.udp_profile_imu) {
        root["udp_profile_imu"] = to_string(config.udp_profile_imu.value());
    }

    // Firmware 3.2 and higher only
    if (config.header_type) {
        root["header_type"] = to_string(config.header_type.value());
    }

    if (config.bloom_reduction_optimization) {
        root["bloom_reduction_optimization"] =
            to_string(config.bloom_reduction_optimization.value());
    }

    if (config.imu_packets_per_frame) {
        root["imu_packets_per_frame"] = config.imu_packets_per_frame.value();
    }

    // Firmware 3.1 and higher options
    if (config.gyro_fsr) {
        root["gyro_fsr"] = to_string(config.gyro_fsr.value());
    }

    if (config.accel_fsr) {
        root["accel_fsr"] = to_string(config.accel_fsr.value());
    }

    if (config.min_range_threshold_cm) {
        root["min_range_threshold_cm"] = config.min_range_threshold_cm.value();
    }

    if (config.return_order) {
        root["return_order"] = to_string(config.return_order.value());
    }

    for (const auto& kv : config.extra_options) {
        try {
            root[kv.first] = jsoncons::json::parse(kv.second);
        } catch (const std::runtime_error& e) {
            throw std::runtime_error("Failed to parse config extra_options['" + kv.first +
                                     "'] as json: " + e.what());
        }
    }

    return root;
}

std::string to_string(const SensorConfig& config) {
    jsoncons::json root = config_to_json(config);
    std::string out;
    root.dump(out);
    return out;
}

SensorConfig::SensorConfig(const std::string& config_json) {
    ValidatorIssues issues;
    if (!parse_and_validate_config(config_json, *this, issues)) {
        throw std::runtime_error(to_string(issues.critical));
    }
}

SensorConfig::SensorConfig() = default;

}  // namespace core
}  // namespace sdk
}  // namespace ouster

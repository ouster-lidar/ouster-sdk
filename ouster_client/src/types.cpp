/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <nonstd/optional.hpp>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "ouster/defaults.h"
#include "ouster/impl/build.h"
#include "ouster/impl/logging.h"
#include "ouster/metadata.h"
#include "ouster/version.h"

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;
using std::stoul;

namespace ouster {
namespace sdk {
namespace core {

namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

extern const Table<LidarMode, const char*, 7> LIDAR_MODE_STRINGS{
    {{LidarMode::UNSPECIFIED, "UNKNOWN"},
     {LidarMode::_512x10, "512x10"},
     {LidarMode::_512x20, "512x20"},
     {LidarMode::_1024x10, "1024x10"},
     {LidarMode::_1024x20, "1024x20"},
     {LidarMode::_2048x10, "2048x10"},
     {LidarMode::_4096x5, "4096x5"}}};

extern const Table<TimestampMode, const char*, 4> TIMESTAMP_MODE_STRINGS{
    {{TimestampMode::UNSPECIFIED, "UNKNOWN"},
     {TimestampMode::TIME_FROM_INTERNAL_OSC, "TIME_FROM_INTERNAL_OSC"},
     {TimestampMode::TIME_FROM_SYNC_PULSE_IN, "TIME_FROM_SYNC_PULSE_IN"},
     {TimestampMode::TIME_FROM_PTP_1588, "TIME_FROM_PTP_1588"}}};

extern const Table<OperatingMode, const char*, 2> OPERATING_MODE_STRINGS{
    {{OperatingMode::NORMAL, "NORMAL"}, {OperatingMode::STANDBY, "STANDBY"}}};

extern const Table<MultipurposeIOMode, const char*, 6>
    MULTIPURPOSE_IO_MODE_STRINGS{
        {{MultipurposeIOMode::OFF, "OFF"},
         {MultipurposeIOMode::INPUT_NMEA_UART, "INPUT_NMEA_UART"},
         {MultipurposeIOMode::OUTPUT_FROM_INTERNAL_OSC,
          "OUTPUT_FROM_INTERNAL_OSC"},
         {MultipurposeIOMode::OUTPUT_FROM_SYNC_PULSE_IN,
          "OUTPUT_FROM_SYNC_PULSE_IN"},
         {MultipurposeIOMode::OUTPUT_FROM_PTP_1588, "OUTPUT_FROM_PTP_1588"},
         {MultipurposeIOMode::OUTPUT_FROM_ENCODER_ANGLE,
          "OUTPUT_FROM_ENCODER_ANGLE"}}};

extern const Table<Polarity, const char*, 2> POLARITY_STRINGS{
    {{Polarity::ACTIVE_LOW, "ACTIVE_LOW"},
     {Polarity::ACTIVE_HIGH, "ACTIVE_HIGH"}}};

#if defined(BAUD_9600)
#undef BAUD_9600
#endif
#if defined(BAUD_115200)
#undef BAUD_115200
#endif
// NOTE[UN]: Not sure we prefix the strings with BAUD_ anymore
extern const Table<NMEABaudRate, const char*, 2> NMEA_BAUD_RATE_STRINGS{
    {{NMEABaudRate::BAUD_9600, "BAUD_9600"},
     {NMEABaudRate::BAUD_115200, "BAUD_115200"}}};

Table<UDPProfileLidar, const char*, MAX_NUM_PROFILES> udp_profile_lidar_strings{
    {
        {UDPProfileLidar::UNKNOWN, "UNKNOWN"},
        {UDPProfileLidar::LEGACY, "LEGACY"},
        {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
         "RNG19_RFL8_SIG16_NIR16_DUAL"},
        {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, "RNG19_RFL8_SIG16_NIR16"},
        {UDPProfileLidar::RNG15_RFL8_NIR8, "RNG15_RFL8_NIR8"},
        {UDPProfileLidar::FIVE_WORD_PIXEL, "FIVE_WORD_PIXEL"},
        {UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL,
         "FUSA_RNG15_RFL8_NIR8_DUAL"},
        {UDPProfileLidar::RNG15_RFL8_NIR8_DUAL, "RNG15_RFL8_NIR8_DUAL"},
        {UDPProfileLidar::RNG15_RFL8_NIR8_ZONE16, "RNG15_RFL8_NIR8_ZONE16"},
        {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_ZONE16,
         "RNG19_RFL8_SIG16_NIR16_ZONE16"},
        {UDPProfileLidar::OFF, "OFF"},
    }};

Table<UDPProfileIMU, const char*, 3> udp_profile_imu_strings{{
    {UDPProfileIMU::LEGACY, "LEGACY"},
    {UDPProfileIMU::ACCEL32_GYRO32_NMEA, "ACCEL32_GYRO32_NMEA"},
    {UDPProfileIMU::OFF, "OFF"},
}};

Table<HeaderType, const char*, 2> udp_profile_type_strings{{
    {HeaderType::STANDARD, "STANDARD"},
    {HeaderType::FUSA, "FUSA"},
}};

Table<FullScaleRange, const char*, 2> full_scale_range_strings{{
    {FullScaleRange::NORMAL, "NORMAL"},
    {FullScaleRange::EXTENDED, "EXTENDED"},
}};

Table<ReturnOrder, const char*, 5> return_order_strings{{
    {ReturnOrder::STRONGEST_TO_WEAKEST, "STRONGEST_TO_WEAKEST"},
    {ReturnOrder::FARTHEST_TO_NEAREST, "FARTHEST_TO_NEAREST"},
    {ReturnOrder::NEAREST_TO_FARTHEST, "NEAREST_TO_FARTHEST"},
    {ReturnOrder::DEPRECATED_STRONGEST_RETURN_FIRST, "STRONGEST_RETURN_FIRST"},
    {ReturnOrder::DEPRECATED_LAST_RETURN_FIRST, "LAST_RETURN_FIRST"},
}};

// NOTE[UN]: This enum and next one stand out as they embed the prefix in the
// string (before my change)
Table<ShotLimitingStatus, const char*, 10> shot_limiting_status_strings{{
    {ShotLimitingStatus::NORMAL, "NORMAL"},
    {ShotLimitingStatus::IMMINENT, "IMMINENT"},
    {ShotLimitingStatus::REDUCTION_0_10, "REDUCTION_0_10"},
    {ShotLimitingStatus::REDUCTION_10_20, "REDUCTION_10_20"},
    {ShotLimitingStatus::REDUCTION_20_30, "REDUCTION_20_30"},
    {ShotLimitingStatus::REDUCTION_30_40, "REDUCTION_30_40"},
    {ShotLimitingStatus::REDUCTION_40_50, "REDUCTION_40_50"},
    {ShotLimitingStatus::REDUCTION_50_60, "REDUCTION_50_60"},
    {ShotLimitingStatus::REDUCTION_60_70, "REDUCTION_60_70"},
    {ShotLimitingStatus::REDUCTION_70_75, "REDUCTION_70_75"},
}};

// NOTE[UN]: Same as ShotLimitingStatus, this enum and next one stand out as
// they used to embed the prefix in the string
Table<ThermalShutdownStatus, const char*, 2> thermal_shutdown_status_strings{{
    {ThermalShutdownStatus::NORMAL, "NORMAL"},
    {ThermalShutdownStatus::IMMINENT, "IMMINENT"},
}};

Table<BloomReductionOptimization, const char*, 2>
    bloom_reduction_optimization_strings{{
        {BloomReductionOptimization::BALANCED, "BALANCED"},
        {BloomReductionOptimization::MINIMIZE_FALSE_POSITIVES,
         "MINIMIZE_FALSE_POSITIVES"},
    }};

// clang-format on

}  // namespace impl

/* Equality operators */

bool operator==(const CalibrationStatus& lhs, const CalibrationStatus& rhs) {
    return (lhs.reflectivity_status == rhs.reflectivity_status &&
            lhs.reflectivity_timestamp == rhs.reflectivity_timestamp);
}

bool operator!=(const CalibrationStatus& lhs, const CalibrationStatus& rhs) {
    return !(lhs == rhs);
}

bool operator==(const SensorConfig& lhs, const SensorConfig& rhs) {
    return (
        lhs.udp_dest == rhs.udp_dest && lhs.udp_dest_zm == rhs.udp_dest_zm &&
        lhs.udp_port_lidar == rhs.udp_port_lidar &&
        lhs.udp_port_imu == rhs.udp_port_imu &&
        lhs.udp_port_zm == rhs.udp_port_zm &&
        lhs.udp_multicast_ttl == rhs.udp_multicast_ttl &&
        lhs.udp_multicast_ttl_zm == rhs.udp_multicast_ttl_zm &&
        lhs.timestamp_mode == rhs.timestamp_mode &&
        lhs.lidar_mode == rhs.lidar_mode &&
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
        lhs.udp_profile_imu == rhs.udp_profile_imu &&
        lhs.gyro_fsr == rhs.gyro_fsr && lhs.accel_fsr == rhs.accel_fsr &&
        lhs.return_order == rhs.return_order &&
        lhs.min_range_threshold_cm == rhs.min_range_threshold_cm &&
        lhs.extra_options == rhs.extra_options &&
        lhs.header_type == rhs.header_type &&
        lhs.imu_packets_per_frame == rhs.imu_packets_per_frame &&
        lhs.bloom_reduction_optimization == rhs.bloom_reduction_optimization);
}

bool operator!=(const SensorConfig& lhs, const SensorConfig& rhs) {
    return !(lhs == rhs);
}

/* Misc operations */

uint32_t n_cols_of_lidar_mode(LidarMode mode) {
    switch (mode) {
        case LidarMode::_512x10:
        case LidarMode::_512x20:
            return 512;
        case LidarMode::_1024x10:
        case LidarMode::_1024x20:
            return 1024;
        case LidarMode::_2048x10:
            return 2048;
        case LidarMode::_4096x5:
            return 4096;
        default:
            throw std::invalid_argument{"n_cols_of_lidar_mode"};
    }
}

int frequency_of_lidar_mode(LidarMode mode) {
    switch (mode) {
        case LidarMode::_4096x5:
            return 5;
        case LidarMode::_512x10:
        case LidarMode::_1024x10:
        case LidarMode::_2048x10:
            return 10;
        case LidarMode::_512x20:
        case LidarMode::_1024x20:
            return 20;
        default:
            throw std::invalid_argument{"frequency_of_lidar_mode"};
    }
}

std::string client_version() {
    return std::string("ouster_client ").append(ouster::sdk::SDK_VERSION);
}

/* String conversion */

template <typename K, typename V, size_t N>
static optional<V> lookup(const impl::Table<K, V, N>& table, const K& key) {
    auto end = table.end();
    auto res = std::find_if(
        table.begin(), end,
        [&](const std::pair<K, V>& pair) { return pair.first == key; });

    return res == end ? nullopt : make_optional<V>(res->second);
}

template <typename K, size_t N>
static optional<K> rlookup(const impl::Table<K, const char*, N>& table,
                           const char* value) {
    auto end = table.end();
    auto res = std::find_if(
        table.begin(), end, [&](const std::pair<K, const char*>& pair) {
            return pair.second && std::strcmp(pair.second, value) == 0;
        });

    return res == end ? nullopt : make_optional<K>(res->first);
}

std::string to_string(LidarMode mode) {
    auto res = lookup(impl::LIDAR_MODE_STRINGS, mode);
    return res ? res.value() : "UNKNOWN";
}

LidarMode lidar_mode_of_string(const std::string& str) {
    auto res = rlookup(impl::LIDAR_MODE_STRINGS, str.c_str());
    return res ? res.value() : LidarMode::UNSPECIFIED;
}

std::string to_string(TimestampMode mode) {
    auto res = lookup(impl::TIMESTAMP_MODE_STRINGS, mode);
    return res ? res.value() : "UNKNOWN";
}

TimestampMode timestamp_mode_of_string(const std::string& str) {
    auto res = rlookup(impl::TIMESTAMP_MODE_STRINGS, str.c_str());
    return res ? res.value() : TimestampMode::UNSPECIFIED;
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

optional<MultipurposeIOMode> multipurpose_io_mode_of_string(
    const std::string& str) {
    return rlookup(impl::MULTIPURPOSE_IO_MODE_STRINGS, str.c_str());
}

std::string to_string(Polarity polarity) {
    auto res = lookup(impl::POLARITY_STRINGS, polarity);
    return res ? res.value() : "UNKNOWN";
}

optional<Polarity> polarity_of_string(const std::string& str) {
    return rlookup(impl::POLARITY_STRINGS, str.c_str());
}

optional<FullScaleRange> full_scale_range_of_string(const std::string& str) {
    return rlookup(impl::full_scale_range_strings, str.c_str());
}

optional<ReturnOrder> return_order_of_string(const std::string& str) {
    return rlookup(impl::return_order_strings, str.c_str());
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
    string_stream << "[" << azimuth_window.first << ", "
                  << azimuth_window.second << "]";
    return string_stream.str();
}

std::string to_string(UDPProfileLidar profile) {
    auto res = lookup(impl::udp_profile_lidar_strings, profile);
    return res ? res.value() : "UNKNOWN";
}

optional<UDPProfileLidar> udp_profile_lidar_of_string(const std::string& str) {
    return rlookup(impl::udp_profile_lidar_strings, str.c_str());
}

std::string to_string(UDPProfileIMU profile) {
    auto res = lookup(impl::udp_profile_imu_strings, profile);
    return res ? res.value() : "UNKNOWN";
}

optional<UDPProfileIMU> udp_profile_imu_of_string(const std::string& str) {
    return rlookup(impl::udp_profile_imu_strings, str.c_str());
}

std::string to_string(HeaderType profile) {
    auto res = lookup(impl::udp_profile_type_strings, profile);
    return res ? res.value() : "UNKNOWN";
}

optional<HeaderType> udp_profile_type_of_string(const std::string& str) {
    return rlookup(impl::udp_profile_type_strings, str.c_str());
}

std::string to_string(ShotLimitingStatus shot_limiting_status) {
    auto res = lookup(impl::shot_limiting_status_strings, shot_limiting_status);
    return res ? res.value() : "UNKNOWN";
}

std::string to_string(ThermalShutdownStatus thermal_shutdown_status) {
    auto res =
        lookup(impl::thermal_shutdown_status_strings, thermal_shutdown_status);
    return res ? res.value() : "UNKNOWN";
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
    auto res = lookup(impl::bloom_reduction_optimization_strings,
                      bloom_reduction_optimization);
    return res ? res.value() : "UNKNOWN";
}

optional<BloomReductionOptimization> bloom_reduction_optimization_of_string(
    const std::string& s) {
    return rlookup(impl::bloom_reduction_optimization_strings, s.c_str());
}

void check_signal_multiplier(const double signal_multiplier) {
    std::string signal_multiplier_error =
        "Provided signal multiplier is invalid: " +
        std::to_string(signal_multiplier) +
        " cannot be converted to one of [0.25, 0.5, 1, 2, 3]";

    std::set<double> valid_values = {0.25, 0.5, 1, 2, 3};
    if (valid_values.count(signal_multiplier) == 0u) {
        throw std::runtime_error(signal_multiplier_error);
    }
}

jsoncons::json cal_to_json(const CalibrationStatus& cal) {
    jsoncons::json root;

    if (cal.reflectivity_status) {
        root["reflectivity"]["valid"] = cal.reflectivity_status.value();
    }
    if (cal.reflectivity_timestamp) {
        root["reflectivity"]["timestamp"] = cal.reflectivity_timestamp.value();
    }

    return root;
}

std::string to_string(const CalibrationStatus& cal) {
    auto root = cal_to_json(cal);
    std::string out;
    root.dump(out);
    return out;
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
        root["multipurpose_io_mode"] =
            to_string(config.multipurpose_io_mode.value());
    }

    if (config.azimuth_window) {
        jsoncons::json azimuth_window(jsoncons::json_array_arg);
        azimuth_window.emplace_back(config.azimuth_window.value().first);
        azimuth_window.emplace_back(config.azimuth_window.value().second);
        root["azimuth_window"] = azimuth_window;
    }

    if (config.lidar_frame_azimuth_offset) {
        root["lidar_frame_azimuth_offset"] =
            config.lidar_frame_azimuth_offset.value();
    }

    if (config.signal_multiplier) {
        check_signal_multiplier(config.signal_multiplier.value());
        if ((config.signal_multiplier == 0.25) ||
            (config.signal_multiplier == 0.5)) {
            root["signal_multiplier"] = config.signal_multiplier.value();
        } else {
            int signal_multiplier_int =
                static_cast<int>(config.signal_multiplier.value());
            root["signal_multiplier"] = signal_multiplier_int;
        }
    }

    if (config.sync_pulse_out_angle) {
        root["sync_pulse_out_angle"] = config.sync_pulse_out_angle.value();
    }

    if (config.sync_pulse_out_pulse_width) {
        root["sync_pulse_out_pulse_width"] =
            config.sync_pulse_out_pulse_width.value();
    }

    if (config.nmea_in_polarity) {
        root["nmea_in_polarity"] = to_string(config.nmea_in_polarity.value());
    }

    if (config.nmea_baud_rate) {
        root["nmea_baud_rate"] = to_string(config.nmea_baud_rate.value());
    }

    if (config.nmea_ignore_valid_char) {
        root["nmea_ignore_valid_char"] =
            config.nmea_ignore_valid_char.value() ? 1 : 0;
    }

    if (config.nmea_leap_seconds) {
        root["nmea_leap_seconds"] = config.nmea_leap_seconds.value();
    }

    if (config.sync_pulse_in_polarity) {
        root["sync_pulse_in_polarity"] =
            to_string(config.sync_pulse_in_polarity.value());
    }

    if (config.sync_pulse_out_polarity) {
        root["sync_pulse_out_polarity"] =
            to_string(config.sync_pulse_out_polarity.value());
    }

    if (config.sync_pulse_out_frequency) {
        root["sync_pulse_out_frequency"] =
            config.sync_pulse_out_frequency.value();
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
        } catch (std::runtime_error& e) {
            throw std::runtime_error("Failed to parse config extra_options['" +
                                     kv.first + "'] as json: " + e.what());
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

ProductInfo ProductInfo::create_product_info(std::string product_info_string) {
    std::regex product_regex(
        R"(^(\w+)-(\d+|DOME)?(?:-(\d+))?(?:-((?!SR)\w+))?-?(SR)?)");
    std::smatch matches;
    if (!product_info_string.empty()) {
        if (regex_search(product_info_string, matches, product_regex)) {
            std::string form_factor = matches.str(1) + matches.str(2);
            bool short_range = (!matches.str(5).empty());
            auto beam_config = matches.str(4);
            if (beam_config.empty()) {
                beam_config = "U";
            }
            int beam_count;
            try {
                beam_count = stoi(matches.str(3));
            } catch (const std::exception& e) {
                beam_count = 0;
            }

            return ProductInfo(product_info_string, form_factor, short_range,
                               beam_config, beam_count);
        } else {
            throw std::runtime_error("Product Info \"" + product_info_string +
                                     "\" is not a recognized product info");
        }
    }
    return ProductInfo();
}

ProductInfo::ProductInfo() : ProductInfo("", "", false, "", 0){};

ProductInfo::ProductInfo(std::string product_info_string,
                         std::string form_factor, bool short_range,
                         std::string beam_config, int beam_count)
    : full_product_info(product_info_string),
      form_factor(form_factor),
      short_range(short_range),
      beam_config(beam_config),
      beam_count(beam_count) {}

bool operator==(const ProductInfo& lhs, const ProductInfo& rhs) {
    return lhs.full_product_info == rhs.full_product_info &&
           lhs.form_factor == rhs.form_factor &&
           lhs.short_range == rhs.short_range &&
           lhs.beam_config == rhs.beam_config &&
           lhs.beam_count == rhs.beam_count;
}

bool operator!=(const ProductInfo& lhs, const ProductInfo& rhs) {
    return !(lhs == rhs);
}

std::string to_string(const ProductInfo& info) {
    std::stringstream output;
    output << "Product Info: " << std::endl;
    output << "\tFull Product Info: \"" << info.full_product_info << "\""
           << std::endl;
    output << "\tForm Factor: \"" << info.form_factor << "\"" << std::endl;
    output << "\tShort Range: \"" << info.short_range << "\"" << std::endl;
    output << "\tBeam Config: \"" << info.beam_config << "\"" << std::endl;
    output << "\tBeam Count: \"" << info.beam_count << "\"" << std::endl;
    return output.str();
}

int DataFormat::valid_columns_per_frame() const {
    auto start = column_window.first;
    auto end = column_window.second;

    if (start <= end) {
        return end - start + 1;
    } else {
        return end + (columns_per_frame - start) + 1;
    }
}

int DataFormat::lidar_packets_per_frame() const {
    if (udp_profile_lidar == UDPProfileLidar::OFF) {
        return 0;
    }

    int start_packet = column_window.first / columns_per_packet;
    int end_packet = column_window.second / columns_per_packet;
    if (column_window.second < column_window.first) {
        // the valid azimuth window wraps through 0
        // Determine the number of packets for a full frame with no window
        int max_packets =
            (columns_per_frame / columns_per_packet) +
            (((columns_per_frame % columns_per_packet) != 0u) ? 1 : 0);
        // We expect to get [start, max_packets] and [0, end] packet indexes
        int expected_packets = (max_packets - start_packet) + 1 + end_packet;
        // If start and end packets are the same, we have every packet
        if (start_packet == end_packet) {
            return max_packets;
        }
        return expected_packets;
    } else {
        // no wrapping of azimuth the window through 0
        return end_packet - start_packet + 1;
    }
}

Version version_from_string(const std::string& version_string) {
    auto rgx = std::regex(
        R"((([\w\d]*)-([\w\d]*)-)?v?(\d*)\.(\d*)\.(\d*)-?([\d\w.]*)?\+?([\d\w.]*)?)");
    std::smatch matches;
    std::regex_search(version_string, matches, rgx);

    if (matches.size() < 9) {
        return INVALID_VERSION;
    }

    try {
        Version version_val;
        version_val.major = static_cast<uint16_t>(stoul(matches[4]));
        version_val.minor = static_cast<uint16_t>(stoul(matches[5]));
        version_val.patch = static_cast<uint16_t>(stoul(matches[6]));
        version_val.stage = matches[2];
        version_val.machine = matches[3];
        version_val.prerelease = matches[7];
        version_val.build = matches[8];
        return version_val;
    } catch (const std::exception&) {
        return INVALID_VERSION;
    }
}
const Version& invalid_version = INVALID_VERSION;
}  // namespace core
}  // namespace sdk
}  // namespace ouster

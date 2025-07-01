/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/defaults.h"
#include "ouster/impl/build.h"
#include "ouster/impl/logging.h"
#include "ouster/metadata.h"
#include "ouster/version.h"

namespace ouster {

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;
using std::stoul;

namespace sensor {

namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

extern const Table<lidar_mode, const char*, 7> lidar_mode_strings{
    {{MODE_UNSPEC, "UNKNOWN"},
     {MODE_512x10, "512x10"},
     {MODE_512x20, "512x20"},
     {MODE_1024x10, "1024x10"},
     {MODE_1024x20, "1024x20"},
     {MODE_2048x10, "2048x10"},
     {MODE_4096x5, "4096x5"}}};

extern const Table<timestamp_mode, const char*, 4> timestamp_mode_strings{
    {{TIME_FROM_UNSPEC, "UNKNOWN"},
     {TIME_FROM_INTERNAL_OSC, "TIME_FROM_INTERNAL_OSC"},
     {TIME_FROM_SYNC_PULSE_IN, "TIME_FROM_SYNC_PULSE_IN"},
     {TIME_FROM_PTP_1588, "TIME_FROM_PTP_1588"}}};

extern const Table<OperatingMode, const char*, 2> operating_mode_strings{
    {{OPERATING_NORMAL, "NORMAL"}, {OPERATING_STANDBY, "STANDBY"}}};

extern const Table<MultipurposeIOMode, const char*, 6>
    multipurpose_io_mode_strings{
        {{MULTIPURPOSE_OFF, "OFF"},
         {MULTIPURPOSE_INPUT_NMEA_UART, "INPUT_NMEA_UART"},
         {MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC, "OUTPUT_FROM_INTERNAL_OSC"},
         {MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN, "OUTPUT_FROM_SYNC_PULSE_IN"},
         {MULTIPURPOSE_OUTPUT_FROM_PTP_1588, "OUTPUT_FROM_PTP_1588"},
         {MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE,
          "OUTPUT_FROM_ENCODER_ANGLE"}}};

extern const Table<Polarity, const char*, 2> polarity_strings{
    {{POLARITY_ACTIVE_LOW, "ACTIVE_LOW"},
     {POLARITY_ACTIVE_HIGH, "ACTIVE_HIGH"}}};

#if defined(BAUD_9600)
#undef BAUD_9600
#endif
#if defined(BAUD_115200)
#undef BAUD_115200
#endif
extern const Table<NMEABaudRate, const char*, 2> nmea_baud_rate_strings{
    {{NMEABaudRate::BAUD_9600, "BAUD_9600"},
     {NMEABaudRate::BAUD_115200, "BAUD_115200"}}};

Table<UDPProfileLidar, const char*, MAX_NUM_PROFILES> udp_profile_lidar_strings{
    {
        {PROFILE_LIDAR_UNKNOWN, "UNKNOWN"},
        {PROFILE_LIDAR_LEGACY, "LEGACY"},
        {PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL, "RNG19_RFL8_SIG16_NIR16_DUAL"},
        {PROFILE_RNG19_RFL8_SIG16_NIR16, "RNG19_RFL8_SIG16_NIR16"},
        {PROFILE_RNG15_RFL8_NIR8, "RNG15_RFL8_NIR8"},
        {PROFILE_FIVE_WORD_PIXEL, "FIVE_WORD_PIXEL"},
        {PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL, "FUSA_RNG15_RFL8_NIR8_DUAL"},
    }};

Table<UDPProfileIMU, const char*, 1> udp_profile_imu_strings{{
    {PROFILE_IMU_LEGACY, "LEGACY"},
}};

Table<FullScaleRange, const char*, 2> full_scale_range_strings{{
    {FSR_NORMAL, "NORMAL"},
    {FSR_EXTENDED, "EXTENDED"},
}};

Table<ReturnOrder, const char*, 5> return_order_strings{{
    {ORDER_STRONGEST_TO_WEAKEST, "STRONGEST_TO_WEAKEST"},
    {ORDER_FARTHEST_TO_NEAREST, "FARTHEST_TO_NEAREST"},
    {ORDER_NEAREST_TO_FARTHEST, "NEAREST_TO_FARTHEST"},
    {ORDER_DEPRECATED_STRONGEST_RETURN_FIRST, "STRONGEST_RETURN_FIRST"},
    {ORDER_DEPRECATED_LAST_RETURN_FIRST, "LAST_RETURN_FIRST"},
}};

// TODO: should we name them something better? feel like the most important is
// SHOT_LIMITING_NORMAL
Table<ShotLimitingStatus, const char*, 10> shot_limiting_status_strings{{
    {SHOT_LIMITING_NORMAL, "SHOT_LIMITING_NORMAL"},
    {SHOT_LIMITING_IMMINENT, "SHOT_LIMITING_IMMINENT"},
    {SHOT_LIMITING_REDUCTION_0_10, "SHOT_LIMITING_REDUCTION_0_10"},
    {SHOT_LIMITING_REDUCTION_10_20, "SHOT_LIMITING_REDUCTION_10_20"},
    {SHOT_LIMITING_REDUCTION_20_30, "SHOT_LIMITING_REDUCTION_20_30"},
    {SHOT_LIMITING_REDUCTION_30_40, "SHOT_LIMITING_REDUCTION_30_40"},
    {SHOT_LIMITING_REDUCTION_40_50, "SHOT_LIMITING_REDUCTION_40_50"},
    {SHOT_LIMITING_REDUCTION_50_60, "SHOT_LIMITING_REDUCTION_50_60"},
    {SHOT_LIMITING_REDUCTION_60_70, "SHOT_LIMITING_REDUCTION_60_70"},
    {SHOT_LIMITING_REDUCTION_70_75, "SHOT_LIMITING_REDUCTION_70_75"},
}};

// TODO: do we want these? do we like the names?
Table<ThermalShutdownStatus, const char*, 2> thermal_shutdown_status_strings{{
    {THERMAL_SHUTDOWN_NORMAL, "THERMAL_SHUTDOWN_NORMAL"},
    {THERMAL_SHUTDOWN_IMMINENT, "THERMAL_SHUTDOWN_IMMINENT"},
}};

}  // namespace impl

/* Equality operators */

bool operator==(const data_format& lhs, const data_format& rhs) {
    return (lhs.pixels_per_column == rhs.pixels_per_column &&
            lhs.columns_per_packet == rhs.columns_per_packet &&
            lhs.columns_per_frame == rhs.columns_per_frame &&
            lhs.pixel_shift_by_row == rhs.pixel_shift_by_row &&
            lhs.column_window == rhs.column_window &&
            lhs.udp_profile_lidar == rhs.udp_profile_lidar &&
            lhs.udp_profile_imu == rhs.udp_profile_imu && lhs.fps == rhs.fps);
}

bool operator!=(const data_format& lhs, const data_format& rhs) {
    return !(lhs == rhs);
}

bool operator==(const calibration_status& lhs, const calibration_status& rhs) {
    return (lhs.reflectivity_status == rhs.reflectivity_status &&
            lhs.reflectivity_timestamp == rhs.reflectivity_timestamp);
}

bool operator!=(const calibration_status& lhs, const calibration_status& rhs) {
    return !(lhs == rhs);
}

bool operator==(const sensor_config& lhs, const sensor_config& rhs) {
    return (lhs.udp_dest == rhs.udp_dest &&
            lhs.udp_port_lidar == rhs.udp_port_lidar &&
            lhs.udp_port_imu == rhs.udp_port_imu &&
            lhs.timestamp_mode == rhs.timestamp_mode &&
            lhs.lidar_mode == rhs.lidar_mode &&
            lhs.operating_mode == rhs.operating_mode &&
            lhs.multipurpose_io_mode == rhs.multipurpose_io_mode &&
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
            lhs.extra_options == rhs.extra_options);
}

bool operator!=(const sensor_config& lhs, const sensor_config& rhs) {
    return !(lhs == rhs);
}

/* Default values */

static ColumnWindow default_column_window(uint32_t columns_per_frame) {
    return {0, columns_per_frame - 1};
}

data_format default_data_format(lidar_mode mode) {
    auto repeat = [](int n, const std::vector<int>& v) {
        std::vector<int> res{};
        for (int i = 0; i < n; i++) res.insert(res.end(), v.begin(), v.end());
        return res;
    };

    uint32_t pixels_per_column = 64;
    uint32_t columns_per_packet = DEFAULT_COLUMNS_PER_PACKET;
    uint32_t columns_per_frame = n_cols_of_lidar_mode(mode);
    ColumnWindow column_window = default_column_window(columns_per_frame);

    std::vector<int> offset;
    switch (columns_per_frame) {
        case 512:
            offset = repeat(16, {9, 6, 3, 0});
            break;
        case 1024:
            offset = repeat(16, {18, 12, 6, 0});
            break;
        case 2048:
            offset = repeat(16, {36, 24, 12, 0});
            break;
        case 4096:
            offset = repeat(16, {72, 48, 24, 0});
            break;
        default:
            throw std::invalid_argument{"default_data_format"};
            break;
    }

    return {pixels_per_column,
            columns_per_packet,
            columns_per_frame,
            offset,
            column_window,
            UDPProfileLidar::PROFILE_LIDAR_LEGACY,
            UDPProfileIMU::PROFILE_IMU_LEGACY,
            static_cast<uint16_t>(frequency_of_lidar_mode(mode))};
}

double default_lidar_origin_to_beam_origin(std::string prod_line) {
    double lidar_origin_to_beam_origin_mm = 12.163;  // default for gen 1
    if (prod_line.find("OS-0-") == 0)
        lidar_origin_to_beam_origin_mm = 27.67;
    else if (prod_line.find("OS-1-") == 0)
        lidar_origin_to_beam_origin_mm = 15.806;
    else if (prod_line.find("OS-2-") == 0)
        lidar_origin_to_beam_origin_mm = 13.762;
    return lidar_origin_to_beam_origin_mm;
}

mat4d default_beam_to_lidar_transform(std::string prod_line) {
    mat4d beam_to_lidar_transform = mat4d::Identity();
    beam_to_lidar_transform(0, 3) =
        default_lidar_origin_to_beam_origin(prod_line);
    return beam_to_lidar_transform;
}

calibration_status default_calibration_status() { return calibration_status{}; }

/* Misc operations */

uint32_t n_cols_of_lidar_mode(lidar_mode mode) {
    switch (mode) {
        case MODE_512x10:
        case MODE_512x20:
            return 512;
        case MODE_1024x10:
        case MODE_1024x20:
            return 1024;
        case MODE_2048x10:
            return 2048;
        case MODE_4096x5:
            return 4096;
        default:
            throw std::invalid_argument{"n_cols_of_lidar_mode"};
    }
}

int frequency_of_lidar_mode(lidar_mode mode) {
    switch (mode) {
        case MODE_4096x5:
            return 5;
        case MODE_512x10:
        case MODE_1024x10:
        case MODE_2048x10:
            return 10;
        case MODE_512x20:
        case MODE_1024x20:
            return 20;
        default:
            throw std::invalid_argument{"frequency_of_lidar_mode"};
    }
}

std::string client_version() {
    return std::string("ouster_client ").append(ouster::SDK_VERSION);
}

/* String conversion */

template <typename K, typename V, size_t N>
static optional<V> lookup(const impl::Table<K, V, N>& table, const K& k) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end, [&](const std::pair<K, V>& p) {
        return p.first == k;
    });

    return res == end ? nullopt : make_optional<V>(res->second);
}

template <typename K, size_t N>
static optional<K> rlookup(const impl::Table<K, const char*, N>& table,
                           const char* v) {
    auto end = table.end();
    auto res = std::find_if(
        table.begin(), end, [&](const std::pair<K, const char*>& p) {
            return p.second && std::strcmp(p.second, v) == 0;
        });

    return res == end ? nullopt : make_optional<K>(res->first);
}

std::string to_string(lidar_mode mode) {
    auto res = lookup(impl::lidar_mode_strings, mode);
    return res ? res.value() : "UNKNOWN";
}

lidar_mode lidar_mode_of_string(const std::string& s) {
    auto res = rlookup(impl::lidar_mode_strings, s.c_str());
    return res ? res.value() : lidar_mode::MODE_UNSPEC;
}

std::string to_string(timestamp_mode mode) {
    auto res = lookup(impl::timestamp_mode_strings, mode);
    return res ? res.value() : "UNKNOWN";
}

timestamp_mode timestamp_mode_of_string(const std::string& s) {
    auto res = rlookup(impl::timestamp_mode_strings, s.c_str());
    return res ? res.value() : timestamp_mode::TIME_FROM_UNSPEC;
}

std::string to_string(OperatingMode mode) {
    auto res = lookup(impl::operating_mode_strings, mode);
    return res ? res.value() : "UNKNOWN";
}

optional<OperatingMode> operating_mode_of_string(const std::string& s) {
    return rlookup(impl::operating_mode_strings, s.c_str());
}

std::string to_string(MultipurposeIOMode mode) {
    auto res = lookup(impl::multipurpose_io_mode_strings, mode);
    return res ? res.value() : "UNKNOWN";
}

optional<MultipurposeIOMode> multipurpose_io_mode_of_string(
    const std::string& s) {
    return rlookup(impl::multipurpose_io_mode_strings, s.c_str());
}

std::string to_string(Polarity polarity) {
    auto res = lookup(impl::polarity_strings, polarity);
    return res ? res.value() : "UNKNOWN";
}

optional<Polarity> polarity_of_string(const std::string& s) {
    return rlookup(impl::polarity_strings, s.c_str());
}

optional<FullScaleRange> full_scale_range_of_string(const std::string& s) {
    return rlookup(impl::full_scale_range_strings, s.c_str());
}

optional<ReturnOrder> return_order_of_string(const std::string& s) {
    return rlookup(impl::return_order_strings, s.c_str());
}

std::string to_string(NMEABaudRate rate) {
    auto res = lookup(impl::nmea_baud_rate_strings, rate);
    return res ? res.value() : "UNKNOWN";
}

optional<NMEABaudRate> nmea_baud_rate_of_string(const std::string& s) {
    return rlookup(impl::nmea_baud_rate_strings, s.c_str());
}

std::string to_string(AzimuthWindow azimuth_window) {
    std::stringstream ss;
    ss << "[" << azimuth_window.first << ", " << azimuth_window.second << "]";
    return ss.str();
}

#if defined(VOID)
#define OUSTER_REMOVED_VOID
#pragma push_macro("VOID")
#undef VOID
#endif
std::string to_string(ChanFieldType ft) {
    switch (ft) {
        case sensor::ChanFieldType::VOID:
            return "VOID";
        case sensor::ChanFieldType::UINT8:
            return "UINT8";
        case sensor::ChanFieldType::UINT16:
            return "UINT16";
        case sensor::ChanFieldType::UINT32:
            return "UINT32";
        case sensor::ChanFieldType::UINT64:
            return "UINT64";
        case sensor::ChanFieldType::INT8:
            return "INT8";
        case sensor::ChanFieldType::INT16:
            return "INT16";
        case sensor::ChanFieldType::INT32:
            return "INT32";
        case sensor::ChanFieldType::INT64:
            return "INT64";
        case sensor::ChanFieldType::FLOAT32:
            return "FLOAT32";
        case sensor::ChanFieldType::FLOAT64:
            return "FLOAT64";
        default:
            return "UNKNOWN";
    }
}
#if defined(OUSTER_REMOVED_VOID)
#pragma pop_macro("VOID")
#undef OUSTER_REMOVED_VOID
#endif

size_t field_type_size(ChanFieldType ft) {
    switch (ft) {
        case sensor::ChanFieldType::INT8:
        case sensor::ChanFieldType::UINT8:
            return 1;
        case sensor::ChanFieldType::INT16:
        case sensor::ChanFieldType::UINT16:
            return 2;
        case sensor::ChanFieldType::INT32:
        case sensor::ChanFieldType::UINT32:
        case sensor::ChanFieldType::FLOAT32:
            return 4;
        case sensor::ChanFieldType::INT64:
        case sensor::ChanFieldType::UINT64:
        case sensor::ChanFieldType::FLOAT64:
            return 8;
        default:
            return 0;
    }
}

uint64_t field_type_mask(ChanFieldType ft) {
    switch (field_type_size(ft)) {
        case 1:
            return 0xff;
        case 2:
            return 0xffff;
        case 4:
            return 0xffffffff;
        case 8:
            return 0xffffffffffffffff;
        default:
            throw std::runtime_error(
                "field_type_mask error: wrong ChanFieldType");
    }
}

std::string to_string(UDPProfileLidar profile) {
    auto res = lookup(impl::udp_profile_lidar_strings, profile);
    return res ? res.value() : "UNKNOWN";
}

optional<UDPProfileLidar> udp_profile_lidar_of_string(const std::string& s) {
    return rlookup(impl::udp_profile_lidar_strings, s.c_str());
}

std::string to_string(UDPProfileIMU profile) {
    auto res = lookup(impl::udp_profile_imu_strings, profile);
    return res ? res.value() : "UNKNOWN";
}

optional<UDPProfileIMU> udp_profile_imu_of_string(const std::string& s) {
    return rlookup(impl::udp_profile_imu_strings, s.c_str());
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

void check_signal_multiplier(const double signal_multiplier) {
    std::string signal_multiplier_error =
        "Provided signal multiplier is invalid: " +
        std::to_string(signal_multiplier) +
        " cannot be converted to one of [0.25, 0.5, 1, 2, 3]";

    std::set<double> valid_values = {0.25, 0.5, 1, 2, 3};
    if (!valid_values.count(signal_multiplier)) {
        throw std::runtime_error(signal_multiplier_error);
    }
}

jsoncons::json cal_to_json(const calibration_status& cal) {
    jsoncons::json root;

    if (cal.reflectivity_status) {
        root["reflectivity"]["valid"] = cal.reflectivity_status.value();
    }
    if (cal.reflectivity_timestamp) {
        root["reflectivity"]["timestamp"] = cal.reflectivity_timestamp.value();
    }

    return root;
}

std::string to_string(const calibration_status& cal) {
    auto root = cal_to_json(cal);
    std::string out;
    root.dump(out);
    return out;
}

jsoncons::json config_to_json(const sensor_config& config) {
    jsoncons::json root;

    if (config.udp_dest) {
        root["udp_dest"] = config.udp_dest.value();
    }

    if (config.udp_port_lidar) {
        root["udp_port_lidar"] = config.udp_port_lidar.value();
    }

    if (config.udp_port_imu) {
        root["udp_port_imu"] = config.udp_port_imu.value();
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

    if (config.signal_multiplier) {
        check_signal_multiplier(config.signal_multiplier.value());
        if ((config.signal_multiplier == 0.25) ||
            (config.signal_multiplier == 0.5)) {
            root["signal_multiplier"] = config.signal_multiplier.value();
        } else {
            int signal_multiplier_int = int(config.signal_multiplier.value());
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

std::string to_string(const sensor_config& config) {
    jsoncons::json root = config_to_json(config);
    std::string out;
    root.dump(out);
    return out;
}

sensor_config::sensor_config(const std::string& config_json) {
    ValidatorIssues issues;
    if (!ouster::parse_and_validate_config(config_json, *this, issues)) {
        throw std::runtime_error(to_string(issues.critical));
    }
}

sensor_config::sensor_config() {}

product_info product_info::create_product_info(
    std::string product_info_string) {
    std::regex product_regex(
        "^(\\w+)-(\\d+|DOME)?(?:-(\\d+))?(?:-((?!SR)\\w+))?-?(SR)?");
    std::smatch matches;
    if (product_info_string.length() > 0) {
        if (regex_search(product_info_string, matches, product_regex) == true) {
            std::string form_factor = matches.str(1) + matches.str(2);
            bool short_range = (matches.str(5).length() > 0);
            auto beam_config = matches.str(4);
            if (beam_config.length() <= 0) {
                beam_config = "U";
            }
            int beam_count;
            try {
                beam_count = stoi(matches.str(3));
            } catch (const std::exception& e) {
                beam_count = 0;
            }

            return product_info(product_info_string, form_factor, short_range,
                                beam_config, beam_count);
        } else {
            throw std::runtime_error("Product Info \"" + product_info_string +
                                     "\" is not a recognized product info");
        }
    }
    return product_info();
}

product_info::product_info() : product_info("", "", false, "", 0){};

product_info::product_info(std::string product_info_string,
                           std::string form_factor, bool short_range,
                           std::string beam_config, int beam_count)
    : full_product_info(product_info_string),
      form_factor(form_factor),
      short_range(short_range),
      beam_config(beam_config),
      beam_count(beam_count) {}

bool operator==(const product_info& lhs, const product_info& rhs) {
    return lhs.full_product_info == rhs.full_product_info &&
           lhs.form_factor == rhs.form_factor &&
           lhs.short_range == rhs.short_range &&
           lhs.beam_config == rhs.beam_config &&
           lhs.beam_count == rhs.beam_count;
}

bool operator!=(const product_info& lhs, const product_info& rhs) {
    return !(lhs == rhs);
}

std::string to_string(const product_info& info) {
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

int data_format::valid_columns_per_frame() const {
    auto start = column_window.first;
    auto end = column_window.second;

    if (start <= end) {
        return end - start + 1;
    } else {
        return end + (columns_per_frame - start) + 1;
    }
}

int data_format::packets_per_frame() const {
    int start_packet = column_window.first / columns_per_packet;
    int end_packet = column_window.second / columns_per_packet;
    if (column_window.second < column_window.first) {
        // the valid azimuth window wraps through 0
        // Determine the number of packets for a full frame with no window
        int max_packets = columns_per_frame / columns_per_packet +
                          (columns_per_frame % columns_per_packet ? 1 : 0);
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

}  // namespace sensor

namespace util {

version version_from_string(const std::string& v) {
    auto rgx = std::regex(
        R"((([\w\d]*)-([\w\d]*)-)?v?(\d*)\.(\d*)\.(\d*)-?([\d\w.]*)?\+?([\d\w.]*)?)");
    std::smatch matches;
    std::regex_search(v, matches, rgx);

    if (matches.size() < 9) return invalid_version;

    try {
        version v;
        v.major = static_cast<uint16_t>(stoul(matches[4]));
        v.minor = static_cast<uint16_t>(stoul(matches[5]));
        v.patch = static_cast<uint16_t>(stoul(matches[6]));
        v.stage = matches[2];
        v.machine = matches[3];
        v.prerelease = matches[7];
        v.build = matches[8];
        return v;
    } catch (const std::exception&) {
        return invalid_version;
    }
}
}  // namespace util
}  // namespace ouster

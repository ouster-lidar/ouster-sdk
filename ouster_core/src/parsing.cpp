/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <nmea/message/rmc.hpp>
#include <nmea/sentence.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include "ouster/core/array_view.h"
#include "ouster/core/chanfield.h"
#include "ouster/core/field.h"
#include "ouster/core/field_decode_info.h"
#include "ouster/core/impl/table.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"
#include "ouster/core/zone_state.h"

namespace ouster {
namespace sdk {
namespace core {

using ouster::impl::Table;

/**
 * FieldDecodeInfo factory function
 *
 * NOTE: FieldDecodeInfo getters and setters require up to 64 bits of valid
 * memory past the bit_start, caution is advised.
 *
 * @param[in] bit_start starting bit of the value in the buffer
 * @param[in] bit_size size, in bits, of the value in the buffer
 * @param[in] upshift amount of bits to shift the value up, if any; this is
 *            used in packet values that are truncating lower significance bits,
 *            e.g. in low bandwidth profiles
 * @param[in] max_length max buffer sized, used to detect invalid arguments or
 *                       shift as necessary to not read past end of buffer
 *
 * @return FieldDecodeInfo
 */
FieldDecodeInfo field_info(size_t bit_start, size_t bit_size, size_t upshift = 0,
                           size_t max_length = 0, size_t num_elements = 1) {
    FieldDecodeInfo info{};

    size_t needs_bits = bit_size + upshift;
    if (needs_bits > 64) {
        throw std::invalid_argument(
            "failed creating FieldDecodeInfo: value cannot store more than 64 "
            "bits");
    }

    info.offset = bit_start / 8;
    bit_start = bit_start % 8;

    for (size_t i = bit_start; i < bit_start + bit_size; ++i) {
        info.mask |= uint64_t{1} << i;
    }

    info.shift = static_cast<int>(bit_start);
    info.shift -= static_cast<int>(upshift);
    info.num_elements = static_cast<int>(num_elements);

    size_t size_bytes = (needs_bits / 8) + (((needs_bits % 8) != 0u) ? 1 : 0);
    size_bytes /= num_elements;

    switch (size_bytes) {
        case 1:
            info.ty_tag = ChanFieldType::UINT8;
            break;
        case 2:
            info.ty_tag = ChanFieldType::UINT16;
            break;
        case 3:
        case 4:
            info.ty_tag = ChanFieldType::UINT32;
            break;
        case 5:
        case 6:
        case 7:
        case 8:
            info.ty_tag = ChanFieldType::UINT64;
            break;
        default:
            info.ty_tag = ChanFieldType::VOID;
    }

    // Make sure we don't read past the end of the buffer if constrained.
    if (max_length > 0) {
        // Throw for anything truly impossible
        if (info.offset + size_bytes > max_length) {
            throw std::invalid_argument(
                "failed creating FieldDecodeInfo: asked to read past end of "
                "packet");
        }

        // Otherwise shift if the bytes requested to be read are against the end
        int needed_shift_bytes = static_cast<int>(info.offset) + 8 - static_cast<int>(max_length);
        if (needed_shift_bytes > 0) {
            info.offset -= needed_shift_bytes;
            info.mask <<= needed_shift_bytes * 8;
            info.shift += needed_shift_bytes * 8;
        }
    }

    return info;
}

namespace {

int count_set_bits(uint64_t value) {
    int count = 0;
    while (value != 0u) {
        count += static_cast<int>(value & 1);
        value >>= 1;
    }
    return count;
}

}  // namespace

namespace impl {

OUSTER_API_FUNCTION
uint64_t get_value_mask(const FieldDecodeInfo& field_info) {
    uint64_t type_mask = field_type_mask(field_info.ty_tag);

    uint64_t mask = field_info.mask;
    if (mask == 0) {
        mask = type_mask;
    }
    if (field_info.shift > 0) {
        mask >>= field_info.shift;
    }
    if (field_info.shift < 0) {
        mask <<= std::abs(field_info.shift);
    }
    // final type *may* cut the resultant mask still
    mask &= type_mask;

    return mask;
}

OUSTER_API_FUNCTION
int get_bitness(const FieldDecodeInfo& field_info) {
    return count_set_bits(get_value_mask(field_info));
}

struct ProfileEntry {
    const std::pair<std::string, FieldDecodeInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

static const Table<std::string, FieldDecodeInfo, 8> LEGACY_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 20)},
    {ChanField::FLAGS, field_info(28, 4)},
    {ChanField::REFLECTIVITY, field_info(32, 8)},
    {ChanField::SIGNAL, field_info(48, 16)},
    {ChanField::NEAR_IR, field_info(64, 16)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 5> LB_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::NEAR_IR, field_info(24, 8, 4)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 5> LB_WINDOW_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::WINDOW, field_info(24, 8)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 13> RGB_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(24, 8)},
    {ChanField::SIGNAL, field_info(32, 16)},
    {ChanField::NEAR_IR, field_info(48, 16)},
    {ChanField::R, field_info(64, 16)},
    {ChanField::G, field_info(64 + 16, 16)},
    {ChanField::B, field_info(64 + 16 * 2, 16)},
    {ChanField::RGB, field_info(64, size_t{16} * 3, 0, 0, 3)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
    {ChanField::RAW32_WORD4, field_info(96, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 18> DUAL_RGB_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(24, 8)},
    {ChanField::RANGE2, field_info(32, 19)},
    {ChanField::FLAGS2, field_info(51, 5)},
    {ChanField::REFLECTIVITY2, field_info(56, 8)},
    {ChanField::SIGNAL, field_info(64, 16)},
    {ChanField::SIGNAL2, field_info(80, 16)},
    {ChanField::NEAR_IR, field_info(96, 16)},
    {ChanField::R, field_info(112, 16)},
    {ChanField::G, field_info(112 + 16, 16)},
    {ChanField::B, field_info(112 + 16 * 2, 16)},
    {ChanField::RGB, field_info(112, size_t{16} * 3, 0, 0, 3)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
    {ChanField::RAW32_WORD4, field_info(96, 32)},
    {ChanField::RAW32_WORD5, field_info(128, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 14> DUAL_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(24, 8)},
    {ChanField::RANGE2, field_info(32, 19)},
    {ChanField::FLAGS2, field_info(51, 5)},
    {ChanField::REFLECTIVITY2, field_info(56, 8)},
    {ChanField::SIGNAL, field_info(64, 16)},
    {ChanField::SIGNAL2, field_info(80, 16)},
    {ChanField::NEAR_IR, field_info(96, 16)},
    {ChanField::WINDOW, field_info(120, 8)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
    {ChanField::RAW32_WORD4, field_info(96, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 9> SINGLE_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(32, 8)},
    {ChanField::SIGNAL, field_info(48, 16)},
    {ChanField::NEAR_IR, field_info(64, 16)},
    {ChanField::WINDOW, field_info(88, 8)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 14> FIVE_WORD_PIXEL_INFO{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(24, 8)},
    {ChanField::RANGE2, field_info(32, 19)},
    {ChanField::FLAGS2, field_info(51, 5)},
    {ChanField::REFLECTIVITY2, field_info(56, 8)},
    {ChanField::SIGNAL, field_info(64, 16)},
    {ChanField::SIGNAL2, field_info(80, 16)},
    {ChanField::NEAR_IR, field_info(96, 16)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
    {ChanField::RAW32_WORD4, field_info(96, 32)},
    {ChanField::RAW32_WORD5, field_info(128, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 8> ZM_LB_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::NEAR_IR, field_info(24, 8, 4)},
    {ChanField::ZONE_MASK, field_info(32, 16)},
    {ChanField::WINDOW, field_info(48, 8)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 10> ZM_SINGLE_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(32, 8)},
    {ChanField::WINDOW, field_info(40, 8)},
    {ChanField::SIGNAL, field_info(48, 16)},
    {ChanField::NEAR_IR, field_info(64, 16)},
    {ChanField::ZONE_MASK, field_info(80, 16)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 10> DUAL_LB_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::NEAR_IR, field_info(24, 8, 4)},
    {ChanField::RANGE2, field_info(32, 15, 3)},
    {ChanField::FLAGS2, field_info(47, 1)},
    {ChanField::REFLECTIVITY2, field_info(48, 8)},
    {ChanField::WINDOW, field_info(56, 8)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 14> DUAL_ZONE_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(24, 8)},
    {ChanField::RANGE2, field_info(32, 19)},
    {ChanField::FLAGS2, field_info(51, 5)},
    {ChanField::REFLECTIVITY2, field_info(56, 8)},
    {ChanField::SIGNAL, field_info(64, 16)},
    {ChanField::SIGNAL2, field_info(80, 16)},
    {ChanField::ZONE_MASK, field_info(96, 16)},
    {ChanField::WINDOW, field_info(120, 8)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
    {ChanField::RAW32_WORD4, field_info(96, 32)},
}};

static const Table<std::string, FieldDecodeInfo, 0> OFF_PROFILE_INFO{};

// TWS 20260113: trying not to touch the profiles table, which is used in tests
Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES> OUSTER_API_FUNCTION profiles{{
    {UDPProfileLidar::LEGACY, {LEGACY_FIELD_INFO.data(), LEGACY_FIELD_INFO.size(), 12}},
    {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
     {DUAL_FIELD_INFO.data(), DUAL_FIELD_INFO.size(), 16}},
    {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16,
     {SINGLE_FIELD_INFO.data(), SINGLE_FIELD_INFO.size(), 12}},
    {UDPProfileLidar::RNG15_RFL8_NIR8, {LB_FIELD_INFO.data(), LB_FIELD_INFO.size(), 4}},
    {UDPProfileLidar::FIVE_WORD_PIXEL,
     {FIVE_WORD_PIXEL_INFO.data(), FIVE_WORD_PIXEL_INFO.size(), 20}},
    {UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL,
     {DUAL_LB_FIELD_INFO.data(), DUAL_LB_FIELD_INFO.size(), 8}},
    {UDPProfileLidar::RNG15_RFL8_NIR8_DUAL,
     {DUAL_LB_FIELD_INFO.data(), DUAL_LB_FIELD_INFO.size(), 8}},
    {UDPProfileLidar::OFF, {OFF_PROFILE_INFO.data(), OFF_PROFILE_INFO.size(), 0}},
    {UDPProfileLidar::RNG15_RFL8_NIR8_ZONE16,
     {ZM_LB_FIELD_INFO.data(), ZM_LB_FIELD_INFO.size(), 8}},
    {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_ZONE16,
     {ZM_SINGLE_FIELD_INFO.data(), ZM_SINGLE_FIELD_INFO.size(), 12}},
    {UDPProfileLidar::RNG15_RFL8_WIN8,
     {LB_WINDOW_FIELD_INFO.data(), LB_WINDOW_FIELD_INFO.size(), 4}},
    {UDPProfileLidar::RNG19_RFL8_SIG16_ZONE16_DUAL,
     {DUAL_ZONE_FIELD_INFO.data(), DUAL_ZONE_FIELD_INFO.size(), 16}},
    {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16,
     {RGB_FIELD_INFO.data(), RGB_FIELD_INFO.size(), 16}},
    {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL,
     {DUAL_RGB_FIELD_INFO.data(), DUAL_RGB_FIELD_INFO.size(), 20}},
}};

OUSTER_API_FUNCTION Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES> get_profiles() {
    return profiles;
}

}  // namespace impl

namespace {
const impl::ProfileEntry& lookup_profile_entry(UDPProfileLidar profile) {
    auto end = impl::profiles.end();
    auto it = std::find_if(impl::profiles.begin(), end, [profile](const auto& profile_pair) {
        return profile_pair.first == profile;
    });

    if (it == end || it->first == UDPProfileLidar::UNKNOWN) {
        throw std::invalid_argument("Unknown lidar udp profile");
    }

    return it->second;
}
}  // namespace

struct PacketFormat::Impl {
    size_t packet_header_size{};
    size_t col_header_size{};
    size_t channel_data_size{};
    size_t col_footer_size{};
    size_t packet_footer_size{};
    size_t imu_measurement_offset{};
    size_t imu_measurement_size{};
    size_t zone_measurement_offset{};
    size_t zone_measurement_size{};

    uint32_t max_frame_id{};

    size_t col_size{};
    size_t lidar_packet_size{};
    size_t imu_packet_size{};
    size_t zone_packet_size{};

    UDPProfileIMU imu_profile{};

    std::map<std::string, FieldDecodeInfo> fields;

    // header infos
    FieldDecodeInfo packet_type_info{};
    FieldDecodeInfo frame_id_info{};
    FieldDecodeInfo init_id_info{};
    FieldDecodeInfo prod_sn_info{};
    FieldDecodeInfo alert_flags_info{};
    FieldDecodeInfo countdown_thermal_shutdown_info{};
    FieldDecodeInfo countdown_shot_limiting_info{};
    FieldDecodeInfo thermal_shutdown_info{};
    FieldDecodeInfo shot_limiting_info{};

    // column infos
    FieldDecodeInfo col_status_info{};
    FieldDecodeInfo col_timestamp_info{};
    FieldDecodeInfo col_measurement_id_info{};

    // imu infos
    FieldDecodeInfo imu_sys_ts_info{};
    FieldDecodeInfo imu_accel_ts_info{};
    FieldDecodeInfo imu_gyro_ts_info{};
    FieldDecodeInfo imu_nmea_ts_info{};
    FieldDecodeInfo imu_la_x_info{};
    FieldDecodeInfo imu_la_y_info{};
    FieldDecodeInfo imu_la_z_info{};
    FieldDecodeInfo imu_av_x_info{};
    FieldDecodeInfo imu_av_y_info{};
    FieldDecodeInfo imu_av_z_info{};

    FieldDecodeInfo zone_timestamp_info{};
    FieldDecodeInfo zone_live_info{};
    FieldDecodeInfo zone_id_info{};
    FieldDecodeInfo zone_error_flags_info{};
    FieldDecodeInfo zone_trigger_type_info{};
    FieldDecodeInfo zone_trigger_status_info{};
    FieldDecodeInfo zone_triggered_frames_info{};
    FieldDecodeInfo zone_count_info{};
    FieldDecodeInfo zone_occlusion_count_info{};
    FieldDecodeInfo zone_invalid_count_info{};
    FieldDecodeInfo zone_max_count_info{};
    FieldDecodeInfo zone_min_range_info{};
    FieldDecodeInfo zone_max_range_info{};
    FieldDecodeInfo zone_mean_range_info{};

    Impl() = default;

    explicit Impl(const DataFormat& format) : Impl() {
        bool legacy = (format.udp_profile_lidar == UDPProfileLidar::LEGACY);
        bool fusa = (format.header_type == HeaderType::FUSA) && !legacy;

        const auto& entry = lookup_profile_entry(format.udp_profile_lidar);

        packet_header_size = legacy ? 0 : 32;
        col_header_size = legacy ? 16 : 12;
        channel_data_size = entry.chan_data_size;
        col_footer_size = legacy ? 4 : 0;
        packet_footer_size = legacy ? 0 : 32;

        col_size = col_header_size +  // NOLINT(cppcoreguidelines-prefer-member-initializer)
                   (format.pixels_per_column * channel_data_size) + col_footer_size;
        lidar_packet_size =
            packet_header_size +  // NOLINT(cppcoreguidelines-prefer-member-initializer)
            (format.columns_per_packet * col_size) + packet_footer_size;

        if (lidar_packet_size > 65535) {
            throw std::invalid_argument("lidar_packet_size cannot exceed 65535");
        }

        fields = {entry.fields, entry.fields + entry.n_fields};
        max_frame_id = format.max_frame_id();

        // LIDAR packet and common LIDAR/IMU headers
        if (legacy) {
            // NOTE: mixing LEGACY lidar packets with non-LEGACY IMU or zone
            // monitoring packets is not a valid configuration and produces
            // nonsensical packet offsets/sizes because the packet headers and footers differ.
            if (format.udp_profile_imu == UDPProfileIMU::ACCEL32_GYRO32_NMEA ||
                format.zone_monitoring_enabled) {
                throw std::runtime_error(
                    "Invalid sensor configuration. Mixing LEGACY lidar packets "
                    "and non-LEGACY IMU and ZONE packets is not possible or "
                    "supported by the SDK. Your udp_profile_lidar may be "
                    "incorrect for this data.");
            }

            // below are absent on legacy, results in mask==0
            packet_type_info = field_info(0, 0);
            init_id_info = field_info(0, 0);
            prod_sn_info = field_info(0, 0);
            alert_flags_info = field_info(0, 0);
            countdown_thermal_shutdown_info = field_info(0, 0);
            countdown_shot_limiting_info = field_info(0, 0);
            thermal_shutdown_info = field_info(0, 0);
            shot_limiting_info = field_info(0, 0);

            // frame_id is baked into the first column header
            frame_id_info = field_info(80, 16);

            // LEGACY col_status sits at the end of the column as opposed to
            // being in column header, and FieldDecodeInfo::get takes 8-byte
            // so make sure it knows not to read past the end by providing a
            // fake size limit at the end of col status info.
            auto start_bit = 8 * (col_size - col_footer_size);
            col_status_info = field_info(start_bit, 32, 0, (start_bit + 32) / 8);
        } else if (fusa) {
            packet_type_info = field_info(0, 8);
            frame_id_info = field_info(32, 32);
            init_id_info = field_info(8, 24);
            alert_flags_info = field_info(64, 8);  // Supposedly supported in both 2.5.X and 3.1.X
            prod_sn_info = field_info(88, 40);
            countdown_thermal_shutdown_info = field_info(128, 8);
            countdown_shot_limiting_info = field_info(136, 8);
            thermal_shutdown_info = field_info(144, 4);
            shot_limiting_info = field_info(152, 4);

            col_status_info = field_info(80, 16);
        } else {
            packet_type_info = field_info(0, 16);
            frame_id_info = field_info(16, 16);
            init_id_info = field_info(32, 24);
            prod_sn_info = field_info(56, 40);
            alert_flags_info = field_info(96, 8);  // Supposedly supported in both 2.5.X and 3.1.X
            countdown_thermal_shutdown_info = field_info(128, 8);
            countdown_shot_limiting_info = field_info(136, 8);
            thermal_shutdown_info = field_info(144, 4);
            shot_limiting_info = field_info(152, 4);

            col_status_info = field_info(80, 16);
        }

        col_timestamp_info = field_info(0, 64);
        col_measurement_id_info = field_info(64, 16);

        // IMU packet
        imu_profile = format.udp_profile_imu;
        if (imu_profile == UDPProfileIMU::LEGACY) {
            imu_packet_size = 48;
            imu_measurement_offset = 0;
            imu_measurement_size = 0;
            imu_sys_ts_info = field_info(0, 64, 0, imu_packet_size);
            imu_accel_ts_info = field_info(64, 64, 0, imu_packet_size);
            imu_gyro_ts_info = field_info(128, 64, 0, imu_packet_size);
            imu_nmea_ts_info = field_info(0, 0, 0, imu_packet_size);
            imu_la_x_info = field_info(192, 32, 0, imu_packet_size);
            imu_la_y_info = field_info(224, 32, 0, imu_packet_size);
            imu_la_z_info = field_info(256, 32, 0, imu_packet_size);
            imu_av_x_info = field_info(288, 32, 0, imu_packet_size);
            imu_av_y_info = field_info(320, 32, 0, imu_packet_size);
            imu_av_z_info = field_info(352, 32, 0, imu_packet_size);
        } else if (imu_profile == UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
            size_t nmea_block_size = 100;
            imu_measurement_size = 36;
            imu_packet_size = packet_header_size + nmea_block_size +
                              (format.imu_measurements_per_packet * imu_measurement_size) +
                              packet_footer_size;
            imu_measurement_offset = packet_header_size + nmea_block_size;
            imu_nmea_ts_info = field_info(packet_header_size * 8, 64);
            imu_la_x_info = field_info(96, 32);
            imu_la_y_info = field_info(128, 32);
            imu_la_z_info = field_info(160, 32);
            imu_av_x_info = field_info(192, 32);
            imu_av_y_info = field_info(224, 32);
            imu_av_z_info = field_info(256, 32);

            // not available
            imu_sys_ts_info = field_info(0, 0);
            imu_accel_ts_info = field_info(0, 0);
            imu_gyro_ts_info = field_info(0, 0);
        }

        zone_measurement_offset = packet_header_size + 8 /*timestamp*/ + 32 /*hash*/;
        zone_measurement_size = 36;
        // zone monitoring
        zone_timestamp_info = field_info(256, 64);
        zone_live_info = field_info(0, 1);
        zone_id_info = field_info(8, 8);
        zone_error_flags_info = field_info(16, 8);
        zone_trigger_type_info = field_info(26, 2);
        zone_trigger_status_info = field_info(31, 1);
        zone_triggered_frames_info = field_info(32, 32);
        zone_count_info = field_info(64, 32);
        zone_occlusion_count_info = field_info(96, 32);
        zone_invalid_count_info = field_info(128, 32);
        zone_max_count_info = field_info(160, 32);
        zone_min_range_info = field_info(192, 19);
        zone_max_range_info = field_info(224, 19);
        zone_mean_range_info = field_info(256, 19);

        zone_packet_size = packet_header_size + 8 /*timestamp*/ + 32 /*hash*/ +
                           (zone_measurement_size * 16) + packet_footer_size;
    }
};

PacketFormat::PacketFormat(const DataFormat& format)
    : impl_{std::make_shared<Impl>(format)},
      udp_profile_lidar{format.udp_profile_lidar},
      udp_profile_imu{format.udp_profile_imu},
      header_type{format.header_type},
      lidar_packet_size{impl_->lidar_packet_size},
      imu_packet_size{impl_->imu_packet_size},
      zone_packet_size{impl_->zone_packet_size},
      columns_per_packet(format.columns_per_packet),
      pixels_per_column(format.pixels_per_column),
      imu_measurements_per_packet(format.imu_measurements_per_packet),
      imu_packets_per_frame(format.imu_packets_per_frame),
      packet_header_size{impl_->packet_header_size},
      col_header_size{impl_->col_header_size},
      col_footer_size{impl_->col_footer_size},
      col_size{impl_->col_size},
      packet_footer_size{impl_->packet_footer_size},
      max_frame_id{impl_->max_frame_id},
      zone_monitoring_enabled{format.zone_monitoring_enabled} {
    for (const auto& kv : impl_->fields) {
        std::pair<ouster::sdk::core::ChanFieldType, int> elm = {kv.second.ty_tag,
                                                                kv.second.num_elements};
        field_types_.emplace_back(kv.first, elm);
    }
}

PacketFormat::PacketFormat(const SensorInfo& info) : PacketFormat(info.format) {}

template <typename T, int BlockDim>
void PacketFormat::block_field(T* data, int cols, const std::string& field_name,
                               const uint8_t* lidar_buf) const {
    FieldDecodeInfo field_info = impl_->fields.at(field_name);

    if (sizeof(T) < field_type_size(field_info.ty_tag) * field_info.num_elements) {
        throw std::invalid_argument("Dest type too small for specified field");
    }

    size_t channel_data_size = impl_->channel_data_size;

    std::array<const uint8_t*, BlockDim> col_buf{};

    for (uint32_t icol = 0; icol < columns_per_packet; icol += BlockDim) {
        for (int i = 0; i < BlockDim; ++i) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            col_buf[i] = nth_col(icol + i, lidar_buf);
        }

        uint16_t m_id = col_measurement_id(col_buf[0]);

        for (uint32_t px = 0; px < pixels_per_column; ++px) {
            std::ptrdiff_t f_offset = (cols * px) + m_id;
            for (int x = 0; x < BlockDim; ++x) {
                auto px_src = col_buf[x] + col_header_size + (px * channel_data_size);
                *(data + f_offset + x) = field_info.get<T>(px_src);
            }
        }
    }
}

template <typename T>
void PacketFormat::col_field(const uint8_t* col_buf, const std::string& field_name, T* dst,
                             int dst_stride) const {
    FieldDecodeInfo field_info = impl_->fields.at(field_name);

    if (sizeof(T) < field_type_size(field_info.ty_tag) * field_info.num_elements) {
        throw std::invalid_argument("Dest type too small for specified field");
    }

    size_t channel_data_size = impl_->channel_data_size;

    for (uint32_t px = 0; px < pixels_per_column; px++) {
        auto px_src = col_buf + col_header_size + (px * channel_data_size);
        T* px_dst = dst + (px * dst_stride);
        *px_dst = field_info.get<T>(px_src);
    }
}

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define TYPE_LIST(size) \
    X(size, uint8_t)    \
    X(size, uint16_t)   \
    X(size, uint32_t)   \
    X(size, uint64_t)   \
    X(size, int8_t)     \
    X(size, int16_t)    \
    X(size, int32_t)    \
    X(size, int64_t)    \
    X(size, float)      \
    X(size, double)     \
    X(size, float16_t)  \
    X(size, ouster::sdk::core::impl::float3x16_t)

// explicitly instantiate for each field type / block dim
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define X(size, type)                                                          \
    template OUSTER_API_FUNCTION void PacketFormat::block_field<type, (size)>( \
        std::add_pointer_t<type>, int, const std::string&, const uint8_t*)     \
        const;  // NOLINT(bugprone-macro-parentheses)

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define BLOCK_FIELD(size) TYPE_LIST(size)

BLOCK_FIELD(4)
BLOCK_FIELD(8)
BLOCK_FIELD(16)

#undef X

// explicitly instantiate for each field type
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define X(size, type)                                                                             \
    template OUSTER_API_FUNCTION void PacketFormat::col_field(const uint8_t*, const std::string&, \
                                                              std::add_pointer_t<type>, int)      \
        const;  // NOLINT(bugprone-macro-parentheses)

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define COL_ELEMENTS() TYPE_LIST(0)

COL_ELEMENTS()
#undef X

ChanFieldType PacketFormat::field_type(const std::string& field_name) const {
    return (impl_->fields.count(field_name) != 0u) ? impl_->fields.at(field_name).ty_tag
                                                   : ChanFieldType::VOID;
}

PacketFormat::FieldIter PacketFormat::begin() const {
    return field_types_.cbegin();
}

PacketFormat::FieldIter PacketFormat::end() const {
    return field_types_.cend();
}

/* Packet headers */

uint16_t PacketFormat::packet_type(const uint8_t* packet_buf) const {
    return impl_->packet_type_info.get<uint16_t>(packet_buf);
}

uint32_t PacketFormat::frame_id(const uint8_t* packet_buf) const {
    return impl_->frame_id_info.get<uint32_t>(packet_buf);
}

uint32_t PacketFormat::init_id(const uint8_t* packet_buf) const {
    return impl_->init_id_info.get<uint32_t>(packet_buf);
}

uint64_t PacketFormat::prod_sn(const uint8_t* packet_buf) const {
    return impl_->prod_sn_info.get<uint64_t>(packet_buf);
}

uint8_t PacketFormat::alert_flags(const uint8_t* lidar_buf) const {
    return impl_->alert_flags_info.get<uint8_t>(lidar_buf);
}

uint16_t PacketFormat::countdown_thermal_shutdown(const uint8_t* lidar_buf) const {
    return impl_->countdown_thermal_shutdown_info.get<uint16_t>(lidar_buf);
}

uint16_t PacketFormat::countdown_shot_limiting(const uint8_t* lidar_buf) const {
    return impl_->countdown_shot_limiting_info.get<uint16_t>(lidar_buf);
}

ThermalShutdownStatus PacketFormat::thermal_shutdown(const uint8_t* lidar_buf) const {
    return static_cast<ThermalShutdownStatus>(impl_->thermal_shutdown_info.get<uint8_t>(lidar_buf));
}

ShotLimitingStatus PacketFormat::shot_limiting(const uint8_t* lidar_buf) const {
    return static_cast<ShotLimitingStatus>(impl_->shot_limiting_info.get<uint8_t>(lidar_buf));
}

uint8_t* PacketFormat::footer(uint8_t* lidar_buf) const {
    if (impl_->packet_footer_size == 0) {
        return nullptr;
    }
    return lidar_buf + impl_->packet_header_size + (columns_per_packet * impl_->col_size);
}

const uint8_t* PacketFormat::footer(const uint8_t* lidar_buf) const {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    return footer(const_cast<uint8_t*>(lidar_buf));
}

/* Measurement block access */

uint8_t* PacketFormat::nth_col(size_t col_idx, uint8_t* lidar_buf) const {
    return lidar_buf + impl_->packet_header_size + (col_idx * impl_->col_size);
}

const uint8_t* PacketFormat::nth_col(size_t col_idx, const uint8_t* lidar_buf) const {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    return nth_col(col_idx, const_cast<uint8_t*>(lidar_buf));
}

uint32_t PacketFormat::col_status(const uint8_t* col_buf) const {
    return impl_->col_status_info.get<uint32_t>(col_buf);
}

uint64_t PacketFormat::col_timestamp(const uint8_t* col_buf) const {
    return impl_->col_timestamp_info.get<uint64_t>(col_buf);
}

uint16_t PacketFormat::col_measurement_id(const uint8_t* col_buf) const {
    return impl_->col_measurement_id_info.get<uint16_t>(col_buf);
}

uint32_t PacketFormat::col_encoder(const uint8_t* col_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::LEGACY) {
        uint32_t res = 0;
        std::memcpy(&res, col_buf + 12, sizeof(uint32_t));
        return res;
    } else {
        return 0;
    }
}

uint16_t PacketFormat::col_frame_id(const uint8_t* col_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::LEGACY) {
        uint16_t res = 0;
        std::memcpy(&res, col_buf + 10, sizeof(uint16_t));
        return res;
    } else {
        return 0;
    }
}

/* Channel data fields */

uint8_t* PacketFormat::nth_px(size_t px_idx, uint8_t* col_buf) const {
    return col_buf + impl_->col_header_size + (px_idx * impl_->channel_data_size);
}

const uint8_t* PacketFormat::nth_px(size_t px_idx, const uint8_t* col_buf) const {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    return nth_px(px_idx, const_cast<uint8_t*>(col_buf));
}

/* IMU packet parsing */

uint8_t* PacketFormat::imu_nth_measurement(size_t meas_idx, uint8_t* imu_buf) const {
    // in LEGACY both offset and size are zero so we always get back imu_buf
    return imu_buf + impl_->imu_measurement_offset + (meas_idx * impl_->imu_measurement_size);
}

const uint8_t* PacketFormat::imu_nth_measurement(size_t meas_idx, const uint8_t* imu_buf) const {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    return imu_nth_measurement(meas_idx, const_cast<uint8_t*>(imu_buf));
}

std::string PacketFormat::imu_nmea_sentence(const uint8_t* imu_buf) const {
    const char* start = reinterpret_cast<const char*>(imu_buf) + impl_->packet_header_size + 8;
    return std::string{start, NMEA_SENTENCE_LENGTH};
}

uint64_t PacketFormat::imu_nmea_ts(const uint8_t* imu_buf) const {
    return impl_->imu_nmea_ts_info.get<uint64_t>(imu_buf);
}

uint64_t PacketFormat::imu_sys_ts(const uint8_t* imu_buf) const {
    return impl_->imu_sys_ts_info.get<uint64_t>(imu_buf);
}

uint64_t PacketFormat::imu_accel_ts(const uint8_t* imu_buf) const {
    return impl_->imu_accel_ts_info.get<uint64_t>(imu_buf);
}

uint64_t PacketFormat::imu_gyro_ts(const uint8_t* imu_buf) const {
    return impl_->imu_gyro_ts_info.get<uint64_t>(imu_buf);
}

float PacketFormat::imu_la_x(const uint8_t* imu_buf) const {
    return impl_->imu_la_x_info.get<float>(imu_buf);
}

float PacketFormat::imu_la_y(const uint8_t* imu_buf) const {
    return impl_->imu_la_y_info.get<float>(imu_buf);
}

float PacketFormat::imu_la_z(const uint8_t* imu_buf) const {
    return impl_->imu_la_z_info.get<float>(imu_buf);
}

float PacketFormat::imu_av_x(const uint8_t* imu_buf) const {
    return impl_->imu_av_x_info.get<float>(imu_buf);
}

float PacketFormat::imu_av_y(const uint8_t* imu_buf) const {
    return impl_->imu_av_y_info.get<float>(imu_buf);
}

float PacketFormat::imu_av_z(const uint8_t* imu_buf) const {
    return impl_->imu_av_z_info.get<float>(imu_buf);
}

void PacketFormat::parse_accel(size_t col_offset, const uint8_t* imu_buf, Field& accel) {
    // copy over FieldDecodeInfos into local stack for speed
    FieldDecodeInfo la_x = impl_->imu_la_x_info;
    FieldDecodeInfo la_y = impl_->imu_la_y_info;
    FieldDecodeInfo la_z = impl_->imu_la_z_info;

    ArrayView2<float> accel_view = accel;

    for (size_t i = 0; i < imu_measurements_per_packet; ++i) {
        const uint8_t* col_buf = imu_nth_measurement(i, imu_buf);
        if ((col_status(col_buf) & 0x1) == 0u) {
            continue;
        }

        ArrayView1<float> linear_accel = accel_view.subview(col_offset + i);
        linear_accel(0) = la_x.get<float>(col_buf);
        linear_accel(1) = la_y.get<float>(col_buf);
        linear_accel(2) = la_z.get<float>(col_buf);
    }
}

void PacketFormat::parse_gyro(size_t col_offset, const uint8_t* imu_buf, Field& gyro) {
    // copy over FieldDecodeInfos into local stack for speed
    FieldDecodeInfo av_x = impl_->imu_av_x_info;
    FieldDecodeInfo av_y = impl_->imu_av_y_info;
    FieldDecodeInfo av_z = impl_->imu_av_z_info;

    ArrayView2<float> gyro_view = gyro;

    for (size_t i = 0; i < imu_measurements_per_packet; ++i) {
        const uint8_t* col_buf = imu_nth_measurement(i, imu_buf);
        if ((col_status(col_buf) & 0x1) == 0u) {
            continue;
        }

        ArrayView1<float> angular_velocity = gyro_view.subview(col_offset + i);
        angular_velocity(0) = av_x.get<float>(col_buf);
        angular_velocity(1) = av_y.get<float>(col_buf);
        angular_velocity(2) = av_z.get<float>(col_buf);
    }
}

bool parse_lat_long(const std::string& nmea_sentence, double& latitude, double& longitude) {
    if (!nmea::sentence::validate(nmea_sentence)) {
        return false;
    }

    nmea::sentence sentence{nmea_sentence};
    if (sentence.type() != "RMC") {
        return false;
    }

    nmea::rmc rmc{sentence};
    if (!rmc.latitude.exists() || !rmc.longitude.exists()) {
        return false;
    }

    latitude = rmc.latitude.get();
    longitude = rmc.longitude.get();

    return true;
}

int PacketFormat::block_parsable() const {
    std::array<int, 3> dims = {16, 8, 4};
    for (const auto& dim : dims) {
        if ((pixels_per_column % dim == 0) && (columns_per_packet % dim == 0)) {
            return dim;
        }
    }
    return 0;
}

/* necessary for what we're about to do in get_format */
// NOLINTNEXTLINE(misc-use-anonymous-namespace, Wunused-function)
static bool operator<(const DataFormat& lhs, const DataFormat& rhs) {
    return std::tie(lhs.pixels_per_column, lhs.columns_per_packet, lhs.columns_per_frame,
                    lhs.imu_measurements_per_packet, lhs.pixel_shift_by_row, lhs.column_window,
                    lhs.udp_profile_lidar, lhs.udp_profile_imu, lhs.header_type) <
           std::tie(rhs.pixels_per_column, rhs.columns_per_packet, rhs.columns_per_frame,
                    rhs.imu_measurements_per_packet, rhs.pixel_shift_by_row, rhs.column_window,
                    rhs.udp_profile_lidar, rhs.udp_profile_imu, rhs.header_type);
}

// TODO[tws] consider removal. This is only used when constructing a
// FrameBatcher, which happens rarely.
const PacketFormat& get_format(const DataFormat& format) {
    static std::map<DataFormat, std::unique_ptr<PacketFormat>> cache{};
    static std::mutex cache_mx{};

    std::lock_guard<std::mutex> lk{cache_mx};
    if (cache.count(format) == 0u) {
        cache[format] = std::make_unique<PacketFormat>(format);
    }

    return *cache.at(format);
}

const PacketFormat& get_format(const SensorInfo& info) {
    return get_format(info.format);
}

uint64_t PacketFormat::field_value_mask(const std::string& field_name) const {
    const auto& field_info = impl_->fields.at(field_name);
    return impl::get_value_mask(field_info);
}

int PacketFormat::field_bitness(const std::string& field_name) const {
    const auto& field_info = impl_->fields.at(field_name);
    return impl::get_bitness(field_info);
}

void PacketFormat::set_col_status(uint8_t* col_buf, uint32_t status) const {
    impl_->col_status_info.set(col_buf, status);
}

void PacketFormat::set_col_timestamp(uint8_t* col_buf, uint64_t timestamp) const {
    impl_->col_timestamp_info.set(col_buf, timestamp);
}

void PacketFormat::set_col_measurement_id(uint8_t* col_buf, uint16_t m_id) const {
    impl_->col_measurement_id_info.set(col_buf, m_id);
}

void PacketFormat::set_frame_id(uint8_t* lidar_buf, uint32_t frame_id) const {
    impl_->frame_id_info.set(lidar_buf, frame_id);
}

void PacketFormat::set_init_id(uint8_t* lidar_buf, uint32_t init_id) const {
    impl_->init_id_info.set(lidar_buf, init_id);
}

void PacketFormat::set_packet_type(uint8_t* packet_buf, uint16_t packet_type) const {
    impl_->packet_type_info.set(packet_buf, packet_type);
}

void PacketFormat::set_prod_sn(uint8_t* lidar_buf, uint64_t serial_number) const {
    impl_->prod_sn_info.set(lidar_buf, serial_number);
}

void PacketFormat::set_alert_flags(uint8_t* lidar_buf, uint8_t alert_flags) const {
    impl_->alert_flags_info.set(lidar_buf, alert_flags);
}

void PacketFormat::set_shutdown(uint8_t* lidar_buf, uint8_t status) const {
    impl_->thermal_shutdown_info.set(lidar_buf, status);
}

void PacketFormat::set_shot_limiting(uint8_t* lidar_buf, uint8_t status) const {
    impl_->shot_limiting_info.set(lidar_buf, status);
}

void PacketFormat::set_shutdown_countdown(uint8_t* lidar_buf, uint8_t shutdown_countdown) const {
    impl_->countdown_thermal_shutdown_info.set(lidar_buf, shutdown_countdown);
}

void PacketFormat::set_shot_limiting_countdown(uint8_t* lidar_buf,
                                               uint8_t shot_limiting_countdown) const {
    impl_->countdown_shot_limiting_info.set(lidar_buf, shot_limiting_countdown);
}

template <typename T>
void PacketFormat::set_block(const T* data, int cols, const std::string& field_name,
                             uint8_t* lidar_buf) const {
    constexpr int max_cols = 32;
    if (columns_per_packet > max_cols) {
        throw std::runtime_error("Recompile set_block_impl with larger N");
    }

    FieldDecodeInfo f_info = impl_->fields.at(field_name);

    size_t channel_data_size = impl_->channel_data_size;

    std::array<uint8_t*, max_cols> col_buf{};
    std::array<bool, max_cols> valid{};
    for (uint32_t i = 0; i < columns_per_packet; ++i) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        col_buf[i] = nth_col(i, lidar_buf);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        valid[i] = col_status(col_buf[i]) & 0x01;
    }
    uint16_t m_id = col_measurement_id(col_buf[0]);

    for (uint32_t px = 0; px < pixels_per_column; ++px) {
        std::ptrdiff_t f_offset = (cols * px) + m_id;
        for (uint32_t x = 0; x < columns_per_packet; ++x) {
            if (!valid[x]) {
                continue;
            }

            auto px_dst = col_buf[x] + col_header_size + (px * channel_data_size);

            f_info.set(px_dst, *(data + f_offset + x));
        }
    }
}

// explicitly instantiate for each field type
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define X(size, type)                                          \
    template OUSTER_API_FUNCTION void PacketFormat::set_block( \
        const type* data, int cols, const std::string& field_name, uint8_t* lidar_buf) const;
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SET_BLOCK() TYPE_LIST(0)
SET_BLOCK()
#undef X

// PacketFormat::unpack_raw_headers<T>(...) is defined inline in types.h to
// work around an Apple Clang mangling bug for the dependent default
// StrideType expression in Eigen::Ref. See the note in types.h.

void PacketFormat::set_imu_nmea_ts(uint8_t* imu_buf, uint64_t timestamp) const {
    impl_->imu_nmea_ts_info.set(imu_buf, timestamp);
}

void PacketFormat::set_imu_nmea_sentence(uint8_t* imu_buf, const std::string& sentence) const {
    int underflow = static_cast<int>(NMEA_SENTENCE_LENGTH) - static_cast<int>(sentence.size());
    if (underflow < 0) {
        throw std::invalid_argument(
            "PacketFormat: set_imu_nmea_sentence failed due to sentence being "
            "over the length limit");
    }

    char* start = reinterpret_cast<char*>(imu_buf) + impl_->packet_header_size + 8;
    // NOLINTNEXTLINE(bugprone-not-null-terminated-result)
    std::memcpy(start, sentence.data(), sentence.size());
    std::memset(start + sentence.size(), '\0', underflow);
}

void PacketFormat::set_imu_nmea_sentence(uint8_t* imu_buf, const char* ptr) const {
    char* start = reinterpret_cast<char*>(imu_buf) + impl_->packet_header_size + 8;
    std::memcpy(start, ptr, NMEA_SENTENCE_LENGTH);
}

void PacketFormat::set_imu_la_x(uint8_t* imu_buf, float la_x) const {
    impl_->imu_la_x_info.set(imu_buf, la_x);
}

void PacketFormat::set_imu_la_y(uint8_t* imu_buf, float la_y) const {
    impl_->imu_la_y_info.set(imu_buf, la_y);
}

void PacketFormat::set_imu_la_z(uint8_t* imu_buf, float la_z) const {
    impl_->imu_la_z_info.set(imu_buf, la_z);
}

void PacketFormat::set_imu_av_x(uint8_t* imu_buf, float av_x) const {
    impl_->imu_av_x_info.set(imu_buf, av_x);
}

void PacketFormat::set_imu_av_y(uint8_t* imu_buf, float av_y) const {
    impl_->imu_av_y_info.set(imu_buf, av_y);
}

void PacketFormat::set_imu_av_z(uint8_t* imu_buf, float av_z) const {
    impl_->imu_av_z_info.set(imu_buf, av_z);
}

void PacketFormat::set_zone_timestamp(uint8_t* zone_buf, uint64_t timestamp) const {
    impl_->zone_timestamp_info.set(zone_buf, timestamp);
}

// TWS 20260113: not made static to preserve the interface
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void PacketFormat::set_live_zoneset_hash(uint8_t* zone_buf, const uint8_t* hash) const {
    size_t offset = 40;  // header(32) + timestamp(8) offset
    size_t zoneset_hash_size = 32;
    std::memcpy(zone_buf + offset, hash, zoneset_hash_size);
}

void PacketFormat::set_zone_state(uint8_t* zone_measurement, const ZoneState& zone) const {
    impl_->zone_live_info.set(zone_measurement, zone.live);
    impl_->zone_id_info.set(zone_measurement, zone.id);
    impl_->zone_error_flags_info.set(zone_measurement, zone.error_flags);
    impl_->zone_trigger_type_info.set(zone_measurement, zone.trigger_type);
    impl_->zone_trigger_status_info.set(zone_measurement, zone.trigger_status);
    impl_->zone_triggered_frames_info.set(zone_measurement, zone.triggered_frames);
    impl_->zone_count_info.set(zone_measurement, zone.count);
    impl_->zone_occlusion_count_info.set(zone_measurement, zone.occlusion_count);
    impl_->zone_invalid_count_info.set(zone_measurement, zone.invalid_count);
    impl_->zone_max_count_info.set(zone_measurement, zone.max_count);
    impl_->zone_min_range_info.set(zone_measurement, zone.min_range);
    impl_->zone_max_range_info.set(zone_measurement, zone.max_range);
    impl_->zone_mean_range_info.set(zone_measurement, zone.mean_range);
}

namespace {

constexpr int CRC64_TABLE_SIZE = 256;

std::array<uint64_t, CRC64_TABLE_SIZE> crc64_init() {
    // Generate LUT of all possible 8-bit CRCs to speed up CRC calculation
    // This is for the ECMA-182 CRC64 implementation used on the sensor.
    constexpr uint64_t poly = 0xC96C5795D7870F42;
    std::array<uint64_t, CRC64_TABLE_SIZE> arr = {0};
    for (uint32_t i = 0; i < CRC64_TABLE_SIZE; ++i) {
        uint64_t crc_register = i;
        for (uint32_t j = 0; j < 8; ++j) {
            crc_register = (crc_register >> 1) ^ (poly & ~((crc_register & 1) - 1));
        }
        // TWS 20260113: we know the array is the right size
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        arr[i] = crc_register;
    }

    return arr;
}

// NOLINTNEXTLINE(readability-identifier-naming)
const std::array<uint64_t, CRC64_TABLE_SIZE> crc64_table = crc64_init();

uint64_t crc64_compute(const uint8_t* buf, size_t len) {
    uint64_t crc = ~0;
    // Use Sarwate algorithm LSB-first to calculate the CRC using the LUT.
    while (len != 0) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        crc = crc64_table[*buf++ ^ (crc & 0xFF)] ^ (crc >> 8);
        --len;
    }

    return ~crc;
}
}  // namespace

optional<uint64_t> PacketFormat::crc(const uint8_t* buffer, size_t buffer_size) const {
    if (udp_profile_lidar == UDPProfileLidar::LEGACY ||
        udp_profile_lidar ==  // TODO: this should check header type now
            UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL ||
        header_type == HeaderType::FUSA) {
        return optional<uint64_t>();
    }

    return *(reinterpret_cast<const uint64_t*>(&buffer[buffer_size - 8]));
}

// TWS 20260113: not made static to preserve the interface
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
uint64_t PacketFormat::calculate_crc(const uint8_t* buffer, size_t buffer_size) const {
    return crc64_compute(buffer, buffer_size - 8);
}

uint64_t PacketFormat::zone_timestamp(const uint8_t* zone_packet) const {
    return impl_->zone_timestamp_info.get<uint64_t>(zone_packet);
}

// TWS 20260113: not made static to preserve the interface
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
std::array<uint8_t, 32> PacketFormat::live_zoneset_hash(const uint8_t* zone_packet) const {
    std::array<uint8_t, 32> out{};
    size_t offset = 40;  // header(32) + timestamp(8) offset
    std::memcpy(out.data(), zone_packet + offset, out.size());
    return out;
}

uint8_t* PacketFormat::zone_nth_measurement(size_t meas_idx, uint8_t* zone_packet) const {
    return zone_packet + impl_->zone_measurement_offset + (meas_idx * impl_->zone_measurement_size);
}
const uint8_t* PacketFormat::zone_nth_measurement(size_t meas_idx,
                                                  const uint8_t* zone_packet) const {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    return zone_nth_measurement(meas_idx, const_cast<uint8_t*>(zone_packet));
}

bool PacketFormat::zone_live(const uint8_t* zone_buffer) const {
    return impl_->zone_live_info.get<uint8_t>(zone_buffer) != 0u;
}

uint8_t PacketFormat::zone_id(const uint8_t* zone_buffer) const {
    return impl_->zone_id_info.get<uint8_t>(zone_buffer);
}

uint8_t PacketFormat::zone_error_flags(const uint8_t* zone_buffer) const {
    return impl_->zone_error_flags_info.get<uint8_t>(zone_buffer);
}

uint8_t PacketFormat::zone_trigger_type(const uint8_t* zone_buffer) const {
    return impl_->zone_trigger_type_info.get<uint8_t>(zone_buffer);
}

uint8_t PacketFormat::zone_trigger_status(const uint8_t* zone_buffer) const {
    return impl_->zone_trigger_status_info.get<uint8_t>(zone_buffer);
}

uint32_t PacketFormat::zone_triggered_frames(const uint8_t* zone_buffer) const {
    return impl_->zone_triggered_frames_info.get<uint32_t>(zone_buffer);
}

uint32_t PacketFormat::zone_points_count(const uint8_t* zone_buffer) const {
    return impl_->zone_count_info.get<uint32_t>(zone_buffer);
}

uint32_t PacketFormat::zone_occlusion_count(const uint8_t* zone_buffer) const {
    return impl_->zone_occlusion_count_info.get<uint32_t>(zone_buffer);
}

uint32_t PacketFormat::zone_invalid_count(const uint8_t* zone_buffer) const {
    return impl_->zone_invalid_count_info.get<uint32_t>(zone_buffer);
}

uint32_t PacketFormat::zone_max_count(const uint8_t* zone_buffer) const {
    return impl_->zone_max_count_info.get<uint32_t>(zone_buffer);
}

uint32_t PacketFormat::zone_min_range(const uint8_t* zone_buffer) const {
    return impl_->zone_min_range_info.get<uint32_t>(zone_buffer);
}

uint32_t PacketFormat::zone_max_range(const uint8_t* zone_buffer) const {
    return impl_->zone_max_range_info.get<uint32_t>(zone_buffer);
}

uint32_t PacketFormat::zone_mean_range(const uint8_t* zone_buffer) const {
    return impl_->zone_mean_range_info.get<uint32_t>(zone_buffer);
}

static_assert(sizeof(ZoneState) == 37, "ZoneState must have a fixed size across all platforms");

int PacketFormat::frame_id_difference(uint32_t current, uint32_t other) const {
    int64_t half = max_frame_id >> 1;
    int64_t delta = static_cast<int64_t>(other) - current;
    if (delta < -half) {
        delta += static_cast<int64_t>(max_frame_id) + 1;
    } else if (delta > half) {
        delta -= static_cast<int64_t>(max_frame_id) + 1;
    }
    return static_cast<int>(delta);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

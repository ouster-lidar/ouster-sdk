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
#include <utility>

#include "ouster/field.h"
#include "ouster/impl/packet_writer.h"
#include "ouster/types.h"

namespace ouster {
namespace sdk {
namespace core {

namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

/**
 * Helper struct to load/store bit sequences from packets
 *
 * NOTE: getters and setters require up to 64 bits of valid memory past the bit
 *       we are attempting to set/retrieve, so caution is advised.
 */
struct FieldInfo {
    ChanFieldType ty_tag;
    size_t offset;
    uint64_t mask;
    int shift;

    /**
     * Retrieves the value from the buffer.
     * NOTE: the check that T is of at least the size of ChanFieldType used
     *       is deferred because this function is used in the hot loop
     *
     * @param[in] buffer buffer to retrieve the value from.
     *
     * @return value
     */
    template <typename T>
    T get(const uint8_t* buffer) const {
        uint64_t word = *reinterpret_cast<const uint64_t*>(buffer + offset);
        word &= mask;
        if (shift > 0) {
            word >>= shift;
        } else if (shift < 0) {
            word <<= std::abs(shift);
        }

        T out{};
        std::memcpy(&out, &word, sizeof(out));
        return out;
    }

    /**
     * Stores the value into the buffer.
     * NOTE: the check that T is of at least the size of ChanFieldType used
     *       is deferred because this function is used in the hot loop
     *
     * @param[in] buffer buffer to retrieve the value from.
     * @param[in] value value to store
     */
    template <typename T>
    void set(uint8_t* buffer, T value) const {
        uint64_t word = 0;
        std::memcpy(&word, &value, sizeof(value));
        if (shift > 0) {
            word <<= shift;
        }
        if (shift < 0) {
            word >>= std::abs(shift);
        }
        word &= mask;
        uint64_t* ptr = reinterpret_cast<uint64_t*>(buffer + offset);
        *ptr &= ~mask;
        *ptr |= word;
    }
};

/**
 * FieldInfo factory function
 *
 * NOTE: FieldInfo getters and setters require up to 64 bits of valid memory
 *       past the bit_start, caution is advised.
 *
 * @param[in] bit_start starting bit of the value in the buffer
 * @param[in] bit_size size, in bits, of the value in the buffer
 * @param[in] upshift amount of bits to shift the value up, if any; this is
 *            used in packet values that are truncating lower significance bits,
 *            e.g. in low bandwidth profiles
 *
 * @return FieldInfo
 */
FieldInfo field_info(size_t bit_start, size_t bit_size, size_t upshift = 0) {
    FieldInfo info{};

    size_t needs_bits = bit_size + upshift;
    if (needs_bits > 64) {
        throw std::invalid_argument(
            "failed creating FieldInfo: value cannot store more than 64 bits");
    }

    info.offset = bit_start / 8;
    bit_start = bit_start % 8;

    for (size_t i = bit_start; i < bit_start + bit_size; ++i) {
        info.mask |= uint64_t{1} << i;
    }

    info.shift = bit_start;
    info.shift -= upshift;

    size_t size_bytes = (needs_bits / 8) + (((needs_bits % 8) != 0u) ? 1 : 0);

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

    return info;
}

static int count_set_bits(uint64_t value) {
    int count = 0;
    while (value != 0u) {
        count += value & 1;
        value >>= 1;
    }
    return count;
};

OUSTER_API_FUNCTION
uint64_t get_value_mask(const FieldInfo& field_info) {
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
int get_bitness(const FieldInfo& field_info) {
    return count_set_bits(get_value_mask(field_info));
}

struct ProfileEntry {
    const std::pair<std::string, FieldInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

static const Table<std::string, FieldInfo, 8> LEGACY_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 20)},
    {ChanField::FLAGS, field_info(28, 4)},
    {ChanField::REFLECTIVITY, field_info(32, 8)},
    {ChanField::SIGNAL, field_info(48, 16)},
    {ChanField::NEAR_IR, field_info(64, 16)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
}};

static const Table<std::string, FieldInfo, 5> LB_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::NEAR_IR, field_info(24, 8, 4)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
}};

static const Table<std::string, FieldInfo, 5> LB_WINDOW_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::WINDOW, field_info(24, 8)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
}};

static const Table<std::string, FieldInfo, 14> DUAL_FIELD_INFO{{
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

static const Table<std::string, FieldInfo, 9> SINGLE_FIELD_INFO{{
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

static const Table<std::string, FieldInfo, 14> FIVE_WORD_PIXEL_INFO{{
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

static const Table<std::string, FieldInfo, 7> ZM_LB_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::NEAR_IR, field_info(24, 8, 4)},
    {ChanField::ZONE_MASK, field_info(32, 16)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
}};

static const Table<std::string, FieldInfo, 9> ZM_SINGLE_FIELD_INFO{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(32, 8)},
    {ChanField::SIGNAL, field_info(48, 16)},
    {ChanField::NEAR_IR, field_info(64, 16)},
    {ChanField::ZONE_MASK, field_info(80, 16)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
}};

static const Table<std::string, FieldInfo, 10> DUAL_LB_FIELD_INFO{{
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

static const Table<std::string, FieldInfo, 0> OFF_PROFILE_INFO{};

Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES> OUSTER_API_FUNCTION
    profiles{{
        {UDPProfileLidar::LEGACY,
         {LEGACY_FIELD_INFO.data(), LEGACY_FIELD_INFO.size(), 12}},
        {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
         {DUAL_FIELD_INFO.data(), DUAL_FIELD_INFO.size(), 16}},
        {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16,
         {SINGLE_FIELD_INFO.data(), SINGLE_FIELD_INFO.size(), 12}},
        {UDPProfileLidar::RNG15_RFL8_NIR8,
         {LB_FIELD_INFO.data(), LB_FIELD_INFO.size(), 4}},
        {UDPProfileLidar::FIVE_WORD_PIXEL,
         {FIVE_WORD_PIXEL_INFO.data(), FIVE_WORD_PIXEL_INFO.size(), 20}},
        {UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL,
         {DUAL_LB_FIELD_INFO.data(), DUAL_LB_FIELD_INFO.size(), 8}},
        {UDPProfileLidar::RNG15_RFL8_NIR8_DUAL,
         {DUAL_LB_FIELD_INFO.data(), DUAL_LB_FIELD_INFO.size(), 8}},
        {UDPProfileLidar::OFF,
         {OFF_PROFILE_INFO.data(), OFF_PROFILE_INFO.size(), 0}},
        {UDPProfileLidar::RNG15_RFL8_NIR8_ZONE16,
         {ZM_LB_FIELD_INFO.data(), ZM_LB_FIELD_INFO.size(), 8}},
        {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_ZONE16,
         {ZM_SINGLE_FIELD_INFO.data(), ZM_SINGLE_FIELD_INFO.size(), 12}},
        {UDPProfileLidar::RNG15_RFL8_WIN8,
         {LB_WINDOW_FIELD_INFO.data(), LB_WINDOW_FIELD_INFO.size(), 4}},
    }};

OUSTER_API_FUNCTION Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES>
get_profiles() {
    return profiles;
}

static const ProfileEntry& lookup_profile_entry(UDPProfileLidar profile) {
    auto end = profiles.end();
    auto it = std::find_if(impl::profiles.begin(), end,
                           [profile](const auto& profile_pair) {
                               return profile_pair.first == profile;
                           });

    if (it == end || it->first == UDPProfileLidar::UNKNOWN) {
        throw std::invalid_argument("Unknown lidar udp profile");
    }

    return it->second;
}

}  // namespace impl

struct PacketFormat::Impl {
    size_t packet_header_size;
    size_t col_header_size;
    size_t channel_data_size;
    size_t col_footer_size;
    size_t packet_footer_size;
    size_t imu_measurement_offset;
    size_t imu_measurement_size;
    size_t zone_measurement_offset;
    size_t zone_measurement_size;

    uint64_t max_frame_id;

    size_t col_size;
    size_t lidar_packet_size;
    size_t imu_packet_size;
    size_t zone_packet_size;

    UDPProfileIMU imu_profile;

    std::map<std::string, impl::FieldInfo> fields;

    // header infos
    impl::FieldInfo packet_type_info;
    impl::FieldInfo frame_id_info;
    impl::FieldInfo init_id_info;
    impl::FieldInfo prod_sn_info;
    impl::FieldInfo alert_flags_info;
    impl::FieldInfo countdown_thermal_shutdown_info;
    impl::FieldInfo countdown_shot_limiting_info;
    impl::FieldInfo thermal_shutdown_info;
    impl::FieldInfo shot_limiting_info;

    // column infos
    impl::FieldInfo col_status_info;
    impl::FieldInfo col_timestamp_info;
    impl::FieldInfo col_measurement_id_info;

    // imu infos
    impl::FieldInfo imu_sys_ts_info;
    impl::FieldInfo imu_accel_ts_info;
    impl::FieldInfo imu_gyro_ts_info;
    impl::FieldInfo imu_nmea_ts_info;
    impl::FieldInfo imu_la_x_info;
    impl::FieldInfo imu_la_y_info;
    impl::FieldInfo imu_la_z_info;
    impl::FieldInfo imu_av_x_info;
    impl::FieldInfo imu_av_y_info;
    impl::FieldInfo imu_av_z_info;

    impl::FieldInfo zone_timestamp_info;
    impl::FieldInfo zone_live_info;
    impl::FieldInfo zone_id_info;
    impl::FieldInfo zone_error_flags_info;
    impl::FieldInfo zone_trigger_type_info;
    impl::FieldInfo zone_trigger_status_info;
    impl::FieldInfo zone_triggered_frames_info;
    impl::FieldInfo zone_count_info;
    impl::FieldInfo zone_occlusion_count_info;
    impl::FieldInfo zone_invalid_count_info;
    impl::FieldInfo zone_max_count_info;
    impl::FieldInfo zone_min_range_info;
    impl::FieldInfo zone_max_range_info;
    impl::FieldInfo zone_mean_range_info;

    Impl() = default;

    Impl(const DataFormat& format) : Impl() {
        bool legacy = (format.udp_profile_lidar == UDPProfileLidar::LEGACY);
        bool fusa = (format.header_type == HeaderType::FUSA) && !legacy;

        const auto& entry =
            impl::lookup_profile_entry(format.udp_profile_lidar);

        packet_header_size = legacy ? 0 : 32;
        col_header_size = legacy ? 16 : 12;
        channel_data_size = entry.chan_data_size;
        col_footer_size = legacy ? 4 : 0;
        packet_footer_size = legacy ? 0 : 32;

        col_size = col_header_size +
                   format.pixels_per_column * channel_data_size +
                   col_footer_size;
        lidar_packet_size = packet_header_size +
                            format.columns_per_packet * col_size +
                            packet_footer_size;

        if (lidar_packet_size > 65535) {
            throw std::invalid_argument(
                "lidar_packet_size cannot exceed 65535");
        }

        fields = {entry.fields, entry.fields + entry.n_fields};

        if (fusa) {
            max_frame_id = std::numeric_limits<uint32_t>::max();
        } else {
            max_frame_id = std::numeric_limits<uint16_t>::max();
        }

        using impl::field_info;

        // LIDAR packet and common LIDAR/IMU headers
        if (legacy) {
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

            col_status_info = field_info(8 * (col_size - col_footer_size), 32);
            /**
             * LEGACY col_status sits at the end of the column as opposed to
             * being in column header, and FieldInfo::get takes 8-byte word,
             * which would read memory values past the end of the packet.
             *
             * This is a crude way of making it read 8-byte word from the left
             * instead.
             * TODO: if we run into this issue again, add "pad_left" parameter
             *       to field_info(), otherwise leaving it here
             * -- Tim T.
             */
            col_status_info.offset -= 4;
            col_status_info.mask <<= 32;
            col_status_info.shift += 32;
        } else if (fusa) {
            packet_type_info = field_info(0, 8);
            frame_id_info = field_info(32, 32);
            init_id_info = field_info(8, 24);
            alert_flags_info = field_info(
                64, 8);  // Supposedly supported in both 2.5.X and 3.1.X
            prod_sn_info = field_info(88, 40);
            countdown_thermal_shutdown_info = field_info(128, 8);
            countdown_shot_limiting_info = field_info(136, 8);
            thermal_shutdown_info = field_info(144, 4);
            shot_limiting_info = field_info(156, 4);

            col_status_info = field_info(80, 16);
        } else {
            packet_type_info = field_info(0, 16);
            frame_id_info = field_info(16, 16);
            init_id_info = field_info(32, 24);
            prod_sn_info = field_info(56, 40);
            alert_flags_info = field_info(
                96, 8);  // Supposedly supported in both 2.5.X and 3.1.X
            countdown_thermal_shutdown_info = field_info(128, 8);
            countdown_shot_limiting_info = field_info(136, 8);
            thermal_shutdown_info = field_info(144, 4);
            shot_limiting_info = field_info(156, 4);

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
            imu_sys_ts_info = field_info(0, 64);
            imu_accel_ts_info = field_info(64, 64);
            imu_gyro_ts_info = field_info(128, 64);
            imu_nmea_ts_info = field_info(0, 0);
            imu_la_x_info = field_info(192, 32);
            imu_la_y_info = field_info(224, 32);
            imu_la_z_info = field_info(256, 32);
            imu_av_x_info = field_info(288, 32);
            imu_av_y_info = field_info(320, 32);
            imu_av_z_info = field_info(352, 32);
        } else if (imu_profile == UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
            size_t nmea_block_size = 100;
            imu_measurement_size = 36;
            imu_packet_size =
                packet_header_size + nmea_block_size +
                format.imu_measurements_per_packet * imu_measurement_size +
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

        zone_measurement_offset =
            packet_header_size + 8 /*timestamp*/ + 32 /*hash*/;
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
                           zone_measurement_size * 16 + packet_footer_size;
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
        field_types_.emplace_back(kv.first, kv.second.ty_tag);
    }
}

PacketFormat::PacketFormat(const SensorInfo& info)
    : PacketFormat(info.format) {}

template <typename T, int BlockDim>
void PacketFormat::block_field(Eigen::Ref<img_t<T>> field,
                               const std::string& chan,
                               const uint8_t* packet_buf) const {
    impl::FieldInfo field_info = impl_->fields.at(chan);

    if (sizeof(T) < field_type_size(field_info.ty_tag)) {
        throw std::invalid_argument("Dest type too small for specified field");
    }

    size_t channel_data_size = impl_->channel_data_size;

    int cols = field.cols();

    T* data = field.data();
    std::array<const uint8_t*, BlockDim> col_buf;

    for (int icol = 0; icol < columns_per_packet; icol += BlockDim) {
        for (int i = 0; i < BlockDim; ++i) {
            col_buf[i] = nth_col(icol + i, packet_buf);
        }

        uint16_t m_id = col_measurement_id(col_buf[0]);

        for (int px = 0; px < pixels_per_column; ++px) {
            std::ptrdiff_t f_offset = (cols * px) + m_id;
            for (int x = 0; x < BlockDim; ++x) {
                auto px_src =
                    col_buf[x] + col_header_size + (px * channel_data_size);
                *(data + f_offset + x) = field_info.get<T>(px_src);
            }
        }
    }
}

template <typename T>
void PacketFormat::col_field(const uint8_t* col_buf, const std::string& chan,
                             T* dst, int dst_stride) const {
    impl::FieldInfo field_info = impl_->fields.at(chan);

    if (sizeof(T) < field_type_size(field_info.ty_tag)) {
        throw std::invalid_argument("Dest type too small for specified field");
    }

    size_t channel_data_size = impl_->channel_data_size;

    for (int px = 0; px < pixels_per_column; px++) {
        auto px_src = col_buf + col_header_size + (px * channel_data_size);
        T* px_dst = dst + (px * dst_stride);
        *px_dst = field_info.get<T>(px_src);
    }
}

// explicitly instantiate for each field type / block dim
template void PacketFormat::block_field<uint8_t, 4>(
    Eigen::Ref<img_t<uint8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint16_t, 4>(
    Eigen::Ref<img_t<uint16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint32_t, 4>(
    Eigen::Ref<img_t<uint32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint64_t, 4>(
    Eigen::Ref<img_t<uint64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int8_t, 4>(
    Eigen::Ref<img_t<int8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int16_t, 4>(
    Eigen::Ref<img_t<int16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int32_t, 4>(
    Eigen::Ref<img_t<int32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int64_t, 4>(
    Eigen::Ref<img_t<int64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<float, 4>(
    Eigen::Ref<img_t<float>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<double, 4>(
    Eigen::Ref<img_t<double>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint8_t, 8>(
    Eigen::Ref<img_t<uint8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint16_t, 8>(
    Eigen::Ref<img_t<uint16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint32_t, 8>(
    Eigen::Ref<img_t<uint32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint64_t, 8>(
    Eigen::Ref<img_t<uint64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int8_t, 8>(
    Eigen::Ref<img_t<int8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int16_t, 8>(
    Eigen::Ref<img_t<int16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int32_t, 8>(
    Eigen::Ref<img_t<int32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int64_t, 8>(
    Eigen::Ref<img_t<int64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<float, 8>(
    Eigen::Ref<img_t<float>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<double, 8>(
    Eigen::Ref<img_t<double>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint8_t, 16>(
    Eigen::Ref<img_t<uint8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint16_t, 16>(
    Eigen::Ref<img_t<uint16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint32_t, 16>(
    Eigen::Ref<img_t<uint32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<uint64_t, 16>(
    Eigen::Ref<img_t<uint64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int8_t, 16>(
    Eigen::Ref<img_t<int8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int16_t, 16>(
    Eigen::Ref<img_t<int16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int32_t, 16>(
    Eigen::Ref<img_t<int32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<int64_t, 16>(
    Eigen::Ref<img_t<int64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<float, 16>(
    Eigen::Ref<img_t<float>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void PacketFormat::block_field<double, 16>(
    Eigen::Ref<img_t<double>> field, const std::string& chan,
    const uint8_t* packet_buf) const;

// explicitly instantiate for each field type
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      uint8_t*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      uint16_t*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      uint32_t*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      uint64_t*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      int8_t*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      int16_t*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      int32_t*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      int64_t*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      float*, int) const;
template void PacketFormat::col_field(const uint8_t*, const std::string&,
                                      double*, int) const;

ChanFieldType PacketFormat::field_type(const std::string& field_name) const {
    return (impl_->fields.count(field_name) != 0u)
               ? impl_->fields.at(field_name).ty_tag
               : ChanFieldType::VOID;
}

PacketFormat::FieldIter PacketFormat::begin() const {
    return field_types_.cbegin();
}

PacketFormat::FieldIter PacketFormat::end() const {
    return field_types_.cend();
}

/* Packet headers */

uint16_t PacketFormat::packet_type(const uint8_t* lidar_buf) const {
    return impl_->packet_type_info.get<uint16_t>(lidar_buf);
}

uint32_t PacketFormat::frame_id(const uint8_t* lidar_buf) const {
    return impl_->frame_id_info.get<uint32_t>(lidar_buf);
}

uint32_t PacketFormat::init_id(const uint8_t* lidar_buf) const {
    return impl_->init_id_info.get<uint32_t>(lidar_buf);
}

uint64_t PacketFormat::prod_sn(const uint8_t* lidar_buf) const {
    return impl_->prod_sn_info.get<uint64_t>(lidar_buf);
}

uint8_t PacketFormat::alert_flags(const uint8_t* lidar_buf) const {
    return impl_->alert_flags_info.get<uint8_t>(lidar_buf);
}

uint16_t PacketFormat::countdown_thermal_shutdown(
    const uint8_t* lidar_buf) const {
    return impl_->countdown_thermal_shutdown_info.get<uint16_t>(lidar_buf);
}

uint16_t PacketFormat::countdown_shot_limiting(const uint8_t* lidar_buf) const {
    return impl_->countdown_shot_limiting_info.get<uint16_t>(lidar_buf);
}

uint8_t PacketFormat::thermal_shutdown(const uint8_t* lidar_buf) const {
    return impl_->thermal_shutdown_info.get<uint8_t>(lidar_buf);
}

uint8_t PacketFormat::shot_limiting(const uint8_t* lidar_buf) const {
    return impl_->shot_limiting_info.get<uint8_t>(lidar_buf);
}

const uint8_t* PacketFormat::footer(const uint8_t* lidar_buf) const {
    if (impl_->packet_footer_size == 0) {
        return nullptr;
    }
    return lidar_buf + impl_->packet_header_size +
           (columns_per_packet * impl_->col_size);
}

/* Measurement block access */

const uint8_t* PacketFormat::nth_col(int col_idx,
                                     const uint8_t* lidar_buf) const {
    return lidar_buf + impl_->packet_header_size + (col_idx * impl_->col_size);
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

const uint8_t* PacketFormat::nth_px(int px_idx, const uint8_t* col_buf) const {
    return col_buf + impl_->col_header_size +
           (px_idx * impl_->channel_data_size);
}

/* IMU packet parsing */

const uint8_t* PacketFormat::imu_nth_measurement(int meas_idx,
                                                 const uint8_t* imu_buf) const {
    // in LEGACY both offset and size are zero so we always get back imu_buf
    return imu_buf + impl_->imu_measurement_offset +
           (meas_idx * impl_->imu_measurement_size);
}

std::string PacketFormat::imu_nmea_sentence(const uint8_t* imu_buf) const {
    const char* start =
        reinterpret_cast<const char*>(imu_buf) + impl_->packet_header_size + 8;
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

void PacketFormat::parse_accel(size_t col_offset, const uint8_t* imu_buf,
                               Field& accel) {
    // copy over FieldInfos into local stack for speed
    impl::FieldInfo la_x = impl_->imu_la_x_info;
    impl::FieldInfo la_y = impl_->imu_la_y_info;
    impl::FieldInfo la_z = impl_->imu_la_z_info;

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

void PacketFormat::parse_gyro(size_t col_offset, const uint8_t* imu_buf,
                              Field& gyro) {
    // copy over FieldInfos into local stack for speed
    impl::FieldInfo av_x = impl_->imu_av_x_info;
    impl::FieldInfo av_y = impl_->imu_av_y_info;
    impl::FieldInfo av_z = impl_->imu_av_z_info;

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

bool parse_lat_long(const std::string& nmea_sentence, double& latitude,
                    double& longitude) {
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
bool operator<(const DataFormat& lhs, const DataFormat& rhs) {
    return std::tie(lhs.pixels_per_column, lhs.columns_per_packet,
                    lhs.columns_per_frame, lhs.imu_measurements_per_packet,
                    lhs.pixel_shift_by_row, lhs.column_window,
                    lhs.udp_profile_lidar, lhs.udp_profile_imu,
                    lhs.header_type) <
           std::tie(rhs.pixels_per_column, rhs.columns_per_packet,
                    rhs.columns_per_frame, rhs.imu_measurements_per_packet,
                    rhs.pixel_shift_by_row, rhs.column_window,
                    rhs.udp_profile_lidar, rhs.udp_profile_imu,
                    rhs.header_type);
}

// TODO[tws] consider removal. This is only used when constructing a
// ScanBatcher, which happens rarely.
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
    return get_value_mask(field_info);
}

int PacketFormat::field_bitness(const std::string& field_name) const {
    const auto& field_info = impl_->fields.at(field_name);
    return get_bitness(field_info);
}

/* packet_writer implementation */
namespace impl {
PacketWriter::PacketWriter(const PacketFormat& packet_format)
    : PacketFormat(packet_format) {}

uint8_t* PacketWriter::nth_col(int col_idx, uint8_t* lidar_buf) const {
    return const_cast<uint8_t*>(PacketFormat::nth_col(col_idx, lidar_buf));
}

uint8_t* PacketWriter::nth_px(int px_idx, uint8_t* col_buf) const {
    return const_cast<uint8_t*>(PacketFormat::nth_px(px_idx, col_buf));
}

uint8_t* PacketWriter::footer(uint8_t* lidar_buf) const {
    return const_cast<uint8_t*>(PacketFormat::footer(lidar_buf));
}

void PacketWriter::set_col_status(uint8_t* col_buf, uint32_t status) const {
    impl_->col_status_info.set(col_buf, status);
}

void PacketWriter::set_col_timestamp(uint8_t* col_buf,
                                     uint64_t timestamp) const {
    impl_->col_timestamp_info.set(col_buf, timestamp);
}

void PacketWriter::set_col_measurement_id(uint8_t* col_buf,
                                          uint16_t m_id) const {
    impl_->col_measurement_id_info.set(col_buf, m_id);
}

void PacketWriter::set_frame_id(uint8_t* lidar_buf, uint32_t frame_id) const {
    impl_->frame_id_info.set(lidar_buf, frame_id);
}

void PacketWriter::set_init_id(uint8_t* lidar_buf, uint32_t init_id) const {
    impl_->init_id_info.set(lidar_buf, init_id);
}

void PacketWriter::set_packet_type(uint8_t* packet_buf,
                                   uint16_t packet_type) const {
    impl_->packet_type_info.set(packet_buf, packet_type);
}

void PacketWriter::set_prod_sn(uint8_t* lidar_buf,
                               uint64_t serial_number) const {
    impl_->prod_sn_info.set(lidar_buf, serial_number);
}

void PacketWriter::set_alert_flags(uint8_t* lidar_buf,
                                   uint8_t alert_flags) const {
    impl_->alert_flags_info.set(lidar_buf, alert_flags);
}

void PacketWriter::set_shutdown(uint8_t* lidar_buf, uint8_t status) const {
    impl_->thermal_shutdown_info.set(lidar_buf, status);
}

void PacketWriter::set_shot_limiting(uint8_t* lidar_buf, uint8_t status) const {
    impl_->shot_limiting_info.set(lidar_buf, status);
}

void PacketWriter::set_shutdown_countdown(uint8_t* lidar_buf,
                                          uint8_t shutdown_countdown) const {
    impl_->countdown_thermal_shutdown_info.set(lidar_buf, shutdown_countdown);
}

void PacketWriter::set_shot_limiting_countdown(
    uint8_t* lidar_buf, uint8_t shot_limiting_countdown) const {
    impl_->countdown_shot_limiting_info.set(lidar_buf, shot_limiting_countdown);
}

template <typename T>
void PacketWriter::set_block(Eigen::Ref<const img_t<T>> field,
                             const std::string& chan,
                             uint8_t* lidar_buf) const {
    constexpr int max_cols = 32;
    if (columns_per_packet > max_cols) {
        throw std::runtime_error("Recompile set_block_impl with larger N");
    }

    impl::FieldInfo f_info = impl_->fields.at(chan);

    size_t channel_data_size = impl_->channel_data_size;

    int cols = field.cols();
    const T* data = field.data();
    std::array<uint8_t*, max_cols> col_buf;
    std::array<bool, max_cols> valid;
    for (int i = 0; i < columns_per_packet; ++i) {
        col_buf[i] = nth_col(i, lidar_buf);
        valid[i] = col_status(col_buf[i]) & 0x01;
    }
    uint16_t m_id = col_measurement_id(col_buf[0]);

    for (int px = 0; px < pixels_per_column; ++px) {
        std::ptrdiff_t f_offset = (cols * px) + m_id;
        for (int x = 0; x < columns_per_packet; ++x) {
            if (!valid[x]) {
                continue;
            }

            auto px_dst =
                col_buf[x] + col_header_size + (px * channel_data_size);

            f_info.set(px_dst, *(data + f_offset + x));
        }
    }
}

template void PacketWriter::set_block(Eigen::Ref<const img_t<uint8_t>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<uint16_t>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<uint32_t>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<uint64_t>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<int8_t>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<int16_t>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<int32_t>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<int64_t>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<float>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;
template void PacketWriter::set_block(Eigen::Ref<const img_t<double>> field,
                                      const std::string& chan,
                                      uint8_t* lidar_buf) const;

template <typename T>
void PacketWriter::unpack_raw_headers(Eigen::Ref<const img_t<T>> field,
                                      uint8_t* lidar_buf) const {
    using ColMajorView = Eigen::Map<Eigen::Array<T, -1, 1, Eigen::ColMajor>>;

    if (sizeof(T) > 4) {
        throw std::invalid_argument(
            "RAW_HEADERS field should be of type"
            "uint32_t or smaller to work correctly");
    }

    uint8_t* col_zero = nth_col(0, lidar_buf);
    uint16_t m_id = col_measurement_id(col_zero);

    size_t ch_size = col_header_size / sizeof(T);
    size_t cf_size = col_footer_size / sizeof(T);
    size_t ph_size = packet_header_size / sizeof(T);
    size_t pf_size = packet_footer_size / sizeof(T);

    size_t ch_offset = 0;
    size_t cf_offset = ch_offset + ch_size;
    size_t ph_offset = cf_offset + cf_size;
    size_t pf_offset = ph_offset + ph_size;

    // fill in header and footer, col0 is sufficient for that
    ColMajorView ph_view(reinterpret_cast<T*>(lidar_buf), ph_size);
    ColMajorView pf_view(reinterpret_cast<T*>(footer(lidar_buf)), pf_size);
    ph_view = field.block(ph_offset, m_id, ph_size, 1);
    pf_view = field.block(pf_offset, m_id, pf_size, 1);

    for (int icol = 0; icol < columns_per_packet; ++icol) {
        uint8_t* col_buf = nth_col(icol, lidar_buf);
        uint8_t* colf_ptr = col_buf + col_size - col_footer_size;

        ColMajorView colh_view(reinterpret_cast<T*>(col_buf), ch_size);
        ColMajorView colf_view(reinterpret_cast<T*>(colf_ptr), cf_size);

        m_id = col_measurement_id(col_buf);

        colh_view = field.block(ch_offset, m_id, ch_size, 1);
        colf_view = field.block(cf_offset, m_id, cf_size, 1);
    }
}

template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<uint8_t>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<uint16_t>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<uint32_t>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<uint64_t>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<int8_t>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<int16_t>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<int32_t>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<int64_t>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<float>> field, uint8_t* lidar_buf) const;
template void PacketWriter::unpack_raw_headers(
    Eigen::Ref<const img_t<double>> field, uint8_t* lidar_buf) const;

uint8_t* PacketWriter::imu_nth_measurement(int meas_idx,
                                           uint8_t* imu_buf) const {
    return const_cast<uint8_t*>(
        PacketFormat::imu_nth_measurement(meas_idx, imu_buf));
}

uint8_t* PacketWriter::zone_nth_measurement(int meas_idx,
                                            uint8_t* zone_buf) const {
    return const_cast<uint8_t*>(
        PacketFormat::zone_nth_measurement(meas_idx, zone_buf));
}

void PacketWriter::set_imu_nmea_ts(uint8_t* imu_buf, uint64_t timestamp) const {
    impl_->imu_nmea_ts_info.set(imu_buf, timestamp);
}

void PacketWriter::set_imu_nmea_sentence(uint8_t* imu_buf,
                                         const std::string& sentence) const {
    int underflow = NMEA_SENTENCE_LENGTH - sentence.size();
    if (underflow < 0) {
        throw std::invalid_argument(
            "PacketWriter: set_imu_nmea_sentence failed due to sentence being "
            "over the length limit");
    }

    char* start =
        reinterpret_cast<char*>(imu_buf) + impl_->packet_header_size + 8;
    std::memcpy(start, sentence.data(), sentence.size());
    std::memset(start + sentence.size(), '\0', underflow);
}

void PacketWriter::set_imu_nmea_sentence(uint8_t* imu_buf,
                                         const char* ptr) const {
    char* start =
        reinterpret_cast<char*>(imu_buf) + impl_->packet_header_size + 8;
    std::memcpy(start, ptr, NMEA_SENTENCE_LENGTH);
}

void PacketWriter::set_imu_la_x(uint8_t* imu_buf, float la_x) const {
    impl_->imu_la_x_info.set(imu_buf, la_x);
}

void PacketWriter::set_imu_la_y(uint8_t* imu_buf, float la_y) const {
    impl_->imu_la_y_info.set(imu_buf, la_y);
}

void PacketWriter::set_imu_la_z(uint8_t* imu_buf, float la_z) const {
    impl_->imu_la_z_info.set(imu_buf, la_z);
}

void PacketWriter::set_imu_av_x(uint8_t* imu_buf, float av_x) const {
    impl_->imu_av_x_info.set(imu_buf, av_x);
}

void PacketWriter::set_imu_av_y(uint8_t* imu_buf, float av_y) const {
    impl_->imu_av_y_info.set(imu_buf, av_y);
}

void PacketWriter::set_imu_av_z(uint8_t* imu_buf, float av_z) const {
    impl_->imu_av_z_info.set(imu_buf, av_z);
}

void PacketWriter::set_zone_timestamp(uint8_t* zone_buf,
                                      uint64_t timestamp) const {
    impl_->zone_timestamp_info.set(zone_buf, timestamp);
}

void PacketWriter::set_live_zoneset_hash(uint8_t* zone_buf,
                                         const uint8_t* hash) const {
    size_t offset = 40;  // header(32) + timestamp(8) offset
    size_t zoneset_hash_size = 32;
    std::memcpy(zone_buf + offset, hash, zoneset_hash_size);
}

void PacketWriter::set_zone_state(uint8_t* zone_m_ptr,
                                  const ZoneState& zone) const {
    impl_->zone_live_info.set(zone_m_ptr, zone.live);
    impl_->zone_id_info.set(zone_m_ptr, zone.id);
    impl_->zone_error_flags_info.set(zone_m_ptr, zone.error_flags);
    impl_->zone_trigger_type_info.set(zone_m_ptr, zone.trigger_type);
    impl_->zone_trigger_status_info.set(zone_m_ptr, zone.trigger_status);
    impl_->zone_triggered_frames_info.set(zone_m_ptr, zone.triggered_frames);
    impl_->zone_count_info.set(zone_m_ptr, zone.count);
    impl_->zone_occlusion_count_info.set(zone_m_ptr, zone.occlusion_count);
    impl_->zone_invalid_count_info.set(zone_m_ptr, zone.invalid_count);
    impl_->zone_max_count_info.set(zone_m_ptr, zone.max_count);
    impl_->zone_min_range_info.set(zone_m_ptr, zone.min_range);
    impl_->zone_max_range_info.set(zone_m_ptr, zone.max_range);
    impl_->zone_mean_range_info.set(zone_m_ptr, zone.mean_range);
}

static std::array<uint64_t, 256> crc64_init() {
    // Generate LUT of all possible 8-bit CRCs to speed up CRC calculation
    // This is for the ECMA-182 CRC64 implementation used on the sensor.
    constexpr uint64_t poly = 0xC96C5795D7870F42;
    std::array<uint64_t, 256> arr = {0};
    for (uint32_t i = 0; i < 256; ++i) {
        uint64_t crc_register = i;
        for (uint32_t j = 0; j < 8; ++j) {
            crc_register =
                (crc_register >> 1) ^ (poly & ~((crc_register & 1) - 1));
        }
        arr[i] = crc_register;
    }

    return arr;
}

static std::array<uint64_t, 256> crc64_table = crc64_init();

uint64_t crc64_compute(const uint8_t* buf, size_t len) {
    uint64_t crc = ~0;
    // Use Sarwate algorithm LSB-first to calculate the CRC using the LUT.
    while (len != 0) {
        crc = crc64_table[*buf++ ^ (crc & 0xFF)] ^ (crc >> 8);
        --len;
    }

    return ~crc;
}
}  // namespace impl

optional<uint64_t> PacketFormat::crc(const uint8_t* buffer,
                                     size_t buffer_size) const {
    if (udp_profile_lidar == UDPProfileLidar::LEGACY ||
        udp_profile_lidar ==  // TODO: this should check header type now
            UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL ||
        header_type == HeaderType::FUSA) {
        return optional<uint64_t>();
    }

    return *(uint64_t*)&buffer[buffer_size - 8];
}

uint64_t PacketFormat::calculate_crc(const uint8_t* buffer,
                                     size_t buffer_size) const {
    return impl::crc64_compute(buffer, buffer_size - 8);
}

uint64_t PacketFormat::zone_timestamp(const uint8_t* zone_packet) const {
    return impl_->zone_timestamp_info.get<uint64_t>(zone_packet);
}

std::array<uint8_t, 32> PacketFormat::live_zoneset_hash(
    const uint8_t* zone_packet) const {
    std::array<uint8_t, 32> out{};
    size_t offset = 40;  // header(32) + timestamp(8) offset
    std::memcpy(out.data(), zone_packet + offset, out.size());
    return out;
}

const uint8_t* PacketFormat::zone_nth_measurement(
    int meas_idx, const uint8_t* imu_buf) const {
    return imu_buf + impl_->zone_measurement_offset +
           (meas_idx * impl_->zone_measurement_size);
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

static_assert(sizeof(ZoneState) == 37,
              "ZoneState must have a fixed size across all platforms");

}  // namespace core
}  // namespace sdk
}  // namespace ouster

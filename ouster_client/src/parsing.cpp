/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "ouster/impl/packet_writer.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor {

namespace impl {

constexpr int imu_packet_size = 48;

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

struct FieldInfo {
    ChanFieldType ty_tag;
    size_t offset;
    uint64_t mask;
    int shift;
};

struct ProfileEntry {
    const std::pair<ChanField, FieldInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

static const Table<ChanField, FieldInfo, 8> legacy_field_info{{
    {ChanField::RANGE, {UINT32, 0, 0x000fffff, 0}},
    {ChanField::FLAGS, {UINT8, 3, 0, 4}},
    {ChanField::REFLECTIVITY, {UINT16, 4, 0, 0}},
    {ChanField::SIGNAL, {UINT16, 6, 0, 0}},
    {ChanField::NEAR_IR, {UINT16, 8, 0, 0}},
    {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
    {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
    {ChanField::RAW32_WORD3, {UINT32, 8, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 5> lb_field_info{{
    {ChanField::RANGE, {UINT32, 0, 0x7fff, -3}},
    {ChanField::FLAGS, {UINT8, 1, 0b10000000, 7}},
    {ChanField::REFLECTIVITY, {UINT8, 2, 0, 0}},
    {ChanField::NEAR_IR, {UINT16, 2, 0xff00, 4}},
    {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 13> dual_field_info{{
    {ChanField::RANGE, {UINT32, 0, 0x0007ffff, 0}},
    {ChanField::FLAGS, {UINT8, 2, 0b11111000, 3}},
    {ChanField::REFLECTIVITY, {UINT8, 3, 0, 0}},
    {ChanField::RANGE2, {UINT32, 4, 0x0007ffff, 0}},
    {ChanField::FLAGS2, {UINT8, 6, 0b11111000, 3}},
    {ChanField::REFLECTIVITY2, {UINT8, 7, 0, 0}},
    {ChanField::SIGNAL, {UINT16, 8, 0, 0}},
    {ChanField::SIGNAL2, {UINT16, 10, 0, 0}},
    {ChanField::NEAR_IR, {UINT16, 12, 0, 0}},
    {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
    {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
    {ChanField::RAW32_WORD3, {UINT32, 8, 0, 0}},
    {ChanField::RAW32_WORD4, {UINT32, 12, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 8> single_field_info{{
    {ChanField::RANGE, {UINT32, 0, 0x0007ffff, 0}},
    {ChanField::FLAGS, {UINT8, 2, 0b11111000, 3}},
    {ChanField::REFLECTIVITY, {UINT8, 4, 0, 0}},
    {ChanField::SIGNAL, {UINT16, 6, 0, 0}},
    {ChanField::NEAR_IR, {UINT16, 8, 0, 0}},
    {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
    {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
    {ChanField::RAW32_WORD3, {UINT32, 8, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 14> five_word_pixel_info{{
    {ChanField::RANGE, {UINT32, 0, 0x0007ffff, 0}},
    {ChanField::FLAGS, {UINT8, 2, 0b11111000, 3}},
    {ChanField::REFLECTIVITY, {UINT8, 3, 0, 0}},
    {ChanField::RANGE2, {UINT32, 4, 0x0007ffff, 0}},
    {ChanField::FLAGS2, {UINT8, 6, 0b11111000, 3}},
    {ChanField::REFLECTIVITY2, {UINT8, 7, 0, 0}},
    {ChanField::SIGNAL, {UINT16, 8, 0, 0}},
    {ChanField::SIGNAL2, {UINT16, 10, 0, 0}},
    {ChanField::NEAR_IR, {UINT16, 12, 0, 0}},
    {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
    {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
    {ChanField::RAW32_WORD3, {UINT32, 8, 0, 0}},
    {ChanField::RAW32_WORD4, {UINT32, 12, 0, 0}},
    {ChanField::RAW32_WORD5, {UINT32, 16, 0, 0}},
}};

static const Table<ChanField, FieldInfo, 9> fusa_two_word_pixel_info{{
    {ChanField::RANGE, {UINT32, 0, 0x7fff, -3}},
    {ChanField::FLAGS, {UINT8, 1, 0b10000000, 7}},
    {ChanField::REFLECTIVITY, {UINT8, 2, 0xff, 0}},
    {ChanField::NEAR_IR, {UINT16, 3, 0xff, -4}},
    {ChanField::RANGE2, {UINT32, 4, 0x7fff, -3}},
    {ChanField::FLAGS2, {UINT8, 5, 0b10000000, 7}},
    {ChanField::REFLECTIVITY2, {UINT8, 6, 0xff, 0}},
    {ChanField::RAW32_WORD1, {UINT32, 0, 0, 0}},
    {ChanField::RAW32_WORD2, {UINT32, 4, 0, 0}},
}};

Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES> profiles{{
    {UDPProfileLidar::PROFILE_LIDAR_LEGACY,
     {legacy_field_info.data(), legacy_field_info.size(), 12}},
    {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
     {dual_field_info.data(), dual_field_info.size(), 16}},
    {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16,
     {single_field_info.data(), single_field_info.size(), 12}},
    {UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8,
     {lb_field_info.data(), lb_field_info.size(), 4}},
    {UDPProfileLidar::PROFILE_FIVE_WORD_PIXEL,
     {five_word_pixel_info.data(), five_word_pixel_info.size(), 20}},
    {UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL,
     {fusa_two_word_pixel_info.data(), fusa_two_word_pixel_info.size(), 8}},
}};

static const ProfileEntry& lookup_profile_entry(UDPProfileLidar profile) {
    auto end = profiles.end();
    auto it =
        std::find_if(impl::profiles.begin(), end,
                     [profile](const auto& kv) { return kv.first == profile; });

    if (it == end || it->first == 0)
        throw std::invalid_argument("Unknown lidar udp profile");

    return it->second;
}

static int count_set_bits(uint64_t value) {
    int count = 0;
    while (value) {
        count += value & 1;
        value >>= 1;
    }
    return count;
};

// TODO: move this out to some generalised FieldInfo utils
uint64_t get_value_mask(const FieldInfo& f) {
    // first get type mask
    uint64_t type_mask = (uint64_t{1} << (field_type_size(f.ty_tag) * 8)) - 1;

    uint64_t mask = f.mask;
    if (mask == 0) mask = type_mask;
    if (f.shift > 0) mask >>= f.shift;
    if (f.shift < 0) mask <<= std::abs(f.shift);
    // final type *may* cut the resultant mask still
    mask &= type_mask;

    return mask;
}

// TODO: move this out to some generalised FieldInfo utils
int get_bitness(const FieldInfo& f) {
    return count_set_bits(get_value_mask(f));
}

}  // namespace impl

struct packet_format::Impl {
    size_t packet_header_size;
    size_t col_header_size;
    size_t channel_data_size;
    size_t col_footer_size;
    size_t packet_footer_size;

    size_t col_size;
    size_t lidar_packet_size;

    size_t timestamp_offset;
    size_t measurement_id_offset;
    size_t status_offset;

    std::map<ChanField, impl::FieldInfo> fields;

    Impl(UDPProfileLidar profile, size_t pixels_per_column,
         size_t columns_per_packet) {
        bool legacy = (profile == UDPProfileLidar::PROFILE_LIDAR_LEGACY);

        const auto& entry = impl::lookup_profile_entry(profile);

        packet_header_size = legacy ? 0 : 32;
        col_header_size = legacy ? 16 : 12;
        channel_data_size = entry.chan_data_size;
        col_footer_size = legacy ? 4 : 0;
        packet_footer_size = legacy ? 0 : 32;

        col_size = col_header_size + pixels_per_column * channel_data_size +
                   col_footer_size;
        lidar_packet_size = packet_header_size + columns_per_packet * col_size +
                            packet_footer_size;

        if (lidar_packet_size > 65535)
            throw std::invalid_argument(
                "lidar_packet_size cannot exceed 65535");

        fields = {entry.fields, entry.fields + entry.n_fields};

        timestamp_offset = 0;
        measurement_id_offset = 8;
        status_offset = legacy ? col_size - col_footer_size : 10;
    }
};

packet_format::packet_format(UDPProfileLidar udp_profile_lidar,
                             size_t pixels_per_column,
                             size_t columns_per_packet)
    : impl_{std::make_shared<Impl>(udp_profile_lidar, pixels_per_column,
                                   columns_per_packet)},
      udp_profile_lidar{udp_profile_lidar},
      lidar_packet_size{impl_->lidar_packet_size},
      imu_packet_size{impl::imu_packet_size},
      columns_per_packet(columns_per_packet),
      pixels_per_column(pixels_per_column),
      packet_header_size{impl_->packet_header_size},
      col_header_size{impl_->col_header_size},
      col_footer_size{impl_->col_footer_size},
      col_size{impl_->col_size},
      packet_footer_size{impl_->packet_footer_size} {
    for (const auto& kv : impl_->fields) {
        field_types_.push_back({kv.first, kv.second.ty_tag});
    }
}

packet_format::packet_format(const sensor_info& info)
    : packet_format(info.format.udp_profile_lidar,
                    info.format.pixels_per_column,
                    info.format.columns_per_packet) {}

template <typename T, typename SRC, int N>
void packet_format::block_field_impl(Eigen::Ref<img_t<T>> field, ChanField chan,
                                     const uint8_t* packet_buf) const {
    if (sizeof(T) < sizeof(SRC))
        throw std::invalid_argument("Dest type too small for specified field");

    const auto& f = impl_->fields.at(chan);

    size_t offset = f.offset;
    uint64_t mask = f.mask;
    int shift = f.shift;
    size_t channel_data_size = impl_->channel_data_size;

    int cols = field.cols();
    T* data = field.data();
    std::array<const uint8_t*, N> col_buf;

    for (int icol = 0; icol < columns_per_packet; icol += N) {
        for (int i = 0; i < N; ++i) {
            col_buf[i] = nth_col(icol + i, packet_buf);
        }

        uint16_t m_id = col_measurement_id(col_buf[0]);

        for (int px = 0; px < pixels_per_column; ++px) {
            std::ptrdiff_t f_offset = cols * px + m_id;
            for (int x = 0; x < N; ++x) {
                auto px_src =
                    col_buf[x] + col_header_size + (px * channel_data_size);
                T dst = *reinterpret_cast<const SRC*>(px_src + offset);
                if (mask) dst &= mask;
                if (shift > 0) dst >>= shift;
                if (shift < 0) dst <<= std::abs(shift);
                *(data + f_offset + x) = dst;
            }
        }
    }
}

template <typename T, int BlockDim,
          typename std::enable_if<std::is_unsigned<T>::value, T>::type>
void packet_format::block_field(Eigen::Ref<img_t<T>> field, ChanField chan,
                                const uint8_t* packet_buf) const {
    const auto& f = impl_->fields.at(chan);

    switch (f.ty_tag) {
        case UINT8:
            block_field_impl<T, uint8_t, BlockDim>(field, chan, packet_buf);
            break;
        case UINT16:
            block_field_impl<T, uint16_t, BlockDim>(field, chan, packet_buf);
            break;
        case UINT32:
            block_field_impl<T, uint32_t, BlockDim>(field, chan, packet_buf);
            break;
        case UINT64:
            block_field_impl<T, uint64_t, BlockDim>(field, chan, packet_buf);
            break;
        default:
            throw std::invalid_argument("Invalid field for packet format");
    }
}

// explicitly instantiate for each field type / block dim
template void packet_format::block_field<uint8_t, 4>(
    Eigen::Ref<img_t<uint8_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint16_t, 4>(
    Eigen::Ref<img_t<uint16_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint32_t, 4>(
    Eigen::Ref<img_t<uint32_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint64_t, 4>(
    Eigen::Ref<img_t<uint64_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint8_t, 8>(
    Eigen::Ref<img_t<uint8_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint16_t, 8>(
    Eigen::Ref<img_t<uint16_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint32_t, 8>(
    Eigen::Ref<img_t<uint32_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint64_t, 8>(
    Eigen::Ref<img_t<uint64_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint8_t, 16>(
    Eigen::Ref<img_t<uint8_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint16_t, 16>(
    Eigen::Ref<img_t<uint16_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint32_t, 16>(
    Eigen::Ref<img_t<uint32_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint64_t, 16>(
    Eigen::Ref<img_t<uint64_t>> field, ChanField chan,
    const uint8_t* packet_buf) const;

template <typename SRC, typename DST>
static void col_field_impl(const uint8_t* col_buf, DST* dst, size_t offset,
                           uint64_t mask, int shift, int pixels_per_column,
                           int dst_stride, size_t channel_data_size,
                           size_t col_header_size) {
    if (sizeof(DST) < sizeof(SRC))
        throw std::invalid_argument("Dest type too small for specified field");

    for (int px = 0; px < pixels_per_column; px++) {
        auto px_src =
            col_buf + col_header_size + offset + (px * channel_data_size);
        DST* px_dst = dst + px * dst_stride;
        DST dst = *reinterpret_cast<const SRC*>(px_src);
        if (mask) dst &= mask;
        if (shift > 0) dst >>= shift;
        if (shift < 0) dst <<= std::abs(shift);
        *px_dst = dst;
    }
}

template <typename T,
          typename std::enable_if<std::is_unsigned<T>::value, T>::type>
void packet_format::col_field(const uint8_t* col_buf, ChanField i, T* dst,
                              int dst_stride) const {
    const auto& f = impl_->fields.at(i);

    switch (f.ty_tag) {
        case UINT8:
            col_field_impl<uint8_t, T>(
                col_buf, dst, f.offset, f.mask, f.shift, pixels_per_column,
                dst_stride, impl_->channel_data_size, impl_->col_header_size);
            break;
        case UINT16:
            col_field_impl<uint16_t, T>(
                col_buf, dst, f.offset, f.mask, f.shift, pixels_per_column,
                dst_stride, impl_->channel_data_size, impl_->col_header_size);
            break;
        case UINT32:
            col_field_impl<uint32_t, T>(
                col_buf, dst, f.offset, f.mask, f.shift, pixels_per_column,
                dst_stride, impl_->channel_data_size, impl_->col_header_size);
            break;
        case UINT64:
            col_field_impl<uint64_t, T>(
                col_buf, dst, f.offset, f.mask, f.shift, pixels_per_column,
                dst_stride, impl_->channel_data_size, impl_->col_header_size);
            break;
        default:
            throw std::invalid_argument("Invalid field for packet format");
    }
}

// explicitly instantiate for each field type
template void packet_format::col_field(const uint8_t*, ChanField, uint8_t*,
                                       int) const;
template void packet_format::col_field(const uint8_t*, ChanField, uint16_t*,
                                       int) const;
template void packet_format::col_field(const uint8_t*, ChanField, uint32_t*,
                                       int) const;
template void packet_format::col_field(const uint8_t*, ChanField, uint64_t*,
                                       int) const;

ChanFieldType packet_format::field_type(ChanField f) const {
    return impl_->fields.count(f) ? impl_->fields.at(f).ty_tag
                                  : ChanFieldType::VOID;
}

packet_format::FieldIter packet_format::begin() const {
    return field_types_.cbegin();
}

packet_format::FieldIter packet_format::end() const {
    return field_types_.cend();
}

/* Packet headers */

uint16_t packet_format::packet_type(const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        // LEGACY profile has no packet_type - use 0 to code as 'legacy'
        return 0;
    }
    uint16_t res = 0;
    if (udp_profile_lidar ==
        UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL) {
        // FuSa profile has 8-bit packet_type
        std::memcpy(&res, lidar_buf + 0, sizeof(uint8_t));
    } else {
        std::memcpy(&res, lidar_buf + 0, sizeof(uint16_t));
    }
    return res;
}

uint32_t packet_format::frame_id(const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        return col_frame_id(nth_col(0, lidar_buf));
    }

    // eUDP frame id is 16 bits, but FUSA frame id is 32 bits
    if (udp_profile_lidar ==
        UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL) {
        uint32_t res = 0;
        std::memcpy(&res, lidar_buf + 4, sizeof(res));
        return res;
    } else {
        uint16_t res = 0;
        std::memcpy(&res, lidar_buf + 2, sizeof(res));
        return res;
    }
}

uint32_t packet_format::init_id(const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        // LEGACY profile has no init_id - use 0 to code as 'legacy'
        return 0;
    }
    uint32_t res = 0;
    if (udp_profile_lidar ==
        UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL) {
        std::memcpy(&res, lidar_buf + 1, sizeof(uint32_t));
    } else {
        std::memcpy(&res, lidar_buf + 4, sizeof(uint32_t));
    }
    return res & 0x00ffffff;
}

uint64_t packet_format::prod_sn(const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        // LEGACY profile has no prod_sn (serial number) - use 0 to code as
        // 'legacy'
        return 0;
    }
    uint64_t res = 0;
    if (udp_profile_lidar ==
        UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL) {
        std::memcpy(&res, lidar_buf + 11, sizeof(uint64_t));
    } else {
        std::memcpy(&res, lidar_buf + 7, sizeof(uint64_t));
    }
    return res & 0x000000ffffffffff;
}

uint16_t packet_format::countdown_thermal_shutdown(
    const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        // LEGACY profile has no shutdown counter in packet header - use 0 for
        // 'normal operation'
        return 0;
    }
    uint16_t res = 0;
    std::memcpy(&res, lidar_buf + 16, sizeof(uint8_t));
    return res;
}

uint16_t packet_format::countdown_shot_limiting(
    const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        // LEGACY profile has no shot limiting countdown in packet header - use
        // 0 for 'normal operation'
        return 0;
    }
    uint16_t res = 0;
    std::memcpy(&res, lidar_buf + 17, sizeof(uint8_t));
    return res;
}

uint8_t packet_format::thermal_shutdown(const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        // LEGACY profile has no shutdown status in packet header - use 0 for
        // 'normal operation'
        return 0;
    }
    uint8_t res = 0;
    std::memcpy(&res, lidar_buf + 18, sizeof(uint8_t));
    return res & 0x0f;
}

uint8_t packet_format::shot_limiting(const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        // LEGACY profile has no shot limiting in packet header - use 0 for
        // 'normal operation'
        return 0;
    }
    uint8_t res = 0;
    std::memcpy(&res, lidar_buf + 19, sizeof(uint8_t));
    return res & 0x0f;
}

const uint8_t* packet_format::footer(const uint8_t* lidar_buf) const {
    if (impl_->packet_footer_size == 0) return nullptr;
    return lidar_buf + impl_->packet_header_size +
           (columns_per_packet * impl_->col_size);
}

/* Measurement block access */

const uint8_t* packet_format::nth_col(int n, const uint8_t* lidar_buf) const {
    return lidar_buf + impl_->packet_header_size + (n * impl_->col_size);
}

uint32_t packet_format::col_status(const uint8_t* col_buf) const {
    uint32_t res = 0;
    std::memcpy(&res, col_buf + impl_->status_offset, sizeof(uint32_t));
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        return res;  // LEGACY was 32 bits of all 1s
    } else {
        return res & 0xffff;  // For eUDP packets, we want the last 16 bits
    }
}

uint64_t packet_format::col_timestamp(const uint8_t* col_buf) const {
    uint64_t res = 0;
    std::memcpy(&res, col_buf + impl_->timestamp_offset, sizeof(uint64_t));
    return res;
}

uint16_t packet_format::col_measurement_id(const uint8_t* col_buf) const {
    uint16_t res = 0;
    std::memcpy(&res, col_buf + impl_->measurement_id_offset, sizeof(uint16_t));
    return res;
}

uint32_t packet_format::col_encoder(const uint8_t* col_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        uint32_t res = 0;
        std::memcpy(&res, col_buf + 12, sizeof(uint32_t));
        return res;
    } else {
        return 0;
    }
}

uint16_t packet_format::col_frame_id(const uint8_t* col_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        uint16_t res = 0;
        std::memcpy(&res, col_buf + 10, sizeof(uint16_t));
        return res;
    } else {
        return 0;
    }
}

/* Channel data fields */

const uint8_t* packet_format::nth_px(int n, const uint8_t* col_buf) const {
    return col_buf + impl_->col_header_size + (n * impl_->channel_data_size);
}

template <typename T>
T packet_format::px_field(const uint8_t* px_buf, ChanField i) const {
    const auto& f = impl_->fields.at(i);

    if (sizeof(T) < field_type_size(f.ty_tag))
        throw std::invalid_argument("Dest type too small for specified field");

    T res = 0;
    std::memcpy(&res, px_buf + f.offset, field_type_size(f.ty_tag));
    if (f.mask) res &= f.mask;
    if (f.shift > 0) res >>= f.shift;
    if (f.shift < 0) res <<= std::abs(f.shift);
    return res;
}

/* IMU packet parsing */

uint64_t packet_format::imu_sys_ts(const uint8_t* imu_buf) const {
    uint64_t res = 0;
    std::memcpy(&res, imu_buf, sizeof(uint64_t));
    return res;
}

uint64_t packet_format::imu_accel_ts(const uint8_t* imu_buf) const {
    uint64_t res = 0;
    std::memcpy(&res, imu_buf + 8, sizeof(uint64_t));
    return res;
}

uint64_t packet_format::imu_gyro_ts(const uint8_t* imu_buf) const {
    uint64_t res = 0;
    std::memcpy(&res, imu_buf + 16, sizeof(uint64_t));
    return res;
}

float packet_format::imu_la_x(const uint8_t* imu_buf) const {
    float res = 0;
    std::memcpy(&res, imu_buf + 24, sizeof(float));
    return res;
}

float packet_format::imu_la_y(const uint8_t* imu_buf) const {
    float res = 0;
    std::memcpy(&res, imu_buf + 28, sizeof(float));
    return res;
}

float packet_format::imu_la_z(const uint8_t* imu_buf) const {
    float res = 0;
    std::memcpy(&res, imu_buf + 32, sizeof(float));
    return res;
}

float packet_format::imu_av_x(const uint8_t* imu_buf) const {
    float res = 0;
    std::memcpy(&res, imu_buf + 36, sizeof(float));
    return res;
}

float packet_format::imu_av_y(const uint8_t* imu_buf) const {
    float res = 0;
    std::memcpy(&res, imu_buf + 40, sizeof(float));
    return res;
}

float packet_format::imu_av_z(const uint8_t* imu_buf) const {
    float res = 0;
    std::memcpy(&res, imu_buf + 44, sizeof(float));
    return res;
}

int packet_format::block_parsable() const {
    std::array<int, 3> dims = {16, 8, 4};
    for (const auto& d : dims) {
        if ((pixels_per_column % d == 0) && (columns_per_packet % d == 0))
            return d;
    }
    return 0;
}

const packet_format& get_format(const sensor_info& info) {
    return get_format(info.format.udp_profile_lidar,
                      info.format.pixels_per_column,
                      info.format.columns_per_packet);
}

const packet_format& get_format(UDPProfileLidar udp_profile_lidar,
                                size_t pixels_per_column,
                                size_t columns_per_packet) {
    using key = std::tuple<size_t, size_t, UDPProfileLidar>;
    static std::map<key, std::unique_ptr<packet_format>> cache{};
    static std::mutex cache_mx{};

    key k{pixels_per_column, columns_per_packet, udp_profile_lidar};

    std::lock_guard<std::mutex> lk{cache_mx};
    if (!cache.count(k)) {
        cache[k] = std::make_unique<packet_format>(
            udp_profile_lidar, pixels_per_column, columns_per_packet);
    }

    return *cache.at(k);
}

uint64_t packet_format::field_value_mask(ChanField i) const {
    const auto& f = impl_->fields.at(i);
    return get_value_mask(f);
}

int packet_format::field_bitness(ChanField i) const {
    const auto& f = impl_->fields.at(i);
    return get_bitness(f);
}

/* packet_writer implementation */
namespace impl {

uint8_t* packet_writer::nth_col(int n, uint8_t* lidar_buf) const {
    return const_cast<uint8_t*>(packet_format::nth_col(n, lidar_buf));
}

uint8_t* packet_writer::nth_px(int n, uint8_t* col_buf) const {
    return const_cast<uint8_t*>(packet_format::nth_px(n, col_buf));
}

uint8_t* packet_writer::footer(uint8_t* lidar_buf) const {
    return const_cast<uint8_t*>(packet_format::footer(lidar_buf));
}

void packet_writer::set_col_status(uint8_t* col_buf, uint32_t status) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        std::memcpy(col_buf + impl_->status_offset, &status, sizeof(uint32_t));
    } else {
        uint16_t s = status & 0xffff;
        std::memcpy(col_buf + impl_->status_offset, &s, sizeof(uint16_t));
    }
}

void packet_writer::set_col_timestamp(uint8_t* col_buf, uint64_t ts) const {
    std::memcpy(col_buf + impl_->timestamp_offset, &ts, sizeof(ts));
}

void packet_writer::set_col_measurement_id(uint8_t* col_buf,
                                           uint16_t m_id) const {
    std::memcpy(col_buf + impl_->measurement_id_offset, &m_id, sizeof(m_id));
}

void packet_writer::set_frame_id(uint8_t* lidar_buf, uint32_t frame_id) const {
    // eUDP frame id is 16 bits, but FUSA frame id is 32 bits
    if (udp_profile_lidar ==
        UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL) {
        std::memcpy(lidar_buf + 4, &frame_id, sizeof(frame_id));
        return;
    }

    uint16_t f_id = static_cast<uint16_t>(frame_id);
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
        std::memcpy(nth_col(0, lidar_buf) + 10, &f_id, sizeof(f_id));
        return;
    }

    std::memcpy(lidar_buf + 2, &f_id, sizeof(f_id));
}

template <typename T>
void packet_writer::set_px(uint8_t* px_buf, ChanField i, T value) const {
    const auto& f = impl_->fields.at(i);

    if (f.shift > 0) value <<= f.shift;
    if (f.shift < 0) value >>= std::abs(f.shift);
    if (f.mask) value &= f.mask;
    T* ptr = reinterpret_cast<T*>(px_buf + f.offset);
    *ptr &= ~f.mask;
    *ptr |= value;
}

template void packet_writer::set_px(uint8_t*, ChanField, uint8_t) const;
template void packet_writer::set_px(uint8_t*, ChanField, uint16_t) const;
template void packet_writer::set_px(uint8_t*, ChanField, uint32_t) const;
template void packet_writer::set_px(uint8_t*, ChanField, uint64_t) const;

template <typename T, typename DST>
void packet_writer::set_block_impl(Eigen::Ref<const img_t<T>> field,
                                   ChanField chan, uint8_t* lidar_buf) const {
    constexpr int N = 32;
    if (columns_per_packet > N)
        throw std::runtime_error("Recompile set_block_impl with larger N");

    const auto& f = impl_->fields.at(chan);

    size_t offset = f.offset;
    uint64_t mask = f.mask;
    int shift = f.shift;
    size_t channel_data_size = impl_->channel_data_size;

    int cols = field.cols();
    const T* data = field.data();
    std::array<uint8_t*, N> col_buf;
    std::array<bool, N> valid;
    for (int i = 0; i < columns_per_packet; ++i) {
        col_buf[i] = nth_col(i, lidar_buf);
        valid[i] = col_status(col_buf[i]) & 0x01;
    }
    uint16_t m_id = col_measurement_id(col_buf[0]);

    for (int px = 0; px < pixels_per_column; ++px) {
        std::ptrdiff_t f_offset = cols * px + m_id;
        for (int x = 0; x < columns_per_packet; ++x) {
            if (!valid[x]) continue;

            auto px_dst =
                col_buf[x] + col_header_size + (px * channel_data_size);

            uint64_t value = *(data + f_offset + x);
            if (shift > 0) value <<= shift;
            if (shift < 0) value >>= std::abs(shift);
            if (mask) value &= mask;
            DST* ptr = reinterpret_cast<DST*>(px_dst + offset);
            *ptr &= ~mask;
            *ptr |= value;
        }
    }
}

template <typename T>
void packet_writer::set_block(Eigen::Ref<const img_t<T>> field, ChanField i,
                              uint8_t* lidar_buf) const {
    const auto& f = impl_->fields.at(i);

    switch (f.ty_tag) {
        case UINT8:
            set_block_impl<T, uint8_t>(field, i, lidar_buf);
            break;
        case UINT16:
            set_block_impl<T, uint16_t>(field, i, lidar_buf);
            break;
        case UINT32:
            set_block_impl<T, uint32_t>(field, i, lidar_buf);
            break;
        case UINT64:
            set_block_impl<T, uint64_t>(field, i, lidar_buf);
            break;
        default:
            throw std::invalid_argument("Invalid field for packet format");
    }
}

template void packet_writer::set_block(Eigen::Ref<const img_t<uint8_t>> field,
                                       ChanField i, uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<uint16_t>> field,
                                       ChanField i, uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<uint32_t>> field,
                                       ChanField i, uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<uint64_t>> field,
                                       ChanField i, uint8_t* lidar_buf) const;

template <typename T>
void packet_writer::unpack_raw_headers(Eigen::Ref<const img_t<T>> field,
                                       uint8_t* lidar_buf) const {
    using ColMajorView = Eigen::Map<Eigen::Array<T, -1, 1, Eigen::ColMajor>>;

    if (sizeof(T) > 4)
        throw std::invalid_argument(
            "RAW_HEADERS field should be of type"
            "uint32_t or smaller to work correctly");

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

template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<uint8_t>> field, uint8_t* lidar_buf) const;
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<uint16_t>> field, uint8_t* lidar_buf) const;
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<uint32_t>> field, uint8_t* lidar_buf) const;
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<uint64_t>> field, uint8_t* lidar_buf) const;

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

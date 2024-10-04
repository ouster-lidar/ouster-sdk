/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
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
        if (shift > 0) word <<= shift;
        if (shift < 0) word >>= std::abs(shift);
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

    size_t size_bytes = needs_bits / 8 + ((needs_bits % 8) ? 1 : 0);

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
    while (value) {
        count += value & 1;
        value >>= 1;
    }
    return count;
};

uint64_t get_value_mask(const FieldInfo& f) {
    uint64_t type_mask = sensor::field_type_mask(f.ty_tag);

    uint64_t mask = f.mask;
    if (mask == 0) mask = type_mask;
    if (f.shift > 0) mask >>= f.shift;
    if (f.shift < 0) mask <<= std::abs(f.shift);
    // final type *may* cut the resultant mask still
    mask &= type_mask;

    return mask;
}

int get_bitness(const FieldInfo& f) {
    return count_set_bits(get_value_mask(f));
}

struct ProfileEntry {
    const std::pair<std::string, FieldInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

static const Table<std::string, FieldInfo, 8> legacy_field_info{{
    {ChanField::RANGE, field_info(0, 20)},
    {ChanField::FLAGS, field_info(28, 4)},
    {ChanField::REFLECTIVITY, field_info(32, 8)},
    {ChanField::SIGNAL, field_info(48, 16)},
    {ChanField::NEAR_IR, field_info(64, 16)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
}};

static const Table<std::string, FieldInfo, 5> lb_field_info{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::NEAR_IR, field_info(24, 8, 4)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
}};

static const Table<std::string, FieldInfo, 13> dual_field_info{{
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
}};

static const Table<std::string, FieldInfo, 8> single_field_info{{
    {ChanField::RANGE, field_info(0, 19)},
    {ChanField::FLAGS, field_info(19, 5)},
    {ChanField::REFLECTIVITY, field_info(32, 8)},
    {ChanField::SIGNAL, field_info(48, 16)},
    {ChanField::NEAR_IR, field_info(64, 16)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
    {ChanField::RAW32_WORD3, field_info(64, 32)},
}};

static const Table<std::string, FieldInfo, 14> five_word_pixel_info{{
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

static const Table<std::string, FieldInfo, 9> fusa_two_word_pixel_info{{
    {ChanField::RANGE, field_info(0, 15, 3)},
    {ChanField::FLAGS, field_info(15, 1)},
    {ChanField::REFLECTIVITY, field_info(16, 8)},
    {ChanField::NEAR_IR, field_info(24, 8, 4)},
    {ChanField::RANGE2, field_info(32, 15, 3)},
    {ChanField::FLAGS2, field_info(47, 1)},
    {ChanField::REFLECTIVITY2, field_info(48, 8)},
    {ChanField::RAW32_WORD1, field_info(0, 32)},
    {ChanField::RAW32_WORD2, field_info(32, 32)},
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

}  // namespace impl

struct packet_format::Impl {
    size_t packet_header_size;
    size_t col_header_size;
    size_t channel_data_size;
    size_t col_footer_size;
    size_t packet_footer_size;

    uint64_t max_frame_id;

    size_t col_size;
    size_t lidar_packet_size;

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

        // TODO: amend how we detect FUSA once we have a different mechanism
        bool fusa = false;
        if (profile == UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL) {
            max_frame_id = std::numeric_limits<uint32_t>::max();
            fusa = true;
        } else {
            max_frame_id = std::numeric_limits<uint16_t>::max();
        }

        using impl::field_info;

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
      packet_footer_size{impl_->packet_footer_size},
      max_frame_id{impl_->max_frame_id} {
    for (const auto& kv : impl_->fields) {
        field_types_.push_back({kv.first, kv.second.ty_tag});
    }
}

packet_format::packet_format(const sensor_info& info)
    : packet_format(info.format.udp_profile_lidar,
                    info.format.pixels_per_column,
                    info.format.columns_per_packet) {}

template <typename T>
class SameSizeInt {
   public:
    typedef T value;
};

template <>
class SameSizeInt<float> {
   public:
    typedef uint32_t value;
};

template <>
class SameSizeInt<double> {
   public:
    typedef uint64_t value;
};

template <typename T, int BlockDim>
void packet_format::block_field(Eigen::Ref<img_t<T>> field,
                                const std::string& chan,
                                const uint8_t* packet_buf) const {
    impl::FieldInfo f = impl_->fields.at(chan);

    if (sizeof(T) < field_type_size(f.ty_tag))
        throw std::invalid_argument("Dest type too small for specified field");

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
            std::ptrdiff_t f_offset = cols * px + m_id;
            for (int x = 0; x < BlockDim; ++x) {
                auto px_src =
                    col_buf[x] + col_header_size + (px * channel_data_size);
                *(data + f_offset + x) = f.get<T>(px_src);
            }
        }
    }
}

template <typename T>
void packet_format::col_field(const uint8_t* col_buf, const std::string& i,
                              T* dst, int dst_stride) const {
    impl::FieldInfo f = impl_->fields.at(i);

    if (sizeof(T) < field_type_size(f.ty_tag))
        throw std::invalid_argument("Dest type too small for specified field");

    size_t channel_data_size = impl_->channel_data_size;

    for (int px = 0; px < pixels_per_column; px++) {
        auto px_src = col_buf + col_header_size + (px * channel_data_size);
        T* px_dst = dst + px * dst_stride;
        *px_dst = f.get<T>(px_src);
    }
}

// explicitly instantiate for each field type / block dim
template void packet_format::block_field<uint8_t, 4>(
    Eigen::Ref<img_t<uint8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint16_t, 4>(
    Eigen::Ref<img_t<uint16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint32_t, 4>(
    Eigen::Ref<img_t<uint32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint64_t, 4>(
    Eigen::Ref<img_t<uint64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int8_t, 4>(
    Eigen::Ref<img_t<int8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int16_t, 4>(
    Eigen::Ref<img_t<int16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int32_t, 4>(
    Eigen::Ref<img_t<int32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int64_t, 4>(
    Eigen::Ref<img_t<int64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<float, 4>(
    Eigen::Ref<img_t<float>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<double, 4>(
    Eigen::Ref<img_t<double>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint8_t, 8>(
    Eigen::Ref<img_t<uint8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint16_t, 8>(
    Eigen::Ref<img_t<uint16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint32_t, 8>(
    Eigen::Ref<img_t<uint32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint64_t, 8>(
    Eigen::Ref<img_t<uint64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int8_t, 8>(
    Eigen::Ref<img_t<int8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int16_t, 8>(
    Eigen::Ref<img_t<int16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int32_t, 8>(
    Eigen::Ref<img_t<int32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int64_t, 8>(
    Eigen::Ref<img_t<int64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<float, 8>(
    Eigen::Ref<img_t<float>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<double, 8>(
    Eigen::Ref<img_t<double>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint8_t, 16>(
    Eigen::Ref<img_t<uint8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint16_t, 16>(
    Eigen::Ref<img_t<uint16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint32_t, 16>(
    Eigen::Ref<img_t<uint32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<uint64_t, 16>(
    Eigen::Ref<img_t<uint64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int8_t, 16>(
    Eigen::Ref<img_t<int8_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int16_t, 16>(
    Eigen::Ref<img_t<int16_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int32_t, 16>(
    Eigen::Ref<img_t<int32_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<int64_t, 16>(
    Eigen::Ref<img_t<int64_t>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<float, 16>(
    Eigen::Ref<img_t<float>> field, const std::string& chan,
    const uint8_t* packet_buf) const;
template void packet_format::block_field<double, 16>(
    Eigen::Ref<img_t<double>> field, const std::string& chan,
    const uint8_t* packet_buf) const;

// explicitly instantiate for each field type
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       uint8_t*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       uint16_t*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       uint32_t*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       uint64_t*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       int8_t*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       int16_t*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       int32_t*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       int64_t*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       float*, int) const;
template void packet_format::col_field(const uint8_t*, const std::string&,
                                       double*, int) const;

ChanFieldType packet_format::field_type(const std::string& f) const {
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
    return impl_->packet_type_info.get<uint16_t>(lidar_buf);
}

uint32_t packet_format::frame_id(const uint8_t* lidar_buf) const {
    return impl_->frame_id_info.get<uint32_t>(lidar_buf);
}

uint32_t packet_format::init_id(const uint8_t* lidar_buf) const {
    return impl_->init_id_info.get<uint32_t>(lidar_buf);
}

uint64_t packet_format::prod_sn(const uint8_t* lidar_buf) const {
    return impl_->prod_sn_info.get<uint64_t>(lidar_buf);
}

uint8_t packet_format::alert_flags(const uint8_t* lidar_buf) const {
    return impl_->alert_flags_info.get<uint8_t>(lidar_buf);
}

uint16_t packet_format::countdown_thermal_shutdown(
    const uint8_t* lidar_buf) const {
    return impl_->countdown_thermal_shutdown_info.get<uint16_t>(lidar_buf);
}

uint16_t packet_format::countdown_shot_limiting(
    const uint8_t* lidar_buf) const {
    return impl_->countdown_shot_limiting_info.get<uint16_t>(lidar_buf);
}

uint8_t packet_format::thermal_shutdown(const uint8_t* lidar_buf) const {
    return impl_->thermal_shutdown_info.get<uint8_t>(lidar_buf);
}

uint8_t packet_format::shot_limiting(const uint8_t* lidar_buf) const {
    return impl_->shot_limiting_info.get<uint8_t>(lidar_buf);
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
    return impl_->col_status_info.get<uint32_t>(col_buf);
}

uint64_t packet_format::col_timestamp(const uint8_t* col_buf) const {
    return impl_->col_timestamp_info.get<uint64_t>(col_buf);
}

uint16_t packet_format::col_measurement_id(const uint8_t* col_buf) const {
    return impl_->col_measurement_id_info.get<uint16_t>(col_buf);
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

uint64_t packet_format::field_value_mask(const std::string& i) const {
    const auto& f = impl_->fields.at(i);
    return get_value_mask(f);
}

int packet_format::field_bitness(const std::string& i) const {
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
    impl_->col_status_info.set(col_buf, status);
}

void packet_writer::set_col_timestamp(uint8_t* col_buf, uint64_t ts) const {
    impl_->col_timestamp_info.set(col_buf, ts);
}

void packet_writer::set_col_measurement_id(uint8_t* col_buf,
                                           uint16_t m_id) const {
    impl_->col_measurement_id_info.set(col_buf, m_id);
}

void packet_writer::set_frame_id(uint8_t* lidar_buf, uint32_t frame_id) const {
    impl_->frame_id_info.set(lidar_buf, frame_id);
}

void packet_writer::set_init_id(uint8_t* lidar_buf, uint32_t init_id) const {
    impl_->init_id_info.set(lidar_buf, init_id);
}

void packet_writer::set_packet_type(uint8_t* lidar_buf,
                                    uint16_t packet_type) const {
    impl_->packet_type_info.set(lidar_buf, packet_type);
}

void packet_writer::set_prod_sn(uint8_t* lidar_buf, uint64_t sn) const {
    impl_->prod_sn_info.set(lidar_buf, sn);
}

void packet_writer::set_alert_flags(uint8_t* lidar_buf,
                                    uint8_t alert_flags) const {
    impl_->alert_flags_info.set(lidar_buf, alert_flags);
}

void packet_writer::set_shutdown(uint8_t* lidar_buf, uint8_t status) const {
    impl_->thermal_shutdown_info.set(lidar_buf, status);
}

void packet_writer::set_shot_limiting(uint8_t* lidar_buf,
                                      uint8_t status) const {
    impl_->shot_limiting_info.set(lidar_buf, status);
}

void packet_writer::set_shutdown_countdown(uint8_t* lidar_buf,
                                           uint8_t shutdown_countdown) const {
    impl_->countdown_thermal_shutdown_info.set(lidar_buf, shutdown_countdown);
}

void packet_writer::set_shot_limiting_countdown(
    uint8_t* lidar_buf, uint8_t shot_limiting_countdown) const {
    impl_->countdown_shot_limiting_info.set(lidar_buf, shot_limiting_countdown);
}

template <typename T>
void packet_writer::set_block(Eigen::Ref<const img_t<T>> field,
                              const std::string& chan,
                              uint8_t* lidar_buf) const {
    constexpr int N = 32;
    if (columns_per_packet > N)
        throw std::runtime_error("Recompile set_block_impl with larger N");

    impl::FieldInfo f = impl_->fields.at(chan);

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

            f.set(px_dst, *(data + f_offset + x));
        }
    }
}

template void packet_writer::set_block(Eigen::Ref<const img_t<uint8_t>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<uint16_t>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<uint32_t>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<uint64_t>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<int8_t>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<int16_t>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<int32_t>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<int64_t>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<float>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;
template void packet_writer::set_block(Eigen::Ref<const img_t<double>> field,
                                       const std::string& i,
                                       uint8_t* lidar_buf) const;

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
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<int8_t>> field, uint8_t* lidar_buf) const;
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<int16_t>> field, uint8_t* lidar_buf) const;
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<int32_t>> field, uint8_t* lidar_buf) const;
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<int64_t>> field, uint8_t* lidar_buf) const;
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<float>> field, uint8_t* lidar_buf) const;
template void packet_writer::unpack_raw_headers(
    Eigen::Ref<const img_t<double>> field, uint8_t* lidar_buf) const;

static std::array<uint64_t, 256> crc64_init(void) {
    // Generate LUT of all possible 8-bit CRCs to speed up CRC calculation
    // This is for the ECMA-182 CRC64 implementation used on the sensor.
    constexpr uint64_t poly = 0xC96C5795D7870F42;
    std::array<uint64_t, 256> arr = {0};
    for (uint32_t i = 0; i < 256; ++i) {
        uint64_t r = i;
        for (uint32_t j = 0; j < 8; ++j) {
            r = (r >> 1) ^ (poly & ~((r & 1) - 1));
        }
        arr[i] = r;
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

optional<uint64_t> packet_format::crc(const uint8_t* lidar_buf) const {
    if (udp_profile_lidar == UDPProfileLidar::PROFILE_LIDAR_LEGACY ||
        udp_profile_lidar ==
            UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL) {
        return optional<uint64_t>();
    }

    return *(uint64_t*)&lidar_buf[lidar_packet_size - 8];
}

uint64_t packet_format::calculate_crc(const uint8_t* lidar_buf) const {
    return impl::crc64_compute(lidar_buf, lidar_packet_size - 8);
}

}  // namespace sensor
}  // namespace ouster

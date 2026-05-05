/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>
#include <string>

#include "ouster/deprecation.h"
#include "ouster/types.h"
#include "ouster/visibility.h"
#include "ouster/zone_state.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

/**
 * Writing counterpart to PacketFormat, used for packet generation
 */
class OUSTER_API_CLASS PacketWriter : public PacketFormat {
   public:
    using PacketFormat::PacketFormat;
    // construct from packet format
    OUSTER_API_FUNCTION
    PacketWriter(const PacketFormat& pf);

    OUSTER_API_FUNCTION
    uint8_t* nth_col(int n, uint8_t* lidar_buf) const;
    OUSTER_API_FUNCTION
    uint8_t* nth_px(int n, uint8_t* col_buf) const;
    OUSTER_API_FUNCTION
    uint8_t* footer(uint8_t* lidar_buf) const;
    OUSTER_API_FUNCTION
    uint8_t* imu_nth_measurement(int n, uint8_t* imu_buf) const;
    OUSTER_API_FUNCTION
    uint8_t* zone_nth_measurement(int n, uint8_t* zone_buf) const;

    OUSTER_API_FUNCTION
    void set_alert_flags(uint8_t* lidar_buf, uint8_t alert_flags) const;
    OUSTER_API_FUNCTION
    void set_col_status(uint8_t* col_buf, uint32_t status) const;
    OUSTER_API_FUNCTION
    void set_col_timestamp(uint8_t* col_buf, uint64_t ts) const;
    OUSTER_API_FUNCTION
    void set_col_measurement_id(uint8_t* col_buf, uint16_t m_id) const;
    OUSTER_API_FUNCTION
    void set_frame_id(uint8_t* lidar_buf, uint32_t frame_id) const;
    OUSTER_API_FUNCTION
    void set_init_id(uint8_t* lidar_buf, uint32_t init_id) const;
    OUSTER_API_FUNCTION
    void set_packet_type(uint8_t* packet_buf, uint16_t packet_type) const;
    OUSTER_API_FUNCTION
    void set_prod_sn(uint8_t* lidar_buf, uint64_t sn) const;
    OUSTER_API_FUNCTION
    void set_shot_limiting(uint8_t* lidar_buf, uint8_t status) const;
    OUSTER_API_FUNCTION
    void set_shot_limiting_countdown(uint8_t* lidar_buf,
                                     uint8_t shot_limiting_countdown) const;
    OUSTER_API_FUNCTION
    void set_shutdown(uint8_t* lidar_buf, uint8_t status) const;
    OUSTER_API_FUNCTION
    void set_shutdown_countdown(uint8_t* lidar_buf,
                                uint8_t shutdown_countdown) const;

    /**
     * Set the specified channel field into a packet measurement block.
     * @tparam T The field type.
     * @param[in] data source array containing field data to write.
     * @param[in] cols number of columns in the source array.
     * @param[in] field_name the name of the channel field to set.
     * @param[in,out] lidar_buf the lidar buffer.
     */
    template <typename T>
    OUSTER_API_FUNCTION void set_block(const T* data, int cols,
                                       const std::string& field_name,
                                       uint8_t* lidar_buf) const;

    /**
     * Unpack the RAW_HEADERS field from a scan into the lidar packet buffer.
     * Note: measurement_id indices will be taken from lidar_buf itself,
     * so this method expects those to be pre-filled
     * @tparam T The field type.
     * @param[in] field source eigen array
     * @param[in,out] lidar_buf the lidar buffer.
     */
    template <typename T>
    OUSTER_API_FUNCTION void unpack_raw_headers(
        Eigen::Ref<const img_t<T>> field, uint8_t* lidar_buf) const;

    OUSTER_API_FUNCTION
    void set_imu_nmea_ts(uint8_t* imu_buf, uint64_t timestamp) const;
    OUSTER_API_FUNCTION
    void set_imu_nmea_sentence(uint8_t* imu_buf,
                               const std::string& sentence) const;
    // will copy 85 bytes, beware.
    OUSTER_API_FUNCTION
    void set_imu_nmea_sentence(uint8_t* imu_buf, const char* ptr) const;

    OUSTER_API_FUNCTION
    void set_imu_la_x(uint8_t* imu_buf, float la_x) const;
    OUSTER_API_FUNCTION
    void set_imu_la_y(uint8_t* imu_buf, float la_y) const;
    OUSTER_API_FUNCTION
    void set_imu_la_z(uint8_t* imu_buf, float la_z) const;
    OUSTER_API_FUNCTION
    void set_imu_av_x(uint8_t* imu_buf, float av_x) const;
    OUSTER_API_FUNCTION
    void set_imu_av_y(uint8_t* imu_buf, float av_y) const;
    OUSTER_API_FUNCTION
    void set_imu_av_z(uint8_t* imu_buf, float av_z) const;

    OUSTER_API_FUNCTION
    void set_zone_timestamp(uint8_t* zone_buf, uint64_t timestamp) const;
    // will copy 32 bytes, beware
    OUSTER_API_FUNCTION
    void set_live_zoneset_hash(uint8_t* zone_buf, const uint8_t* hash) const;
    OUSTER_API_FUNCTION
    void set_zone_state(uint8_t* zone_measurement_ptr,
                        const ZoneState& zone_state) const;
};

/**
 * @deprecated Use `PacketWriter` instead.
 */
OUSTER_DEPRECATED_TYPE(packet_writer, PacketWriter,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);

// Inline definition of PacketWriter::unpack_raw_headers<T>. Defined here
// (rather than in parsing.cpp with explicit instantiations) to work around
// an Apple Clang mangling bug for the dependent default StrideType
// expression in Eigen::Ref. See the note on the declaration above.
template <typename T>
inline void PacketWriter::unpack_raw_headers(Eigen::Ref<const img_t<T>> field,
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

    for (uint32_t icol = 0; icol < columns_per_packet; ++icol) {
        uint8_t* col_buf = nth_col(icol, lidar_buf);
        uint8_t* colf_ptr = col_buf + col_size - col_footer_size;

        ColMajorView colh_view(reinterpret_cast<T*>(col_buf), ch_size);
        ColMajorView colf_view(reinterpret_cast<T*>(colf_ptr), cf_size);

        m_id = col_measurement_id(col_buf);

        colh_view = field.block(ch_offset, m_id, ch_size, 1);
        colf_view = field.block(cf_offset, m_id, cf_size, 1);
    }
}

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster

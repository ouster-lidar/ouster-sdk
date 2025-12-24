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

    template <typename T>
    void set_block(Eigen::Ref<const img_t<T>> field, const std::string& i,
                   uint8_t* lidar_buf) const;

    /**
     * Note: measurement_id indices will be taken from lidar_buf itself,
     * so this method expects those to be pre-filled
     */
    template <typename T>
    void unpack_raw_headers(Eigen::Ref<const img_t<T>> field,
                            uint8_t* lidar_buf) const;

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

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster

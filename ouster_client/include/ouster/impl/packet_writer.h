/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include "ouster/types.h"

namespace ouster {
namespace sensor {
namespace impl {

/**
 * Writing counterpart to packet_format, used for packet generation
 */
class packet_writer : public packet_format {
   public:
    using packet_format::packet_format;
    // construct from packet format
    packet_writer(const packet_format& pf) : packet_format(pf) {}

    uint8_t* nth_col(int n, uint8_t* lidar_buf) const;
    uint8_t* nth_px(int n, uint8_t* col_buf) const;
    uint8_t* footer(uint8_t* lidar_buf) const;

    void set_alert_flags(uint8_t* lidar_buf, uint8_t alert_flags) const;
    void set_col_status(uint8_t* col_buf, uint32_t status) const;
    void set_col_timestamp(uint8_t* col_buf, uint64_t ts) const;
    void set_col_measurement_id(uint8_t* col_buf, uint16_t m_id) const;
    void set_frame_id(uint8_t* lidar_buf, uint32_t frame_id) const;
    void set_init_id(uint8_t* lidar_buf, uint32_t init_id) const;
    void set_packet_type(uint8_t* lidar_buf, uint16_t packet_type) const;
    void set_prod_sn(uint8_t* lidar_buf, uint64_t sn) const;
    void set_shot_limiting(uint8_t* lidar_buf, uint8_t status) const;
    void set_shot_limiting_countdown(uint8_t* lidar_buf,
                                     uint8_t shot_limiting_countdown) const;
    void set_shutdown(uint8_t* lidar_buf, uint8_t status) const;
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
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

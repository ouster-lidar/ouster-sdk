/**
 * @file
 * @brief Packet parsing internals
 */

#pragma once

#include <cstdint>
#include <cstring>

#include "ouster/types.h"

namespace ouster {
namespace sensor {
namespace impl {

constexpr int pixel_bytes = 12;
constexpr int imu_packet_size = 48;
constexpr int cols_per_packet = 16;
constexpr int64_t encoder_ticks_per_rev = 90112;

constexpr int column_bytes(int n_pixels) {
    return 16 + (n_pixels * pixel_bytes) + 4;
}

constexpr int packet_bytes(int n_pixels) {
    return cols_per_packet * column_bytes(n_pixels);
}

template <int N_PIXELS>
const uint8_t* nth_col(int n, const uint8_t* lidar_buf) {
    return lidar_buf + (n * column_bytes(N_PIXELS));
}

template <int N_PIXELS>
inline uint32_t col_status(const uint8_t* col_buf) {
    uint32_t res;
    std::memcpy(&res, col_buf + column_bytes(N_PIXELS) - 4, sizeof(uint32_t));
    return res;
}

inline uint64_t col_timestamp(const uint8_t* col_buf) {
    uint64_t res;
    std::memcpy(&res, col_buf, sizeof(uint64_t));
    return res;  // nanoseconds
}

inline uint32_t col_encoder(const uint8_t* col_buf) {
    uint32_t res;
    std::memcpy(&res, col_buf + 12, sizeof(uint32_t));
    return res;
}

inline uint16_t col_measurement_id(const uint8_t* col_buf) {
    uint16_t res;
    std::memcpy(&res, col_buf + 8, sizeof(uint16_t));
    return res;
}

inline uint16_t col_frame_id(const uint8_t* col_buf) {
    uint16_t res;
    std::memcpy(&res, col_buf + 10, sizeof(uint16_t));
    return res;
}

inline const uint8_t* nth_px(int n, const uint8_t* col_buf) {
    return col_buf + 16 + (n * pixel_bytes);
}

inline uint32_t px_range(const uint8_t* px_buf) {
    uint32_t res;
    std::memcpy(&res, px_buf, sizeof(uint32_t));
    res &= 0x000fffff;
    return res;
}

inline uint16_t px_reflectivity(const uint8_t* px_buf) {
    uint16_t res;
    std::memcpy(&res, px_buf + 4, sizeof(uint16_t));
    return res;
}

inline uint16_t px_signal(const uint8_t* px_buf) {
    uint16_t res;
    std::memcpy(&res, px_buf + 6, sizeof(uint16_t));
    return res;
}

inline uint16_t px_ambient(const uint8_t* px_buf) {
    uint16_t res;
    std::memcpy(&res, px_buf + 8, sizeof(uint16_t));
    return res;
}

inline uint64_t imu_sys_ts(const uint8_t* imu_buf) {
    uint64_t res;
    std::memcpy(&res, imu_buf, sizeof(uint64_t));
    return res;
}

inline uint64_t imu_accel_ts(const uint8_t* imu_buf) {
    uint64_t res;
    std::memcpy(&res, imu_buf + 8, sizeof(uint64_t));
    return res;
}

inline uint64_t imu_gyro_ts(const uint8_t* imu_buf) {
    uint64_t res;
    std::memcpy(&res, imu_buf + 16, sizeof(uint64_t));
    return res;
}

inline float imu_la_x(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 24, sizeof(float));
    return res;
}

inline float imu_la_y(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 28, sizeof(float));
    return res;
}

inline float imu_la_z(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 32, sizeof(float));
    return res;
}

inline float imu_av_x(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 36, sizeof(float));
    return res;
}

inline float imu_av_y(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 40, sizeof(float));
    return res;
}

inline float imu_av_z(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 44, sizeof(float));
    return res;
}

template <int N_PIXELS>
constexpr packet_format packet_2_0() {
    return {
        impl::packet_bytes(N_PIXELS),
        impl::imu_packet_size,
        impl::cols_per_packet,
        N_PIXELS,
        impl::encoder_ticks_per_rev,

        impl::nth_col<N_PIXELS>,
        impl::col_timestamp,
        impl::col_encoder,
        impl::col_measurement_id,
        impl::col_frame_id,
        impl::col_status<N_PIXELS>,

        impl::nth_px,
        impl::px_range,
        impl::px_reflectivity,
        impl::px_signal,
        impl::px_ambient,

        impl::imu_sys_ts,
        impl::imu_accel_ts,
        impl::imu_gyro_ts,
        impl::imu_la_x,
        impl::imu_la_y,
        impl::imu_la_z,
        impl::imu_av_x,
        impl::imu_av_y,
        impl::imu_av_z,
    };
}

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

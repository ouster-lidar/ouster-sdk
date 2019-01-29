/**
 * @file
 * @brief Utilities to parse lidar and imu packets
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace ouster {
namespace OS1 {

const int pixels_per_column = 64;
const int columns_per_buffer = 16;

const int pixel_bytes = 12;
const int column_bytes = 16 + (pixels_per_column * pixel_bytes) + 4;

const int encoder_ticks_per_rev = 90112;

// lidar column fields
inline const uint8_t* nth_col(int n, const uint8_t* udp_buf) {
    return udp_buf + (n * column_bytes);
}

inline uint64_t col_timestamp(const uint8_t* col_buf) {
    uint64_t res;
    memcpy(&res, col_buf, sizeof(uint64_t));
    return res;
}

inline float col_h_angle(const uint8_t* col_buf) {
    uint32_t ticks;
    memcpy(&ticks, col_buf + 12, sizeof(uint32_t));
    return (2.0 * M_PI * ticks / (float)encoder_ticks_per_rev);
}

inline uint32_t col_h_encoder_count(const uint8_t* col_buf) {
    uint32_t res;
    std::memcpy(&res, col_buf + 12, sizeof(uint32_t));
    return res;
}

inline uint16_t col_measurement_id(const uint8_t* col_buf) {
    uint16_t res;
    memcpy(&res, col_buf + 8, sizeof(uint16_t));
    return res;
}

inline uint16_t col_frame_id(const uint8_t* col_buf) {
    uint16_t res;
    memcpy(&res, col_buf + 10, sizeof(uint16_t));
    return res;
}

inline uint32_t col_valid(const uint8_t* col_buf) {
    uint32_t res;
    memcpy(&res, col_buf + (16 + pixels_per_column * pixel_bytes),
           sizeof(uint32_t));
    return res;
}

// lidar pixel fields
inline const uint8_t* nth_px(int n, const uint8_t* col_buf) {
    return col_buf + 16 + (n * pixel_bytes);
}

inline uint32_t px_range(const uint8_t* px_buf) {
    uint32_t res;
    memcpy(&res, px_buf, sizeof(uint32_t));
    res &= 0x000fffff;
    return res;
}

inline uint16_t px_reflectivity(const uint8_t* px_buf) {
    uint16_t res;
    memcpy(&res, px_buf + 4, sizeof(uint16_t));
    return res;
}

inline uint16_t px_signal_photons(const uint8_t* px_buf) {
    uint16_t res;
    memcpy(&res, px_buf + 6, sizeof(uint16_t));
    return res;
}

inline uint16_t px_noise_photons(const uint8_t* px_buf) {
    uint16_t res;
    memcpy(&res, px_buf + 8, sizeof(uint16_t));
    return res;
}

// imu packets
inline uint64_t imu_sys_ts(const uint8_t* imu_buf) {
    uint64_t res;
    memcpy(&res, imu_buf, sizeof(uint64_t));
    return res;
}

inline uint64_t imu_accel_ts(const uint8_t* imu_buf) {
    uint64_t res;
    memcpy(&res, imu_buf + 8, sizeof(uint64_t));
    return res;
}

inline uint64_t imu_gyro_ts(const uint8_t* imu_buf) {
    uint64_t res;
    memcpy(&res, imu_buf + 16, sizeof(uint64_t));
    return res;
}

// imu linear acceleration
inline float imu_la_x(const uint8_t* imu_buf) {
    float res;
    memcpy(&res, imu_buf + 24, sizeof(float));
    return res;
}

inline float imu_la_y(const uint8_t* imu_buf) {
    float res;
    memcpy(&res, imu_buf + 28, sizeof(float));
    return res;
}

inline float imu_la_z(const uint8_t* imu_buf) {
    float res;
    memcpy(&res, imu_buf + 32, sizeof(float));
    return res;
}

// imu angular velocity
inline float imu_av_x(const uint8_t* imu_buf) {
    float res;
    memcpy(&res, imu_buf + 36, sizeof(float));
    return res;
}

inline float imu_av_y(const uint8_t* imu_buf) {
    float res;
    memcpy(&res, imu_buf + 40, sizeof(float));
    return res;
}

inline float imu_av_z(const uint8_t* imu_buf) {
    float res;
    memcpy(&res, imu_buf + 44, sizeof(float));
    return res;
}
}
}

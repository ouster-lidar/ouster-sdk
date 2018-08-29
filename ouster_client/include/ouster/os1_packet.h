/**
 * Utilities to parse lidar and imu packets
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace ouster {
namespace OS1 {

namespace {

const int pixels_per_column = 64;
const int columns_per_buffer = 16;

const int pixel_bytes = 12;
const int column_bytes = 16 + (pixels_per_column * pixel_bytes) + 4;

const int encoder_ticks_per_rev = 90112;

const std::array<float, OS1::pixels_per_column> beam_altitude_angles = {
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};
const std::array<float, OS1::pixels_per_column> beam_azimuth_angles = {
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};

struct trig_table_entry {
    float sin_beam_altitude_angles;
    float cos_beam_altitude_angles;
    float beam_azimuth_angles;
};

// table of vertical angle cos, sin, and horizontal offset of each pixel
static trig_table_entry trig_table[pixels_per_column];
}

static bool init_tables() {
    for (int i = 0; i < pixels_per_column; i++) {
        trig_table[i] = {sinf(beam_altitude_angles[i] * 2 * M_PI / 360.0f),
                         cosf(beam_altitude_angles[i] * 2 * M_PI / 360.0f),
                         beam_azimuth_angles[i] * 2 * (float)M_PI / 360.0f};
    }
    return true;
}

static bool tables_initialized = init_tables();

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
    memcpy(&ticks, col_buf + (3 * 4), sizeof(uint32_t));
    return (2.0 * M_PI * ticks / (float)encoder_ticks_per_rev);
}

inline float col_h_encoder_count(const uint8_t* col_buf) {
    uint32_t res;
    std::memcpy(&res, col_buf + (3 * 4), sizeof(uint32_t));
    return res;
}

inline uint32_t col_measurement_id(const uint8_t* col_buf) {
    uint32_t res;
    memcpy(&res, col_buf + (2 * 4), sizeof(uint32_t));
    return res;
}

inline uint32_t col_valid(const uint8_t* col_buf) {
    uint32_t res;
    memcpy(&res, col_buf + (196 * 4), sizeof(uint32_t));
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

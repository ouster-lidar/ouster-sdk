#pragma once

#include <cstring>

namespace ouster {
namespace OS1 {
namespace impl {

// TODO: these should be read from the packet format

constexpr int pixel_bytes = 12;
constexpr int imu_packet_size = 48;
constexpr int cols_per_packet = 16;

constexpr int64_t encoder_ticks_per_rev = 90112;

constexpr int column_bytes_1_13_0 = 16 + (64 * pixel_bytes) + 4;

constexpr int column_bytes_1_14_16 = 16 + (16 * pixel_bytes) + 4;
constexpr int column_bytes_1_14_32 = 16 + (32 * pixel_bytes) + 4;
constexpr int column_bytes_1_14_64 = column_bytes_1_13_0;
constexpr int column_bytes_1_14_128 = 16 + (128 * pixel_bytes) + 4;

constexpr int packet_bytes_1_13_0 = cols_per_packet * column_bytes_1_13_0;

constexpr int packet_bytes_1_14_16 = cols_per_packet * column_bytes_1_14_16;
constexpr int packet_bytes_1_14_32 = cols_per_packet * column_bytes_1_14_32;
constexpr int packet_bytes_1_14_64 = packet_bytes_1_13_0;
constexpr int packet_bytes_1_14_128 = cols_per_packet * column_bytes_1_14_128;

// Reading from the lidar packet.

inline const uint8_t* nth_col(int n, const uint8_t* lidar_buf) {
    return lidar_buf + (n * column_bytes_1_13_0);
}

inline const uint8_t* nth_col_16(int n, const uint8_t* lidar_buf) {
    return lidar_buf + (n * column_bytes_1_14_16);
}

inline const uint8_t* nth_col_32(int n, const uint8_t* lidar_buf) {
    return lidar_buf + (n * column_bytes_1_14_32);
}

inline const uint8_t* nth_col_64(int n, const uint8_t* lidar_buf) {
    return lidar_buf + (n * column_bytes_1_14_64);
}

inline const uint8_t* nth_col_128(int n, const uint8_t* lidar_buf) {
    return lidar_buf + (n * column_bytes_1_14_128);
}

inline uint64_t col_timestamp(const uint8_t* col_buf) {
    uint64_t res;
    std::memcpy(&res, col_buf, sizeof(uint64_t));
    return res;  // nanoseconds
}

inline uint32_t col_h_angle(const uint8_t* col_buf) {
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

inline uint32_t col_valid_16(const uint8_t* col_buf) {
    uint32_t res;
    memcpy(&res, col_buf + column_bytes_1_14_16 - 4,
           sizeof(uint32_t));
    return res;
}

inline uint32_t col_valid_32(const uint8_t* col_buf) {
    uint32_t res;
    memcpy(&res, col_buf + column_bytes_1_14_32 - 4,
           sizeof(uint32_t));
    return res;
}

inline uint32_t col_valid_64(const uint8_t* col_buf) {
    uint32_t res;
    memcpy(&res, col_buf + column_bytes_1_14_64 - 4,
           sizeof(uint32_t));
    return res;
}


inline uint32_t col_valid_128(const uint8_t* col_buf) {
    uint32_t res;
    memcpy(&res, col_buf + column_bytes_1_14_128 - 4,
           sizeof(uint32_t));
    return res;
}

inline const uint8_t* nth_px_1_0_0(int n, const uint8_t* col_buf) {
    return col_buf + 16 + ((n + 4) * pixel_bytes);
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

inline uint16_t px_signal_photons(const uint8_t* px_buf) {
    uint16_t res;
    std::memcpy(&res, px_buf + 6, sizeof(uint16_t));
    return res;
}

inline uint16_t px_noise_photons(const uint8_t* px_buf) {
    uint16_t res;
    std::memcpy(&res, px_buf + 8, sizeof(uint16_t));
    return res;
}

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

// for 1.12 imu flip

inline float imu_la_x_1_12_0(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 24, sizeof(float));
    return -res;
}

inline float imu_la_y_1_12_0(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 28, sizeof(float));
    return -res;
}

inline float imu_la_z_1_12_0(const uint8_t* imu_buf) {
    float res;
    std::memcpy(&res, imu_buf + 32, sizeof(float));
    return -res;
}

}  // namespace impl
}  // namespace OS1
}  // namespace sensors

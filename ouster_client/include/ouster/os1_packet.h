/**
 * @file
 * @brief Utilities to parse lidar and imu packets
 */

#pragma once

#include <cstdint>

#include "ouster/impl/os1_packet_impl.h"
#include "ouster/version.h"

namespace ouster {
namespace OS1 {

/**
 * A packet_format is a table of accessors and indices for extracting data
 * from imu and lidar packets.
 *
 * It is useful both to select a packet format at runtime in non-performance
 * critical code, and to generate specialized code for each packet format
 * statically. The former can be done using get_format(), while the latter can
 * be done by writing code as a template over a packet_format.
 *
 * Note:
 * Passing a packet format as a template argument requires a slight hack in
 * C++14. The argument must have external linkage, but to enable inlining of
 * function calls, we want to include the packet_format tables in a header.
 * In C++17, we could use inline variables, but in C++14 and earlier we have to
 * pass a function that returns the table.
 */
struct packet_format {
    const size_t lidar_packet_size;
    const size_t imu_packet_size;
    const int columns_per_packet;
    const int pixels_per_column;
    const int encoder_ticks_per_rev;

    const uint8_t* (*const nth_col)(int n, const uint8_t* lidar_buf);
    uint64_t (*const col_timestamp)(const uint8_t* col_buf);
    uint32_t (*const col_h_angle)(const uint8_t* col_buf);
    uint16_t (*const col_measurement_id)(const uint8_t* col_buf);
    uint16_t (*const col_frame_id)(const uint8_t* col_buf);
    uint32_t (*const col_valid)(const uint8_t* col_buf);

    const uint8_t* (*const nth_px)(int n, const uint8_t* col_buf);
    uint32_t (*const px_range)(const uint8_t* px_buf);
    uint16_t (*const px_reflectivity)(const uint8_t* px_buf);
    uint16_t (*const px_signal_photons)(const uint8_t* px_buf);
    uint16_t (*const px_noise_photons)(const uint8_t* px_buf);

    uint64_t (*const imu_sys_ts)(const uint8_t* imu_buf);
    uint64_t (*const imu_accel_ts)(const uint8_t* imu_buf);
    uint64_t (*const imu_gyro_ts)(const uint8_t* imu_buf);
    float (*const imu_la_x)(const uint8_t* imu_buf);
    float (*const imu_la_y)(const uint8_t* imu_buf);
    float (*const imu_la_z)(const uint8_t* imu_buf);
    float (*const imu_av_x)(const uint8_t* imu_buf);
    float (*const imu_av_y)(const uint8_t* imu_buf);
    float (*const imu_av_z)(const uint8_t* imu_buf);
};

constexpr packet_format packet_1_13_0 = {
    impl::packet_bytes_1_13_0,
    impl::imu_packet_size,
    impl::cols_per_packet,
    64,
    impl::encoder_ticks_per_rev,

    impl::nth_col,
    impl::col_timestamp,
    impl::col_h_angle,
    impl::col_measurement_id,
    impl::col_frame_id,
    impl::col_valid_64,

    impl::nth_px,
    impl::px_range,
    impl::px_reflectivity,
    impl::px_signal_photons,
    impl::px_noise_photons,

    impl::imu_sys_ts,
    impl::imu_accel_ts,
    impl::imu_gyro_ts,
    impl::imu_la_x_1_12_0,
    impl::imu_la_y_1_12_0,
    impl::imu_la_z_1_12_0,
    impl::imu_av_x,
    impl::imu_av_y,
    impl::imu_av_z,
};


constexpr packet_format packet_1_14_0_16 = {
    impl::packet_bytes_1_14_16,
    impl::imu_packet_size,
    impl::cols_per_packet,
    16,
    impl::encoder_ticks_per_rev,

    impl::nth_col_16,
    impl::col_timestamp,
    impl::col_h_angle,
    impl::col_measurement_id,
    impl::col_frame_id,
    impl::col_valid_16,

    impl::nth_px,
    impl::px_range,
    impl::px_reflectivity,
    impl::px_signal_photons,
    impl::px_noise_photons,

    impl::imu_sys_ts,
    impl::imu_accel_ts,
    impl::imu_gyro_ts,
    impl::imu_la_x_1_12_0,
    impl::imu_la_y_1_12_0,
    impl::imu_la_z_1_12_0,
    impl::imu_av_x,
    impl::imu_av_y,
    impl::imu_av_z,
};

constexpr packet_format packet_1_14_0_32 = {
    impl::packet_bytes_1_14_32,
    impl::imu_packet_size,
    impl::cols_per_packet,
    32,
    impl::encoder_ticks_per_rev,

    impl::nth_col_32,
    impl::col_timestamp,
    impl::col_h_angle,
    impl::col_measurement_id,
    impl::col_frame_id,
    impl::col_valid_32,

    impl::nth_px,
    impl::px_range,
    impl::px_reflectivity,
    impl::px_signal_photons,
    impl::px_noise_photons,

    impl::imu_sys_ts,
    impl::imu_accel_ts,
    impl::imu_gyro_ts,
    impl::imu_la_x_1_12_0,
    impl::imu_la_y_1_12_0,
    impl::imu_la_z_1_12_0,
    impl::imu_av_x,
    impl::imu_av_y,
    impl::imu_av_z,
};

constexpr packet_format packet_1_14_0_64 = {
    impl::packet_bytes_1_14_64,
    impl::imu_packet_size,
    impl::cols_per_packet,
    64,
    impl::encoder_ticks_per_rev,

    impl::nth_col_64,
    impl::col_timestamp,
    impl::col_h_angle,
    impl::col_measurement_id,
    impl::col_frame_id,
    impl::col_valid_64,

    impl::nth_px,
    impl::px_range,
    impl::px_reflectivity,
    impl::px_signal_photons,
    impl::px_noise_photons,

    impl::imu_sys_ts,
    impl::imu_accel_ts,
    impl::imu_gyro_ts,
    impl::imu_la_x_1_12_0,
    impl::imu_la_y_1_12_0,
    impl::imu_la_z_1_12_0,
    impl::imu_av_x,
    impl::imu_av_y,
    impl::imu_av_z,
};

constexpr packet_format packet_1_14_0_128 = {
    impl::packet_bytes_1_14_128,
    impl::imu_packet_size,
    impl::cols_per_packet,
    128,
    impl::encoder_ticks_per_rev,

    impl::nth_col_128,
    impl::col_timestamp,
    impl::col_h_angle,
    impl::col_measurement_id,
    impl::col_frame_id,
    impl::col_valid_128,

    impl::nth_px,
    impl::px_range,
    impl::px_reflectivity,
    impl::px_signal_photons,
    impl::px_noise_photons,

    impl::imu_sys_ts,
    impl::imu_accel_ts,
    impl::imu_gyro_ts,
    impl::imu_la_x_1_12_0,
    impl::imu_la_y_1_12_0,
    impl::imu_la_z_1_12_0,
    impl::imu_av_x,
    impl::imu_av_y,
    impl::imu_av_z,
};


/**
 * Due to some unfortunate restrictions in C++14 (removed in C++11), we have to
 * wrap packet_formats in trivial accessor functions in order to use them as
 * template arguments. See os1.cpp for an example.
 */
using packet_format_f = const packet_format& (*const)();

inline const packet_format& packet_1_13_0_f() { return packet_1_13_0; }
inline const packet_format& packet_1_14_0_16_f() { return packet_1_14_0_16; }
inline const packet_format& packet_1_14_0_32_f() { return packet_1_14_0_32; }
inline const packet_format& packet_1_14_0_64_f() { return packet_1_14_0_64; }
inline const packet_format& packet_1_14_0_128_f() { return packet_1_14_0_128; }

}
}

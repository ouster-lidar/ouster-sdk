/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster data format information
 */

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/** Profile indicating packet format of lidar data. */
namespace UDPProfileLidar {
/** Unknown lidar profile, typically used as a fallback. */
static constexpr const char* UNKNOWN = "UNKNOWN";

/** Legacy lidar data */
static constexpr const char* LEGACY = "LEGACY";

/** Dual Returns data */
static constexpr const char* RNG19_RFL8_SIG16_NIR16_DUAL = "RNG19_RFL8_SIG16_NIR16_DUAL";

/** Single Returns data */
static constexpr const char* RNG19_RFL8_SIG16_NIR16 = "RNG19_RFL8_SIG16_NIR16";

/** Single Returns Low Data Rate */
static constexpr const char* RNG15_RFL8_NIR8 = "RNG15_RFL8_NIR8";

/** Five Word Profile */
static constexpr const char* FIVE_WORD_PIXEL = "FIVE_WORD_PIXEL";

/** Legacy Dual Returns low data profile */
static constexpr const char* FUSA_RNG15_RFL8_NIR8_DUAL = "FUSA_RNG15_RFL8_NIR8_DUAL";

/** Dual Returns low data profile */
static constexpr const char* RNG15_RFL8_NIR8_DUAL = "RNG15_RFL8_NIR8_DUAL";

/** Single Return Low Data Rate Zone Monitoring */
static constexpr const char* RNG15_RFL8_NIR8_ZONE16 = "RNG15_RFL8_NIR8_ZONE16";

/** Single Return Zone Monitoring */
static constexpr const char* RNG19_RFL8_SIG16_NIR16_ZONE16 = "RNG19_RFL8_SIG16_NIR16_ZONE16";

/** Single Return Low Data Rate Window Status */
static constexpr const char* RNG15_RFL8_WIN8 = "RNG15_RFL8_WIN8";

/** Dual Return Zone Monitoring */
static constexpr const char* RNG19_RFL8_SIG16_ZONE16_DUAL = "RNG19_RFL8_SIG16_ZONE16_DUAL";

/** Single Return RGB */
static constexpr const char* RNG19_RFL8_SIG16_NIR16_RGB16 = "RNG19_RFL8_SIG16_NIR16_RGB16";

/** Dual Return RGB */
static constexpr const char* RNG19_RFL8_SIG16_NIR16_RGB16_DUAL =
    "RNG19_RFL8_SIG16_NIR16_RGB16_DUAL";

/** disabled */
static constexpr const char* OFF = "OFF";
};  // namespace UDPProfileLidar

/** Profile indicating packet format of IMU data. */
enum class UDPProfileIMU {
    /** Legacy IMU data */
    LEGACY = 0,

    /** Accelerometer and gyroscope data with NMEA sentences. */
    ACCEL32_GYRO32_NMEA = 1,

    /** disabled */
    OFF = 100,
};

/**
 * Profile indicating packet header layout of Lidar and IMU packets.
 *
 * LEGACY profiles of Lidar or IMU packets ignore this setting.
 */
enum class HeaderType {
    /** Standard eUDP headers */
    STANDARD = 0,

    /** FUSA headers */
    FUSA = 1,
};

/**
 * Convenience type alias for column windows, the window over which the
 * sensor fires in columns.
 */
using ColumnWindow = std::pair<int, int>;

/** Stores data format information. */
struct OUSTER_API_CLASS DataFormat {
    uint32_t pixels_per_column;            ///< pixels per column
    uint32_t columns_per_packet;           ///< columns per packet
    uint32_t columns_per_frame;            ///< columns per frame, should match with lidar mode
    uint32_t imu_measurements_per_packet;  ///< imu measurements per packet
    uint32_t imu_packets_per_frame;        ///< imu packets per frame
    std::vector<int> pixel_shift_by_row;   ///< shift of pixels by row to enable destagger
    ColumnWindow column_window;            ///< window of columns over which sensor fires
    std::string udp_profile_lidar{};       ///< profile of lidar packet
    UDPProfileIMU udp_profile_imu{};       ///< profile of imu packet
    HeaderType header_type{};              ///< profile of lidar/imu headers
    uint16_t fps;                          ///< frames per second
    bool imu_enabled{false};               ///< if yes, imu stream is on
    bool zone_monitoring_enabled{false};   ///< if yes, zone monitoring is on

    /// Return the number of valid columns per complete frame of data with the
    /// column_window applied.
    /// @return the number of columns
    OUSTER_API_FUNCTION
    int valid_columns_per_frame() const;

    /// Return the number of valid packets actually sent per frame of data
    /// with the column_window applied.
    /// @return the number of packets
    OUSTER_API_FUNCTION
    int lidar_packets_per_frame() const;

    /// Return the maximum frame id for this data format, based on the header type and lidar
    /// profile.
    /// @return the maximum frame id
    OUSTER_API_FUNCTION
    uint32_t max_frame_id() const;
};

/**
 * Equality for DataFormat.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const DataFormat& lhs, const DataFormat& rhs);

/**
 * Not-Equality for DataFormat.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const DataFormat& lhs, const DataFormat& rhs);

/**
 * Get string representation of an IMU profile.
 *
 * @param[in] profile The profile to get the string representation of.
 *
 * @return string representation of the lidar profile.
 */
OUSTER_API_FUNCTION
std::string to_string(UDPProfileIMU profile);

/**
 * Get imu profile from string
 *
 * @param[in] str_val The string to decode into an imu profile.
 *
 * @return imu profile corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
nonstd::optional<UDPProfileIMU> udp_profile_imu_of_string(const std::string& str_val);

/**
 * Get string representation of a header layout profile.
 *
 * @param[in] profile The profile to get the string representation of.
 *
 * @return string representation of the header layout profile.
 */
OUSTER_API_FUNCTION
std::string to_string(HeaderType profile);

/**
 * Get header layout profile from string
 *
 * @param[in] str_val The string to decode into an imu profile.
 *
 * @return header layout profile corresponding to the string, or nullopt on
 *         error.
 */
OUSTER_API_FUNCTION
nonstd::optional<HeaderType> udp_profile_type_of_string(const std::string& str_val);

}  // namespace core
}  // namespace sdk
}  // namespace ouster

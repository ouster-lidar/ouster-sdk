/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster client datatypes and constants
 */

#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/chanfield.h"
#include "ouster/deprecation.h"
#include "ouster/typedefs.h"
#include "ouster/visibility.h"
#include "ouster/zone_monitor.h"
#include "version.h"

namespace ouster {
namespace sdk {
namespace core {

using nonstd::optional;

/** Forward declaration for Field */
class Field;

/** Unit of range from sensor packet, in meters. */
constexpr double RANGE_UNIT = 0.001;
/**
 * @deprecated Use RANGE_UNIT
 */
OUSTER_DEPRECATED_CONSTEXP(range_unit, RANGE_UNIT,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

/** Design values for altitude and azimuth offset angles for gen1 sensors. */
extern const std::vector<double> GEN1_ALTITUDE_ANGLES;
/** Design values for altitude and azimuth offset angles for gen1 sensors. */
extern const std::vector<double> GEN1_AZIMUTH_ANGLES;

/** Design values for imu and lidar to sensor-frame transforms. */
extern const mat4d DEFAULT_IMU_TO_SENSOR_TRANSFORM;

/** Design values for imu and lidar to sensor-frame transforms. */
extern const mat4d DEFAULT_LIDAR_TO_SENSOR_TRANSFORM;

/**
 * Constants used for configuration. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum class LidarMode {
    UNSPECIFIED = 0,  ///< lidar mode: unspecified
    _512x10,          ///< lidar mode: 10 scans of 512 columns per second
    _512x20,          ///< lidar mode: 20 scans of 512 columns per second
    _1024x10,         ///< lidar mode: 10 scans of 1024 columns per second
    _1024x20,         ///< lidar mode: 20 scans of 1024 columns per second
    _2048x10,         ///< lidar mode: 10 scans of 2048 columns per second
    _4096x5,          ///< lidar mode: 5 scans of 4096 columns per second.
                      ///< Only available on select sensors

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::LidarMode::UNSPECIFIED instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MODE_UNSPEC, UNSPECIFIED,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::LidarMode::_512x10 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MODE_512x10, _512x10,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::LidarMode::_512x20 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MODE_512x20, _512x20,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::LidarMode::_1024x10 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MODE_1024x10, _1024x10,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::LidarMode::_1024x20 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MODE_1024x20, _1024x20,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::LidarMode::_2048x10 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MODE_2048x10, _2048x10,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::LidarMode::_4096x5 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MODE_4096x5, _4096x5,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/**
 * @deprecated Use ouster::sdk::core::LidarMode instead.
 */
OUSTER_DEPRECATED_TYPE(lidar_mode, LidarMode,                   // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

/**
 * Mode controlling timestamp method. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum class TimestampMode {
    /**
     * Timestamp mode unspecified.
     */
    UNSPECIFIED = 0,

    /**
     * Use the internal clock.
     */
    TIME_FROM_INTERNAL_OSC,

    /**
     * A free running counter synced to the SYNC_PULSE_IN input
     * counts seconds (# of pulses) and nanoseconds since sensor turn
     * on.
     */
    TIME_FROM_SYNC_PULSE_IN,

    /** Synchronize with an external PTP master. */
    TIME_FROM_PTP_1588,

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::TimestampMode::UNSPECIFIED instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(TIME_FROM_UNSPEC, UNSPECIFIED,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/**
 * @deprecated Use ouster::sdk::core::TimestampMode instead.
 */
OUSTER_DEPRECATED_TYPE(timestamp_mode, TimestampMode,           // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

/**
 * Mode controlling sensor operation. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum class OperatingMode {
    UNSPECIFIED = 0,  ///< Unspecified sensor operation
    NORMAL,           ///< Normal sensor operation
    STANDBY,          ///< Standby

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::OperatingMode::UNSPECIFIED instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(OPERATING_UNSPEC, UNSPECIFIED,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::OperatingMode::NORMAL instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(OPERATING_NORMAL, NORMAL,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::OperatingMode::STANDBY instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(OPERATING_STANDBY, STANDBY,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/**
 * Mode controlling ways to input timesync information. Refer to the sensor
 * documentation for the meaning of each option.
 */
enum class MultipurposeIOMode {

    OFF = 1,  ///< Multipurpose IO is turned off (default)

    /**
     * Used in conjunction with timestamp_mode::TIME_FROM_SYNC_PULSE_IN
     * to enable time pulses in on the multipurpose io input.
     */
    INPUT_NMEA_UART,

    /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * the internal clock.
     */
    OUTPUT_FROM_INTERNAL_OSC,

    /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * a SYNC_PULSE_IN provided to the unit.
     */
    OUTPUT_FROM_SYNC_PULSE_IN,

    /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * an external PTP IEEE 1588 master.
     */
    OUTPUT_FROM_PTP_1588,

    /**
     * Output a SYNC_PULSE_OUT signal with a user defined
     * rate in an integer number of degrees.
     */
    OUTPUT_FROM_ENCODER_ANGLE,

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::MultipurposeIOMode::OFF instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MULTIPURPOSE_OFF, OFF,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::MultipurposeIOMode::INPUT_NMEA_UART
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MULTIPURPOSE_INPUT_NMEA_UART,
                                       INPUT_NMEA_UART,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::MultipurposeIOMode::OUTPUT_FROM_INTERNAL_OSC
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC,
                                       OUTPUT_FROM_INTERNAL_OSC,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::MultipurposeIOMode::OUTPUT_FROM_SYNC_PULSE_IN
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN,
                                       OUTPUT_FROM_SYNC_PULSE_IN,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::MultipurposeIOMode::OUTPUT_FROM_PTP_1588 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MULTIPURPOSE_OUTPUT_FROM_PTP_1588,
                                       OUTPUT_FROM_PTP_1588,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::MultipurposeIOMode::OUTPUT_FROM_ENCODER_ANGLE
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE,
                                       OUTPUT_FROM_ENCODER_ANGLE,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/**
 * Polarity represents polarity of NMEA UART and SYNC_PULSE inputs and outputs.
 * See sensor docs for more details.
 */
enum class Polarity {
    ACTIVE_LOW = 1,  ///< ACTIVE_LOW
    ACTIVE_HIGH,     ///< ACTIVE_HIGH

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::Polarity::ACTIVE_LOW instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(POLARITY_ACTIVE_LOW, ACTIVE_LOW,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::Polarity::ACTIVE_HIGH instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(POLARITY_ACTIVE_HIGH, ACTIVE_HIGH,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

#if defined(_WIN32)
#if defined(BAUD_9600)
/**
 * @note On windows platforms, the windows headers do a global define on
 * BAUD_9600 which causes issues with defines in types.h. Netcompat.h must
 * be included after every other header file to avoid this issue. This #error
 * is included to notify people of the issue.
 */
#undef BAUD_9600
#endif
#if defined(BAUD_115200)
/**
 * @note On windows platforms, the windows headers do a global define on
 * BAUD_115200 which causes issues with defines in types.h. Netcompat.h must
 * be included after every other header file to avoid this issue. This #error
 * is included to notify people of the issue.
 */
#undef BAUD_115200
#endif
#endif

/**
 * Baud rate the sensor attempts for NMEA UART input $GPRMC messages
 * See sensor docs for more details.
 */
enum class NMEABaudRate {
    BAUD_9600 = 1,  ///< 9600 bits per second UART baud rate
    BAUD_115200,    ///< 115200 bits per second UART baud rate
};

/** Profile indicating packet format of lidar data. */
enum class UDPProfileLidar {
    /** Unknown lidar profile, typically used as a fallback. */
    UNKNOWN = 0,

    /** Legacy lidar data */
    LEGACY,

    /** Dual Returns data */
    RNG19_RFL8_SIG16_NIR16_DUAL,

    /** Single Returns data */
    RNG19_RFL8_SIG16_NIR16,

    /** Single Returns Low Data Rate */
    RNG15_RFL8_NIR8,

    /** Five Word Profile */
    FIVE_WORD_PIXEL,

    /** Legacy Dual Returns low data profile */
    FUSA_RNG15_RFL8_NIR8_DUAL,

    /** Dual Returns low data profile */
    RNG15_RFL8_NIR8_DUAL,

    /** Single Return Low Data Rate Zone Monitoring */
    RNG15_RFL8_NIR8_ZONE16,

    /** Single Return Zone Monitoring */
    RNG19_RFL8_SIG16_NIR16_ZONE16,

    /** disabled */
    OFF = 100,

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::UDPProfileLidar::UNKNOWN instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_LIDAR_UNKNOWN, UNKNOWN,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::UDPProfileLidar::LEGACY instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_LIDAR_LEGACY, LEGACY,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
                                       RNG19_RFL8_SIG16_NIR16_DUAL,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_RNG19_RFL8_SIG16_NIR16,
                                       RNG19_RFL8_SIG16_NIR16,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::UDPProfileLidar::RNG15_RFL8_NIR8
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_RNG15_RFL8_NIR8, RNG15_RFL8_NIR8,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::UDPProfileLidar::FIVE_WORD_PIXEL
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_FIVE_WORD_PIXEL, FIVE_WORD_PIXEL,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL,
                                       FUSA_RNG15_RFL8_NIR8_DUAL,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::UDPProfileLidar::RNG15_RFL8_NIR8_DUAL instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_RNG15_RFL8_NIR8_DUAL,
                                       RNG15_RFL8_NIR8_DUAL,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::UDPProfileLidar::RNG15_RFL8_NIR8_ZONE16 instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_RNG15_RFL8_NIR8_ZONE16,
                                       RNG15_RFL8_NIR8_ZONE16,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_ZONE16
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_RNG19_RFL8_SIG16_NIR16_ZONE16,
                                       RNG19_RFL8_SIG16_NIR16_ZONE16,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::UDPProfileLidar::OFF instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_LIDAR_OFF, OFF,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/** Profile indicating packet format of IMU data. */
enum class UDPProfileIMU {
    /** NOTE[UN]: Why no UNKNOWN? Should we add one? */

    /** Legacy IMU data */
    LEGACY = 1,

    /** Accelerometer and gyroscope data with NMEA sentences. */
    ACCEL32_GYRO32_NMEA = 2,

    /** disabled */
    OFF = 100,

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::UDPProfileIMU::LEGACY instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_IMU_LEGACY, LEGACY,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::UDPProfileIMU::ACCEL32_GYRO32_NMEA
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_ACCEL32_GYRO32_NMEA,
                                       ACCEL32_GYRO32_NMEA,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::UDPProfileIMU::OFF instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(PROFILE_IMU_OFF, OFF,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/**
 * Profile indicating packet header layout of Lidar and IMU packets.
 *
 * LEGACY profiles of Lidar or IMU packets ignore this setting.
 */
enum class HeaderType {
    /** Standard eUDP headers */
    STANDARD = 1,

    /** FUSA headers */
    FUSA = 2,
};

/** Full scale range for IMU data. */
enum class FullScaleRange {
    /** Higher precision lower range measurement mode */
    NORMAL = 0,

    /** Lower precision higher range measurement mode */
    EXTENDED,

    // Deprecated entries
    /// @deprecated Use ouster::sdk::core::FullScaleRange::NORMAL instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(FSR_NORMAL, NORMAL,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    /// @deprecated Use ouster::sdk::core::FullScaleRange::EXTENDED instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(FSR_EXTENDED, EXTENDED,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/** Priority of returns for the lidar to output.
 *   Lidar can have more than 1 or 2 detected "returns".
 *   This indicates to the lidar which ones it should output.
 *   See sensor docs for more details.
 */
enum class ReturnOrder {
    /** Lidar returns the strongest returns first */
    STRONGEST_TO_WEAKEST = 0,

    /** Lidar returns the furthest returns first */
    FARTHEST_TO_NEAREST,

    /** Lidar returns the nearest returns first */
    NEAREST_TO_FARTHEST,

    /** DEPRECATED: Only Present In Old Test Firmware */
    DEPRECATED_STRONGEST_RETURN_FIRST,

    /** DEPRECATED: Only Present In Old Test Firmware */
    DEPRECATED_LAST_RETURN_FIRST,

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::ReturnOrder::STRONGEST_TO_WEAKEST
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(ORDER_STRONGEST_TO_WEAKEST,
                                       STRONGEST_TO_WEAKEST,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ReturnOrder::FARTHEST_TO_NEAREST
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(ORDER_FARTHEST_TO_NEAREST,
                                       FARTHEST_TO_NEAREST,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ReturnOrder::NEAREST_TO_FARTHEST
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(ORDER_NEAREST_TO_FARTHEST,
                                       NEAREST_TO_FARTHEST,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::ReturnOrder::DEPRECATED_STRONGEST_RETURN_FIRST
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(ORDER_DEPRECATED_STRONGEST_RETURN_FIRST,
                                       DEPRECATED_STRONGEST_RETURN_FIRST,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use
    ///< ouster::sdk::core::ReturnOrder::DEPRECATED_LAST_RETURN_FIRST instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(ORDER_DEPRECATED_LAST_RETURN_FIRST,
                                       DEPRECATED_LAST_RETURN_FIRST,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/** Thermal Shutdown status. */
enum class ThermalShutdownStatus : uint8_t {
    NORMAL = 0x00,    ///< Normal operation
    IMMINENT = 0x01,  ///< Thermal Shutdown imminent

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::ThermalShutdownStatus::NORMAL
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(THERMAL_SHUTDOWN_NORMAL, NORMAL,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ThermalShutdownStatus::IMMINENT
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(THERMAL_SHUTDOWN_IMMINENT, IMMINENT,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/** Shot Limiting status. */
enum class ShotLimitingStatus : uint8_t {
    NORMAL = 0x00,           ///< Normal operation
    IMMINENT = 0x01,         ///< Shot limiting imminent
    REDUCTION_0_10 = 0x02,   ///< Shot limiting reduction by 0 to 10%
    REDUCTION_10_20 = 0x03,  ///< Shot limiting reduction by 10 to 20%
    REDUCTION_20_30 = 0x04,  ///< Shot limiting reduction by 20 to 30%
    REDUCTION_30_40 = 0x05,  ///< Shot limiting reduction by 30 to 40%
    REDUCTION_40_50 = 0x06,  ///< Shot limiting reduction by 40 to 50%
    REDUCTION_50_60 = 0x07,  ///< Shot limiting reduction by 50 to 60%
    REDUCTION_60_70 = 0x08,  ///< Shot limiting reduction by 60 to 70%
    REDUCTION_70_75 = 0x09,  ///< Shot limiting reduction by 70 to 80%

    // Deprecated entries
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::NORMAL instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_NORMAL, NORMAL,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::IMMINENT
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_IMMINENT, IMMINENT,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::REDUCTION_0_10
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_REDUCTION_0_10,
                                       REDUCTION_0_10,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::REDUCTION_10_20
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_REDUCTION_10_20,
                                       REDUCTION_10_20,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::REDUCTION_20_30
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_REDUCTION_20_30,
                                       REDUCTION_20_30,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::REDUCTION_30_40
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_REDUCTION_30_40,
                                       REDUCTION_30_40,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::REDUCTION_40_50
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_REDUCTION_40_50,
                                       REDUCTION_40_50,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::REDUCTION_50_60
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_REDUCTION_50_60,
                                       REDUCTION_50_60,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::REDUCTION_60_70
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_REDUCTION_60_70,
                                       REDUCTION_60_70,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< @deprecated Use ouster::sdk::core::ShotLimitingStatus::REDUCTION_70_75
    ///< instead.
    OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(SHOT_LIMITING_REDUCTION_70_75,
                                       REDUCTION_70_75,
                                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16),
    ///< end of deprecated entries
};

/** Bloom reduction optimization settings. */
enum class BloomReductionOptimization {
    /// Balanced optimization
    BALANCED = 0,
    /// Optimization that minimizes false positives
    MINIMIZE_FALSE_POSITIVES = 1,
};

/**
 * Convenience type alias for azimuth windows, the window over which the
 * sensor fires in millidegrees.
 */
using AzimuthWindow = std::pair<unsigned int, unsigned int>;
/**
 * Convenience type alias for column windows, the window over which the
 * sensor fires in columns.
 */
using ColumnWindow = std::pair<int, int>;

/**
 * Struct for sensor configuration parameters.
 */
struct OUSTER_API_CLASS SensorConfig {
    optional<std::string> udp_dest;     ///< The destination address for the
                                        ///< lidar/imu data to be sent to
    optional<std::string> udp_dest_zm;  ///< The destination address for the
                                        ///< ZM data to be sent to
    optional<uint16_t> udp_port_lidar;  ///< The destination port for the lidar
                                        ///< data to be sent to
    optional<uint16_t> udp_port_imu;    ///< The destination port for the imu
                                        ///< data to be sent to
    optional<uint16_t> udp_port_zm;     ///< The destination port for the ZM
                                        ///< data to be sent to

    /**
     * Multicast TTL for IMU and lidar UDP traffic.
     */
    optional<uint32_t> udp_multicast_ttl;

    /**
     * Multicast TTL for ZM UDP traffic.
     */
    optional<uint32_t> udp_multicast_ttl_zm;

    /**
     * The timestamp mode for the sensor to use.
     * Refer to TimestampMode for more details.
     */
    optional<TimestampMode> timestamp_mode;

    /**
     * The lidar mode for the sensor to use.
     * Refer to LidarMode for more details.
     */
    optional<LidarMode> lidar_mode;

    /**
     * The operating mode for the sensor to use.
     * Refer to OperatingMode for more details.
     */
    optional<OperatingMode> operating_mode;

    /**
     * The multipurpose io mode for the sensor to use.
     * Refer to MultipurposeIOMode for more details.
     */
    optional<MultipurposeIOMode> multipurpose_io_mode;

    /**
     * The azimuth window for the sensor to use.
     * Refer to AzimuthWindow for more details.
     */
    optional<AzimuthWindow> azimuth_window;

    /**
     * The lidar frame azimuth offset for the sensor to use.
     * Refer to the sensor docs for more details.
     */
    optional<unsigned int> lidar_frame_azimuth_offset;

    /**
     * Multiplier for signal strength of sensor. See the sensor docs for
     * more details on usage.
     */
    optional<double> signal_multiplier;

    /**
     * The nmea polarity for the sensor to use.
     * Refer to Polarity for more details.
     */
    optional<Polarity> nmea_in_polarity;

    /**
     * Whether NMEA UART input $GPRMC messages should be ignored.
     * Refer to the sensor docs for more details.
     */
    optional<bool> nmea_ignore_valid_char;

    /**
     * The nmea baud rate for the sensor to use.
     * Refer to Polarity> for more details.
     */
    optional<NMEABaudRate> nmea_baud_rate;

    /**
     * Number of leap seconds added to UDP timestamp.
     * See the sensor docs for more details.
     */
    optional<int> nmea_leap_seconds;

    /**
     * Polarity of SYNC_PULSE_IN input.
     * See Polarity for more details.
     */
    optional<Polarity> sync_pulse_in_polarity;

    /**
     * Polarity of SYNC_PULSE_OUT output.
     * See Polarity for more details.
     */
    optional<Polarity> sync_pulse_out_polarity;

    /**
     * Angle in degrees that sensor traverses between each SYNC_PULSE_OUT
     * pulse. See senor docs for more details.
     */
    optional<int> sync_pulse_out_angle;

    /**
     * Width of SYNC_PULSE_OUT pulse in ms.
     * See sensor docs for more details.
     */
    optional<int> sync_pulse_out_pulse_width;

    /**
     * Frequency of SYNC_PULSE_OUT pulse in Hz.
     * See sensor docs for more details.
     */
    optional<int> sync_pulse_out_frequency;

    /**
     * Whether phase locking is enabled.
     * See sensor docs for more details.
     */
    optional<bool> phase_lock_enable;

    /**
     * Angle that sensors are locked to in millidegrees.
     * See sensor docs for more details.
     */
    optional<int> phase_lock_offset;

    /**
     * Columns per packet.
     * See sensor docs for more details.
     */
    optional<int> columns_per_packet;

    /**
     * The lidar profile for the sensor to use.
     * Refer to UDPProfileLidar for more details.
     */
    optional<UDPProfileLidar> udp_profile_lidar;

    /**
     * The imu profile for the sensor to use.
     * Refer to UDPProfileIMU for more details.
     */
    optional<UDPProfileIMU> udp_profile_imu;

    /**
     * The udp profile type for the sensor to use.
     * Refer to HeaderType for more details.
     */
    optional<HeaderType> header_type;

    /**
     * The gyro full scale measurement range to use.
     * Refer to FullScaleRange for more details.
     */
    optional<FullScaleRange> gyro_fsr;

    /**
     * The accelerometer full scale measurement range to use.
     * Refer to FullScaleRange for more details.
     */
    optional<FullScaleRange> accel_fsr;

    /**
     * The priority of returns for the lidar to output.
     * Refer to ReturnOrder for more details.
     */
    optional<ReturnOrder> return_order;

    /**
     * The minimum detection range of the lidar in cm.
     */
    optional<int> min_range_threshold_cm;

    /**
     * The number of imu packets per frame.
     */
    optional<uint32_t> imu_packets_per_frame;

    /**
     * The bloom reduction optimization setting.
     */
    optional<BloomReductionOptimization> bloom_reduction_optimization;

    /**
     * Extra config options to apply that arent in the standard set.
     * Each value should be stringized json
     */
    std::map<std::string, std::string> extra_options;

    /**
     * Constructs a SensorConfig object by parsing a JSON-formatted
     * configuration string.
     *
     * @param[in] config_json JSON string containing configuration settings for
     * the sensor.
     * @throws std::runtime_error If the SensorConfig JSON string is invalid or
     * incomplete.
     */
    OUSTER_API_FUNCTION
    explicit SensorConfig(const std::string& config_json);

    /** Default constructor */
    OUSTER_API_FUNCTION
    SensorConfig();
};

/**
 * @deprecated Use ouster::sdk::core::SensorConfig instead.
 */
OUSTER_DEPRECATED_TYPE(sensor_config, SensorConfig,             // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

/** Stores data format information. */
struct OUSTER_API_CLASS DataFormat {
    uint32_t pixels_per_column;   ///< pixels per column
    uint32_t columns_per_packet;  ///< columns per packet
    uint32_t
        columns_per_frame;  ///< columns per frame, should match with lidar mode
    uint32_t imu_measurements_per_packet;  ///< imu measurements per packet
    uint32_t imu_packets_per_frame;        ///< imu packets per frame
    std::vector<int>
        pixel_shift_by_row;      ///< shift of pixels by row to enable destagger
    ColumnWindow column_window;  ///< window of columns over which sensor fires
    UDPProfileLidar udp_profile_lidar{};  ///< profile of lidar packet
    UDPProfileIMU udp_profile_imu{};      ///< profile of imu packet
    HeaderType header_type{};             ///< profile of lidar/imu headers
    uint16_t fps;                         ///< frames per second
    bool zone_monitoring_enabled{false};  ///< if yes, zone monitoring is on

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
};

/**
 * @deprecated Use ouster::sdk::core::DataFormat instead.
 */
OUSTER_DEPRECATED_TYPE(data_format, DataFormat,                 // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

/** Stores from-sensor calibration information */
struct OUSTER_API_CLASS CalibrationStatus {
    optional<bool>
        reflectivity_status;  ///< Whether reflectivity calibration is present.
    optional<std::string>
        reflectivity_timestamp;  ///< Timestamp of reflectivity calibration.
};

/**
 * @deprecated Use ouster::sdk::core::CalibrationStatus instead.
 */
OUSTER_DEPRECATED_TYPE(calibration_status, CalibrationStatus,   // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

/** Stores parsed information about the prod line */
class OUSTER_API_CLASS ProductInfo {
   public:
    /**
     * The original full product line string.
     */
    const std::string full_product_info;

    /**
     * The form factor of the sensor. This will
     * look like: "OS1" or "OS2" and such.
     */
    const std::string form_factor;

    /**
     * Is the sensor a short range build or not.
     */
    const bool short_range;

    /**
     * The beam configuration of the sensor.
     */
    const std::string beam_config;

    /**
     * The number of beams on the sensor.
     */
    const int beam_count;

    /**
     * Static method used to create ProductInfo classes.
     *
     * @throws std::runtime_error on a bad product info line.
     *
     * @param[in] product_info_string The product info string to create
     *                                the ProductInfo class from.
     * @return The new ProductInfo class.
     */
    OUSTER_API_FUNCTION
    static ProductInfo create_product_info(std::string product_info_string);

    /**
     * Default constructor for ProductInfo that
     * sets everything to blank.
     */
    OUSTER_API_FUNCTION
    ProductInfo();

   protected:
    /**
     * Constructor to initialize each of the members off of.
     * @brief Constructor for ProductInfo that takes params (internal only)
     *
     * @param[in] product_info_string The full product line string.
     * @param[in] form_factor The sensor form factor.
     * @param[in] short_range If the sensor is short range or not.
     * @param[in] beam_config The beam configuration for the sensor.
     * @param[in] beam_count The number of beams for a sensor.
     *
     * @internal
     */
    OUSTER_API_FUNCTION
    ProductInfo(std::string product_info_string, std::string form_factor,
                bool short_range, std::string beam_config, int beam_count);
};
/**
 * @deprecated Use ouster::sdk::core::ProductInfo instead.
 */
OUSTER_DEPRECATED_TYPE(product_info, ProductInfo,               // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

/**
 * Equality for ProductInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const ProductInfo& lhs, const ProductInfo& rhs);

/**
 * Inequality for ProductInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const ProductInfo& lhs, const ProductInfo& rhs);

/**
 * Get string representation of a product info.
 *
 * @param[in] info Product Info to get the string representation from.
 *
 * @return string representation of the product info.
 */
OUSTER_API_FUNCTION
std::string to_string(const ProductInfo& info);

/**
 * Stores parsed information from metadata.
 */
class OUSTER_API_CLASS SensorInfo {
   public:
    /// Copy constructor for sensor_info.
    OUSTER_API_FUNCTION
    SensorInfo(const SensorInfo&) = default;
    /// Move constructor for SensorInfo.
    OUSTER_API_FUNCTION
    SensorInfo(SensorInfo&&) = default;
    OUSTER_API_FUNCTION
    ~SensorInfo() = default;
    OUSTER_API_FUNCTION
    SensorInfo& operator=(const SensorInfo&) = default;

    // clang-format off
    uint64_t sn{};              ///< sensor serial number corresponding to prod_sn in
                                ///< metadata.json
    std::string
        fw_rev{};               ///< fw revision corresponding to build_rev in metadata.json
    std::string prod_line{};    ///< prod line

    DataFormat format{};       ///< data format of sensor
    std::vector<double>
        beam_azimuth_angles{};  ///< beam azimuth angles for 3D projection
    std::vector<double>
        beam_altitude_angles{}; ///< beam altitude angles for 3D projection
    double lidar_origin_to_beam_origin_mm{};  ///< distance between lidar origin
                                              ///< and beam origin in mm
    mat4d beam_to_lidar_transform =
        mat4d::Zero();          ///< transform between beam and lidar frame
    mat4d imu_to_sensor_transform =
        mat4d::Zero();          ///< transform between sensor coordinate
                                ///< frame and imu
    mat4d lidar_to_sensor_transform =
        mat4d::Zero();          ///< transform between lidar and sensor
                                ///< coordinate frames
    mat4d extrinsic =
        mat4d::Zero();          ///< user-convenience client-side assignable extrinsic
                                ///< matrix, currently is not read from metadata.json
    uint32_t init_id{};         ///< initialization ID updated every reinit

    std::string build_date{};   ///< build date from FW SensorInfo
    std::string image_rev{};    ///< image rev from FW SensorInfo
    std::string prod_pn{};      ///< prod pn
    std::string status{};       ///< sensor status at time of pulling metadata

    CalibrationStatus cal{};   ///< sensor calibration
    SensorConfig config{};     ///< parsed sensor config if available from metadata
    std::string user_data{};    ///< userdata from sensor if available
    nonstd::optional<ZoneSet> zone_set{};  ///< zone monitor configuration, if present

    /**
     * Constructs a SensorInfo object by parsing a metadata.
     *
     * @param[in] metadata JSON-formatted string representing sensor metadata.
     * @throws std::runtime_error If the metadata string is empty.
     */
    OUSTER_API_FUNCTION
    explicit SensorInfo(const std::string& metadata);

    /* Empty constructor -- keep for  */
    OUSTER_API_FUNCTION
    SensorInfo();

    /** Return an updated version of the metadata string reflecting any
     * changes to the SensorInfo.
     * Errors out if changes are incompatible but does not check for validity
     *
     * @return json serialized version of this object
     */
    OUSTER_API_FUNCTION
    std::string to_json_string() const;

    /**
     * Parse and return version info about this sensor.
     *
     * @return sensor version info
     */
    OUSTER_API_FUNCTION
    Version get_version() const;

    /**
     * Extracts and returns parsed product information for the sensor.
     *
     * @return A ProductInfo object containing the form factor, beam count, and other
     *         relevant product metadata parsed from the SensorInfo.
     */
    OUSTER_API_FUNCTION
    ProductInfo get_product_info() const;

    /**
     * Compares the current SensorInfo object to another and checks if the
     * fields (excluding user metadata) are equal.
     *
     * @param[in] other Another SensorInfo object to compare with.
     * @return True if key fields are equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool has_fields_equal(const SensorInfo& other) const;

    /**
     * Get the number of returns output by the sensor in its current setup.
     *
     * @return number of returns
     */
    OUSTER_API_FUNCTION
    int num_returns() const;

    /**
     * Retrieves the width of a frame
     *
     * @return width of a frame.
     */
    OUSTER_API_FUNCTION
    auto w() const -> decltype(format.columns_per_frame);  ///< returns the width of a frame (equivalent to format.columns_per_frame)

    /**
     * Retrieves the height of a frame
     *
     * @return height of a frame.
     */
    OUSTER_API_FUNCTION
    auto h() const -> decltype(format.pixels_per_column);  ///< returns the height of a frame (equivalent to format.pixels_per_column)
    // clang-format on
};

/**
 * @deprecated Use ouster::sdk::core::SensorInfo instead.
 */
OUSTER_DEPRECATED_TYPE(sensor_info, SensorInfo,                 // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

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
 * Equality for SensorInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const SensorInfo& lhs, const SensorInfo& rhs);

/**
 * Inequality for SensorInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const SensorInfo& lhs, const SensorInfo& rhs);

/**
 * Equality for sensor config.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const SensorConfig& lhs, const SensorConfig& rhs);

/**
 * Inequality for sensor config.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const SensorConfig& lhs, const SensorConfig& rhs);

/**
 * Equality of sensor calibration.
 *
 * @param[in] lhs The first object to compare.
 * @param[out] rhs The second object to compare.
 */
OUSTER_API_FUNCTION
bool operator==(const CalibrationStatus& lhs, const CalibrationStatus& rhs);

/**
 * Inequality of sensor calibration.
 *
 * @param[in] lhs The first object to compare.
 * @param[out] rhs The second object to compare.
 */
OUSTER_API_FUNCTION
bool operator!=(const CalibrationStatus& lhs, const CalibrationStatus& rhs);

/**
 * Get a default SensorInfo for the given lidar mode.
 *
 * @param[in] mode lidar mode to generate default SensorInfo for.
 *
 * @return default SensorInfo for the OS1-64.
 */
OUSTER_API_FUNCTION
SensorInfo default_sensor_info(LidarMode mode);

/**
 * Get string representation of a lidar mode.
 *
 * @param[in] mode LidarMode to get the string representation for.
 *
 * @return string representation of the lidar mode, or "UNKNOWN".
 */
OUSTER_API_FUNCTION
std::string to_string(LidarMode mode);

/**
 * Get lidar mode from string.
 *
 * @param[in] s String to decode.
 *
 * @return lidar mode corresponding to the string, or 0 on error.
 */
OUSTER_API_FUNCTION
LidarMode lidar_mode_of_string(const std::string& s);

/**
 * Get number of columns in a scan for a lidar mode.
 *
 * @param[in] mode LidarMode to get the number of columns for.
 *
 * @return number of columns per rotation for the mode.
 */
OUSTER_API_FUNCTION
uint32_t n_cols_of_lidar_mode(LidarMode mode);

/**
 * Get the lidar rotation frequency from lidar mode.
 *
 * @param[in] mode Lidar mode to get the rotation frequency from.
 *
 * @return lidar rotation frequency in Hz.
 */
OUSTER_API_FUNCTION
int frequency_of_lidar_mode(LidarMode mode);

/**
 * Get string representation of a timestamp mode.
 *
 * @param[in] mode TimestampMode to get the string representation for.
 *
 * @return string representation of the timestamp mode, or "UNKNOWN".
 */
OUSTER_API_FUNCTION
std::string to_string(TimestampMode mode);

/**
 * Get timestamp mode from string.
 *
 * @param[in] s String to decode into a timestamp mode.
 *
 * @return timestamp mode corresponding to the string, or 0 on error.
 */
OUSTER_API_FUNCTION
TimestampMode timestamp_mode_of_string(const std::string& s);

/**
 * Get string representation of an operating mode.
 *
 * @param[in] mode Operating mode to get the string representation from.
 *
 * @return string representation of the operating mode, or "UNKNOWN".
 */
OUSTER_API_FUNCTION
std::string to_string(OperatingMode mode);

/**
 * Get operating mode from string.
 *
 * @param[in] s String to get the operating mode from.
 *
 * @return operating mode corresponding to the string, or 0 on error.
 */
OUSTER_API_FUNCTION
optional<OperatingMode> operating_mode_of_string(const std::string& s);

/**
 * Get string representation of a multipurpose io mode.
 *
 * @param[in] mode Multipurpose io mode to get a string representation from.
 *
 * @return string representation of the multipurpose io mode, or "UNKNOWN".
 */
OUSTER_API_FUNCTION
std::string to_string(MultipurposeIOMode mode);

/**
 * Get multipurpose io mode from string.
 *
 * @param[in] s String to decode into a multipurpose io mode.
 *
 * @return multipurpose io mode corresponding to the string, or 0 on error.
 */
OUSTER_API_FUNCTION
optional<MultipurposeIOMode> multipurpose_io_mode_of_string(
    const std::string& s);

/**
 * Get string representation of a polarity.
 *
 * @param[in] polarity The polarity to get the string representation of.
 *
 * @return string representation of the polarity, or "UNKNOWN".
 */
OUSTER_API_FUNCTION
std::string to_string(Polarity polarity);

/**
 * Get polarity from string.
 *
 * @param[in] s The string to decode into a polarity.
 *
 * @return polarity corresponding to the string, or 0 on error.
 */
OUSTER_API_FUNCTION
optional<Polarity> polarity_of_string(const std::string& s);

/**
 * Get string representation of a NMEA Baud Rate.
 *
 * @param[in] rate The NNEABaudRate to get the string representation of.
 *
 * @return string representation of the NMEA baud rate, or "UNKNOWN".
 */
OUSTER_API_FUNCTION
std::string to_string(NMEABaudRate rate);

/**
 * Get nmea baud rate from string.
 *
 * @param[in] s The string to decode into a NMEA baud rate.
 *
 * @return nmea baud rate corresponding to the string, or 0 on error.
 */
OUSTER_API_FUNCTION
optional<NMEABaudRate> nmea_baud_rate_of_string(const std::string& s);

/**
 * Get string representation of an Azimuth Window.
 *
 * @param[in] azimuth_window The azimuth window to get the string
representation. of
 *
 * @return string representation of the azimuth window.
 */
OUSTER_API_FUNCTION
std::string to_string(AzimuthWindow azimuth_window);

/**
 * Get string representation of a lidar profile.
 *
 * @param[in] profile The profile to get the string representation of.
 *
 * @return string representation of the lidar profile.
 */
OUSTER_API_FUNCTION
std::string to_string(UDPProfileLidar profile);

/**
 * Get lidar profile from string.
 *
 * @param[in] s The string to decode into a lidar profile.
 *
 * @return lidar profile corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<UDPProfileLidar> udp_profile_lidar_of_string(const std::string& s);

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
 * @param[in] s The string to decode into an imu profile.
 *
 * @return imu profile corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<UDPProfileIMU> udp_profile_imu_of_string(const std::string& s);

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
 * Get header layut profile from string
 *
 * @param[in] s The string to decode into an imu profile.
 *
 * @return header layout profile corresponding to the string, or nullopt on
 *         error.
 */
OUSTER_API_FUNCTION
optional<HeaderType> udp_profile_type_of_string(const std::string& s);

/**
 * Get full scale range setting from string
 *
 * @param[in] s The string to decode into a full scale range.
 *
 * @return full scale range corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<FullScaleRange> full_scale_range_of_string(const std::string& s);

/**
 * Get return order setting from string
 *
 * @param[in] s The string to decode into a return order.
 *
 * @return return order corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<ReturnOrder> return_order_of_string(const std::string& s);

/**
 * Get string representation of a Return Order.
 *
 * @param[in] return_order The return order to get the string
 * representation of.
 *
 * @return string representation of the return order.
 */
OUSTER_API_FUNCTION
std::string to_string(ReturnOrder return_order);

/**
 * Get string representation of a Full Scale Range.
 *
 * @param[in] full_scale_range The shot limiting status to get the string
 *                             representation of.
 *
 * @return string representation of the full scale range.
 */
OUSTER_API_FUNCTION
std::string to_string(FullScaleRange full_scale_range);

/**
 * Get string representation of a Shot Limiting Status.
 *
 * @param[in] shot_limiting_status The shot limiting status to get the string
 *                                 representation of.
 *
 * @return string representation of the shot limiting status.
 */
OUSTER_API_FUNCTION
std::string to_string(ShotLimitingStatus shot_limiting_status);

/**
 * Get string representation of Thermal Shutdown Status.
 *
 * @param[in] thermal_shutdown_status The thermal shutdown status to get the
 *                                    string representation of.
 *
 * @return string representation of thermal shutdown status.
 */
OUSTER_API_FUNCTION
std::string to_string(ThermalShutdownStatus thermal_shutdown_status);

/**
 * Get string representation of Bloom Reduction Optimization setting.
 *
 * @param[in] bloom_reduction_optimization The bloom reduction optimization
 *                                         setting to get the string
 *                                         representation of.
 *
 * @return string representation of bloom reduction optimization setting.
 */
OUSTER_API_FUNCTION
std::string to_string(BloomReductionOptimization bloom_reduction_optimization);

/**
 * Get Bloom Reduction Optimization setting from string.
 *
 * @param[in] s The string to decode into a bloom reduction optimization
 *              setting.
 *
 * @return bloom reduction optimization setting corresponding to the string,
 *         or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<BloomReductionOptimization> bloom_reduction_optimization_of_string(
    const std::string& s);

/**
 * Determine validity of provided signal multiplier value
 *
 * @param[in] signal_multiplier Signal multiplier value.
 */
OUSTER_API_FUNCTION
void check_signal_multiplier(const double signal_multiplier);

/**
 * Parse metadata given path to a json file.
 *
 * @throw runtime_error if json file does not exist or is malformed.
 *
 * @param[in] json_file path to a json file containing sensor metadata.
 * @param[in] skip_beam_validation whether to skip validation on metadata - not
 *                                 for use on recorded data or metadata
 *                                 from sensors
 *
 * @return a SensorInfo struct populated with a subset of the metadata.
 */
OUSTER_API_FUNCTION
SensorInfo metadata_from_json(const std::string& json_file,
                              bool skip_beam_validation = false);

// clang-format off
/**
 * String representation of the SensorInfo. All fields included. NOT equivalent
 * or interchangeable with metadata from sensor.
 *
 * @param[in] info SensorInfo struct
 *
 * @return a debug string in json format
 */
[[deprecated("This is a debug function. Use original_string() or "
              "updated_metadata_string()")]]
OUSTER_API_FUNCTION
std::string to_string(const SensorInfo& info);

// clang-format on

/**
 * Get a string representation of sensor config. Only set fields will be
 * represented.
 *
 * @param[in] config a struct of sensor config.
 *
 * @return a json sensor config string.
 */
OUSTER_API_FUNCTION
std::string to_string(const SensorConfig& config);

/**
 * Get a string representation of sensor calibration. Only set fields will be
 * represented.
 *
 * @param[in] cal a struct of calibration.
 *
 * @return string representation of sensor calibration.
 */
OUSTER_API_FUNCTION
std::string to_string(const CalibrationStatus& cal);

/**
 * Get client version.
 *
 * @return client version string
 */
OUSTER_API_FUNCTION
std::string client_version();

// clang-format off

/**
 * Table of accessors for extracting data from imu and lidar packets.
 *
 * In the user guide, refer to section 9 for the lidar packet format and section
 * 10 for imu packets.
 *
 * For 0 <= n < columns_per_packet, nth_col(n, packet_buf) returns a pointer to
 * the nth measurement block. For 0 <= m < pixels_per_column, nth_px(m, col_buf)
 * returns the mth channel data block.
 *
 * Use imu_la_{x,y,z} to access the acceleration in the corresponding
 * direction. Use imu_av_{x,y,z} to read the angular velocity.
 */
class OUSTER_API_CLASS PacketFormat {
   protected:
    struct Impl;
    std::shared_ptr<const Impl> impl_;

    std::vector<std::pair<std::string, ChanFieldType>> field_types_;

   public:
    /**
     * Construct packet format from data format.
     *
     * @param[in] format data format
     */
    OUSTER_API_FUNCTION
    PacketFormat(const DataFormat& format);

    /**
     * Construct packet format from sensor info
     *
     * @param[in] info sensor info
     */
    OUSTER_API_FUNCTION
    PacketFormat(const SensorInfo& info);

    using FieldIter =
        decltype(field_types_)::const_iterator;  ///< iterator over field types
                                                 ///< of packet

    const UDPProfileLidar
        udp_profile_lidar;  ///< udp lidar profile of packet format
    const UDPProfileIMU udp_profile_imu;  ///< udp imu profile of packet format
    const HeaderType header_type;         ///< header type of packets
    const size_t lidar_packet_size;       ///< lidar packet size
    const size_t imu_packet_size;         ///< imu packet size
    const size_t zone_packet_size;        ///< zone monitoring packet size
    const int columns_per_packet;         ///< columns per lidar packet
    const int pixels_per_column;          ///< pixels per column for lidar

    const size_t imu_measurements_per_packet;  ///< number of accel/gyro
                                               ///< measurements per imu packet
    const size_t imu_packets_per_frame;        ///< number of imu packets per
                                               ///< frame

    const size_t packet_header_size;  ///< Size in bytes of the packet header.
    const size_t col_header_size;     ///< Size in bytes of the column header.
    const size_t col_footer_size;     ///< Size in bytes of the column footer
    const size_t col_size;  ///< Total size in bytes of a single column block
                            ///< (header + data + footer).
    const size_t packet_footer_size;  ///< Size in bytes of the packet footer

    const uint64_t max_frame_id;   ///< maximum frame id for this packet format
    bool zone_monitoring_enabled;  ///< if yes, zone monitoring is on

    /**
     * Read the packet type packet header.
     * Applicable to non-legacy Lidar and IMU packets.
     *
     * @param[in] packet_buf the packet buffer.
     *
     * @return the packet type.
     */
    OUSTER_API_FUNCTION
    uint16_t packet_type(const uint8_t* packet_buf) const;

    /**
     * Read the frame_id packet header.
     * Applicable to lidar packets and non-legacy IMU packets.
     *
     * @param[in] packet_buf the packet buffer.
     *
     * @return the frame id.
     */
    OUSTER_API_FUNCTION
    uint32_t frame_id(const uint8_t* packet_buf) const;

    /**
     * Read the initialization id packet header.
     * Applicable to non-legacy Lidar and IMU packets.
     *
     * @param[in] packet_buf the packet buffer.
     *
     * @return the init id.
     */
    OUSTER_API_FUNCTION
    uint32_t init_id(const uint8_t* packet_buf) const;

    /**
     * Read the packet serial number header.
     * Applicable to non-legacy Lidar and IMU packets.
     *
     * @param[in] packet_buf the packet buffer.
     *
     * @return the serial number.
     */
    OUSTER_API_FUNCTION
    uint64_t prod_sn(const uint8_t* packet_buf) const;

    /**
     * Read the alert flags.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the alert flags byte.
     */
    OUSTER_API_FUNCTION
    uint8_t alert_flags(const uint8_t* lidar_buf) const;

    /**
     * Read the packet thermal shutdown countdown
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the thermal shutdown countdown.
     */
    OUSTER_API_FUNCTION
    uint16_t countdown_thermal_shutdown(const uint8_t* lidar_buf) const;

    /**
     * Read the packet shot limiting countdown
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the shot limiting countdown.
     */
    OUSTER_API_FUNCTION
    uint16_t countdown_shot_limiting(const uint8_t* lidar_buf) const;

    /**
     * Read the packet thermal shutdown header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the thermal shutdown status
     */
    OUSTER_API_FUNCTION
    uint8_t thermal_shutdown(const uint8_t* lidar_buf) const;

    /**
     * Read the packet shot limiting header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the shot limiting status
     */
    OUSTER_API_FUNCTION
    uint8_t shot_limiting(const uint8_t* lidar_buf) const;

    /**
     * Get the bit width of the specified channel field.
     *
     * @param[in] f the channel field to query.
     *
     * @return a type tag specifying the bitwidth of the requested field or
     * ChannelFieldType::VOID if it is not supported by the packet format.
     */
    OUSTER_API_FUNCTION
    ChanFieldType field_type(const std::string& f) const;

    /**
     * A const forward iterator over field / type pairs.
     *
     * @return Iterator pointing to the first element in the field type of
     * packets.
     *
     */
    OUSTER_API_FUNCTION
    FieldIter begin() const;

    /**
     * A const forward iterator over field / type pairs.
     *
     * @return Iterator pointing to the last element in the field type of
     * packets.
     */
    OUSTER_API_FUNCTION
    FieldIter end() const;

    /**
     * Get pointer to the packet footer of a lidar buffer.
     *
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return pointer to packet footer of lidar buffer, can be nullptr if
     * packet format doesn't have packet footer.
     */
    OUSTER_API_FUNCTION
    const uint8_t* footer(const uint8_t* lidar_buf) const;

    // Measurement block accessors

    /**
     * Get pointer to nth column of a lidar buffer.
     *
     * @param[in] n which column.
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return pointer to nth column of lidar buffer.
     */
    OUSTER_API_FUNCTION
    const uint8_t* nth_col(int n, const uint8_t* lidar_buf) const;

    /**
     * Read column timestamp from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column timestamp.
     */
    OUSTER_API_FUNCTION
    uint64_t col_timestamp(const uint8_t* col_buf) const;

    /**
     * Read measurement id from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column measurement id.
     */
    OUSTER_API_FUNCTION
    uint16_t col_measurement_id(const uint8_t* col_buf) const;

    /**
     * Read column status from column buffer.
     *
     * @param[in] col_buf the column buffer.
     *
     * @return column status.
     */
    OUSTER_API_FUNCTION
    uint32_t col_status(const uint8_t* col_buf) const;

    // clang-format off
    /**
     * @brief Encodes the column value.
     *
     * This function encodes the column value.
     *
     * @deprecated Use col_measurement_id instead. This function will be removed
     * in future versions.
     *
     * @param[in] col_buf A measurement block pointer returned by `nth_col()`.
     *
     * @return Encoded column value.
     */
    [[deprecated("Use col_measurement_id instead")]]
    OUSTER_API_FUNCTION
    uint32_t col_encoder(
        const uint8_t* col_buf)
        const;  ///< @deprecated Encoder count is deprecated as it is redundant
                ///< with measurement id, barring a multiplication factor which
                ///< varies by lidar mode. Use col_measurement_id instead
    // clang-format on

    // clang-format off
    /**
     * @brief Retrieves the current frame id
     *
     * This function returns the frame id of a column
     *
     * @deprecated Use frame_id instead. This function will be removed
     * in future versions.
     *
     * @param[in] col_buf A measurement block pointer returned by `nth_col()`.
     *
     * @return The current frame id.
     */
    [[deprecated("Use frame_id instead")]]
    OUSTER_API_FUNCTION
    uint16_t col_frame_id(
        const uint8_t* col_buf) const;  ///< @deprecated Use frame_id instead
    // clang-format on

    /**
     * Copy the specified channel field out of a packet measurement block.
     *
     * @tparam T T should be a numeric type large enough to store
     * values of the specified field. Otherwise, data will be truncated.
     *
     * @param[in] col_buf a measurement block pointer returned by `nth_col()`.
     * @param[in] f the channel field to copy.
     * @param[out] dst destination array of size pixels_per_column * dst_stride.
     * @param[in] dst_stride stride for writing to the destination array.
     */
    template <typename T>
    void col_field(const uint8_t* col_buf, const std::string& f, T* dst,
                   int dst_stride = 1) const;

    /**
     * Returns maximum available size of parsing block usable with block_field
     *
     * @return if packet format does not allow for block parsing, returns 0
     */
    OUSTER_API_FUNCTION
    int block_parsable() const;

    /**
     * Copy the specified channel field out of a packet measurement block.
     * Faster traversal than col_field, but has to copy the entire packet all at
     * once.
     *
     * @tparam T T should be a numeric type large enough to store
     * values of the specified field. Otherwise, data will be truncated.
     *
     * @param[out] field destination eigen array
     * @param[in] f the channel field to copy.
     * @param[in] lidar_buf the lidar buffer.
     */
    template <typename T, int BlockDim>
    void block_field(Eigen::Ref<img_t<T>> field, const std::string& f,
                     const uint8_t* lidar_buf) const;

    // Per-pixel channel data block accessors

    /**
     * Get pointer to nth pixel of a column buffer.
     *
     * @param[in] n which pixel.
     * @param[in] col_buf the column buffer.
     *
     * @return pointer to nth pixel of a column buffer.
     */
    OUSTER_API_FUNCTION
    const uint8_t* nth_px(int n, const uint8_t* col_buf) const;

    // IMU packet accessors

    /**
     * Get pointer to nth measurement of an IMU buffer.
     * This applies to PROFILE_IMU_ACCEL32_GYRO32_NMEA which contains multiple
     * measurements within each packet.
     * In PROFILE_IMU_LEGACY profile, each packet only contains one measurement.
     *
     * @param[in] n which measurement.
     * @param[in] imu_buf the imu buffer.
     *
     * @return pointer to nth measurement of an imu buffer.
     */
    OUSTER_API_FUNCTION
    const uint8_t* imu_nth_measurement(int n, const uint8_t* imu_buf) const;

    /**
     * Read NMEA timestamp from imu packet buffer.
     * Only available in PROFILE_ACCEL32_GYRO32_NMEA, otherwise returns 0.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return NMEA timestamp from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_nmea_ts(const uint8_t* imu_buf) const;

    // uint32_t imu_nmea_status(const uint8_t* imu_buf) const;

    // TODO: would have been std::string_view if we had access to cpp17
    /**
     * Read NMEA sentence from an IMU buffer.
     * Only available in PROFILE_ACCEL32_GYRO32_NMEA.
     *
     * @param[in] imu_buf the imu buffer.
     *
     * @return NMEA sentence string
     */
    OUSTER_API_FUNCTION
    std::string imu_nmea_sentence(const uint8_t* imu_buf) const;

    /**
     * Read sys ts from imu packet buffer.
     * Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return sys ts from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_sys_ts(const uint8_t* imu_buf) const;

    /**
     * Read acceleration timestamp.
     * Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration ts from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_accel_ts(const uint8_t* imu_buf) const;

    /**
     * Read gyro timestamp.
     * Only available in PROFILE_IMU_LEGACY, otherwise returns 0.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return gyro ts from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_gyro_ts(const uint8_t* imu_buf) const;

    /**
     * Read acceleration in x.
     * Acceleration unit is g.
     *
     * @param[in] buffer pointer to the imu buffer containing the measurement
     *
     * @return acceleration in x.
     */
    OUSTER_API_FUNCTION
    float imu_la_x(const uint8_t* buffer) const;

    /**
     * Read acceleration in y.
     * Acceleration unit is g.
     *
     * @param[in] buffer pointer to the imu buffer containing the measurement
     *
     * @return acceleration in y.
     */
    OUSTER_API_FUNCTION
    float imu_la_y(const uint8_t* buffer) const;

    /**
     * Read acceleration in z.
     * Acceleration unit is g.
     *
     * @param[in] buffer pointer to the imu buffer containing the measurement
     *
     * @return acceleration in z.
     */
    OUSTER_API_FUNCTION
    float imu_la_z(const uint8_t* buffer) const;

    /**
     * Read angular velocity in x.
     * Angular velocity unit is degrees/second.
     *
     * @param[in] buffer pointer to the imu buffer containing the measurement
     *
     * @return angular velocity in x.
     */
    OUSTER_API_FUNCTION
    float imu_av_x(const uint8_t* buffer) const;

    /**
     * Read angular velocity in y.
     * Angular velocity unit is degrees/second.
     *
     * @param[in] buffer pointer to the imu buffer containing the measurement
     *
     * @return angular velocity in y.
     */
    OUSTER_API_FUNCTION
    float imu_av_y(const uint8_t* buffer) const;

    /**
     * Read angular velocity in z.
     * Angular velocity unit is degrees/second.
     *
     * @param[in] buffer pointer to the imu buffer containing the measurement
     *
     * @return angular velocity in z.
     */
    OUSTER_API_FUNCTION
    float imu_av_z(const uint8_t* buffer) const;

    /**
     * Parse imu measurements of acceleration.
     *
     * Does not work with non-legacy IMU format.
     *
     * @param[in] col_offset offset to the first column to fill data in
     * @param[in] imu_buf pointer to imu packet data
     * @param[out] accel field to store acceleration data, which must be of
     *             shape (N,3) and float type
     */
    OUSTER_API_FUNCTION
    void parse_accel(size_t col_offset, const uint8_t* imu_buf, Field& accel);

    /**
     * Parse imu measurements of angular velocity.
     *
     * Does not work with non-legacy IMU format.
     *
     * @param[in] col_offset offset to the first column to fill data in
     * @param[in] imu_buf pointer to imu packet data
     * @param[out] gyro field to store angular velocity data, which must be of
     *             shape (N,3) and float type
     */
    OUSTER_API_FUNCTION
    void parse_gyro(size_t col_offset, const uint8_t* imu_buf, Field& gyro);

    /**
     * Get the mask of possible values that can be parsed by the channel field
     *
     * @param[in] f the channel field
     *
     * @return mask of possible values
     */
    OUSTER_API_FUNCTION
    uint64_t field_value_mask(const std::string& f) const;

    /**
     * Get number of bits in the channel field
     *
     * @param[in] f the channel field
     *
     * @return number of bits
     */
    OUSTER_API_FUNCTION
    int field_bitness(const std::string& f) const;

    /**
     * Return the CRC contained in the packet if present
     *
     * @param[in] buf the packet buffer.
     * @param[in] buffer_size size of buffer
     *
     * @return crc contained in the packet if present
     */
    OUSTER_API_FUNCTION
    optional<uint64_t> crc(const uint8_t* buf, size_t buffer_size) const;

    /**
     * Calculate the CRC for the given packet.
     *
     * @param[in] buf the packet buffer.
     * @param[in] buffer_size size of buffer
     *
     * @return calculated crc of the packet
     */
    OUSTER_API_FUNCTION
    uint64_t calculate_crc(const uint8_t* buf, size_t buffer_size) const;

    /**
     * Get zone monitoring timestamp.
     *
     * @param[in] zone_packet zone monitoring packet
     * @return zone monitoring timestamp
     */
    OUSTER_API_FUNCTION
    uint64_t zone_timestamp(const uint8_t* zone_packet) const;

    /**
     * Get live zoneset hash.
     *
     * @param[in] zone_packet zone monitoring packet
     * @return 256bit hash of zone config
     */
    OUSTER_API_FUNCTION
    std::array<uint8_t, 32> live_zoneset_hash(const uint8_t* zone_packet) const;

    /**
     * Get pointer to nth measurement of a Zone Monitoring packet.
     *
     * @param[in] n which measurement
     * @param[in] zone_packet zone monitoring packet
     *
     * @return pointer to nth measurement of a zone monitoring packet.
     */
    OUSTER_API_FUNCTION
    const uint8_t* zone_nth_measurement(int n,
                                        const uint8_t* zone_packet) const;

    /**
     * Get zone status (live or not live).
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return true if the zone is live
     */
    OUSTER_API_FUNCTION
    bool zone_live(const uint8_t* zone_buffer) const;

    /**
     * Get zone id.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone id
     */
    OUSTER_API_FUNCTION
    uint8_t zone_id(const uint8_t* zone_buffer) const;

    /**
     * Get zone error flags.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone error flags
     */
    OUSTER_API_FUNCTION
    uint8_t zone_error_flags(const uint8_t* zone_buffer) const;

    /**
     * Get zone trigger type (Occupancy / Non-occupancy).
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone trigger type
     */
    OUSTER_API_FUNCTION
    uint8_t zone_trigger_type(const uint8_t* zone_buffer) const;

    /**
     * Get zone trigger status.
     * 0x0 deasserted, 0x1 asserted
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone trigger status
     */
    OUSTER_API_FUNCTION
    uint8_t zone_trigger_status(const uint8_t* zone_buffer) const;

    /**
     * Get the count of frames triggered consecutively for the zone.
     * Resets on deassertion.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return zone triggered frames count
     */
    OUSTER_API_FUNCTION
    uint32_t zone_triggered_frames(const uint8_t* zone_buffer) const;

    /**
     * Get the count of points in the zone.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return count of points in the zone
     */
    OUSTER_API_FUNCTION
    uint32_t zone_points_count(const uint8_t* zone_buffer) const;

    /**
     * Return the number of measurements that return a range below the zone's
     * minimum range, i.e. the number of measurements that occlude the zone.
     *
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return the count of measurements that occlude the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_occlusion_count(const uint8_t* zone_buffer) const;

    /**
     * Get the number of pixels that overlap the zone for which an invalid or
     * undetectable range was returned.
     * @param[in] zone_buffer zone monitoring measurement
     *
     * @return the count of measurements with undetected or invalid range for
     * the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_invalid_count(const uint8_t* zone_buffer) const;

    /**
     * Get the maximum number of points that can be detected in the zone.
     * @param[in] zone_buffer zone monitoring measurement
     * @return maximum number of points that can be detected in the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_max_count(const uint8_t* zone_buffer) const;

    /**
     * Get the minimum range value detected in a zone from the provided zone
     * buffer.
     *
     * @param[in] zone_buffer Pointer to the buffer containing zone monitoring
     * data.
     * @return Minimum range in millimeters for the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_min_range(const uint8_t* zone_buffer) const;

    /**
     * Get the maximum range value detected in a zone from the provided zone
     * buffer.
     *
     * @param[in] zone_buffer Pointer to the buffer containing zone monitoring
     * data.
     * @return Maximum range in millimeters for the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_max_range(const uint8_t* zone_buffer) const;

    /**
     * Get the mean (average) range value detected in a zone from the provided
     * zone buffer.
     *
     * @param[in] zone_buffer Pointer to the buffer containing zone monitoring
     * data.
     * @return Mean range in millimeters for the zone.
     */
    OUSTER_API_FUNCTION
    uint32_t zone_mean_range(const uint8_t* zone_buffer) const;
};

/**
 * @deprecated Use ouster::sdk::core::PacketFormat instead.
 */
OUSTER_DEPRECATED_TYPE(packet_format, PacketFormat,             // NOLINT
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);  // NOLINT

/** @defgroup OusterClientTypeGetFormat Get Packet Format functions */

/**
 * Get a packet parser for a particular data format.
 *
 * @ingroup OusterClientTypeGetFormat
 *
 * @param[in] info parameters provided by the sensor.
 *
 * @return a PacketFormat suitable for parsing UDP packets sent by the sensor.
 */
OUSTER_API_FUNCTION
const PacketFormat& get_format(const SensorInfo& info);

/**
 * Get a packet parser for a particular data format.
 *
 * @ingroup OusterClientTypeGetFormat
 *
 * @param[in] format DataFormat
 *
 * @return a PacketFormat suitable for parsing UDP packets sent by the sensor.
 */
OUSTER_API_FUNCTION
const PacketFormat& get_format(const DataFormat& format);

/**
 * Parse latitude and longitude from an nmea sentence
 *
 * @param[in] nmea_sentence pointer to the beginning of nmea string
 * @param[out] latitude latitude
 * @param[out] longitude longitude
 *
 * @return true if sentence is successfully parsed, otherwise false
 */
OUSTER_API_FUNCTION
bool parse_lat_long(const std::string& nmea_sentence, double& latitude,
                    double& longitude);

namespace impl {

/** Maximum number of allowed lidar profiles */
constexpr int MAX_NUM_PROFILES = 32;

}  // namespace impl

/** Maximum supported NMEA sentence length */
constexpr size_t NMEA_SENTENCE_LENGTH = 85;

/**
 * The type to represent json data in string form.
 */
using json_string = std::string;

}  // namespace core
}  // namespace sdk
}  // namespace ouster

#include "impl/deprecated/types_enums.h"

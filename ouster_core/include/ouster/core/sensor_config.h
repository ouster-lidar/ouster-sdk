/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster sensor configuration information
 */

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include "nonstd/optional.hpp"
#include "ouster/core/data_format.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

using nonstd::optional;

/// Output resolution and framerate of the lidar
struct OUSTER_API_CLASS LidarMode {
    /**
     * Construct a lidar mode from a string of the form COLUMNSxFPS (e.g. 10x20)
     *
     * @param[in] mode String form of LidarMode to construct from.
     */
    OUSTER_API_FUNCTION
    explicit LidarMode(const std::string& mode);

    /**
     * Construct a lidar mode from a number of columns and FPS.
     *
     * @param[in] cols Number of columns per frame.
     * @param[in] framerate Number of frames per second.
     */
    OUSTER_API_FUNCTION
    LidarMode(unsigned int cols, unsigned int framerate);

    /// number of columns per frame
    unsigned int columns;

    /// number of frames per second
    unsigned int fps;

    /// lidar mode: 10 frames of 512 columns per second
    OUSTER_API_VAR static const LidarMode
        _512x10;  /// lidar mode: 20 frames of 512 columns per second
    OUSTER_API_VAR static const LidarMode _512x20;
    /// lidar mode: 10 frames of 1024 columns per second
    OUSTER_API_VAR static const LidarMode _1024x10;
    /// lidar mode: 20 frames of 1024 columns per second
    OUSTER_API_VAR static const LidarMode _1024x20;
    /// lidar mode: 10 frames of 2048 columns per second
    OUSTER_API_VAR static const LidarMode _2048x10;
    /// lidar mode: 5 frames of 4096 columns per second
    OUSTER_API_VAR static const LidarMode _4096x5;
};

/**
 * Equality for LidarMode.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const LidarMode& lhs, const LidarMode& rhs);

/**
 * Inequality for LidarMode.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const LidarMode& lhs, const LidarMode& rhs);

/**
 * Mode controlling timestamp method. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum class TimestampMode {
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
};

/**
 * Mode controlling sensor operation. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum class OperatingMode {
    NORMAL,   ///< Normal sensor operation
    STANDBY,  ///< Standby
};

/**
 * Mode controlling ways to input timesync information. Refer to the sensor
 * documentation for the meaning of each option.
 */
enum class MultipurposeIOMode {

    OFF = 0,  ///< Multipurpose IO is turned off (default)

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
};

/**
 * Polarity represents polarity of NMEA UART and SYNC_PULSE inputs and outputs.
 * See sensor docs for more details.
 */
enum class Polarity {
    ACTIVE_LOW = 0,  ///< ACTIVE_LOW
    ACTIVE_HIGH,     ///< ACTIVE_HIGH
};

/**
 * Baud rate the sensor attempts for NMEA UART input $GPRMC messages
 * See sensor docs for more details.
 */
enum class NMEABaudRate {
    BAUD_9600 = 0,  ///< 9600 bits per second UART baud rate
    BAUD_115200,    ///< 115200 bits per second UART baud rate
};

/** Full scale range for IMU data. */
enum class FullScaleRange {
    /** Higher precision lower range measurement mode */
    NORMAL = 0,

    /** Lower precision higher range measurement mode */
    EXTENDED,
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
 * Get string representation of a lidar mode.
 *
 * @param[in] mode LidarMode to get the string representation for.
 *
 * @return string representation of the lidar mode.
 */
OUSTER_API_FUNCTION
std::string to_string(LidarMode mode);

/**
 * Get lidar mode from string.
 *
 * @param[in] str_val String to decode.
 *
 * @return lidar mode corresponding to the string, or nullopt on error
 */
OUSTER_API_FUNCTION
optional<LidarMode> lidar_mode_of_string(const std::string& str_val);

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
 * @param[in] str_val String to decode into a timestamp mode.
 *
 * @return timestamp mode corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<TimestampMode> timestamp_mode_of_string(const std::string& str_val);

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
 * @param[in] str_val String to get the operating mode from.
 *
 * @return operating mode corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<OperatingMode> operating_mode_of_string(const std::string& str_val);

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
 * @param[in] str_val String to decode into a multipurpose io mode.
 *
 * @return multipurpose io mode corresponding to the string, or nullopt on
 * error.
 */
OUSTER_API_FUNCTION
optional<MultipurposeIOMode> multipurpose_io_mode_of_string(const std::string& str_val);

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
 * @param[in] str_val The string to decode into a polarity.
 *
 * @return polarity corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<Polarity> polarity_of_string(const std::string& str_val);

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
 * @param[in] str_val The string to decode into a NMEA baud rate.
 *
 * @return nmea baud rate corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<NMEABaudRate> nmea_baud_rate_of_string(const std::string& str_val);

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
 * Get full scale range setting from string
 *
 * @param[in] str_val The string to decode into a full scale range.
 *
 * @return full scale range corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<FullScaleRange> full_scale_range_of_string(const std::string& str_val);

/**
 * Get return order setting from string
 *
 * @param[in] str_val The string to decode into a return order.
 *
 * @return return order corresponding to the string, or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<ReturnOrder> return_order_of_string(const std::string& str_val);

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
 * @param[in] str_val The string to decode into a bloom reduction optimization
 *              setting.
 *
 * @return bloom reduction optimization setting corresponding to the string,
 *         or nullopt on error.
 */
OUSTER_API_FUNCTION
optional<BloomReductionOptimization> bloom_reduction_optimization_of_string(
    const std::string& str_val);

/**
 * Determine validity of provided signal multiplier value
 *
 * @param[in] signal_multiplier Signal multiplier value.
 */
OUSTER_API_FUNCTION
void check_signal_multiplier(const double signal_multiplier);

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

}  // namespace core
}  // namespace sdk
}  // namespace ouster

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
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/visibility.h"
#include "version.h"

namespace ouster {

using nonstd::optional;

/**
 * For image operations.
 *
 * @tparam T The data type for the array.
 */
template <typename T>
using img_t = Eigen::Array<T, -1, -1, Eigen::RowMajor>;

/** Used for transformations. */
using mat4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;

namespace sensor {

/** Unit of range from sensor packet, in meters. */
constexpr double range_unit = 0.001;

/** Design values for altitude and azimuth offset angles for gen1 sensors. */
extern const std::vector<double> gen1_altitude_angles;
/** Design values for altitude and azimuth offset angles for gen1 sensors. */
extern const std::vector<double> gen1_azimuth_angles;

/** Design values for imu and lidar to sensor-frame transforms. */
extern const mat4d default_imu_to_sensor_transform;

/** Design values for imu and lidar to sensor-frame transforms. */
extern const mat4d default_lidar_to_sensor_transform;

/**
 * Constants used for configuration. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum lidar_mode {
    MODE_UNSPEC = 0,  ///< lidar mode: unspecified
    MODE_512x10,      ///< lidar mode: 10 scans of 512 columns per second
    MODE_512x20,      ///< lidar mode: 20 scans of 512 columns per second
    MODE_1024x10,     ///< lidar mode: 10 scans of 1024 columns per second
    MODE_1024x20,     ///< lidar mode: 20 scans of 1024 columns per second
    MODE_2048x10,     ///< lidar mode: 10 scans of 2048 columns per second
    MODE_4096x5       ///< lidar mode: 5 scans of 4096 columns per second. Only
                      ///< available on select sensors
};

/**
 * Mode controlling timestamp method. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum timestamp_mode {
    /**
     * Timestamp mode unspecified.
     */
    TIME_FROM_UNSPEC = 0,

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
    TIME_FROM_PTP_1588
};

/**
 * Mode controlling sensor operation. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum OperatingMode {
    OPERATING_UNSPEC = 0,  ///< Unspecified sensor operation
    OPERATING_NORMAL,      ///< Normal sensor operation
    OPERATING_STANDBY      ///< Standby
};

/**
 * Mode controlling ways to input timesync information. Refer to the sensor
 * documentation for the meaning of each option.
 */
enum MultipurposeIOMode {

    MULTIPURPOSE_OFF = 1,  ///< Multipurpose IO is turned off (default)

    /**
     * Used in conjunction with timestamp_mode::TIME_FROM_SYNC_PULSE_IN
     * to enable time pulses in on the multipurpose io input.
     */
    MULTIPURPOSE_INPUT_NMEA_UART,

    /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * the internal clock.
     */
    MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC,

    /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * a SYNC_PULSE_IN provided to the unit.
     */
    MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN,

    /**
     * Output a SYNC_PULSE_OUT signal synchronized with
     * an external PTP IEEE 1588 master.
     */
    MULTIPURPOSE_OUTPUT_FROM_PTP_1588,

    /**
     * Output a SYNC_PULSE_OUT signal with a user defined
     * rate in an integer number of degrees.
     */
    MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE
};

/**
 * Polarity represents polarity of NMEA UART and SYNC_PULSE inputs and outputs.
 * See sensor docs for more details.
 */
enum Polarity {
    POLARITY_ACTIVE_LOW = 1,  ///< ACTIVE_LOW
    POLARITY_ACTIVE_HIGH      ///< ACTIVE_HIGH
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
enum NMEABaudRate {
    BAUD_9600 = 1,  ///< 9600 bits per second UART baud rate
    BAUD_115200     ///< 115200 bits per second UART baud rate
};

/** Profile indicating packet format of lidar data. */
enum UDPProfileLidar {
    PROFILE_LIDAR_UNKNOWN = 0,

    /** Legacy lidar data */
    PROFILE_LIDAR_LEGACY,

    /** Dual Returns data */
    PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,

    /** Single Returns data */
    PROFILE_RNG19_RFL8_SIG16_NIR16,

    /** Single Returns Low Data Rate */
    PROFILE_RNG15_RFL8_NIR8,

    /** Five Word Profile */
    PROFILE_FIVE_WORD_PIXEL,

    /** FuSa two-word pixel */
    PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL,
};

/** Profile indicating packet format of IMU data. */
enum UDPProfileIMU {
    PROFILE_IMU_LEGACY = 1,  ///< Legacy IMU data
};

/** Full scale range for IMU data. */
enum FullScaleRange {
    /** Higher precision lower range measurement mode */
    FSR_NORMAL = 0,

    /** Lower precision higher range measurement mode */
    FSR_EXTENDED
};

/** Priority of returns for the lidar to output.
 *   Lidar can have more than 1 or 2 detected "returns".
 *   This indicates to the lidar which ones it should output.
 *   See sensor docs for more details.
 */
enum ReturnOrder {
    /** Lidar returns the strongest returns first */
    ORDER_STRONGEST_TO_WEAKEST = 0,

    /** Lidar returns the furthest returns first */
    ORDER_FARTHEST_TO_NEAREST,

    /** Lidar returns the nearest returns first */
    ORDER_NEAREST_TO_FARTHEST,

    /** DEPRECATED: Only Present In Old Test Firmware */
    ORDER_DEPRECATED_STRONGEST_RETURN_FIRST,
    ORDER_DEPRECATED_LAST_RETURN_FIRST
};

/** Thermal Shutdown status. */
enum ThermalShutdownStatus {
    THERMAL_SHUTDOWN_NORMAL = 0x00,    ///< Normal operation
    THERMAL_SHUTDOWN_IMMINENT = 0x01,  ///< Thermal Shutdown imminent
};

/** Shot Limiting status. */
enum ShotLimitingStatus {
    SHOT_LIMITING_NORMAL = 0x00,    ///< Normal operation
    SHOT_LIMITING_IMMINENT = 0x01,  ///< Shot limiting imminent
    SHOT_LIMITING_REDUCTION_0_10 =
        0x02,  ///< Shot limiting reduction by 0 to 10%
    SHOT_LIMITING_REDUCTION_10_20 =
        0x03,  ///< Shot limiting reduction by 10 to 20%
    SHOT_LIMITING_REDUCTION_20_30 =
        0x04,  ///< Shot limiting reduction by 20 to 30%
    SHOT_LIMITING_REDUCTION_30_40 =
        0x05,  ///< Shot limiting reduction by 30 to 40%
    SHOT_LIMITING_REDUCTION_40_50 =
        0x06,  ///< Shot limiting reduction by 40 to 50%
    SHOT_LIMITING_REDUCTION_50_60 =
        0x07,  ///< Shot limiting reduction by 50 to 60%
    SHOT_LIMITING_REDUCTION_60_70 =
        0x08,  ///< Shot limiting reduction by 60 to 70%
    SHOT_LIMITING_REDUCTION_70_75 =
        0x09,  ///< Shot limiting reduction by 70 to 80%
};

/**
 * Convenience type alias for azimuth windows, the window over which the
 * sensor fires in millidegrees.
 */
using AzimuthWindow = std::pair<int, int>;
/**
 * Convenience type alias for column windows, the window over which the
 * sensor fires in columns.
 */
using ColumnWindow = std::pair<int, int>;

/**
 * Struct for sensor configuration parameters.
 */
struct OUSTER_API_CLASS sensor_config {
    optional<std::string> udp_dest;     ///< The destination address for the
                                        ///< lidar/imu data to be sent to
    optional<uint16_t> udp_port_lidar;  ///< The destination port for the lidar
                                        ///< data to be sent to
    optional<uint16_t> udp_port_imu;    ///< The destination port for the imu
                                        ///< data to be sent to

    // TODO: change timestamp_mode and lidar_mode to UpperCamel
    /**
     * The timestamp mode for the sensor to use.
     * Refer to timestamp_mode for more details.
     */
    optional<sensor::timestamp_mode> timestamp_mode;

    /**
     * The lidar mode for the sensor to use.
     * Refer to lidar_mode for more details.
     */
    optional<sensor::lidar_mode> lidar_mode;

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
};

/** Stores data format information. */
struct OUSTER_API_CLASS data_format {
    uint32_t pixels_per_column;   ///< pixels per column
    uint32_t columns_per_packet;  ///< columns per packet
    uint32_t
        columns_per_frame;  ///< columns per frame, should match with lidar mode
    std::vector<int>
        pixel_shift_by_row;      ///< shift of pixels by row to enable destagger
    ColumnWindow column_window;  ///< window of columns over which sensor fires
    UDPProfileLidar udp_profile_lidar{};  ///< profile of lidar packet
    UDPProfileIMU udp_profile_imu{};      ///< profile of imu packet
    uint16_t fps;                         ///< frames per second
};

/** Stores from-sensor calibration information */
struct OUSTER_API_CLASS calibration_status {
    optional<bool> reflectivity_status;
    optional<std::string> reflectivity_timestamp;
};

/** Stores parsed information about the prod line */
class OUSTER_API_CLASS product_info {
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
     * Static method used to create product_info classes.
     *
     * @throws std::runtime_error on a bad product info line.
     *
     * @param[in] product_info_string The product info string to create
     *                                the product_info class from.
     * @return The new product_info class.
     */
    OUSTER_API_FUNCTION
    static product_info create_product_info(std::string product_info_string);

    /**
     * Default constructor for product_info that
     * sets everything to blank.
     */
    OUSTER_API_FUNCTION
    product_info();

   protected:
    /**
     * Constructor to initialize each of the members off of.
     * @brief Constructor for product_info that takes params (internal only)
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
    product_info(std::string product_info_string, std::string form_factor,
                 bool short_range, std::string beam_config, int beam_count);
};

/**
 * Equality for product_info.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const product_info& lhs, const product_info& rhs);

/**
 * In-Equality for product_info.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const product_info& lhs, const product_info& rhs);

/**
 * Get string representation of a product info.
 *
 * @param[in] info Product Info to get the string representation from.
 *
 * @return string representation of the product info.
 */
OUSTER_API_FUNCTION
std::string to_string(const product_info& info);

/**
 * Stores parsed information from metadata.
 */
class OUSTER_API_CLASS sensor_info {
   public:
    OUSTER_API_FUNCTION
    sensor_info(const sensor_info&) = default;
    OUSTER_API_FUNCTION
    sensor_info(sensor_info&&) = default;
    OUSTER_API_FUNCTION
    ~sensor_info() = default;
    OUSTER_API_FUNCTION
    sensor_info& operator=(const ouster::sensor::sensor_info&) = default;

    // clang-format off
    uint64_t sn{};              ///< sensor serial number corresponding to prod_sn in
                                ///< metadata.json
    std::string
        fw_rev{};               ///< fw revision corresponding to build_rev in metadata.json
    std::string prod_line{};    ///< prod line

    data_format format{};       ///< data format of sensor
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

    std::string build_date{};   ///< build date from FW sensor_info
    std::string image_rev{};    ///< image rev from FW sensor_info
    std::string prod_pn{};      ///< prod pn
    std::string status{};       ///< sensor status at time of pulling metadata

    calibration_status cal{};   ///< sensor calibration
    sensor_config config{};     ///< parsed sensor config if available from metadata
    std::string user_data{};    ///< userdata from sensor if available

    /* Constructor from metadata */
    [[deprecated("skip_beam_validation does not do anything anymore")]] 
    OUSTER_API_FUNCTION
    explicit sensor_info(const std::string& metadata, bool skip_beam_validation);
    OUSTER_API_FUNCTION
    explicit sensor_info(const std::string& metadata);

    /* Empty constructor -- keep for  */
    OUSTER_API_FUNCTION
    sensor_info();

    /** Return an updated version of the metadata string reflecting any
     * changes to the sensor_info.
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
    ouster::util::version get_version() const;

    OUSTER_API_FUNCTION
    product_info get_product_info() const;

    OUSTER_API_FUNCTION
    bool has_fields_equal(const sensor_info& other) const;

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

   private:
    bool was_legacy_ = false;
    // clang-format on
};

/**
 * Equality for data_format.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const data_format& lhs, const data_format& rhs);

/**
 * Not-Equality for data_format.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const data_format& lhs, const data_format& rhs);

/**
 * Equality for sensor_info.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const sensor_info& lhs, const sensor_info& rhs);

/**
 * Not-Equality for sensor_info.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const sensor_info& lhs, const sensor_info& rhs);

/**
 * Equality for sensor config.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const sensor_config& lhs, const sensor_config& rhs);

/**
 * Not-Equality for sensor config.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const sensor_config& lhs, const sensor_config& rhs);

/**
 * Equality of sensor calibration.
 *
 * @param[in] lhs The first object to compare.
 * @param[out] rhs The second object to compare.
 */
OUSTER_API_FUNCTION
bool operator==(const calibration_status& lhs, const calibration_status& rhs);

/**
 * Not-Equality of sensor calibration.
 *
 * @param[in] lhs The first object to compare.
 * @param[out] rhs The second object to compare.
 */
OUSTER_API_FUNCTION
bool operator!=(const calibration_status& lhs, const calibration_status& rhs);

/**
 * Get a default sensor_info for the given lidar mode.
 *
 * @param[in] mode lidar mode to generate default sensor_info for.
 *
 * @return default sensor_info for the OS1-64.
 */
OUSTER_API_FUNCTION
sensor_info default_sensor_info(lidar_mode mode);

/**
 * Get string representation of a lidar mode.
 *
 * @param[in] mode lidar_mode to get the string representation for.
 *
 * @return string representation of the lidar mode, or "UNKNOWN".
 */
OUSTER_API_FUNCTION
std::string to_string(lidar_mode mode);

/**
 * Get lidar mode from string.
 *
 * @param[in] s String to decode.
 *
 * @return lidar mode corresponding to the string, or 0 on error.
 */
OUSTER_API_FUNCTION
lidar_mode lidar_mode_of_string(const std::string& s);

/**
 * Get number of columns in a scan for a lidar mode.
 *
 * @param[in] mode lidar_mode to get the number of columns for.
 *
 * @return number of columns per rotation for the mode.
 */
OUSTER_API_FUNCTION
uint32_t n_cols_of_lidar_mode(lidar_mode mode);

/**
 * Get the lidar rotation frequency from lidar mode.
 *
 * @param[in] mode Lidar mode to get the rotation frequency from.
 *
 * @return lidar rotation frequency in Hz.
 */
OUSTER_API_FUNCTION
int frequency_of_lidar_mode(lidar_mode mode);

/**
 * Get string representation of a timestamp mode.
 *
 * @param[in] mode timestamp_mode to get the string representation for.
 *
 * @return string representation of the timestamp mode, or "UNKNOWN".
 */
OUSTER_API_FUNCTION
std::string to_string(timestamp_mode mode);

/**
 * Get timestamp mode from string.
 *
 * @param[in] s String to decode into a timestamp mode.
 *
 * @return timestamp mode corresponding to the string, or 0 on error.
 */
OUSTER_API_FUNCTION
timestamp_mode timestamp_mode_of_string(const std::string& s);

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
 * @return a sensor_info struct populated with a subset of the metadata.
 */
OUSTER_API_FUNCTION
sensor_info metadata_from_json(const std::string& json_file,
                               bool skip_beam_validation = false);

// clang-format off
/**
 * String representation of the sensor_info. All fields included. NOT equivalent
 * or interchangeable with metadata from sensor.
 *
 * @param[in] info sensor_info struct
 *
 * @return a debug string in json format
 */
[[deprecated("This is a debug function. Use original_string() or "
              "updated_metadata_string()")]]
OUSTER_API_FUNCTION
std::string to_string(const sensor_info& info);

// clang-format on

/**
 * Parse config text blob from the sensor into a sensor_config struct.
 *
 * All fields are optional, and will only be set if found.
 *
 * @throw runtime_error if the text is not valid json.
 *
 * @param[in] config a text blob given by get_config from client.h.
 *
 * @return a sensor_config struct populated with the sensor config.
 * parameters.
 */
[[deprecated(
    "Please switch to using parse_and_validate_config")]] OUSTER_API_FUNCTION
    sensor_config
    parse_config(const std::string& config);

/**
 * Get a string representation of sensor config. Only set fields will be
 * represented.
 *
 * @param[in] config a struct of sensor config.
 *
 * @return a json sensor config string.
 */
OUSTER_API_FUNCTION
std::string to_string(const sensor_config& config);

/**
 * Get a string representation of sensor calibration. Only set fields will be
 * represented.
 *
 * @param[in] cal a struct of calibration.
 *
 * @return string representation of sensor calibration.
 */
OUSTER_API_FUNCTION
std::string to_string(const calibration_status& cal);

/**
 * Get client version.
 *
 * @return client version string
 */
OUSTER_API_FUNCTION
std::string client_version();

// clang-format off
/**
 * Get version information from the metadata.
 *
 * @param[in] metadata string.
 *
 * @return version corresponding to the string, or invalid_version on error.
 */
[[deprecated("Use sensor_info::get_version() instead")]] 
OUSTER_API_FUNCTION
ouster::util::version firmware_version_from_metadata(const std::string& metadata);

typedef const char* cf_type;
/**
 * @namespace ChanField
 * Tag to identitify a paricular value reported in the sensor channel data
 * block. */
namespace ChanField {
    static constexpr cf_type RANGE = "RANGE";            ///< 1st return range in mm
    static constexpr cf_type RANGE2 = "RANGE2";           ///< 2nd return range in mm
    static constexpr cf_type SIGNAL = "SIGNAL";           ///< 1st return signal in photons
    static constexpr cf_type SIGNAL2 = "SIGNAL2";          ///< 2nd return signal in photons
    static constexpr cf_type REFLECTIVITY = "REFLECTIVITY";     ///< 1st return reflectivity, calibrated by range and sensor
                          ///< sensitivity in FW 2.1+. See sensor docs for more details
    static constexpr cf_type REFLECTIVITY2 = "REFLECTIVITY2";    ///< 2nd return reflectivity, calibrated by range and sensor
                          ///< sensitivity in FW 2.1+. See sensor docs for more details
    
    static constexpr cf_type NEAR_IR = "NEAR_IR";          ///< near_ir in photons
    static constexpr cf_type FLAGS = "FLAGS";            ///< 1st return flags
    static constexpr cf_type FLAGS2 = "FLAGS2";           ///< 2nd return flags
    static constexpr cf_type RAW_HEADERS = "RAW_HEADERS";     ///< raw headers for packet/footer/column for dev use
    static constexpr cf_type RAW32_WORD5 = "RAW32_WORD5";     ///< raw word access to packet for dev use
    static constexpr cf_type RAW32_WORD6 = "RAW32_WORD6";     ///< raw word access to packet for dev use
    static constexpr cf_type RAW32_WORD7 = "RAW32_WORD7";     ///< raw word access to packet for dev use
    static constexpr cf_type RAW32_WORD8 = "RAW32_WORD8";     ///< raw word access to packet for dev use
    static constexpr cf_type RAW32_WORD9 = "RAW32_WORD9";     ///< raw word access to packet for dev use
    static constexpr cf_type RAW32_WORD1 = "RAW32_WORD1";     ///< raw word access to packet for dev use
    static constexpr cf_type RAW32_WORD2 = "RAW32_WORD2";     ///< raw word access to packet for dev use
    static constexpr cf_type RAW32_WORD3 = "RAW32_WORD3";     ///< raw word access to packet for dev use
    static constexpr cf_type RAW32_WORD4 = "RAW32_WORD4";     ///< raw word access to packet for dev use
};

// clang-format on
/**
 * Types of channel fields.
 */
#if defined(VOID)
#define OUSTER_REMOVED_VOID
#pragma push_macro("VOID")
#undef VOID
#endif
enum ChanFieldType {
    VOID = 0,
    UINT8 = 1,
    UINT16 = 2,
    UINT32 = 3,
    UINT64 = 4,
    INT8 = 5,
    INT16 = 6,
    INT32 = 7,
    INT64 = 8,
    FLOAT32 = 9,
    FLOAT64 = 10,
    UNREGISTERED = 100
};
#if defined(OUSTER_REMOVED_VOID)
#pragma pop_macro("VOID")
#undef OUSTER_REMOVED_VOID
#endif

/**
 * Get the size of the ChanFieldType in bytes.
 *
 * @param[in] ft the field type
 *
 * @return size of the field type in bytes
 */
OUSTER_API_FUNCTION
size_t field_type_size(ChanFieldType ft);

/**
 * Get the bit mask of the ChanFieldType.
 *
 * @param[in] ft the field type
 *
 * @return 64 bit mask
 */
OUSTER_API_FUNCTION
uint64_t field_type_mask(ChanFieldType ft);

/**
 * Get string representation of a channel field.
 *
 * @param[in] ft The field type to get the string representation of.
 *
 * @return string representation of the channel field type.
 */
OUSTER_API_FUNCTION
std::string to_string(ChanFieldType ft);

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
class OUSTER_API_CLASS packet_format {
   protected:
    struct Impl;
    std::shared_ptr<const Impl> impl_;

    std::vector<std::pair<std::string, sensor::ChanFieldType>> field_types_;

   public:
    OUSTER_API_FUNCTION
    packet_format(UDPProfileLidar udp_profile_lidar, size_t pixels_per_column,
                  size_t columns_per_packet);

    OUSTER_API_FUNCTION
    packet_format(
        const sensor_info& info);  //< create packet_format from sensor_info

    using FieldIter =
        decltype(field_types_)::const_iterator;  ///< iterator over field types
                                                 ///< of packet

    const UDPProfileLidar
        udp_profile_lidar;           ///< udp lidar profile of packet format
    const size_t lidar_packet_size;  ///< lidar packet size
    const size_t imu_packet_size;    ///< imu packet size
    const int columns_per_packet;    ///< columns per lidar packet
    const int pixels_per_column;     ///< pixels per column for lidar

    const size_t packet_header_size;
    const size_t col_header_size;
    const size_t col_footer_size;
    const size_t col_size;
    const size_t packet_footer_size;

    const uint64_t max_frame_id;

    /**
     * Read the packet type packet header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the packet type.
     */
    OUSTER_API_FUNCTION
    uint16_t packet_type(const uint8_t* lidar_buf) const;

    /**
     * Read the frame_id packet header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the frame id.
     */
    OUSTER_API_FUNCTION
    uint32_t frame_id(const uint8_t* lidar_buf) const;

    /**
     * Read the initialization id packet header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the init id.
     */
    OUSTER_API_FUNCTION
    uint32_t init_id(const uint8_t* lidar_buf) const;

    /**
     * Read the packet serial number header.
     *
     * @param[in] lidar_buf the lidar buf.
     *
     * @return the serial number.
     */
    OUSTER_API_FUNCTION
    uint64_t prod_sn(const uint8_t* lidar_buf) const;

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
     * Read sys ts from imu packet buffer.
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return sys ts from imu pacet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_sys_ts(const uint8_t* imu_buf) const;

    /**
     * Read acceleration timestamp.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration ts from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_accel_ts(const uint8_t* imu_buf) const;

    /**
     * Read gyro timestamp.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return gyro ts from imu packet buffer.
     */
    OUSTER_API_FUNCTION
    uint64_t imu_gyro_ts(const uint8_t* imu_buf) const;

    /**
     * Read acceleration in x.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration in x.
     */
    OUSTER_API_FUNCTION
    float imu_la_x(const uint8_t* imu_buf) const;

    /**
     * Read acceleration in y.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration in y.
     */
    OUSTER_API_FUNCTION
    float imu_la_y(const uint8_t* imu_buf) const;

    /**
     * Read acceleration in z.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return acceleration in z.
     */
    OUSTER_API_FUNCTION
    float imu_la_z(const uint8_t* imu_buf) const;

    /**
     * Read angular velocity in x.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return angular velocity in x.
     */
    OUSTER_API_FUNCTION
    float imu_av_x(const uint8_t* imu_buf) const;

    /**
     * Read angular velocity in y.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return angular velocity in y.
     */
    OUSTER_API_FUNCTION
    float imu_av_y(const uint8_t* imu_buf) const;

    /**
     * Read angular velocity in z.
     *
     * @param[in] imu_buf the imu packet buffer.
     *
     * @return angular velocity in z.
     */
    OUSTER_API_FUNCTION
    float imu_av_z(const uint8_t* imu_buf) const;

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
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return crc contained in the packet if present
     */
    OUSTER_API_FUNCTION
    optional<uint64_t> crc(const uint8_t* lidar_buf) const;

    /**
     * Calculate the CRC for the given packet.
     *
     * @param[in] lidar_buf the lidar buffer.
     *
     * @return calculated crc of the packet
     */
    OUSTER_API_FUNCTION
    uint64_t calculate_crc(const uint8_t* lidar_buf) const;
};

/** @defgroup OusterClientTypeGetFormat Get Packet Format functions */

/**
 * Get a packet parser for a particular data format.
 *
 * @ingroup OusterClientTypeGetFormat
 *
 * @param[in] info parameters provided by the sensor.
 *
 * @return a packet_format suitable for parsing UDP packets sent by the sensor.
 */
OUSTER_API_FUNCTION
const packet_format& get_format(const sensor_info& info);

/**
 * Get a packet parser for a particular data format.
 *
 * @ingroup OusterClientTypeGetFormat
 *
 * @param[in] udp_profile_lidar   lidar profile
 * @param[in] pixels_per_column   pixels per column
 * @param[in] columns_per_packet  columns per packet
 *
 * @return a packet_format suitable for parsing UDP packets sent by the sensor.
 */
OUSTER_API_FUNCTION
const packet_format& get_format(UDPProfileLidar udp_profile_lidar,
                                size_t pixels_per_column,
                                size_t columns_per_packet);

namespace impl {

/** Maximum number of allowed lidar profiles */
constexpr int MAX_NUM_PROFILES = 32;

}  // namespace impl

}  // namespace sensor
/**
 * The type to represent json data in string form.
 */
typedef std::string json_string;
}  // namespace ouster

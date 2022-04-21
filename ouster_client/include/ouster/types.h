/**
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

namespace ouster {

using nonstd::optional;

/** For image operations. 
 */
template <typename T>
using img_t = Eigen::Array<T, -1, -1, Eigen::RowMajor>;

/** Used for transformations
 */
using mat4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;

namespace sensor {

/** Unit of range from sensor packet, in meters. */
constexpr double range_unit = 0.001;

/** Design values for altitude and azimuth offset angles for gen1 sensors. */
extern const std::vector<double> gen1_altitude_angles;
extern const std::vector<double> gen1_azimuth_angles;

/** Design values for imu and lidar to sensor-frame transforms. */
extern const mat4d default_imu_to_sensor_transform;
extern const mat4d default_lidar_to_sensor_transform;

/*
 * Constants used for configuration. Refer to the sensor documentation for the
 * meaning of each option.
 */
enum lidar_mode {
    MODE_UNSPEC = 0, ///< @todo fill me in
    MODE_512x10, ///< @todo fill me in
    MODE_512x20, ///< @todo fill me in
    MODE_1024x10, ///< @todo fill me in
    MODE_1024x20, ///< @todo fill me in
    MODE_2048x10 ///< @todo fill me in
};

enum timestamp_mode {
    TIME_FROM_UNSPEC = 0, ///< @todo fill me in
    TIME_FROM_INTERNAL_OSC, ///< @todo fill me in
    TIME_FROM_SYNC_PULSE_IN, ///< @todo fill me in
    TIME_FROM_PTP_1588 ///< @todo fill me in
};

enum OperatingMode {
    OPERATING_NORMAL = 1, ///< @todo fill me in
    OPERATING_STANDBY  ///< @todo fill me in
};

enum MultipurposeIOMode {
    MULTIPURPOSE_OFF = 1, ///< @todo fill me in
    MULTIPURPOSE_INPUT_NMEA_UART, ///< @todo fill me in
    MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC, ///< @todo fill me in
    MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN, ///< @todo fill me in
    MULTIPURPOSE_OUTPUT_FROM_PTP_1588, ///< @todo fill me in
    MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE ///< @todo fill me in
};

enum Polarity {
    POLARITY_ACTIVE_LOW = 1, ///< @todo fill me in
    POLARITY_ACTIVE_HIGH ///< @todo fill me in
};

enum NMEABaudRate {
    BAUD_9600 = 1, ///< @todo fill me in
    BAUD_115200 ///< @todo fill me in
};

enum UDPProfileLidar {
    PROFILE_LIDAR_LEGACY = 1, ///< @todo fill me in
    PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL, ///< @todo fill me in
    PROFILE_RNG19_RFL8_SIG16_NIR16, ///< @todo fill me in
    PROFILE_RNG15_RFL8_NIR8, ///< @todo fill me in
};

enum UDPProfileIMU {
    PROFILE_IMU_LEGACY = 1 ///< @todo fill me in
};

/** Fill in Docs Here
 */
using AzimuthWindow = std::pair<int, int>;
/** Fill in Docs Here
 */
using ColumnWindow = std::pair<int, int>;

struct sensor_config {
    optional<std::string> udp_dest; ///< @todo fill in documentation here for it to show up in sphinx
    optional<int> udp_port_lidar; ///< @todo fill in documentation here for it to show up in sphinx
    optional<int> udp_port_imu; ///< @todo fill in documentation here for it to show up in sphinx

    // TODO: replace ts_mode and ld_mode when timestamp_mode and
    // lidar_mode get changed to CapsCase
    optional<timestamp_mode> ts_mode; ///< @todo fill in documentation here for it to show up in sphinx
    optional<lidar_mode> ld_mode; ///< @todo fill in documentation here for it to show up in sphinx
    optional<OperatingMode> operating_mode; ///< @todo fill in documentation here for it to show up in sphinx
    optional<MultipurposeIOMode> multipurpose_io_mode; ///< @todo fill in documentation here for it to show up in sphinx

    optional<AzimuthWindow> azimuth_window; ///< @todo fill in documentation here for it to show up in sphinx
    optional<int> signal_multiplier; ///< @todo fill in documentation here for it to show up in sphinx

    optional<Polarity> nmea_in_polarity; ///< @todo fill in documentation here for it to show up in sphinx
    optional<bool> nmea_ignore_valid_char; ///< @todo fill in documentation here for it to show up in sphinx
    optional<NMEABaudRate> nmea_baud_rate; ///< @todo fill in documentation here for it to show up in sphinx
    optional<int> nmea_leap_seconds; ///< @todo fill in documentation here for it to show up in sphinx

    optional<Polarity> sync_pulse_in_polarity; ///< @todo fill in documentation here for it to show up in sphinx
    optional<Polarity> sync_pulse_out_polarity; ///< @todo fill in documentation here for it to show up in sphinx
    optional<int> sync_pulse_out_angle; ///< @todo fill in documentation here for it to show up in sphinx
    optional<int> sync_pulse_out_pulse_width; ///< @todo fill in documentation here for it to show up in sphinx
    optional<int> sync_pulse_out_frequency; ///< @todo fill in documentation here for it to show up in sphinx

    optional<bool> phase_lock_enable; ///< @todo fill in documentation here for it to show up in sphinx
    optional<int> phase_lock_offset; ///< @todo fill in documentation here for it to show up in sphinx

    optional<int> columns_per_packet; ///< @todo fill in documentation here for it to show up in sphinx
    optional<UDPProfileLidar> udp_profile_lidar; ///< @todo fill in documentation here for it to show up in sphinx
    optional<UDPProfileIMU> udp_profile_imu; ///< @todo fill in documentation here for it to show up in sphinx
};

struct data_format {
    uint32_t pixels_per_column; ///< @todo fill in documentation here for it to show up in sphinx
    uint32_t columns_per_packet; ///< @todo fill in documentation here for it to show up in sphixn
    uint32_t columns_per_frame; ///< @todo fill in documentation here for it to show up in sphinx
    std::vector<int> pixel_shift_by_row; ///< @todo fill in documentation here for it to show up in sphinx
    ColumnWindow column_window; ///< @todo fill in documentation here for it to show up in sphinx
    UDPProfileLidar udp_profile_lidar; ///< @todo fill in documentation here for it to show up in sphinx
    UDPProfileIMU udp_profile_imu; ///< @todo fill in documentation here for it to show up in sphinx
};

struct sensor_info {
    [[deprecated]] std::string name; ///< @todo fill in documentation here for it to show up in sphinx
    std::string sn; ///< @todo fill in documentation here for it to show up in sphinx
    std::string fw_rev; ///< @todo fill in documentation here for it to show up in sphinx
    lidar_mode mode; ///< @todo fill in documentation here for it to show up in sphinx
    std::string prod_line; ///< @todo fill in documentation here for it to show up in sphinx
    data_format format; ///< @todo fill in documentation here for it to show up in sphinx
    std::vector<double> beam_azimuth_angles; ///< @todo fill in documentation here for it to show up in sphinx
    std::vector<double> beam_altitude_angles;///< @todo fill in documentation here for it to show up in sphinx 
    double lidar_origin_to_beam_origin_mm; ///< @todo fill in documentation here for it to show up in sphinx
    mat4d imu_to_sensor_transform; ///< @todo fill in documentation here for it to show up in sphinx
    mat4d lidar_to_sensor_transform; ///< @todo fill in documentation here for it to show up in sphinx
    mat4d extrinsic; ///< @todo fill in documentation here for it to show up in sphinx
    uint32_t init_id; ///< @todo fill in documentation here for it to show up in sphinx
    uint16_t udp_port_lidar; ///< @todo fill in documentation here for it to show up in sphinx
    uint16_t udp_port_imu; ///< @todo fill in documentation here for it to show up in sphinx
};

/** \defgroup ouster_client_types_operators Ouster Client types.h Operators
 * @{
 */

/** Equality for data_format */
bool operator==(const data_format& lhs, const data_format& rhs);
/** Not-Equality for data_format */
bool operator!=(const data_format& lhs, const data_format& rhs);

/** Equality for sensor_info */
bool operator==(const sensor_info& lhs, const sensor_info& rhs);
/** Not-Equality for sensor_info */
bool operator!=(const sensor_info& lhs, const sensor_info& rhs);

/** Equality for sensor config */
bool operator==(const sensor_config& lhs, const sensor_config& rhs);
/** Not-Equality for sensor config */
bool operator!=(const sensor_config& lhs, const sensor_config& rhs);

/** @}*/

/** \defgroup ouster_sensor_types_to_string Ouster Client types.h to_string functions
 */

/** \defgroup ouster_sensor_types_of_string Ouster Client types.h of string functions
 */

/**
 * Get a default sensor_info for the given lidar mode.
 *
 * @param lidar_mode
 * @return default sensor_info for the OS1-64
 */
sensor_info default_sensor_info(lidar_mode mode);

/**
 * Get string representation of a lidar mode.
 *
 * @ingroup ouster_sensor_types_to_string
 * @param lidar_mode
 * @return string representation of the lidar mode, or "UNKNOWN"
 */
std::string to_string(lidar_mode mode);

/**
 * Get lidar mode from string.
 *
 * @ingroup ouster_sensor_types_of_string
 * @param string
 * @return lidar mode corresponding to the string, or 0 on error
 */
lidar_mode lidar_mode_of_string(const std::string& s);

/**
 * Get number of columns in a scan for a lidar mode.
 *
 * @param lidar_mode
 * @return number of columns per rotation for the mode
 */
uint32_t n_cols_of_lidar_mode(lidar_mode mode);

/**
 * Get the lidar rotation frequency from lidar mode.
 *
 * @param lidar_mode
 * @return lidar rotation frequency in Hz
 */
int frequency_of_lidar_mode(lidar_mode mode);

/**
 * Get string representation of a timestamp mode.
 *
 * @ingroup ouster_sensor_types_to_string
 * @param timestamp_mode
 * @return string representation of the timestamp mode, or "UNKNOWN"
 */
std::string to_string(timestamp_mode mode);

/**
 * Get timestamp mode from string.
 *
 * @ingroup ouster_sensor_types_of_string
 * @param string
 * @return timestamp mode corresponding to the string, or 0 on error
 */
timestamp_mode timestamp_mode_of_string(const std::string& s);

/**
 * Get string representation of an operating mode.
 *
 * @ingroup ouster_sensor_types_to_string
 * @param mode
 * @return string representation of the operating mode, or "UNKNOWN"
 */
std::string to_string(OperatingMode mode);

/**
 * Get operating mode from string.
 *
 * @ingroup ouster_sensor_types_of_string
 * @param string
 * @return operating mode corresponding to the string, or 0 on error
 */
optional<OperatingMode> operating_mode_of_string(const std::string& s);

/**
 * Get string representation of a multipurpose io mode.
 *
 * @ingroup ouster_sensor_types_to_string
 * @param mode
 * @return string representation of the multipurpose io mode, or "UNKNOWN"
 */
std::string to_string(MultipurposeIOMode mode);

/**
 * Get multipurpose io mode from string.
 *
 * @ingroup ouster_sensor_types_of_string
 * @param string
 * @return multipurpose io mode corresponding to the string, or 0 on error
 */
optional<MultipurposeIOMode> multipurpose_io_mode_of_string(
    const std::string& s);

/**
 * Get string representation of a polarity.
 *
 * @ingroup ouster_sensor_types_to_string
 * @param polarity
 * @return string representation of the polarity, or "UNKNOWN"
 */
std::string to_string(Polarity polarity);

/**
 * Get polarity from string.
 *
 * @ingroup ouster_sensor_types_of_string
 * @param string
 * @return polarity corresponding to the string, or 0 on error
 */
optional<Polarity> polarity_of_string(const std::string& s);

/**
 * Get string representation of a NMEA Baud Rate
 *
 * @ingroup ouster_sensor_types_to_string
 * @param rate
 * @return string representation of the NMEA baud rate, or "UNKNOWN"
 */
std::string to_string(NMEABaudRate rate);

/**
 * Get nmea baud rate from string.
 *
 * @ingroup ouster_sensor_types_of_string
 * @param string
 * @return nmea baud rate corresponding to the string, or 0 on error
 */
optional<NMEABaudRate> nmea_baud_rate_of_string(const std::string& s);

/**
 * Get string representation of an Azimuth Window
 *
 * @ingroup ouster_sensor_types_to_string
 * @param azimuth_window
 * @return string representation of the azimuth window
 */
std::string to_string(AzimuthWindow azimuth_window);

/**
 * Get string representation of a lidar profile
 *
 * @ingroup ouster_sensor_types_to_string
 * @param packet profile
 * @return string representation of the lidar profile
 */
std::string to_string(UDPProfileLidar profile);

/**
 * Get lidar profile from string
 *
 * @ingroup ouster_sensor_types_of_string
 * @param string
 * @return lidar profile corresponding to the string, or nullopt on error
 */
optional<UDPProfileLidar> udp_profile_lidar_of_string(const std::string& s);

/**
 * Get string representation of an IMU profile
 *
 * @ingroup ouster_sensor_types_to_string
 * @param packet profile
 * @return string representation of the lidar profile
 */
std::string to_string(UDPProfileIMU profile);

/**
 * Get imu profile from string
 *
 * @ingroup ouster_sensor_types_of_string
 * @param string
 * @return imu profile corresponding to the string, or nullopt on error
 */
optional<UDPProfileIMU> udp_profile_imu_of_string(const std::string& s);

/**
 * Parse metadata text blob from the sensor into a sensor_info struct.
 *
 * String and vector fields will have size 0 if the parameter cannot
 * be found or parsed, while lidar_mode will be set to 0 (invalid).
 *
 * @throw runtime_error if the text is not valid json
 * @param metadata a text blob returned by get_metadata from client.h
 * @return a sensor_info struct populated with a subset of the metadata
 */
sensor_info parse_metadata(const std::string& metadata);

/**
 * Parse metadata given path to a json file.
 *
 * @throw runtime_error if json file does not exist or is malformed
 * @param json_file path to a json file containing sensor metadata
 * @return a sensor_info struct populated with a subset of the metadata
 */
sensor_info metadata_from_json(const std::string& json_file);

/**
 * Get a string representation of metadata. All fields included.
 *
 * @ingroup ouster_sensor_types_to_string
 * @param info sensor_info struct
 * @return a json metadata string
 */
std::string to_string(const sensor_info& info);

/**
 * Parse config text blob from the sensor into a sensor_config struct
 *
 * All fields are optional, and will only be set if found.
 *
 * @throw runtime_error if the text is not valid json
 * @param metadata a text blob given by get_config from client.h
 * @return a sensor_config struct populated with the sensor config
 * parameters
 */
sensor_config parse_config(const std::string& config);

/**
 * Get a string representation of sensor config. Only set fields will be
 * represented.
 *
 * @ingroup ouster_sensor_types_to_string
 * @param config a struct of sensor config
 * @return a json sensor config string
 */
std::string to_string(const sensor_config& config);

/**
 * Convert non-legacy string representation of metadata to legacy
 *
 * @param metadata non-legacy string representation of metadata
 * @return legacy string representation of metadata
 */
std::string convert_to_legacy(const std::string& metadata);
/** @}*/

/**
 * Get client version
 *
 * @return client version string
 */
std::string client_version();

/**
 * Tag to identitify a paricular value reported in the sensor channel data block
 */
enum ChanField {
    RANGE = 1, ///< @todo document me
    RANGE2 = 2, ///< @todo document me
    INTENSITY = 3,  // deprecated (gcc 5.4 doesn't support annotations here)
    SIGNAL = 3, ///< @todo document me
    SIGNAL2 = 4, ///< @todo document me
    REFLECTIVITY = 5, ///< @todo document me
    REFLECTIVITY2 = 6, ///< @todo document me
    AMBIENT = 7,  // deprecated
    NEAR_IR = 7, ///< @todo document me
    FLAGS = 8, ///< @todo document me
    FLAGS2 = 9, ///< @todo document me
    RAW32_WORD1 = 60, ///< @todo document me
    RAW32_WORD2 = 61, ///< @todo document me
    RAW32_WORD3 = 62, ///< @todo document me
    RAW32_WORD4 = 63, ///< @todo document me
    CHAN_FIELD_MAX = 64, ///< @todo document me
};

/**
 * Get string representation of a channel field
 *
 * @ingroup ouster_sensor_types_to_string
 * @param field
 * @return string representation of the channel field
 */
std::string to_string(ChanField field);

/**
 * Types of channel fields
 */
enum ChanFieldType { VOID = 0, UINT8, UINT16, UINT32, UINT64 };

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
class packet_format final {
    packet_format(const sensor_info& info);

    template <typename T>
    T px_field(const uint8_t* px_buf, ChanField i) const;

    struct Impl;
    std::shared_ptr<const Impl> impl_;

    std::vector<std::pair<sensor::ChanField, sensor::ChanFieldType>>
        field_types_;

   public:
    using FieldIter = decltype(field_types_)::const_iterator;

    const UDPProfileLidar udp_profile_lidar; ///< @todo document me
    const size_t lidar_packet_size; ///< @todo document me
    const size_t imu_packet_size; ///< @todo document me
    const int columns_per_packet; ///< @todo document me
    const int pixels_per_column; ///< @todo document me
    [[deprecated]] const int encoder_ticks_per_rev;

    /**
     * Read the packet type packet header
     */
    uint16_t packet_type(const uint8_t* lidar_buf) const;

    /**
     * Read the frame_id packet header
     */
    uint16_t frame_id(const uint8_t* lidar_buf) const;

    /**
     * Read the initialization id packet header
     */
    uint32_t init_id(const uint8_t* lidar_buf) const;

    /**
     * Read the packet serial number header
     */
    uint64_t prod_sn(const uint8_t* lidar_buf) const;

    /**
     * Get the bit width of the specified channel field.
     *
     * @param field the channel field to query
     * @return a type tag specifying the bitwidth of the requested field or
     * ChannelFieldType::VOID if it is not supported by the packet format
     */
    ChanFieldType field_type(ChanField f) const;

    /**
     * A const forward iterator over field / type pairs
     */
    FieldIter begin() const;
    FieldIter end() const;

    // Measurement block accessors
    const uint8_t* nth_col(int n, const uint8_t* lidar_buf) const; ///< @todo document me
    uint64_t col_timestamp(const uint8_t* col_buf) const; ///< @todo document me
    uint16_t col_measurement_id(const uint8_t* col_buf) const; ///< @todo document me
    uint32_t col_status(const uint8_t* col_buf) const; ///< @todo document me
    [[deprecated]] uint32_t col_encoder(const uint8_t* col_buf) const;
    [[deprecated]] uint16_t col_frame_id(const uint8_t* col_buf) const;

    /**
     * Copy the specified channel field out of a packet measurement block.
     *
     * T should be an unsigned integer type large enough to store values of the
     * specified field. Otherwise, data will be truncated.
     *
     * @param col_buf a measurement block pointer returned by `nth_col()`
     * @param field the channel field to copy
     * @param[out] dst destination array of size pixels_per_column * dst_stride
     * @param dst_stride stride for writing to the destination array
     */
    template <typename T,
              typename std::enable_if<std::is_unsigned<T>::value, T>::type = 0>
    void col_field(const uint8_t* col_buf, ChanField f, T* dst,
                   int dst_stride = 1) const; ///< @todo document me

    // Per-pixel channel data block accessors
    const uint8_t* nth_px(int n, const uint8_t* col_buf) const; ///< @todo document me
    uint32_t px_range(const uint8_t* px_buf) const; ///< @todo document me
    uint16_t px_reflectivity(const uint8_t* px_buf) const; ///< @todo document me
    uint16_t px_signal(const uint8_t* px_buf) const; ///< @todo document me
    uint16_t px_ambient(const uint8_t* px_buf) const; ///< @todo document me

    // IMU packet accessors
    uint64_t imu_sys_ts(const uint8_t* imu_buf) const; ///< @todo document me
    uint64_t imu_accel_ts(const uint8_t* imu_buf) const; ///< @todo document me
    uint64_t imu_gyro_ts(const uint8_t* imu_buf) const; ///< @todo document me
    float imu_la_x(const uint8_t* imu_buf) const; ///< @todo document me
    float imu_la_y(const uint8_t* imu_buf) const; ///< @todo document me
    float imu_la_z(const uint8_t* imu_buf) const; ///< @todo document me
    float imu_av_x(const uint8_t* imu_buf) const; ///< @todo document me
    float imu_av_y(const uint8_t* imu_buf) const; ///< @todo document me
    float imu_av_z(const uint8_t* imu_buf) const; ///< @todo document me

    friend const packet_format& get_format(const sensor_info&); ///< @todo document me
};

/**
 * Get a packet parser for a particular data format.
 *
 * @param data_format parameters provided by the sensor
 * @return a packet_format suitable for parsing UDP packets sent by the sensor
 */
const packet_format& get_format(const sensor_info& info);

}  // namespace sensor
}  // namespace ouster

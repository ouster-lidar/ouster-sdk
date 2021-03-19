/**
 * @file
 * @brief Ouster client datatypes and constants
 */

#pragma once

#include <Eigen/Eigen>
#include <string>
#include <vector>

namespace ouster {

/** For image operations. */
template <typename T>
using img_t = Eigen::Array<T, -1, -1, Eigen::RowMajor>;

/** Used for transformations */
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

enum lidar_mode {
    MODE_UNSPEC = 0,
    MODE_512x10,
    MODE_512x20,
    MODE_1024x10,
    MODE_1024x20,
    MODE_2048x10
};

enum timestamp_mode {
    TIME_FROM_UNSPEC = 0,
    TIME_FROM_INTERNAL_OSC,
    TIME_FROM_SYNC_PULSE_IN,
    TIME_FROM_PTP_1588
};

enum multipurpose_io_mode {
  mio_mode_UNSPEC = 0,
  mio_mode_OFF = 1,
  mio_INPUT_NMEA_UART,
  mio_OUTPUT_FROM_INTERNAL_OSC,
  mio_OUTPUT_FROM_SYNC_PULSE_IN,
  mio_OUTPUT_FROM_PTP_1588,
  mio_OUTPUT_FROM_ENCODER_ANGLE
};

enum nmea_baud_rate {
  BAUD_UNSPEC = 0,
  BAUD_9600 = 1,
  BAUD_115200
};

enum nmea_in_polarity {
  nmea_polarity_UNSPEC = 0,
  nmea_polarity_ACTIVE_HIGH = 1,
  nmea_polarity_ACTIVE_LOW
};

enum sync_pulse_in_polarity {
  sync_pulse_in_UNSPEC = 0,
  sync_pulse_in_ACTIVE_HIGH = 1,
  sync_pulse_in_ACTIVE_LOW
};

enum nmea_ignore_valid_char{
  nmea_val_char_UNSPEC = 0,
  nmea_val_char_ignore = 1,
  nmea_val_char_not_ignore
};


enum configuration_version {
    FW_2_0 = 3
};

struct data_format {
    uint32_t pixels_per_column;
    uint32_t columns_per_packet;
    uint32_t columns_per_frame;
    std::vector<int> pixel_shift_by_row;
};

struct sensor_info {
    std::string name;
    std::string sn;
    std::string fw_rev;
    lidar_mode mode;
    std::string prod_line;
    data_format format;
    std::vector<double> beam_azimuth_angles;
    std::vector<double> beam_altitude_angles;
    double lidar_origin_to_beam_origin_mm;
    mat4d imu_to_sensor_transform;
    mat4d lidar_to_sensor_transform;
    mat4d extrinsic;
};

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
 * @param lidar_mode
 * @return string representation of the lidar mode, or "UNKNOWN"
 */
std::string to_string(lidar_mode mode);

/**
 * Get lidar mode from string.
 *
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
 * @param timestamp_mode
 * @return string representation of the timestamp mode, or "UNKNOWN"
 */
std::string to_string(timestamp_mode mode);

/**
 * Get timestamp mode from string.
 *
 * @param string
 * @return timestamp mode corresponding to the string, or 0 on error
 */
timestamp_mode timestamp_mode_of_string(const std::string& s);

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
 * Get string representation of metadata.
 *
 * @param metadata a struct of sensor metadata
 * @return a json metadata string
 */
std::string to_string(const sensor_info& metadata);

/**
 * Get string representation of a multipurpose_io_mode
 * @param multipurpose_io_mode
 * @return string representation of the multipurpose_io_mode
 */
std::string to_string(multipurpose_io_mode v);

/**
 * Get multipurpose_io_mode from string
 * @param string
 * @return multipurpose_io_mode corresponding to the string, or 0 on error
 */
multipurpose_io_mode multipurpose_io_mode_of_string(const std::string& s);

/**
 * Get string representation of a nmea_baud_rate
 * @param nmea_baud_rate
 * @return string representation of the nmea_baud_rate
 */
std::string to_string(nmea_baud_rate v);

/**
 * Get nmea_baud_rate from string
 * @param string
 * @return nmea_baud_rate corresponding to the string, or 0 on error
 */
nmea_baud_rate nmea_baud_rate_of_string(const std::string& s);

/**
 * Get string representation of a nmea_in_polarity
 * @param nmea_in_polarity
 * @return string representation of the nmea_in_polarity
 */
std::string to_string(nmea_in_polarity v);

/**
 * Get nmea_in_polarity from string
 * @param string
 * @return nmea_in_polarity corresponding to the string, or 0 on error
 */
nmea_in_polarity nmea_in_polarity_of_string(const std::string& s);

/**
 * Get string representation of a sync_pulse_in_polarity
 * @param sync_pulse_in_polarity
 * @return string representation of the sync_pulse_in_polarity
 */
std::string to_string(sync_pulse_in_polarity v);

/**
 * Get sync_pulse_in_polarity from string
 * @param string
 * @return sync_pulse_in_polarity corresponding to the string, or 0 on error
 */
sync_pulse_in_polarity sync_pulse_in_polarity_of_string(const std::string& s);


/**
 * Get string representation of a nmea_ignore_valid_char
 * @param nmea_ignore_valid_char
 * @return string representation of the nmea_ignore_valid_char
 */
std::string to_string(nmea_ignore_valid_char v);

/**
 * Get nmea_ignore_valid_char from string
 * @param string
 * @return nmea_ignore_valid_char corresponding to the string, or 0 on error
 */
nmea_ignore_valid_char nmea_ignore_valid_char_of_string(const std::string& s);

/**
 * Get nmea_leap_seconds from string
 * @param string
 * @return nmea_leap_seconds corresponding to the string, or -999999 on error
 */
int nmea_leap_seconds_of_string(const std::string& s);

/**
 * Get azimuth_window value from string
 * @param string
 * @return azimuth_window corresponding to the string
 */
int azimuth_window_of_string(const std::string& s);

/**
 * Get string representation of a azimuth_window_start and azimuth_window_end
 * @param int azimuth_window_start, int azimuth_window_end
 * @return string representation of the azimuth_window
 */
std::string azimuth_window_to_string(const int start, const int end);

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
struct packet_format {
    const size_t lidar_packet_size;
    const size_t imu_packet_size;
    const int columns_per_packet;
    const int pixels_per_column;
    const int encoder_ticks_per_rev;

    // Measurement block accessors
    const uint8_t* (*const nth_col)(int n, const uint8_t* lidar_buf);
    uint64_t (*const col_timestamp)(const uint8_t* col_buf);
    uint32_t (*const col_encoder)(const uint8_t* col_buf);
    uint16_t (*const col_measurement_id)(const uint8_t* col_buf);
    uint16_t (*const col_frame_id)(const uint8_t* col_buf);
    uint32_t (*const col_status)(const uint8_t* col_buf);

    // Channel data block accessors
    const uint8_t* (*const nth_px)(int n, const uint8_t* col_buf);
    uint32_t (*const px_range)(const uint8_t* px_buf);
    uint16_t (*const px_reflectivity)(const uint8_t* px_buf);
    uint16_t (*const px_signal)(const uint8_t* px_buf);
    uint16_t (*const px_ambient)(const uint8_t* px_buf);

    // IMU packet accessors
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

/**
 * Get a packet parser for a particular data format.
 *
 * @param data_format parameters provided by the sensor
 * @returns a packet_format suitable for parsing UDP packets sent by the sensor
 */
const packet_format& get_format(const sensor_info& info);

}  // namespace sensor
}  // namespace ouster

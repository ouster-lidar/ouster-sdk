/**
 * @file
 * @brief Ouster client datatypes and constants
 */

#pragma once

#include <string>
#include <vector>

namespace ouster {
namespace sensor {

/**
 * Unit of range from sensor packet, in meters
 */
constexpr double range_unit = 0.001;

/**
 * Design values for altitude and azimuth offset angles for gen1 sensors. Can be
 * used if calibrated values are not available.
 */
extern const std::vector<double> gen1_altitude_angles;
extern const std::vector<double> gen1_azimuth_angles;

/**
 * Design values for imu and lidar to sensor-frame transforms. See the manual
 * for details.
 */
extern const std::vector<double> imu_to_sensor_transform;
extern const std::vector<double> lidar_to_sensor_transform;

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

struct data_format {
    uint32_t pixels_per_column;
    uint32_t columns_per_packet;
    uint32_t columns_per_frame;
    std::vector<int> pixel_shift_by_row;
};

/**
 * @param lidar_mode
 * @return data format struct for gen1, < 1.14 fw devices
 */
data_format default_data_format(lidar_mode mode);

struct sensor_info {
    std::string hostname;
    std::string sn;
    std::string fw_rev;
    lidar_mode mode;
    std::string prod_line;
    data_format format;
    std::vector<double> beam_azimuth_angles;
    std::vector<double> beam_altitude_angles;
    std::vector<double> imu_to_sensor_transform;
    std::vector<double> lidar_to_sensor_transform;
    double lidar_origin_to_beam_origin_mm;
};

/**
 * @return default sensor_info
 */
sensor_info default_sensor_info();

/**
 * Get string representation of a lidar mode
 * @param lidar_mode
 * @return string representation of the lidar mode, or "UNKNOWN"
 */
std::string to_string(lidar_mode mode);

/**
 * Get lidar mode from string
 * @param string
 * @return lidar mode corresponding to the string, or 0 on error
 */
lidar_mode lidar_mode_of_string(const std::string& s);

/**
 * Get number of columns in a scan for a lidar mode
 * @param lidar_mode
 * @return number of columns per rotation for the mode
 */
int n_cols_of_lidar_mode(lidar_mode mode);

/**
 * Get string representation of a timestamp mode
 * @param timestamp_mode
 * @return string representation of the timestamp mode, or "UNKNOWN"
 */
std::string to_string(timestamp_mode mode);

/**
 * Get timestamp mode from string
 * @param string
 * @return timestamp mode corresponding to the string, or 0 on error
 */
timestamp_mode timestamp_mode_of_string(const std::string& s);

/**
 * Parse metadata text blob from the sensor into a sensor_info struct. String
 * and vector fields will have size 0 if the parameter cannot be found or
 * parsed, while lidar_mode will be set to 0 (invalid)
 * @throw runtime_error if the text is not valid json
 * @param metadata a text blob returned by get_metadata above
 * @return a sensor_info struct populated with a subset of the metadata
 */
sensor_info parse_metadata(const std::string& metadata);

/**
 * Get string representation of metadata
 * @param metadata a struct of sensor metadata
 * @return a json metadata string
 */
std::string to_string(const sensor_info& metadata);

/**
 * A packet_format is a table of accessors and indices for extracting data
 * from imu and lidar packets.
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

/**
 * Get a packet parser for a particular data format
 * @param data_format parameters provided by the sensor
 * @returns a packet_format suitable for parsing UDP packets sent by the sensor
 */
const packet_format& get_format(const data_format& format);

}  // namespace sensor
}  // namespace ouster

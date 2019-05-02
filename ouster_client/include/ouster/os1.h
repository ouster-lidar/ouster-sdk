/**
 * @file
 * @brief OS-1 sample client
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ouster {
namespace OS1 {

const size_t lidar_packet_bytes = 12608;
const size_t imu_packet_bytes = 48;

struct client;

enum client_state {
    TIMEOUT = 0,
    ERROR = 1,
    LIDAR_DATA = 2,
    IMU_DATA = 4,
    EXIT = 8
};

enum lidar_mode {
    MODE_512x10 = 1,
    MODE_512x20,
    MODE_1024x10,
    MODE_1024x20,
    MODE_2048x10
};

struct version {
    int16_t major;
    int16_t minor;
    int16_t patch;
};

const version invalid_version = {0, 0, 0};

/**
 * Minimum supported version
 */
const OS1::version min_version = {1, 9, 0};

inline bool operator==(const version& u, const version& v) {
    return u.major == v.major && u.minor == v.minor && u.patch == v.patch;
}

inline bool operator<(const version& u, const version& v) {
    return (u.major < v.major) || (u.major == v.major && u.minor < v.minor) ||
           (u.major == v.major && u.minor == v.minor && u.patch < v.patch);
}

inline bool operator<=(const version& u, const version& v) {
    return u < v || u == v;
}

struct sensor_info {
    std::string hostname;
    std::string sn;
    std::string fw_rev;
    lidar_mode mode;
    std::vector<double> beam_azimuth_angles;
    std::vector<double> beam_altitude_angles;
    std::vector<double> imu_to_sensor_transform;
    std::vector<double> lidar_to_sensor_transform;
};

/**
 * Get string representation of a version
 * @param version
 * @return string representation of the version
 */
std::string to_string(version v);

/**
 * Get lidar mode from string
 * @param string
 * @return lidar mode corresponding to the string, or invalid_version on error
 */
version version_of_string(const std::string& s);

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
 * Listen for OS1 data on the specified ports
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
std::shared_ptr<client> init_client(int lidar_port = 7502, int imu_port = 7503);

/**
 * Connect to and configure the sensor and start listening for data
 * @param hostname hostname or ip of the sensor
 * @param udp_dest_host hostname or ip where the sensor should send data
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode mode = MODE_1024x10,
                                    int lidar_port = 7502, int imu_port = 7503);

/**
 * Block for up to timeout_sec until either data is ready or an error occurs.
 * @param cli client returned by init_client associated with the connection
 * @param timeout_sec seconds to block while waiting for data
 * @return client_state s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read, and (s & IMU_DATA) is
 * true if imu data is ready to read
 */
client_state poll_client(const client& cli, int timeout_sec = 1);

/**
 * Read lidar data from the sensor. Will not block.
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @return true if a lidar packet was successfully read
 */
bool read_lidar_packet(const client& cli, uint8_t* buf);

/**
 * Read imu data from the sensor. Will not block.
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write imu data. Must be at least
 * imu_packet_bytes + 1 bytes
 * @return true if an imu packet was successfully read
 */
bool read_imu_packet(const client& cli, uint8_t* buf);

/**
 * Get metadata text blob from the sensor
 * @param cli client returned by init_client associated with the connection
 * @return a text blob of metadata parseable into a sensor_info struct
 */
std::string get_metadata(const client& cli);

/**
 * Parse metadata text blob from the sensor into a sensor_info struct. String
 * and vector fields will have size 0 if the parameter cannot be found or
 * parsed,
 * while lidar_mode will be set to 0 (invalid).
 * @throw runtime_error if the text is not valid json
 * @param metadata a text blob returned by get_metadata above
 * @return a sensor_info struct populated with a subset of the metadata
 */
sensor_info parse_metadata(const std::string& metadata);
}
}

/**
 * @file
 * @brief OS-1 sample client
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "version.h"
#include "os1_packet.h"

namespace ouster {
namespace OS1 {

struct client;

enum client_state {
    TIMEOUT = 0,
    CLIENT_ERROR = 1,
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


/**
 * Minimum supported version
 */
const OS1::version min_version = {1, 12, 0};

struct data_format {
    uint32_t pixels_per_column;
    uint32_t columns_per_packet;
    uint32_t columns_per_frame;
    std::vector<uint32_t> pixel_shift_by_row;
};

/**
 * @param lidar_mode
 * @return data format struct for gen1, < 1.14 fw devices
 */
data_format default_data_format(lidar_mode mode);

/**
 * @param prod_line
 * @return beam_nodal_point_radius for < 1.14 fw devices
 */
double default_lidar_origin_to_beam_origin(std::string prod_line);

struct sensor_info {
    std::string hostname;
    std::string sn;
    std::string fw_rev;
    lidar_mode mode;
    std::string prod_line;
    data_format format;
    double lidar_origin_to_beam_origin_mm;
    std::vector<double> beam_azimuth_angles;
    std::vector<double> beam_altitude_angles;
    std::vector<double> imu_to_sensor_transform;
    std::vector<double> lidar_to_sensor_transform;
};

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
 * Listen for OS1 data on the specified ports; do not configure the sensor
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
std::shared_ptr<client> init_client(const std::string& hostname = "",
                                    int lidar_port = 7502, int imu_port = 7503);

/**
 * Connect to and configure the sensor and start listening for data
 * @param hostname hostname or ip of the sensor
 * @param udp_dest_host hostname or ip where the sensor should send data
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @param timeout_sec how long to wait for the sensor to initialize
 * @return pointer owning the resources associated with the connection
 */
std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode mode = MODE_1024x10,
                                    int lidar_port = 0, int imu_port = 0,
                                    int timeout_sec = 30);

/**
 * Block for up to timeout_sec until either data is ready or an error occurs
 * @param cli client returned by init_client associated with the connection
 * @param timeout_sec seconds to block while waiting for data
 * @return client_state s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read, and (s & IMU_DATA) is
 * true if imu data is ready to read
 */
client_state poll_client(const client& cli, int timeout_sec = 1);

/**
 * Read lidar data from the sensor. Will not block
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @return true if a lidar packet was successfully read
 */
bool read_lidar_packet(const client& cli, uint8_t* buf,
                       const packet_format& pf);

/**
 * Read imu data from the sensor. Will not block
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write imu data. Must be at least
 * imu_packet_bytes + 1 bytes
 * @return true if an imu packet was successfully read
 */
bool read_imu_packet(const client& cli, uint8_t* buf, const packet_format& pf);

/**
 * Get metadata text blob from the sensor. Attempt to fetch from the network if
 * not already populated
 * @param cli client returned by init_client associated with the connection
 * @param timeout_sec how long to wait for the sensor to initialize
 * @return a text blob of metadata parseable into a sensor_info struct
 */
std::string get_metadata(client& cli, int timeout_sec = 30);

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
 * Get a packet parser for a particular data format
 * @param data_format parameters provided by the sensor
 * @returns a packet_format suitable for parsing UDP packets sent by the sensor
 * TODO: rename. Sensor fw chose "get_data_format" for tcp command, which is
 * easily confused with "packet_format"
 */
inline const packet_format& get_format(const data_format& format) {
    switch (format.pixels_per_column) {
        case 16:
            return packet_1_14_0_16;
        case 32:
            return packet_1_14_0_32;
        case 64:
            return packet_1_14_0_64;
        case 128:
            return packet_1_14_0_128;
        default:
            return packet_1_13_0;
    }
}

}
}

/**
 * @file
 * @brief sample sensor client
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ouster/types.h"
#include "ouster/version.h"

namespace ouster {
namespace sensor {

struct client;

enum client_state {
    TIMEOUT = 0,
    CLIENT_ERROR = 1,
    LIDAR_DATA = 2,
    IMU_DATA = 4,
    EXIT = 8
};

/** Minimum supported version. */
const util::version min_version = {1, 12, 0};

/**
 * Listen for sensor data on the specified ports; do not configure the sensor.
 *
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
std::shared_ptr<client> init_client(const std::string& hostname = "",
                                    int lidar_port = 7502, int imu_port = 7503);

/**
 * Connect to and configure the sensor and start listening for data.
 *
 * @param hostname hostname or ip of the sensor
 * @param udp_dest_host hostname or ip where the sensor should send data
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @param timeout_sec how long to wait for the sensor to initialize
 * @return pointer owning the resources associated with the connection
 */
std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode mode = MODE_UNSPEC,
                                    timestamp_mode ts_mode = TIME_FROM_UNSPEC,
                                    int lidar_port = 0, int imu_port = 0,
                                    int timeout_sec = 30);

/**
 * Block for up to timeout_sec until either data is ready or an error occurs.
 *
 * NOTE: will return immediately if LIDAR_DATA or IMU_DATA are set and not
 * cleared by read_lidar_data() and read_imu_data() before the next call
 * @param cli client returned by init_client associated with the connection
 * @param timeout_sec seconds to block while waiting for data
 * @return client_state s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read, and (s & IMU_DATA) is
 * true if imu data is ready to read
 */
client_state poll_client(const client& cli, int timeout_sec = 1);

/**
 * Read lidar data from the sensor. Will not block.
 *
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @return true if a lidar packet was successfully read
 */
bool read_lidar_packet(const client& cli, uint8_t* buf,
                       const packet_format& pf);

/**
 * Read imu data from the sensor. Will not block.
 *
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write imu data. Must be at least
 * imu_packet_bytes + 1 bytes
 * @return true if an imu packet was successfully read
 */
bool read_imu_packet(const client& cli, uint8_t* buf, const packet_format& pf);

/**
 * Get metadata text blob from the sensor.
 *
 * Will attempt to fetch from the network if not already populated.
 *
 * @param cli client returned by init_client associated with the connection
 * @param timeout_sec how long to wait for the sensor to initialize
 * @return a text blob of metadata parseable into a sensor_info struct
 */
std::string get_metadata(client& cli, int timeout_sec = 30);

}  // namespace sensor
}  // namespace ouster

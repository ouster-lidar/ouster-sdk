/**
 * OS1 sample client
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace ouster {
namespace OS1 {

const size_t lidar_packet_bytes = 12608;
const size_t imu_packet_bytes = 48;

struct client;

enum client_state { ERROR = 1, LIDAR_DATA = 2, IMU_DATA = 4 };

/**
 * Connect to the sensor and start listening for data
 * @param hostname hostname or ip of the sensor
 * @param udp_dest_host hostname or ip where the sensor should send data
 * @param lidar_port port on which the sensor will send lidar data
 * @param imu_port port on which the sensor will send imu data
 * @return pointer owning the resources associated with the connection
 */
std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    int lidar_port, int imu_port);

/**
 * Block for up to a second until either data is ready or an error occurs.
 * @param cli client returned by init_client associated with the connection
 * @return client_state s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read, and (s & IMU_DATA) is
 * true if imu data is ready to read
 */
client_state poll_client(const client& cli);

/**
 * Read lidar data from the sensor. Will block for up to a second if no data is
 * available.
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @returns true if a lidar packet was successfully read
 */
bool read_lidar_packet(const client& cli, uint8_t* buf);

/**
 * Read imu data from the sensor. Will block for up to a second if no data is
 * available.
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write imu data. Must be at least
 * imu_packet_bytes + 1 bytes
 * @returns true if an imu packet was successfully read
 */
bool read_imu_packet(const client& cli, uint8_t* buf);
}
}

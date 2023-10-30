/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief sample sensor client
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "ouster/defaults.h"
#include "ouster/types.h"
#include "ouster/version.h"

namespace ouster {
namespace sensor {

struct client;

/** Returned by poll_client. */
enum client_state {
    TIMEOUT = 0,       ///< Client has timed out
    CLIENT_ERROR = 1,  ///< Client has reported an error
    LIDAR_DATA = 2,    ///< New lidar data available
    IMU_DATA = 4,      ///< New IMU data available
    EXIT = 8           ///< Client has exited
};

/** Minimum supported version. */
const util::version min_version = {1, 12, 0};

/**
 * Initializes and configures ouster_client logs. This method should be invoked
 * only once before calling any other method from the library if the user wants
 * to direct the library log statements to a different medium (other than
 * console which is the default).
 *
 * @param[in] log_level Control the level of log messages outputed by the
 * client. Valid options are (case-sensitive): "trace", "debug", "info",
 * "warning", "error", "critical" and "off".
 * @param[in] log_file_path Path to location where log files are stored. The
 * path must be in a location that the process has write access to. If an empty
 * string is provided then the logs will be directed to the console. When an
 * empty string is passed then the rest of parameters are ignored.
 * @param[in] rotating Configure the log file with rotation, rotation rules are
 * specified through the two following parameters max_size_in_bytes and
 * max_files. If rotating is set to false the following parameters are ignored.
 * @param[in] max_size_in_bytes Maximum number of bytes to write to a rotating
 * log file before starting a new file. ignored if rotating is set to false.
 * @param[in] max_files Maximum number of rotating files to accumlate before
 * re-using the first file. ignored if rotating is set to false.
 *
 * @return true on success, otherwise false.
 */
bool init_logger(const std::string& log_level,
                 const std::string& log_file_path = "", bool rotating = false,
                 int max_size_in_bytes = 0, int max_files = 0);

/** \defgroup ouster_client_init Ouster Client Client Initialization
 * @{
 */

/**
 * Listen for sensor data on the specified ports; do not configure the sensor.
 *
 * @param[in] hostname The hostname to connect to.
 * @param[in] lidar_port port on which the sensor will send lidar data.
 * @param[in] imu_port port on which the sensor will send imu data.
 *
 * @return pointer owning the resources associated with the connection.
 */
std::shared_ptr<client> init_client(const std::string& hostname, int lidar_port,
                                    int imu_port);

/**
 * Connect to and configure the sensor and start listening for data.
 *
 * @param[in] hostname hostname or ip of the sensor.
 * @param[in] udp_dest_host hostname or ip where the sensor should send data
 * or "" for automatic detection of destination.
 * @param[in] ld_mode The lidar mode to use.
 * @param[in] ts_mode The timestamp mode to use.
 * @param[in] lidar_port port on which the sensor will send lidar data. When
 * using zero the method will automatically acquire and assign any free port.
 * @param[in] imu_port port on which the sensor will send imu data. When
 * using zero the method will automatically acquire and assign any free port.
 * @param[in] timeout_sec how long to wait for the sensor to initialize.
 *
 * @return pointer owning the resources associated with the connection.
 */
std::shared_ptr<client> init_client(
    const std::string& hostname, const std::string& udp_dest_host,
    lidar_mode ld_mode = MODE_UNSPEC, timestamp_mode ts_mode = TIME_FROM_UNSPEC,
    int lidar_port = 0, int imu_port = 0,
    int timeout_sec = DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS);

/**
 * [BETA] Connect to and configure the sensor and start listening for data via
 * multicast.
 *
 * @param[in] hostname hostname or ip of the sensor.
 * @param[in] config sensor config to set on sensor.
 * @param[in] mtp_dest_host the address of the host network interface that
 * should join the multicast group; if empty, use any appropriate interface.
 * @param[in] main a flag that indicates this is the main connection to the
 * sensor in an multicast setup.
 * @param[in] timeout_sec how long to wait for the sensor to initialize.
 *
 * @return pointer owning the resources associated with the connection.
 *
 * @remarks when main flag is set the config object will be used to configure
 * the sensor, otherwise only the port values within the config object will be
 * used and the rest will be ignored.
 */
std::shared_ptr<client> mtp_init_client(
    const std::string& hostname, const sensor_config& config,
    const std::string& mtp_dest_host, bool main,
    int timeout_sec = DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS);

/** @}*/

/**
 * Block for up to timeout_sec until either data is ready or an error occurs.
 *
 * NOTE: will return immediately if LIDAR_DATA or IMU_DATA are set and not
 * cleared by read_lidar_data() and read_imu_data() before the next call.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[in] timeout_sec seconds to block while waiting for data.
 *
 * @return client_state s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read, and (s & IMU_DATA) is
 * true if imu data is ready to read.
 */
client_state poll_client(const client& cli, int timeout_sec = 1);

/**
 * Read lidar data from the sensor. Will not block.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[out] buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes.
 * @param[in] pf The packet format.
 *
 * @return true if a lidar packet was successfully read.
 */
bool read_lidar_packet(const client& cli, uint8_t* buf,
                       const packet_format& pf);

/**
 * Read lidar data from the sensor. Will not block.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[out] packet A LidarPacket to store lidar data read from a sensor. In
 * addition, the LidarPacket's host_timestamp attribute is also set.
 * @param[in] pf The packet format.
 *
 * @return true if a lidar packet was successfully read.
 */
bool read_lidar_packet(const client& cli, LidarPacket& packet,
                       const packet_format& pf);

/**
 * Read imu data from the sensor. Will not block.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[out] buf buffer to which to write imu data. Must be at least
 * imu_packet_bytes + 1 bytes.
 * @param[in] pf The packet format.
 *
 * @return true if an imu packet was successfully read.
 */
bool read_imu_packet(const client& cli, uint8_t* buf, const packet_format& pf);

/**
 * Read imu data from the sensor. Will not block.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[out] packet An ImuPacket to store imu data read from a sensor. In
 * addition, the ImuPacket's host_timestamp attribute is also set.
 * imu_packet_bytes + 1 bytes.
 * @param[in] pf The packet format.
 *
 * @return true if an imu packet was successfully read.
 */
bool read_imu_packet(const client& cli, ImuPacket& packet,
                     const packet_format& pf);

/**
 * Get metadata text blob from the sensor.
 *
 * Will attempt to fetch from the network if not already populated.
 *
 * @throw runtime_error if the sensor is in ERROR state, the firmware version
 * used to initialize the HTTP or TCP client is invalid, the metadata could
 * not be retrieved from the sensor within the timeout period,
 * a timeout occured while waiting for the sensor to finish initializing,
 * or the response could not be parsed.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[in] timeout_sec how long to wait for the sensor to initialize.
 * @param[in] legacy_format whether to use legacy format of metadata output.
 *
 * @return a text blob of metadata parseable into a sensor_info struct.
 */
std::string get_metadata(client& cli,
                         int timeout_sec = DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS,
                         bool legacy_format = false);

/**
 * Get sensor config from the sensor.
 *
 * Populates passed in config with the results of get_config.
 *
 * @param[in] hostname sensor hostname.
 * @param[out] config sensor config to populate.
 * @param[in] active whether to pull active or passive configs.
 *
 * @return true if sensor config successfully populated.
 */
bool get_config(const std::string& hostname, sensor_config& config,
                bool active = true,
                int timeout_sec = DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS);

// clang-format off
/**
 * Flags for set_config()
 */
enum config_flags : uint8_t {
    CONFIG_UDP_DEST_AUTO    = (1 << 0), ///< Set udp_dest automatically
    CONFIG_PERSIST          = (1 << 1), ///< Make configuration persistent
    CONFIG_FORCE_REINIT     = (1 << 2)  ///< Forces the sensor to re-init during
                                        ///< set_config even when config params
                                        ///< have not changed
};
// clang-format on

/**
 * Set sensor config on sensor.
 *
 * @throw runtime_error on failure to communcate with the sensor.
 * @throw invalid_argument when config parameters fail validation.
 *
 * @param[in] hostname sensor hostname.
 * @param[in] config sensor config.
 * @param[in] config_flags flags to pass in.
 *
 * @return true if config params successfuly set on sensor.
 */
bool set_config(const std::string& hostname, const sensor_config& config,
                uint8_t config_flags = 0,
                int timeout_sec = DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS);

/**
 * Return the port used to listen for lidar UDP data.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 *
 * @return the port number.
 */
int get_lidar_port(client& cli);

/**
 * Return the port used to listen for imu UDP data.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 *
 * @return the port number.
 */
int get_imu_port(client& cli);

/**
 * Check if ip address in multicast range.
 *
 * @param[in] addr ip address to test.
 *
 * @return true if addr is in multicast range.
 */
bool in_multicast(const std::string& addr);

}  // namespace sensor
}  // namespace ouster

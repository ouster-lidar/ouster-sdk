/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief sample sensor client
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "ouster/core/defaults.h"
#include "ouster/core/packet.h"
#include "ouster/core/types.h"
#include "ouster/core/version.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace sensor {

/** Minimum supported version. */
const auto MIN_VERSION = ouster::sdk::core::Version(1, 12, 0);

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
OUSTER_API_FUNCTION
bool init_logger(const std::string& log_level, const std::string& log_file_path = "",
                 bool rotating = false, int max_size_in_bytes = 0, int max_files = 0);

/**
 * Get sensor config from the sensor.
 *
 * @param[in] hostname sensor hostname.
 * @param[in] active whether to pull active or passive configs.
 * @param[in] timeout_sec set the timeout for the request,
 *                        this argument is optional.
 *
 * @throws std::runtime_error if failed to parse config
 * @return retrieved sensor config
 */
OUSTER_API_FUNCTION
ouster::sdk::core::SensorConfig get_config(const std::string& hostname, bool active = true,
                                           int timeout_sec = LONG_HTTP_REQUEST_TIMEOUT_SECONDS);

// clang-format off
/**
 * Flags for set_config()
 */
enum ConfigFlags : uint8_t {
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
 * @param[in] timeout_sec timeout in seconds for http requests
 */
OUSTER_API_FUNCTION
void set_config(const std::string& hostname, const ouster::sdk::core::SensorConfig& config,
                uint8_t config_flags = 0, int timeout_sec = LONG_HTTP_REQUEST_TIMEOUT_SECONDS);

/**
 * Check if ip address in multicast range.
 *
 * @param[in] addr ip address to test.
 *
 * @return true if addr is in multicast range.
 */
OUSTER_API_FUNCTION
bool in_multicast(const std::string& addr);

}  // namespace sensor
}  // namespace sdk
}  // namespace ouster

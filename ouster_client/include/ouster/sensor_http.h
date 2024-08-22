/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_http.h
 * @brief A high level HTTP interface for Ouster sensors.
 *
 */

#pragma once

#include <json/json.h>
#include <ouster/defaults.h>
#include <ouster/version.h>

#include <memory>

#include "ouster/ouster_client_export.h"

namespace ouster {
namespace sensor {
namespace util {

/**
 * Result for get_user_data_and_policy on SensorHttp
 */
struct UserDataAndPolicy {
    bool keep_on_config_delete;
    std::string data;
};

/**
 * An interface to communicate with Ouster sensors via http requests
 */
class OUSTER_CLIENT_EXPORT SensorHttp {
   protected:
    /**
     * Constructs an http interface to communicate with the sensor.
     */
    SensorHttp() = default;

   public:
    /**
     * Deconstruct the sensor http interface.
     */
    virtual ~SensorHttp() = default;

    /**
     * Queries the sensor metadata.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return returns a Json object of the sensor metadata.
     */
    virtual Json::Value metadata(int timeout_sec = 1) const = 0;

    /**
     * Queries the sensor_info.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return returns a Json object representing the sensor_info.
     */
    virtual Json::Value sensor_info(int timeout_sec = 1) const = 0;

    /**
     * Queries active/staged configuration on the sensor
     *
     * @param[in] active if true retrieve active, otherwise get staged configs.
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return a string represnting the active or staged config
     */
    virtual std::string get_config_params(bool active,
                                          int timeout_sec = 1) const = 0;

    /**
     * Set the value of a specfic configuration on the sensor, the changed
     * configuration is not active until the sensor is restarted.
     *
     * @param[in] key name of the config to change.
     * @param[in] value the new value to set for the selected configuration.
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual void set_config_param(const std::string& key,
                                  const std::string& value,
                                  int timeout_sec = 1) const = 0;

    /**
     * Retrieves the active configuration on the sensor
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual Json::Value active_config_params(int timeout_sec = 1) const = 0;

    /**
     * Retrieves the staged configuration on the sensor
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual Json::Value staged_config_params(int timeout_sec = 1) const = 0;

    /**
     * Enables automatic assignment of udp destination ports.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual void set_udp_dest_auto(int timeout_sec = 1) const = 0;

    /**
     * Retrieves beam intrinsics of the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual Json::Value beam_intrinsics(int timeout_sec = 1) const = 0;

    /**
     * Retrieves imu intrinsics of the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual Json::Value imu_intrinsics(int timeout_sec = 1) const = 0;

    /**
     * Retrieves lidar intrinsics of the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual Json::Value lidar_intrinsics(int timeout_sec = 1) const = 0;

    /**
     * Retrieves lidar data format.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual Json::Value lidar_data_format(int timeout_sec = 1) const = 0;

    /**
     * Gets the calibaration status of the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual Json::Value calibration_status(int timeout_sec = 1) const = 0;

    /**
     * Restarts the sensor applying all staged configurations.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual void reinitialize(int timeout_sec = 1) const = 0;

    /**
     * Persist active configuration parameters to the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual void save_config_params(int timeout_sec = 1) const = 0;

    /**
     * Gets the user data stored on the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual std::string get_user_data(int timeout_sec = 1) const = 0;

    /**
     * Gets the user data stored on the sensor and the retention policy.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual UserDataAndPolicy get_user_data_and_policy(
        int timeout_sec = 1) const = 0;

    /**
     * Sets the user data stored on the sensor.
     *
     * @param[in] data Value of userdata to set on the sensor.
     * @param[in] keep_on_config_delete If true, keep the userdata when
                                        configuration is deleted from the sensor
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual void set_user_data(const std::string& data,
                               bool keep_on_config_delete = true,
                               int timeout_sec = 1) const = 0;

    /**
     * Deletes the user data stored on the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    virtual void delete_user_data(int timeout_sec = 1) const = 0;

    /**
     * Retrieves sensor firmware version information as a string.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     * @param[in] timeout_sec The timeout to use in seconds for the version
     *                        request, this argument is optional.
     */
    static std::string firmware_version_string(
        const std::string& hostname,
        int timeout_sec = DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS);

    /**
     * Retrieves sensor firmware version information.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     * @param[in] timeout_sec The timeout to use in seconds for the version
     *                        request, this argument is optional.
     */
    static ouster::util::version firmware_version(
        const std::string& hostname,
        int timeout_sec = DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS);

    /**
     * Creates an instance of the SensorHttp interface.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     * @param[in] timeout_sec The timeout to use in seconds for the version
     *                        request, this argument is optional.
     */
    static std::unique_ptr<SensorHttp> create(
        const std::string& hostname,
        int timeout_sec = DEFAULT_HTTP_REQUEST_TIMEOUT_SECONDS);
};

}  // namespace util
}  // namespace sensor
}  // namespace ouster

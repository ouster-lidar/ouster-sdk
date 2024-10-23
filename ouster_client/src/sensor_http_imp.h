/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_http_imp.h
 * @brief An implementation of the HTTP interface for Ouster sensors.
 *
 */

#pragma once

#include "http_client.h"
#include "ouster/sensor_http.h"

namespace ouster {
namespace sensor {
namespace impl {

/**
 * An implementation of the sensor http interface
 */
class SensorHttpImp : public util::SensorHttp {
   public:
    /**
     * Constructs an http interface to communicate with the sensor.
     *
     * @param[in] hostname Hostname of the sensor to communicate with.
     */
    SensorHttpImp(const std::string& hostname);

    /**
     * Deconstruct the sensor http interface.
     */
    ~SensorHttpImp() override;

    /**
     * Queries the sensor metadata.
     *
     * @return returns a Json object of the sensor metadata.
     */
    Json::Value metadata(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Queries the sensor_info.
     *
     * @return returns a Json object representing the sensor_info.
     */
    Json::Value sensor_info(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Queries active/staged configuration on the sensor
     *
     * @param[in] active if true retrieve active, otherwise get staged configs.
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return a string represnting the active or staged config
     */
    std::string get_config_params(
        bool active,
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Set the value of a specfic configuration on the sensor, the changed
     * configuration is not active until the sensor is restarted.
     *
     * @param[in] key name of the config to change.
     * @param[in] value the new value to set for the selected configuration.
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    void set_config_param(
        const std::string& key, const std::string& value,
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves the active configuration on the sensor
     */
    Json::Value active_config_params(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves the staged configuration on the sensor
     */
    Json::Value staged_config_params(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Enables automatic assignment of udp destination ports.
     */
    void set_udp_dest_auto(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves beam intrinsics of the sensor.
     */
    Json::Value beam_intrinsics(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves imu intrinsics of the sensor.
     */
    Json::Value imu_intrinsics(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves lidar intrinsics of the sensor.
     */
    Json::Value lidar_intrinsics(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves lidar data format.
     */
    Json::Value lidar_data_format(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Gets the calibaration status of the sensor.
     */
    Json::Value calibration_status(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Restarts the sensor applying all staged configurations.
     */
    void reinitialize(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Persist active configuration parameters to the sensor.
     */
    void save_config_params(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Gets the user data stored on the sensor.
     */
    std::string get_user_data(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Gets the user data stored on the sensor and the retention policy.
     */
    util::UserDataAndPolicy get_user_data_and_policy(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Sets the user data stored on the sensor.
     */
    void set_user_data(
        const std::string& data, bool keep_on_config_delete = true,
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Deletes the user data stored on the sensor.
     */
    void delete_user_data(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Gets sensor IP address information.
     *
     * @param[in] timeout_sec The timeout to use in seconds for the version
     *                        request, this argument is optional.
     *
     * @return a JSON string containing sensor IP address information.
     */
    std::string network(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

   protected:
    std::string get(const std::string& url, int timeout_sec) const;

    Json::Value get_json(const std::string& url, int timeout_sec) const;

    void execute(const std::string& url, const std::string& validation,
                 int timeout_sec) const;

   protected:
    std::unique_ptr<ouster::util::HttpClient> http_client;
};

class SensorHttpImp_2_4_or_3 : public SensorHttpImp {
   public:
    SensorHttpImp_2_4_or_3(const std::string& hostname);

    /**
     * Gets the user data stored on the sensor.
     */
    std::string get_user_data(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Gets the user data stored on the sensor and the retention policy.
     */
    util::UserDataAndPolicy get_user_data_and_policy(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Sets the user data stored on the sensor.
     */
    void set_user_data(
        const std::string& data, bool keep_on_config_delete = true,
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Deletes the user data stored on the sensor.
     */
    void delete_user_data(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;
};

// TODO: remove when firmware 2.2 has been fully phased out
class SensorHttpImp_2_2 : public SensorHttpImp_2_4_or_3 {
   public:
    SensorHttpImp_2_2(const std::string& hostname);

    void set_udp_dest_auto(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;
};

/**
 * An implementation of the sensor http interface
 */
class SensorHttpImp_2_1 : public SensorHttpImp_2_2 {
   public:
    /**
     * Constructs an http interface to communicate with the sensor.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    SensorHttpImp_2_1(const std::string& hostname);

    /**
     * Queries the sensor metadata.
     *
     * @return returns a Json object of the sensor metadata.
     */
    Json::Value metadata(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Queries the sensor_info.
     *
     * @return returns a Json object representing the sensor_info.
     */
    Json::Value sensor_info(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves beam intrinsics of the sensor.
     */
    Json::Value beam_intrinsics(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves imu intrinsics of the sensor.
     */
    Json::Value imu_intrinsics(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves lidar intrinsics of the sensor.
     */
    Json::Value lidar_intrinsics(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Retrieves lidar data format.
     */
    Json::Value lidar_data_format(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;

    /**
     * Gets the calibaration status of the sensor.
     */
    Json::Value calibration_status(
        int timeout_sec = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS) const override;
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

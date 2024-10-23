/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_tcp_imp.h
 * @brief A high level TCP interface for Ouster sensors.
 *
 */

#pragma once

#include "ouster/impl/netcompat.h"
#include "ouster/sensor_http.h"

namespace ouster {
namespace sensor {
namespace impl {

/**
 * A TCP implementation of the SensorHTTP interface
 */
class SensorTcpImp : public util::SensorHttp {
    // timeout for reading from a TCP socket during config
    const int RCVTIMEOUT_SEC = 10;
    // maximum size to to handle during recv
    const size_t MAX_RESULT_LENGTH = 16 * 1024;

   public:
    /**
     * Constructs an tcp interface to communicate with the sensor.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    SensorTcpImp(const std::string& hostname);

    /**
     * Deconstruct the sensor tcp interface.
     */
    ~SensorTcpImp() override;
    /**
     * Queries the sensor metadata.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return returns a Json object of the sensor metadata.
     */
    Json::Value metadata(int timeout_sec = 1) const override;

    /**
     * Queries the sensor_info.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return returns a Json object representing the sensor_info.
     */
    Json::Value sensor_info(int timeout_sec = 1) const override;

    /**
     * Queries active/staged configuration on the sensor
     *
     * @param[in] active if true retrieve active, otherwise get staged configs.
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return a string represnting the active or staged config
     */
    std::string get_config_params(bool active,
                                  int timeout_sec = 1) const override;

    /**
     * Set the value of a specfic configuration on the sensor, the changed
     * configuration is not active until the sensor is restarted.
     *
     * @param[in] key name of the config to change.
     * @param[in] value the new value to set for the selected configuration.
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    void set_config_param(const std::string& key, const std::string& value,
                          int timeout_sec = 1) const override;

    /**
     * Retrieves the active configuration on the sensor
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    Json::Value active_config_params(int timeout_sec = 1) const override;

    /**
     * Retrieves the staged configuration on the sensor
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    Json::Value staged_config_params(int timeout_sec = 1) const override;

    /**
     * Enables automatic assignment of udp destination ports.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    void set_udp_dest_auto(int timeout_sec = 1) const override;

    /**
     * Retrieves beam intrinsics of the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    Json::Value beam_intrinsics(int timeout_sec = 1) const override;

    /**
     * Retrieves imu intrinsics of the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    Json::Value imu_intrinsics(int timeout_sec = 1) const override;

    /**
     * Retrieves lidar intrinsics of the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    Json::Value lidar_intrinsics(int timeout_sec = 1) const override;

    /**
     * Retrieves lidar data format.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    Json::Value lidar_data_format(int timeout_sec = 1) const override;

    /**
     * Gets the calibaration status of the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    Json::Value calibration_status(int timeout_sec = 1) const override;

    /**
     * Restarts the sensor applying all staged configurations.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    void reinitialize(int timeout_sec = 1) const override;

    /**
     * Persist active configuration parameters to the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    void save_config_params(int timeout_sec = 1) const override;

    /**
     * Gets the user data stored on the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    std::string get_user_data(int timeout_sec = 1) const override;

    /**
     * Gets the user data stored on the sensor and the retention policy.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    util::UserDataAndPolicy get_user_data_and_policy(
        int timeout_sec = 1) const override;

    /**
     * Sets the user data stored on the sensor.
     *
     * @param[in] data Value of userdata to set on the sensor.
     * @param[in] keep_on_config_delete If true, keep the userdata when
                                        configuration is deleted from the sensor
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    void set_user_data(const std::string& data,
                       bool keep_on_config_delete = true,
                       int timeout_sec = 1) const override;

    /**
     * Deletes the user data stored on the sensor.
     *
     * @param[in] timeout_sec The timeout for the request in seconds.
     */
    void delete_user_data(int timeout_sec = 1) const override;

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

   private:
    SOCKET cfg_socket(const char* addr);

    std::string tcp_cmd(const std::vector<std::string>& cmd_tokens) const;

    void tcp_cmd_with_validation(const std::vector<std::string>& cmd_tokens,
                                 const std::string& validation) const;

    Json::Value tcp_cmd_json(const std::vector<std::string>& cmd_tokens,
                             bool exception_on_parse_errors = true) const;

   private:
    SOCKET socket_handle;
    std::unique_ptr<char[]> read_buf;
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

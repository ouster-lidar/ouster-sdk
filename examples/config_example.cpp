/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * For more comprehensive explanation, see the Ouster SDK Docs
 *
 * Note: This is an example meant to demonstrate use of the CPP Sensor
 * Configuration API. Users who merely need to set parameters without doing so
 * programmatically may find it easier to do so using the sensor homepage at
 * <SENSOR-NAME>.local or using our Python SDK, available as ouster-sdk on PyPi.
 */
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "ouster/client.h"
#include "ouster/impl/build.h"

using namespace ouster::sdk;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM
                  << ")"
                  << "\n\nUsage: config_example <sensor_hostname> "
                  << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string sensor_hostname = argv[1];

    // 1. Get the current config on the sensor
    std::cerr << "1. Get original config of sensor... ";

    //! [doc-stag-cpp-get-config]
    core::SensorConfig original_config;
    if (!sensor::get_config(sensor_hostname, original_config)) {
        std::cerr << "..error: could not connect to sensor!" << std::endl;
        return EXIT_FAILURE;
    }
    //! [doc-etag-cpp-get-config]
    std::cout << "success! Got original config\nOriginal config of sensor:\n"
              << to_string(original_config) << std::endl;

    // 2. Make an empty sensor config and set a few config parameters
    std::cout << "\n2. Make new config and set sensor to it... ";
    //! [doc-stag-cpp-make-config]
    core::SensorConfig config;
    config.azimuth_window = std::pair<int, int>(90000, 270000);
    config.lidar_mode = core::LidarMode::_512x10;

    // If relevant, use config_flag to set udp dest automatically
    uint8_t config_flags = 0;
    const bool udp_dest_auto = true;  // whether or not to use auto destination
    const bool persist =
        false;  // whether or not we will persist the settings on the sensor

    if (udp_dest_auto) {
        config_flags |= sensor::CONFIG_UDP_DEST_AUTO;
    }
    if (persist) {
        config_flags |= sensor::CONFIG_PERSIST;
    }
    //! [doc-etag-cpp-make-config]

    if (!sensor::set_config(sensor_hostname, config, config_flags)) {
        std::cerr << "..error: could not connect to sensor" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "..success! Updated sensor to new config" << std::endl;

    // 3. Get the config from sensor after update
    std::cout << "\n3. Get back updated sensor config... ";
    core::SensorConfig new_config;
    if (!sensor::get_config(sensor_hostname, new_config)) {
        std::cerr << "..error: could not connect to sensor" << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "..success! Got updated config" << std::endl;
    }

    // Confirm that only what we wanted to change changed
    assert(original_config != new_config);
    assert(new_config.azimuth_window == config.azimuth_window);
    assert(new_config.lidar_mode == config.lidar_mode);

    std::cout << "Updated config: \n" << to_string(new_config) << std::endl;

    // 4. You cannot set the udp_dest flag while simultaneously setting
    // config.udp_dest Will throw an invalid_argument if you do
    std::cout << "\n4. Test setting udp_dest and config.udp_dest "
                 "simultaneously... ";
    config.udp_dest = "100.100.100.100";
    try {
        if (sensor::set_config(sensor_hostname, config, config_flags)) {
            // should not happen
            std::cerr << "..error: unexpected failure of set_config example"
                      << std::endl;
            return EXIT_FAILURE;
        }
    } catch (const std::invalid_argument&) {
        // expected result
        std::cout << "..success! Got expected failure to set udp_dest while "
                     "auto flag is set."
                  << std::endl;

    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // 5. Set the sensor back to how it started
    std::cout << "\n5. Setting sensor back to original state... ";
    if (!sensor::set_config(sensor_hostname, original_config)) {
        std::cerr << "..error: could not connect to sensor" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "..success! Returned sensor to original state." << std::endl;

    return EXIT_SUCCESS;
}

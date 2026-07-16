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

#include "ouster/core/impl/build.h"

// NOLINTBEGIN(google-build-using-namespace)
//! [doc-stag-cpp-config-imports]
#include "ouster/core/types.h"
#include "ouster/sensor/client.h"
using namespace ouster::sdk;
//! [doc-etag-cpp-config-imports]
// NOLINTEND(google-build-using-namespace)

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM << ")"
                  << "\n\nUsage: config_example <sensor_hostname> " << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string sensor_hostname = argv[1];

    // 1. Get the current config on the sensor
    std::cerr << "1. Get original config of sensor... ";
    //! [doc-stag-cpp-get-config]
    core::SensorConfig config = sensor::get_config(sensor_hostname);
    std::cerr << "Sensor config of " << sensor_hostname << ":\n"
              << core::to_string(config) << std::endl;
    //! [doc-etag-cpp-get-config]
    std::cout << "Success! Got original config\nOriginal config of sensor:\n"
              << core::to_string(config) << std::endl;

    // 2. Make an empty sensor config and set a few config parameters
    std::cout << "\n2. Make new config and set sensor to it... ";
    uint8_t config_flags = 0;
    //! [doc-stag-cpp-make-config]
    // set the values that you need: see sensor documentation for param meanings
    config.operating_mode = core::OperatingMode::NORMAL;
    config.lidar_mode = core::LidarMode::_1024x10;
    config.udp_dest = "@auto";
    //! [doc-etag-cpp-make-config]
    config.azimuth_window = std::pair<int, int>(90000, 270000);
    //! [doc-stag-cpp-set-config]
    auto persist = true;
    config_flags |= sensor::CONFIG_PERSIST;
    sensor::set_config(sensor_hostname, config, config_flags);
    //! [doc-etag-cpp-set-config]
    std::cout << "Success! Updated sensor to new config" << std::endl;

    // 3. Get the config from sensor after update
    std::cout << "\n3. Get back updated sensor config... ";
    core::SensorConfig new_config = sensor::get_config(sensor_hostname);
    std::cout << "..success! Got updated config" << std::endl;

    // Confirm that only what we wanted to change changed
    assert(config != new_config);
    assert(new_config.azimuth_window == config.azimuth_window);
    assert(new_config.lidar_mode == config.lidar_mode);

    std::cout << "Updated config: \n" << core::to_string(new_config) << std::endl;

    // 4. You cannot set the udp_dest flag while simultaneously setting
    // config.udp_dest Will throw an invalid_argument if you do
    std::cout << "\n4. Test setting udp_dest and config.udp_dest "
                 "simultaneously... ";
    config.udp_dest = "100.100.100.100";
    const auto conflicting_flags =
        static_cast<uint8_t>(config_flags | sensor::CONFIG_UDP_DEST_AUTO);
    try {
        sensor::set_config(sensor_hostname, config, conflicting_flags);
    } catch (const std::invalid_argument&) {
        // expected result
        std::cout << "..success! Got expected failure to set udp_dest while "
                     "auto flag is set."
                  << std::endl;

    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // 5. Set dual returns profile (requires FW support)
    // clang-format off
    //! [doc-stag-config-udp-profile]
    config.udp_profile_lidar = core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL;
    //! [doc-etag-config-udp-profile]
    // clang-format on

    //! [doc-stag-config-timing]
    // or TIME_FROM_PTP_1588, TIME_FROM_SYNC_PULSE_IN
    config.timestamp_mode = core::TimestampMode::TIME_FROM_INTERNAL_OSC;
    config.multipurpose_io_mode = core::MultipurposeIOMode::INPUT_NMEA_UART;
    config.nmea_baud_rate = core::NMEABaudRate::BAUD_9600;
    //! [doc-etag-config-timing]

    config_flags = 0;
    new_config.udp_dest = "@auto";
    if (persist) {
        config_flags |= sensor::CONFIG_PERSIST;
    }

    sensor::set_config(sensor_hostname, new_config, config_flags);
    std::cerr << "..success! Updated sensor to new config" << std::endl;

    // 5. Set the sensor back to how it started
    std::cout << "\n5. Setting sensor back to original state... ";
    sensor::set_config(sensor_hostname, config);
    std::cout << "..success! Returned sensor to original state." << std::endl;

    return EXIT_SUCCESS;
}

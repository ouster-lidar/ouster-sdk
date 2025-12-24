/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * An example of how to use the Zone Monitoring API to set live zones,
 * configure UDP output for zone states, and read them from a sensor.
 */
#include <iostream>
#include <string>
#include <vector>

#include "cxxopts.hpp"
#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/open_source.h"
#include "ouster/sensor_http.h"
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"

using namespace ouster::sdk;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    cxxopts::Options options(
        "zone_monitor_zone_states_example",
        "Ouster SDK Zone Monitor States Example. This example demonstrates "
        "how to set live zones, configure UDP output for zone states, and "
        "read them from a sensor.");

    // clang-format off
    options.add_options()
        ("s,sensor_hostname", "Sensor hostname or IP address", cxxopts::value<std::string>())
        ("h,help", "Print usage");
    // clang-format on

    try {
        auto result = options.parse(argc, argv);

        if (result.count("help") != 0 || result.count("sensor_hostname") == 0) {
            std::cout << options.help() << std::endl;
            return EXIT_SUCCESS;
        }

        const auto hostname = result["sensor_hostname"].as<std::string>();

        // [doc-stag-set-live-zones]
        auto http = sensor::SensorHttp::create(hostname);
        std::cout << "Setting live zones to {0, 1, 2, 3}..." << std::endl;
        http->set_zone_monitor_live_ids({0, 1, 2, 3});
        // [doc-etag-set-live-zones]

        // [doc-stag-set-zm-udp-dest]
        core::SensorConfig config{};
        config.udp_dest_zm = "169.254.100.204";
        config.udp_port_zm = 7504;
        std::cout << "Setting Zone Monitor UDP destination to "
                  << config.udp_dest_zm.value() << ":"
                  << config.udp_port_zm.value() << "..." << std::endl;
        if (!sensor::set_config(hostname, config)) {
            std::cerr << "Failed to set sensor config." << std::endl;
            return EXIT_FAILURE;
        }
        // [doc-etag-set-zm-udp-dest]

        // [doc-stag-read-zone-states]
        std::cout << "Connecting to sensor and reading zone states..."
                  << std::endl;
        auto source = open_source(hostname);
        for (const auto& scan_set : source) {
            if (scan_set.size() > 0 && scan_set[0]) {
                const auto& scan = *(scan_set[0]);
                if (scan.has_field(core::ChanField::ZONE_STATES)) {
                    auto zone_states = scan.zones();
                    std::cout << "Frame ID: " << scan.frame_id
                              << ", Zone States: [";
                    bool first = true;
                    for (const auto& zone : zone_states) {
                        if (zone.live != 0) {
                            if (!first) {
                                std::cout << ", ";
                            }
                            std::cout
                                << "(" << static_cast<int>(zone.id) << ", "
                                << static_cast<int>(zone.trigger_status) << ")";
                            first = false;
                        }
                    }
                    std::cout << "]" << std::endl;
                }
            }
        }
        // [doc-etag-read-zone-states]

        return EXIT_SUCCESS;

    } catch (const cxxopts::exceptions::exception& e) {
        std::cerr << "Error parsing options: " << e.what() << std::endl;
        std::cout << options.help() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::runtime_error& e) {
        std::cerr << "Runtime error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

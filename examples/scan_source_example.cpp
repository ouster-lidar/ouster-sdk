/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "ouster/impl/build.h"
#include "ouster/open_source.h"
#ifdef OUSTER_OSF
#include "ouster/osf/osf_scan_source.h"
#endif
#ifdef OUSTER_PCAP
#include "ouster/pcap_scan_source.h"
#endif
#ifdef OUSTER_SENSOR
#include "ouster/sensor_scan_source.h"
#endif

using namespace ouster::sdk;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM
                  << ")"
                  << "\n\nUsage: scan_source_example <source_file>"
                  << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string source_file = argv[1];

    // open source file non-collated
    auto source = open_source(
        source_file, [](auto& source_options) { source_options.index = true; },
        false);

    // read all message in timestamp order
    std::cout << "Printing out all scans..." << std::endl;
    for (const auto& scans : source) {
        for (const auto& lidar_scan : scans) {
            if (!lidar_scan) {
                continue;
            }
            std::cout << "ls = " << to_string(*lidar_scan) << std::endl;
        }
    }

    // read all message in timestamp order from the first sensor
    std::cout << std::endl
              << "Printing out all scans from sensor 0..." << std::endl;
    for (const auto& scans : source.single(0)) {
        for (const auto& lidar_scan : scans) {
            if (!lidar_scan) {
                continue;
            }
            std::cout << "ls = " << to_string(*lidar_scan) << std::endl;
        }
    }

    // read only the middle 10 messages, skipping every other
    std::cout << std::endl << "Printing out some scans..." << std::endl;
    for (const auto& scans : source[{10, 20, 2}]) {
        for (const auto& lidar_scan : scans) {
            if (!lidar_scan) {
                continue;
            }
            std::cout << "ls = " << to_string(*lidar_scan) << std::endl;
        }
    }

    // read only the last scan
    std::cout << std::endl << "Printing out last scans..." << std::endl;
    auto scans = source[-1];
    for (const auto& lidar_scan : scans) {
        if (!lidar_scan) {
            continue;
        }
        std::cout << "ls = " << to_string(*lidar_scan) << std::endl;
    }
}

/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include <iomanip>
#include <iostream>

#include "ouster/impl/build.h"
#include "ouster/open_source.h"
#include "ouster/osf/osf_scan_source.h"
#include "ouster/pcap_scan_source.h"
#include "ouster/sensor_scan_source.h"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: scan_source_example <source_file>"
                  << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string source_file = argv[1];

    // open source file non-collated
    auto source = ouster::open_source(
        source_file, [](auto& r) { r.index = true; }, false);

    // read all message in timestamp order
    std::cout << "Printing out all scans..." << std::endl;
    for (const auto& scans : source) {
        for (const auto& ls : scans) {
            if (!ls) {
                continue;
            }
            std::cout << "ls = " << to_string(*ls) << std::endl;
        }
    }

    // read all message in timestamp order from the first sensor
    std::cout << std::endl
              << "Printing out all scans from sensor 0..." << std::endl;
    for (const auto& scans : source.single(0)) {
        for (const auto& ls : scans) {
            if (!ls) {
                continue;
            }
            std::cout << "ls = " << to_string(*ls) << std::endl;
        }
    }

    // read only the middle 10 messages, skipping every other
    std::cout << std::endl << "Printing out some scans..." << std::endl;
    for (const auto& scans : source[{10, 20, 2}]) {
        for (const auto& ls : scans) {
            if (!ls) {
                continue;
            }
            std::cout << "ls = " << to_string(*ls) << std::endl;
        }
    }

    // read only the last scan
    std::cout << std::endl << "Printing out last scans..." << std::endl;
    auto scans = source[-1];
    for (const auto& ls : scans) {
        if (!ls) {
            continue;
        }
        std::cout << "ls = " << to_string(*ls) << std::endl;
    }
}

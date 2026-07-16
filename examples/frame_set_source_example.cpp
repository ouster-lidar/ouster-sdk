/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "ouster/core/impl/build.h"
#include "ouster/core/open_source.h"
#ifdef OUSTER_OSF
#include "ouster/osf/osf_frame_set_source.h"
#endif
#ifdef OUSTER_PCAP
#include "ouster/pcap/pcap_frame_set_source.h"
#endif
#ifdef OUSTER_SENSOR
#include "ouster/sensor/sensor_frame_set_source.h"
#endif

using namespace ouster::sdk;  // NOLINT(google-build-using-namespace)
int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM << ")"
                  << "\n\nUsage: frame_set_source_example <source_file>" << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string source_file = argv[1];

    // open source file non-collated
    auto source = open_source(
        source_file, [](auto& source_options) { source_options.index = true; }, false);

    // read all frames in timestamp order
    std::cout << "Printing out all frames..." << std::endl;
    for (const auto& frame_set : source) {
        for (const auto& frame : frame_set) {
            if (!frame) {
                continue;
            }
            std::cout << "frame = " << ouster::sdk::core::to_string(*frame) << std::endl;
        }
    }

    // read all frames in timestamp order from the first sensor
    std::cout << std::endl << "Printing out all frames from sensor 0..." << std::endl;
    for (const auto& frame_set : source.single(0)) {
        for (const auto& frame : frame_set) {
            if (!frame) {
                continue;
            }
            std::cout << "frame = " << ouster::sdk::core::to_string(*frame) << std::endl;
        }
    }

    // read only the middle 8 frames, skipping every other
    std::cout << std::endl << "Printing out some frames..." << std::endl;
    for (const auto& frame_set : source[{2, 10, 2}]) {
        for (const auto& frame : frame_set) {
            if (!frame) {
                continue;
            }
            std::cout << "frame = " << ouster::sdk::core::to_string(*frame) << std::endl;
        }
    }

    // read only the last frame
    std::cout << std::endl << "Printing out last frames..." << std::endl;
    auto frame_set = source[-1];
    for (const auto& frame : frame_set) {
        if (!frame) {
            continue;
        }
        std::cout << "frame = " << ouster::sdk::core::to_string(*frame) << std::endl;
    }
}

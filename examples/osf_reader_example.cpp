/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the LidarScan class of the
 * C++ Ouster SDK. Please see the sdk docs at static.ouster.dev for clearer
 * explanations.
 */

//! [doc-stag-osf-read-cpp]
#include <cstdlib>
#include <iostream>
#include <string>

#include "ouster/impl/build.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"

using namespace ouster::sdk;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM
                  << ")"
                  << "\n\nUsage: osf_reader_example <osf_file>" << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string osf_file = argv[1];

    // open OSF file
    osf::Reader reader(osf_file);

    // read all messages from OSF in timestamp order
    for (const auto& message : reader.messages()) {
        std::cout << "message.ts: " << message.ts().count()
                  << ", message.id: " << message.id() << std::endl;

        // In OSF file there maybe different type of messages stored, so here we
        // only interested in LidarScan messages
        if (message.is<osf::LidarScanStream>()) {
            // Decoding LidarScan messages
            auto lidar_scan = message.decode_msg<osf::LidarScanStream>();

            // if decoded successfully just print on the screen LidarScan
            if (lidar_scan) {
                std::cout << "ls = " << to_string(*lidar_scan) << std::endl;
            }
        }
    }
}
//! [doc-etag-osf-read-cpp]

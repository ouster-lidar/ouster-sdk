/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the LidarScan class of the
 * C++ Ouster SDK. Please see the sdk docs at static.ouster.dev for clearer
 * explanations.
 */

#include <iostream>

#include "ouster/impl/build.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"

using namespace ouster;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: osf_reader_example <osf_file>" << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string osf_file = argv[1];

    // open OSF file
    osf::Reader reader(osf_file);

    // read all messages from OSF in timestamp order
    for (const auto& m : reader.messages()) {
        std::cout << "m.ts: " << m.ts().count() << ", m.id: " << m.id()
                  << std::endl;

        // In OSF file there maybe different type of messages stored, so here we
        // only interested in LidarScan messages
        if (m.is<osf::LidarScanStream>()) {
            // Decoding LidarScan messages
            auto ls = m.decode_msg<osf::LidarScanStream>();

            // if decoded successfully just print on the screen LidarScan
            if (ls) {
                std::cout << "ls = " << to_string(*ls) << std::endl;
            }
        }
    }
}
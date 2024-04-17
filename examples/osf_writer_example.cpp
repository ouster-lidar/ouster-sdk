/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the osf::Writer class of
 * the C++ Ouster SDK. Please see the sdk docs at static.ouster.dev for clearer
 * explanations.
 */

//! [doc-stag-osf-write-cpp]
#include <iostream>

#include "ouster/impl/build.h"
#include "ouster/osf/writer.h"

using namespace ouster;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: osf_writer_example <osf_file>" << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string osf_file = argv[1];

    // Start writing a 1 stream OSF file with a default initialized sensor info
    osf::Writer writer(
        osf_file, sensor::default_sensor_info(ouster::sensor::MODE_512x10));

    // Instantiate a lidar scan with the expected width and height
    // default_sensor_info assumes a 64 plane sensor
    LidarScan scan(512, 64);
    // Manipulate the scan as desired here
    // Write it to file on stream 0
    writer.save(0, scan);
}
//! [doc-2tag-osf-write-cpp]

/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "ouster/core/impl/build.h"
//! [doc-stag-osf-read-imports]
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/osf/osf_frame_set_source.h"
using namespace ouster::sdk;
//! [doc-etag-osf-read-imports]

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << sdk::SDK_VERSION_FULL << " (" << sdk::BUILD_SYSTEM << ")"
                  << "\n\nUsage: osf_open_source_example <osf_file>" << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string osf_file = argv[1];

    // clang-format off
    //! [doc-stag-osf-read-cpp]
    // open the OSF file through the open_source factory
    auto frame_set_source = open_source(osf_file);
    // iterate over all lidar frames stored inside the OSF
    for (const auto& frame_set : frame_set_source) {
        for (const auto& lidar_frame : frame_set) {
            if (!lidar_frame) {
                continue;
            }
            std::cout << "frame = " << to_string(*lidar_frame)
                      << ", WxH=" << lidar_frame->w << "x"
                      << lidar_frame->h << std::endl;
        }
    }
    //! [doc-etag-osf-read-cpp]
    // clang-format on
}

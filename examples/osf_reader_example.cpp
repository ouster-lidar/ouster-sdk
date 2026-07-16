/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the LidarFrame class of the
 * C++ Ouster SDK. Please see the sdk docs at static.ouster.dev for clearer
 * explanations.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "ouster/core/impl/build.h"
#include "ouster/osf/metadata_stream.h"

// NOLINTBEGIN(google-build-using-namespace)
// [doc-stag-osf-reader-metadata-imports]
#include "ouster/osf/reader.h"
using namespace ouster::sdk;
// [doc-etag-osf-reader-metadata-imports]
// NOLINTEND(google-build-using-namespace)

// for osf::LidarFrameStream
#include "ouster/osf/stream_lidar_frame.h"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM << ")"
                  << "\n\nUsage: osf_reader_example <osf_file>" << std::endl;

        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    const std::string osf_file = argv[1];
    // open OSF file
    // clang-format off
    //! [doc-stag-osf-reader-metadata]
    osf::Reader metadata_reader(osf_file);
    const auto lidar_metadata = metadata_reader.meta_store().find<osf::LidarSensor>();
    if (lidar_metadata.empty()) {
        std::cout << "No LidarSensor metadata entries found." << std::endl;
    } else {
        for (const auto& entry : lidar_metadata) {
            const auto meta_id = entry.first;
            const auto& info = entry.second->info();
            std::cout << "meta[" << meta_id << "] -> sn=" << info.sn
                      << ", fw_rev=" << info.fw_rev
                      << ", prod_line=" << info.prod_line << std::endl; } }
    //! [doc-etag-osf-reader-metadata]

    //! [doc-stag-osf-read-cpp]
    osf::Reader reader(osf_file);
    // Read all messages from OSF in timestamp order
    for (const auto& message : reader.messages()) {
        std::cout << "message.ts: " << message.ts().count()
                  << ", message.id: " << message.id() << std::endl;
        // In OSF file there maybe different type of messages stored.
        // We must check the type of message before decoding it.
        if (message.is<osf::LidarFrameStream>()) {
            // Decoding LidarFrame messages
            auto lidar_frame = message.decode_msg<osf::LidarFrameStream>();
            // if decoded successfully just print on the screen LidarFrame
            if (lidar_frame) {
                std::cout << "ls = " << to_string(*lidar_frame) << std::endl;
            }
        } else if (message.is<osf::FrameSetSourceMetadataStream>()) {
            // Decoding FrameSetSourceMetadataStream messages
            auto frame_set_source_meta =
                message.decode_msg<osf::FrameSetSourceMetadataStream>();

            // if decoded successfully just print on the screen
            // FrameSetSourceMetadata
            if (frame_set_source_meta) {
                std::cout << "Found metadata collection with keys:\n";
                auto keys = frame_set_source_meta->keys();
                for (const auto& key : keys) {
                    std::cout << "  key: " << key << std::endl;
                }
            }
    //! [doc-etag-osf-read-cpp]
            // clang-format on
        }
    }
}

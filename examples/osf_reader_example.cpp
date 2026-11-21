/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the LidarFrame class of the
 * C++ Ouster SDK. Please see the sdk docs at docs.ouster.com for clearer
 * explanations.
 */

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "ouster/core/camera_info.h"
#include "ouster/core/impl/build.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/lidar_info.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/metadata_stream.h"

// NOLINTBEGIN(google-build-using-namespace)
// [doc-stag-osf-reader-metadata-imports]
#include "ouster/osf/reader.h"
using namespace ouster::sdk;
// [doc-etag-osf-reader-metadata-imports]
// NOLINTEND(google-build-using-namespace)

// for osf::LidarFrameStream and osf::CameraFrameStream
#include "ouster/osf/stream_camera_frame.h"
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
    // A LidarSensor metadata entry stores either a LidarInfo or a CameraInfo
    // (cameras reuse the same footer metadata type). Use sensor_info() for the
    // polymorphic base and downcast to inspect kind-specific fields.
    const auto sensor_metadata = metadata_reader.meta_store().find<osf::LidarSensor>();
    if (sensor_metadata.empty()) {
        std::cout << "No sensor metadata entries found." << std::endl;
    } else {
        for (const auto& entry : sensor_metadata) {
            const auto meta_id = entry.first;
            const auto info = entry.second->sensor_info();
            if (const auto lidar = std::dynamic_pointer_cast<core::LidarInfo>(info)) {
                std::cout << "meta[" << meta_id << "] -> lidar sn=" << lidar->sn
                          << ", fw_rev=" << lidar->fw_rev
                          << ", prod_line=" << lidar->prod_line << std::endl;
            } else if (const auto camera = std::dynamic_pointer_cast<core::CameraInfo>(info)) {
                std::cout << "meta[" << meta_id << "] -> camera sn=" << camera->sn
                          << ", width=" << camera->w()
                          << ", height=" << camera->h() << std::endl; } } }
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
        } else if (message.is<osf::CameraFrameStream>()) {
            // Decoding CameraFrame messages
            auto camera_frame = message.decode_msg<osf::CameraFrameStream>();
            // if decoded successfully print a short summary of the CameraFrame
            if (camera_frame) {
                std::cout << "camera frame ts " << camera_frame->timestamp()[0] << " ("
                          << camera_frame->w << "x" << camera_frame->h << ")" << std::endl;
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

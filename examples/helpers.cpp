/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include "helpers.h"

#include <memory>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/packet_source.h"
#include "ouster/core/types.h"

using namespace ouster::sdk;  // NOLINT(google-build-using-namespace)

void get_complete_frame(core::PacketSource& source, core::LidarFrame& frame) {
    // Helper variable to help us identify first full frame
    int64_t first_frame_id = 0;
    auto info = *source.sensor_info()[0];
    core::FrameBatcher batch_to_frame(info);

    auto packet_format = std::make_shared<core::PacketFormat>(info);

    auto iter = source.begin();
    while (iter != source.end()) {
        auto idx_and_packet = *iter;
        auto packet = idx_and_packet.second;
        if (batch_to_frame.batch(*packet, frame)) {
            if (first_frame_id == 0) {
                // end of first frame -- assume it is incomplete and skip
                first_frame_id = frame.frame_id;
            } else if (first_frame_id != frame.frame_id) {
                return;
            }
        }
        ++iter;
    }
}

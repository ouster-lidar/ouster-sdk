/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include <memory>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/packet_source.h"
#include "ouster/core/types.h"

// Fill frame with data from the 2nd frame in the pcap
// If there is no 2nd frame, frame will remain unchanged
void get_complete_frame(ouster::sdk::core::PacketSource& source,
                        ouster::sdk::core::LidarFrame& frame);

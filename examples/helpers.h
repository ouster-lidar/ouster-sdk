/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/lidar_scan.h"
#include "ouster/os_pcap.h"
#include "ouster/types.h"

// Fill scan with data from the 2nd scan in the pcap
// If there is no 2nd scan, scan will remain unchanged
void get_complete_scan(
    std::shared_ptr<ouster::sensor_utils::playback_handle> handle,
    ouster::LidarScan& scan, ouster::sensor::sensor_info& info);

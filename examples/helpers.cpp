/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include "helpers.h"

#include "ouster/lidar_scan.h"
#include "ouster/os_pcap.h"
#include "ouster/types.h"

using namespace ouster::sensor;

constexpr std::size_t BUF_SIZE = 65536;

void get_complete_scan(
    std::shared_ptr<ouster::sensor_utils::playback_handle> handle,
    ouster::LidarScan& scan, sensor_info& info) {
    // Make sure we start at beginning
    ouster::sensor_utils::replay_reset(*handle);

    // Helper variable to help us identify first full frame
    int first_frame_id = 0;

    auto pf = get_format(info);
    ouster::ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);

    // Buffer to store raw packet data
    auto packet_buf = std::make_unique<uint8_t[]>(BUF_SIZE);

    ouster::sensor_utils::packet_info packet_info;

    while (ouster::sensor_utils::next_packet_info(*handle, packet_info)) {
        auto packet_size = ouster::sensor_utils::read_packet(
            *handle, packet_buf.get(), BUF_SIZE);

        if (packet_size == pf.lidar_packet_size &&
            packet_info.dst_port == info.udp_port_lidar) {
            if (batch_to_scan(packet_buf.get(), scan)) {
                if (first_frame_id == 0) {
                    // end of first frame -- assume it is incomplete and skip
                    first_frame_id = scan.frame_id;
                } else if (first_frame_id != scan.frame_id)
                    break;
            }
        }
    }
}

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
    ouster::ScanBatcher batch_to_scan(info);

    // Buffer to store raw packet data
    ouster::sensor::LidarPacket packet(pf.lidar_packet_size);

    ouster::sensor_utils::packet_info packet_info;

    while (ouster::sensor_utils::next_packet_info(*handle, packet_info)) {
        auto packet_size = ouster::sensor_utils::read_packet(
            *handle, packet.buf.data(), packet.buf.size());

        if (packet_size == pf.lidar_packet_size &&
            packet_info.dst_port == info.config.udp_port_lidar) {
            if (batch_to_scan(packet, scan)) {
                if (first_frame_id == 0) {
                    // end of first frame -- assume it is incomplete and skip
                    first_frame_id = scan.frame_id;
                } else if (first_frame_id != scan.frame_id)
                    break;
            }
        }
    }
}

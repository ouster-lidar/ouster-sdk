/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include "helpers.h"

#include <memory>

#include "ouster/lidar_scan.h"
#include "ouster/os_pcap.h"
#include "ouster/types.h"

using namespace ouster::sdk;  // NOLINT(google-build-using-namespace)

void get_complete_scan(
    std::shared_ptr<pcap::PlaybackHandle>
        handle,  // NOLINT(performance-unnecessary-value-param)
    core::LidarScan& scan, core::SensorInfo& info) {
    // Make sure we start at beginning
    pcap::replay_reset(*handle);

    // Helper variable to help us identify first full frame
    int64_t first_frame_id = 0;

    auto packet_format = get_format(info);
    core::ScanBatcher batch_to_scan(info);

    // Buffer to store raw packet data
    core::LidarPacket packet(static_cast<int>(packet_format.lidar_packet_size));

    pcap::PacketInfo packet_info;

    while (pcap::next_packet_info(*handle, packet_info)) {
        auto packet_size =
            pcap::read_packet(*handle, packet.buf.data(), packet.buf.size());

        if (packet_size == packet_format.lidar_packet_size &&
            packet_info.dst_port == info.config.udp_port_lidar) {
            if (batch_to_scan(packet, scan)) {
                if (first_frame_id == 0) {
                    // end of first frame -- assume it is incomplete and skip
                    first_frame_id = scan.frame_id;
                } else if (first_frame_id != scan.frame_id) {
                    break;
                }
            }
        }
    }
}

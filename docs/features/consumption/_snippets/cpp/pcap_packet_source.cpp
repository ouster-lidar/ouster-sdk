// Copyright (c) 2024, Ouster, Inc.
// All rights reserved.

#include <cstddef>
#include <iostream>
#include <memory>
#include <string>

//! [doc-stag-pcap-packet-source-cpp-imports]
#include "ouster/core/packet.h"
#include "ouster/pcap/pcap_packet_source.h"

using namespace ouster::sdk;
//! [doc-etag-pcap-packet-source-cpp-imports]

namespace ouster {
namespace sdk {
namespace docs {

struct PacketCounts {
    std::size_t lidar = 0;
    std::size_t imu = 0;
};

PacketCounts read_pcap_packets(const std::string& pcap_file) {
    PacketCounts counts;

    //! [doc-stag-pcap-packet-source-cpp]
    pcap::PcapPacketSource source(pcap_file);

    for (const auto& item : source) {
        const int sensor_idx = item.first;
        const auto& packet_ptr = item.second;
        if (!packet_ptr) continue;
        if (packet_ptr->type() == core::PacketType::Lidar) {
            std::cout << "sensor=" << sensor_idx << " lidar frame_id=" << packet_ptr->frame_id()
                      << " bytes=" << packet_ptr->buf.size() << '\n';
            ++counts.lidar;
        } else if (packet_ptr->type() == core::PacketType::Imu) {
            std::cout << "sensor=" << sensor_idx << " imu host_ts=" << packet_ptr->host_timestamp
                      << '\n';
            ++counts.imu;
        }
    }
    //! [doc-etag-pcap-packet-source-cpp]

    return counts;
}

}  // namespace docs
}  // namespace sdk
}  // namespace ouster

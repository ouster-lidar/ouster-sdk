/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/pcap_source.h"

namespace ouster {
namespace osf {

template <typename H>
PcapRawSource::PacketHandler use_packet(H&& handler) {
    return
        [handler](const sensor_utils::packet_info& p_info, const uint8_t* buf) {
            osf::ts_t ts(p_info.timestamp);
            handler(ts, buf);
        };
}

PcapRawSource::PcapRawSource(const std::string& filename)
    : pcap_filename_{filename} {
    pcap_handle_ = sensor_utils::replay_initialize(pcap_filename_);
}

void PcapRawSource::runAll() {
    sensor_utils::packet_info p_info;
    while (sensor_utils::next_packet_info(*pcap_handle_, p_info)) {
        handleCurrentPacket(p_info);
    }
}

void PcapRawSource::runWhile(const PacketInfoPredicate& pred) {
    sensor_utils::packet_info p_info;
    while (sensor_utils::next_packet_info(*pcap_handle_, p_info)) {
        if (!pred(p_info)) {
            break;
        }
        handleCurrentPacket(p_info);
    }
}

void PcapRawSource::addLidarDataHandler(int dst_port,
                                        const sensor::sensor_info& info,
                                        LidarDataHandler&& lidar_handler) {
    auto build_ls = osf::make_build_ls(info, lidar_handler);
    packet_handlers_.insert({dst_port, use_packet(build_ls)});
}

void PcapRawSource::addLidarDataHandler(
    int dst_port, const sensor::sensor_info& info,
    const LidarScanFieldTypes& ls_field_types,
    LidarDataHandler&& lidar_handler) {
    auto build_ls = osf::make_build_ls(info, ls_field_types, lidar_handler);
    packet_handlers_.insert({dst_port, use_packet(build_ls)});
}

void PcapRawSource::handleCurrentPacket(
    const sensor_utils::packet_info& pinfo) {
    constexpr uint32_t buf_size = 65536;  // 2^16
    uint8_t buf[buf_size];
    auto handler_it = packet_handlers_.find(pinfo.dst_port);
    if (handler_it != packet_handlers_.end()) {
        auto size_read =
            sensor_utils::read_packet(*pcap_handle_, buf, buf_size);
        if (size_read > 0 && size_read < buf_size &&
            size_read == pinfo.payload_size) {
            handler_it->second(pinfo, buf);
        }
    }
}

PcapRawSource::~PcapRawSource() {
    if (pcap_handle_) {
        sensor_utils::replay_uninitialize(*pcap_handle_);
    }
}

}  // namespace osf
}  // namespace ouster

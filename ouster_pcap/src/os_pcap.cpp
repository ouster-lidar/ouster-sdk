/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 */

#include "ouster/os_pcap.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "ouster/pcap.h"

namespace ouster {
namespace sensor_utils {

struct record_handle {
    record_handle(const std::string& path,
                  PcapWriter::PacketEncapsulation encap, uint16_t frag_size)
        : writer{std::make_unique<PcapWriter>(path, encap, frag_size)} {}

    std::unique_ptr<PcapWriter> writer;
};

struct playback_handle {
    std::string path;
    std::unique_ptr<PcapReader> pcap;

    playback_handle(const std::string& path)
        : path{path}, pcap{std::make_unique<PcapReader>(path)} {}

    playback_handle& operator=(playback_handle&& other) = default;

    ~playback_handle() {}
};

std::ostream& operator<<(std::ostream& stream_in, const packet_info& data) {
    stream_in << "Source IP: \"" << data.src_ip << "\" ";
    stream_in << "Source Port: " << data.src_port << std::endl;

    stream_in << "Destination IP: \"" << data.dst_ip << "\" ";
    stream_in << "Destination Port: " << data.dst_port << std::endl;

    stream_in << "Payload Size: " << data.payload_size << std::endl;
    stream_in << "Timestamp: " << data.timestamp.count() << std::endl;

    stream_in << "Fragments In Packet: " << data.fragments_in_packet
              << std::endl;
    stream_in << "Encapsulation Protocol: " << data.encapsulation_protocol
              << std::endl;
    stream_in << "Network Protocol: " << data.network_protocol << std::endl;

    return stream_in;
}

std::shared_ptr<playback_handle> replay_initialize(
    const std::string& file_path) {
    return std::make_shared<playback_handle>(file_path);
}

void replay_uninitialize(playback_handle& handle) { handle.pcap.reset(); }

void replay_reset(playback_handle& handle) {
    handle = playback_handle{handle.path};
}

bool next_packet_info(playback_handle& handle, packet_info& info) {
    if (handle.pcap && handle.pcap->next_packet()) {
        info = handle.pcap->current_info();
        return true;
    }
    return false;
}

size_t read_packet(playback_handle& handle, uint8_t* buf, size_t buffer_size) {
    size_t len = handle.pcap->current_length();
    if (len <= buffer_size) {
        std::memcpy(buf, handle.pcap->current_data(), len);
    } else {
        len = 0;
    }
    return len;
}

std::shared_ptr<record_handle> record_initialize(const std::string& file_name,
                                                 int frag_size,
                                                 bool use_sll_encapsulation) {
    PcapWriter::PacketEncapsulation encap =
        (use_sll_encapsulation) ? PcapWriter::PacketEncapsulation::SLL
                                : PcapWriter::PacketEncapsulation::ETHERNET;
    return std::make_shared<record_handle>(file_name, encap, frag_size);
}

void record_uninitialize(record_handle& handle) { handle.writer.reset(); }

void record_packet(record_handle& handle, const packet_info& info,
                   const uint8_t* buf, size_t buffer_size) {
    handle.writer->write_packet(buf, buffer_size, info);
}

void record_packet(record_handle& handle, const std::string& src_ip,
                   const std::string& dst_ip, int src_port, int dst_port,
                   const uint8_t* buf, size_t buffer_size,
                   uint64_t microsecond_timestamp) {
    if (!handle.writer) return;

    auto time = packet_info::ts{microsecond_timestamp};
    handle.writer->write_packet(buf, buffer_size, src_ip, dst_ip, src_port,
                                dst_port, time);
}

}  // namespace sensor_utils
}  // namespace ouster

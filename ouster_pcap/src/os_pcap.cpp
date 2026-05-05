/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 */

#include "ouster/os_pcap.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "ouster/indexed_pcap_reader.h"
#include "ouster/pcap.h"

namespace ouster {
namespace sdk {
namespace pcap {

// TODO: IndexedPcapReader bypasses PlaybackHandle and RecordHandle, either
// use the same fencing mechansim or switch to use OOP style

struct RecordHandle {
    RecordHandle(const std::string& path, PcapWriter::PacketEncapsulation encap,
                 uint16_t frag_size)
        : writer{std::make_unique<PcapWriter>(path, encap, frag_size)} {}

    std::unique_ptr<PcapWriter> writer;
};

struct PlaybackHandle {
    std::string path;
    std::unique_ptr<PcapReader> pcap;

    explicit PlaybackHandle(const std::string& path)
        : path{path}, pcap{std::make_unique<PcapReader>(path)} {}

    PlaybackHandle(const PlaybackHandle&) = delete;
    PlaybackHandle& operator=(const PlaybackHandle&) = delete;

    PlaybackHandle(PlaybackHandle&&) = default;
    PlaybackHandle& operator=(PlaybackHandle&& other) = default;

    ~PlaybackHandle() = default;
};

std::ostream& operator<<(std::ostream& stream_in, const PacketInfo& data) {
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

bool StreamKey::operator==(const struct StreamKey& other) const {
    return dst_ip == other.dst_ip && src_ip == other.src_ip &&
           src_port == other.src_port && dst_port == other.dst_port;
}

std::ostream& operator<<(std::ostream& stream_in, const StreamKey& data) {
    stream_in << "Source IP: \"" << data.src_ip << "\" " << std::endl;
    stream_in << "Destination IP: \"" << data.dst_ip << "\" " << std::endl;
    stream_in << "Source Port: " << data.src_port << std::endl;
    stream_in << "Destination Port: " << data.dst_port << std::endl;
    return stream_in;
}

std::ostream& operator<<(std::ostream& stream_in, const StreamData& data) {
    stream_in << "Count: " << data.count << " ";
    stream_in << "Payload Sizes: " << std::endl;
    for (auto const& it : data.payload_size_counts) {
        stream_in << "Size: " << it.first << " Count: " << it.second
                  << std::endl;
    }

    stream_in << "Fragments In Packets: " << std::endl;
    for (auto const& it : data.fragment_counts) {
        stream_in << "Number of Fragments: " << it.first
                  << " Count: " << it.second << std::endl;
    }

    stream_in << "IP Versions: " << std::endl;
    for (auto const& it : data.ip_version_counts) {
        stream_in << "IP Version: " << it.first << " Count: " << it.second
                  << std::endl;
    }

    return stream_in;
}

std::ostream& operator<<(std::ostream& stream_in, const StreamInfo& data) {
    stream_in << "Total Packets: " << data.total_packets << std::endl;
    stream_in << "Encapsultion Protocol: " << data.encapsulation_protocol
              << std::endl;
    stream_in << "Max Timestamp: " << data.timestamp_max.count() << std::endl;
    stream_in << "Min Timestamp: " << data.timestamp_min.count() << std::endl;

    for (const auto& it : data.udp_streams) {
        stream_in << "Key: " << std::endl << it.first << std::endl;
        stream_in << "Data: " << std::endl << it.second << std::endl;
        stream_in << std::endl << std::endl << std::endl;
    }
    return stream_in;
}

std::shared_ptr<PlaybackHandle> replay_initialize(
    const std::string& file_path) {
    return std::make_shared<PlaybackHandle>(file_path);
}

void replay_uninitialize(PlaybackHandle& handle) { handle.pcap.reset(); }

void replay_reset(PlaybackHandle& handle) {
    handle = PlaybackHandle{handle.path};
}

bool next_packet_info(PlaybackHandle& handle, PacketInfo& info) {
    if (handle.pcap && (handle.pcap->next_packet() != 0u)) {
        info = handle.pcap->current_info();
        return true;
    }
    return false;
}

size_t read_packet(PlaybackHandle& handle, uint8_t* buf, size_t buffer_size) {
    size_t len = handle.pcap->current_length();
    if (len <= buffer_size) {
        std::memcpy(buf, handle.pcap->current_data(), len);
    } else {
        len = 0;
    }
    return len;
}

std::shared_ptr<RecordHandle> record_initialize(const std::string& file_name,
                                                int frag_size,
                                                bool use_sll_encapsulation) {
    PcapWriter::PacketEncapsulation encap =
        (use_sll_encapsulation) ? PcapWriter::PacketEncapsulation::SLL
                                : PcapWriter::PacketEncapsulation::ETHERNET;
    return std::make_shared<RecordHandle>(file_name, encap, frag_size);
}

void record_uninitialize(RecordHandle& handle) { handle.writer.reset(); }

void record_packet(RecordHandle& handle, const PacketInfo& info,
                   const uint8_t* buf, size_t buffer_size) {
    handle.writer->write_packet(buf, buffer_size, info);
}

void record_packet(RecordHandle& handle, const std::string& src_ip,
                   const std::string& dst_ip, int src_port, int dst_port,
                   const uint8_t* buf, size_t buffer_size,
                   uint64_t microsecond_timestamp) {
    if (!handle.writer) {
        return;
    }
    auto time = PacketInfo::ts{microsecond_timestamp};
    handle.writer->write_packet(buf, buffer_size, src_ip, dst_ip, src_port,
                                dst_port, time);
}

// TODO: make a member of `PcapReader` ?
std::shared_ptr<StreamInfo> get_stream_info(
    PcapReader& pcap_reader,
    const std::function<void(uint64_t, uint64_t, uint64_t)>& progress_callback,
    int packets_per_callback, int packets_to_process) {
    uint64_t file_size = pcap_reader.file_size();
    std::shared_ptr<StreamInfo> result = std::make_shared<StreamInfo>();

    int callback_count = 0;
    uint64_t last_current = 0;
    uint64_t diff_acc = 0;

    int i = 0;
    PacketInfo info;
    bool first = true;
    uint64_t prev_location = 0;

    while (((packets_to_process <= 0) || (i < packets_to_process)) &&
           (pcap_reader.next_packet() != 0u)) {
        info = pcap_reader.current_info();

        // TODO: if `pcap_reader` is an IndexedPcapReader,
        // get the `sensor_info` that matches the packet
        // and the `packet_format` from the `sensor_info`
        // and possibly use a `ScanBatcher` (one for each sensor stream)
        // to determine if a scan boundary has been found
        if (IndexedPcapReader* indexed_pcap_reader_ptr =
                dynamic_cast<IndexedPcapReader*>(&pcap_reader)) {
            indexed_pcap_reader_ptr->update_index_for_current_packet();
        }

        callback_count++;
        if (first) {
            first = false;
            result->encapsulation_protocol = info.encapsulation_protocol;
            result->timestamp_max = info.timestamp;
            result->timestamp_min = info.timestamp;
        }
        result->total_packets++;

        if (info.timestamp < result->timestamp_min) {
            result->timestamp_min = info.timestamp;
        }
        if (info.timestamp > result->timestamp_max) {
            result->timestamp_max = info.timestamp;
        }

        StreamKey key;

        key.dst_ip = info.dst_ip;
        key.src_ip = info.src_ip;
        key.dst_port = info.dst_port;
        key.src_port = info.src_port;

        auto& stream = result->udp_streams[key];
        stream.count++;
        stream.payload_size_counts[info.payload_size]++;
        stream.fragment_counts[info.fragments_in_packet]++;
        stream.ip_version_counts[info.ip_version]++;

        diff_acc += (info.file_offset - prev_location);
        last_current = info.file_offset;

        if (callback_count > packets_per_callback &&
            packets_per_callback >= 0) {
            progress_callback(info.file_offset, diff_acc, file_size);
            diff_acc = 0;
            callback_count = 0;
        }
        prev_location = info.file_offset;
        i++;
    }
    if (diff_acc > 0 && packets_per_callback >= 0) {
        progress_callback(last_current, diff_acc, file_size);
    }
    pcap_reader.reset();

    return result;
}

std::shared_ptr<StreamInfo> get_stream_info(
    const std::string& file,
    const std::function<void(uint64_t, uint64_t, uint64_t)>& progress_callback,
    int packets_per_callback, int packets_to_process) {
    auto handle = replay_initialize(file);
    if (handle) {
        return get_stream_info(*(handle->pcap), progress_callback,
                               packets_per_callback, packets_to_process);
    }
    return std::make_shared<StreamInfo>();
}

std::shared_ptr<StreamInfo> get_stream_info(const std::string& file,
                                            int packets_to_process) {
    return get_stream_info(
        file, [](uint64_t, uint64_t, uint64_t) {}, -1, packets_to_process);
}

/*
          The current approach is roughly: 1) treat each unique source /
   destination port and IP as a single logical 'stream' of data, 2) filter out
   streams that don't match the expected packet sizes specified by the metadata,
   3) pair up any potential lidar/imu streams that appear to be coming from the
   same sensor (have matching source IPs) 4) and finally, filter out the pairs
   that contradict any ports specified in the metadata.
*/
std::vector<GuessedPorts> guess_ports(StreamInfo& info, int lidar_packet_sizes,
                                      int imu_packet_sizes, int lidar_spec,
                                      int imu_spec) {
    std::vector<GuessedPorts> temp_result;
    std::vector<GuessedPorts> lidar_result;
    std::vector<GuessedPorts> imu_result;
    std::vector<GuessedPorts> result;

    std::vector<StreamKey> lidar_keys;
    std::vector<StreamKey> imu_keys;
    std::vector<std::string> lidar_src_ips;
    std::vector<std::string> imu_src_ips;

    for (const auto& it : info.udp_streams) {
        if (it.second.payload_size_counts.count(lidar_packet_sizes) > 0) {
            lidar_keys.push_back(it.first);
            lidar_src_ips.push_back(it.first.src_ip);
        }
        if (it.second.payload_size_counts.count(imu_packet_sizes) > 0) {
            imu_keys.push_back(it.first);
            imu_src_ips.push_back(it.first.src_ip);
        }
    }

    // Full join on lidar and IMU
    for (const auto& lidar_it : lidar_keys) {
        bool imu_processed = false;
        for (const auto& imu_it : imu_keys) {
            // This case runs for when we have both Lidar and IMU data
            if (lidar_it.src_ip == imu_it.src_ip) {
                GuessedPorts ports{};
                ports.lidar = lidar_it.dst_port;
                ports.imu = imu_it.dst_port;
                imu_processed = true;
                temp_result.push_back(ports);
            }
            // This case runs if we just have an IMU unmatched with the lidar
            else if (std::find(lidar_src_ips.begin(), lidar_src_ips.end(),
                               imu_it.src_ip) == lidar_src_ips.end()) {
                GuessedPorts ports{};
                ports.lidar = 0;
                ports.imu = imu_it.dst_port;
                imu_result.push_back(ports);
            }
        }
        // This case runs if we just have Lidar data
        if (!imu_processed && std::find(imu_src_ips.begin(), imu_src_ips.end(),
                                        lidar_it.src_ip) == imu_src_ips.end()) {
            GuessedPorts ports{};
            ports.lidar = lidar_it.dst_port;
            ports.imu = 0;
            lidar_result.push_back(ports);
        }
    }
    // This case runs if we just have IMU data
    if (lidar_keys.empty()) {
        for (const auto& imu_it : imu_keys) {
            GuessedPorts ports{};
            ports.lidar = 0;
            ports.imu = imu_it.dst_port;
            imu_result.push_back(ports);
        }
    }

    temp_result.insert(temp_result.end(), lidar_result.begin(),
                       lidar_result.end());
    temp_result.insert(temp_result.end(), imu_result.begin(), imu_result.end());

    for (auto it : temp_result) {
        if (((it.lidar == lidar_spec) || (lidar_spec == 0) ||
             (it.lidar == 0)) &&
            ((it.imu == imu_spec) || (imu_spec == 0) || (it.imu == 0))) {
            result.push_back(it);
        }
    }
    return result;
}

}  // namespace pcap
}  // namespace sdk
}  // namespace ouster

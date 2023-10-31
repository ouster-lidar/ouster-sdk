/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief OS-1 pcap replay
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/pcap.h"
#include "ouster/types.h"
namespace ouster {
namespace sensor_utils {
/**
 * Structure representing a hash key/sorting key for a udp stream
 */
struct stream_key {
    std::string dst_ip;  ///< The destination IP
    std::string src_ip;  ///< The source IP
    int src_port;        ///< The src port
    int dst_port;        ///< The destination port

    bool operator==(const struct stream_key& other) const;
};
}  // namespace sensor_utils
}  // namespace ouster

template <>
struct std::hash<ouster::sensor_utils::stream_key> {
    std::size_t operator()(
        const ouster::sensor_utils::stream_key& key) const noexcept {
        return std::hash<std::string>{}(key.src_ip) ^
               (std::hash<std::string>{}(key.src_ip) << 1) ^
               (std::hash<int>{}(key.src_port << 2)) ^
               (std::hash<int>{}(key.dst_port << 3));
    }
};

namespace ouster {
namespace sensor_utils {

using ts = std::chrono::microseconds;  ///< Microsecond timestamp

/**
 * To string method for packet info structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The packet_info to output.
 *
 * @return The new output stream containing concatted stream_in and data.
 */
std::ostream& operator<<(std::ostream& stream_in, const packet_info& data);

/**
 * Structure representing a hash key/sorting key for a udp stream
 */
struct guessed_ports {
    int lidar;  ///< Guessed lidar port
    int imu;    ///< Guessed imu port
};

/**
 * To string method for stream_key structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The stream_key to output.
 *
 * @return The new output stream containing concatted stream_in and data.
 */
std::ostream& operator<<(std::ostream& stream_in, const stream_key& data);

struct stream_data {
    uint64_t count;  ///< Number of packets in a specified stream
    std::map<uint64_t, uint64_t>
        payload_size_counts;  ///< Packet sizes detected in a specified stream
                              ///< Key: Packet Size
                              ///< Value: Count of a specific packet size
    std::map<uint64_t, uint64_t>
        fragment_counts;  ///< Fragments detected in a specified stream
                          ///< Key: Number of fragments
                          ///< Value: Count of a specific number of packets
    std::map<uint64_t, uint64_t>
        ip_version_counts;  ///< IP version detected in a specified stream
                            ///< Key: IP Version
                            ///< Value: Count of the specific ip version
};

/**
 * To string method for stream_data structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The stream_data to output.
 *
 * @return The new output stream containing concatted stream_in and data.
 */
std::ostream& operator<<(std::ostream& stream_in, const stream_data& data);

/**
 * Structure representing the information about network streams in a pcap file
 */
struct stream_info {
    uint64_t total_packets;           ///< The total number of packets detected
    uint32_t encapsulation_protocol;  ///< The encapsulation protocol for the
                                      ///< pcap file

    ts timestamp_max;  ///< The latest timestamp detected
    ts timestamp_min;  ///< The earliest timestamp detected

    std::unordered_map<stream_key, stream_data>
        udp_streams;  ///< Datastructure containing info on all of the different
                      ///< streams
};

/**
 * To string method for stream info structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The stream_info to output.
 *
 * @return The new output stream containing concatted stream_info and data.
 */
std::ostream& operator<<(std::ostream& stream_in, const stream_info& data);

/** Struct to hide the stepwise playback details. */
struct playback_handle;

/** Struct to hide the record details. */
struct record_handle;

/**
 * Initialize the stepwise playback handle.
 *
 * @param[in] file The file path of the pcap file.
 *
 * @return A handle to the initialized playback struct.
 */
std::shared_ptr<playback_handle> replay_initialize(const std::string& file);

/**
 * Uninitialize the stepwise playback handle.
 *
 * @param[in] handle A handle to the initialized playback struct.
 */
void replay_uninitialize(playback_handle& handle);

/**
 * Restart playback from the beginning of the pcap file.
 *
 * @param[in] handle A handle to the initialized playback struct.
 */
void replay_reset(playback_handle& handle);

/**
 * Return the information on the next packet avaliable in the playback_handle.
 * This must be called BEFORE calling the read_next_packet function.
 *
 * @param[in] handle The playback handle.
 * @param[out] info The returned information on the next packet.
 *
 * @return The status on whether there is a new packet or not.
 */
bool next_packet_info(playback_handle& handle, packet_info& info);

/**
 * Read the data from the next packet avaliable in the playback_handle.
 * This must be called AFTER calling the next_packet_info function.
 *
 * @param[in] handle The playback handle.
 * @param[out] buf The buffer to write the recieved data to (Must be sized
 * appropriately.
 * @param[in] buffer_size The size of the output buffer.
 *
 * @return 0 on no new packet, > 0 the size of the bytes recieved.
 */
size_t read_packet(playback_handle& handle, uint8_t* buf, size_t buffer_size);

/**
 * Initialize the record handle for recording multi sensor pcap files. Source
 * and destination IPs must be provided with each packet.
 *
 * @param[in] file The file path to the target pcap to record to.
 * @param[in] frag_size The size of the fragments for packet fragmentation.
 * @param[in] use_sll_encapsulation Whether to use sll encapsulation.
 */
std::shared_ptr<record_handle> record_initialize(
    const std::string& file, int frag_size, bool use_sll_encapsulation = false);

/**
 * Uninitialize the record handle, closing underlying file.
 *
 * @param[in] handle An initialized handle for the recording state.
 */
void record_uninitialize(record_handle& handle);

/**
 * Record a buffer to a multi sensor record_handle pcap file.
 *
 * @param[in] handle The record handle that record_initialize has initted.
 * @param[in] src_ip The source address to label the packets with.
 * @param[in] dst_ip The destination address to label the packets with.
 * @param[in] src_port The source port to label the packets with.
 * @param[in] dst_port The destination port to label the packets with.
 * @param[in] buf The buffer to record to the pcap file.
 * @param[in] buffer_size The size of the buffer to record to the pcap file.
 * @param[in] microsecond_timestamp The timestamp to record the packet as
 * microseconds.
 */
void record_packet(record_handle& handle, const std::string& src_ip,
                   const std::string& dst_ip, int src_port, int dst_port,
                   const uint8_t* buf, size_t buffer_size,
                   uint64_t microsecond_timestamp);

/**
 * Record a buffer to a multi sensor record_handle pcap file.
 *
 * @param[in] handle The record handle that record_initialize has initted.
 * @param[in] info The packet_info object to use for the packet.
 * @param[in] buf The buffer to record to the pcap file.
 * @param[in] buffer_size The size of the buffer to record to the pcap file.
 */
void record_packet(record_handle& handle, const packet_info& info,
                   const uint8_t* buf, size_t buffer_size);

/**
 * Return the information about network streams in a pcap file.
 *
 * @param[in] file The pcap file to read.
 * @param[in] packets_to_process Number of packets to process < 0 for all of
 * them
 *
 * @return A pointer to the resulting stream_info
 */
std::shared_ptr<stream_info> get_stream_info(const std::string& file,
                                             int packets_to_process = -1);

/**
 * Return the information about network streams in a pcap file.
 *
 * @param[in] file The pcap file to read.
 * @param[in] progress_callback A callback to invoke after each packet is
 * scanned current: The current file offset delta: The delta in file offset
 *                              total: The total size of the file
 * @param[in] packets_per_callback Callback every n packets
 * @param[in] packets_to_process Number of packets to process < 0 for all of
 * them
 *
 * @return A pointer to the resulting stream_info
 */
std::shared_ptr<stream_info> get_stream_info(
    const std::string& file,
    std::function<void(uint64_t current, uint64_t delta, uint64_t total)>
        progress_callback,
    int packets_per_callback, int packets_to_process = -1);

/**
 * Return the information about network streams in a PcapReader and generate
 * indicies (if the PcapReader is an IndexedPcapReader).
 *
 * @param[in] pcap_reader The PcapReader
 * @param[in] sensor_info a set of sensor_info used to parse packets contained
 * in the file
 * @param[in] progress_callback A callback to invoke after each packet is
 * scanned current: The current file offset delta: The delta in file offset
 *                              total: The total size of the file
 * @param[in] packets_per_callback Callback every n packets
 * @param[in] packets_to_process Number of packets to process < 0 for all of
 * them
 *
 * @return A pointer to the resulting stream_info
 */
std::shared_ptr<stream_info> get_stream_info(
    PcapReader& pcap_reader,
    std::function<void(uint64_t, uint64_t, uint64_t)> progress_callback,
    int packets_per_callback, int packets_to_process = -1);
/**
 * Return a guess of the correct ports located in a pcap file.
 *
 * @param[in] info The stream_info structure generated from a specific pcap file
 * @param[in] lidar_packet_sizes The size of the lidar packets
 * @param[in] imu_packet_sizes The size of the imu packets
 * @param[in] lidar_spec The expected lidar port from the metadata(pass 0 for
 * unknown)
 * @param[in] imu_spec The expected imu port from the metadata(pass 0 for
 * unknown)
 *
 * @return A vector (sorted by most likely to least likely) of the guessed ports
 */
std::vector<guessed_ports> guess_ports(stream_info& info, int lidar_packet_size,
                                       int imu_packet_size,
                                       int expected_lidar_port,
                                       int expected_imu_port);
}  // namespace sensor_utils
}  // namespace ouster

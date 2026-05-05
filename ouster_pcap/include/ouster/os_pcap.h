/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief OS-1 pcap replay
 */

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/deprecation.h"
#include "ouster/pcap.h"
#include "ouster/types.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace pcap {
/**
 * Structure representing a hash key/sorting key for a udp stream
 */
struct OUSTER_API_CLASS StreamKey {
    std::string dst_ip;  ///< The destination IP
    std::string src_ip;  ///< The source IP
    int src_port;        ///< The src port
    int dst_port;        ///< The destination port

    OUSTER_API_FUNCTION
    bool operator==(const struct StreamKey& other) const;
};
/**
 * @deprecated Use `StreamKey` instead.
 */
OUSTER_DEPRECATED_TYPE(stream_key, StreamKey,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
}  // namespace pcap
}  // namespace sdk
}  // namespace ouster

/**
 * @brief Hash function specialization for stream_key.
 *
 * Allows stream_key to be used in hash-based containers like
 * std::unordered_map.
 */
template <>
struct OUSTER_API_CLASS std::hash<ouster::sdk::pcap::StreamKey> {
    OUSTER_API_FUNCTION
    std::size_t operator()(
        const ouster::sdk::pcap::StreamKey& key) const noexcept {
        return std::hash<std::string>{}(key.src_ip) ^
               (std::hash<std::string>{}(key.src_ip) << 1) ^
               (std::hash<int>{}(key.src_port << 2)) ^
               (std::hash<int>{}(key.dst_port << 3));
    }
};

namespace ouster {
namespace sdk {
namespace pcap {

using ts = std::chrono::microseconds;  ///< Microsecond timestamp

/**
 * To string method for packet info structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The packet_info to output.
 *
 * @return The new output stream containing concatted stream_in and data.
 */
OUSTER_API_FUNCTION
std::ostream& operator<<(std::ostream& stream_in, const PacketInfo& data);

/**
 * Structure representing a hash key/sorting key for a udp stream
 */
struct OUSTER_API_CLASS GuessedPorts {
    int lidar;  ///< Guessed lidar port
    int imu;    ///< Guessed imu port
};

/**
 * @deprecated Use `GuessedPorts` instead.
 */
OUSTER_DEPRECATED_TYPE(guessed_ports, GuessedPorts,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);

/**
 * To string method for stream_key structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The stream_key to output.
 *
 * @return The new output stream containing concatted stream_in and data.
 */
OUSTER_API_FUNCTION
std::ostream& operator<<(std::ostream& stream_in, const StreamKey& data);

/**
 * @brief Structure containing metadata for a single UDP stream.
 *
 * Holds packet counts, payload size distribution, fragmentation details,
 * and IP version usage.
 */
struct OUSTER_API_CLASS StreamData {
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
 * @deprecated Use `StreamData` instead.
 */
OUSTER_DEPRECATED_TYPE(stream_data, StreamData,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
/**
 * To string method for stream_data structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The stream_data to output.
 *
 * @return The new output stream containing concatted stream_in and data.
 */
OUSTER_API_FUNCTION
std::ostream& operator<<(std::ostream& stream_in, const StreamData& data);

/**
 * Structure representing the information about network streams in a pcap file
 */
struct OUSTER_API_CLASS StreamInfo {
    uint64_t total_packets;           ///< The total number of packets detected
    uint32_t encapsulation_protocol;  ///< The encapsulation protocol for the
                                      ///< pcap file

    ts timestamp_max;  ///< The latest timestamp detected
    ts timestamp_min;  ///< The earliest timestamp detected

    std::unordered_map<StreamKey, StreamData>
        udp_streams;  ///< Datastructure containing info on all of the different
                      ///< streams
};

/**
 * @deprecated Use `StreamInfo` instead.
 */
OUSTER_DEPRECATED_TYPE(stream_info, StreamInfo,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);

/**
 * To string method for stream info structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The stream_info to output.
 *
 * @return The new output stream containing concatted stream_info and data.
 */
OUSTER_API_FUNCTION
std::ostream& operator<<(std::ostream& stream_in, const StreamInfo& data);

/**
 * @struct PlaybackHandle
 *
 * @brief struct to hide the stepwise playback details.
 *
 * This struct handles stepwise playback details.
 */
struct PlaybackHandle;
/**
 * @deprecated Use `PlaybackHandle` instead.
 */
OUSTER_DEPRECATED_TYPE(playback_handle, PlaybackHandle,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);

/**
 * @struct RecordHandle
 *
 * @brief struct to hide the record details.
 *
 * This struct handles hiding the record details.
 */
struct RecordHandle;
/**
 * @deprecated Use `RecordHandle` instead.
 */
OUSTER_DEPRECATED_TYPE(record_handle, RecordHandle,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);

/**
 * Initialize the stepwise playback handle.
 *
 * @param[in] file The file path of the pcap file.
 *
 * @return A handle to the initialized playback struct.
 */
OUSTER_API_FUNCTION
std::shared_ptr<PlaybackHandle> replay_initialize(const std::string& file);

/**
 * Uninitialize the stepwise playback handle.
 *
 * @param[in] handle A handle to the initialized playback struct.
 */
OUSTER_API_FUNCTION
void replay_uninitialize(PlaybackHandle& handle);

/**
 * Restart playback from the beginning of the pcap file.
 *
 * @param[in] handle A handle to the initialized playback struct.
 */
OUSTER_API_FUNCTION
void replay_reset(PlaybackHandle& handle);

/**
 * Return the information on the next packet avaliable in the PlaybackHandle.
 * This must be called BEFORE calling the read_next_packet function.
 *
 * @param[in] handle The playback handle.
 * @param[out] info The returned information on the next packet.
 *
 * @return The status on whether there is a new packet or not.
 */
OUSTER_API_FUNCTION
bool next_packet_info(PlaybackHandle& handle, PacketInfo& info);

/**
 * Read the data from the next packet avaliable in the PlaybackHandle.
 * This must be called AFTER calling the next_packet_info function.
 *
 * @param[in] handle The playback handle.
 * @param[out] buf The buffer to write the recieved data to (Must be sized
 *                 appropriately.
 * @param[in] buffer_size The size of the output buffer.
 *
 * @return 0 on no new packet, > 0 the size of the bytes recieved.
 */
OUSTER_API_FUNCTION
size_t read_packet(PlaybackHandle& handle, uint8_t* buf, size_t buffer_size);

/**
 * Initialize the record handle for recording multi sensor pcap files. Source
 * and destination IPs must be provided with each packet.
 *
 * @param[in] file The file path to the target pcap to record to.
 * @param[in] frag_size The size of the fragments for packet fragmentation.
 * @param[in] use_sll_encapsulation Whether to use sll encapsulation.
 * @return RecordHandle A handle to the initialized record.
 */
OUSTER_API_FUNCTION
std::shared_ptr<RecordHandle> record_initialize(
    const std::string& file, int frag_size, bool use_sll_encapsulation = false);

/**
 * Uninitialize the record handle, closing underlying file.
 *
 * @param[in] handle An initialized handle for the recording state.
 */
OUSTER_API_FUNCTION
void record_uninitialize(RecordHandle& handle);

/**
 * Record a buffer to a multi sensor RecordHandle pcap file.
 *
 * @param[in] handle The record handle that record_initialize has initted.
 * @param[in] src_ip The source address to label the packets with.
 * @param[in] dst_ip The destination address to label the packets with.
 * @param[in] src_port The source port to label the packets with.
 * @param[in] dst_port The destination port to label the packets with.
 * @param[in] buf The buffer to record to the pcap file.
 * @param[in] buffer_size The size of the buffer to record to the pcap file.
 * @param[in] microsecond_timestamp The timestamp to record the packet as
 *                                  microseconds.
 */
OUSTER_API_FUNCTION
void record_packet(RecordHandle& handle, const std::string& src_ip,
                   const std::string& dst_ip, int src_port, int dst_port,
                   const uint8_t* buf, size_t buffer_size,
                   uint64_t microsecond_timestamp);

/**
 * Record a buffer to a multi sensor RecordHandle pcap file.
 *
 * @param[in] handle The record handle that record_initialize has initted.
 * @param[in] info The packet_info object to use for the packet.
 * @param[in] buf The buffer to record to the pcap file.
 * @param[in] buffer_size The size of the buffer to record to the pcap file.
 */
OUSTER_API_FUNCTION
void record_packet(RecordHandle& handle, const PacketInfo& info,
                   const uint8_t* buf, size_t buffer_size);

/**
 * Return the information about network streams in a pcap file.
 *
 * @param[in] file The pcap file to read.
 * @param[in] packets_to_process Number of packets to process < 0 for all of
 *                               them
 *
 * @return A pointer to the resulting stream_info
 */
OUSTER_API_FUNCTION
std::shared_ptr<StreamInfo> get_stream_info(const std::string& file,
                                            int packets_to_process = -1);

/**
 * Return the information about network streams in a pcap file.
 *
 * @param[in] file The pcap file to read.
 * @param[in] progress_callback A callback to invoke after each packet is
 *                              scanned
 *                                  current: The current file offset
 *                                  delta: The delta in file offset
 *                                  total: The total size of the file
 * @param[in] packets_per_callback Callback every n packets
 * @param[in] packets_to_process Number of packets to process < 0 for all of
 *                               them
 *
 * @return A pointer to the resulting stream_info
 */
OUSTER_API_FUNCTION
std::shared_ptr<StreamInfo> get_stream_info(
    const std::string& file,
    const std::function<void(uint64_t current, uint64_t delta, uint64_t total)>&
        progress_callback,
    int packets_per_callback, int packets_to_process = -1);

/**
 * Return the information about network streams in a PcapReader and generate
 * indicies (if the PcapReader is an IndexedPcapReader).
 *
 * @param[in] pcap_reader The PcapReader
 * @param[in] progress_callback A callback to invoke after each packet is
 *                              scanned
 *                                  current: The current file offset
 *                                  delta: The delta in file offset
 *                                  total: The total size of the file
 * @param[in] packets_per_callback Callback every n packets
 * @param[in] packets_to_process Number of packets to process < 0 for all of
 *                               them
 *
 * @return A pointer to the resulting stream_info
 */
OUSTER_API_FUNCTION
std::shared_ptr<StreamInfo> get_stream_info(
    PcapReader& pcap_reader,
    const std::function<void(uint64_t, uint64_t, uint64_t)>& progress_callback,
    int packets_per_callback, int packets_to_process = -1);
/**
 * Return a guess of the correct ports located in a pcap file.
 *
 * @param[in] info The stream_info structure generated from a specific pcap file
 * @param[in] lidar_packet_size The size of the lidar packets
 * @param[in] imu_packet_size The size of the imu packets
 * @param[in] expected_lidar_port The expected lidar port from the metadata
 *                                (pass 0 for unknown)
 * @param[in] expected_imu_port The expected imu port from the metadata
 *                              (pass 0 for unknown)
 *
 * @return A vector (sorted by most likely to least likely) of the guessed ports
 */
OUSTER_API_FUNCTION
std::vector<GuessedPorts> guess_ports(StreamInfo& info, int lidar_packet_size,
                                      int imu_packet_size,
                                      int expected_lidar_port,
                                      int expected_imu_port);
}  // namespace pcap
}  // namespace sdk
}  // namespace ouster

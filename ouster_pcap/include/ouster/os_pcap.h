/**
 * @file
 * @brief OS-1 pcap replay
 */

#pragma once

#include <tins/tins.h>

#include <cstdint>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ouster/impl/netcompat.h"

#define PROTOCOL_UDP 17

namespace ouster {
namespace sensor_utils {

struct packet_info {
    std::string dst_ip;   ///< The destination IP
    std::string src_ip;   ///< The source IP
    int dst_port;         ///< The destination port
    int src_port;         ///< The source port
    size_t payload_size;  ///< The size of the packet payload
    std::chrono::microseconds
        timestamp;  ///< The packet timestamp in std::chrono::duration
};

/**
 * To string method for packet info structs
 */
std::ostream& operator<<(std::ostream& stream_in, const packet_info& data);

/**
 * Struct to hide the stepwise playback details
 * @TODO This really should be opaque, however pybind does not like opaque
types, maybe make pybind trampolines to bypass
 */
struct playback_handle {
    std::string dst_ip;     ///< The destination IP
    std::string src_ip;     ///< The source IP
    std::string file_name;  ///< The filename of the pcap file
    std::unordered_map<int, int>
        port_map;  ///< Map containing port rewrite rules
    SOCKET replay_socket;

    std::unique_ptr<Tins::FileSniffer>
        pcap_reader;  ///< Object that holds the unified pcap reader
    Tins::Packet packet_cache;
    bool have_new_packet;

    Tins::IPv4Reassembler
        reassembler;  ///< The reassembler mainly for lidar packets
};

/**
 * Struct to hide the record details
 * @TODO This really should be opaque, however pybind does not like opaque
types, maybe make pybind trampolines to bypass
 */
struct record_handle {
    std::string dst_ip;     ///< The destination IP
    std::string src_ip;     ///< The source IP
    std::string file_name;  ///< The filename of the output pcap file
    size_t frag_size;       ///< The size of the udp data fragmentation
    std::unique_ptr<Tins::PacketWriter>
        pcap_file_writer;  ///< Object that holds the pcap writer
    bool use_sll_encapsulation;
};

/**
 * Struct that holds the summary of the stream info
 */
struct stream_info {
    std::unordered_map<int, std::unordered_map<int, int>>
        port_to_packet_sizes;  ///< Map from port numbers to packet sizes
    std::unordered_map<int, int>
        port_to_packet_count;  ///< Map from port numbers to packet counts
    std::unordered_map<int, std::unordered_map<int, int>>
        packet_size_to_port;  ///< Reverse map from packet sizes to port numbers
    uint64_t ipv6_packets;    ///< Number of ipv6 packets processed
    uint64_t ipv4_packets;    ///< Number of ipv4 packets reassembled
    uint64_t non_udp_packets;      ///< Number of non udp packets reassembled
    uint64_t packets_processed;    ///< Number of packets processed
    uint64_t packets_reassembled;  ///< Number of packets reassembled
};

/**
 * To string method for stream info structs
 */
std::ostream& operator<<(std::ostream& str_in, const stream_info& stream_data);

/**
 * Replay udp packets from pcap file
 * @param[in] handle A handle to the initialized playback struct
 * @param[in] double rate Speed multiplier; 1 is real-time, 0 plays back as fast
 * as packets can be read
 * @return number of packets sent
 */
int replay(playback_handle& handle, double rate);

/**
 * Get the stream info for a pcap file
 * @param[in] file The filename for the pcap file
 * @param[in] max_packets_to_process The maximum number of packets to process
 * @return The stream_info struct describing the pcap file
 * @relates stream_info
 */
std::shared_ptr<stream_info> replay_get_pcap_info(
    const std::string& file, size_t max_packets_to_process);

/**
 * Initialize the stepwise playback handle
 * @param[in] file The file path of the pcap file
 * @param[in] src_ip The source IP to send the packets from
 * @param[in] dst_ip The destination IP to send the packets to
 * @param[in] port_rewrite_map A map to handle destination port retargeting. If
 * a port does not exist in the map, the current port on the packet will be used
 * @return A handle to the initialized playback struct
 */
std::shared_ptr<playback_handle> replay_initialize(
    const std::string& file, const std::string& src_ip,
    const std::string& dst_ip, std::unordered_map<int, int> port_map);

/**
 * Initialize the stepwise playback handle
 * @param[in] file The file path of the pcap file
 * @return A handle to the initialized playback struct
 */
std::shared_ptr<playback_handle> replay_initialize(const std::string& file);

/**
 * Uninitialize the stepwise playback handle
 * @param[in] handle A handle to the initialized playback struct
 */
void replay_uninitialize(playback_handle& handle);

/**
 * Restart playback from the beginning of the pcap file
 * @param[in] handle A handle to the initialized playback struct
 */
void replay_reset(playback_handle& handle);

/**
 * Send the packet
 * @param[in] handle A handle to the initialized playback struct
 * @return If the packet was successfully sent, false normally means there are
no more packets
 */
bool replay_packet(playback_handle& handle);

/**
 * Return the information on the next packet avaliable in the playback_handle
 * This must be called BEFORE calling the read_next_packet function
 * @param[in] handle The playback handle
 * @param[out] info The returned information on the next packet
 * @return The status on whether there is a new packet or not
 */
bool next_packet_info(playback_handle& handle, packet_info& info);

/**
 * Read the data from the next packet avaliable in the playback_handle
 * This must be called AFTER calling the next_packet_info function
 * @param[in] handle The playback handle
 * @param[out] buf The buffer to write the recieved data to (Must be sized
 * appropriately
 * @param[in] buffer_size The size of the output buffer
 * @return 0 on no new packet, > 0 the size of the bytes recieved
 */
size_t read_packet(playback_handle& handle, uint8_t* buf, size_t buffer_size);

/**
 * Initialize the record handle for recording pcap files
 * @param[in] file The file path to the target pcap to record to
 * @param[in] src_ip The source address to label the packets with
 * @param[in] dst_ip The destination address to label the packets with
 * @param[in] frag_size The size of the fragments for packet fragmentation
 */
std::shared_ptr<record_handle> record_initialize(const std::string& file,
                                                 const std::string& src_ip,
                                                 const std::string& dst_ip,
                                                 int frag_size,
                                                 bool use_sll_encapsulation = false);
/**
 * Uninitialize the record handle, closing underlying file
 * @param[in] handle An initialized handle for the recording state
 */
void record_uninitialize(record_handle& handle);

/**
 * Record a buffer to a record_handle pcap file
 * @param[in] handle The record handle that record_initialize has initted
 * @param[in] src_port The source port to label the packets with
 * @param[in] dst_port The destination port to label the packets with
 * @param[in] buf The buffer to record to the pcap file
 * @param[in] buffer_size The size of the buffer to record to the pcap file
 * @param[in] microsecond_timestamp The timestamp to record the packet as
 * microseconds
 */
void record_packet(record_handle& handle, int src_port, int dst_port,
                   const uint8_t* buf, size_t buffer_size,
                   uint64_t microsecond_timestamp);

}  // namespace sensor_utils
}  // namespace ouster

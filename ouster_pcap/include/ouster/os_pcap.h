/**
 * @file
 * @brief OS-1 pcap replay
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
        timestamp;            ///< The packet timestamp in std::chrono::duration
    int fragments_in_packet;  ///< Number of fragments in the packet
    int ip_version;  ///< The version of the ip stack (if it is there, otherwise
                     ///< this will be 0)
    int encapsulation_protocol;  ///< The protocol of the encapsulation
    int network_protocol;  ///< The protocol of the network layer. Values listed
                           ///< here
                           ///< https://www.iana.org/assignments/protocol-numbers/protocol-numbers.xhtml
};

/**
 * To string method for packet info structs
 */
std::ostream& operator<<(std::ostream& stream_in, const packet_info& data);

/**
 * Struct to hide the stepwise playback details
 */
struct playback_handle;
std::shared_ptr<playback_handle> playback_handle_init();

/**
 * Struct to hide the record details
 */
struct record_handle;
std::shared_ptr<record_handle> record_handle_init();

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
std::shared_ptr<record_handle> record_initialize(
    const std::string& file, const std::string& src_ip,
    const std::string& dst_ip, int frag_size,
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

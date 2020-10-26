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

#define PROTOCOL_UDP 17

namespace ouster {
namespace sensor_utils {

/**
 * Struct to hold lidar/imu port info
 *
 */
struct port_couple {
    uint16_t lidar_port;
    uint16_t imu_port;
};

/**
 * To string method for port couple structs
 */
std::ostream& operator<<(std::ostream& stream_in, const port_couple& data);

/**
 * Struct to hide the stepwise playback details
 * @TODO This really should be opaque, however pybind does not like opaque types,
maybe make pybind trampolines to bypass
 */
struct playback_handle {
    std::string dst_ip;     ///< The destination IP
    std::string src_ip;     ///< The source IP
    std::string file_name;  ///< The filename of the pcap file
    port_couple src_ports;  ///< The source ports
    port_couple dst_ports;  ///< The destination ports
    std::unique_ptr<Tins::FileSniffer>
        pcap_file_lidar_reader;  ///< Object that holds the pcap reader for
                                 ///< lidar packets
    std::unique_ptr<Tins::FileSniffer>
        pcap_file_imu_reader;  ///< Object that holds the pcap reader for imu
                               ///< packets
    Tins::IPv4Reassembler
        reassembler;            ///< The reassembler mainly for lidar packets
    Tins::PacketSender sender;  ///< The sender object for sending packets
};

/**
 * Struct to hide the record details
 * @TODO This really should be opaque, however pybind does not like opaque types,
maybe make pybind trampolines to bypass
 */
struct record_handle {
    std::string dst_ip;     ///< The destination IP
    std::string src_ip;     ///< The source IP
    std::string file_name;  ///< The filename of the output pcap file
    size_t frag_size;       ///< The size of the udp data fragmentation
    std::unique_ptr<Tins::PacketWriter>
        pcap_file_writer;   ///< Object that holds the pcap writer
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
    int packets_processed;    ///< Number of packets processed
    int packets_reassembled;  ///< Number of packets reassembled
};

/**
 * To string method for stream info structs
 */
std::ostream& operator<<(std::ostream& str_in, const stream_info& stream_data);

/**
 * Replay udp packets from pcap file
 * @param[in] file Pcap file containing sensor udp packets
 * @param[in] dest_ip Ipv4 address to replace destination of each socket
 * @param[in] double rate Speed multiplier; 1 is real-time, 0 plays back as fast
 * as packets can be read
 * @return 0 on success, 1 on error
 */
int replay(const std::string& file, const std::string& src_ip,
           const std::string& dest_ip, double rate);

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
 * Guess the ports for lidar and imu streams from a stream_info struct
 * @param[in] stream_data The stream_info struct for a pcap file
 * @return A port_couple for the guessed lidar and imu ports
 */
port_couple guess_ports(const stream_info& stream_data);

/**
 * Initialize the stepwise playback handle
 * @param[in] file The file path of the pcap file
 * @param[in] src_ip The source IP to send the packets from
 * @param[in] dst_ip The destination IP to send the packets to
 * @param[in] src_ports The source ports to filter on (get this from
guess_ports)
 * @param[in] dst_ports The destination ports to send the packets to
 * @return A handle to the initialized playback struct
 */
std::shared_ptr<playback_handle> replay_initalize(const std::string& file,
                                                  const std::string& src_ip,
                                                  const std::string& dst_ip,
                                                  const port_couple& src_ports,
                                                  const port_couple& dst_ports);
/**
 * Initialize the stepwise playback handle
 * @param[in] file The file path of the pcap file
 * @param[in] src_ip The source IP to send the packets from
 * @param[in] dst_ip The destination IP to send the packets to
 * @param[in] src_ports The source ports to filter on (get this from
guess_ports)
 * @param[in] dst_lidar_port,dst_imu_port The destination ports to send the
packets to
 * @return A handle to the initialized playback struct
 */
std::shared_ptr<playback_handle> replay_initalize(const std::string& file,
                                                  const std::string& src_ip,
                                                  const std::string& dst_ip,
                                                  const port_couple& src_ports,
                                                  int dst_lidar_port,
                                                  int dst_imu_port);
/**
 * Initialize the stepwise playback handle
 * @param[in] file The file path of the pcap file
 * @return A handle to the initialized playback struct
 */
std::shared_ptr<playback_handle> replay_initalize(const std::string& file);

/**
 * Uninitialize the stepwise playback handle
 * @param[in] handle A handle to the initialized playback struct
 */
void replay_uninitialize(playback_handle& handle);

/**
 * Send the next lidar packet
 * @param[in] handle A handle to the initialized playback struct
 * @return If the packet was successfully sent, false normally means there are
no more packets
 */
bool replay_next_lidar_packet(playback_handle& handle);

/**
 * Send the next imu packet
 * @param[in] handle A handle to the initialized playback struct
 * @return If the packet was successfully sent, false normally means there are
no more packets
 */
bool replay_next_imu_packet(playback_handle& handle);

/**
 * Get the next lidar packet
 * @param[in] handle A handle to the initialized playback struct
 * @param[out] buf A character buffer to write data to
 * @param[in] buffer_size The size of the buffer in bytes
 * @return If the packet was successfully read, false normally means there are
no more packets
 */
bool get_next_lidar_data(playback_handle& handle, uint8_t* buf, size_t buffer_size);

/**
 * Get the next imu packet
 * @param[in] handle A handle to the initialized playback struct
 * @param[out] buf A character buffer to write data to
 * @param[in] buffer_size The size of the buffer in bytes
 * @return If the packet was successfully read, false normally means there are
no more packets
 */
bool get_next_imu_data(playback_handle& handle, uint8_t* buf,
                       size_t buffer_size);

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
                                                 int frag_size);

/**
 * Record a buffer to a record_handle pcap file
 * @param[in] handle The record handle that record_initialize has initted
 * @param[in] src_port The source port to label the packets with
 * @param[in] dst_port The destination port to label the packets with
 * @param[in] buf The buffer to record to the pcap file
 * @param[in] buffer_size The size of the buffer to record to the pcap file
 */
void record_packet(record_handle& handle,
                   int src_port,
                   int dst_port,
                   const uint8_t* buf, size_t buffer_size);

}  // namespace sensor_utils
}  // namespace ouster

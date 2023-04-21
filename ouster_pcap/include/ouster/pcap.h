/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace ouster {
namespace sensor_utils {

struct pcap_impl;
struct pcap_writer_impl;

static constexpr int IANA_UDP = 17;

struct packet_info {
    using ts = std::chrono::microseconds;

    // TODO: use numerical IPs for efficient filtering
    std::string dst_ip;          ///< The destination IP
    std::string src_ip;          ///< The source IP
    int dst_port;                ///< The destination port
    int src_port;                ///< The source port
    size_t payload_size;         ///< The size of the packet payload
    size_t packet_size;          ///< The size of the full packet
    ts timestamp;                ///< The packet capture timestamp
    int fragments_in_packet;     ///< Number of fragments in the packet
    int ip_version;              ///< The ip version, 4 or 6
    int encapsulation_protocol;  ///< PCAP encapsulation type
    uint64_t file_offset;        ///< Where the packet is in the pcap
    // TODO: remove, library ignores non-UDP packes
    int network_protocol;  ///< IANA protocol number. Always 17 (UDP)
};

/**
 * Class for dealing with reading pcap files
 */
class PcapReader {
    std::unique_ptr<pcap_impl> impl;    ///< Private implementation pointer
    packet_info info;                   ///< Cached packet info
    std::map<int, int> fragment_count;  ///< Map to count fragments per packet
    uint8_t* data;                      ///< Cached packet data

   public:
    /**
     * @param file[in] A filepath of the pcap to read
     */
    PcapReader(const std::string& file);
    virtual ~PcapReader();

    /**
     * Advances to the next packet and returns the size of that packet.
     * Will also populate data and info for next_packet(), current_data(),
     * current_length(), and current_info()
     *
     * @return The size of the packet payload
     */
    size_t next_packet();

    /**
     * Return the current packets data.
     * To advance to a new packet please use next_packet()
     * To get the size of the data use current_length()
     *
     * @return A pointer to a byte array containing the packet data
     */
    const uint8_t* current_data() const;

    /**
     * Return the current packets data size.
     * To advance to a new packet please use next_packet()
     *
     * @return The size of the byte array
     */
    size_t current_length() const;

    /**
     * Return the current packets info.
     * To advance to a new packet please use next_packet()
     *
     * @return A packet_info object on the current packet
     */
    const packet_info& current_info() const;

    /**
     * @return The size of the PCAP file in bytes
     */
    int64_t file_size() const;

    /**
     * Return the read position to the start of the PCAP file
     */
    void reset();

    /**
     * Seek to the position in the file represented by the
     * number of bytes from the beginning of the file.
     *
     * @param offset[in] The position to seek to in bytes,
     * starting from the beginning of the file.
     *
     * @pre \paramname{offset} must be the offset of a PCAP
     * record header. If any other value is provided,
     * subsequent packet reads from this PcapReader will be
     * invalid until \functionname{reset} is called.
     */
    void seek(uint64_t offset);

    int64_t current_offset() const;

   private:
    int64_t file_size_{};
    int64_t file_start_{};
};

/**
 * Class for dealing with writing udp pcap files
 */
class PcapWriter {
   public:
    /**
     * Enum to describe the current encapsulation for a pcap file
     */
    enum PacketEncapsulation {
        NULL_LOOPBACK = 0x0,  ///< Null Loopback Encapsulation
        ETHERNET = 0x1,       ///< Ethernet II Encapsulation
        SLL = 0x71,           ///< Linux Cooked Capture Encapsulation
    };

    /**
     * @param file[in] The file path to write the pcap to
     * @param encap[in] The encapsulation to use for the pcap
     * @param frag_size[in] The fragmentation size to use (Currently broken)
     */
    PcapWriter(const std::string& file, PacketEncapsulation encap,
               uint16_t frag_size);
    virtual ~PcapWriter();

    /**
     * Write a packet using a buffer to the pcap
     *
     * @param buf[in] The buffer to write
     * @param buf_size[in] The size of the buffer to write
     * @param src_ip[in] The source ip address to use for the packet
     * @param dst_ip[in] The destination ip address to use for the packet
     * @param src_port[in] The source port number to use for the packet
     * @param dst_port[in] The destination port number to use for the packet
     * @param timestamp[in] The timestamp of the packet to record
     * @note The timestamp parameter does not affect the order of packets being
     * recorded, it is strictly recorded FIFO.
     */
    void write_packet(const uint8_t* buf, size_t buf_size,
                      const std::string& src_ip, const std::string& dst_ip,
                      uint16_t src_port, uint16_t dst_port,
                      packet_info::ts timestamp);

    /**
     * Write a packet using a buffer to the pcap
     *
     * @param buf[in] The buffer to write
     * @param buf_size[in] The size of the buffer to write
     * @param info[in] The packet info object to use for the recording
     * parameters
     * @note The timestamp parameter in info does not affect the order of
     * packets being recorded, it is strictly recorded FIFO.
     */
    void write_packet(const uint8_t* buf, size_t buf_size,
                      const packet_info& info);

    /**
     * Write all pending data to the pcap file
     */
    void flush();

    /**
     * Flushes and cleans up all memory in use by the pap writer
     */
    void close();

   protected:
    std::unique_ptr<pcap_writer_impl> impl;  ///< Internal data

    uint16_t id;                ///< An incrementing id to record packets with
    PacketEncapsulation encap;  ///< Encapsulation to record with
    uint16_t frag_size;         ///< Fragmentation size(not currently used)
    bool closed;
};

/**
 * To string method for packet info structs.
 *
 * @param[inout] stream_in The pre-existing ostream to concat with data.
 * @param[in] data The packet_info to output.
 *
 * @return The new output stream containing concatted stream_in and data.
 */
std::ostream& operator<<(std::ostream& stream_in, const packet_info& data);
}  // namespace sensor_utils
}  // namespace ouster

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

#include "ouster/visibility.h"

namespace ouster {
namespace sensor_utils {

struct pcap_impl;
struct pcap_writer_impl;

static constexpr int IANA_UDP = 17;

struct OUSTER_API_CLASS packet_info {
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
class OUSTER_API_CLASS PcapReader {
   protected:
    std::unique_ptr<pcap_impl> impl_;  ///< Private implementation pointer
    packet_info info_;                 ///< Cached packet info
    uint8_t* data_;                    ///< Cached packet data

   public:
    /**
     * @param[in] file A filepath of the pcap to read
     */
    explicit OUSTER_API_FUNCTION PcapReader(const std::string& file);

    // Deleted due to the unique pointer not working with
    // the following
    OUSTER_API_FUNCTION
    PcapReader(const PcapReader& other) = delete;

    /**
     * Move construct from one PcapReader to a new PcapReader.
     *
     * @param[in] other The other PcapReader to move from.
     */
    OUSTER_API_FUNCTION
    PcapReader(PcapReader&& other);

    // Deleted due to the unique pointer not working with
    // the following
    OUSTER_API_FUNCTION
    PcapReader& operator=(PcapReader& other) = delete;

    /**
     * Assign move resources from one PcapReader to another.
     *
     * @param[in] other The other PcapReader to move from.
     */
    OUSTER_API_FUNCTION
    PcapReader& operator=(PcapReader&& other);

    /**
     * Destructor for cleaning up after PcapReader.
     */
    OUSTER_API_FUNCTION
    virtual ~PcapReader();

    /**
     * Advances to the next packet and returns the size of that packet.
     * Will also populate data and info for next_packet(), current_data(),
     * current_length(), and current_info()
     *
     * @return The size of the packet payload
     */
    OUSTER_API_FUNCTION
    size_t next_packet();

    /**
     * Return the current packets data.
     * To advance to a new packet please use next_packet()
     * To get the size of the data use current_length()
     *
     * @return A pointer to a byte array containing the packet data
     */
    OUSTER_API_FUNCTION
    const uint8_t* current_data() const;

    /**
     * Return the current packets data size.
     * To advance to a new packet please use next_packet()
     *
     * @return The size of the byte array
     */
    OUSTER_API_FUNCTION
    size_t current_length() const;

    /**
     * Return the current packets info.
     * To advance to a new packet please use next_packet()
     *
     * @return A packet_info object on the current packet
     */
    OUSTER_API_FUNCTION
    const packet_info& current_info() const;

    /**
     * @return The size of the PCAP file in bytes
     */
    OUSTER_API_FUNCTION
    int64_t file_size() const;

    /**
     * Return the read position to the start of the PCAP file
     */
    OUSTER_API_FUNCTION
    void reset();

    /**
     * Seek to the position in the file represented by the
     * number of bytes from the beginning of the file.
     *
     * @param[in] offset The position to seek to in bytes,
     * starting from the beginning of the file.
     *
     * @pre offset must be the offset of a PCAP
     * record header. If any other value is provided,
     * subsequent packet reads from this PcapReader will be
     * invalid until PcapReader::reset is called.
     */
    OUSTER_API_FUNCTION
    void seek(uint64_t offset);

    OUSTER_API_FUNCTION
    int64_t current_offset() const;

   private:
    int64_t file_size_{};
    int64_t file_start_{};
};

/**
 * Class for dealing with writing udp pcap files
 */
class OUSTER_API_CLASS PcapWriter {
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
     * @param[in] file The file path to write the pcap to
     * @param[in] encap The encapsulation to use for the pcap
     * @param[in] frag_size The fragmentation size to use (Currently broken)
     */
    OUSTER_API_FUNCTION
    PcapWriter(const std::string& file, PacketEncapsulation encap,
               uint16_t frag_size);

    // Deleted due to the unique pointer not working with
    // the following
    OUSTER_API_FUNCTION
    PcapWriter(const PcapWriter& other) = delete;

    /**
     * Move construct from one PcapWriter to a new PcapWriter.
     *
     * @param[in] other The other PcapWriter to move from.
     */
    OUSTER_API_FUNCTION
    PcapWriter(PcapWriter&& other);

    // Deleted due to the unique pointer not working with
    // the following
    OUSTER_API_FUNCTION
    PcapWriter& operator=(PcapWriter& other) = delete;

    /**
     * Assign move resources from one PcapWriter to another.
     *
     * @param[in] other The other PcapWriter to move from.
     */
    OUSTER_API_FUNCTION
    PcapWriter& operator=(PcapWriter&& other);

    /**
     * Destructor for cleaning up after PcapWriter.
     */
    OUSTER_API_FUNCTION
    virtual ~PcapWriter();

    /**
     * Write a packet using a buffer to the pcap
     *
     * @param[in] buf The buffer to write
     * @param[in] buf_size The size of the buffer to write
     * @param[in] src_ip The source ip address to use for the packet
     * @param[in] dst_ip The destination ip address to use for the packet
     * @param[in] src_port The source port number to use for the packet
     * @param[in] dst_port The destination port number to use for the packet
     * @param[in] timestamp The timestamp of the packet to record
     * @note The timestamp parameter does not affect the order of packets being
     * recorded, it is strictly recorded FIFO.
     */
    OUSTER_API_FUNCTION
    void write_packet(const uint8_t* buf, size_t buf_size,
                      const std::string& src_ip, const std::string& dst_ip,
                      uint16_t src_port, uint16_t dst_port,
                      packet_info::ts timestamp);

    /**
     * Write a packet using a buffer to the pcap
     *
     * @param[in] buf The buffer to write
     * @param[in] buf_size The size of the buffer to write
     * @param[in] info The packet info object to use for the recording
     * parameters
     * @note The timestamp parameter in info does not affect the order of
     * packets being recorded, it is strictly recorded FIFO.
     */
    OUSTER_API_FUNCTION
    void write_packet(const uint8_t* buf, size_t buf_size,
                      const packet_info& info);

    /**
     * Write all pending data to the pcap file
     */
    OUSTER_API_FUNCTION
    void flush();

    /**
     * Flushes and cleans up all memory in use by the pap writer
     */
    OUSTER_API_FUNCTION
    void close();

   protected:
    std::unique_ptr<pcap_writer_impl> impl_;  ///< Internal data

    /// @todo figure out if some of these are still needed
    uint16_t id_;                ///< An incrementing id to record packets with
    PacketEncapsulation encap_;  ///< Encapsulation to record with
    uint16_t frag_size_;         ///< Fragmentation size(not currently used)
    bool closed_;
};
}  // namespace sensor_utils
}  // namespace ouster

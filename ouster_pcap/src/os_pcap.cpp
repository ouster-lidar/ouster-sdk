/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 */

#include "ouster/os_pcap.h"

#include <stddef.h>
#include <tins/tins.h>

#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <thread>

using namespace Tins;

namespace ouster {
namespace sensor_utils {

static constexpr int PROTOCOL_UDP = 17;

struct record_handle {
    std::string dst_ip;     ///< The destination IP
    std::string src_ip;     ///< The source IP
    std::string file_name;  ///< The filename of the output pcap file
    size_t frag_size;       ///< The size of the udp data fragmentation
    std::unique_ptr<Tins::PacketWriter>
        pcap_file_writer;  ///< Object that holds the pcap writer
    bool use_sll_encapsulation;

    record_handle() {}

    ~record_handle() {}
};

struct playback_handle {
    std::string file_name;  ///< The filename of the pcap file

    std::unique_ptr<Tins::FileSniffer>
        pcap_reader;  ///< Object that holds the unified pcap reader
    Tins::Packet packet_cache;
    bool have_new_packet;

    Tins::IPv4Reassembler
        reassembler;  ///< The reassembler mainly for lidar packets

    playback_handle() {}

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
    const std::string& file_name) {
    std::shared_ptr<playback_handle> result =
        std::make_shared<playback_handle>();

    result->file_name = file_name;
    result->pcap_reader.reset(new FileSniffer(file_name));

    return result;
}

void replay_uninitialize(playback_handle& handle) {
    handle.pcap_reader.reset();
}

void replay_reset(playback_handle& handle) {
    handle.pcap_reader.reset(new FileSniffer(handle.file_name));
}

bool next_packet_info(playback_handle& handle, packet_info& info) {
    bool result = false;

    bool reassm = false;
    int reassm_packets = 0;
    while (!reassm) {
        reassm_packets++;
        handle.packet_cache = handle.pcap_reader->next_packet();
        if (handle.packet_cache) {
            auto pdu = handle.packet_cache.pdu();
            if (pdu) {
                IP* ip = pdu->find_pdu<IP>();
                IPv6* ipv6 = pdu->find_pdu<IPv6>();
                // Using short circuiting here
                if ((ip && ip->protocol() == PROTOCOL_UDP) ||
                    (ipv6 && ipv6->next_header() == PROTOCOL_UDP)) {
                    // reassm is also used in the while loop
                    reassm = (handle.reassembler.process(*pdu) !=
                              IPv4Reassembler::FRAGMENTED);
                    if (reassm) {
                        info.fragments_in_packet = reassm_packets;
                        reassm_packets = 0;

                        info.encapsulation_protocol = (int)pdu->pdu_type();
                        result = true;
                        UDP* udp = pdu->find_pdu<UDP>();
                        auto raw = pdu->find_pdu<RawPDU>();
                        if (ip) {
                            info.dst_ip = ip->dst_addr().to_string();
                            info.src_ip = ip->src_addr().to_string();
                            info.ip_version = 4;
                            info.network_protocol = ip->protocol();
                        } else if (ipv6) {
                            info.dst_ip = ipv6->dst_addr().to_string();
                            info.src_ip = ipv6->src_addr().to_string();
                            info.ip_version = 6;
                            info.network_protocol = ipv6->next_header();
                        } else {
                            throw std::runtime_error(
                                "Malformed packet: no IP headers");
                        }
                        // find_pdu<UDP> will only return NULL when the ipv4
                        // reassembly succeeds on an ipv6 packet, leading to a
                        // malformed packet
                        if (udp != NULL) {
                            info.dst_port = udp->dport();
                            info.src_port = udp->sport();
                            info.payload_size = raw->payload_size();
                            info.timestamp = handle.packet_cache.timestamp();
                            handle.have_new_packet = true;
                        } else {
                            throw std::runtime_error(
                                "Malformed Packet: No UDP Detected");
                        }
                    }
                }
            }
        } else {
            reassm = true;
        }
    }

    return result;
}

size_t read_packet(playback_handle& handle, uint8_t* buf, size_t buffer_size) {
    size_t result = 0;
    if (handle.have_new_packet) {
        result = true;
        auto pdu = handle.packet_cache.pdu();
        auto raw = pdu->find_pdu<RawPDU>();
        if (raw) {
            auto temp = (uint32_t*)&(raw->payload()[0]);
            auto size = raw->payload_size();
            if (size > buffer_size) {
                throw std::invalid_argument(
                    "Incompatible argument: expected a bytearray of "
                    "size > " +
                    std::to_string(size));
            } else {
                handle.have_new_packet = false;
                result = size;
                memcpy(buf, temp, size);
            }
        }
    }

    return result;
}

std::shared_ptr<record_handle> record_initialize(const std::string& file_name,
                                                 const std::string& src_ip,
                                                 const std::string& dst_ip,
                                                 int frag_size,
                                                 bool use_sll_encapsulation) {
    std::shared_ptr<record_handle> result = std::make_shared<record_handle>();

    result->file_name = file_name;
    result->frag_size = frag_size;
    result->src_ip = src_ip;
    result->dst_ip = dst_ip;
    result->use_sll_encapsulation = use_sll_encapsulation;
    if (use_sll_encapsulation) {
        result->pcap_file_writer.reset(
            new PacketWriter(file_name, DataLinkType<SLL>()));
    } else {
        result->pcap_file_writer.reset(
            new PacketWriter(file_name, DataLinkType<EthernetII>()));
    }
    return result;
}

void record_uninitialize(record_handle& handle) {
    if (handle.pcap_file_writer) handle.pcap_file_writer.reset();
}

/*
 * This was a tricky problem, due to how the ip stack is set up.
 *
 * SLL Container -> ipv4 container -> udp container -> raw data
 *
 * The ipv4 container is what does the packet fragmentation and reassembly.
 * With each full packet we need the ipv4 reassembly to contain only one udp
header.
* Due to this, we have to only create one official Tins::UDP packet.
* We grab the packet id from this packet
* Every packet after the first Tins::UDP packet needs to just be a ipv4
* container with a manually set packet type of UDP(dec 17)
* Every packet but the final packet needs to have the current flag set:
*
* pkt.flags(IP::MORE_FRAGMENTS);
*
*/

// SLL is the linux pcap capture container
std::vector<IP> buffer_to_frag_packets(record_handle& handle, int src_port,
                                       int dst_port, const uint8_t* buf,
                                       size_t buf_size) {
    std::vector<IP> result;

    int id = -1;   ///< This variable is used to track the packet id,
                   ///< if -1 then create a packet and grab its id
    size_t i = 0;  ///< Loop variable that contains current bytes processed
    size_t offset_modifier =
        0;  ///< This variable contains the offset to account
            ///< for the udp packet with the fragment_offset

    while (i < buf_size) {
        // First create the ipv4 packet
        IP pkt = IP(handle.src_ip, handle.dst_ip);

        // Now figure out the size of the packet payload
        size_t size = std::min(handle.frag_size, (buf_size - i));

        // Correctly set this packets fragment offset
        // NOTE: for some reason this has to be divided by 8
        // NOTE: Reference here
        // http://libtins.github.io/docs/latest/dd/d3f/classTins_1_1IP.html#a32a6bf84af274748317ef61ce1a91ce5
        pkt.fragment_offset((i + offset_modifier) / 8);

        // If this is the first packet
        if (i == 0) {
            // Fully create the libtins udp structure
            auto udp = UDP(dst_port, src_port);

            if ((size + udp.header_size()) > handle.frag_size) {
                // Due to the udp header being included in the payload,
                // we need to subtract its size from the payload
                size -= udp.header_size();
                // Set the "There is more data to follow" flag
                pkt.flags(IP::MORE_FRAGMENTS);
            }

            // Pack what we can minus the udp header size into the payload
            pkt /= udp / RawPDU((uint8_t*)(buf + i), size);

            // Manually set the ipv4 protocol to UDP
            pkt.protocol(PROTOCOL_UDP);

            // Set the fragment_offset offset with the size of the udp header
            offset_modifier = udp.header_size();
        }
        // This is a packet in the middle or end
        else {
            // Set the "There is more data to follow" flag
            if (i + size < buf_size) pkt.flags(IP::MORE_FRAGMENTS);

            // Manually set the ipv4 protocol to UDP
            pkt.protocol(PROTOCOL_UDP);

            // Pack what we can into the payload
            pkt /= RawPDU((uint8_t*)(buf + i), size);
        }

        // Here is where we correctly set the packet id
        if (id < 0) {
            // If this is the first packet, set id to the generated packet id
            id = pkt.id();
        } else {
            // If this is a following packet, use the first packets id
            pkt.id(id);
        }

        // Add the resulting packet to the vector
        result.push_back(pkt);

        // Increment the current byte being processed
        i += size;
    }

    return result;
}

void record_packet(record_handle& handle, int src_port, int dst_port,
                   const uint8_t* buf, size_t buffer_size,
                   uint64_t microsecond_timestamp) {
    // For each of the packets write it to the pcap file
    for (auto item :
         buffer_to_frag_packets(handle, src_port, dst_port, buf, buffer_size)) {
        Packet packet;
        PDU* pdu;
        if (handle.use_sll_encapsulation) {
            pdu = new SLL();
        } else {
            pdu = new EthernetII();
        }
        *pdu /= item;
        // Nasty libtins bug that causes write to fail
        // https://www.gitmemory.com/issue/mfontanini/libtins/348/488141933
        auto _ = pdu->serialize();
        /*
         * The next block is due to the fact that the previous serialize does
         * not treat the udp packet as if it were decodable. Manually tell
         * libtins to go in and serialize the udp packet as well
         */
        if (pdu->inner_pdu()->inner_pdu()->inner_pdu() != NULL) {
            _ = pdu->inner_pdu()->inner_pdu()->inner_pdu()->serialize();
        }
        packet = Packet(
            *pdu, Timestamp(std::chrono::microseconds{microsecond_timestamp}));
        handle.pcap_file_writer->write(packet);
        delete pdu;
    }
}

}  // namespace sensor_utils
}  // namespace ouster

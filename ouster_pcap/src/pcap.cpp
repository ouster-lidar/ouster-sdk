/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @todo check that the header casting is idiomatic libpcap
 * @todo warn on dropped packets when pcap contains garbage, when fragments
 * missing, buffer reused before sending
 * @todo split up reading / playback
 * @todo improve error reporting
 */

#include "ouster/pcap.h"

#if defined _WIN32
#include <winsock2.h>
#define FTELL _ftelli64
#define FSEEK _fseeki64
#elif defined __EMSCRIPTEN__
#define FTELL ftello
#define FSEEK fseeko
#else
#include <arpa/inet.h>  // inet_ntop
#include <sys/time.h>   // timeval
#define FTELL ftello
#define FSEEK fseeko
#endif

#include <pcap.h>
#if defined _WIN32
#include <pcap/bpf.h>
#else
#include <pcap/dlt.h>
#endif
#include <pcap/pcap.h>
#include <tins/tins.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>

#include "ip_reassembler.h"

using us = std::chrono::microseconds;
using timepoint = std::chrono::system_clock::time_point;
using namespace Tins;

static constexpr size_t UDP_BUF_SIZE = 65535;
static constexpr int PROTOCOL_UDP = 17;

namespace ouster {
namespace sensor_utils {

struct pcap_impl {
    pcap_t* handle;
    std::unique_ptr<Tins::FileSniffer>
        pcap_reader;  ///< Object that holds the unified pcap reader
    FILE* pcap_reader_internals;
    Tins::Packet packet_cache;
    IPv4Reassembler2 reassembler;  ///< The reassembler mainly for lidar packets
    bool have_new_packet;
    int encap_proto;
};

struct pcap_writer_impl {
    pcap_t* handle;
    pcap_dumper* dumper;
    std::unique_ptr<Tins::PacketWriter>
        pcap_file_writer;  ///< Object that holds the pcap writer
};

PcapReader::PcapReader(const std::string& file) : impl_(new pcap_impl) {
    std::ifstream fileSizeStream(file, std::ios::binary);
    if (fileSizeStream) {
        fileSizeStream.seekg(0, std::ios::end);
        file_size_ = fileSizeStream.tellg();
    }
    impl_->pcap_reader = std::make_unique<Tins::FileSniffer>(file);
    impl_->encap_proto = impl_->pcap_reader->link_type();
    impl_->pcap_reader_internals =
        pcap_file(impl_->pcap_reader->get_pcap_handle());
    file_start_ = FTELL(impl_->pcap_reader_internals);
}

PcapReader::PcapReader(PcapReader&& other) = default;

PcapReader& PcapReader::operator=(PcapReader&& other) = default;

PcapReader::~PcapReader() = default;

const uint8_t* PcapReader::current_data() const { return data_; }

size_t PcapReader::current_length() const { return info_.payload_size; }

const packet_info& PcapReader::current_info() const { return info_; }

void PcapReader::seek(uint64_t offset) {
    if (offset < sizeof(struct pcap_file_header)) {
        offset = sizeof(struct pcap_file_header);
    }
    if (FSEEK(impl_->pcap_reader_internals, offset, SEEK_SET)) {
        throw std::runtime_error("pcap seek failed");
    }
}

int64_t PcapReader::file_size() const { return file_size_; }

int64_t PcapReader::current_offset() const {
    int64_t ret = FTELL(impl_->pcap_reader_internals);

    if (ret == -1L) {
        fclose(impl_->pcap_reader_internals);
        throw std::runtime_error("ftell error: errno " + std::to_string(errno));
    }
    return ret;
}

void PcapReader::reset() { seek(file_start_); }

size_t PcapReader::next_packet() {
    size_t result = 0;

    bool reassm = false;
    int reassm_packets = 0;
    while (!reassm) {
        reassm_packets++;
        info_.file_offset = current_offset();
        impl_->packet_cache = impl_->pcap_reader->next_packet();
        if (impl_->packet_cache) {
            auto pdu = impl_->packet_cache.pdu();
            if (pdu) {
                info_.packet_size = pdu->size();
                IP* ip = pdu->find_pdu<IP>();
                IPv6* ipv6 = pdu->find_pdu<IPv6>();
                // Using short circuiting here
                if ((ip && ip->protocol() == PROTOCOL_UDP) ||
                    (ipv6 && ipv6->next_header() == PROTOCOL_UDP)) {
                    // reassm is also used in the while loop
                    reassm = (impl_->reassembler.process(
                                  impl_->packet_cache.timestamp(), *pdu) !=
                              IPv4Reassembler2::FRAGMENTED);
                    if (reassm) {
                        info_.fragments_in_packet = reassm_packets;
                        info_.encapsulation_protocol = impl_->encap_proto;

                        if (ip) {
                            info_.dst_ip = ip->dst_addr().to_string();
                            info_.src_ip = ip->src_addr().to_string();
                            info_.ip_version = 4;
                            info_.network_protocol = ip->protocol();
                        } else if (ipv6) {
                            info_.dst_ip = ipv6->dst_addr().to_string();
                            info_.src_ip = ipv6->src_addr().to_string();
                            info_.ip_version = 6;
                            info_.network_protocol = ipv6->next_header();
                        }

                        // find_pdu<UDP> will only return NULL when the ipv4
                        // reassembly succeeds on an ipv6 packet, leading to a
                        // malformed packet
                        UDP* udp = pdu->find_pdu<UDP>();
                        if (udp != nullptr) {
                            auto raw = pdu->find_pdu<RawPDU>();
                            info_.dst_port = udp->dport();
                            info_.src_port = udp->sport();
                            info_.payload_size = raw->payload_size();
                            result = info_.payload_size;
                            data_ = raw->payload().data();
                            info_.timestamp = impl_->packet_cache.timestamp();
                            impl_->have_new_packet = true;
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

PcapWriter::PcapWriter(
    const std::string& file,
    PcapWriter::PacketEncapsulation encap = PcapWriter::ETHERNET,
    uint16_t frag_size = 1500)
    : impl_(new pcap_writer_impl),
      id_{0},
      encap_(encap),
      frag_size_(frag_size),
      closed_(false) {
    if (encap_ != PcapWriter::ETHERNET) {
        impl_->pcap_file_writer.reset(
            new Tins::PacketWriter((file), Tins::DataLinkType<Tins::SLL>()));
    } else {
        impl_->pcap_file_writer.reset(new Tins::PacketWriter(
            (file), Tins::DataLinkType<Tins::EthernetII>()));
    }
}

void PcapWriter::flush() {}

void PcapWriter::close() {
    flush();
    closed_ = true;
}

PcapWriter::PcapWriter(PcapWriter&& other) = default;

PcapWriter& PcapWriter::operator=(PcapWriter&& other) = default;

PcapWriter::~PcapWriter() { close(); }

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
size_t global_id = 1;
// SLL is the linux pcap capture container
std::vector<IP> buffer_to_frag_packets(size_t frag_size,
                                       const std::string& src_ip,
                                       const std::string& dst_ip, int src_port,
                                       int dst_port, const uint8_t* buf,
                                       size_t buf_size) {
    /// @todo check fragsize to make sure it is in acceptable bounds
    frag_size = UDP_BUF_SIZE;

    std::vector<IP> result;

    int id = -1;   ///< This variable is used to track the packet id,
                   ///< if -1 then create a packet and grab its id
    size_t i = 0;  ///< Loop variable that contains current bytes processed
    size_t offset_modifier =
        0;  ///< This variable contains the offset to account
            ///< for the udp packet with the fragment_offset

    while (i < buf_size) {
        // First create the ipv4 packet
        IP pkt = IP(dst_ip, src_ip);

        // Now figure out the size of the packet payload
        size_t size = std::min(frag_size, (buf_size - i));

        // Correctly set this packets fragment offset
        // NOTE: for some reason this has to be divided by 8
        // NOTE: Reference here
        // http://libtins.github.io/docs/latest/dd/d3f/classTins_1_1IP.html#a32a6bf84af274748317ef61ce1a91ce5
        pkt.fragment_offset((i + offset_modifier) / 8);

        // If this is the first packet
        if (i == 0) {
            // Fully create the libtins udp structure
            auto udp = UDP(dst_port, src_port);

            if ((size + udp.header_size()) > frag_size) {
                // Due to the udp header being included in the payload,
                // we need to subtract its size from the payload
                size -= udp.header_size();
                // Set the "There is more data to follow" flag
                pkt.flags(IP::MORE_FRAGMENTS);
            }

            // Pack what we can minus the udp header size into the payload
            pkt /= udp / RawPDU(buf + i, size);

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
            pkt /= RawPDU(buf + i, size);
        }

        // Here is where we correctly set the packet id
        if (id < 0) {
            // If this is the first packet, set id to the generated packet id
            id = global_id;
            pkt.id(id);
            global_id++;
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

void PcapWriter::write_packet(const uint8_t* buf, size_t buf_size,
                              const std::string& src_ip,
                              const std::string& dst_ip, uint16_t src_port,
                              uint16_t dst_port, packet_info::ts timestamp) {
    // ensure IPs were provided
    if (dst_ip.empty() || src_ip.empty()) {
        throw std::invalid_argument(
            "PcapWriter: dst_ip and/or src_ip arguments to write_packet cannot "
            "be empty.");
    }
    // For each of the packets write it to the pcap file
    for (auto item : buffer_to_frag_packets(
             frag_size_, src_ip, dst_ip, src_port, dst_port, buf, buf_size)) {
        Packet packet;
        PDU* pdu;
        switch (encap_) {
            case PcapWriter::PacketEncapsulation::ETHERNET:
                pdu = new Tins::EthernetII();
                break;
            case PcapWriter::PacketEncapsulation::SLL:
                pdu = new Tins::SLL();
                break;
            case PcapWriter::PacketEncapsulation::NULL_LOOPBACK:
                throw std::runtime_error(
                    "PcapWriter: NULL_LOOPBACK packet encapsulation not "
                    "supported");
                break;
            default:
                throw std::runtime_error(
                    "PcapWriter: packet encapsulation not supported");
        }
        *pdu /= item;

        /// @todo look into cleaning up the serialize call.
        /// Need to figure out if we can just ignore the
        /// return variable or not
        // Nasty libtins bug that causes write to fail
        // https://github.com/mfontanini/libtins/issues/348
        auto ignore_output = pdu->serialize();
        /*
         * The next block is due to the fact that the previous serialize does
         * not treat the udp packet as if it were decodable. Manually tell
         * libtins to go in and serialize the udp packet as well
         */
        if (pdu->inner_pdu()->inner_pdu()->inner_pdu() != NULL) {
            ignore_output =
                pdu->inner_pdu()->inner_pdu()->inner_pdu()->serialize();
        }
        packet = Packet(*pdu, timestamp);
        impl_->pcap_file_writer->write(packet);
        delete pdu;
    }
}

void PcapWriter::write_packet(const uint8_t* buf, size_t buf_size,
                              const packet_info& info) {
    write_packet(buf, buf_size, info.src_ip, info.dst_ip, info.src_port,
                 info.dst_port, info.timestamp);
}

}  // namespace sensor_utils
}  // namespace ouster

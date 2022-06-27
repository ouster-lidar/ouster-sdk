/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @TODO check that the header casting is idiomatic libpcap
 * @TODO warn on dropped packets when pcap contains garbage, when fragments
 * missing, buffer reused before sending
 * @TODO split up reading / playback
 * @TODO improve error reporting
 */

#include "ouster/pcap.h"

#if defined _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>  // inet_ntop
#include <sys/time.h>   // timeval
#endif

#include <pcap.h>
#if defined _WIN32
#include <pcap/bpf.h>
#else
#include <pcap/dlt.h>
#endif
#include <pcap/pcap.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <EthLayer.h>
#include <IPReassembly.h>
#include <IPv4Layer.h>
#include <IPv6Extensions.h>
#include <IPv6Layer.h>
#include <NullLoopbackLayer.h>
#include <Packet.h>
#include <PayloadLayer.h>
#include <ProtocolType.h>
#include <SllLayer.h>
#include <UdpLayer.h>
#pragma GCC diagnostic pop
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include "GeneralUtils.h"

using us = std::chrono::microseconds;
using timepoint = std::chrono::system_clock::time_point;

static constexpr size_t UDP_BUF_SIZE = 65535;

static constexpr uint16_t SLL_PACKET_SENT_TO_US = 0;
static constexpr uint16_t SLL_ARPHRD_ETHER = 3;
static constexpr uint32_t NULL_FAMILY_IPV4 = 2;

constexpr size_t UDPHDR_LEN = 8;
constexpr size_t IPHDR_LEN = 20;

namespace ouster {
namespace sensor_utils {

struct pcap_impl {
    pcpp::IPReassembly assembler;
    pcap_t* handle;
};

struct pcap_writer_impl {
    pcap_t* handle;
    pcap_dumper* dumper;
};

PcapReader::PcapReader(const std::string& file) : impl(new pcap_impl) {
    char errbuf[PCAP_ERRBUF_SIZE];
    impl->handle = pcap_open_offline(file.c_str(), errbuf);

    if (!impl->handle) {
        throw std::invalid_argument{std::string{"pcap_open: "} + errbuf};
    }
}

PcapReader::~PcapReader() { pcap_close(impl->handle); }

const uint8_t* PcapReader::current_data() const { return data; }

size_t PcapReader::current_length() const { return info.payload_size; }

const packet_info& PcapReader::current_info() const { return info; }

static struct timeval ts_to_timeval(us ts) {
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(ts);
    auto usec = std::chrono::duration_cast<us>(ts - sec);

    struct timeval tv;
    tv.tv_sec = sec.count();
    tv.tv_usec = usec.count();
    return tv;
}

static us timeval_to_ts(struct timeval tv) {
    return us{std::chrono::seconds{tv.tv_sec} + us{tv.tv_usec}};
}

int fragment_id(pcpp::Packet& packet) {
    int id = -1;
    pcpp::IPv4Layer* tempIpv4 = packet.getLayerOfType<pcpp::IPv4Layer>();
    if (tempIpv4 && tempIpv4->isFragment()) {
        auto header = tempIpv4->getIPv4Header();
        if (header) {
            id = header->ipId;
        }
    }
    pcpp::IPv6Layer* tempIpv6 = packet.getLayerOfType<pcpp::IPv6Layer>();
    if (tempIpv6 && tempIpv6->isFragment()) {
        pcpp::IPv6FragmentationHeader* frag_extension =
            tempIpv6->getExtensionOfType<pcpp::IPv6FragmentationHeader>();
        if (frag_extension) {
            pcpp::IPv6FragmentationHeader::ipv6_frag_header* frag_header =
                frag_extension->getFragHeader();
            if (frag_header) {
                id = frag_header->id;
            }
        }
    }
    return id;
}

size_t PcapReader::next_packet() {
    pcap_pkthdr header;
    const u_char* p;
    size_t result = 0;
    pcpp::IPReassembly::ReassemblyStatus status;
    pcpp::Packet* packet;
    auto datalink = pcap_datalink(impl->handle);

    while ((p = pcap_next(impl->handle, &header))) {
        pcpp::RawPacket raw((uint8_t*)p, header.len, header.ts, false,
                            (pcpp::LinkLayerType)datalink);

        pcpp::Packet processed(&raw);
        int fragId = fragment_id(processed);
        fragment_count[fragId]++;
        packet = impl->assembler.processPacket(&processed, status);

        if (status != pcpp::IPReassembly::ReassemblyStatus::REASSEMBLED &&
            status != pcpp::IPReassembly::ReassemblyStatus::NON_FRAGMENT)
            continue;
        if (!packet) continue;
        pcpp::IPv4Layer* ipv4Layer = packet->getLayerOfType<pcpp::IPv4Layer>();
        pcpp::IPv6Layer* ipv6Layer = packet->getLayerOfType<pcpp::IPv6Layer>();
        pcpp::UdpLayer* udpLayer = packet->getLayerOfType<pcpp::UdpLayer>();
        auto dataLayer = packet->getLastLayer();

        info.encapsulation_protocol = (int)datalink;

        info.timestamp = timeval_to_ts(header.ts);

        /// @TODO implement max map size for fragment_count logic (with
        /// flushing)
        info.fragments_in_packet = fragment_count[fragId]++;
        fragment_count.erase(fragId);

        info.network_protocol = IANA_UDP;

        if (!ipv4Layer && !ipv6Layer) continue;
        if (ipv4Layer) {
            info.dst_ip = ipv4Layer->getDstIPAddress().toString();
            info.src_ip = ipv4Layer->getSrcIPAddress().toString();

            info.ip_version = 4;
        } else {
            info.dst_ip = ipv6Layer->getDstIPAddress().toString();
            info.src_ip = ipv6Layer->getSrcIPAddress().toString();

            info.ip_version = 6;
        }

        if (!udpLayer) continue;
        auto udpHeader = udpLayer->getUdpHeader();
        if (!udpHeader) continue;
        // @TODO Latest pcapplusplus removes the need for this, remove this
        info.dst_port = htons(udpHeader->portDst);
        // @TODO Latest pcapplusplus removes the need for this, remove this
        info.src_port = htons(udpHeader->portSrc);
        if (!dataLayer) continue;
        info.payload_size = dataLayer->getDataLen();
        data = dataLayer->getData();
        result = info.payload_size;

        break;
    }

    return result;
}

PcapWriter::PcapWriter(
    const std::string& file,
    PcapWriter::PacketEncapsulation encap = PcapWriter::ETHERNET,
    uint16_t frag_size = 1500)
    : impl(new pcap_writer_impl), id{0}, encap(encap), frag_size(frag_size) {
    impl->handle = pcap_open_dead((int)encap, UDP_BUF_SIZE);

    impl->dumper = pcap_dump_open(impl->handle, file.c_str());
    if (!impl->dumper) {
        pcap_close(impl->handle);
        throw std::invalid_argument{std::string{"pcap_dump_open: "} +
                                    pcap_geterr(impl->handle)};
    }
}

PcapWriter::~PcapWriter() {
    pcap_dump_flush((pcap_dumper_t*)impl->dumper);
    pcap_dump_close((pcap_dumper_t*)impl->dumper);
    pcap_close(impl->handle);
}

inline pcpp::Layer* generate_encap(PcapWriter::PacketEncapsulation encap) {
    switch (encap) {
        case PcapWriter::PacketEncapsulation::ETHERNET:
            return new pcpp::EthLayer(pcpp::MacAddress("00:00:00:00:00:00"),
                                      pcpp::MacAddress("00:00:00:00:00:00"));
            break;
        case PcapWriter::PacketEncapsulation::SLL:
            return new pcpp::SllLayer(SLL_PACKET_SENT_TO_US, SLL_ARPHRD_ETHER);
            break;
        case PcapWriter::PacketEncapsulation::NULL_LOOPBACK:
            return new pcpp::NullLoopbackLayer(NULL_FAMILY_IPV4);
            break;
        default:
            throw "Encap not supported";
    }
}

std::vector<pcpp::Packet> generate_packets(
    const uint8_t* buf, size_t buf_size, uint16_t packet_id, size_t frag_size,
    PcapWriter::PacketEncapsulation encap, const std::string& src_ip,
    const std::string& dst_ip, int src_port, int dst_port) {
    (void)frag_size;
    /// @todo get packet fragmentation working
    /// Otherwise we have a hard time unit testing the packet frag recv
    std::vector<pcpp::Packet> result;

    pcpp::Packet newPacket(buf_size + IPHDR_LEN + UDPHDR_LEN);

    // Add encapsulation layer
    pcpp::Layer* encap_layer = generate_encap(encap);
    encap_layer->computeCalculateFields();
    newPacket.addLayer(encap_layer, true);

    // Add ip layer
    pcpp::IPv4Layer* IP = new pcpp::IPv4Layer(pcpp::IPv4Address(src_ip),
                                              pcpp::IPv4Address(dst_ip));
    IP->getIPv4Header()->timeToLive = 64;
    IP->getIPv4Header()->ipId = htons(packet_id);
    IP->computeCalculateFields();
    newPacket.addLayer(IP, true);

    // Add udp layer
    pcpp::UdpLayer* udp = new pcpp::UdpLayer(src_port, dst_port);
    udp->getUdpHeader()->portSrc = htons(src_port);
    udp->getUdpHeader()->portDst = htons(dst_port);

    udp->computeCalculateFields();
    newPacket.addLayer(udp, true);

    // Add payload layer
    pcpp::PayloadLayer* payload =
        new pcpp::PayloadLayer((uint8_t*)(buf), buf_size, false);
    newPacket.addLayer(payload, true);

    newPacket.computeCalculateFields();
    result.push_back(newPacket);

    return result;
}

void PcapWriter::write_packet(const uint8_t* buf, size_t buf_size,
                                const std::string& src_ip,
                                const std::string& dst_ip, uint16_t src_port,
                                uint16_t dst_port, packet_info::ts timestamp) {
    auto packets = generate_packets(buf, buf_size, id++, 1500, encap, src_ip,
                                    dst_ip, src_port, dst_port);
    for (auto it : packets) {
        auto raw = it.getRawPacket();
        pcap_pkthdr pcap_header;
        pcap_header.caplen = raw->getRawDataLen();
        pcap_header.len = pcap_header.caplen;
        pcap_header.ts = ts_to_timeval(timestamp);
        pcap_dump((u_char*)impl->dumper, &pcap_header, raw->getRawData());
    }
}

void PcapWriter::write_packet(const uint8_t* buf, size_t buf_size,
                                const packet_info& info) {
    write_packet(buf, buf_size, info.src_ip, info.dst_ip, info.src_port,
                 info.dst_port, info.timestamp);
}

}  // namespace sensor_utils
}  // namespace ouster

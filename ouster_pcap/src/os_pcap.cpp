// TODO:
// - check that the header casting is idiomatic libpcap
// - warn on dropped packets
//   + when pcap contains garbage
//   + when fragments missing, buffer reused before sending
// - split up reading / playback
// - improve error reporting
// - support ipv6
#include <stddef.h>
#include <tins/tins.h>

#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>

using namespace Tins;
#include "ouster/os_pcap.h"

using us = std::chrono::microseconds;

namespace ouster {
namespace sensor_utils {

std::ostream& operator<<(std::ostream& stream_in,
                         const stream_info& stream_data) {
    stream_in << "Replay Pcap Info:" << std::endl;
    stream_in << "  Packets Processed: " << stream_data.packets_processed
              << std::endl;
    stream_in << "  Packets Reassembled: " << stream_data.packets_reassembled
              << std::endl;
    stream_in << "  IPV6 Packets: " << stream_data.ipv6_packets << std::endl;
    stream_in << "  IPV4 Packets: " << stream_data.ipv4_packets << std::endl;
    stream_in << "  Non-UDP Packets: " << stream_data.non_udp_packets
              << std::endl;
    stream_in << "  Ports To Packet Sizes: " << std::endl;
    for (auto item : stream_data.port_to_packet_sizes) {
        stream_in << "    Port: " << item.first << std::endl;
        for (auto item2 : item.second) {
            stream_in << "      Data Size: " << item2.first
                      << " Count: " << item2.second << std::endl;
        }
    }
    stream_in << "  Port Count: " << std::endl;
    for (auto item : stream_data.port_to_packet_count) {
        stream_in << "    Port: " << item.first << " Count: " << item.second
                  << std::endl;
    }
    stream_in << "  Packet Size To Ports: " << std::endl;
    for (auto item : stream_data.packet_size_to_port) {
        stream_in << "    Packet Size: " << item.first << std::endl;
        for (auto item2 : item.second) {
            stream_in << "      Port: " << item2.first
                      << " Count: " << item2.second << std::endl;
        }
    }
    stream_in << "  Port Count: " << std::endl;
    for (auto item : stream_data.port_to_packet_count) {
        stream_in << "    Port: " << item.first << " Count: " << item.second
                  << std::endl;
    }
    return stream_in;
}

std::ostream& operator<<(std::ostream& stream_in, const packet_info& data) {
    stream_in << "Source IP: \"" << data.src_ip << "\" ";
    stream_in << "Source Port: " << data.src_port << std::endl;

    stream_in << "Destination IP: \"" << data.dst_ip << "\" ";
    stream_in << "Destination Port: " << data.dst_port << std::endl;

    stream_in << "Payload Size: " << data.payload_size << std::endl;
    stream_in << "Timestamp: " << data.timestamp.count() << std::endl;

    return stream_in;
}

int replay(playback_handle& handle, double rate) {
    packet_info info;
    bool started = false;
    us pcap_start_time;
    auto real_start_time = std::chrono::steady_clock::now();
    int packets_sent = 0;
    while (next_packet_info(handle, info)) {
        if (replay_packet(handle)) {
            if (!started) {
                started = true;
                pcap_start_time = us(info.timestamp);
            }

            if (rate > 0.0) {
                const auto delta =
                    (us(info.timestamp) - pcap_start_time) / rate;

                std::this_thread::sleep_until(real_start_time + delta);
            }
        }
    }

    return packets_sent;
}

// This function is here so that in the future the same code can be used for
// live packet captures as well
std::shared_ptr<stream_info> get_info(BaseSniffer& sniffer,
                                      size_t max_packets_to_process) {
    std::shared_ptr<stream_info> result = std::make_shared<stream_info>();

    IPv4Reassembler reassembler;
    for (auto& packet : sniffer) {
        if (result->packets_processed >= max_packets_to_process &&
            max_packets_to_process != 0)
            break;
        try {
            auto pdu = packet.pdu();
            if (pdu != NULL) {
                IP* ip = pdu->find_pdu<IP>();
                IPv6* ipv6 = pdu->find_pdu<IPv6>();
                if (ip != NULL || ipv6 != NULL) {
                    result->packets_processed++;
                    if (reassembler.process(*pdu) !=
                        IPv4Reassembler::FRAGMENTED) {
                        result->packets_reassembled++;
                        auto udp = pdu->find_pdu<UDP>();
                        auto raw = pdu->find_pdu<RawPDU>();
                        if (ip != NULL) result->ipv4_packets++;
                        if (ipv6 != NULL) result->ipv6_packets++;
                        if (udp != NULL) {
                            size_t port = udp->dport();
                            auto packet_size = raw->size();
                            result->port_to_packet_sizes[port][packet_size]++;
                            result->port_to_packet_count[port]++;
                            result->packet_size_to_port[packet_size][port]++;
                        } else {
                            result->non_udp_packets++;
                        }
                    }
                }
            }
        } catch (exception_base& e) {
            std::cout << "Issue reading packet: " << e.what() << std::endl;
            continue;
        }
    }

    return result;
}

std::shared_ptr<stream_info> replay_get_pcap_info(
    const std::string& file, size_t max_packets_to_process) {
    std::shared_ptr<stream_info> result;
    try {
        FileSniffer sniffer(file, "udp");
        result = get_info(sniffer, max_packets_to_process);
    } catch (exception_base& e) {
        std::cout << "FileSniffer Exception: " << e.what() << std::endl;
        throw;
    }

    return result;
}

std::shared_ptr<playback_handle> replay_initialize(
    const std::string& file_name, const std::string& src_ip,
    const std::string& dst_ip, std::unordered_map<int, int> port_map) {
    std::shared_ptr<playback_handle> result =
        std::make_shared<playback_handle>();

    result->file_name = file_name;

    result->src_ip = src_ip;
    result->dst_ip = dst_ip;

    result->port_map = port_map;
    result->replay_socket = socket(AF_INET, SOCK_DGRAM, 0);

    result->pcap_reader.reset(new FileSniffer(file_name));
    return result;
}

std::shared_ptr<playback_handle> replay_initialize(const std::string& file) {
    std::unordered_map<int, int> port_map;
    return replay_initialize(file, "", "", port_map);
}

void replay_uninitialize(playback_handle& handle) {
    ouster::impl::socket_close(handle.replay_socket);
    handle.pcap_reader.reset();
}

void replay_reset(playback_handle& handle) {
    handle.pcap_reader.reset(new FileSniffer(handle.file_name));
}

bool replay_packet(playback_handle& handle) {
    bool result = false;
    if (handle.have_new_packet) {
        struct sockaddr_in addr;

        result = true;
        auto pdu = handle.packet_cache.pdu();
        auto udp = pdu->find_pdu<UDP>();
        auto raw = pdu->find_pdu<RawPDU>();
        if (raw != NULL) {
            auto temp = (uint32_t*)&(raw->payload()[0]);
            auto size = raw->payload_size();
            int port = (handle.port_map.count(udp->dport()) == 0)
                           ? udp->dport()
                           : handle.port_map[udp->dport()];
            memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            addr.sin_port = htons(port);
            IP* ip = pdu->find_pdu<IP>();
            IPv6* ipv6 = pdu->find_pdu<IPv6>();
            if (ip || ipv6) {
                addr.sin_addr.s_addr = inet_addr(handle.dst_ip.c_str());
            }

            sendto(handle.replay_socket, (char*)temp, size, 0,
                   (const struct sockaddr*)&addr, sizeof(addr));
        }
    }

    return result;
}

bool next_packet_info(playback_handle& handle, packet_info& info) {
    bool result = false;

    bool reassm = false;
    while (!reassm) {
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
                        result = true;
                        UDP* udp = pdu->find_pdu<UDP>();
                        auto raw = pdu->find_pdu<RawPDU>();
                        if (ip) {
                            info.dst_ip = ip->dst_addr().to_string();
                            info.src_ip = ip->src_addr().to_string();
                        } else if (ipv6) {
                            info.dst_ip = ipv6->dst_addr().to_string();
                            info.src_ip = ipv6->src_addr().to_string();
                        } else {
                            throw "Error: No Ip Packet Found";
                        }
                        // find_pdu<UDP> will only return NULL when the ipv4
                        // reassembly succeeds on an ipv6 packet, leading to a
                        // malformed packet
                        if (udp != NULL) {
                            info.dst_port = udp->dport();
                            info.src_port = udp->sport();
                            info.payload_size = raw->payload_size();
                            info.timestamp =
                                static_cast<std::chrono::microseconds>(
                                    handle.packet_cache.timestamp());
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
    using namespace std::chrono;

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
         * The next block is due to the fact that the previous serialize does not treat
         * the udp packet as if it were decodable. Manually tell libtins to
         * go in and serialize the udp packet as well
         */
        if (pdu->inner_pdu()->inner_pdu()->inner_pdu() != NULL) {
            _ = pdu->inner_pdu()->inner_pdu()->inner_pdu()->serialize();
        }
        packet = Packet(*pdu, Timestamp(static_cast<std::chrono::microseconds>(
                                  microsecond_timestamp)));
        handle.pcap_file_writer->write(packet);
        delete pdu;
    }
}

}  // namespace sensor_utils
}  // namespace ouster

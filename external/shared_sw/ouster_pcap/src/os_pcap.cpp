// TODO:
// - check that the header casting is idiomatic libpcap
// - warn on dropped packets
//   + when pcap contains garbage
//   + when fragments missing, buffer reused before sending
// - split up reading / playback
// - improve error reporting
// - support ipv6

// clang format off
#include "ouster/compat.h"
// clang format on

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
    return stream_in;
}

std::ostream& operator<<(std::ostream& stream_in, const port_couple& data) {
    stream_in << "Lidar Port: " << data.lidar_port << std::endl;
    stream_in << "IMU Port: " << data.imu_port << std::endl;
    return stream_in;
}

bool reassemble_packet(PDU& pdu, IP* ip, IPv4Reassembler& reassembler) {
    bool result = false;
   
    if (ip != NULL) {
        if (reassembler.process(pdu) != IPv4Reassembler::FRAGMENTED) {
            result = true;
        }
    }
    return result;
}

bool send_packet(PDU& pdu, PacketSender& sender, IPv4Reassembler& reassembler,
                 const std::string& src_ip, const std::string& dest_ip,
                 port_couple src_ports = {0, 0},
                 port_couple dst_ports = {0, 0})
{
    bool sent = false;
    IP* ip = pdu.find_pdu<IP>();
    if (reassemble_packet(pdu, ip, reassembler)) {
        if (src_ip.length() > 0) ip->src_addr(src_ip);
        if (dest_ip.length() > 0) ip->dst_addr(dest_ip);

        UDP* udp = pdu.find_pdu<UDP>();
        if (udp) {
            if (udp->dport() == src_ports.lidar_port) {
                if (dst_ports.lidar_port != 0) {
                    udp->dport(dst_ports.lidar_port);
                }
            }
            else if (udp->dport() == src_ports.imu_port) {
                if (dst_ports.imu_port != 0) {
                    udp->dport(dst_ports.imu_port);
                }
            }
        }
        sender.send(*ip);
        sent = true;
    }

    return sent;
}

int replay(const std::string& file_name, const std::string& source,
           const std::string& dest, double rate) {
#ifdef _WIN32
    socket_init();
#endif
    std::unique_ptr<FileSniffer> sniffer;
    PacketSender sender;

    try {
        sniffer.reset(new FileSniffer(file_name, ""));
    } catch (exception_base& e) {
        std::cout << "FileSniffer Exception: " << e.what() << std::endl;

        return 1;
    }

    try {
        sender.open_l3_socket(Tins::PacketSender::SocketType::IP_UDP_SOCKET);
    } catch (exception_base& e) {
        std::cout << "Socket Open Issue, perhaps run as admin: " << e.what()
                  << std::endl;

        return 1;
    }

    IPv4Reassembler reassembler;
    bool started = false;
    us pcap_start_time;
    auto real_start_time = std::chrono::steady_clock::now();

    for (auto& packet : *sniffer) {
        const Timestamp& timestamp = packet.timestamp();
        PDU* pdu = NULL;
        try {
            pdu = packet.pdu();
            if (pdu != NULL) {
                bool sent = false;
                sent = send_packet(*pdu, sender, reassembler, source, dest);

                if (sent) {
                    if (!started) {
                        started = true;
                        pcap_start_time = us(timestamp);
                    }

                    if (rate > 0.0) {
                        const auto delta = (us(timestamp) - pcap_start_time) / rate;

                        std::this_thread::sleep_until(real_start_time + delta);
                    }
                }
            }
        } catch (exception_base& e) {
            std::cout << "Issue sending packet: " << e.what() << std::endl;
            continue;
        }
    }
#ifdef _WIN32
    socket_quit();
#endif
    return 0;
}

// This function is here so that in the future the same code can be used for
// live packet captures as well
std::shared_ptr<stream_info> get_info(BaseSniffer& sniffer,
                                      size_t max_packets_to_process) {
    std::shared_ptr<stream_info> result = std::make_shared<stream_info>();

    IPv4Reassembler reassembler;
    for (auto& packet : sniffer) {
        if (result->packets_processed >= max_packets_to_process
            && max_packets_to_process != 0) break;
        try {
            auto pdu = packet.pdu();
            if (pdu != NULL) {
                IP* ip = pdu->find_pdu<IP>();
                if (ip != NULL) {
                    result->packets_processed++;
                    if (reassembler.process(*pdu) != IPv4Reassembler::FRAGMENTED) {
                        result->packets_reassembled++;
                        auto udp = pdu->find_pdu<UDP>();
                        auto raw = pdu->find_pdu<RawPDU>();
                        size_t port = udp->dport();
                        auto packet_size = raw->size();
                        result->port_to_packet_sizes[port][packet_size]++;
                        result->port_to_packet_count[port]++;
                        result->packet_size_to_port[packet_size][port]++;
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

int guess_imu_port(const stream_info& stream_data) {
    int result = 0;
    const int imu_size = 48;
    if (stream_data.packet_size_to_port.find(imu_size) != stream_data.packet_size_to_port.end()) {
        auto correct_packet_size = stream_data.packet_size_to_port.at(imu_size);
        if (correct_packet_size.size() > 1) {
            throw "Error: Multiple possible imu packets found";
        } else {
            result = correct_packet_size.begin()->first;
        }
    }

    return result;
}

int guess_lidar_port(const stream_info& stream_data) {
    int result = 0;
    std::vector<int> lidar_sizes = {3392, 6464, 12608, 24896};

    int hit_count = 0;

    for (auto s : lidar_sizes) {
        if (stream_data.packet_size_to_port.find(s) != stream_data.packet_size_to_port.end()) {
            auto correct_packet_size = stream_data.packet_size_to_port.at(s);
            hit_count++;
            if (correct_packet_size.size() > 1) {
                throw "Error: Multiple possible lidar packets found";
            } else {
                result = correct_packet_size.begin()->first;
            }
        }
    }

    if (hit_count > 1) {
        throw "Error: Multiple possible lidar packets found";
    }

    return result;
}

port_couple guess_ports(const stream_info& stream_data) {
    port_couple result;

    result.imu_port = guess_imu_port(stream_data);
    result.lidar_port = guess_lidar_port(stream_data);

    return result;
}

std::shared_ptr<playback_handle> replay_initalize(
    const std::string& file_name, const std::string& src_ip,
    const std::string& dst_ip, const port_couple& src_ports,
    const port_couple& dst_ports) {
#ifdef _WIN32
    socket_init();
#endif
    std::shared_ptr<playback_handle> result =
        std::make_shared<playback_handle>();

    result->file_name = file_name;

    result->src_ip = src_ip;
    result->dst_ip = dst_ip;

    result->src_ports = src_ports;
    result->dst_ports = dst_ports;

    std::stringstream lidar_filter;

    if (src_ports.lidar_port != 0) {
        // Reference Here:
        // https://www.wains.be/pub/networking/tcpdump_advanced_filters.txt
        lidar_filter << "((ip[6:2] > 0) and (not ip[6] = 64))";
    }
    std::stringstream imu_filter;
    if (src_ports.imu_port != 0) {
        imu_filter << "(dst port " << src_ports.imu_port << ")";
    }
    result->pcap_file_lidar_reader.reset(
        new FileSniffer(file_name, lidar_filter.str()));
    result->pcap_file_imu_reader.reset(
        new FileSniffer(file_name, imu_filter.str()));

    return result;
}

std::shared_ptr<playback_handle> replay_initalize(const std::string& file,
                                                  const std::string& src_ip,
                                                  const std::string& dst_ip,
                                                  const port_couple& src_ports,
                                                  int dst_lidar_port,
                                                  int dst_imu_port) {
    port_couple dst = {dst_lidar_port, dst_imu_port};
    return replay_initalize(file, src_ip, dst_ip, src_ports, dst);
}

std::shared_ptr<playback_handle> replay_initalize(const std::string& file) {
    port_couple src = {0, 0};
    port_couple dst = {0, 0};
    return replay_initalize(file, "", "", src, dst);
}

void replay_uninitialize(playback_handle& handle) {
#ifdef _WIN32
    socket_quit();
#endif
}

bool replay_next_lidar_packet(playback_handle& handle) {
    bool result = false;
    bool sent = false;
    while (!sent) {
        Packet pkt = handle.pcap_file_lidar_reader->next_packet();
        if (pkt) {
            auto pdu = pkt.pdu();
            if (pdu) {
                sent = send_packet(*pdu, handle.sender, handle.reassembler,
                                   handle.src_ip, handle.dst_ip,
                                   handle.src_ports, handle.dst_ports);
                if (sent) result = true;
            }
        } else {
            sent = true;
        }
    }
    return result;
}

bool replay_next_imu_packet(playback_handle& handle) {
    bool result = false;
    bool sent = false;
    while (!sent) {
        Packet pkt = handle.pcap_file_imu_reader->next_packet();
        if (pkt) {
            auto pdu = pkt.pdu();
            if (pdu) {
                sent = send_packet(*pdu, handle.sender, handle.reassembler,
                                   handle.src_ip, handle.dst_ip,
                                   handle.src_ports, handle.dst_ports);
                if (sent) result = true;
            }
        } else {
            sent = true;
        }
    }
    return result;
}

bool get_next_lidar_data(playback_handle& handle, uint8_t* buf) {
    bool result = false;

    bool reassm = false;
    
    while (!reassm) {
        Packet pkt = handle.pcap_file_lidar_reader->next_packet();
        if (pkt) {
            auto pdu = pkt.pdu();
            if (pdu) {
                IP* ip = pdu->find_pdu<IP>();
                reassm = reassemble_packet(*pdu, ip, handle.reassembler);
                if (reassm) {
                    result = true;
                    auto raw = pdu->find_pdu<RawPDU>();
                    auto temp = (uint32_t*)&(raw->payload()[0]);
                    memcpy(buf, temp, raw->payload_size());
                }
            }
        } else {
            reassm = true;
        }
    }
    
    return result;
}

bool get_next_imu_data(playback_handle& handle, uint8_t* buf) {
    bool result = false;

    bool reassm = false;
    while (!reassm) {
        Packet pkt = handle.pcap_file_imu_reader->next_packet();
        if (pkt) {
            auto pdu = pkt.pdu();
            if (pdu) {
                IP* ip = pdu->find_pdu<IP>();
                reassm = reassemble_packet(*pdu, ip, handle.reassembler);
                if (reassm) {
                    result = true;
                    auto raw = pdu->find_pdu<RawPDU>();
                    auto temp = (uint32_t*)&(raw->payload()[0]);
                    memcpy((uint32_t*)buf, temp, raw->payload_size());
                }
            }
        } else {
            reassm = true;
        }
    }
    
    return result;
}

// Record functionality removed for a short amount of time
// until we switch it over to support libtins
}  // namespace sensor_utils
}  // namespace ouster

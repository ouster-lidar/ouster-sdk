/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/sensor_client.h"

#include "ouster/defaults.h"
#include "ouster/impl/logging.h"

using ouster::sensor::impl::Logger;
using ouster::sensor::util::SensorHttp;

namespace ouster {
namespace sensor {

// External imports of internal methods
SOCKET udp_data_socket(int port);
int32_t get_sock_port(SOCKET sock_fd);
Json::Value collect_metadata(SensorHttp& sensor_http, int timeout_sec);
SOCKET mtp_data_socket(int port, const std::vector<std::string>& udp_dest_hosts,
                       const std::string& mtp_dest_host = "");
bool set_config(SensorHttp& sensor_http, const sensor_config& config,
                uint8_t config_flags, int timeout_sec);

void add_socket_to_groups(SOCKET sock_fd,
                          const std::vector<std::string>& udp_dest_hosts,
                          const std::string& mtp_dest_host = "") {
    // join to multicast groups
    for (const auto& udp_dest_host : udp_dest_hosts) {
        ip_mreq mreq;
        mreq.imr_multiaddr.s_addr = inet_addr(udp_dest_host.c_str());
        if (!mtp_dest_host.empty()) {
            mreq.imr_interface.s_addr = inet_addr(mtp_dest_host.c_str());
        } else {
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        }

        if (setsockopt(sock_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq,
                       sizeof(mreq))) {
            logger().warn("mtp setsockopt(): {}", impl::socket_get_error());
        }
    }
}

Sensor::Sensor(const std::string& hostname, const sensor_config& config)
    : hostname_(hostname), config_(config) {}

sensor_info Sensor::fetch_metadata(int timeout) const {
    Json::FastWriter writer;
    return sensor_info(writer.write(collect_metadata(*http_client(), timeout)));
}

std::shared_ptr<ouster::sensor::util::SensorHttp> Sensor::http_client() const {
    // construct the client if we haven't already
    if (!http_client_) {
        http_client_ = ouster::sensor::util::SensorHttp::create(
            hostname_, SHORT_HTTP_REQUEST_TIMEOUT_SECONDS);
    }
    return http_client_;
}

SensorClient::~SensorClient() { close(); }

SensorClient::SensorClient(const std::vector<Sensor>& sensors, double timeout,
                           double buffer_time)
    : SensorClient(sensors, {}, timeout, buffer_time) {}

SensorClient::SensorClient(const std::vector<Sensor>& sensors,
                           const std::vector<sensor_info>& infos,
                           double config_timeout, double buffer_time) {
    // if we need an ephemeral port, create it now
    int ephemeral_port = -1;
    for (const auto& sensor : sensors) {
        const auto& config = sensor.desired_config();
        if (config.udp_port_lidar == 0 || config.udp_port_imu == 0) {
            SOCKET sock = udp_data_socket(0);
            if (sock == SOCKET_ERROR) {
                close();
                throw std::runtime_error("failed to obtain a UDP socket");
            }
            ephemeral_port = get_sock_port(sock);
            logger().info("Opening ephemeral port: {}", ephemeral_port);
            sockets_.push_back(sock);
            break;
        }
    }

    // if we have existing infos, do not reconfigure sensors and just use the
    // infos
    if (infos.size()) {
        sensor_info_ = infos;
        if (infos.size() != sensors.size()) {
            throw std::invalid_argument(
                "Incorrect number of sensor_infos provided to SensorClient for "
                "provided sensors.");
        }
        // update with ports from config if > 0
        for (size_t i = 0; i < sensors.size(); i++) {
            const auto& config = sensors[i].desired_config();
            if (config.udp_port_lidar == 0 || config.udp_port_imu == 0) {
                throw std::invalid_argument(
                    "Cannot specify ephemeral ports when providing metadata to "
                    "SensorClient for sensor '" +
                    sensors[i].hostname() + "'");
            }
            if ((config.udp_port_lidar &&
                 config.udp_port_lidar !=
                     sensor_info_[i].config.udp_port_lidar) ||
                (config.udp_port_imu &&
                 config.udp_port_imu != sensor_info_[i].config.udp_port_imu)) {
                throw std::invalid_argument(
                    "UDP ports must be null or match provided metadata if "
                    "metadata is provided for sensor '" +
                    sensors[i].hostname() + "'");
            }
        }
    } else {
        // configure sensors if necessary for the new ports
        std::map<int, sensor_info> fetched;
        sensor_config empty_config;
        for (size_t i = 0; i < sensors.size(); i++) {
            const auto& sensor = sensors[i];
            auto desired_config = sensors[i].desired_config();
            auto metadata = sensor.fetch_metadata(config_timeout);

            if (desired_config.udp_port_lidar == 0)
                desired_config.udp_port_lidar = ephemeral_port;
            else if (!desired_config.udp_port_lidar)
                desired_config.udp_port_lidar = metadata.config.udp_port_lidar;
            if (desired_config.udp_port_imu == 0)
                desired_config.udp_port_imu = ephemeral_port;
            else if (!desired_config.udp_port_imu)
                desired_config.udp_port_imu = metadata.config.udp_port_imu;

            // Don't do anything no configuration is requested
            if (desired_config == empty_config) {
                fetched[i] = metadata;
                continue;
            }

            set_config(*sensor.http_client(), desired_config, 0 /*flags*/,
                       config_timeout);
        }

        // if we need to, try and fetch missing configs
        // do this last so we dont wait N*reinit time to reconfigure lidars
        for (size_t i = 0; i < sensors.size(); i++) {
            // fetch any missing metadata
            auto res = fetched.find(i);
            if (res != fetched.end()) {
                sensor_info_.push_back(res->second);
            } else {
                sensor_info_.push_back(
                    sensors[i].fetch_metadata(config_timeout));
            }
        }
    }

    // build a list of any multicast addresses we need to listen to
    std::vector<std::string> multicast_addrs;
    for (const auto& sensor : sensor_info_) {
        auto udp_dest = sensor.config.udp_dest.value_or("");
        if (ouster::sensor::in_multicast(udp_dest)) {
            multicast_addrs.push_back(udp_dest);
            logger().info("Adding sockets to multicast group {}",
                          udp_dest.c_str());
        }
    }

    for (const auto& sensor : sensors) {
        // figure out addresses for sensors
        struct addrinfo hints;
        struct addrinfo* result;

        // Set up the hints structure to specify the desired options (IPv4, TCP)
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;      // IPv4, todo dont care
        hints.ai_socktype = SOCK_STREAM;  // TCP socket

        // Use getaddrinfo to resolve the address.
        if (getaddrinfo(sensor.hostname().c_str(), NULL, &hints, &result) !=
            0) {
            throw std::runtime_error("Could not resolve address '" +
                                     sensor.hostname() + "' for sensor.");
        }

        // Find addresses
        bool found = false;
        Addr addr;
        addr.ipv4 = 0;
        memset(addr.ipv6, 0, 16);
        for (auto rp = result; rp != NULL; rp = rp->ai_next) {
            if (rp->ai_family == AF_INET6) {
                struct sockaddr_in6* ipv6 = (struct sockaddr_in6*)rp->ai_addr;
                addr.ipv4 = 0;  // none
                memcpy(addr.ipv6, ipv6->sin6_addr.s6_addr, 16);
                found = true;
            } else if (rp->ai_family == AF_INET) {
                struct sockaddr_in* ipv4 = (struct sockaddr_in*)rp->ai_addr;

                // ok, now make the ipv4 and ipv6 mapped version (in net
                // ordering)
                uint32_t hipv4 = ntohl(ipv4->sin_addr.s_addr);
                addr.ipv4 = ipv4->sin_addr.s_addr;
                memset(addr.ipv6_4, 0, 16);
                addr.ipv6_4[15] = hipv4 & 0xFF;
                addr.ipv6_4[14] = (hipv4 >> 8) & 0xFF;
                addr.ipv6_4[13] = (hipv4 >> 16) & 0xFF;
                addr.ipv6_4[12] = (hipv4 >> 24) & 0xFF;
                addr.ipv6_4[11] = 0xFF;
                addr.ipv6_4[10] = 0xFF;
                found = true;
            }
        }
        freeaddrinfo(result);
        if (!found) {
            throw std::runtime_error("Could not find address for sensor '" +
                                     sensor.hostname() + "'");
        }
        addresses_.push_back(addr);
    }

    // build a list of ports to open and form mappings from ip/port to lidar
    std::map<int, bool> ports;
    for (const auto& info : sensor_info_) {
        ports[info.config.udp_port_lidar.value()] = true;
        ports[info.config.udp_port_imu.value()] = true;
        formats_.push_back(packet_format(info));
    }

    // now open sockets
    for (auto port : ports) {
        if (port.first == ephemeral_port) {
            continue;  // we already added it
        }
        // just add every socket to the multicast group to simplify things
        SOCKET sock = mtp_data_socket(port.first, multicast_addrs);
        if (sock == SOCKET_ERROR) {
            close();
            throw std::runtime_error("failed to obtain a UDP socket");
        }
        sockets_.push_back(sock);
        logger().info("Opening port: {}", port.first);
    }

    // if we have an ephemeral socket, add it to multicast groups
    if (ephemeral_port > 0) {
        add_socket_to_groups(sockets_[0], multicast_addrs);
    }

    // finally create our buffer thread if requested
    if (buffer_time > 0) {
        start_buffer_thread(buffer_time);
    }
}

void SensorClient::start_buffer_thread(double buffer_time) {
    do_buffer_ = true;
    buffer_thread_ = std::thread([this, buffer_time]() {
        std::vector<uint8_t> data;
        const uint64_t buffer_ns = buffer_time * 1000000000.0;
        while (do_buffer_) {
            uint64_t ts;
            ClientEvent ev = get_packet_internal(data, ts, 0.01);
            if (ev.type == ClientEvent::PollTimeout) {
                continue;
            }
            // Enqueue received packets
            {
                std::unique_lock<std::mutex> lock(buffer_mutex_);
                BufferEvent be;
                be.event = ev;
                be.timestamp = ts;
                std::swap(be.data, data);
                buffer_.push_back(std::move(be));

                // Discard old buffered packets if our consumer couldn't keep up
                uint64_t expiry_time = ts - buffer_ns;
                while (buffer_.size() &&
                       (buffer_.front().timestamp < expiry_time)) {
                    buffer_.pop_front();
                    dropped_packets_++;
                }
            }
            buffer_cv_.notify_one();
        }
    });
}

void SensorClient::flush() {
    if (!do_buffer_) {
        return;
    }
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
}

void SensorClient::close() {
    // signal our thread to exit and join if joinable
    if (buffer_thread_.joinable()) {
        do_buffer_ = false;
        buffer_thread_.join();
    }
    // close all our sockets
    for (auto socket : sockets_) {
        impl::socket_close(socket);
    }
    sockets_.clear();
}

size_t SensorClient::buffer_size() {
    if (do_buffer_) {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        return buffer_.size();
    }
    return 0;
}

ClientEvent SensorClient::get_packet_internal(std::vector<uint8_t>& data,
                                              uint64_t& ts,
                                              double timeout_sec) {
    if (sockets_.size() == 0) {
        auto now = std::chrono::system_clock::now();
        auto now_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          now.time_since_epoch())
                          .count();
        ts = now_ts;
        return {-1, ClientEvent::Exit};  // someone called us while shut down
    }
    // setup poll
    SOCKET max_fd = 0;
    fd_set fds;
    FD_ZERO(&fds);
    for (auto sock : sockets_) {
        FD_SET(sock, &fds);
        max_fd = std::max(max_fd, sock);
    }

    // poll up to timeout for a new packet
    timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = fmod(timeout_sec, 1.0) * 1000000.0;

    int ret =
        select(max_fd + 1, &fds, NULL, NULL, timeout_sec < 0 ? NULL : &tv);
    auto now = std::chrono::system_clock::now();
    ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
             now.time_since_epoch())
             .count();
    if (ret == 0) {
        return {-1, ClientEvent::PollTimeout};
    } else if (ret < 0) {
        return {-1, ClientEvent::Error};
    }
    struct sockaddr_storage from_addr;
    socklen_t addr_len = sizeof(from_addr);

    char buffer[65535];  // this isnt great, but otherwise have to reserve this
                         // much in every packet
    for (auto sock : sockets_) {
        if (!FD_ISSET(sock, &fds)) continue;

        auto size = recvfrom(sock, buffer, 65535, 0,
                             (struct sockaddr*)&from_addr, &addr_len);
        if (size <= 0) continue;  // this is unexpected

        sockaddr_in6* addr6 = (sockaddr_in6*)&from_addr;
        sockaddr_in* addr4 = (sockaddr_in*)&from_addr;
        int source = -1;
        for (size_t i = 0; i < addresses_.size(); i++) {
            if (from_addr.ss_family == AF_INET6 &&
                memcmp(addr6->sin6_addr.s6_addr, addresses_[i].ipv6, 16) == 0) {
                source = i;
                break;
            }
            if (from_addr.ss_family == AF_INET6 &&
                memcmp(addr6->sin6_addr.s6_addr, addresses_[i].ipv6_4, 16) ==
                    0) {
                source = i;
                break;
            } else if (from_addr.ss_family == AF_INET &&
                       addr4->sin_addr.s_addr == addresses_[i].ipv4) {
                source = i;
                break;
            }
        }
        if (source == -1) {
            // if we got a random packet, just say we got nothing
            return {-1, ClientEvent::PollTimeout};
        }

        // detect packet type by size
        const int imu_size = formats_[source].imu_packet_size;
        if (size > imu_size) {
            data.resize(size);
            memcpy(data.data(), buffer, size);
            return {(int)source, ClientEvent::LidarPacket};
        } else if (size == imu_size) {
            data.resize(size);
            memcpy(data.data(), buffer, size);
            return {(int)source, ClientEvent::ImuPacket};
        } else {
            // The sensor returned an invalid packet size, say we got nothing
            return {-1, ClientEvent::PollTimeout};
        }
    }
    return {-1, ClientEvent::Error};  // this shouldnt happen
}

ClientEvent SensorClient::get_packet(LidarPacket& lp, ImuPacket& ip,
                                     double timeout_sec) {
    // poll on all our sockets
    ClientEvent ev;
    uint64_t ts;
    std::vector<uint8_t> data;
    if (do_buffer_) {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        // if the buffer if empty, wait for a new event
        if (!buffer_.size()) {
            auto duration = std::chrono::duration<double>(timeout_sec);
            auto res = buffer_cv_.wait_for(lock, duration);
            // check for timeout or for spurious wakeup of "wait_for"
            // by checking whether the buffer is empty
            if (res == std::cv_status::timeout || buffer_.empty()) {
                return {-1, ClientEvent::PollTimeout};
            }
        }
        // dequeue
        auto& buf = buffer_.front();
        ev = buf.event;
        ts = buf.timestamp;
        std::swap(data, buf.data);
        buffer_.pop_front();
        lock.unlock();  // unlock asap
    } else {
        ev = get_packet_internal(data, ts, timeout_sec);
    }

    if (ev.type == ClientEvent::LidarPacket) {
        lp.host_timestamp = ts;
        std::swap(data, lp.buf);
    } else if (ev.type == ClientEvent::ImuPacket) {
        ip.host_timestamp = ts;
        std::swap(data, ip.buf);
    }
    return ev;  // todo finish
}

uint64_t SensorClient::dropped_packets() {
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    return dropped_packets_;
}

}  // namespace sensor
}  // namespace ouster

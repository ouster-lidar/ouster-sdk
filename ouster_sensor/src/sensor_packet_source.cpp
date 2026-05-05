/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/sensor_packet_source.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <jsoncons/json.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <nonstd/optional.hpp>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ouster/defaults.h"
#include "ouster/impl/logging.h"
#include "ouster/metadata.h"
#include "ouster/sensor_scan_source.h"

using ouster::sdk::core::impl::Logger;
using ouster::sdk::sensor::SensorHttp;
using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace core {
bool parse_and_validate_metadata(const jsoncons::json& json_data,
                                 SensorInfo& sensor_info,
                                 ValidatorIssues& issues);
}
namespace sensor {

// External imports of internal methods
int32_t get_sock_port(SOCKET sock_fd);
jsoncons::json collect_metadata(SensorHttp& sensor_http, int timeout_sec);

SOCKET mtp_data_socket(int port, const std::set<std::string>& udp_dest_hosts,
                       const std::string& mtp_dest_host = "",
                       bool reuse_ports = true);
bool set_config(SensorHttp& sensor_http, const SensorConfig& config,
                uint8_t config_flags, int timeout_sec);

void add_socket_to_groups(SOCKET sock_fd,
                          const std::set<std::string>& udp_dest_hosts,
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

        if (setsockopt(sock_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                       reinterpret_cast<char*>(&mreq), sizeof(mreq)) != 0) {
            logger().warn("mtp setsockopt(): {}", impl::socket_get_error());
        }
    }
}

Sensor::Sensor(const std::string& hostname, const SensorConfig& config)
    : hostname_(hostname), config_(config) {}

SensorInfo Sensor::fetch_metadata(int timeout) const {
    // TODO[tws] dedupe this method's logic with legacy iface
    auto http_client_ptr = http_client();
    auto data = collect_metadata(*http_client_ptr, timeout);

    SensorInfo result;
    ValidatorIssues issues;
    core::parse_and_validate_metadata(data, result, issues);
    if (!issues.critical.empty()) {
        throw std::runtime_error(to_string(issues.critical));
    }

    if (http_client_ptr->firmware_version() >=
        ouster::sdk::core::Version{3, 2, 0}) {
        try {
            auto zone_set_config_bytes =
                http_client_ptr->get_zone_monitor_config_zip();

            // Load the zone set config from the zip bytes
            result.zone_set =
                std::move(ZoneSet(std::move(zone_set_config_bytes)));
        } catch (const std::runtime_error& e) {
            auto exception_msg = std::string(e.what());
            if (exception_msg.find("[404]") != std::string::npos) {
                logger().info("No zone monitor config found.");
            } else {
                throw std::runtime_error("Error parsing zone monitor config: " +
                                         exception_msg);
            }
        }
    }

    return result;
}

std::shared_ptr<SensorHttp> Sensor::http_client() const {
    // construct the client if we haven't already
    if (!http_client_) {
        http_client_ =
            SensorHttp::create(hostname_, SHORT_HTTP_REQUEST_TIMEOUT_SECONDS);
    }
    return http_client_;
}

SensorPacketSource::~SensorPacketSource() { close(); }

static std::vector<Sensor> calculate_sensors(
    const std::vector<std::string>& sources,
    SensorPacketSourceOptions& options) {
    std::vector<Sensor> sensors;

    // just to mark these as retrieved
    options.imu_port.retrieve();
    options.lidar_port.retrieve();
    bool do_not_reinitialize = options.do_not_reinitialize.retrieve();
    bool no_auto_udp_dest = options.no_auto_udp_dest.retrieve();
    auto input_configs = options.sensor_config.retrieve();

    int i = -1;
    for (const auto& hostname : sources) {
        i++;
        if (!options.sensor_info.retrieve().empty()) {
            sensors.emplace_back(hostname, SensorConfig());
            continue;
        }
        // todo make it so I dont have to create this twice per sensor
        // could theoretically pass it into the Sensor constructor
        auto http_client =
            SensorHttp::create(hostname, SHORT_HTTP_REQUEST_TIMEOUT_SECONDS);

        auto orig_config = SensorConfig(http_client->get_config_params(true));

        // calculate the correct ports for this sensor
        nonstd::optional<uint16_t> desired_lidar;
        nonstd::optional<uint16_t> desired_imu;
        if (i < static_cast<int>(input_configs.size())) {
            desired_lidar = input_configs[i].udp_port_lidar;
            desired_imu = input_configs[i].udp_port_imu;
        }
        if (options.imu_port.was_set()) {
            desired_imu = options.imu_port.retrieve();
        }
        if (options.lidar_port.was_set()) {
            desired_lidar = options.lidar_port.retrieve();
        }

        SensorConfig config;
        if (!do_not_reinitialize) {
            if (i < static_cast<int>(input_configs.size())) {
                config = input_configs[i];
            }
            if (!no_auto_udp_dest) {
                if (!orig_config.udp_dest ||
                    !in_multicast(orig_config.udp_dest.value())) {
                    try {
                        std::string dst = http_client->auto_detected_udp_dest(
                            SHORT_HTTP_REQUEST_TIMEOUT_SECONDS,
                            orig_config.udp_dest);
                        if (dst != orig_config.udp_dest) {
                            printf(
                                "Will change sensor's udp_dest from '%s' to "
                                "automatically detected '%s'\n",
                                orig_config.udp_dest.value_or("").c_str(),
                                dst.c_str());
                            config.udp_dest = dst;
                        }
                        if (!http_client->is_vlp() &&
                            orig_config.udp_dest_zm.has_value() &&
                            orig_config.udp_dest_zm != dst) {
                            printf(
                                "Will change sensor's udp_dest_zm from '%s' to "
                                "automatically detected '%s'\n",
                                orig_config.udp_dest_zm.value_or("").c_str(),
                                dst.c_str());
                            config.udp_dest_zm = dst;
                        }
                    } catch (std::runtime_error& ex) {
                        if (std::string(ex.what()) !=
                            "retrieving auto detected udp destination not "
                            "supported on this firmware version") {
                            throw ex;
                        }
                        // fallback if not supported by this fw
                        printf(
                            "Will change sensor's udp_dest from '%s' to "
                            "automatically detected UDP DEST\n",
                            orig_config.udp_dest.value_or("").c_str());
                        config.udp_dest = "@auto";
                    }
                }
            } else {
                printf(
                    "WARNING: You have opted not to allow us to reset your "
                    "auto UDP dest "
                    "by using either -x or -y. If you get a Timeout error, "
                    "drop -x and -y "
                    "from  your arguments to allow automatic udp_dest "
                    "setting.\n");
            }
            config.udp_port_imu = desired_imu;
            config.udp_port_lidar = desired_lidar;
            config.operating_mode = OperatingMode::NORMAL;

            if (orig_config.operating_mode != config.operating_mode) {
                printf("Will change sensor's operating mode from %s to %s\n",
                       to_string(orig_config.operating_mode.value()).c_str(),
                       to_string(config.operating_mode.value()).c_str());
            }

            if (config.udp_port_imu &&
                orig_config.udp_port_imu != config.udp_port_imu) {
                std::string port_name =
                    config.udp_port_imu.value() == 0
                        ? "ephemeral"
                        : std::to_string(config.udp_port_imu.value());
                printf("Will change sensor's IMU port from %i to %s\n",
                       orig_config.udp_port_imu.value(), port_name.c_str());
            }

            if (config.udp_port_lidar &&
                orig_config.udp_port_lidar != config.udp_port_lidar) {
                std::string port_name =
                    config.udp_port_lidar.value() == 0
                        ? "ephemeral"
                        : std::to_string(config.udp_port_lidar.value());
                printf("Will change sensor's lidar port from %i to %s\n",
                       orig_config.udp_port_lidar.value(), port_name.c_str());
            }
        } else {
            try {
                std::string dst = http_client->auto_detected_udp_dest(
                    SHORT_HTTP_REQUEST_TIMEOUT_SECONDS, orig_config.udp_dest);
                if (dst != orig_config.udp_dest) {
                    printf(
                        "WARNING: Your sensor's udp destination '%s' does "
                        "not match the detected udp destination '%s'. "
                        "If you get a Timeout error, drop -x and -y from your "
                        "arguments to allow automatic udp_dest setting.\n",
                        orig_config.udp_dest.value_or("").c_str(), dst.c_str());
                }
            } catch (std::runtime_error& ex) {
                if (std::string(ex.what()) !=
                    "retrieving auto detected udp destination not "
                    "supported on this firmware version") {
                    throw ex;
                }

                printf(
                    "WARNING: You have opted not to allow us to reset your "
                    "auto UDP dest "
                    "by using either -x or -y. If you get a Timeout error, "
                    "drop -x and -y "
                    "from  your arguments to allow automatic udp_dest "
                    "setting.\n");
            }

            if (orig_config.operating_mode == OperatingMode::STANDBY) {
                throw std::runtime_error(
                    "Your sensor is in STANDBY mode but you have disallowed "
                    "reinitialization.");
            }

            if (desired_lidar &&
                orig_config.udp_port_lidar != desired_lidar.value()) {
                throw std::runtime_error(
                    "Sensor's lidar port " +
                    std::to_string(orig_config.udp_port_lidar.value()) +
                    " does "
                    "not match provided lidar port but you have disallowed "
                    "reinitialization. Drop -x to allow reinitialization or "
                    "change your specified lidar_port to " +
                    std::to_string(orig_config.udp_port_lidar.value()) + ".");
            }

            if (desired_imu &&
                orig_config.udp_port_imu != desired_imu.value()) {
                throw std::runtime_error(
                    "Sensor's IMU port " +
                    std::to_string(orig_config.udp_port_imu.value()) +
                    " does "
                    "not match provided IMU port but you have disallowed "
                    "reinitialization. Drop -x to allow reinitialization or "
                    "change your specified imu_port to " +
                    std::to_string(orig_config.udp_port_imu.value()) + ".");
            }
        }

        sensors.emplace_back(hostname, config);
    }
    return sensors;
}
SensorPacketSource::SensorPacketSource(
    const std::string& source,
    const std::function<void(SensorPacketSourceOptions&)>& options)
    : SensorPacketSource(source,
                         ouster::sdk::impl::get_packet_options(options)) {}

SensorPacketSource::SensorPacketSource(
    const std::vector<std::string>& source,
    const std::function<void(SensorPacketSourceOptions&)>& options)
    : SensorPacketSource(source,
                         ouster::sdk::impl::get_packet_options(options)) {}

SensorPacketSource::SensorPacketSource(const std::string& source,
                                       SensorPacketSourceOptions options)
    : SensorPacketSource(std::vector<std::string>({source}), options) {}

SensorPacketSource::SensorPacketSource(const std::vector<std::string>& sources,
                                       SensorPacketSourceOptions options)
    : SensorPacketSource(
          calculate_sensors(sources, options), options.sensor_info.retrieve(),
          options.config_timeout.retrieve(), options.buffer_time_sec.retrieve(),
          options.reuse_ports.retrieve()) {
    iterator_timeout_ = options.timeout.retrieve();
    populate_extrinsics(options.extrinsics_file.retrieve(),
                        options.extrinsics.retrieve(), sensor_info_);
}

SensorPacketSource::SensorPacketSource(const std::vector<Sensor>& sensors,
                                       double timeout, double buffer_time)
    : SensorPacketSource(sensors, {}, timeout, buffer_time) {}

SensorPacketSource::SensorPacketSource(const std::vector<Sensor>& sensors,
                                       const std::vector<SensorInfo>& infos,
                                       double config_timeout,
                                       double buffer_time, bool reuse_ports) {
    // if we need an ephemeral port, create it now
    int ephemeral_port = -1;
    for (const auto& sensor : sensors) {
        const auto& config = sensor.desired_config();

        bool need_ephemeral_port = config.udp_port_lidar == 0 ||
                                   config.udp_port_imu == 0 ||
                                   config.udp_port_zm == 0;

        if (need_ephemeral_port) {
            // todo this probably needs to support multicast
            SOCKET sock = mtp_data_socket(0, {}, "", reuse_ports);
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
    if (!infos.empty()) {
        for (const auto& info : infos) {
            sensor_info_.emplace_back(new SensorInfo(info));
        }
        if (infos.size() != sensors.size()) {
            throw std::invalid_argument(
                "Incorrect number of SensorInfos provided to SensorClient for "
                "provided sensors.");
        }
        // update with ports from config if > 0
        for (size_t i = 0; i < sensors.size(); i++) {
            const auto& config = sensors[i].desired_config();
            bool need_ephemeral_port = config.udp_port_lidar == 0 ||
                                       config.udp_port_imu == 0 ||
                                       config.udp_port_zm == 0;
            if (need_ephemeral_port) {
                throw std::invalid_argument(
                    "Cannot specify ephemeral ports when providing metadata to "
                    "SensorClient for sensor '" +
                    sensors[i].hostname() + "'");
            }
            if ((config.udp_port_lidar &&
                 config.udp_port_lidar !=
                     sensor_info_[i]->config.udp_port_lidar) ||
                (config.udp_port_imu &&
                 config.udp_port_imu != sensor_info_[i]->config.udp_port_imu)) {
                throw std::invalid_argument(
                    "UDP ports must be null or match provided metadata if "
                    "metadata is provided for sensor '" +
                    sensors[i].hostname() + "'");
            }
        }
    } else {
        // configure sensors if necessary for the new ports
        SensorConfig empty_config;
        for (size_t i = 0; i < sensors.size(); i++) {
            const auto& sensor = sensors[i];
            auto desired_config = sensors[i].desired_config();

            if (desired_config.udp_port_lidar == 0) {
                desired_config.udp_port_lidar = ephemeral_port;
            }
            if (desired_config.udp_port_imu == 0) {
                desired_config.udp_port_imu = ephemeral_port;
            }
            if (desired_config.udp_port_zm == 0) {
                desired_config.udp_port_zm = ephemeral_port;
            }

            set_config(*sensor.http_client(), desired_config, 0 /*flags*/,
                       config_timeout);
        }

        // fetch metadata
        // do this last so we dont wait N*reinit time to reconfigure lidars
        for (size_t i = 0; i < sensors.size(); i++) {
            sensor_info_.emplace_back(
                new SensorInfo(sensors[i].fetch_metadata(config_timeout)));
        }
    }

    // build a list of any multicast addresses we need to listen to
    std::set<std::string> multicast_addrs;
    auto check_multicast_address = [&multicast_addrs](std::string addr) {
        if (in_multicast(addr)) {
            multicast_addrs.insert(addr);
        }
    };
    for (const auto& sensor : sensor_info_) {
        auto udp_dest = sensor->config.udp_dest.value_or("");
        check_multicast_address(udp_dest);
        if (sensor->config.udp_dest_zm) {
            check_multicast_address(*sensor->config.udp_dest_zm);
        }
    }
    for (const auto& addr : multicast_addrs) {
        logger().info("Adding sockets to multicast group {}", addr.c_str());
    }

    int sensor_index = 0;
    for (const auto& sensor : sensors) {
        // figure out addresses for sensors
        struct addrinfo hints;
        struct addrinfo* result;

        // Set up the hints structure to specify the desired options (IPv4, TCP)
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;      // IPv4, todo dont care
        hints.ai_socktype = SOCK_STREAM;  // TCP socket

        // Use getaddrinfo to resolve the address.
        if (getaddrinfo(sensor.hostname().c_str(), nullptr, &hints, &result) !=
            0) {
            throw std::runtime_error("Could not resolve address '" +
                                     sensor.hostname() + "' for sensor.");
        }

        // Find addresses
        bool found = false;
        for (auto rp = result; rp != nullptr; rp = rp->ai_next) {
            if (rp->ai_family == AF_INET6) {
                struct sockaddr_in6* ipv6 =
                    reinterpret_cast<struct sockaddr_in6*>(rp->ai_addr);
                Addr6 addr;
                addr.sensor_index = sensor_index;
                memcpy(addr.address, ipv6->sin6_addr.s6_addr, 16);
                found = true;
                addresses6_.push_back(addr);
            } else if (rp->ai_family == AF_INET) {
                struct sockaddr_in* ipv4 =
                    reinterpret_cast<struct sockaddr_in*>(rp->ai_addr);

                // ok, now make the ipv4 and ipv6 mapped version (in net
                // ordering)
                uint32_t hipv4 = ntohl(ipv4->sin_addr.s_addr);
                Addr4 addr4;
                Addr6 addr6;
                addr4.address = ipv4->sin_addr.s_addr;
                addr4.sensor_index = addr6.sensor_index = sensor_index;
                memset(addr6.address, 0, 16);
                addr6.address[15] = hipv4 & 0xFF;
                addr6.address[14] = (hipv4 >> 8) & 0xFF;
                addr6.address[13] = (hipv4 >> 16) & 0xFF;
                addr6.address[12] = (hipv4 >> 24) & 0xFF;
                addr6.address[11] = 0xFF;
                addr6.address[10] = 0xFF;
                found = true;
                addresses4_.push_back(addr4);
                addresses6_.push_back(addr6);
            }
        }
        sensor_index++;
        freeaddrinfo(result);
        if (!found) {
            throw std::runtime_error("Could not find address for sensor '" +
                                     sensor.hostname() + "'");
        }
    }

    // build a list of ports to open and form mappings from ip/port to lidar
    std::map<int, bool> ports;
    for (const auto& info : sensor_info_) {
        ports[info->config.udp_port_lidar.value()] = true;
        ports[info->config.udp_port_imu.value()] = true;
        if (info->config.udp_port_zm) {
            ports[info->config.udp_port_zm.value()] = true;
        }
        formats_.push_back(std::make_shared<PacketFormat>(*info));
    }

    // now open sockets
    for (auto port : ports) {
        if (port.first == ephemeral_port) {
            continue;  // we already added it
        }
        if (port.first == 0) {
            continue;  // not a valid port (this stream was disabled)
        }
        // just add every socket to the multicast group to simplify things
        SOCKET sock =
            mtp_data_socket(port.first, multicast_addrs, "", reuse_ports);
        if (sock == SOCKET_ERROR) {
            close();
            throw std::runtime_error("failed to obtain a UDP socket on port " +
                                     std::to_string(port.first));
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

void SensorPacketSource::start_buffer_thread(double buffer_time) {
    do_buffer_ = true;
    buffer_thread_ = std::thread([this, buffer_time]() {
        std::vector<uint8_t> data;
        const uint64_t buffer_ns = buffer_time * 1000000000.0;
        while (do_buffer_) {
            uint64_t timestamp;
            InternalEvent internal_event =
                get_packet_internal(data, timestamp, 0.01);
            if (internal_event.event_type == ClientEvent::POLL_TIMEOUT) {
                continue;
            }
            // Enqueue received packets
            {
                std::unique_lock<std::mutex> lock(buffer_mutex_);
                BufferEvent buffer_event;
                buffer_event.event = internal_event;
                buffer_event.timestamp = timestamp;
                std::swap(buffer_event.data, data);
                buffer_event.data.shrink_to_fit();  // can save a lot of memory,
                                                    // though does force a copy
                buffer_.push_back(std::move(buffer_event));

                // Discard old buffered packets if our consumer couldn't keep up
                uint64_t expiry_time = timestamp - buffer_ns;
                while (!buffer_.empty() &&
                       (buffer_.front().timestamp < expiry_time)) {
                    buffer_.pop_front();
                    dropped_packets_++;
                }
            }
            buffer_cv_.notify_one();
        }
    });
}

void SensorPacketSource::flush() {
    if (!do_buffer_) {
        return;
    }
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
}

void SensorPacketSource::close() {
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

size_t SensorPacketSource::buffer_size() {
    if (do_buffer_) {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        return buffer_.size();
    }
    return 0;
}

static PacketType get_packet_type(const PacketFormat& format, uint8_t* buf,
                                  size_t size) {
    // we could consider extra checks for ports here
    if (format.udp_profile_lidar == UDPProfileLidar::LEGACY &&
        size == format.lidar_packet_size) {
        return PacketType::Lidar;
    } else if (format.udp_profile_imu == UDPProfileIMU::LEGACY &&
               size == format.imu_packet_size) {
        return PacketType::Imu;
    }

    uint16_t type = format.packet_type(buf);
    switch (type) {
        case 0x01:
            return PacketType::Lidar;
        case 0x02:
            return PacketType::Imu;
        case 0x03:
            return PacketType::Zone;
        default:
            return PacketType::Unknown;
    }
}

SensorPacketSource::InternalEvent SensorPacketSource::get_packet_internal(
    std::vector<uint8_t>& data, uint64_t& timestamp, double timeout_sec) {
    if (sockets_.empty()) {
        auto now = std::chrono::system_clock::now();
        auto now_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          now.time_since_epoch())
                          .count();
        timestamp = now_ts;
        return {-1, PacketType::Unknown,
                ClientEvent::EXIT};  // someone called us while shut down
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
    timeval time_value;
    time_value.tv_sec = timeout_sec;
    time_value.tv_usec = fmod(timeout_sec, 1.0) * 1000000.0;

    int ret = select(max_fd + 1, &fds, nullptr, nullptr,
                     timeout_sec < 0 ? nullptr : &time_value);
    auto now = std::chrono::system_clock::now();
    timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    now.time_since_epoch())
                    .count();
    if (ret == 0) {
        return {-1, PacketType::Unknown, ClientEvent::POLL_TIMEOUT};
    } else if (ret < 0) {
        return {-1, PacketType::Unknown, ClientEvent::ERR};
    }
    struct sockaddr_storage from_addr;
    socklen_t addr_len = sizeof(from_addr);

    data.resize(65535);  // need enough room for maximum possible packet size
    for (auto sock : sockets_) {
        if (!FD_ISSET(sock, &fds)) {
            continue;
        }

        auto size =
            recvfrom(sock, reinterpret_cast<char*>(data.data()), 65535, 0,
                     reinterpret_cast<struct sockaddr*>(&from_addr), &addr_len);
        if (size <= 0) {
            continue;  // this is unexpected
        }

        sockaddr_in6* addr6 = reinterpret_cast<sockaddr_in6*>(&from_addr);
        sockaddr_in* addr4 = reinterpret_cast<sockaddr_in*>(&from_addr);
        int source = -1;
        if (from_addr.ss_family == AF_INET6) {
            for (const auto& addr : addresses6_) {
                if (memcmp(addr6->sin6_addr.s6_addr, addr.address, 16) == 0) {
                    source = addr.sensor_index;
                    break;
                }
            }
        } else {
            for (const auto& addr : addresses4_) {
                if (addr4->sin_addr.s_addr == addr.address) {
                    source = addr.sensor_index;
                    break;
                }
            }
        }
        if (source == -1) {
            // if we got a random packet, just say we got nothing
            return {-1, PacketType::Unknown, ClientEvent::POLL_TIMEOUT};
        }

        PacketType packet_type =
            get_packet_type(*formats_[source], data.data(), size);

        if (packet_type == PacketType::Unknown) {
            // The sensor returned an invalid packet size, say we got nothing
            return {-1, PacketType::Unknown, ClientEvent::POLL_TIMEOUT};
        }

        data.resize(size);
        return {source, packet_type, ClientEvent::PACKET};
    }
    return {-1, PacketType::Unknown, ClientEvent::ERR};  // this shouldnt happen
}

ClientEvent SensorPacketSource::get_packet(double timeout_sec) {
    // poll on all our sockets
    InternalEvent internal_event;
    uint64_t timestamp;
    if (do_buffer_) {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        // if the buffer if empty, wait for a new event
        if (buffer_.empty()) {
            auto duration = std::chrono::duration<double>(timeout_sec);
            auto res = buffer_cv_.wait_for(lock, duration);
            // check for timeout or for spurious wakeup of "wait_for"
            // by checking whether the buffer is empty
            if (res == std::cv_status::timeout || buffer_.empty()) {
                return ClientEvent(nullptr, -1, ClientEvent::POLL_TIMEOUT);
            }
        }
        // dequeue
        auto& buf = buffer_.front();
        internal_event = buf.event;
        timestamp = buf.timestamp;
        std::swap(staging_buffer_, buf.data);
        buffer_.pop_front();
        lock.unlock();  // unlock asap
    } else {
        internal_event =
            get_packet_internal(staging_buffer_, timestamp, timeout_sec);
    }

    ClientEvent rev;
    rev.source = internal_event.source;
    rev.type = internal_event.event_type;
    if (internal_event.event_type == ClientEvent::PACKET) {
        if (internal_event.packet_type == PacketType::Imu) {
            rev.packet_ = &imu_packet_;
        } else if (internal_event.packet_type == PacketType::Lidar) {
            rev.packet_ = &lidar_packet_;
        } else if (internal_event.packet_type == PacketType::Zone) {
            rev.packet_ = &zone_packet_;
        } else {
            // Should never happen, but who knows
            throw std::runtime_error(
                "SensorPacketSource received wrong packet type");
        }
        rev.packet_->host_timestamp = timestamp;
        rev.packet_->format = formats_[internal_event.source];
        std::swap(rev.packet_->buf, staging_buffer_);
    } else {
        rev.packet_ = nullptr;
    }
    return rev;
}

uint64_t SensorPacketSource::dropped_packets() {
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    return dropped_packets_;
}

const std::vector<std::shared_ptr<SensorInfo>>&
SensorPacketSource::sensor_info() const {
    return sensor_info_;
}

namespace {
int64_t get_time_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}
}  // namespace

class SensorPacketSource;
class SensorPacketIteratorImpl : public core::PacketIteratorImpl {
    SensorPacketSource* client_;
    std::pair<int, std::shared_ptr<Packet>> packet_;

   public:
    SensorPacketIteratorImpl(SensorPacketSource* client) : client_(client) {}

    bool advance(size_t offset) override {
        int64_t last_time = get_time_ns();
        int64_t timeout_ns = client_->iterator_timeout_ * 1e9;
        double current_timeout_s = client_->iterator_timeout_;
        for (size_t i = 0; i < offset; i++) {
            // get packet can return spurriously if we got bad packets
            // so need to check for timeout independently
            auto event = client_->get_packet(current_timeout_s);
            if (event.type != ClientEvent::PACKET) {
                if (event.type == ClientEvent::POLL_TIMEOUT) {
                    // check for timeout if enabled
                    if (client_->iterator_timeout_ > 0) {
                        auto time_elapsed_ns = get_time_ns() - last_time;
                        if (time_elapsed_ns > timeout_ns) {
                            throw ouster::sdk::sensor::ClientTimeout(
                                "No packets received in timeout.");
                        }
                        // update timeout for timeout remaining
                        current_timeout_s =
                            current_timeout_s - time_elapsed_ns / 1e9;
                    }
                } else {
                    throw std::runtime_error(
                        "An error occurred while reading packets.");
                }
                // try getting a packet again
                offset++;
                continue;
            }

            last_time = event.packet().host_timestamp;
            current_timeout_s = client_->iterator_timeout_;
            packet_ = std::pair<int, std::shared_ptr<Packet>>(
                event.source,
                std::make_shared<Packet>(event.packet()));  // todo dont copy
        }
        return false;
    }

    std::pair<int, std::shared_ptr<Packet>> value() override { return packet_; }
};

core::PacketIterator SensorPacketSource::begin() const {
    return core::PacketIterator(
        this,
        new SensorPacketIteratorImpl(const_cast<SensorPacketSource*>(this)));
}

bool SensorPacketSource::is_live() const { return true; }

ClientEvent::ClientEvent() = default;

}  // namespace sensor
}  // namespace sdk
}  // namespace ouster

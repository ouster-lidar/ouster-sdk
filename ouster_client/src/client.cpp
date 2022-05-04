/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/client.h"

#include <json/json.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ouster/build.h"
#include "ouster/impl/netcompat.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor {

namespace chrono = std::chrono;

struct client {
    SOCKET lidar_fd;
    SOCKET imu_fd;
    std::string hostname;
    Json::Value meta;
    ~client() {
        impl::socket_close(lidar_fd);
        impl::socket_close(imu_fd);
    }
};

// defined in types.cpp
Json::Value to_json(const sensor_config& config, bool compat);

namespace {

// default udp receive buffer size on windows is very low -- use 256K
const int RCVBUF_SIZE = 256 * 1024;

// timeout for reading from a TCP socket during config
const int RCVTIMEOUT_SEC = 10;

int32_t get_sock_port(SOCKET sock_fd) {
    struct sockaddr_storage ss;
    socklen_t addrlen = sizeof ss;

    if (!impl::socket_valid(
            getsockname(sock_fd, (struct sockaddr*)&ss, &addrlen))) {
        std::cerr << "udp getsockname(): " << impl::socket_get_error()
                  << std::endl;
        return SOCKET_ERROR;
    }

    if (ss.ss_family == AF_INET)
        return ntohs(((struct sockaddr_in*)&ss)->sin_port);
    else if (ss.ss_family == AF_INET6)
        return ntohs(((struct sockaddr_in6*)&ss)->sin6_port);
    else
        return SOCKET_ERROR;
}

SOCKET udp_data_socket(int port) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

    auto port_s = std::to_string(port);

    int ret = getaddrinfo(NULL, port_s.c_str(), &hints, &info_start);
    if (ret != 0) {
        std::cerr << "udp getaddrinfo(): " << gai_strerror(ret) << std::endl;
        return SOCKET_ERROR;
    }
    if (info_start == NULL) {
        std::cerr << "udp getaddrinfo(): empty result" << std::endl;
        return SOCKET_ERROR;
    }

    // try to bind a dual-stack ipv6 socket, but fall back to ipv4 only if that
    // fails (when ipv6 is disabled via kernel parameters). Use two passes to
    // deal with glibc addrinfo ordering:
    // https://sourceware.org/bugzilla/show_bug.cgi?id=9981
    for (auto preferred_af : {AF_INET6, AF_INET}) {
        for (ai = info_start; ai != NULL; ai = ai->ai_next) {
            if (ai->ai_family != preferred_af) continue;

            // choose first addrinfo where bind() succeeds
            SOCKET sock_fd =
                socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
            if (!impl::socket_valid(sock_fd)) {
                std::cerr << "udp socket(): " << impl::socket_get_error()
                          << std::endl;
                continue;
            }

            int off = 0;
            if (ai->ai_family == AF_INET6 &&
                setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY, (char*)&off,
                           sizeof(off))) {
                std::cerr << "udp setsockopt(): " << impl::socket_get_error()
                          << std::endl;
                impl::socket_close(sock_fd);
                continue;
            }

            if (impl::socket_set_reuse(sock_fd)) {
                std::cerr << "udp socket_set_reuse(): "
                          << impl::socket_get_error() << std::endl;
            }

            if (bind(sock_fd, ai->ai_addr, (socklen_t)ai->ai_addrlen)) {
                std::cerr << "udp bind(): " << impl::socket_get_error()
                          << std::endl;
                impl::socket_close(sock_fd);
                continue;
            }

            // bind() succeeded; set some options and return
            if (impl::socket_set_non_blocking(sock_fd)) {
                std::cerr << "udp fcntl(): " << impl::socket_get_error()
                          << std::endl;
                impl::socket_close(sock_fd);
                continue;
            }

            if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, (char*)&RCVBUF_SIZE,
                           sizeof(RCVBUF_SIZE))) {
                std::cerr << "udp setsockopt(): " << impl::socket_get_error()
                          << std::endl;
                impl::socket_close(sock_fd);
                continue;
            }

            freeaddrinfo(info_start);
            return sock_fd;
        }
    }

    // could not bind() a UDP server socket
    freeaddrinfo(info_start);
    return SOCKET_ERROR;
}

SOCKET cfg_socket(const char* addr) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    // try to parse as numeric address first: avoids spurious errors from DNS
    // lookup when not using a hostname (and should be faster)
    hints.ai_flags = AI_NUMERICHOST;
    int ret = getaddrinfo(addr, "7501", &hints, &info_start);
    if (ret != 0) {
        hints.ai_flags = 0;
        ret = getaddrinfo(addr, "7501", &hints, &info_start);
        if (ret != 0) {
            std::cerr << "cfg getaddrinfo(): " << gai_strerror(ret)
                      << std::endl;
            return SOCKET_ERROR;
        }
    }

    if (info_start == NULL) {
        std::cerr << "cfg getaddrinfo(): empty result" << std::endl;
        return SOCKET_ERROR;
    }

    SOCKET sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (!impl::socket_valid(sock_fd)) {
            std::cerr << "cfg socket(): " << impl::socket_get_error()
                      << std::endl;
            continue;
        }

        if (connect(sock_fd, ai->ai_addr, (socklen_t)ai->ai_addrlen) < 0) {
            impl::socket_close(sock_fd);
            continue;
        }

        if (impl::socket_set_rcvtimeout(sock_fd, RCVTIMEOUT_SEC)) {
            std::cerr << "cfg set_rcvtimeout(): " << impl::socket_get_error()
                      << std::endl;
            impl::socket_close(sock_fd);
            continue;
        }

        break;
    }

    freeaddrinfo(info_start);
    if (ai == NULL) {
        return SOCKET_ERROR;
    }

    return sock_fd;
}

bool do_tcp_cmd(SOCKET sock_fd, const std::vector<std::string>& cmd_tokens,
                std::string& res) {
    const size_t max_res_len = 16 * 1024;
    auto read_buf = std::unique_ptr<char[]>{new char[max_res_len + 1]};

    std::stringstream ss;
    for (const auto& token : cmd_tokens) ss << token << " ";
    ss << "\n";
    std::string cmd = ss.str();

    ssize_t len = send(sock_fd, cmd.c_str(), cmd.length(), 0);
    if (len != (ssize_t)cmd.length()) {
        return false;
    }

    // need to synchronize with server by reading response
    std::stringstream read_ss;
    do {
        len = recv(sock_fd, read_buf.get(), max_res_len, 0);
        if (len < 0) {
            std::cerr << "do_tcp_cmd recv(): " << impl::socket_get_error()
                      << std::endl;
            return false;
        }
        read_buf.get()[len] = '\0';
        read_ss << read_buf.get();
    } while (len > 0 && read_buf.get()[len - 1] != '\n');

    res = read_ss.str();
    res.erase(res.find_last_not_of(" \r\n\t") + 1);

    return true;
}

bool collect_metadata(client& cli, SOCKET sock_fd, chrono::seconds timeout) {
    Json::CharReaderBuilder builder{};
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root{};
    std::string errors{};

    std::string res;
    bool success = true;

    auto timeout_time = chrono::steady_clock::now() + timeout;

    std::string status;
    do {
        success &= do_tcp_cmd(sock_fd, {"get_sensor_info"}, res);
        success &=
            reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);

        if (chrono::steady_clock::now() >= timeout_time) return false;
        std::this_thread::sleep_for(chrono::seconds(1));
        status = root["status"].asString();
    } while (success && status == "INITIALIZING");
    cli.meta["sensor_info"] = root;

    // not all metadata available when sensor isn't RUNNING
    if (status != "RUNNING") {
        throw std::runtime_error(
            "Cannot initialize with sensor status: " + status +
            ". Please ensure operating mode is set to NORMAL");
    }

    success &= do_tcp_cmd(sock_fd, {"get_beam_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);
    cli.meta["beam_intrinsics"] = root;

    success &= do_tcp_cmd(sock_fd, {"get_imu_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);
    cli.meta["imu_intrinsics"] = root;

    success &= do_tcp_cmd(sock_fd, {"get_lidar_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);
    cli.meta["lidar_intrinsics"] = root;

    success &= do_tcp_cmd(sock_fd, {"get_lidar_data_format"}, res);
    if (success) {
        if (reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL)) {
            cli.meta["lidar_data_format"] = root;
        } else {
            cli.meta["lidar_data_format"] = res;
        }
    }

    success &= do_tcp_cmd(sock_fd, {"get_calibration_status"}, res);
    if (success) {
        if (reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL)) {
            cli.meta["calibration_status"] = root;
        } else {
            cli.meta["calibration_status"] = res;
        }
    }

    success &= do_tcp_cmd(sock_fd, {"get_config_param", "active"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);
    cli.meta["config_params"] = root;

    // merge extra info into metadata
    cli.meta["client_version"] = client_version();

    return success;
}

bool set_config_helper(SOCKET sock_fd, const sensor_config& config,
                       uint8_t config_flags) {
    std::string res{};

    // reset staged config to avoid spurious errors
    std::string active_params;
    if (!do_tcp_cmd(sock_fd, {"get_config_param", "active"}, active_params))
        throw std::runtime_error("Failed to run 'get_config_param'");
    if (!do_tcp_cmd(sock_fd, {"set_config_param", ".", active_params}, res))
        throw std::runtime_error("Failed to run 'set_config_param'");
    if (res != "set_config_param")
        throw std::runtime_error("Error on 'set_config_param': " + res);

    // set automatic udp dest, if flag specified
    if (config_flags & CONFIG_UDP_DEST_AUTO) {
        if (config.udp_dest)
            throw std::invalid_argument(
                "UDP_DEST_AUTO flag set but provided config has udp_dest");

        if (!do_tcp_cmd(sock_fd, {"set_udp_dest_auto"}, res))
            throw std::runtime_error("Failed to run 'set_udp_dest_auto'");

        if (res != "set_udp_dest_auto")
            throw std::runtime_error("Error on 'set_udp_dest_auto': " + res);
    }

    // set all desired config parameters
    Json::Value config_json = to_json(config, true);
    for (const auto& key : config_json.getMemberNames()) {
        auto value = Json::FastWriter().write(config_json[key]);
        value.erase(std::remove(value.begin(), value.end(), '\n'), value.end());

        if (!do_tcp_cmd(sock_fd, {"set_config_param", key, value}, res))
            throw std::runtime_error("Failed to run 'set_config_param'");

        if (res != "set_config_param")
            throw std::invalid_argument("Error on 'set_config_param': " + res);
    }

    // reinitialize to make all staged parameters effective
    if (!do_tcp_cmd(sock_fd, {"reinitialize"}, res))
        throw std::runtime_error("Failed to run 'reinitialize'");

    // reinit will report an error only when staged configs are incompatible
    if (res != "reinitialize") throw std::invalid_argument(res);

    // save if indicated, use deprecated write_config_txt to support 1.13
    if (config_flags & CONFIG_PERSIST) {
        if (!do_tcp_cmd(sock_fd, {"write_config_txt"}, res))
            throw std::runtime_error("Failed to run 'write_config_txt'");

        if (res != "write_config_txt")
            throw std::runtime_error("Error on 'write_config_txt': " + res);
    }

    return true;
}
}  // namespace

bool get_config(const std::string& hostname, sensor_config& config,
                bool active) {
    Json::CharReaderBuilder builder{};
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root{};
    std::string errors{};

    SOCKET sock_fd = cfg_socket(hostname.c_str());
    if (sock_fd < 0) return false;

    std::string res;
    bool success = true;

    std::string active_or_staged = active ? "active" : "staged";
    success &= do_tcp_cmd(sock_fd, {"get_config_param", active_or_staged}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);

    config = parse_config(res);

    impl::socket_close(sock_fd);

    return success;
}

bool set_config(const std::string& hostname, const sensor_config& config,
                uint8_t config_flags) {
    // open socket
    SOCKET sock_fd = cfg_socket(hostname.c_str());
    if (sock_fd < 0) return false;

    try {
        set_config_helper(sock_fd, config, config_flags);
    } catch (...) {
        impl::socket_close(sock_fd);
        throw;
    }

    impl::socket_close(sock_fd);
    return true;
}

std::string get_metadata(client& cli, int timeout_sec, bool legacy_format) {
    if (!cli.meta) {
        SOCKET sock_fd = cfg_socket(cli.hostname.c_str());
        if (sock_fd < 0) return "";

        bool success =
            collect_metadata(cli, sock_fd, chrono::seconds{timeout_sec});

        impl::socket_close(sock_fd);

        if (!success) return "";
    }

    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "    ";

    auto metadata_string = Json::writeString(builder, cli.meta);

    return legacy_format ? convert_to_legacy(metadata_string) : metadata_string;
}

std::shared_ptr<client> init_client(const std::string& hostname, int lidar_port,
                                    int imu_port) {
    auto cli = std::make_shared<client>();
    cli->hostname = hostname;

    cli->lidar_fd = udp_data_socket(lidar_port);
    cli->imu_fd = udp_data_socket(imu_port);

    if (!impl::socket_valid(cli->lidar_fd) || !impl::socket_valid(cli->imu_fd))
        return std::shared_ptr<client>();

    return cli;
}

std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode mode, timestamp_mode ts_mode,
                                    int lidar_port, int imu_port,
                                    int timeout_sec) {
    auto cli = init_client(hostname, lidar_port, imu_port);
    if (!cli) return std::shared_ptr<client>();

    // update requested ports to actual bound ports
    lidar_port = get_sock_port(cli->lidar_fd);
    imu_port = get_sock_port(cli->imu_fd);
    if (!impl::socket_valid(lidar_port) || !impl::socket_valid(imu_port))
        return std::shared_ptr<client>();

    SOCKET sock_fd = cfg_socket(hostname.c_str());
    if (!impl::socket_valid(sock_fd)) return std::shared_ptr<client>();

    std::string res;
    bool success = true;

    // fail fast if we can't reach the sensor via TCP
    success &= do_tcp_cmd(sock_fd, {"get_sensor_info"}, res);
    if (!success) {
        impl::socket_close(sock_fd);
        return std::shared_ptr<client>();
    }

    // if dest address is not specified, have the sensor to set it automatically
    if (udp_dest_host == "") {
        success &= do_tcp_cmd(sock_fd, {"set_udp_dest_auto"}, res);
        success &= res == "set_udp_dest_auto";
    } else {
        success &= do_tcp_cmd(
            sock_fd, {"set_config_param", "udp_ip", udp_dest_host}, res);
        success &= res == "set_config_param";
    }

    success &= do_tcp_cmd(
        sock_fd,
        {"set_config_param", "udp_port_lidar", std::to_string(lidar_port)},
        res);
    success &= res == "set_config_param";

    success &= do_tcp_cmd(
        sock_fd, {"set_config_param", "udp_port_imu", std::to_string(imu_port)},
        res);
    success &= res == "set_config_param";

    // if specified (not UNSPEC), set the lidar and timestamp modes
    if (mode) {
        success &= do_tcp_cmd(
            sock_fd, {"set_config_param", "lidar_mode", to_string(mode)}, res);
        success &= res == "set_config_param";
    }

    if (ts_mode) {
        success &= do_tcp_cmd(
            sock_fd, {"set_config_param", "timestamp_mode", to_string(ts_mode)},
            res);
        success &= res == "set_config_param";
    }

    // wake up from STANDBY, if necessary
    success &=
        do_tcp_cmd(sock_fd, {"set_config_param", "auto_start_flag", "1"}, res);
    success &= res == "set_config_param";

    // reinitialize to activate new settings
    success &= do_tcp_cmd(sock_fd, {"reinitialize"}, res);
    success &= res == "reinitialize";

    // will block until no longer INITIALIZING
    success &= collect_metadata(*cli, sock_fd, chrono::seconds{timeout_sec});

    // check for sensor error states
    auto status = cli->meta["sensor_info"]["status"].asString();
    success &= (status != "ERROR" && status != "UNCONFIGURED");

    impl::socket_close(sock_fd);

    return success ? cli : std::shared_ptr<client>();
}

client_state poll_client(const client& c, const int timeout_sec) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(c.lidar_fd, &rfds);
    FD_SET(c.imu_fd, &rfds);

    timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;

    SOCKET max_fd = std::max(c.lidar_fd, c.imu_fd);

    SOCKET retval = select((int)max_fd + 1, &rfds, NULL, NULL, &tv);

    client_state res = client_state(0);

    if (!impl::socket_valid(retval) && impl::socket_exit()) {
        res = EXIT;
    } else if (!impl::socket_valid(retval)) {
        std::cerr << "select: " << impl::socket_get_error() << std::endl;
        res = client_state(res | CLIENT_ERROR);
    } else if (retval) {
        if (FD_ISSET(c.lidar_fd, &rfds)) res = client_state(res | LIDAR_DATA);
        if (FD_ISSET(c.imu_fd, &rfds)) res = client_state(res | IMU_DATA);
    }

    return res;
}

static bool recv_fixed(SOCKET fd, void* buf, int64_t len) {
    int64_t n = recv(fd, (char*)buf, len + 1, 0);
    if (n == len) {
        return true;
    } else if (n == -1) {
        std::cerr << "recvfrom: " << impl::socket_get_error() << std::endl;
    } else {
        std::cerr << "Unexpected udp packet length: " << n << std::endl;
    }
    return false;
}

bool read_lidar_packet(const client& cli, uint8_t* buf,
                       const packet_format& pf) {
    return recv_fixed(cli.lidar_fd, buf, pf.lidar_packet_size);
}

bool read_imu_packet(const client& cli, uint8_t* buf, const packet_format& pf) {
    return recv_fixed(cli.imu_fd, buf, pf.imu_packet_size);
}

int get_lidar_port(client& cli) { return get_sock_port(cli.lidar_fd); }

int get_imu_port(client& cli) { return get_sock_port(cli.imu_fd); }

}  // namespace sensor
}  // namespace ouster

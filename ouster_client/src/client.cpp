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

namespace {

// default udp receive buffer size on windows is very low -- use 256K
const int RCVBUF_SIZE = 256 * 1024;

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
    hints.ai_family = AF_INET6;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

    auto port_s = std::to_string(port);

    int ret = getaddrinfo(NULL, port_s.c_str(), &hints, &info_start);
    if (ret != 0) {
        std::cerr << "getaddrinfo(): " << gai_strerror(ret) << std::endl;
        return SOCKET_ERROR;
    }
    if (info_start == NULL) {
        std::cerr << "getaddrinfo: empty result" << std::endl;
        return SOCKET_ERROR;
    }

    SOCKET sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (!impl::socket_valid(sock_fd)) {
            std::cerr << "udp socket(): " << impl::socket_get_error()
                      << std::endl;
            continue;
        }

        int off = 0;
        if (setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY, (char*)&off,
                       sizeof(off))) {
            std::cerr << "udp setsockopt(): " << impl::socket_get_error()
                      << std::endl;
            impl::socket_close(sock_fd);
            return SOCKET_ERROR;
        }

        if (bind(sock_fd, ai->ai_addr, (socklen_t)ai->ai_addrlen)) {
            impl::socket_close(sock_fd);
            std::cerr << "udp bind(): " << impl::socket_get_error()
                      << std::endl;
            continue;
        }

        break;
    }

    freeaddrinfo(info_start);
    if (ai == NULL) {
        impl::socket_close(sock_fd);
        return SOCKET_ERROR;
    }

    if (!impl::socket_valid(impl::socket_set_non_blocking(sock_fd))) {
        std::cerr << "udp fcntl(): " << impl::socket_get_error() << std::endl;
        impl::socket_close(sock_fd);
        return SOCKET_ERROR;
    }

    if (!impl::socket_valid(setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF,
                                       (char*)&RCVBUF_SIZE,
                                       sizeof(RCVBUF_SIZE)))) {
        std::cerr << "udp setsockopt(): " << impl::socket_get_error()
                  << std::endl;
        impl::socket_close(sock_fd);
        return SOCKET_ERROR;
    }

    return sock_fd;
}

SOCKET cfg_socket(const char* addr) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int ret = getaddrinfo(addr, "7501", &hints, &info_start);
    if (ret != 0) {
        std::cerr << "getaddrinfo: " << gai_strerror(ret) << std::endl;
        return SOCKET_ERROR;
    }
    if (info_start == NULL) {
        std::cerr << "getaddrinfo: empty result" << std::endl;
        return SOCKET_ERROR;
    }

    SOCKET sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (!impl::socket_valid(sock_fd)) {
            std::cerr << "socket: " << impl::socket_get_error() << std::endl;
            continue;
        }

        if (connect(sock_fd, ai->ai_addr, (socklen_t)ai->ai_addrlen) < 0) {
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
            return false;
        }
        read_buf.get()[len] = '\0';
        read_ss << read_buf.get();
    } while (len > 0 && read_buf.get()[len - 1] != '\n');

    res = read_ss.str();
    res.erase(res.find_last_not_of(" \r\n\t") + 1);

    return true;
}

void update_json_obj(Json::Value& dst, const Json::Value& src) {
    const std::vector<std::string>& members = src.getMemberNames();
    for (const auto& key : members) {
        dst[key] = src[key];
    }
}

bool collect_metadata(client& cli, SOCKET sock_fd, chrono::seconds timeout) {
    Json::CharReaderBuilder builder{};
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root{};
    std::string errors{};

    std::string res;
    bool success = true;

    auto timeout_time = chrono::steady_clock::now() + timeout;

    do {
        success &= do_tcp_cmd(sock_fd, {"get_sensor_info"}, res);
        success &=
            reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);

        if (chrono::steady_clock::now() >= timeout_time) return false;
        std::this_thread::sleep_for(chrono::seconds(1));
    } while (success && root["status"].asString() == "INITIALIZING");

    update_json_obj(cli.meta, root);
    success &= cli.meta["status"].asString() == "RUNNING";

    success &= do_tcp_cmd(sock_fd, {"get_beam_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);
    update_json_obj(cli.meta, root);

    success &= do_tcp_cmd(sock_fd, {"get_imu_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);
    update_json_obj(cli.meta, root);

    success &= do_tcp_cmd(sock_fd, {"get_lidar_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);
    update_json_obj(cli.meta, root);

    // try to query data format
    bool got_format = true;
    got_format &= do_tcp_cmd(sock_fd, {"get_lidar_data_format"}, res);
    got_format &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);
    if (got_format) cli.meta["data_format"] = root;

    // get lidar mode
    success &= do_tcp_cmd(sock_fd, {"get_config_param", "active"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, NULL);

    // merge extra info into metadata
    cli.meta["hostname"] = cli.hostname;
    cli.meta["lidar_mode"] = root["lidar_mode"];
    cli.meta["client_version"] = ouster::CLIENT_VERSION;

    cli.meta["json_calibration_version"] = FW_2_0;

    return success;
}
}  // namespace

std::string get_metadata(client& cli, int timeout_sec) {
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
    return Json::writeString(builder, cli.meta);
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

    success &=
        do_tcp_cmd(sock_fd, {"set_config_param", "udp_ip", udp_dest_host}, res);
    success &= res == "set_config_param";

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

    success &= do_tcp_cmd(sock_fd, {"reinitialize"}, res);
    success &= res == "reinitialize";

    success &= collect_metadata(*cli, sock_fd, chrono::seconds{timeout_sec});

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

}  // namespace sensor
}  // namespace ouster

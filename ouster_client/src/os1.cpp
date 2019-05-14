#include <json/json.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"

namespace ouster {
namespace OS1 {

using ns = std::chrono::nanoseconds;

struct client {
    int lidar_fd;
    int imu_fd;
    Json::Value meta;
    ~client() {
        close(lidar_fd);
        close(imu_fd);
    }
};

namespace {

const std::array<std::pair<lidar_mode, std::string>, 5> lidar_mode_strings = {
    {{MODE_512x10, "512x10"},
     {MODE_512x20, "512x20"},
     {MODE_1024x10, "1024x10"},
     {MODE_1024x20, "1024x20"},
     {MODE_2048x10, "2048x10"}}};

const std::array<std::pair<timestamp_mode, std::string>, 3>
    timestamp_mode_strings = {
        {{TIME_FROM_INTERNAL_OSC, "TIME_FROM_INTERNAL_OSC"},
         {TIME_FROM_SYNC_PULSE_IN, "TIME_FROM_SYNC_PULSE_IN"},
         {TIME_FROM_PTP_1588, "TIME_FROM_PTP_1588"}}};

int32_t get_sock_port(int sock_fd) {
    struct sockaddr_storage ss;
    socklen_t addrlen = sizeof ss;

    if (getsockname(sock_fd, (struct sockaddr*)&ss, &addrlen) < 0) {
        std::cerr << "udp getsockname(): " << std::strerror(errno) << std::endl;
        return -1;
    }

    if (ss.ss_family == AF_INET)
        return ntohs(((struct sockaddr_in*)&ss)->sin_port);
    else if (ss.ss_family == AF_INET6)
        return ntohs(((struct sockaddr_in6*)&ss)->sin6_port);
    else
        return -1;
}

int udp_data_socket(int port) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET6;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

    auto port_s = std::to_string(port);

    int ret = getaddrinfo(NULL, port_s.c_str(), &hints, &info_start);
    if (ret != 0) {
        std::cerr << "getaddrinfo(): " << gai_strerror(ret) << std::endl;
        return -1;
    }
    if (info_start == NULL) {
        std::cerr << "getaddrinfo: empty result" << std::endl;
        return -1;
    }

    int sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock_fd < 0) {
            std::cerr << "udp socket(): " << std::strerror(errno) << std::endl;
            continue;
        }

        if (bind(sock_fd, ai->ai_addr, ai->ai_addrlen) < 0) {
            close(sock_fd);
            std::cerr << "udp bind(): " << std::strerror(errno) << std::endl;
            continue;
        }

        break;
    }

    freeaddrinfo(info_start);
    if (ai == NULL) {
        close(sock_fd);
        return -1;
    }

    if (fcntl(sock_fd, F_SETFL, fcntl(sock_fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
        std::cerr << "udp fcntl(): " << std::strerror(errno) << std::endl;
        close(sock_fd);
        return -1;
    }

    return sock_fd;
}

int cfg_socket(const char* addr) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int ret = getaddrinfo(addr, "7501", &hints, &info_start);
    if (ret != 0) {
        std::cerr << "getaddrinfo: " << gai_strerror(ret) << std::endl;
        return -1;
    }
    if (info_start == NULL) {
        std::cerr << "getaddrinfo: empty result" << std::endl;
        return -1;
    }

    int sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock_fd < 0) {
            std::cerr << "socket: " << std::strerror(errno) << std::endl;
            continue;
        }

        if (connect(sock_fd, ai->ai_addr, ai->ai_addrlen) == -1) {
            close(sock_fd);
            continue;
        }

        break;
    }

    freeaddrinfo(info_start);
    if (ai == NULL) {
        return -1;
    }

    return sock_fd;
}

bool do_tcp_cmd(int sock_fd, const std::vector<std::string>& cmd_tokens,
                std::string& res) {
    const size_t max_res_len = 16 * 1024;
    auto read_buf = std::unique_ptr<char[]>{new char[max_res_len + 1]};

    std::stringstream ss;
    for (const auto& token : cmd_tokens) ss << token << " ";
    ss << "\n";
    std::string cmd = ss.str();

    ssize_t len = write(sock_fd, cmd.c_str(), cmd.length());
    if (len != (ssize_t)cmd.length()) {
        return false;
    }

    // need to synchronize with server by reading response
    std::stringstream read_ss;
    do {
        len = read(sock_fd, read_buf.get(), max_res_len);
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
    for (const auto& key : src.getMemberNames()) dst[key] = src[key];
}
}  // namespace

std::string to_string(version v) {
    if (v == invalid_version) return "UNKNOWN";

    std::stringstream ss{};
    ss << "v" << v.major << "." << v.minor << "." << v.patch;
    return ss.str();
}

version version_of_string(const std::string& s) {
    std::istringstream is{s};
    char c1, c2, c3;
    version v;

    is >> c1 >> v.major >> c2 >> v.minor >> c3 >> v.patch;

    if (is && is.eof() && c1 == 'v' && c2 == '.' && c3 == '.' && v.major >= 0 &&
        v.minor >= 0 && v.patch >= 0)
        return v;
    else
        return invalid_version;
};

std::string to_string(lidar_mode mode) {
    auto end = lidar_mode_strings.end();
    auto res = std::find_if(lidar_mode_strings.begin(), end,
                            [&](const std::pair<lidar_mode, std::string>& p) {
                                return p.first == mode;
                            });

    return res == end ? "UNKNOWN" : res->second;
}

lidar_mode lidar_mode_of_string(const std::string& s) {
    auto end = lidar_mode_strings.end();
    auto res = std::find_if(lidar_mode_strings.begin(), end,
                            [&](const std::pair<lidar_mode, std::string>& p) {
                                return p.second == s;
                            });

    return res == end ? lidar_mode(0) : res->first;
}

int n_cols_of_lidar_mode(lidar_mode mode) {
    switch (mode) {
        case MODE_512x10:
        case MODE_512x20:
            return 512;
        case MODE_1024x10:
        case MODE_1024x20:
            return 1024;
        case MODE_2048x10:
            return 2048;
        default:
            throw std::invalid_argument{"n_cols_of_lidar_mode"};
    }
}

std::string to_string(timestamp_mode mode) {
    auto end = timestamp_mode_strings.end();
    auto res =
        std::find_if(timestamp_mode_strings.begin(), end,
                     [&](const std::pair<timestamp_mode, std::string>& p) {
                         return p.first == mode;
                     });

    return res == end ? "UNKNOWN" : res->second;
}

timestamp_mode timestamp_mode_of_string(const std::string& s) {
    auto end = timestamp_mode_strings.end();
    auto res =
        std::find_if(timestamp_mode_strings.begin(), end,
                     [&](const std::pair<timestamp_mode, std::string>& p) {
                         return p.second == s;
                     });

    return res == end ? timestamp_mode(0) : res->first;
}

std::string get_metadata(const client& cli) {
    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "    ";
    return Json::writeString(builder, cli.meta);
}

sensor_info parse_metadata(const std::string& meta) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{meta};

    if (meta.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::runtime_error{errors.c_str()};
    }

    sensor_info info = {"UNKNOWN", "UNKNOWN", {}, lidar_mode(0),
                        {},        {},        {}, {}};
    info.hostname = root["hostname"].asString();
    info.sn = root["prod_sn"].asString();
    info.fw_rev = root["build_rev"].asString();

    info.mode = lidar_mode_of_string(root["lidar_mode"].asString());

    for (const auto& v : root["beam_altitude_angles"])
        info.beam_altitude_angles.push_back(v.asDouble());
    if (info.beam_altitude_angles.size() != OS1::pixels_per_column)
        info.beam_altitude_angles = {};

    for (const auto& v : root["beam_azimuth_angles"])
        info.beam_azimuth_angles.push_back(v.asDouble());
    if (info.beam_azimuth_angles.size() != OS1::pixels_per_column)
        info.beam_azimuth_angles = {};

    for (const auto& v : root["imu_to_sensor_transform"])
        info.imu_to_sensor_transform.push_back(v.asDouble());
    if (info.imu_to_sensor_transform.size() != 16)
        info.imu_to_sensor_transform = {};

    for (const auto& v : root["lidar_to_sensor_transform"])
        info.lidar_to_sensor_transform.push_back(v.asDouble());
    if (info.lidar_to_sensor_transform.size() != 16)
        info.lidar_to_sensor_transform = {};

    return info;
}

std::shared_ptr<client> init_client(int lidar_port, int imu_port) {
    auto cli = std::make_shared<client>();

    cli->lidar_fd = udp_data_socket(lidar_port);
    cli->imu_fd = udp_data_socket(imu_port);

    if (cli->lidar_fd < 0 || cli->imu_fd < 0)
        return std::shared_ptr<client>();

    return cli;
}

std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode mode, timestamp_mode ts_mode,
                                    int lidar_port, int imu_port) {
    auto cli = init_client(lidar_port, imu_port);
    if (!cli) return std::shared_ptr<client>();

    // update requested ports to actual bound ports
    lidar_port = get_sock_port(cli->lidar_fd);
    imu_port = get_sock_port(cli->imu_fd);
    if (lidar_port == -1 || imu_port == -1) return std::shared_ptr<client>();

    int sock_fd = cfg_socket(hostname.c_str());

    Json::CharReaderBuilder builder{};
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root{};
    std::string errors{};

    if (sock_fd < 0) return std::shared_ptr<client>();

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

    success &= do_tcp_cmd(
        sock_fd, {"set_config_param", "lidar_mode", to_string(mode)}, res);
    success &= res == "set_config_param";

    success &= do_tcp_cmd(
        sock_fd, {"set_config_param", "timestamp_mode", to_string(ts_mode)},
        res);
    success &= res == "set_config_param";

    success &= do_tcp_cmd(sock_fd, {"get_sensor_info"}, res);
    success &= reader->parse(res.c_str(), res.c_str() + res.size(), &cli->meta,
                             &errors);

    success &= do_tcp_cmd(sock_fd, {"get_beam_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, &errors);
    update_json_obj(cli->meta, root);

    success &= do_tcp_cmd(sock_fd, {"get_imu_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, &errors);
    update_json_obj(cli->meta, root);

    success &= do_tcp_cmd(sock_fd, {"get_lidar_intrinsics"}, res);
    success &=
        reader->parse(res.c_str(), res.c_str() + res.size(), &root, &errors);
    update_json_obj(cli->meta, root);

    success &= do_tcp_cmd(sock_fd, {"reinitialize"}, res);
    success &= res == "reinitialize";

    close(sock_fd);

    // merge extra info into metadata
    cli->meta["hostname"] = hostname;
    cli->meta["lidar_mode"] = to_string(mode);

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

    int max_fd = std::max(c.lidar_fd, c.imu_fd);

    int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);

    client_state res = client_state(0);
    if (retval == -1 && errno == EINTR) {
        res = EXIT;
    } else if (retval == -1) {
        std::cerr << "select: " << std::strerror(errno) << std::endl;
        res = client_state(res | ERROR);
    } else if (retval) {
        if (FD_ISSET(c.lidar_fd, &rfds)) res = client_state(res | LIDAR_DATA);
        if (FD_ISSET(c.imu_fd, &rfds)) res = client_state(res | IMU_DATA);
    }
    return res;
}

static bool recv_fixed(int fd, void* buf, ssize_t len) {
    ssize_t n = recv(fd, (char*)buf, len + 1, 0);
    if (n == len)
        return true;
    else if (n == -1)
        std::cerr << "recvfrom: " << std::strerror(errno) << std::endl;
    else
        std::cerr << "Unexpected udp packet length: " << n << std::endl;
    return false;
}

bool read_lidar_packet(const client& cli, uint8_t* buf) {
    return recv_fixed(cli.lidar_fd, buf, lidar_packet_bytes);
}

bool read_imu_packet(const client& cli, uint8_t* buf) {
    return recv_fixed(cli.imu_fd, buf, imu_packet_bytes);
}

}  // namespace OS1
}  // namespace ouster

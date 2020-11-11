#include "ouster/os1.h"

#include <json/json.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "ouster/compat.h"
#include "ouster/os1_impl.h"
#include "ouster/os1_packet.h"

namespace ouster {
namespace OS1 {

namespace chrono = std::chrono;

namespace {

const std::array<std::pair<lidar_mode, std::string>, 5> lidar_mode_strings = {
    {{MODE_512x10, "512x10"},
     {MODE_512x20, "512x20"},
     {MODE_1024x10, "1024x10"},
     {MODE_1024x20, "1024x20"},
     {MODE_2048x10, "2048x10"}}};

int32_t get_sock_port(int sock_fd) {
    struct sockaddr_storage ss;
    socklen_t addrlen = sizeof ss;

    if (!socket_valid(getsockname(sock_fd, (struct sockaddr*)&ss, &addrlen))) {
        std::cerr << "udp getsockname(): " << socket_get_error() << std::endl;
        return SOCKET_ERROR;
    }

    if (ss.ss_family == AF_INET)
        return ntohs(((struct sockaddr_in*)&ss)->sin_port);
    else if (ss.ss_family == AF_INET6)
        return ntohs(((struct sockaddr_in6*)&ss)->sin6_port);
    else
        return SOCKET_ERROR;
}

int udp_data_socket(int port) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
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

    int sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (!socket_valid(sock_fd)) {
            std::cerr << "udp socket(): " << socket_get_error() << std::endl;
            continue;
        }

        if (!socket_valid(bind(sock_fd, ai->ai_addr, ai->ai_addrlen))) {
            socket_close(sock_fd);
            std::cerr << "udp bind(): " << socket_get_error() << std::endl;
            continue;
        }

        break;
    }

    freeaddrinfo(info_start);
    if (ai == NULL) {
        socket_close(sock_fd);
        return SOCKET_ERROR;
    }

    if (!socket_valid(socket_set_non_blocking(sock_fd))) {
        std::cerr << "udp fcntl(): " << socket_get_error() << std::endl;
        socket_close(sock_fd);
        return SOCKET_ERROR;
    }

    return sock_fd;
}

int cfg_socket(const char* addr) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
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

    int sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (!socket_valid(sock_fd)) {
            std::cerr << "socket: " << socket_get_error() << std::endl;
            continue;
        }

        if (connect(sock_fd, ai->ai_addr, ai->ai_addrlen) == -1) {
            socket_close(sock_fd);
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

bool do_tcp_cmd(int sock_fd, const std::vector<std::string>& cmd_tokens,
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

bool collect_metadata(client& cli, const int sock_fd, chrono::seconds timeout) {
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

    return success;
}
}  // namespace

data_format default_data_format(lidar_mode mode) {
    auto repeat = [](int n, const std::vector<uint32_t>& v) {
        std::vector<uint32_t> res{};
        for (int i = 0; i < n; i++) res.insert(res.end(), v.begin(), v.end());
        return res;
    };

    uint32_t pixels_per_column = 64;
    uint32_t columns_per_packet = 16;
    uint32_t columns_per_frame = n_cols_of_lidar_mode(mode);

    std::vector<uint32_t> offset;
    switch (columns_per_frame) {
        case 512:
            offset = repeat(16, {9, 6, 3, 0});
            break;
        case 1024:
            offset = repeat(16, {18, 12, 6, 0});
            break;
        case 2048:
            offset = repeat(16, {36, 24, 12, 0});
            break;
        default:
            offset = repeat(16, {18, 12, 6, 0});
            break;
    }

    return {pixels_per_column, columns_per_packet, columns_per_frame, offset};
}

double default_lidar_origin_to_beam_origin(std::string prod_line){
    double lidar_origin_to_beam_origin_mm = 12.163; // default for gen 1
    if (prod_line.find("OS-0-") == 0)
        lidar_origin_to_beam_origin_mm = 27.67;
    else if (prod_line.find("OS-1-") == 0)
        lidar_origin_to_beam_origin_mm = 15.806;
    else if (prod_line.find("OS-2-") == 0)
        lidar_origin_to_beam_origin_mm = 13.762;
    return lidar_origin_to_beam_origin_mm;
}

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

std::string get_metadata(client& cli, int timeout_sec) {
    if (!cli.meta) {
        int sock_fd = cfg_socket(cli.hostname.c_str());
        if (sock_fd < 0) return "";

        bool success =
            collect_metadata(cli, sock_fd, chrono::seconds{timeout_sec});

        socket_close(sock_fd);

        if (!success) return "";
    }

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

    sensor_info info{};

    info.hostname = root["hostname"].asString();
    info.sn = root["prod_sn"].asString();
    info.fw_rev = root["build_rev"].asString();
    info.mode = lidar_mode_of_string(root["lidar_mode"].asString());
    info.prod_line = root["prod_line"].asString();

    // "data_format" introduced in fw 1.14. Fall back to common 1.13 parameters
    // otherwise
    if (root.isMember("data_format")) {
        info.format.pixels_per_column =
            root["data_format"]["pixels_per_column"].asInt();
        info.format.columns_per_packet =
            root["data_format"]["columns_per_packet"].asInt();
        info.format.columns_per_frame =
            root["data_format"]["columns_per_frame"].asInt();

        for (const auto& v : root["data_format"]["pixel_shift_by_row"])
            info.format.pixel_shift_by_row.push_back(v.asInt());
    } else {
        info.format = default_data_format(info.mode);
    }
    // "lidar_origin_to_beam_origin_mm" introduced in fw 1.14. Fall back to common
    // 1.13 parameters otherwise
    if (root.isMember("lidar_origin_to_beam_origin_mm")) {
        info.lidar_origin_to_beam_origin_mm =
            root["lidar_origin_to_beam_origin_mm"].asDouble();
    } else {
        info.lidar_origin_to_beam_origin_mm =
            default_lidar_origin_to_beam_origin(info.prod_line);
    }

    for (const auto& v : root["beam_altitude_angles"])
        info.beam_altitude_angles.push_back(v.asDouble());

    for (const auto& v : root["beam_azimuth_angles"])
        info.beam_azimuth_angles.push_back(v.asDouble());

    for (const auto& v : root["imu_to_sensor_transform"])
        info.imu_to_sensor_transform.push_back(v.asDouble());

    for (const auto& v : root["lidar_to_sensor_transform"])
        info.lidar_to_sensor_transform.push_back(v.asDouble());

    return info;
}

std::string to_string(const sensor_info& info) {
    Json::Value root{};
    root["hostname"] = info.hostname;
    root["prod_sn"] = info.sn;
    root["build_rev"] = info.fw_rev;
    root["lidar_mode"] = to_string(info.mode);
    root["prod_line"] = info.prod_line;
    root["data_format"]["pixels_per_column"] = info.format.pixels_per_column;
    root["data_format"]["columns_per_packet"] = info.format.columns_per_packet;
    root["data_format"]["columns_per_frame"] = info.format.columns_per_frame;
    root["lidar_origin_to_beam_origin_mm"] = info.lidar_origin_to_beam_origin_mm;

    for (auto i : info.format.pixel_shift_by_row)
        root["data_format"]["pixel_shift_by_row"].append(i);

    for (auto i : info.beam_azimuth_angles)
        root["beam_azimuth_angles"].append(i);

    for (auto i : info.beam_altitude_angles)
        root["beam_altitude_angles"].append(i);

    for (auto i : info.imu_to_sensor_transform)
        root["imu_to_sensor_transform"].append(i);

    for (auto i : info.imu_to_sensor_transform)
        root["lidar_to_sensor_transform"].append(i);

    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "    ";
    return Json::writeString(builder, root);
}

std::shared_ptr<client> init_client(const std::string& hostname, int lidar_port,
                                    int imu_port) {
    auto cli = std::make_shared<client>();
    cli->hostname = hostname;

    cli->lidar_fd = udp_data_socket(lidar_port);
    cli->imu_fd = udp_data_socket(imu_port);

    return cli;
}

std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode mode, int lidar_port,
                                    int imu_port, int timeout_sec) {
    auto cli = init_client(hostname, lidar_port, imu_port);

    // update requested ports to actual bound ports
    lidar_port = get_sock_port(cli->lidar_fd);
    imu_port = get_sock_port(cli->imu_fd);
    if (!socket_valid(lidar_port) || !socket_valid(imu_port))
        return std::shared_ptr<client>();

    int sock_fd = cfg_socket(hostname.c_str());

    std::string errors{};

    if (!socket_valid(sock_fd)) return std::shared_ptr<client>();

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

    success &= do_tcp_cmd(sock_fd, {"reinitialize"}, res);
    success &= res == "reinitialize";

    success &= collect_metadata(*cli, sock_fd, chrono::seconds{timeout_sec});

    socket_close(sock_fd);

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

    if (!socket_valid(retval) && socket_exit()) {
        res = EXIT;
    } else if (!socket_valid(retval)) {
        std::cerr << "select: " << socket_get_error() << std::endl;
        res = client_state(res | CLIENT_ERROR);
    } else if (retval) {
        if (FD_ISSET(c.lidar_fd, &rfds)) res = client_state(res | LIDAR_DATA);
        if (FD_ISSET(c.imu_fd, &rfds)) res = client_state(res | IMU_DATA);
    }

    return res;
}

static bool recv_fixed(int fd, void* buf, int64_t len) {
    int64_t n = recv(fd, (char*)buf, len + 1, 0);
    if (n == len) {
        return true;
    } else if (n == static_cast<int64_t>(-1)) {
        std::cerr << "recvfrom: " << socket_get_error() << std::endl;
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

}  // namespace OS1
}  // namespace ouster

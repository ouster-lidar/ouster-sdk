/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include "sensor_tcp_imp.h"

#include <algorithm>
#include <cstring>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <sstream>

#include "ouster/impl/logging.h"

using ouster::sensor::util::UserDataAndPolicy;
using namespace ouster::sensor::impl;

SensorTcpImp::SensorTcpImp(const std::string& hostname)
    : socket_handle(cfg_socket(hostname.c_str())),
      read_buf(std::unique_ptr<char[]>{new char[MAX_RESULT_LENGTH + 1]}) {}

SensorTcpImp::~SensorTcpImp() { socket_close(socket_handle); }

std::string SensorTcpImp::metadata(int timeout_sec) const {
    auto res = get_config_params(true, timeout_sec);
    jsoncons::json config_params;
    bool parse_success = false;
    try {
        config_params = jsoncons::json::parse(res);
        parse_success = true;
    } catch (jsoncons::ser_error&) {
    }

    jsoncons::json root;
    root["sensor_info"] = jsoncons::json::parse(sensor_info(timeout_sec));
    root["beam_intrinsics"] =
        jsoncons::json::parse(beam_intrinsics(timeout_sec));
    root["imu_intrinsics"] = jsoncons::json::parse(imu_intrinsics(timeout_sec));
    root["lidar_intrinsics"] =
        jsoncons::json::parse(lidar_intrinsics(timeout_sec));
    root["lidar_data_format"] =
        jsoncons::json::parse(lidar_data_format(timeout_sec));
    root["calibration_status"] =
        jsoncons::json::parse(calibration_status(timeout_sec));
    root["config_params"] = (parse_success) ? config_params : res;
    std::string result;
    root.dump(result);
    return result;
}

std::string SensorTcpImp::sensor_info(int /*timeout_sec*/) const {
    return tcp_cmd({"get_sensor_info"});
}

std::string SensorTcpImp::get_config_params(bool active,
                                            int /*timeout_sec*/) const {
    auto config_type = active ? "active" : "staged";
    return tcp_cmd({"get_config_param", config_type});
}

namespace {
std::string rtrim(const std::string& s) {
    return s.substr(
        0,
        std::distance(s.begin(),
                      std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
                          return !isspace(ch);
                      }).base()));
}
}  // namespace

void SensorTcpImp::set_config_param(const std::string& key,
                                    const std::string& value,
                                    int /*timeout_sec*/) const {
    tcp_cmd_with_validation({"set_config_param", key, rtrim(value)},
                            "set_config_param");
}

std::string SensorTcpImp::active_config_params(int /*timeout_sec*/) const {
    return tcp_cmd({"get_config_param", "active"});
}

std::string SensorTcpImp::staged_config_params(int /*timeout_sec*/) const {
    return tcp_cmd({"get_config_param", "staged"});
}

void SensorTcpImp::set_udp_dest_auto(int /*timeout_sec*/) const {
    tcp_cmd_with_validation({"set_udp_dest_auto"}, "set_udp_dest_auto");
}

std::string SensorTcpImp::beam_intrinsics(int /*timeout_sec*/) const {
    return tcp_cmd({"get_beam_intrinsics"});
}

std::string SensorTcpImp::imu_intrinsics(int /*timeout_sec*/) const {
    return tcp_cmd({"get_imu_intrinsics"});
}

std::string SensorTcpImp::lidar_intrinsics(int /*timeout_sec*/) const {
    return tcp_cmd({"get_lidar_intrinsics"});
}

std::string SensorTcpImp::lidar_data_format(int /*timeout_sec*/) const {
    return tcp_cmd({"get_lidar_data_format"});
}

std::string SensorTcpImp::calibration_status(int /*timeout_sec*/) const {
    return tcp_cmd({"get_calibration_status"});
}

void SensorTcpImp::reinitialize(int /*timeout_sec*/) const {
    // reinitialize to make all staged parameters effective
    tcp_cmd_with_validation({"reinitialize"}, "reinitialize");
}

void SensorTcpImp::save_config_params(int /*timeout_sec*/) const {
    tcp_cmd_with_validation({"write_config_txt"}, "write_config_txt");
}

std::string SensorTcpImp::get_user_data(int /*timeout_sec*/) const {
    throw std::runtime_error("user data API not supported on this FW version");
}

UserDataAndPolicy SensorTcpImp::get_user_data_and_policy(
    int /*timeout_sec*/) const {
    throw std::runtime_error("user data API not supported on this FW version");
}

void SensorTcpImp::set_user_data(const std::string& /*data*/,
                                 bool /*keep_on_config_delete*/,
                                 int /*timeout_sec*/) const {
    throw std::runtime_error("user data API not supported on this FW version");
}

void SensorTcpImp::delete_user_data(int /*timeout_sec*/) const {
    throw std::runtime_error("user data API not supported on this FW version");
}

std::string SensorTcpImp::network(int /*timeout_sec*/) const {
    throw std::runtime_error(
        "This endpoint is not supported on this FW version");
}

void SensorTcpImp::set_static_ip(const std::string& /*ip_address*/,
                                 int /*timeout_sec*/) const {
    throw std::runtime_error(
        "This endpoint is not supported on this FW version");
}

void SensorTcpImp::delete_static_ip(int /*timeout_sec*/) const {
    throw std::runtime_error(
        "This endpoint is not supported on this FW version");
}

std::vector<uint8_t> SensorTcpImp::diagnostics_dump(int /*timeout_sec*/) const {
    throw std::runtime_error(
        "This endpoint is not supported on this FW version");
}

SOCKET SensorTcpImp::cfg_socket(const char* addr) {
    struct addrinfo hints, *info_start, *ai;

    std::memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    // try to parse as numeric address first: avoids spurious errors from
    // DNS lookup when not using a hostname (and should be faster)
    hints.ai_flags = AI_NUMERICHOST;
    int ret = getaddrinfo(addr, "7501", &hints, &info_start);
    if (ret != 0) {
        hints.ai_flags = 0;
        ret = getaddrinfo(addr, "7501", &hints, &info_start);
        if (ret != 0) {
            logger().error("cfg getaddrinfo(): {}", gai_strerror(ret));
            return SOCKET_ERROR;
        }
    }

    if (info_start == nullptr) {
        logger().error("cfg getaddrinfo(): empty result");
        return SOCKET_ERROR;
    }

    SOCKET sock_fd;
    for (ai = info_start; ai != nullptr; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (!socket_valid(sock_fd)) {
            logger().error("cfg socket(): {}", socket_get_error());
            continue;
        }

        if (connect(sock_fd, ai->ai_addr, (socklen_t)ai->ai_addrlen) < 0) {
            socket_close(sock_fd);
            continue;
        }

        if (socket_set_rcvtimeout(sock_fd, RCVTIMEOUT_SEC)) {
            logger().error("cfg set_rcvtimeout(): {}", socket_get_error());
            socket_close(sock_fd);
            continue;
        }

        break;
    }

    freeaddrinfo(info_start);
    if (ai == nullptr) {
        return SOCKET_ERROR;
    }

    return sock_fd;
}

std::string SensorTcpImp::tcp_cmd(
    const std::vector<std::string>& cmd_tokens) const {
    std::stringstream ss;
    for (const auto& token : cmd_tokens) ss << token << " ";
    ss << "\n";
    std::string cmd = ss.str();

    ssize_t len = send(socket_handle, cmd.c_str(), cmd.length(), 0);
    if (len != (ssize_t)cmd.length()) {
        throw std::runtime_error("tcp_cmd socket::send failed");
    }

    // need to synchronize with server by reading response
    std::stringstream read_ss;
    do {
        len = recv(socket_handle, read_buf.get(), MAX_RESULT_LENGTH, 0);
        if (len < 0) {
            throw std::runtime_error("tcp_cmd recv(): " + socket_get_error());
        }

        read_buf.get()[len] = '\0';
        read_ss << read_buf.get();
    } while (len > 0 && read_buf.get()[len - 1] != '\n');

    auto res = read_ss.str();
    res.erase(res.find_last_not_of(" \r\n\t") + 1);
    return res;
}

void SensorTcpImp::tcp_cmd_with_validation(
    const std::vector<std::string>& cmd_tokens,
    const std::string& validation) const {
    auto result = tcp_cmd(cmd_tokens);
    if (result != validation) {
        throw std::runtime_error("SensorTcp::tcp_cmd failed: " + cmd_tokens[0] +
                                 " command returned [" + result +
                                 "], expected [" + validation + "]");
    }
}

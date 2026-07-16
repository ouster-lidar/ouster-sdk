/**
 * Copyright(c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * Linting exceptions:
 * modernize-return-braced-init-list : To preserve the readability of return.
 * misc-include-cleaner : clang-tidy unable to locate include headers from
 *                        netcompat.h contains platform specific headers
 * misc-use-internal-linkage: recommends methods exposed in the shared library
 *                        to be marked as static, point viz tests will fail if
 *                        these are marked static.
 */

#include "ouster/sensor/client.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "jsoncons/basic_json.hpp"
#include "ouster/core/defaults.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/metadata.h"
#include "ouster/core/types.h"
#include "ouster/sensor/impl/netcompat.h"
#include "ouster/sensor/sensor_http.h"

using std::chrono_literals::operator""s;

using namespace std::chrono_literals;
namespace chrono = std::chrono;
using ouster::sdk::core::logger;
using ouster::sdk::core::impl::Logger;
using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace core {
// defined in types.cpp
// NOLINTNEXTLINE (misc-use-internal-linkage)
jsoncons::json config_to_json(const ouster::sdk::core::SensorConfig& config);
}  // namespace core
namespace sensor {

// NOLINTNEXTLINE (misc-use-internal-linkage)
int32_t get_sock_port(SOCKET sock_fd) {
    // NOLINTBEGIN (misc-include-cleaner)
    struct sockaddr_storage socket_addr = {};
    socklen_t addrlen = sizeof socket_addr;

    if (!impl::socket_valid(getsockname(sock_fd, (struct sockaddr*)&socket_addr, &addrlen))) {
        logger().error("udp getsockname(): {}", impl::socket_get_error());
        return SOCKET_ERROR;
    }

    if (socket_addr.ss_family == AF_INET) {
        return ntohs(((struct sockaddr_in*)&socket_addr)->sin_port);
    } else if (socket_addr.ss_family == AF_INET6) {
        return ntohs(((struct sockaddr_in6*)&socket_addr)->sin6_port);
    } else {
        return SOCKET_ERROR;
    }
    // NOLINTEND (misc-include-cleaner)
}

// NOLINTNEXTLINE (misc-use-internal-linkage)
SOCKET mtp_data_socket(int port, const std::set<std::string>& udp_dest_hosts,
                       const std::string& mtp_dest_host = "", bool reuse_ports = true,
                       uint32_t rcvbuf_size = 1024 * 1024) {
    // try to bind a dual-stack ipv6 socket, but fall back to ipv4 only if that
    // fails (when ipv6 is disabled via kernel parameters)
    for (auto preferred_af : {AF_INET6, AF_INET}) {
        // choose first addrinfo where bind() succeeds
        // NOLINTNEXTLINE (misc-include-cleaner)
        SOCKET sock_fd = socket(preferred_af, SOCK_DGRAM, 0);
        if (!impl::socket_valid(sock_fd)) {
            logger().warn("udp socket(): {}", impl::socket_get_error());
            continue;
        }

        int off = 0;  // 1 to enable IPV6_V6ONLY option
        if (preferred_af == AF_INET6 &&
            // NOLINTNEXTLINE (misc-include-cleaner)
            setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY, (char*)&off, sizeof(off))) {
            logger().warn("udp setsockopt(IPV6_V6ONLY): {}", impl::socket_get_error());
            impl::socket_close(sock_fd);
            continue;
        }

        if (reuse_ports && impl::socket_set_reuse(sock_fd) != 0) {
            logger().warn("udp socket_set_reuse(): {}", impl::socket_get_error());
        }

        if (preferred_af == AF_INET6) {
            struct sockaddr_in6 address {};
            memset(&address, 0, sizeof(address));
            address.sin6_family = AF_INET6;
            address.sin6_addr = in6addr_any;  // NOLINT (misc-include-cleaner)
            address.sin6_port = htons(port);
            address.sin6_scope_id = 0;
            if (::bind(sock_fd, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) !=
                0) {
                logger().warn("udp bind(): {}", impl::socket_get_error());
                impl::socket_close(sock_fd);
                continue;
            }
        } else {
            struct sockaddr_in address {};
            memset(&address, 0, sizeof(address));
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;  // NOLINT (misc-include-cleaner)
            address.sin_port = htons(port);        // NOLINT (misc-include-cleaner)
            if (::bind(sock_fd, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) !=
                0) {
                logger().warn("udp bind(): {}", impl::socket_get_error());
                impl::socket_close(sock_fd);
                continue;
            }
        }

        // bind() succeeded; join to multicast groups
        for (const auto& udp_dest_host : udp_dest_hosts) {
            // NOLINTBEGIN (misc-include-cleaner)
            ip_mreq mreq{};
            mreq.imr_multiaddr.s_addr = inet_addr(udp_dest_host.c_str());
            if (!mtp_dest_host.empty()) {
                mreq.imr_interface.s_addr = inet_addr(mtp_dest_host.c_str());
            } else {
                mreq.imr_interface.s_addr = htonl(INADDR_ANY);
            }

            if (setsockopt(sock_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq))) {
                logger().warn("udp setsockopt(IP_ADD_MEMBERSHIP): {}", impl::socket_get_error());
                impl::socket_close(sock_fd);
                continue;
            }
            // NOLINTEND (misc-include-cleaner)
        }

        // join to multicast group succeeded; set some options and return
        if (impl::socket_set_non_blocking(sock_fd) != 0) {
            logger().warn("udp fcntl(): {}", impl::socket_get_error());
            impl::socket_close(sock_fd);
            continue;
        }

        const int ircvbuf_size = static_cast<int>(rcvbuf_size);

        // NOLINTNEXTLINE (misc-include-cleaner)
        if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&ircvbuf_size),
                       sizeof(ircvbuf_size))) {
            logger().warn("udp setsockopt(SO_RCVBUF): {}", impl::socket_get_error());
            impl::socket_close(sock_fd);
            continue;
        }

        // Validate that we successfully set the rcvbuf size
        int actual_rcvbuf_value = 0;
        socklen_t actual_rcvbuf_size = sizeof(actual_rcvbuf_value);
        if (getsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<char*>(&actual_rcvbuf_value), &actual_rcvbuf_size) != 0) {
            logger().warn("udp getsockopt(SO_RCVBUF): {}", impl::socket_get_error());
        } else if (actual_rcvbuf_value < ircvbuf_size) {
            logger().warn(
                "Failed to set desired SO_RCVBUF size to {}. Actual was {}. "
                "You may experience packet drop unless you allow a larger "
                "receive buffer.",
                ircvbuf_size, actual_rcvbuf_value);
        }

        return sock_fd;
    }

    // could not bind() a MTP server socket
    logger().error("failed to bind udp socket");
    return SOCKET_ERROR;
}

// NOLINTNEXTLINE (misc-use-internal-linkage)
SOCKET udp_data_socket(int port) {
    return mtp_data_socket(port, {});
}

// NOLINTNEXTLINE (misc-use-internal-linkage)
jsoncons::json collect_metadata(SensorHttp& sensor_http, int timeout_sec) {
    // Note, this function throws std::runtime_error if
    // 1. the metadata couldn't be retrieved
    // 2. the sensor is in the INITIALIZING state when timeout is reached
    auto timeout_time = chrono::steady_clock::now() + chrono::seconds{timeout_sec};

    std::string status;
    // TODO: can remove this loop when we drop support for FW 2.4
    while (true) {
        if (chrono::steady_clock::now() >= timeout_time) {
            throw std::runtime_error(
                "A timeout occurred while waiting for the sensor to "
                "initialize.");
        }

        status =
            jsoncons::json::parse(sensor_http.sensor_info(timeout_sec))["status"].as<std::string>();
        if (status != "INITIALIZING") {
            break;
        }
        std::this_thread::sleep_for(1s);
    }

    std::string user_data;
    try {
        user_data = sensor_http.get_user_data(timeout_sec);
    } catch (const std::runtime_error& e) {
        if (strcmp(e.what(), "user data API not supported on this FW version") != 0) {
            throw e;
        }
    }

    try {
        auto metadata = jsoncons::json::parse(sensor_http.metadata(timeout_sec));

        metadata["ouster-sdk"]["client_version"] = ouster::sdk::core::client_version();
        metadata["ouster-sdk"]["output_source"] = "collect_metadata";
        metadata["user_data"] = user_data;

        // We can't insert this logic into the light init_client since its
        // advantage is that it doesn't make network calls but we need it to run
        // every time there is a valid connection to the sensor So we insert it
        // here
        // TODO: remove after release of FW 3.2/3.3 (sufficient warning)
        const auto& fw_version = sensor_http.firmware_version();

        // only warn for people on the latest FW, as people on older FWs may not
        // care
        if (fw_version.major >= 3 && metadata["config_params"]["udp_profile_lidar"] == "LEGACY") {
            logger().warn(
                "Please note that the Legacy Lidar Profile will be deprecated "
                "in the sensor FW soon. If you plan to upgrade your FW, we "
                "recommend using the Single Return Profile instead. For users "
                "sticking with older FWs, the Ouster SDK will continue to "
                "parse "
                "the legacy lidar profile.");
        }
        return metadata;
    } catch (const std::runtime_error& e) {
        throw std::runtime_error("Cannot obtain full metadata with sensor status: " + status +
                                 ". Please ensure that sensor is not in a STANDBY, UNCONFIGURED, "
                                 "WARMUP, or ERROR state");
    }
}

namespace {
ouster::sdk::core::SensorConfig get_config(SensorHttp& sensor_http, bool active = true,
                                           int timeout_sec = LONG_HTTP_REQUEST_TIMEOUT_SECONDS) {
    auto res = sensor_http.get_config_params(active, timeout_sec);
    ValidatorIssues issues;
    ouster::sdk::core::SensorConfig config;
    if (!parse_and_validate_config(res, config, issues)) {
        throw std::runtime_error(to_string(issues.critical));
    }
    return config;
}
}  // anonymous namespace

ouster::sdk::core::SensorConfig get_config(const std::string& hostname, bool active,
                                           int timeout_sec) {
    auto sensor_http = SensorHttp::create(hostname, timeout_sec);
    return get_config(*sensor_http, active, timeout_sec);
}

// NOLINTNEXTLINE (misc-use-internal-linkage)
void set_config(SensorHttp& sensor_http, const SensorConfig& config, uint8_t config_flags,
                int timeout_sec) {
    // reset staged config to avoid spurious errors
    jsoncons::json config_params =
        jsoncons::json::parse(sensor_http.active_config_params(timeout_sec));
    jsoncons::json config_params_copy = config_params;

    // set all desired config parameters
    jsoncons::json config_json = core::config_to_json(config);
    for (const auto& it : config_json.object_range()) {
        config_params[it.key()] = it.value();
    }

    if (config_json.contains("operating_mode") && config_params.contains("auto_start_flag")) {
        // we're setting operating mode and this sensor has a FW with
        // auto_start_flag
        config_params["auto_start_flag"] = config_json["operating_mode"] == "NORMAL" ? 1 : 0;
    }

    // Signal multiplier changed from int to double for FW 3.0/2.5+, with
    // corresponding change to config.signal_multiplier.
    // Change values 1, 2, 3 back to ints to support older FWs
    if (config_json.contains("signal_multiplier")) {
        check_signal_multiplier(config_params["signal_multiplier"].as<double>());
        if (config_params["signal_multiplier"].as<double>() != 0.25 &&
            config_params["signal_multiplier"].as<double>() != 0.5) {
            config_params["signal_multiplier"] = config_params["signal_multiplier"].as<int>();
        }
    }

    // detect and handle @auto udp dest properly
    if (config.udp_dest == "@auto") {
        config_flags |= CONFIG_UDP_DEST_AUTO;
    }

    // set automatic udp dest, if flag specified
    if ((config_flags & CONFIG_UDP_DEST_AUTO) != 0) {
        if (config.udp_dest && config.udp_dest != "@auto") {
            throw std::invalid_argument("UDP_DEST_AUTO flag set but provided config has udp_dest");
        }
        sensor_http.set_udp_dest_auto(timeout_sec);

        auto staged = jsoncons::json::parse(sensor_http.staged_config_params(timeout_sec));

        // now we set config_params according to the staged udp_dest from the
        // sensor
        if (staged.contains("udp_ip")) {  // means the FW version carries udp_ip
            config_params["udp_ip"] = staged["udp_ip"].as<std::string>();
            config_params["udp_dest"] = staged["udp_ip"].as<std::string>();
        } else {  // don't need to worry about udp_ip
            config_params["udp_dest"] = staged["udp_dest"].as<std::string>();
        }

        // if ZM is enabled/present set the dest there too
        if (staged.contains("udp_dest_zm")) {
            config_params["udp_dest_zm"] = staged["udp_dest"].as<std::string>();
        }
    }

    // if configuration didn't change then skip applying the params
    // note: comparison will fail if config_params contains newer config params
    // introduced after the verison of FW the sensor is on
    if ((config_flags & CONFIG_FORCE_REINIT) != 0 || (config_params_copy != config_params)) {
        // send full string -- depends on older FWs not rejecting a blob even
        // when it contains unknown keys
        std::string config_params_str;
        config_params.dump(config_params_str);
        sensor_http.set_config_param(".", config_params_str, timeout_sec);
        // reinitialize to make all staged parameters effective
        sensor_http.reinitialize(timeout_sec);
    }

    // save if indicated
    if ((config_flags & CONFIG_PERSIST) != 0) {
        sensor_http.save_config_params(timeout_sec);
    }
}

void set_config(const std::string& hostname, const SensorConfig& config, uint8_t config_flags,
                int timeout_sec) {
    auto sensor_http = SensorHttp::create(hostname, timeout_sec);
    set_config(*sensor_http, config, config_flags, timeout_sec);
}

bool init_logger(const std::string& log_level, const std::string& log_file_path, bool rotating,
                 int max_size_in_bytes, int max_files) {
    if (log_file_path.empty()) {
        return Logger::instance().configure_stderr_sink(log_level);
    } else {
        return Logger::instance().configure_file_sink(log_level, log_file_path, rotating,
                                                      max_size_in_bytes, max_files);
    }
}

bool in_multicast(const std::string& addr) {
    // NOLINTNEXTLINE (misc-include-cleaner)
    return IN_MULTICAST(ntohl(inet_addr(addr.c_str())));
}

}  // namespace sensor
}  // namespace sdk
}  // namespace ouster

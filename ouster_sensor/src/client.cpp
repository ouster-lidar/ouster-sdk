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

#include "ouster/client.h"

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
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "jsoncons/basic_json.hpp"
#include "ouster/defaults.h"
#include "ouster/impl/client_poller.h"
#include "ouster/impl/logging.h"
#include "ouster/impl/netcompat.h"
#include "ouster/metadata.h"
#include "ouster/sensor_http.h"
#include "ouster/types.h"

using namespace std::chrono_literals;
namespace chrono = std::chrono;
using ouster::sensor::impl::Logger;
using ouster::sensor::util::SensorHttp;

namespace ouster {
namespace sensor {

struct client {
    SOCKET lidar_fd{};
    SOCKET imu_fd{};
    std::string hostname;
    ~client() {
        impl::socket_close(lidar_fd);
        impl::socket_close(imu_fd);
    }
};

// defined in types.cpp
// NOLINTNEXTLINE (misc-use-internal-linkage)
jsoncons::json config_to_json(const sensor_config& config);

// default udp receive buffer size on windows is very low -- use 1MB
const int RCVBUF_SIZE = 1024 * 1024;

// NOLINTNEXTLINE (misc-use-internal-linkage)
int32_t get_sock_port(SOCKET sock_fd) {
    // NOLINTBEGIN (misc-include-cleaner)
    struct sockaddr_storage socket_addr = {};
    socklen_t addrlen = sizeof socket_addr;

    if (!impl::socket_valid(
            getsockname(sock_fd, (struct sockaddr*)&socket_addr, &addrlen))) {
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
SOCKET mtp_data_socket(int port, const std::vector<std::string>& udp_dest_hosts,
                       const std::string& mtp_dest_host = "") {
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
            setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY, (char*)&off,
                       sizeof(off))) {
            logger().warn("udp setsockopt(IPV6_V6ONLY): {}",
                          impl::socket_get_error());
            impl::socket_close(sock_fd);
            continue;
        }

        if (impl::socket_set_reuse(sock_fd)) {
            logger().warn("udp socket_set_reuse(): {}",
                          impl::socket_get_error());
        }

        if (preferred_af == AF_INET6) {
            struct sockaddr_in6 address {};
            memset(&address, 0, sizeof(address));
            address.sin6_family = AF_INET6;
            address.sin6_addr = in6addr_any;  // NOLINT (misc-include-cleaner)
            address.sin6_port = htons(port);
            address.sin6_scope_id = 0;
            if (::bind(sock_fd, reinterpret_cast<struct sockaddr*>(&address),
                       sizeof(address))) {
                logger().warn("udp bind(): {}", impl::socket_get_error());
                impl::socket_close(sock_fd);
                continue;
            }
        } else {
            struct sockaddr_in address {};
            memset(&address, 0, sizeof(address));
            address.sin_family = AF_INET;
            address.sin_addr.s_addr =
                INADDR_ANY;                  // NOLINT (misc-include-cleaner)
            address.sin_port = htons(port);  // NOLINT (misc-include-cleaner)
            if (::bind(sock_fd, reinterpret_cast<struct sockaddr*>(&address),
                       sizeof(address))) {
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

            if (setsockopt(sock_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq,
                           sizeof(mreq))) {
                logger().warn("udp setsockopt(IP_ADD_MEMBERSHIP): {}",
                              impl::socket_get_error());
                impl::socket_close(sock_fd);
                continue;
            }
            // NOLINTEND (misc-include-cleaner)
        }

        // join to multicast group succeeded; set some options and return
        if (impl::socket_set_non_blocking(sock_fd)) {
            logger().warn("udp fcntl(): {}", impl::socket_get_error());
            impl::socket_close(sock_fd);
            continue;
        }

        // NOLINTNEXTLINE (misc-include-cleaner)
        if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<const char*>(&RCVBUF_SIZE),
                       sizeof(RCVBUF_SIZE))) {
            logger().warn("udp setsockopt(SO_RCVBUF): {}",
                          impl::socket_get_error());
            impl::socket_close(sock_fd);
            continue;
        }

        return sock_fd;
    }

    // could not bind() a MTP server socket
    logger().error("failed to bind udp socket");
    return SOCKET_ERROR;
}

// NOLINTNEXTLINE (misc-use-internal-linkage)
SOCKET udp_data_socket(int port) { return mtp_data_socket(port, {}); }

// NOLINTNEXTLINE (misc-use-internal-linkage)
jsoncons::json collect_metadata(SensorHttp& sensor_http, int timeout_sec) {
    // Note, this function throws std::runtime_error if
    // 1. the metadata couldn't be retrieved
    // 2. the sensor is in the INITIALIZING state when timeout is reached
    auto timeout_time =
        chrono::steady_clock::now() + chrono::seconds{timeout_sec};

    std::string status;
    // TODO: can remove this loop when we drop support for FW 2.4
    while (true) {
        if (chrono::steady_clock::now() >= timeout_time) {
            throw std::runtime_error(
                "A timeout occurred while waiting for the sensor to "
                "initialize.");
        }

        status = jsoncons::json::parse(
                     sensor_http.sensor_info(timeout_sec))["status"]
                     .as<std::string>();
        if (status != "INITIALIZING") {
            break;
        }
        std::this_thread::sleep_for(1s);
    }

    std::string user_data = "";
    try {
        user_data = sensor_http.get_user_data(timeout_sec);
    } catch (const std::runtime_error& e) {
        if (strcmp(e.what(),
                   "user data API not supported on this FW version") != 0) {
            throw e;
        }
    }

    try {
        auto metadata =
            jsoncons::json::parse(sensor_http.metadata(timeout_sec));

        metadata["ouster-sdk"]["client_version"] = client_version();
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
        if (fw_version.major >= 3 &&
            metadata["config_params"]["udp_profile_lidar"] == "LEGACY") {
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
        throw std::runtime_error(
            "Cannot obtain full metadata with sensor status: " + status +
            ". Please ensure that sensor is not in a STANDBY, UNCONFIGURED, "
            "WARMUP, or ERROR state");
    }
}

namespace {
bool get_config(SensorHttp& sensor_http, sensor_config& config,
                bool active = true,
                int timeout_sec = LONG_HTTP_REQUEST_TIMEOUT_SECONDS) {
    auto res = sensor_http.get_config_params(active, timeout_sec);
    ValidatorIssues issues;
    if (!parse_and_validate_config(res, config, issues)) {
        throw std::runtime_error(to_string(issues.critical));
    }
    return true;
}
}  // anonymous namespace

bool get_config(const std::string& hostname, sensor_config& config, bool active,
                int timeout_sec) {
    auto sensor_http = SensorHttp::create(hostname, timeout_sec);
    return get_config(*sensor_http, config, active, timeout_sec);
}

// NOLINTNEXTLINE (misc-use-internal-linkage)
bool set_config(SensorHttp& sensor_http, const sensor_config& config,
                uint8_t config_flags, int timeout_sec) {
    // reset staged config to avoid spurious errors
    jsoncons::json config_params =
        jsoncons::json::parse(sensor_http.active_config_params(timeout_sec));
    jsoncons::json config_params_copy = config_params;

    // set all desired config parameters
    jsoncons::json config_json = config_to_json(config);
    for (const auto& it : config_json.object_range()) {
        config_params[it.key()] = it.value();
    }

    if (config_json.contains("operating_mode") &&
        config_params.contains("auto_start_flag")) {
        // we're setting operating mode and this sensor has a FW with
        // auto_start_flag
        config_params["auto_start_flag"] =
            config_json["operating_mode"] == "NORMAL" ? 1 : 0;
    }

    // Signal multiplier changed from int to double for FW 3.0/2.5+, with
    // corresponding change to config.signal_multiplier.
    // Change values 1, 2, 3 back to ints to support older FWs
    if (config_json.contains("signal_multiplier")) {
        check_signal_multiplier(
            config_params["signal_multiplier"].as<double>());
        if (config_params["signal_multiplier"].as<double>() != 0.25 &&
            config_params["signal_multiplier"].as<double>() != 0.5) {
            config_params["signal_multiplier"] =
                config_params["signal_multiplier"].as<int>();
        }
    }

    // detect and handle @auto udp dest properly
    if (config.udp_dest == "@auto") {
        config_flags |= CONFIG_UDP_DEST_AUTO;
    }

    // set automatic udp dest, if flag specified
    if ((config_flags & CONFIG_UDP_DEST_AUTO) != 0) {
        if (config.udp_dest && config.udp_dest != "@auto") {
            throw std::invalid_argument(
                "UDP_DEST_AUTO flag set but provided config has udp_dest");
        }
        sensor_http.set_udp_dest_auto(timeout_sec);

        auto staged = jsoncons::json::parse(
            sensor_http.staged_config_params(timeout_sec));

        // now we set config_params according to the staged udp_dest from the
        // sensor
        if (staged.contains("udp_ip")) {  // means the FW version carries udp_ip
            config_params["udp_ip"] = staged["udp_ip"].as<std::string>();
            config_params["udp_dest"] = staged["udp_ip"].as<std::string>();
        } else {  // don't need to worry about udp_ip
            config_params["udp_dest"] = staged["udp_dest"].as<std::string>();
        }
    }

    // if configuration didn't change then skip applying the params
    // note: comparison will fail if config_params contains newer config params
    // introduced after the verison of FW the sensor is on
    if ((config_flags & CONFIG_FORCE_REINIT) != 0 ||
        (config_params_copy != config_params)) {
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

    return true;
}

bool set_config(const std::string& hostname, const sensor_config& config,
                uint8_t config_flags, int timeout_sec) {
    auto sensor_http = SensorHttp::create(hostname, timeout_sec);
    return set_config(*sensor_http, config, config_flags, timeout_sec);
}

std::string get_metadata(client& cli, int timeout_sec) {
    // Note, this function calls functions that throw std::runtime_error
    // on timeout.
    auto sensor_http = SensorHttp::create(cli.hostname, timeout_sec);
    std::string meta;
    try {
        auto temp_data = collect_metadata(*sensor_http, timeout_sec);
        temp_data.dump(meta);
    } catch (const std::exception& e) {
        logger().warn(std::string("Unable to retrieve sensor metadata: ") +
                      e.what());
        throw;
    }
    return meta;
}

bool init_logger(const std::string& log_level, const std::string& log_file_path,
                 bool rotating, int max_size_in_bytes, int max_files) {
    if (log_file_path.empty()) {
        return Logger::instance().configure_stdout_sink(log_level);
    } else {
        return Logger::instance().configure_file_sink(
            log_level, log_file_path, rotating, max_size_in_bytes, max_files);
    }
}

std::shared_ptr<client> init_client(const std::string& hostname, int lidar_port,
                                    int imu_port) {
    logger().info(
        "initializing sensor client: {} expecting lidar port/imu port: {}/{}",
        hostname, lidar_port, imu_port);

    auto cli = std::make_shared<client>();
    cli->hostname = hostname;

    cli->lidar_fd = udp_data_socket(lidar_port);
    cli->imu_fd = udp_data_socket(imu_port);

    if (!impl::socket_valid(cli->lidar_fd) ||
        !impl::socket_valid(cli->imu_fd)) {
        return {};
    }

    return cli;
}

std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode ld_mode, timestamp_mode ts_mode,
                                    int lidar_port, int imu_port,
                                    int timeout_sec, bool persist_config,
                                    OperatingMode operating_mode) {
    auto cli = init_client(hostname, lidar_port, imu_port);

    if (!cli) {
        return {};
    }
    logger().info("(0 means a random port will be chosen)");

    // update requested ports to actual bound ports
    lidar_port = get_sock_port(cli->lidar_fd);
    imu_port = get_sock_port(cli->imu_fd);
    if (!impl::socket_valid(lidar_port) || !impl::socket_valid(imu_port)) {
        return {};
    }

    try {
        auto sensor_http = SensorHttp::create(hostname, timeout_sec);
        sensor::sensor_config config;
        uint8_t config_flags = 0;
        if (udp_dest_host.empty()) {
            config_flags |= CONFIG_UDP_DEST_AUTO;
        } else {
            config.udp_dest = udp_dest_host;
        }
        if (ld_mode) {
            config.lidar_mode = ld_mode;
        }
        if (ts_mode) {
            config.timestamp_mode = ts_mode;
        }
        if (lidar_port) {
            config.udp_port_lidar = lidar_port;
        }
        if (imu_port) {
            config.udp_port_imu = imu_port;
        }
        if (persist_config) {
            config_flags |= CONFIG_PERSIST;
        }
        config.operating_mode = operating_mode;
        set_config(*sensor_http, config, config_flags, timeout_sec);

        // will block until no longer INITIALIZING
        auto meta = collect_metadata(*sensor_http, timeout_sec);
        // check for sensor error states
        std::string status = meta["sensor_info"]["status"].as<std::string>();
        if (status == "ERROR" || status == "UNCONFIGURED") {
            return {};
        }
    } catch (const std::runtime_error& e) {
        // log error message
        logger().error("init_client(): {}", e.what());
        return {};
    }

    return cli;
}

std::shared_ptr<client> mtp_init_client(const std::string& hostname,
                                        const sensor_config& config,
                                        const std::string& mtp_dest_host,
                                        bool main, int timeout_sec,
                                        bool persist_config) {
    int lidar_port = config.udp_port_lidar ? config.udp_port_lidar.value() : 0;
    int imu_port = config.udp_port_imu ? config.udp_port_imu.value() : 0;
    auto udp_dest = config.udp_dest ? config.udp_dest.value() : "";

    logger().info(
        "initializing sensor client: {} expecting ports: {}/{}, multicast "
        "group: {} (0 means a random port will be chosen)",
        hostname, lidar_port, imu_port, udp_dest);

    auto cli = std::make_shared<client>();
    cli->hostname = hostname;

    cli->lidar_fd = mtp_data_socket(lidar_port, {udp_dest}, mtp_dest_host);
    cli->imu_fd = mtp_data_socket(imu_port, {udp_dest}, mtp_dest_host);

    if (!impl::socket_valid(cli->lidar_fd) ||
        !impl::socket_valid(cli->imu_fd)) {
        return {};
    }

    if (main) {
        auto lidar_port = get_sock_port(cli->lidar_fd);
        auto imu_port = get_sock_port(cli->imu_fd);

        sensor_config config_copy{config};
        try {
            auto sensor_http = SensorHttp::create(hostname, timeout_sec);
            uint8_t config_flags = 0;
            if (lidar_port) {
                config_copy.udp_port_lidar = lidar_port;
            }
            if (imu_port) {
                config_copy.udp_port_imu = imu_port;
            }
            if (persist_config) {
                config_flags |= CONFIG_PERSIST;
            }
            config_copy.operating_mode = OPERATING_NORMAL;
            set_config(*sensor_http, config_copy, config_flags, timeout_sec);

            // will block until no longer INITIALIZING
            auto meta = collect_metadata(*sensor_http, timeout_sec);
            // check for sensor error states
            auto status = meta["sensor_info"]["status"].as<std::string>();
            if (status == "ERROR" || status == "UNCONFIGURED") {
                return {};
            }
        } catch (const std::runtime_error& e) {
            // log error message
            logger().error("init_client(): {}", e.what());
            return {};
        }
    }

    return cli;
}

namespace impl {

struct client_poller {
    fd_set rfds;  // NOLINT (misc-include-cleaner)
    SOCKET max_fd;
    client_state err;
};

std::shared_ptr<client_poller> make_poller() {
    return std::make_unique<client_poller>();
}

void reset_poll(client_poller& poller) {
    FD_ZERO(&poller.rfds);  // NOLINT (misc-include-cleaner)
    poller.max_fd = 0;
    poller.err = client_state::TIMEOUT;
}

void set_poll(client_poller& poller, const client& client) {
    FD_SET(client.lidar_fd, &poller.rfds);  // NOLINT (misc-include-cleaner)
    FD_SET(client.imu_fd, &poller.rfds);    // NOLINT (misc-include-cleaner)
    poller.max_fd = std::max({poller.max_fd, client.lidar_fd, client.imu_fd});
}

int poll(client_poller& poller, int timeout_sec) {
    // NOLINTNEXTLINE (misc-include-cleaner)
    timeval timeout_val = {timeout_sec, 0};

    // NOLINTBEGIN (misc-include-cleaner)
    SOCKET retval =
        select((int)poller.max_fd + 1, &poller.rfds, NULL, NULL, &timeout_val);
    // NOLINTEND (misc-include-cleaner)
    if (!impl::socket_valid(retval)) {
        if (impl::socket_exit()) {
            poller.err = client_state::EXIT;
        } else {
            logger().error("select: {}", impl::socket_get_error());
            poller.err = client_state::CLIENT_ERROR;
        }

        return -1;
    }

    return retval;
}

client_state get_error(const client_poller& poller) { return poller.err; }

client_state get_poll(const client_poller& poller, const client& client) {
    auto state = client_state(0);

    // NOLINTBEGIN (misc-include-cleaner)
    if (FD_ISSET(client.lidar_fd, &poller.rfds)) {
        state = client_state(state | LIDAR_DATA);
    }
    if (FD_ISSET(client.imu_fd, &poller.rfds))
        state = client_state(state | IMU_DATA);
    // NOLINTEND (misc-include-cleaner)
    return state;
}

}  // namespace impl

client_state poll_client(const client& client, const int timeout_sec) {
    impl::client_poller poller{};
    impl::reset_poll(poller);
    impl::set_poll(poller, client);
    int res = impl::poll(poller, timeout_sec);
    if (res <= 0) {
        // covers TIMEOUT and error states
        return impl::get_error(poller);
    } else {
        return impl::get_poll(poller, client);
    }
}

namespace {
bool recv_fixed(SOCKET file_desc, void* buf, int64_t len) {
    // Have to read longer than len because you need to know if the packet
    // is too large
    // NOLINTNEXTLINE (misc-include-cleaner)
    int64_t bytes_read = recv(file_desc, static_cast<char*>(buf), len + 1, 0);

    if (bytes_read == len) {
        return true;
    } else if (bytes_read == -1) {
        logger().error("recvfrom: {}", impl::socket_get_error());
    } else {
        logger().warn("Unexpected udp packet length: {}", bytes_read);
    }
    return false;
}
}  // namespace

bool read_lidar_packet(const client& client, uint8_t* buf, size_t bytes) {
    return recv_fixed(client.lidar_fd, buf, bytes);
}

bool read_lidar_packet(const client& client, uint8_t* buf,
                       const packet_format& packet_format) {
    return read_lidar_packet(client, buf, packet_format.lidar_packet_size);
}

// Types.h is included which defines LidarPacket
// NOLINTNEXTLINE (misc-include-cleaner)
bool read_lidar_packet(const client& client, LidarPacket& packet) {
    auto now = std::chrono::system_clock::now();
    packet.host_timestamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch())
            .count();
    return read_lidar_packet(client, packet.buf.data(), packet.buf.size());
}

bool read_imu_packet(const client& cli, uint8_t* buf, size_t bytes) {
    return recv_fixed(cli.imu_fd, buf, static_cast<int64_t>(bytes));
}

bool read_imu_packet(const client& cli, uint8_t* buf,
                     const packet_format& packet_format) {
    return read_imu_packet(cli, buf, packet_format.imu_packet_size);
}

// Types.h is included which defines ImuPacket
// NOLINTNEXTLINE (misc-include-cleaner)
bool read_imu_packet(const client& cli, ImuPacket& packet) {
    auto now = std::chrono::system_clock::now();
    packet.host_timestamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch())
            .count();
    return read_imu_packet(cli, packet.buf.data(), packet.buf.size());
}

int get_lidar_port(const client& cli) { return get_sock_port(cli.lidar_fd); }

int get_imu_port(const client& cli) { return get_sock_port(cli.imu_fd); }

bool in_multicast(const std::string& addr) {
    // NOLINTNEXTLINE (misc-include-cleaner)
    return IN_MULTICAST(ntohl(inet_addr(addr.c_str())));
}

/**
 * Return the socket file descriptor used to listen for lidar UDP data.
 *
 * @param[in] cli client returned by init_client associated with the
 * connection.
 *
 * @return the socket file descriptor.
 */
extern SOCKET get_lidar_socket_fd(client& cli) { return cli.lidar_fd; }

/**
 * Return the socket file descriptor used to listen for imu UDP data.
 *
 * @param[in] cli client returned by init_client associated with the
 * connection.
 *
 * @return the socket file descriptor.
 */
extern SOCKET get_imu_socket_fd(client& cli) { return cli.imu_fd; }

}  // namespace sensor
}  // namespace ouster

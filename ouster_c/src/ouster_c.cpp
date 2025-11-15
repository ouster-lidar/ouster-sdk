// C wrapper implementation
#include "ouster_c.h"

#include <chrono>
#include <cstring>
#include <exception>
#include <memory>
#include <new>
#include <string>
#include <thread>

#include "ouster/cartesian.h"
#include "ouster/client.h"  // init_client, poll_client, read_*_packet, get_metadata
#include "ouster/field.h"
#include "ouster/lidar_scan.h"
#include "ouster/metadata.h"  // parse_and_validate_metadata
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"  // sensor_info, packet_format

struct ouster_client {
    std::shared_ptr<ouster::sensor::client> cli;        // underlying C++ client
    std::string metadata;                               // cached metadata json
    std::unique_ptr<ouster::sensor::sensor_info> info;  // parsed info
    std::unique_ptr<ouster::sensor::packet_format> pf;  // packet format
};

static int set_error_and_return(int code) { return code; }

extern "C" {

ouster_client_t* ouster_client_create(const char* hostname, int lidar_port,
                                      int imu_port) {
    if (!hostname) return nullptr;
    try {
        auto cpp_cli = ouster::sensor::init_client(std::string(hostname),
                                                   lidar_port, imu_port);
        if (!cpp_cli) return nullptr;
        ouster_client_t* handle = new (std::nothrow) ouster_client();
        if (!handle) return nullptr;
        handle->cli = cpp_cli;
        return handle;
    } catch (...) {
        return nullptr;
    }
}

void ouster_client_destroy(ouster_client_t* client) {
    if (!client) return;
    client->pf.reset();
    client->info.reset();
    client->cli.reset();
    delete client;
}

int ouster_client_poll(ouster_client_t* client, int timeout_sec) {
    if (!client || !client->cli) return OU_CLIENT_ERROR;
    try {
        auto state = ouster::sensor::poll_client(*client->cli, timeout_sec);
        return static_cast<int>(state);
    } catch (...) {
        return OU_CLIENT_ERROR;
    }
}

int ouster_client_get_metadata(ouster_client_t* client, char* buffer,
                               size_t capacity) {
    if (!client || !client->cli) return -1;
    try {
        if (client->metadata.empty()) {
            client->metadata = ouster::sensor::get_metadata(*client->cli);
        }
        if (!buffer || capacity == 0) return (int)client->metadata.size();
        size_t to_copy = client->metadata.size() < capacity
                             ? client->metadata.size()
                             : capacity - 1;  // reserve NUL
        std::memcpy(buffer, client->metadata.data(), to_copy);
        buffer[to_copy] = '\0';
        return (int)to_copy;
    } catch (...) {
        return -1;
    }
}

int ouster_client_fetch_and_parse_metadata(ouster_client_t* client,
                                           int timeout_sec) {
    if (!client || !client->cli) return set_error_and_return(-1);
    try {
        client->metadata =
            ouster::sensor::get_metadata(*client->cli, timeout_sec);
        ouster::ValidatorIssues issues;
        nonstd::optional<ouster::sensor::sensor_info> si_opt;
        bool ok = ouster::parse_and_validate_metadata(client->metadata, si_opt,
                                                      issues);
        if (!ok || !si_opt.has_value()) {
            return set_error_and_return(-2);
        }
        client->info =
            std::make_unique<ouster::sensor::sensor_info>(si_opt.value());
        client->pf =
            std::make_unique<ouster::sensor::packet_format>(*client->info);
        return 0;
    } catch (...) {
        return set_error_and_return(-3);
    }
}

int ouster_client_get_frame_dimensions(const ouster_client_t* client,
                                       int* width, int* height) {
    if (!client || !client->info) return -1;
    if (width) *width = static_cast<int>(client->info->w());
    if (height) *height = static_cast<int>(client->info->h());
    return 0;
}

int ouster_client_get_packet_sizes(const ouster_client_t* client,
                                   size_t* lidar_packet_size,
                                   size_t* imu_packet_size) {
    if (!client || !client->pf) return -1;
    if (lidar_packet_size) *lidar_packet_size = client->pf->lidar_packet_size;
    if (imu_packet_size) *imu_packet_size = client->pf->imu_packet_size;
    return 0;
}

int ouster_client_read_lidar_packet(ouster_client_t* client, uint8_t* buf,
                                    size_t buf_size) {
    if (!client || !client->cli || !client->pf || !buf) return 0;
    size_t needed = client->pf->lidar_packet_size;
    if (buf_size < needed) return 0;
    try {
        bool ok = ouster::sensor::read_lidar_packet(*client->cli, buf, needed);
        return ok ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int ouster_client_read_imu_packet(ouster_client_t* client, uint8_t* buf,
                                  size_t buf_size) {
    if (!client || !client->cli || !client->pf || !buf) return 0;
    size_t needed = client->pf->imu_packet_size;
    if (buf_size < needed) return 0;
    try {
        bool ok = ouster::sensor::read_imu_packet(*client->cli, buf, needed);
        return ok ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int ouster_client_get_lidar_port(const ouster_client_t* client) {
    if (!client || !client->cli) return -1;
    try {
        return ouster::sensor::get_lidar_port(*client->cli);
    } catch (...) {
        return -1;
    }
}

int ouster_client_get_imu_port(const ouster_client_t* client) {
    if (!client || !client->cli) return -1;
    try {
        return ouster::sensor::get_imu_port(*client->cli);
    } catch (...) {
        return -1;
    }
}

}  // extern "C"

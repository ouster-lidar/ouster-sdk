#pragma once

#include <json/json.h>
#include "ouster/compat.h"
#include "stdio.h"

namespace ouster {
namespace OS1 {
struct client {
    SOCKET lidar_fd;
    SOCKET imu_fd;
    std::string hostname;
    Json::Value meta;
    ~client() {
        socket_close(lidar_fd);
        socket_close(imu_fd);
    }
};
}
}

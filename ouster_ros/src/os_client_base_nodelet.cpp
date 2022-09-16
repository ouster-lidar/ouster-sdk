/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_client_base_nodelet.cpp
 * @brief Implementatin of OusterClientBase
 */

#include "ouster_ros/os_client_base_nodelet.h"

#include "ouster/impl/build.h"
#include "ouster_ros/GetMetadata.h"

namespace sensor = ouster::sensor;
using ouster_ros::GetMetadata;

namespace nodelets_os {

void OusterClientBase::onInit() {
    auto& nh = getNodeHandle();
    get_metadata_srv =
        nh.advertiseService<GetMetadata::Request, GetMetadata::Response>(
            "get_metadata",
            [this](GetMetadata::Request&, GetMetadata::Response& res) {
                res.metadata = cached_metadata;
                return cached_metadata.size() > 0;
            });

    NODELET_INFO("get_metadata service created");
}

void OusterClientBase::display_lidar_info(const sensor::sensor_info& info) {
    NODELET_WARN("Client version: %s", ouster::SDK_VERSION_FULL);
    NODELET_WARN("Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
    NODELET_WARN("%s sn: %s firmware rev: %s", info.prod_line.c_str(),
                 info.sn.c_str(), info.fw_rev.c_str());
}

}  // namespace nodelets_os

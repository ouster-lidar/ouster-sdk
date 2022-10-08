/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_replay_nodelet.cpp
 * @brief This nodelet mainly handles publishing saved metadata
 *
 */

#include <pluginlib/class_list_macros.h>

#include "ouster_ros/os_client_base_nodelet.h"

namespace sensor = ouster::sensor;

namespace nodelets_os {

class OusterReplay : public OusterClientBase {
   private:
    virtual void onInit() override {
        auto& pnh = getPrivateNodeHandle();
        auto meta_file = pnh.param("metadata", std::string{});
        if (!meta_file.size()) {
            NODELET_ERROR("Must specify metadata file in replay mode");
            throw std::runtime_error("metadata no specificed");
        }

        NODELET_INFO("Running in replay mode");

        // populate info for config service
        try {
            info = sensor::metadata_from_json(meta_file);
            cached_metadata = to_string(info);
            display_lidar_info(info);
        } catch (const std::runtime_error& e) {
            NODELET_ERROR("Error when running in replay mode: %s", e.what());
        }

        OusterClientBase::onInit();
    }
};

}  // namespace nodelets_os

PLUGINLIB_EXPORT_CLASS(nodelets_os::OusterReplay, nodelet::Nodelet)
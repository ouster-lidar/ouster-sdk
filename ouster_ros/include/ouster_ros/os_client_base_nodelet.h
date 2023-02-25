/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Base class for ouster_ros sensor and replay nodelets
 *
 */

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "ouster/types.h"

namespace nodelets_os {

class OusterClientBase : public nodelet::Nodelet {
   protected:
    virtual void onInit() override;

   protected:
    void display_lidar_info(const ouster::sensor::sensor_info& info);

   protected:
    ouster::sensor::sensor_info info;
    ros::ServiceServer get_metadata_srv;
    std::string cached_metadata;
};

}  // namespace nodelets_os
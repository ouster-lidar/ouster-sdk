/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Example node to publish raw sensor output on ROS topics
 *
 * ROS Parameters
 * sensor_hostname: hostname or IP in dotted decimal form of the sensor
 * udp_dest: hostname or IP where the sensor will send data packets
 * lidar_port: port to which the sensor should send lidar data
 * imu_port: port to which the sensor should send imu data
 */

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "ouster/impl/build.h"
#include "ouster/client.h"
#include "ouster/types.h"
#include "ouster_ros/ros.h"

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
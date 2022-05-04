/**
 * Copyright (c) 2019, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Example node to publish point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using Cloud = ouster_ros::Cloud;
using Point = ouster_ros::Point;
namespace sensor = ouster::sensor;

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_cloud_node");
    ros::NodeHandle nh("~");

    auto tf_prefix = nh.param("tf_prefix", std::string{});
    if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
    auto sensor_frame = tf_prefix + "os_sensor";
    auto imu_frame = tf_prefix + "os_imu";
    auto lidar_frame = tf_prefix + "os_lidar";

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("os_cloud_node: Calling config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    uint32_t H = info.format.pixels_per_column;
    uint32_t W = info.format.columns_per_frame;
    auto udp_profile_lidar = info.format.udp_profile_lidar;

    const int n_returns =
        (udp_profile_lidar == sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY)
            ? 1
            : 2;
    auto pf = sensor::get_format(info);

    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto img_suffix = [](int ind) {
        if (ind == 0) return std::string();
        return std::to_string(ind + 1);  // need second return to return 2
    };

    auto lidar_pubs = std::vector<ros::Publisher>();
    for (int i = 0; i < n_returns; i++) {
        auto pub = nh.advertise<sensor_msgs::PointCloud2>(
            std::string("points") + img_suffix(i), 10);
        lidar_pubs.push_back(pub);
    }

    auto xyz_lut = ouster::make_xyz_lut(info);

    ouster::LidarScan ls{W, H, udp_profile_lidar};
    Cloud cloud{W, H};

    ouster::ScanBatcher batch(W, pf);

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        if (batch(pm.buf.data(), ls)) {
            auto h = std::find_if(
                ls.headers.begin(), ls.headers.end(), [](const auto& h) {
                    return h.timestamp != std::chrono::nanoseconds{0};
                });
            if (h != ls.headers.end()) {
                for (int i = 0; i < n_returns; i++) {
                    scan_to_cloud(xyz_lut, h->timestamp, ls, cloud, i);
                    lidar_pubs[i].publish(ouster_ros::cloud_to_cloud_msg(
                        cloud, h->timestamp, sensor_frame));
                }
            }
        }
    };

    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::packet_to_imu_msg(p, imu_frame, pf));
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}

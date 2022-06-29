/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_cloud_nodelet.cpp
 * @brief A nodelet to publish point clouds and imu topics
 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
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
#include "ouster_ros/GetMetadata.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

namespace sensor = ouster::sensor;
using ouster_ros::PacketMsg;
using sensor::UDPProfileLidar;
using namespace std::chrono_literals;

namespace nodelets_os {
class OusterCloud : public nodelet::Nodelet {
   private:
    virtual void onInit() override {
        auto& pnh = getPrivateNodeHandle();

        auto tf_prefix = pnh.param("tf_prefix", std::string{});
        if (!tf_prefix.empty() && tf_prefix.back() != '/')
            tf_prefix.append("/");
        sensor_frame = tf_prefix + "os_sensor";
        imu_frame = tf_prefix + "os_imu";
        lidar_frame = tf_prefix + "os_lidar";

        auto& nh = getNodeHandle();
        ouster_ros::GetMetadata metadata{};
        auto client = nh.serviceClient<ouster_ros::GetMetadata>("get_metadata");
        client.waitForExistence();
        if (!client.call(metadata)) {
            NODELET_ERROR("Calling get_metadata service failed");
            throw std::runtime_error("Calling get_metadata service failed");
        }

        NODELET_INFO("OusterCloud: retrieved sensor metadata!");

        info = sensor::parse_metadata(metadata.response.metadata);
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;
        auto udp_profile_lidar = info.format.udp_profile_lidar;

        n_returns = udp_profile_lidar ==
                            UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
                        ? 2
                        : 1;

        NODELET_INFO_STREAM("Profile has " << n_returns << " return(s)");

        imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

        auto img_suffix = [](int ind) {
            if (ind == 0) return std::string();
            return std::to_string(ind + 1);  // need second return to return 2
        };

        lidar_pubs.resize(n_returns);
        for (int i = 0; i < n_returns; i++) {
            auto pub = nh.advertise<sensor_msgs::PointCloud2>(
                std::string("points") + img_suffix(i), 10);
            lidar_pubs[i] = pub;
        }

        xyz_lut = ouster::make_xyz_lut(info);

        ls = ouster::LidarScan{W, H, udp_profile_lidar};
        cloud = ouster_ros::Cloud{W, H};

        scan_batcher = std::make_unique<ouster::ScanBatcher>(info);

        lidar_packet_sub = nh.subscribe<PacketMsg>(
            "lidar_packets", 2048, &OusterCloud::lidar_handler, this);
        imu_packet_sub = nh.subscribe<PacketMsg>(
            "imu_packets", 100, &OusterCloud::imu_handler, this);

        // publish transforms
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame));

        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame));
    }

    void lidar_handler(const PacketMsg::ConstPtr& packet) {
        if ((*scan_batcher)(packet->buf.data(), ls)) {
            auto h = std::find_if(
                ls.headers.begin(), ls.headers.end(), [](const auto& h) {
                    return h.timestamp != 0ns;
                });
            if (h != ls.headers.end()) {
                for (int i = 0; i < n_returns; i++) {
                    scan_to_cloud(xyz_lut, h->timestamp, ls, cloud, i);
                    sensor_msgs::PointCloud2 pc =
                        ouster_ros::cloud_to_cloud_msg(cloud, h->timestamp,
                                                       sensor_frame);
                    sensor_msgs::PointCloud2Ptr pc_ptr =
                        boost::make_shared<sensor_msgs::PointCloud2>(pc);
                    lidar_pubs[i].publish(pc_ptr);
                }
            }
        }
    }

    void imu_handler(const PacketMsg::ConstPtr& packet) {
        auto pf = sensor::get_format(info);
        sensor_msgs::Imu imu_msg =
            ouster_ros::packet_to_imu_msg(*packet, imu_frame, pf);
        sensor_msgs::ImuPtr imu_msg_ptr =
            boost::make_shared<sensor_msgs::Imu>(imu_msg);
        imu_pub.publish(imu_msg_ptr);
    };

   private:
    ros::Subscriber lidar_packet_sub;
    std::vector<ros::Publisher> lidar_pubs;
    ros::Subscriber imu_packet_sub;
    ros::Publisher imu_pub;

    sensor::sensor_info info;
    int n_returns = 0;

    ouster::XYZLut xyz_lut;
    ouster::LidarScan ls;
    ouster_ros::Cloud cloud;
    std::unique_ptr<ouster::ScanBatcher> scan_batcher;

    std::string sensor_frame;
    std::string imu_frame;
    std::string lidar_frame;

    tf2_ros::StaticTransformBroadcaster tf_bcast;
};
}  // namespace nodelets_os

PLUGINLIB_EXPORT_CLASS(nodelets_os::OusterCloud, nodelet::Nodelet)

/**
 * @file
 * @brief Example node to publish OS-1 point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>

#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using CloudOS1 = ouster_ros::OS1::CloudOS1;
using PointOS1 = ouster_ros::OS1::PointOS1;

namespace OS1 = ouster::OS1;

int main(int argc, char** argv) {
    ros::init(argc, argv, "os1_cloud_node");
    ros::NodeHandle nh("~");

    auto tf_prefix = nh.param("tf_prefix", std::string{});
    if (!tf_prefix.empty() && tf_prefix.back() != '/')
        tf_prefix.append("/");
    auto sensor_frame = tf_prefix + "os1_sensor";
    auto imu_frame = tf_prefix + "os1_imu";
    auto lidar_frame = tf_prefix + "os1_lidar";

    ouster_ros::OS1ConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OS1ConfigSrv>("os1_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling os1 config service failed");
        return EXIT_FAILURE;
    }

    uint32_t H = OS1::pixels_per_column;
    uint32_t W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string(cfg.response.lidar_mode));

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto xyz_lut = OS1::make_xyz_lut(W, H, cfg.response.beam_azimuth_angles,
                                     cfg.response.beam_altitude_angles);

    CloudOS1 cloud{W, H};
    auto it = cloud.begin();
    sensor_msgs::PointCloud2 msg{};

    auto batch_and_publish = OS1::batch_to_iter<CloudOS1::iterator>(
        xyz_lut, W, H, {}, &PointOS1::make,
        [&](uint64_t scan_ts) mutable {
            msg = ouster_ros::OS1::cloud_to_cloud_msg(
                cloud, std::chrono::nanoseconds{scan_ts}, lidar_frame);
            lidar_pub.publish(msg);
        });

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        batch_and_publish(pm.buf.data(), it);
    };

    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::OS1::packet_to_imu_msg(p, imu_frame));
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}

/**
 * Example node to publish OS1 output on ROS topics
 *
 * Additionally, this node can be used to record and replay raw sesnor output
 * by publishing and listening to PacketMsg topics
 *
 * ROS Parameters
 * scan_dur_ns: nanoseconds to batch lidar packets before publishing a cloud
 * os1_hostname: hostname or IP in dotted decimal form of the sensor
 * os1_udp_dest: hostname or IP where the sensor will send data packets
 * os1_lidar_port: port to which the sensor should send lidar data
 * os1_imu_port: port to which the sensor should send imu data
 * replay_mode: when true, the node will listen on ~/lidar_packets and
 *   ~/imu_packets for data instead of attempting to connect to a sensor
 */

#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"

using ns = std::chrono::nanoseconds;
using PacketMsg = ouster_ros::PacketMsg;

int main(int argc, char** argv) {
    ros::init(argc, argv, "os1_node");
    ros::NodeHandle nh("~");

    auto scan_dur = ns(nh.param("scan_dur_ns", 100000000));
    auto os1_hostname = nh.param("os1_lidar_address", std::string("localhost"));
    auto os1_udp_dest = nh.param("pc_address", std::string("192.168.1.1"));
    auto os1_lidar_port = nh.param("os1_lidar_port", -1);
    auto os1_imu_port = nh.param("os1_imu_port", -1);
    auto replay_mode = nh.param("replay", true);
    auto points_topic_name = nh.param("points_topic_name", std::string("/points_raw"));
    auto imu_topic_name = nh.param("imu_topic_name", std::string("/imu_raw"));
    auto lidar_frame_name = nh.param("lidar_frame_name", std::string("/os1"));
    auto imu_frame_name = nh.param("imu_frame_name", std::string("/os1_imu"));
    auto mode_xyzir = nh.param("mode_xyzir", false);
    auto operation_mode = nh.param("operation_mode", 1);
    auto pulse_mode = nh.param("pulse_mode", 0);
    auto window_rejection = nh.param("window_rejection", true);
    
    /**
     * @note Added to support Velodyne compatible pointcloud format for Autoware
     */
    //defines the pointcloud mode
    ouster_ros::OS1::set_point_mode(mode_xyzir);
    //----------------
    /**
     * @note Added to support advanced mode parameters configuration for Autoware
     */
    //defines the advanced parameters
    ouster::OS1::set_advanced_params((ouster::OS1::OperationMode)operation_mode, (ouster::OS1::PulseMode)pulse_mode, window_rejection);
    auto queue_size = 10;
    if ((ouster::OS1::OperationMode)operation_mode == ouster::OS1::MODE_512x20 || (ouster::OS1::OperationMode)operation_mode == ouster::OS1::MODE_1024x20) {
    	queue_size = 20;
    	scan_dur = scan_dur / 2; //scan duration should be smaller at faster frame rates
    }
    //----------------

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(points_topic_name, queue_size);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic_name, queue_size);

    auto lidar_handler = ouster_ros::OS1::batch_packets(
        scan_dur, [&](ns scan_ts, const ouster_ros::OS1::CloudOS1& cloud) {
            lidar_pub.publish(
                ouster_ros::OS1::cloud_to_cloud_msg(cloud, scan_ts, lidar_frame_name));
        });

    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::OS1::packet_to_imu_msg(p, imu_frame_name));
    };

    if (replay_mode) {
        auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
            "lidar_packets", 500, lidar_handler);
        auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
            "imu_packets", 500, imu_handler);
        ros::spin();
    } else {
        auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 500);
        auto imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 500);

        auto cli = ouster::OS1::init_client(os1_hostname, os1_udp_dest,
                                            os1_lidar_port, os1_imu_port);
        if (!cli) {
            ROS_ERROR("Failed to initialize sensor at: %s", os1_hostname.c_str());
            return 1;
        }

        ouster_ros::OS1::spin(*cli,
                              [&](const PacketMsg& pm) {
                                  lidar_packet_pub.publish(pm);
                                  lidar_handler(pm);
                              },
                              [&](const PacketMsg& pm) {
                                  imu_packet_pub.publish(pm);
                                  imu_handler(pm);
                              });
    }
    return 0;
}

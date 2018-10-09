/**
 * Higher-level functions to read data from the OS1 as ROS messages
 */

#pragma once

#include <chrono>
#include <functional>
#include <string>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "ouster_ros/PacketMsg.h"
#include "ouster/os1.h"
#include "ouster_ros/point_os1.h"

namespace ouster_ros {
namespace OS1 {

using CloudOS1 = pcl::PointCloud<PointOS1>;
using ns = std::chrono::nanoseconds;

/**
 * Read an imu packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param cli the OS1 client
 * @param pm the destination packet message
 * @return whether reading was successful
 */
bool read_imu_packet(const ouster::OS1::client& cli, PacketMsg& pm);

/**
 * Read a lidar packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param cli the OS1 client
 * @param pm the destination packet message
 * @return whether reading was successful
 */
bool read_lidar_packet(const ouster::OS1::client& cli, PacketMsg& pm);

/**
 * Read a timestamp from an imu packet message
 * @param cli the OS1 client
 * @param pm packet message populated by read_imu_packet
 * @returns timestamp in nanoseconds
 */
ns timestamp_of_imu_packet(const PacketMsg& pm);

/**
 * Read a timestamp from a lidar packet message
 * @param pm packet message populated by read_lidar_packet
 * @returns timestamp in nanoseconds
 */
ns timestamp_of_lidar_packet(const PacketMsg& pm);

/**
 * Parse an imu packet message into a ROS imu message
 * @param pm packet message populated by read_imu_packet
 * @returns ROS sensor message with fields populated from the OS1 packet
 */
sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& pm);

/**
 * Accumulate points from a lidar packet message into a PCL point cloud. All
 * points are timestamped relative to scan_start_ts as a fraction of
 * scan_duration, a float usually [0.0, 1.0)
 * @param scan_start_ts the timestamp of the beginning of the batch
 * @param scan_duration length of a scan used to compute point timestamps
 * @param pm packet message populated by read_lidar_packet
 * @param cloud PCL point cloud of PointOS1s to accumulate
 */
void add_packet_to_cloud(ns scan_start_ts, ns scan_duration,
                         const PacketMsg& pm, CloudOS1& cloud);

/**
 * Serialize a PCL point cloud to a ROS message
 * @param cloud the PCL point cloud to convert
 * @param timestamp the timestamp to give the resulting ROS message
 * @param frame the frame to set in the resulting ROS message
 * @returns a ROS message containing the point cloud. Can be deserialized with
 * fromROSMsg in pcl_conversions
 */
sensor_msgs::PointCloud2 cloud_to_cloud_msg(const CloudOS1& cloud, ns timestamp,
                                            const std::string& frame = "os1");
/**
 * Loop reading from the OS1 client and invoking callbacks with packet messages.
 * Returns when ROS exits. Also runs the ROS event loop via ros::spinOnce().
 * @param cli the OS1 client
 * @param lidar_handler callback invoked with messages populated by
 * read_lidar_packet
 * @param imu_handler callback invoked with messages populated by
 * read_imu_packet
 */
void spin(const ouster::OS1::client& cli,
          const std::function<void(const PacketMsg& pm)>& lidar_handler,
          const std::function<void(const PacketMsg& pm)>& imu_handler);

/**
 * Construct a function that will batch packets into a PCL point cloud for the
 * given duration, then invoke f with the complete cloud.
 * @param scan_dur duration to batch packets in nanoseconds
 * @param f a callback taking a PCL pointcloud to invoke on a complete batch
 * @returns a function that batches packet messages populated by
 * read_lidar_packet and invokes f on the result
 */
std::function<void(const PacketMsg&)> batch_packets(
    ns scan_dur, const std::function<void(ns, const CloudOS1&)>& f);
}
}

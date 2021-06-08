#include "ouster_ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>

#include <cassert>
#include <chrono>
#include <string>
#include <vector>

#include "ouster/types.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;

bool read_imu_packet(const sensor::client& cli, PacketMsg& m,
                     const sensor::packet_format& pf) {
    m.buf.resize(pf.imu_packet_size + 1);
    return read_imu_packet(cli, m.buf.data(), pf);
}

bool read_lidar_packet(const sensor::client& cli, PacketMsg& m,
                       const sensor::packet_format& pf) {
    m.buf.resize(pf.lidar_packet_size + 1);
    return read_lidar_packet(cli, m.buf.data(), pf);
}

sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& p, const std::string& frame,
                                   const sensor::packet_format& pf) {
    const double standard_g = 9.80665;
    sensor_msgs::Imu m;
    const uint8_t* buf = p.buf.data();

    m.header.stamp.fromNSec(pf.imu_gyro_ts(buf));
    m.header.frame_id = frame;

    m.orientation.x = 0;
    m.orientation.y = 0;
    m.orientation.z = 0;
    m.orientation.w = 0;

    m.linear_acceleration.x = pf.imu_la_x(buf) * standard_g;
    m.linear_acceleration.y = pf.imu_la_y(buf) * standard_g;
    m.linear_acceleration.z = pf.imu_la_z(buf) * standard_g;

    m.angular_velocity.x = pf.imu_av_x(buf) * M_PI / 180.0;
    m.angular_velocity.y = pf.imu_av_y(buf) * M_PI / 180.0;
    m.angular_velocity.z = pf.imu_av_z(buf) * M_PI / 180.0;

    for (int i = 0; i < 9; i++) {
        m.orientation_covariance[i] = -1;
        m.angular_velocity_covariance[i] = 0;
        m.linear_acceleration_covariance[i] = 0;
    }
    for (int i = 0; i < 9; i += 4) {
        m.linear_acceleration_covariance[i] = 0.01;
        m.angular_velocity_covariance[i] = 6e-4;
    }

    return m;
}

void scan_to_cloud(const ouster::XYZLut& xyz_lut,
                   ouster::LidarScan::ts_t scan_ts, const ouster::LidarScan& ls,
                   ouster_ros::Cloud& cloud) {
    cloud.resize(ls.w * ls.h);
    auto points = ouster::cartesian(ls, xyz_lut);

    for (auto u = 0; u < ls.h; u++) {
        for (auto v = 0; v < ls.w; v++) {
            const auto xyz = points.row(u * ls.w + v);
            const auto pix = ls.data.row(u * ls.w + v);
            const auto ts = (ls.header(v).timestamp - scan_ts).count();
            cloud(v, u) = ouster_ros::Point{
                {{static_cast<float>(xyz(0)), static_cast<float>(xyz(1)),
                  static_cast<float>(xyz(2)), 1.0f}},
                static_cast<float>(pix(ouster::LidarScan::INTENSITY)),
                static_cast<uint32_t>(ts),
                static_cast<uint16_t>(pix(ouster::LidarScan::REFLECTIVITY)),
                static_cast<uint8_t>(u),
                static_cast<uint16_t>(pix(ouster::LidarScan::AMBIENT)),
                static_cast<uint32_t>(pix(ouster::LidarScan::RANGE))};
        }
    }
}

sensor_msgs::PointCloud2 cloud_to_cloud_msg(const Cloud& cloud, ns timestamp,
                                            const std::string& frame) {
    sensor_msgs::PointCloud2 msg{};
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame;
    msg.header.stamp.fromNSec(timestamp.count());
    return msg;
}

geometry_msgs::TransformStamped transform_to_tf_msg(
    const ouster::mat4d& mat, const std::string& frame,
    const std::string& child_frame) {
    Eigen::Affine3d aff;
    aff.linear() = mat.block<3, 3>(0, 0);
    aff.translation() = mat.block<3, 1>(0, 3) * 1e-3;

    geometry_msgs::TransformStamped msg = tf2::eigenToTransform(aff);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame;
    msg.child_frame_id = child_frame;

    return msg;
}
}  // namespace ouster_ros

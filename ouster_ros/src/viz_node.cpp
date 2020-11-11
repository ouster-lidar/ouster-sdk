/**
 * Example node to visualize ouster lidar data
 */

#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <iterator>
#include <thread>
#include <utility>

#include "ouster/lidar_scan.h"
#include "ouster/lidar_scan_viz.h"
#include "ouster/packet.h"
#include "ouster/point_viz.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

namespace viz = ouster::viz;
namespace sensor = ouster::sensor;
using PacketMsg = ouster_ros::PacketMsg;

int main(int argc, char** argv) {
    ros::init(argc, argv, "viz_node");
    ros::NodeHandle nh("~");

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    const size_t H = info.format.pixels_per_column;
    const size_t W = info.format.columns_per_frame;

    auto packet_format = sensor::get_format(info);

    auto xyz_lut = ouster::make_xyz_lut(info);

    viz::PointViz point_viz(
        {viz::CloudSetup{
            xyz_lut.direction.data(), xyz_lut.offset.data(),
            info.format.columns_per_frame * info.format.pixels_per_column,
            info.format.columns_per_frame, info.extrinsic.data()}},
        "Ouster Viz (ROS)");

    viz::LidarScanViz lidar_scan_viz(info, point_viz);

    ouster::LidarScan ls(W, H);

    auto batch_and_display = sensor::batch_to_scan(
        W, packet_format,
        [&](std::chrono::nanoseconds) mutable { lidar_scan_viz.draw(ls); });

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        batch_and_display(pm.buf.data(), ls);
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);

    ros::spin();

    return EXIT_SUCCESS;
}

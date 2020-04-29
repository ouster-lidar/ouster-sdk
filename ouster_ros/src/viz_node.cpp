/**
 * Example node to visualize OS1 lidar data
 *
 * ROS Parameters
 * lidar_mode: width of lidar scan - either 512, 1024 (default) or 2048
 */

#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <iterator>
#include <thread>
#include <utility>

#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster/viz.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/os1_ros.h"
#include "ouster_ros/point_os1.h"

using CloudOS1 = ouster_ros::OS1::CloudOS1;
using PointOS1 = ouster_ros::OS1::PointOS1;

namespace viz = ouster::viz;
namespace OS1 = ouster::OS1;

int main(int argc, char** argv) {
    ros::init(argc, argv, "viz_node");
    ros::NodeHandle nh("~");

    ouster_ros::OS1ConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OS1ConfigSrv>("os1_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling os1 config service failed");
        return EXIT_FAILURE;
    }

    auto H = OS1::pixels_per_column;
    auto W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string(cfg.response.lidar_mode));
    const auto xyz_lut = ouster::make_xyz_lut(
        W, H, OS1::range_unit, cfg.response.beam_azimuth_angles,
        cfg.response.beam_altitude_angles);

    auto range_radii =
        nh.param<std::vector<double>>("range_radii", viz::default_range_radii);

    auto vh =
        viz::init_viz(W, H, &xyz_lut, cfg.response.prod_line, range_radii);
    auto ls = std::unique_ptr<ouster::LidarScan>{new ouster::LidarScan(W, H)};

    auto cloud_handler = [&](const CloudOS1& cloud) {
        if ((int)cloud.size() != W * H) {
            ROS_ERROR("Unexpected cloud size; check lidar_mode");
            ros::shutdown();
        }

        std::transform(
            cloud.begin(), cloud.end(), ls->begin(), [](const PointOS1& p) {
                return ouster::LidarScan::pixel(
                    static_cast<ouster::LidarScan::index_t>(p.ring),
                    static_cast<ouster::LidarScan::index_t>(0),
                    std::chrono::nanoseconds(p.t), std::chrono::nanoseconds(0),
                    static_cast<ouster::LidarScan::raw_t>(p.range),
                    static_cast<ouster::LidarScan::raw_t>(p.intensity),
                    static_cast<ouster::LidarScan::raw_t>(p.noise),
                    static_cast<ouster::LidarScan::raw_t>(p.reflectivity));
            });

        viz::update(*vh, ls);
    };

    auto pc_sub =
        nh.subscribe<CloudOS1, const CloudOS1&>("points", 10, cloud_handler);

    std::thread render([&] {
        viz::run_viz(*vh);
        ros::shutdown();
    });

    ros::spin();
    viz::shutdown(*vh);
    render.join();
    return EXIT_SUCCESS;
}

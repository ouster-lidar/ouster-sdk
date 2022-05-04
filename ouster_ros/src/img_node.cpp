/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Example node to visualize range, near ir and signal images
 *
 * Publishes ~/range_image, ~/nearir_image, and ~/signal_image.  Please bear
 * in mind that there is rounding/clamping to display 8 bit images. For computer
 * vision applications, use higher bit depth values in /os_cloud_node/points
 */

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ouster/client.h"
#include "ouster/image_processing.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/ros.h"

namespace sensor = ouster::sensor;
namespace viz = ouster::viz;

using pixel_type = uint16_t;
const size_t pixel_value_max = std::numeric_limits<pixel_type>::max();

sensor_msgs::ImagePtr make_image_msg(size_t H, size_t W,
                                     const ros::Time& stamp) {
    sensor_msgs::ImagePtr msg{new sensor_msgs::Image{}};
    msg->width = W;
    msg->height = H;
    msg->step = W * sizeof(pixel_type);
    msg->encoding = sensor_msgs::image_encodings::MONO16;
    msg->data.resize(W * H * sizeof(pixel_type));
    msg->header.stamp = stamp;

    return msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "img_node");
    ros::NodeHandle nh("~");

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("img_node: Calling os config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    size_t H = info.format.pixels_per_column;
    size_t W = info.format.columns_per_frame;

    auto udp_profile_lidar = info.format.udp_profile_lidar;
    const int n_returns =
        (udp_profile_lidar == sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY)
            ? 1
            : 2;

    const auto& px_offset = info.format.pixel_shift_by_row;

    std::vector<ros::Publisher> img_pubs;
    std::vector<viz::AutoExposure> aes;

    ros::Publisher nearir_image_pub =
        nh.advertise<sensor_msgs::Image>("nearir_image", 100);

    std::vector<ros::Publisher> range_image_pubs;
    std::vector<ros::Publisher> signal_image_pubs;
    std::vector<ros::Publisher> reflec_image_pubs;

    auto topic = [](auto base, int ind) {
        if (ind == 0) return std::string(base);
        return std::string(base) +
               std::to_string(ind + 1);  // need second return to return 2
    };
    for (int i = 0; i < n_returns; i++) {
        ros::Publisher range_image_pub =
            nh.advertise<sensor_msgs::Image>(topic("range_image", i), 100);
        range_image_pubs.push_back(range_image_pub);

        ros::Publisher signal_image_pub =
            nh.advertise<sensor_msgs::Image>(topic("signal_image", i), 100);
        signal_image_pubs.push_back(signal_image_pub);

        ros::Publisher reflec_image_pub =
            nh.advertise<sensor_msgs::Image>(topic("reflec_image", i), 100);
        reflec_image_pubs.push_back(reflec_image_pub);
    }

    ouster_ros::Cloud cloud{};

    viz::AutoExposure nearir_ae, signal_ae, reflec_ae;
    viz::BeamUniformityCorrector nearir_buc;

    sensor_msgs::ImagePtr nearir_image;

    auto base_cloud_handler = [&](const sensor_msgs::PointCloud2::ConstPtr& m,
                                  int return_index) {
        pcl::fromROSMsg(*m, cloud);

        auto range_image = make_image_msg(H, W, m->header.stamp);
        auto signal_image = make_image_msg(H, W, m->header.stamp);
        auto reflec_image = make_image_msg(H, W, m->header.stamp);
        nearir_image = make_image_msg(H, W, m->header.stamp);

        ouster::img_t<float> nearir_image_eigen(H, W);
        ouster::img_t<float> signal_image_eigen(H, W);
        ouster::img_t<float> reflec_image_eigen(H, W);

        // views into message data
        auto range_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)range_image->data.data(), H, W);
        auto signal_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)signal_image->data.data(), H, W);
        auto reflec_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)reflec_image->data.data(), H, W);
        auto nearir_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)nearir_image->data.data(), H, W);

        // copy data out of Cloud message, with destaggering
        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                const size_t vv = (v + W - px_offset[u]) % W;
                const auto& pt = cloud[u * W + vv];

                // 16 bit img: use 4mm resolution and throw out returns >
                // 260m
                auto r = (pt.range + 0b10) >> 2;
                range_image_map(u, v) = r > pixel_value_max ? 0 : r;

                signal_image_eigen(u, v) = pt.intensity;
                reflec_image_eigen(u, v) = pt.reflectivity;
                nearir_image_eigen(u, v) = pt.ambient;
            }
        }

        const bool first = (return_index == 0);

        signal_ae(signal_image_eigen, first);
        reflec_ae(reflec_image_eigen, first);
        nearir_buc(nearir_image_eigen);
        nearir_ae(nearir_image_eigen, first);
        nearir_image_eigen = nearir_image_eigen.sqrt();
        signal_image_eigen = signal_image_eigen.sqrt();

        // copy data into image messages
        signal_image_map =
            (signal_image_eigen * pixel_value_max).cast<pixel_type>();
        reflec_image_map =
            (reflec_image_eigen * pixel_value_max).cast<pixel_type>();
        if (first)
            nearir_image_map =
                (nearir_image_eigen * pixel_value_max).cast<pixel_type>();

        // publish at return index
        range_image_pubs[return_index].publish(range_image);
        signal_image_pubs[return_index].publish(signal_image);
        reflec_image_pubs[return_index].publish(reflec_image);
    };

    auto first_cloud_handler =
        [&](const sensor_msgs::PointCloud2::ConstPtr& m) {
            base_cloud_handler(m, 0);
            nearir_image_pub.publish(nearir_image);
        };

    auto second_cloud_handler =
        [&](const sensor_msgs::PointCloud2::ConstPtr& m) {
            base_cloud_handler(m, 1);
        };

    // image processing
    auto pc1_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        topic("points", 0), 100, first_cloud_handler);
    auto pc2_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        topic("points", 1), 100, second_cloud_handler);

    ros::spin();
    return EXIT_SUCCESS;
}

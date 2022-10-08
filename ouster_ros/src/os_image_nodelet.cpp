/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_image_nodelet.cpp
 * @brief A nodelet to decode range, near ir and signal images from ouster
 * point cloud
 *
 * Publishes ~/range_image, ~/nearir_image, and ~/signal_image.  Please bear
 * in mind that there is rounding/clamping to display 8 bit images. For computer
 * vision applications, use higher bit depth values in /os_cloud_node/points
 */

#include "ouster_ros/ros.h"

#include <nodelet/nodelet.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ouster/image_processing.h"
#include "ouster_ros/GetMetadata.h"

namespace sensor = ouster::sensor;
namespace viz = ouster::viz;
using sensor::UDPProfileLidar;

using pixel_type = uint16_t;
const size_t pixel_value_max = std::numeric_limits<pixel_type>::max();

namespace nodelets_os {
class OusterImage : public nodelet::Nodelet {
   private:
    virtual void onInit() override {
        auto& nh = getNodeHandle();

        ouster_ros::GetMetadata metadata{};
        auto client = nh.serviceClient<ouster_ros::GetMetadata>("get_metadata");
        client.waitForExistence();
        if (!client.call(metadata)) {
            auto error_msg = "OusterImage: Calling get_metadata service failed";
            NODELET_ERROR_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }

        NODELET_INFO("OusterImage: retrieved sensor metadata!");

        info = sensor::parse_metadata(metadata.response.metadata);

        const int n_returns =
            info.format.udp_profile_lidar ==
                    UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
                ? 2
                : 1;

        nearir_image_pub =
            nh.advertise<sensor_msgs::Image>("nearir_image", 100);

        auto topic = [](auto base, int ind) {
            if (ind == 0) return std::string(base);
            return std::string(base) +
                   std::to_string(ind + 1);  // need second return to return 2
        };

        ros::Publisher a_pub;
        for (int i = 0; i < n_returns; i++) {
            a_pub =
                nh.advertise<sensor_msgs::Image>(topic("range_image", i), 100);
            range_image_pubs.push_back(a_pub);

            a_pub =
                nh.advertise<sensor_msgs::Image>(topic("signal_image", i), 100);
            signal_image_pubs.push_back(a_pub);

            a_pub =
                nh.advertise<sensor_msgs::Image>(topic("reflec_image", i), 100);
            reflec_image_pubs.push_back(a_pub);
        }

        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;
        cloud = ouster_ros::Cloud{W, H};

        // image processing
        pc1_sub = nh.subscribe<sensor_msgs::PointCloud2>(
            topic("points", 0), 100, &OusterImage::first_cloud_handler, this);

        if (n_returns > 1) {
            pc2_sub = nh.subscribe<sensor_msgs::PointCloud2>(
                topic("points", 1), 100, &OusterImage::second_cloud_handler,
                this);
        }
    }

    void base_cloud_handler(const sensor_msgs::PointCloud2::ConstPtr& m,
                            int return_index) {
        pcl::fromROSMsg(*m, cloud);

        const bool first = (return_index == 0);
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;

        auto range_image = make_image_msg(H, W, m->header.stamp);
        auto signal_image = make_image_msg(H, W, m->header.stamp);
        auto reflec_image = make_image_msg(H, W, m->header.stamp);
        auto nearir_image = make_image_msg(H, W, m->header.stamp);

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

        const auto& px_offset = info.format.pixel_shift_by_row;

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
        if (first) {
            nearir_image_map =
                (nearir_image_eigen * pixel_value_max).cast<pixel_type>();
            nearir_image_pub.publish(nearir_image);
        }

        // publish at return index
        range_image_pubs[return_index].publish(range_image);
        signal_image_pubs[return_index].publish(signal_image);
        reflec_image_pubs[return_index].publish(reflec_image);
    }

    void first_cloud_handler(const sensor_msgs::PointCloud2::ConstPtr& m) {
        base_cloud_handler(m, 0);
    }

    void second_cloud_handler(const sensor_msgs::PointCloud2::ConstPtr& m) {
        base_cloud_handler(m, 1);
    }

    static sensor_msgs::ImagePtr make_image_msg(size_t H, size_t W,
                                                const ros::Time& stamp) {
        auto msg = boost::make_shared<sensor_msgs::Image>();
        msg->width = W;
        msg->height = H;
        msg->step = W * sizeof(pixel_type);
        msg->encoding = sensor_msgs::image_encodings::MONO16;
        msg->data.resize(W * H * sizeof(pixel_type));
        msg->header.stamp = stamp;
        return msg;
    }

   private:
    ros::Publisher nearir_image_pub;
    std::vector<ros::Publisher> range_image_pubs;
    std::vector<ros::Publisher> signal_image_pubs;
    std::vector<ros::Publisher> reflec_image_pubs;

    ros::Subscriber pc1_sub;
    ros::Subscriber pc2_sub;

    sensor::sensor_info info;

    ouster_ros::Cloud cloud;
    viz::AutoExposure nearir_ae, signal_ae, reflec_ae;
    viz::BeamUniformityCorrector nearir_buc;
};
}  // namespace nodelets_os

PLUGINLIB_EXPORT_CLASS(nodelets_os::OusterImage, nodelet::Nodelet)

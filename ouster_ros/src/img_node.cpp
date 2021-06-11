/**
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
        ROS_ERROR("Calling os config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    size_t H = info.format.pixels_per_column;
    size_t W = info.format.columns_per_frame;

    const auto& px_offset = info.format.pixel_shift_by_row;

    ros::Publisher range_image_pub =
        nh.advertise<sensor_msgs::Image>("range_image", 100);
    ros::Publisher nearir_image_pub =
        nh.advertise<sensor_msgs::Image>("nearir_image", 100);
    ros::Publisher signal_image_pub =
        nh.advertise<sensor_msgs::Image>("signal_image", 100);
    ros::Publisher reflec_image_pub =
        nh.advertise<sensor_msgs::Image>("reflec_image", 100);

    ouster_ros::Cloud cloud{};

    viz::AutoExposure nearir_ae, signal_ae, reflec_ae;
    viz::BeamUniformityCorrector nearir_buc;

    ouster::img_t<double> nearir_image_eigen(H, W);
    ouster::img_t<double> signal_image_eigen(H, W);
    ouster::img_t<double> reflec_image_eigen(H, W);

    auto cloud_handler = [&](const sensor_msgs::PointCloud2::ConstPtr& m) {
        pcl::fromROSMsg(*m, cloud);

        auto range_image = make_image_msg(H, W, m->header.stamp);
        auto nearir_image = make_image_msg(H, W, m->header.stamp);
        auto signal_image = make_image_msg(H, W, m->header.stamp);
        auto reflec_image = make_image_msg(H, W, m->header.stamp);

        // views into message data
        auto range_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)range_image->data.data(), H, W);
        auto nearir_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)nearir_image->data.data(), H, W);
        auto signal_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)signal_image->data.data(), H, W);
        auto reflec_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)reflec_image->data.data(), H, W);

        // copy data out of Cloud message, with destaggering
        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                const size_t vv = (v + W - px_offset[u]) % W;
                const auto& pt = cloud[u * W + vv];

                // 16 bit img: use 4mm resolution and throw out returns > 260m
                auto r = (pt.range + 0b10) >> 2;
                range_image_map(u, v) = r > pixel_value_max ? 0 : r;

                nearir_image_eigen(u, v) = pt.ambient;
                signal_image_eigen(u, v) = pt.intensity;
                reflec_image_eigen(u, v) = pt.reflectivity;
            }
        }

        // image processing
        nearir_buc(nearir_image_eigen);
        nearir_ae(nearir_image_eigen);
        signal_ae(signal_image_eigen);
        reflec_ae(reflec_image_eigen);
        nearir_image_eigen = nearir_image_eigen.sqrt();
        signal_image_eigen = signal_image_eigen.sqrt();

        // copy data into image messages
        nearir_image_map =
            (nearir_image_eigen * pixel_value_max).cast<pixel_type>();
        signal_image_map =
            (signal_image_eigen * pixel_value_max).cast<pixel_type>();
        reflec_image_map =
            (reflec_image_eigen * pixel_value_max).cast<pixel_type>();

        // publish
        range_image_pub.publish(range_image);
        nearir_image_pub.publish(nearir_image);
        signal_image_pub.publish(signal_image);
        reflec_image_pub.publish(reflec_image);
    };

    auto pc_sub =
        nh.subscribe<sensor_msgs::PointCloud2>("points", 100, cloud_handler);

    ros::spin();
    return EXIT_SUCCESS;
}

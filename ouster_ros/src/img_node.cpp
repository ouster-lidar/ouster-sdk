/**
 * @file
 * @brief Example node to visualize range, noise and intensity images
 *
 * Publishes ~/range_image, ~/noise_image, and ~/intensity_image.  Please bear
 * in mind that there is rounding/clamping to display 8 bit images. For computer
 * vision applications, use higher bit depth values in /os_cloud_node/points
 */

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ouster/autoexposure.h"
#include "ouster/beam_uniformity.h"
#include "ouster/client.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/ros.h"

namespace sensor = ouster::sensor;
namespace viz = ouster::viz;

using pixel_type = uint8_t;
constexpr size_t bit_depth = 8 * sizeof(pixel_type);
const size_t pixel_value_max = std::numeric_limits<pixel_type>::max();
constexpr double range_multiplier =
    1.0 / 200.0;  // assuming 200 m range typical

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
    ros::Publisher noise_image_pub =
        nh.advertise<sensor_msgs::Image>("noise_image", 100);
    ros::Publisher intensity_image_pub =
        nh.advertise<sensor_msgs::Image>("intensity_image", 100);

    ouster_ros::Cloud cloud{};

    viz::AutoExposure noise_ae, intensity_ae;
    viz::BeamUniformityCorrector noise_buc;

    std::stringstream encoding_ss;
    encoding_ss << "mono" << bit_depth;
    std::string encoding = encoding_ss.str();

    auto cloud_handler = [&](const sensor_msgs::PointCloud2::ConstPtr& m) {
        pcl::fromROSMsg(*m, cloud);

        sensor_msgs::Image range_image;
        sensor_msgs::Image noise_image;
        sensor_msgs::Image intensity_image;

        range_image.width = W;
        range_image.height = H;
        range_image.step = W;
        range_image.encoding = encoding;
        range_image.data.resize(W * H * bit_depth /
                                (8 * sizeof(*range_image.data.data())));
        range_image.header.stamp = m->header.stamp;

        noise_image.width = W;
        noise_image.height = H;
        noise_image.step = W;
        noise_image.encoding = encoding;
        noise_image.data.resize(W * H * bit_depth /
                                (8 * sizeof(*noise_image.data.data())));
        noise_image.header.stamp = m->header.stamp;

        intensity_image.width = W;
        intensity_image.height = H;
        intensity_image.step = W;
        intensity_image.encoding = encoding;
        intensity_image.data.resize(W * H * bit_depth /
                                    (8 * sizeof(*intensity_image.data.data())));
        intensity_image.header.stamp = m->header.stamp;

        Eigen::ArrayXXd noise_image_eigen(H, W);
        Eigen::ArrayXXd intensity_image_eigen(H, W);

        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                const size_t vv = (v + W - px_offset[u]) % W;
                const size_t index = vv * H + u;
                const auto& pt = cloud[index];

                if (pt.range == 0) {
                    reinterpret_cast<pixel_type*>(
                        range_image.data.data())[u * W + v] = 0;
                } else {
                    reinterpret_cast<pixel_type*>(
                        range_image.data.data())[u * W + v] =
                        pixel_value_max -
                        std::min(std::round(pt.range * range_multiplier),
                                 static_cast<double>(pixel_value_max));
                }
                noise_image_eigen(u, v) = pt.noise;
                intensity_image_eigen(u, v) = pt.intensity;
            }
        }

        noise_buc.correct(noise_image_eigen);
        noise_ae(Eigen::Map<Eigen::ArrayXd>(noise_image_eigen.data(), W * H));
        intensity_ae(
            Eigen::Map<Eigen::ArrayXd>(intensity_image_eigen.data(), W * H));
        noise_image_eigen = noise_image_eigen.sqrt();
        intensity_image_eigen = intensity_image_eigen.sqrt();
        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                reinterpret_cast<pixel_type*>(
                    noise_image.data.data())[u * W + v] =
                    noise_image_eigen(u, v) * pixel_value_max;
                reinterpret_cast<pixel_type*>(
                    intensity_image.data.data())[u * W + v] =
                    intensity_image_eigen(u, v) * pixel_value_max;
            }
        }

        range_image_pub.publish(range_image);
        noise_image_pub.publish(noise_image);
        intensity_image_pub.publish(intensity_image);
    };

    auto pc_sub =
        nh.subscribe<sensor_msgs::PointCloud2>("points", 500, cloud_handler);

    ros::spin();
    return EXIT_SUCCESS;
}

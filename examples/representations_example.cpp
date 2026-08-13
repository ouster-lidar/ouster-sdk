/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with 2D and 3D representations of
 * lidar data with the C++ Ouster SDK. Please see the sdk docs at
 * docs.ouster.com for clearer explanations.
 */

// NOLINTBEGIN(cppcoreguidelines-narrowing-conversions)
// Suppress warnings to avoid explicit static_cast<long> calls for size_t values
// in example code - Eigen::Map and array indexing require signed types

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "helpers.h"
#include "ouster/core/impl/build.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/packet_source.h"
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"
#include "ouster/pcap/pcap_frame_set_source.h"
#include "ouster/pcap/pcap_packet_source.h"

// NOLINTNEXTLINE(google-build-using-namespace)
using namespace ouster::sdk;

// clang-format off
//! [docs-stag-x-image-form]
core::img_t<double> get_x_in_image_form(const core::LidarFrame& frame,
                                        const core::SensorInfo& info,
                                        bool destaggered = false) {
    // For convenience, save w and h to variables
    const size_t w = info.format.columns_per_frame;
    const size_t h = info.format.pixels_per_column;

    // Get the XYZ in ouster::Points (n x 3 Eigen array) form
    core::XYZLut xyzlut(info, /*use_extrinsics=*/ false);
    auto cloud = xyzlut(frame);

    // Access x and reshape as needed, not that the values in cloud.col(0) are ordered
    auto x_image = Eigen::Map<const core::img_t<double>>(cloud.col(0).data(), h, w);
    // Apply destagger if desired
    if (!destaggered) {
        return x_image;
    }
    auto x_destaggered = core::destagger<double>(x_image, info.format.pixel_shift_by_row);
    return x_destaggered;
}
//! [docs-etag-x-image-form]
// clang-format on

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM << ")"
                  << "\n\nUsage: representation_example <pcap_file>" << std::endl;
        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string pcap_file = argv[1];

    auto source = ouster::sdk::open_packet_source(pcap_file);
    auto info = *source.sensor_info()[0];

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    auto frame = core::LidarFrame(source.sensor_info()[0]);

    std::cerr << "Reading in frame from pcap..." << std::endl;
    get_complete_frame(source, frame);
    // 1. Getting XYZ
    std::cout << "1. Calculating 3d Points... " << std::endl;
    // clang-format off
    //! [doc-stag-cpp-xyz]
    // sensor::SensorFrameSetSource source(sensor_hostname);
    // auto sensor_info = source.sensor_info()[0];
    // auto frame = LidarFrame(sensor_info);
    auto xyzlut = core::XYZLut(info, /*use_extrinsics=*/ false);
    auto range = frame.field(core::ChanField::RANGE);
    auto cloud = xyzlut(range);
    //! [doc-etag-cpp-xyz]

    std::cerr << "\nLet's see what the 2000th point in this cloud is...  ("
              << cloud(2000, 0) << ", " << cloud(2000, 1) << ", "
              << cloud(2000, 2) << ")" << std::endl;

    // 2. Providing a transfomration to XYZ
    // You can also make an XYZLut by specifying a special transform if you
    // have a different frame you would like to be in, say if you have an
    // extrinsics matrix:
    //! [doc-stag-extrinsics-to-xyzlut]
    core::mat4d transformation = core::mat4d::Identity();

    // Let's turn it upside down and put it on a very tall pole and shift x
    transformation(2, 2) = -1;
    transformation(1, 1) = -1;
    transformation(2, 3) = 20;  // unit is meters
    transformation(0, 3) = 1.5; // unit is meters
    info.sensor_to_body = transformation;
    std::cout
        << "2. Now we will apply this transformation to the look-up table:\n"
        << transformation << std::endl;
    // Remember to apply the lidar_to_sensor_transform if your extrinsics
    // matrix was between sensor coordinate system and some stable point, say a vehicle center
    auto lut_extrinsics = core::XYZLut(info, /*use_extrinsics=*/true);
    std::cout << "Calculating 3d Points of with special transform provided.." << std::endl;
    auto cloud_adjusted = lut_extrinsics(range);
    //! [doc-etag-extrinsics-to-xyzlut]
    // clang-format on

    std::cerr << "And now the 2000th point in the transformed point cloud... ("
              << cloud_adjusted(2000, 0) << ", " << cloud_adjusted(2000, 1) << ", "
              << cloud_adjusted(2000, 2) << ")" << std::endl;

    // 3. Destaggering
    // Fields come in w x h arrays, but they are staggered, so that a column
    // reflects the timestamp. To get each column to make visual sense,
    // destagger the image
    // clang-format off
    //! [doc-stag-destagger]
    std::cout << "\n3. Getting staggered and destaggered images of Reflectivity..." << std::endl;

    auto reflectivity = frame.field(core::ChanField::REFLECTIVITY);
    auto reflectivity_destaggered = core::destagger(info, reflectivity, false);
    //! [doc-etag-destagger]
    // clang-format on

    // 4. You can get XYZ in w x h arrays too
    std::cerr << "4. Getting staggered and destaggered images of X Coordinate..." << std::endl;
    auto x_image_staggered = get_x_in_image_form(frame, info, false);
    auto x_image_destaggered = get_x_in_image_form(frame, info, true);

    const auto print_row = std::min<size_t>(123, h - 3);
    const auto print_column = std::min<size_t>(1507, (w / 2) + 5);

    const std::string point_string =
        "(" + std::to_string(print_row) + ", " + std::to_string(print_column) + ")";

    // Convert Fields to img_t<uint8_t> for to print row, column later
    const uint8_t* data_ptr_1 = reflectivity.get<uint8_t>();
    Eigen::Map<const core::img_t<uint8_t>> reflectivity_img(data_ptr_1, h, w);

    const uint8_t* data_ptr = reflectivity_destaggered.get<uint8_t>();
    Eigen::Map<const core::img_t<uint8_t>> reflectivity_destaggered_img(data_ptr, h, w);

    std::cerr << "In the staggered image, the point at " << point_string << " has reflectivity "
              << static_cast<int>(reflectivity_img(print_row, print_column))
              << " and an x coordinate of " << x_image_staggered(print_row, print_column) << "."
              << std::endl;
    std::cerr << "In the destagged image, the point at " << point_string << " has reflectivity "
              << static_cast<int>(reflectivity_destaggered_img(print_row, print_column))
              << " and an x coordinate of " << x_image_destaggered(print_row, print_column) << "."
              << std::endl;
}
// NOLINTEND(cppcoreguidelines-narrowing-conversions)

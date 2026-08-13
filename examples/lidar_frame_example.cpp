/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the LidarFrame class of the
 * C++ Ouster SDK. Please see the sdk docs at docs.ouster.com for clearer
 * explanations.
 */
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "helpers.h"
#include "ouster/core/impl/build.h"
// NOLINTBEGIN(google-build-using-namespace)
//! [doc-stag-lidarframe-imports]
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/pcap/pcap_frame_set_source.h"
#include "ouster/pcap/pcap_packet_source.h"
using namespace ouster::sdk;
//! [doc-etag-lidarframe-imports]
// NOLINTEND(google-build-using-namespace)

void print_field_dtypes(const std::string& data_path) {
    auto source = open_source(data_path);

    auto it = source.begin();
    if (it == source.end()) {
        std::cout << "No frames available" << std::endl;
        return;
    }

    const auto& frame_set = *it;
    std::cout << "Available fields and corresponding dtype in LidarFrame" << std::endl;
    // clang-format off
    // [doc-stag-pcap-query-frame]
    for (const auto& frame : frame_set) {
        if (!frame) { continue; }
        for (const auto& kv : frame->fields()) {
            const auto& field_type = kv.first;
            const auto& field_tag = core::to_string(kv.second.tag());
            std::cout << std::left << std::setw(15) << field_type << " "
                                                    << field_tag << std::endl;
    // [doc-etag-pcap-query-frame]
            // clang-format on
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM << ")"
                  << "\n\nUsage: lidar_frame_example <pcap_file>" << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string pcap_path = argv[1];

    // clang-format off

    // Specifiying only w and h for a lidar frame creates one using the LEGACY udp
    // profile
    auto source = ouster::sdk::open_packet_source(pcap_path);
    auto sensor_info = source.sensor_info()[0];

    // You can create a LidarFrame by providing the sensor_info
    //! [doc-stag-lidarframe-sensorinfo-constructor]
    auto profile_frame = core::LidarFrame(sensor_info);
    //! [doc-etag-lidarframe-sensorinfo-constructor]

    // Finally, you can construct by specifying fields directly
    //! [doc-stag-lidarframe-reduced-slots]
    static const std::vector<core::FieldType> reduced_fields{
        {{core::ChanField::RANGE, core::ChanFieldType::UINT32},
        {core::ChanField::NEAR_IR, core::ChanFieldType::UINT16}}
    };
    auto reduced_fields_frame = core::LidarFrame(sensor_info, reduced_fields);
    //! [doc-etag-lidarframe-reduced-slots]
    // clang-format on

    std::cerr << "Creating frames from pcap... ";
    get_complete_frame(source, profile_frame);
    get_complete_frame(source, reduced_fields_frame);

    std::cerr << "Frames created!" << std::endl;

    // Headers
    //! [doc-stag-profile-frameid]
    auto frame_id = profile_frame.frame_id;
    //! [doc-etag-profile-frameid]
    //! [doc-stag-lidarframe-cpp-headers]
    auto timestamp = profile_frame.timestamp();
    auto status = profile_frame.status();
    auto measurement_id = profile_frame.measurement_id();
    //! [doc-etag-lidarframe-cpp-headers]

    // to access a field:
    // clang-format off
    //! [doc-stag-lidarframe-cpp-fields]
    Eigen::Ref<core::img_t<uint32_t>> range = profile_frame.field(core::ChanField::RANGE);
    Eigen::Ref<core::img_t<uint8_t>>  reflectivity = profile_frame.field(core::ChanField::REFLECTIVITY); // Surface reflectance values
    Eigen::Ref<core::img_t<uint32_t>> range2 = profile_frame.field(core::ChanField::RANGE2); // Second return measurements (if available and enabled)
    Eigen::Ref<core::img_t<uint8_t>>  reflectivity2 = profile_frame.field(core::ChanField::REFLECTIVITY2);
    Eigen::Ref<core::img_t<uint16_t>> near_ir = profile_frame.field(core::ChanField::NEAR_IR); // Near IR measurements
    //! [doc-etag-lidarframe-cpp-fields]
    // clang-format on

    std::cerr << "\nPrinting first element of received frame headers\n\tframe_id : " << frame_id
              << "\n\ttimestamp : " << timestamp(0) << "\n\tstatus : " << status(0)
              << "\n\tmeasurement_id : " << measurement_id(0) << "\n " << std::endl;

    std::cerr << "\nPrinting range of pixel at 15th row and 498th "
                 "column...\n\trange(15, 498): "
              << range(15, 498) << std::endl;

    std::cerr << "Printing other values for pixel at 15th row and 498th column, "
              << "reflectivity: " << reflectivity(15, 498) << "\nnear_ir: " << near_ir(15, 498)
              << "\nreflectivity2: " << reflectivity2(15, 498) << std::endl;

    if (sensor_info->format.udp_profile_lidar ==
        core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL) {
        std::cerr << "\nPrinting range of second return at 15th row and 498th "
                     "column...\n\trange(15, 498): "
                  << range2(15, 498) << std::endl;
    }
    // Let's see what happens if you try to access a field that isn't in a
    // LidarFrame
    std::cerr << "Accessing field that isn't available...";
    try {
        auto signal_field = reduced_fields_frame.field<uint32_t>(core::ChanField::SIGNAL);
        std::cerr << signal_field(0, 0) << std::endl;
    } catch (const std::out_of_range&) {
        std::cerr << " ..received expected out of range error. Continuing..." << std::endl;
    }

    std::cerr << "\nLet's see what's in each of these frames!" << std::endl;
    // If you want to iterate through the available fields, you can use an
    // iterator
    auto print_el = [](core::LidarFrame& lidar_frame, const std::string& label) {
        std::cerr << "Available fields in " << label << "...\n";
        // clang-format off
        //! [doc-stag-cpp-frame-iter]
        for (const auto& kv : lidar_frame.fields()) {
            const auto& field_type = kv.first;
            const auto& field_info = kv.second;
            std::cerr << "\t" << std::setw(15) << std::left << field_type
                      << core::to_string(field_info.tag())
                      << "\n";
        //! [doc-etag-cpp-frame-iter]
            // clang-format on
        }
        std::cerr << std::endl;
    };

    std::cerr << "\nLet's create a frame with a custom field." << std::endl;
    // clang-format off
    //![doc-stag-cpp-frame-add-field]
    auto custom_frame = core::LidarFrame(sensor_info, {});
    custom_frame.add_field("my-custom-field", core::fd_array<uint8_t>(sensor_info->h(), sensor_info->w()));
    custom_frame.field<uint8_t>("my-custom-field") = 1; // set all pixels
    custom_frame.field<uint8_t>("my-custom-field").block(10, 10, 20, 20) = 255;  // set a block of pixels again 
    //![doc-etag-cpp-frame-add-field]
    // clang-format on

    print_el(profile_frame, std::string("Profile Frame"));
    print_el(reduced_fields_frame, std::string("Reduced fields Frame"));
    print_el(custom_frame, std::string("Custom fields Frame"));
    print_field_dtypes(pcap_path);
    std::cout << "Example completed successfully!" << std::endl;
    return EXIT_SUCCESS;
}

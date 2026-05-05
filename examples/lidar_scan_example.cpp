/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the LidarScan class of the
 * C++ Ouster SDK. Please see the sdk docs at static.ouster.dev for clearer
 * explanations.
 */
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "helpers.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/os_pcap.h"
#include "ouster/types.h"

using namespace ouster::sdk;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM
                  << ")"
                  << "\n\nUsage: lidar_scan_example <pcap_file> <json_file>"
                  << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string pcap_file = argv[1];
    const std::string json_file = argv[2];

    auto handle = pcap::replay_initialize(pcap_file);
    auto info = core::metadata_from_json(json_file);

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    // Specifiying only w and h for lidar scan creates one using the LEGACY udp
    // profile
    //! [doc-stag-lidarscan-default-constructor]
    auto legacy_scan = core::LidarScan(w, h);
    //! [doc-etag-lidarscan-default-constructor]

    // You can also create a LidarScan by providing a lidar profile, avilable
    // through the sensor_info
    //! [doc-stag-lidarscan-profile-constructor]
    auto profile_scan = core::LidarScan(w, h, info.format.udp_profile_lidar);
    //! [doc-etag-lidarscan-profile-constructor]

    // You might have a dual returns sensor, in which case your profile will
    // reflect that it is a dual return profile:
    //! [doc-stag-lidarscan-dual-profile-constructor]
    auto dual_returns_scan = core::LidarScan(
        w, h, core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
    //! [doc-etag-lidarscan-dual-profile-constructor]

    //! [doc-stag-lidarscan-reduced-slots]
    // Finally, you can construct by specifying fields directly
    static const std::array<core::FieldType, 2> reduced_slots{
        {{core::ChanField::RANGE, core::ChanFieldType::UINT32},
         {core::ChanField::NEAR_IR, core::ChanFieldType::UINT16}}};
    auto reduced_fields_scan =
        core::LidarScan(w, h, reduced_slots.begin(), reduced_slots.end());
    //! [doc-etag-lidarscan-reduced-slots]

    std::cerr << "Creating scans from pcap... ";

    get_complete_scan(handle, profile_scan, info);
    get_complete_scan(handle, reduced_fields_scan, info);

    if (info.format.udp_profile_lidar !=
        core::UDPProfileLidar::RNG15_RFL8_NIR8) {
        get_complete_scan(handle, legacy_scan, info);
    }

    if (info.format.udp_profile_lidar ==
        core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL) {
        get_complete_scan(handle, dual_returns_scan, info);
    }
    std::cerr << "Scans created!" << std::endl;

    pcap::replay_uninitialize(*handle);

    // Headers
    auto frame_id = profile_scan.frame_id;
    //! [doc-stag-lidarscan-cpp-headers]
    auto timestamp = profile_scan.timestamp();
    auto status = profile_scan.status();
    auto measurement_id = profile_scan.measurement_id();
    //! [doc-etag-lidarscan-cpp-headers]

    // to access a field:
    //! [doc-stag-lidarscan-cpp-fields]
    // Distance measurements in millimeters
    auto range = profile_scan.field<uint32_t>(core::ChanField::RANGE);

    // Surface reflectance values
    auto reflectivity =
        profile_scan.field<uint8_t>(core::ChanField::REFLECTIVITY);

    // Near IR measurements
    auto near_ir = profile_scan.field<uint16_t>(core::ChanField::NEAR_IR);

    // Second return measurements (if available and enabled)
    auto range2 = profile_scan.field<uint32_t>(core::ChanField::RANGE2);
    auto reflectivity2 =
        profile_scan.field<uint8_t>(core::ChanField::REFLECTIVITY2);
    //! [doc-etag-lidarscan-cpp-fields]

    std::cerr
        << "\nPrinting first element of received scan headers\n\tframe_id : "
        << frame_id << "\n\ttimestamp : " << timestamp(0)
        << "\n\tstatus : " << status(0)
        << "\n\tmeasurement_id : " << measurement_id(0) << "\n " << std::endl;

    std::cerr << "\nPrinting range of pixel at 15th row and 498th "
                 "column...\n\trange(15, 498): "
              << range(15, 498) << std::endl;

    std::cerr
        << "Printing other values for pixel at 15th row and 498th column, "
        << "reflectivity: " << reflectivity(15, 498)
        << "\nnear_ir: " << near_ir(15, 498)
        << "\nreflectivity2: " << reflectivity2(15, 498) << std::endl;

    if (info.format.udp_profile_lidar ==
        core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL) {
        std::cerr << "\nPrinting range of second return at 15th row and 498th "
                     "column...\n\trange(15, 498): "
                  << range2(15, 498) << std::endl;
    }
    // Let's see what happens if you try to access a field that isn't in a
    // LidarScan
    std::cerr << "Accessing field that isn't available...";
    try {
        auto signal_field =
            reduced_fields_scan.field<uint32_t>(core::ChanField::SIGNAL);
        std::cerr << signal_field(0, 0) << std::endl;
    } catch (const std::out_of_range&) {
        std::cerr << " ..received expected out of range error. Continuing..."
                  << std::endl;
    }

    std::cerr << "\nLet's see what's in each of these scans!" << std::endl;
    // If you want to iterate through the available fields, you can use an
    // iterator
    auto print_el = [](core::LidarScan& scan, std::string label) {
        std::cerr << "Available fields in " << label << "...\n";
        //! [doc-stag-cpp-scan-iter]
        for (const auto& kv : scan.fields()) {
            auto field_name = kv.first;
            // auto& field = kv.second;
            std::cerr << "\t" << field_name << "\n ";
        }
        //! [doc-etag-cpp-scan-iter]
        std::cerr << std::endl;
    };

    std::cerr << "\nLet's create a scan with a custom field." << std::endl;
    //![doc-stag-cpp-scan-add-field]
    auto custom_scan = core::LidarScan(info);
    custom_scan.add_field("my-custom-field",
                          core::fd_array<uint8_t>(info.h(), info.w()));

    // Custom fields
    custom_scan.field<uint8_t>("my-custom-field") = 1;  // set all pixels
    custom_scan.field<uint8_t>("my-custom-field").block(10, 10, 20, 20) =
        255;  // set a block of pixels
    //![doc-etag-cpp-scan-add-field]

    print_el(legacy_scan, std::string("Legacy Scan"));
    print_el(profile_scan, std::string("Profile Scan"));
    print_el(dual_returns_scan, std::string("Dual Returns Scan"));
    print_el(reduced_fields_scan, std::string("Reduced fields Scan"));
    print_el(custom_scan, std::string("Custom fields Scan"));
}

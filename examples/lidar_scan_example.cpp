/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * This file contains example code for working with the LidarScan class of the
 * C++ Ouster SDK. Please see the sdk docs at static.ouster.dev for clearer
 * explanations.
 */
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "helpers.h"
#include "ouster/impl/build.h"
#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/os_pcap.h"
#include "ouster/types.h"

using namespace ouster::sensor;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: lidar_scan_example <pcap_file> <json_file>"
                  << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string pcap_file = argv[1];
    const std::string json_file = argv[2];

    auto handle = ouster::sensor_utils::replay_initialize(pcap_file);
    auto info = metadata_from_json(json_file);

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    // Specifiying only w and h for lidar scan creates one using the LEGACY udp
    // profile
    //! [doc-stag-lidarscan-default-constructor]
    auto legacy_scan = ouster::LidarScan(w, h);
    //! [doc-etag-lidarscan-default-constructor]

    // You can also create a LidarScan by providing a lidar profile, avilable
    // through the sensor_info
    //! [doc-stag-lidarscan-profile-constructor]
    auto profile_scan = ouster::LidarScan(w, h, info.format.udp_profile_lidar);
    //! [doc-etag-lidarscan-profile-constructor]

    // You might have a dual returns sensor, in which case your profile will
    // reflect that it is a dual return profile:
    //! [doc-stag-lidarscan-dual-profile-constructor]
    auto dual_returns_scan = ouster::LidarScan(
        w, h, UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
    //! [doc-etag-lidarscan-dual-profile-constructor]

    //! [doc-stag-lidarscan-reduced-slots]
    // Finally, you can construct by specifying fields directly
    static const std::array<std::pair<ChanField, ChanFieldType>, 2>
        reduced_slots{{{ChanField::RANGE, ChanFieldType::UINT32},
                       {ChanField::NEAR_IR, ChanFieldType::UINT16}}};
    auto reduced_fields_scan =
        ouster::LidarScan(w, h, reduced_slots.begin(), reduced_slots.end());
    //! [doc-etag-lidarscan-reduced-slots]

    std::cerr << "Creating scans from pcap... ";

    get_complete_scan(handle, profile_scan, info);
    get_complete_scan(handle, reduced_fields_scan, info);

    if (info.format.udp_profile_lidar !=
        UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8) {
        get_complete_scan(handle, legacy_scan, info);
    }

    if (info.format.udp_profile_lidar ==
        UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL) {
        get_complete_scan(handle, dual_returns_scan, info);
    }
    std::cerr << "Scans created!" << std::endl;

    ouster::sensor_utils::replay_uninitialize(*handle);

    // Headers
    auto frame_id = profile_scan.frame_id;
    //! [doc-stag-lidarscan-cpp-headers]
    auto ts = profile_scan.timestamp();
    auto status = profile_scan.status();
    auto measurement_id = profile_scan.measurement_id();
    //! [doc-etag-lidarscan-cpp-headers]

    // to access a field:
    //! [doc-stag-lidarscan-cpp-fields]
    auto range = profile_scan.field(ChanField::RANGE);
    //! [doc-etag-lidarscan-cpp-fields]

    // On dual returns, second returns are often the same field name with 2
    // appended:
    auto range2 = dual_returns_scan.field(ChanField::RANGE2);

    std::cerr
        << "\nPrinting first element of received scan headers\n\tframe_id : "
        << frame_id << "\n\tts : " << ts(0) << "\n\tstatus : " << status(0)
        << "\n\tmeasurement_id : " << measurement_id(0) << "\n " << std::endl;

    std::cerr << "\nPrinting range of pixel at 15th row and 498th "
                 "column...\n\trange(15, 498): "
              << range(15, 498) << std::endl;

    if (info.format.udp_profile_lidar ==
        ouster::sensor::UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL) {
        std::cerr << "\nPrinting range of second return at 15th row and 498th "
                     "column...\n\trange(15, 498): "
                  << range2(15, 498) << std::endl;
    }
    // Let's see what happens if you try to access a field that isn't in a
    // LidarScan
    std::cerr << "Accessing field that isn't available...";
    try {
        auto signal_field = reduced_fields_scan.field(ChanField::SIGNAL);
        std::cerr << signal_field(0, 0) << std::endl;
    } catch (const std::out_of_range&) {
        std::cerr << " ..received expected out of range error. Continuing..."
                  << std::endl;
    }

    std::cerr << "\nLet's see what's in each of these scans!" << std::endl;
    // If you want to iterate through the available fields, you can use an
    // iterator
    auto print_el = [](ouster::LidarScan& scan, std::string label) {
        std::cerr << "Available fields in " << label << "...\n";
        //! [doc-stag-cpp-scan-iter]
        for (auto it = scan.begin(); it != scan.end(); it++) {
            auto field = it->first;
            // auto field_type = it->second;
            std::cerr << "\t" << to_string(field) << "\n ";
        }
        //! [doc-etag-cpp-scan-iter]
        std::cerr << std::endl;
    };

    print_el(legacy_scan, std::string("Legacy Scan"));
    print_el(profile_scan, std::string("Profile Scan"));
    print_el(dual_returns_scan, std::string("Dual Returns Scan"));
    print_el(reduced_fields_scan, std::string("Reduced fields Scan"));
}

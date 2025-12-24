/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * An example of how to use the Zone Monitoring API to create and upload
 * zone configurations to a sensor.
 */
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cxxopts.hpp"
#include "ouster/client.h"
#include "ouster/open_source.h"
#include "ouster/sensor_http.h"
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"
#include "ouster/version.h"
#include "ouster/zone_monitor.h"

using namespace ouster::sdk;  // NOLINT(google-build-using-namespace)

// Creates a ZoneSet from STL files.
void create_stl_zone_set(const std::string& data_dir,
                         const std::string& zip_path,
                         const core::SensorInfo& /*sensor_info*/) {
    // [doc-stag-stl-zone-set]
    // Define a zone from STL file
    core::Stl stl_0(data_dir + "/0.stl");
    stl_0.coordinate_frame = core::Stl::CoordinateFrame::BODY;
    core::Zone zone_0{};
    zone_0.stl = stl_0;
    zone_0.point_count = 10;
    zone_0.frame_count = 1;
    zone_0.mode = core::Zone::ZoneMode::OCCUPANCY;

    // Define another zone from STL file
    core::Stl stl_1(data_dir + "/1.stl");
    stl_1.coordinate_frame = core::Stl::CoordinateFrame::BODY;
    core::Zone zone_1{};
    zone_1.stl = stl_1;
    zone_1.point_count = 20;
    zone_1.frame_count = 2;
    zone_1.mode = core::Zone::ZoneMode::VACANCY;

    // Create a zone set and add the zones
    core::ZoneSet zone_set;
    zone_set.sensor_to_body_transform = core::mat4d::Identity();
    zone_set.zones = {{0, zone_0}, {1, zone_1}};
    zone_set.power_on_live_ids = {0, 1};

    // Print the JSON representation of the zone set
    std::cout << zone_set.to_json(core::ZoneSetOutputFilter::STL) << std::endl;

    // Write out the zone set to a zip file
    zone_set.save(zip_path, core::ZoneSetOutputFilter::STL);
    // [doc-etag-stl-zone-set]
}

// Creates a ZoneSet from ZRB files.
void create_zrb_zone_set(const std::string& data_dir,
                         const std::string& zip_path,
                         const core::SensorInfo& sensor_info) {
    // [doc-stag-zrb-zone-set]
    core::mat4d sensor_to_body_transform = core::mat4d::Identity();

    // Define a zone from a pair of images
    core::Zrb zrb{};
    zrb.near_range_mm = core::img_t<uint32_t>::Constant(sensor_info.h(),
                                                        sensor_info.w(), 10000);
    zrb.far_range_mm = core::img_t<uint32_t>::Constant(sensor_info.h(),
                                                       sensor_info.w(), 100000);
    zrb.serial_number = sensor_info.sn;
    zrb.beam_to_lidar_transform = sensor_info.beam_to_lidar_transform;
    zrb.lidar_to_sensor_transform = sensor_info.lidar_to_sensor_transform;
    zrb.sensor_to_body_transform = sensor_to_body_transform;

    core::Zone zone_0{};
    zone_0.point_count = 100;
    zone_0.frame_count = 1;
    zone_0.mode = core::Zone::ZoneMode::OCCUPANCY;
    zone_0.zrb = zrb;

    // Create a second zone from an STL file
    core::Zone zone_1{};
    core::Stl stl_1(data_dir + "/1.stl");
    zone_1.stl = stl_1;
    zone_1.stl.value().coordinate_frame = core::Stl::CoordinateFrame::BODY;
    zone_1.point_count = 10;
    zone_1.frame_count = 1;
    zone_1.mode = core::Zone::ZoneMode::OCCUPANCY;

    // Create a zone set and add the zones
    core::ZoneSet zone_set{};
    zone_set.sensor_to_body_transform = sensor_to_body_transform;
    zone_set.power_on_live_ids = {0, 1};
    zone_set.zones = {{0, zone_0}, {1, zone_1}};

    // render all STLs to ZRBs
    zone_set.render(sensor_info);

    // Print the JSON representation of the zone set
    std::cout << zone_set.to_json(core::ZoneSetOutputFilter::ZRB) << std::endl;

    // Write out the zone set to a zip file
    zone_set.save(zip_path, core::ZoneSetOutputFilter::ZRB);
    // [doc-etag-zrb-zone-set]
}

// Uploads the zone set to the sensor.
void upload(const core::ZoneSet& zone_set,
            core::ZoneSetOutputFilter output_filter,
            const std::string& sensor_hostname) {
    // [doc-stag-upload-zone-set]
    auto http = sensor::SensorHttp::create(sensor_hostname);
    std::cout << "Uploading zone monitor config..." << std::endl;
    http->set_zone_monitor_config_zip(zone_set.to_zip_blob(output_filter));
    std::cout << "Applying staged config to active..." << std::endl;
    http->apply_zone_monitor_staged_config_to_active();
    std::cout << "Reinitializing sensor..." << std::endl;
    http->reinitialize();
    // [doc-etag-upload-zone-set]
}

int main(int argc, char* argv[]) {  // NOLINT(bugprone-exception-escape)
    cxxopts::Options options(
        "zone_monitor_example",
        "Ouster SDK Zone Monitor example. This example demonstrates how to "
        "create and upload zone configurations to a sensor.");

    // clang-format off
    options.add_options()
        ("z,zone_set_type", "Type of zone set to create", cxxopts::value<std::string>())
        ("d,data_dir", "Directory containing data files", cxxopts::value<std::string>())
        ("p,zip_path", "Path to save the zone set zip file", cxxopts::value<std::string>())
        ("s,sensor_hostname", "Sensor hostname or IP address", cxxopts::value<std::string>())
        ("no-auto-udp-dest", "Disable auto UDP destination configuration", cxxopts::value<bool>()->default_value("false"))
        ("upload", "Upload the zone set to the sensor", cxxopts::value<bool>()->default_value("false"))
        ("h,help", "Print usage");
    // clang-format on

    try {
        auto result = options.parse(argc, argv);

        if (result.count("help") != 0 || result.count("zone_set_type") == 0 ||
            result.count("data_dir") == 0 || result.count("zip_path") == 0 ||
            result.count("sensor_hostname") == 0) {
            std::cout << options.help() << std::endl;
            return EXIT_SUCCESS;
        }

        const auto zone_set_type = result["zone_set_type"].as<std::string>();
        const auto data_dir = result["data_dir"].as<std::string>();
        const auto zip_path = result["zip_path"].as<std::string>();
        const auto sensor_hostname =
            result["sensor_hostname"].as<std::string>();
        const bool no_auto_udp_dest = result["no-auto-udp-dest"].as<bool>();
        const bool do_upload = result["upload"].as<bool>();

        auto source =
            open_source(sensor_hostname, [&](ScanSourceOptions& options) {
                options.no_auto_udp_dest = no_auto_udp_dest;
            });
        auto sensor_info = source.sensor_info()[0];

        core::ZoneSetOutputFilter output_filter{};
        if (zone_set_type == "STL") {
            create_stl_zone_set(data_dir, zip_path, *sensor_info);
            output_filter = core::ZoneSetOutputFilter::STL;
        } else if (zone_set_type == "ZRB") {
            create_zrb_zone_set(data_dir, zip_path, *sensor_info);
            output_filter = core::ZoneSetOutputFilter::ZRB;
        } else {
            std::cerr << "Invalid zone_set_type: " << zone_set_type
                      << ". Must be 'STL' or 'ZRB'." << std::endl;
            return EXIT_FAILURE;
        }

        // Load zone set from zip
        core::ZoneSet zone_set_from_zip(zip_path);

        if (do_upload) {
            upload(zone_set_from_zip, output_filter, sensor_hostname);
        }

        return EXIT_SUCCESS;

    } catch (const cxxopts::exceptions::exception& e) {
        std::cerr << "Error parsing options: " << e.what() << std::endl;
        std::cout << options.help() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::runtime_error& e) {
        std::cerr << "Runtime error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

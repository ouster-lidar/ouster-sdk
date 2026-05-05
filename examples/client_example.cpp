/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ouster/client.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"
#include "ouster/xyzlut.h"

using namespace ouster::sdk;

const size_t N_SCANS = 5;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM
                  << ")"
                  << "\n\nUsage: client_example <sensor_hostname> "
                     "[<sensor_hostname>]..."
                  << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    // Limit ouster_client log statements to "info"
    sensor::init_logger("info");

    std::cout << "Ouster client example " << SDK_VERSION << std::endl;
    /*
     * The SensorScanSource is the high level client for working with Ouster
     * sensors. It is used to connect to sensors, configure them and batch
     * LidarScans from them.
     *
     * It supports unicast, multicast and multiple sensors on the same ports.
     */

    // Build list of all sensors to connect to
    std::vector<sensor::Sensor> sensors;

    std::vector<int> count;
    for (int arg = 1; arg < argc; arg++) {
        const std::string sensor_hostname = argv[arg];

        std::cout << "Connecting to \"" << sensor_hostname << "\"...\n";

        core::SensorConfig config;
        config.udp_dest = "@auto";  // autodetect the UDP destination IP
        // config.udp_port_lidar = 0; // If you set any of the ports to 0, an
        // ephemeral port is used
        sensor::Sensor sensor(sensor_hostname, config);
        sensors.push_back(sensor);
        count.push_back(0);
    }

    // Finally create the client. This will configure all the sensors
    // and wait for them to initialize.
    // After this, the source immediately begins collecting data in the
    // background
    sensor::SensorScanSource source(sensors);

    std::cout << "Connection to sensors succeeded" << std::endl;

    // Now we can print metadata about each sensor since it was collected by the
    // source already While we are at it build necessary lookup tables
    std::vector<core::XYZLut> luts;
    for (const auto& info : source.sensor_info()) {
        std::cout << "Sensor " << info->sn << " Information:" << std::endl;

        size_t w = info->format.columns_per_frame;
        size_t h = info->format.pixels_per_column;

        core::ColumnWindow column_window = info->format.column_window;

        std::cout << "  Firmware version:  " << info->image_rev
                  << "\n  Serial number:     " << info->sn
                  << "\n  Product line:      " << info->prod_line
                  << "\n  Scan dimensions:   " << w << " x " << h
                  << "\n  Column window:     [" << column_window.first << ", "
                  << column_window.second << "]" << std::endl;

        // Pre-compute a table for efficiently calculating point clouds from
        // range
        luts.push_back(core::XYZLut(
            *info, true /* if extrinsics should be used or not */));
    }

    std::cout << "Capturing scans... ";

    /*
     * The example code includes functions for efficiently and accurately
     * computing point clouds from range measurements. LidarScan data can
     * also be accessed directly using the Eigen[0] linear algebra library.
     *
     * [0] http://eigen.tuxfamily.org
     */

    int file_ind = 0;
    std::string file_base{"cloud_"};
    // Loop until we get at least the desired number of scans from each sensor
    while (true) {
        std::pair<int, std::unique_ptr<core::LidarScan>> result =
            source.get_scan();

        auto& scan = *result.second;
        auto index = result.first;

        // grab scans until we get N from each sensor
        if (!result.second) {
            continue;
        }

        // Now process the cloud and save it
        // First compute a point cloud using the lookup table
        auto cloud = luts[index](scan);

        // channel fields can be queried as well
        auto n_valid_first_returns =
            (scan.field<uint32_t>(core::ChanField::RANGE) != 0).count();

        // LidarScan also provides access to header information such as
        // status and timestamp
        auto status = scan.status();
        auto it = std::find_if(status.data(), status.data() + status.size(),
                               [](const uint32_t status_val) {
                                   return (status_val & 0x01);
                               });  // find first valid status
        if (it != status.data() + status.size()) {
            auto ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::nanoseconds(scan.timestamp()(
                    it - status.data())));  // get corresponding timestamp

            std::cout << "  Frame no. " << scan.frame_id << " with "
                      << n_valid_first_returns << " valid first returns at "
                      << ts_ms.count() << " ms" << std::endl;
        }

        // Finally save the scan
        std::string filename = file_base + std::to_string(file_ind++) + ".csv";
        std::ofstream out;
        out.open(filename);
        out << std::fixed << std::setprecision(4);

        // write each point, filtering out points without returns
        for (int i = 0; i < cloud.rows(); i++) {
            auto xyz = cloud.row(i);
            if (!xyz.isApproxToConstant(0.0)) {
                out << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
            }
        }

        out.close();
        std::cout << "  Wrote " << filename << std::endl;

        // Increment our count of that scan and check if we got all 5
        count[index]++;
        bool all = true;
        for (size_t count_index = 0; count_index < count.size();
             count_index++) {
            if (count[count_index] != N_SCANS) {
                all = false;
            }
        }
        if (all) {
            break;
        }
    }

    std::cout << "Done" << std::endl;

    return EXIT_SUCCESS;
}

/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "ouster/client.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/sensor_client.h"
#include "ouster/types.h"

using namespace ouster;

const size_t N_SCANS = 5;

void FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: client_example <sensor_hostname> "
                     "[<sensor_hostname>]..."
                  << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    // Limit ouster_client log statements to "info"
    sensor::init_logger("info");

    std::cerr << "Ouster client example " << ouster::SDK_VERSION << std::endl;
    /*
     * The sensor client consists of the network client and a library for
     * reading and working with data.
     *
     * The network client supports reading and writing a limited number of
     * configuration parameters and receiving data without working directly with
     * the socket APIs. See the `client.h` for more details. The minimum
     * required parameters are the sensor hostname/ip and the data destination
     * hostname/ip.
     */

    // build list of all sensors
    std::vector<ouster::sensor::Sensor> sensors;
    std::vector<int> count;
    for (int a = 1; a < argc; a++) {
        const std::string sensor_hostname = argv[a];

        std::cerr << "Connecting to \"" << sensor_hostname << "\"...\n";

        ouster::sensor::sensor_config config;
        config.udp_dest = "@auto";
        ouster::sensor::Sensor s(sensor_hostname, config);

        sensors.push_back(s);
        count.push_back(0);
    }

    ouster::sensor::SensorClient client(sensors);

    std::cerr << "Connection to sensors succeeded" << std::endl;

    // Since the client has fetched and cached the metadata, we can now print
    // info about each sensor and build objects necessary for batching.
    std::vector<ScanBatcher> batch_to_scan;
    std::vector<std::vector<LidarScan>> scans;

    std::vector<XYZLut> luts;
    for (const auto& info : client.get_sensor_info()) {
        size_t w = info.format.columns_per_frame;
        size_t h = info.format.pixels_per_column;

        ouster::sensor::ColumnWindow column_window = info.format.column_window;

        std::cerr << "  Firmware version:  " << info.image_rev
                  << "\n  Serial number:     " << info.sn
                  << "\n  Product line:      " << info.prod_line
                  << "\n  Scan dimensions:   " << w << " x " << h
                  << "\n  Column window:     [" << column_window.first << ", "
                  << column_window.second << "]" << std::endl;
        batch_to_scan.push_back(ScanBatcher(info));
        scans.push_back({N_SCANS, LidarScan{info}});
        // pre-compute a table for efficiently calculating point clouds from
        // range
        luts.push_back(ouster::make_xyz_lut(info, true));
    }

    /*
     * The network client provides some convenience wrappers around socket APIs
     * to facilitate reading lidar and IMU data from the network. It is also
     * possible to configure the sensor offline and read data directly from a
     * UDP socket.
     */
    std::cerr << "Capturing points... ";

    // buffer to store raw packet data
    auto lidar_packet = sensor::LidarPacket();
    auto imu_packet = sensor::ImuPacket();

    while (true) {
        auto ev = client.get_packet(lidar_packet, imu_packet, 0.1);
        if (ev.type == ouster::sensor::ClientEvent::LidarPacket) {
            if (count[ev.source] == N_SCANS) continue;
            //  batcher will return "true" when the current scan is complete
            if (batch_to_scan[ev.source](lidar_packet,
                                         scans[ev.source][count[ev.source]])) {
                // retry until we receive a full set of valid measurements
                // (accounting for azimuth_window settings if any)
                if (scans[ev.source][count[ev.source]].complete(
                        client.get_sensor_info()[ev.source]
                            .format.column_window)) {
                    count[ev.source]++;
                }
            }
        } else if (ev.type == ouster::sensor::ClientEvent::ImuPacket) {
            // got an IMU packet
        }

        if (ev.type == ouster::sensor::ClientEvent::Error) {
            FATAL("Sensor client returned error state!");
        }

        if (ev.type == ouster::sensor::ClientEvent::PollTimeout) {
            FATAL("Sensor client returned poll timeout state!");
        }

        // exit when we got all our scans
        bool all = true;
        for (size_t p = 0; p < count.size(); p++) {
            if (count[p] != N_SCANS) all = false;
        }
        if (all) break;
    }
    std::cerr << "ok" << std::endl;

    /*
     * The example code includes functions for efficiently and accurately
     * computing point clouds from range measurements. LidarScan data can
     * also be accessed directly using the Eigen[0] linear algebra library.
     *
     * [0] http://eigen.tuxfamily.org
     */
    std::cerr << "Computing point clouds... " << std::endl;

    std::vector<std::vector<LidarScan::Points>> clouds;
    clouds.resize(scans.size());
    for (size_t i = 0; i < scans.size(); i++) {
        for (const LidarScan& scan : scans[i]) {
            // compute a point cloud using the lookup table
            clouds[i].push_back(ouster::cartesian(scan, luts[i]));

            // channel fields can be queried as well
            auto n_valid_first_returns =
                (scan.field<uint32_t>(sensor::ChanField::RANGE) != 0).count();

            // LidarScan also provides access to header information such as
            // status and timestamp
            auto status = scan.status();
            auto it = std::find_if(status.data(), status.data() + status.size(),
                                   [](const uint32_t s) {
                                       return (s & 0x01);
                                   });  // find first valid status
            if (it != status.data() + status.size()) {
                auto ts_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::nanoseconds(scan.timestamp()(
                            it -
                            status.data())));  // get corresponding timestamp

                std::cerr << "  Frame no. " << scan.frame_id << " with "
                          << n_valid_first_returns << " valid first returns at "
                          << ts_ms.count() << " ms" << std::endl;
            }
        }
    }

    /*
     * Write output to CSV files. The output can be viewed in a point cloud
     * viewer like CloudCompare:
     *
     * [0] https://github.com/cloudcompare/cloudcompare
     */
    std::cerr << "Writing files... " << std::endl;

    for (size_t i = 0; i < clouds.size(); i++) {
        std::string file_base{"cloud_"};
        file_base += std::to_string(i) + "_";
        int file_ind = 0;
        for (const LidarScan::Points& cloud : clouds[i]) {
            std::string filename =
                file_base + std::to_string(file_ind++) + ".csv";
            std::ofstream out;
            out.open(filename);
            out << std::fixed << std::setprecision(4);

            // write each point, filtering out points without returns
            for (int i = 0; i < cloud.rows(); i++) {
                auto xyz = cloud.row(i);
                if (!xyz.isApproxToConstant(0.0))
                    out << xyz(0) << ", " << xyz(1) << ", " << xyz(2)
                        << std::endl;
            }

            out.close();
            std::cerr << "  Wrote " << filename << std::endl;
        }
    }

    return EXIT_SUCCESS;
}

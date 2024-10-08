/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
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

/*
 * Display some stats about the captured Lidar Scan
 */
void display_scan_summary(const LidarScan& scan);

/*
 * Write output to CSV files. The output can be viewed in a point cloud
 * viewer like CloudCompare:
 *
 * [0] https://github.com/cloudcompare/cloudcompare
 */
void write_cloud(const std::string& file_path, const LidarScan::Points& cloud);

int main(int argc, char* argv[]) {
    if (argc != 2 && argc != 3) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: async_client_example <sensor_hostname> "
                     "[<udp_destination>]"
                     "\n\n<udp_destination> is optional: leave blank for "
                     "automatic destination detection"
                  << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    // Limit ouster_client log statements to "info" and direct the output to log
    // file rather than the console (default).
    sensor::init_logger("info", "ouster.log");

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
    const std::string sensor_hostname = argv[1];
    const std::string data_destination = (argc == 3) ? argv[2] : "@auto";

    std::cerr << "Connecting to \"" << sensor_hostname << "\"...\n";

    ouster::sensor::sensor_config config;
    config.udp_dest = data_destination;
    sensor::SensorClient client(
        {ouster::sensor::Sensor(sensor_hostname, config)});
    std::cerr << "Connection to sensor succeeded" << std::endl;

    /*
     * Configuration and calibration parameters can be queried directly from the
     * sensor. These are required for parsing the packet stream and calculating
     * accurate point clouds.
     */
    std::cerr << "Gathering metadata..." << std::endl;

    // You can access the retrieved metadata from the SensorClient class
    sensor::sensor_info info = client.get_sensor_info()[0];

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    ouster::sensor::ColumnWindow column_window = info.format.column_window;

    std::cerr << "  Firmware version:  " << info.fw_rev
              << "\n  Serial number:     " << info.sn
              << "\n  Product line:      " << info.prod_line
              << "\n  Scan dimensions:   " << w << " x " << h
              << "\n  Column window:     [" << column_window.first << ", "
              << column_window.second << "]" << std::endl;

    // A LidarScan holds lidar data for an entire rotation of the device
    LidarScan scan{info};

    // pre-compute a table for efficiently calculating point clouds from
    // range
    // the second argument specifies if sensor extrinsics should be applied to
    // the output point cloud
    XYZLut lut = ouster::make_xyz_lut(info, true);
    // A an array of points to hold the projected representation of the scan
    LidarScan::Points cloud;

    // A ScanBatcher can be used to batch packets into scans
    ScanBatcher batch_to_scan(info);

    /*
     * The network client provides some convenience wrappers around socket APIs
     * to facilitate reading lidar and IMU data from the network. It is also
     * possible to configure the sensor offline and read data directly from a
     * UDP socket.
     */

    // Place to store raw packets as they pass between threads
    ouster::sensor::LidarPacket lidar_packet;
    ouster::sensor::ImuPacket imu_packet;

    /*
    In this example we spin two threads one to receive lidar packets while the
    other thread accumlates lidar packets of the same frame into a LidarScan
    object, computes the xyz coordinates and then writes these coordiantes into
    a file. The example is a show case of utilizing threads to decouple
    reception of packets from processing the point cloud. For a more complete
    examples on how to efficient stream and process lidar packets please refer
    to the async_udp_source_example.cpp or the ouster_ros driver implementation
    */
    size_t n_scans = 0;  // counter to track the number of complete scans that
                         // we have successfully captured and processed.
    std::mutex mtx;
    std::condition_variable receiving_cv;
    std::condition_variable processing_cv;
    bool packet_processed = true;

    std::thread packet_receiving_thread([&]() {
        while (n_scans < N_SCANS) {
            // wait until sensor data is available
            auto ev = client.get_packet(lidar_packet, imu_packet, 1.0);

            // check for timeout
            if (ev.type == ouster::sensor::ClientEvent::PollTimeout)
                FATAL("Client has timed out");

            // check for error state
            if (ev.type == ouster::sensor::ClientEvent::Error)
                FATAL("Exit was requested");

            // check for lidar data, read a packet and add it to the current
            // batch
            if (ev.type == ouster::sensor::ClientEvent::LidarPacket) {
                std::unique_lock<std::mutex> lock(mtx);
                receiving_cv.wait(
                    lock, [&packet_processed] { return packet_processed; });
                packet_processed = false;
                processing_cv.notify_one();
            }

            // check if IMU data is available (but don't do anything with it)
            if (ev.type == ouster::sensor::ClientEvent::ImuPacket) {
                std::unique_lock<std::mutex> lock(mtx);
                receiving_cv.wait(
                    lock, [&packet_processed] { return packet_processed; });
                // we are not going to processor imu data
                // so we will keep packet_processed set to true
            }
        }
    });

    std::thread packet_processing_thread([&]() {
        while (n_scans < N_SCANS) {
            std::unique_lock<std::mutex> lock(mtx);
            processing_cv.wait(
                lock, [&packet_processed] { return !packet_processed; });
            // batcher will return "true" when the current scan is complete
            if (batch_to_scan(lidar_packet, scan)) {
                // retry until we receive a full set of valid measurements
                // (accounting for azimuth_window settings if any)
                if (scan.complete(info.format.column_window)) {
                    display_scan_summary(scan);
                    std::cerr << "Computing point cloud... " << std::endl;
                    cloud = ouster::cartesian(scan, lut);
                    std::string file_name =
                        "cloud_" + std::to_string(n_scans) + ".csv";
                    write_cloud(file_name, cloud);
                    ++n_scans;
                }
            }

            packet_processed = true;
            receiving_cv.notify_one();
        }
    });

    packet_receiving_thread.join();
    packet_processing_thread.join();

    std::cerr << "done" << std::endl;

    return EXIT_SUCCESS;
}

void display_scan_summary(const LidarScan& scan) {
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
        auto ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::nanoseconds(scan.timestamp()(
                it - status.data())));  // get corresponding timestamp

        std::cerr << "  Frame no. " << scan.frame_id << " with "
                  << n_valid_first_returns << " valid first returns at "
                  << ts_ms.count() << " ms" << std::endl;
    }
}

void write_cloud(const std::string& file_path, const LidarScan::Points& cloud) {
    std::ofstream out;
    out.open(file_path);
    out << std::fixed << std::setprecision(4);

    // write each point, filtering out points without returns
    for (int i = 0; i < cloud.rows(); i++) {
        auto xyz = cloud.row(i);
        if (!xyz.isApproxToConstant(0.0))
            out << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
    }

    out.close();
    std::cerr << "  Wrote " << file_path << std::endl;
}

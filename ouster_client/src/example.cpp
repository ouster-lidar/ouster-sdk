/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "ouster/build.h"
#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

using namespace ouster;

const size_t N_SCANS = 5;
const size_t UDP_BUF_SIZE = 65536;

void FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

int main(int argc, char* argv[]) {
    if (argc != 2 && argc != 3) {
        std::cerr << "Version: " << ouster::CLIENT_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: ouster_client_example <sensor_hostname> "
                     "[<data_destination_ip>]"
                     "\n\n<data_destination_ip> is optional: leave blank for "
                     "automatic destination detection"
                  << std::endl;

        return EXIT_FAILURE;
    }
    std::cerr << "Ouster client example " << ouster::CLIENT_VERSION
              << std::endl;
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
    const std::string data_destination = (argc == 3) ? argv[2] : "";

    std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";

    auto handle = sensor::init_client(sensor_hostname, data_destination);
    if (!handle) FATAL("Failed to connect");
    std::cerr << "ok" << std::endl;

    /*
     * Configuration and calibration parameters can be queried directly from the
     * sensor. These are required for parsing the packet stream and calculating
     * accurate point clouds.
     */
    std::cerr << "Gathering metadata..." << std::endl;
    auto metadata = sensor::get_metadata(*handle);

    // Raw metadata can be parsed into a `sensor_info` struct
    sensor::sensor_info info = sensor::parse_metadata(metadata);

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    ouster::sensor::ColumnWindow column_window = info.format.column_window;

    // azimuth_window config param reduce the amount of valid columns per scan
    // that we will receive
    int column_window_length =
        (column_window.second - column_window.first + w) % w + 1;

    std::cerr << "  Firmware version:  " << info.fw_rev
              << "\n  Serial number:     " << info.sn
              << "\n  Product line:      " << info.prod_line
              << "\n  Scan dimensions:   " << w << " x " << h
              << "\n  Column window:     [" << column_window.first << ", "
              << column_window.second << "]" << std::endl;

    // A LidarScan holds lidar data for an entire rotation of the device
    std::vector<LidarScan> scans{
        N_SCANS, LidarScan{w, h, info.format.udp_profile_lidar}};

    // A ScanBatcher can be used to batch packets into scans
    sensor::packet_format pf = sensor::get_format(info);
    ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);

    /*
     * The network client provides some convenience wrappers around socket APIs
     * to facilitate reading lidar and IMU data from the network. It is also
     * possible to configure the sensor offline and read data directly from a
     * UDP socket.
     */
    std::cerr << "Capturing points... ";

    // buffer to store raw packet data
    std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);

    for (size_t i = 0; i < N_SCANS;) {
        // wait until sensor data is available
        sensor::client_state st = sensor::poll_client(*handle);

        // check for error status
        if (st & sensor::CLIENT_ERROR)
            FATAL("Sensor client returned error state!");

        // check for lidar data, read a packet and add it to the current batch
        if (st & sensor::LIDAR_DATA) {
            if (!sensor::read_lidar_packet(*handle, packet_buf.get(), pf))
                FATAL("Failed to read a packet of the expected size!");

            // batcher will return "true" when the current scan is complete
            if (batch_to_scan(packet_buf.get(), scans[i])) {
                // LidarScan provides access to azimuth block data and headers
                auto n_invalid = std::count_if(
                    scans[i].headers.begin(), scans[i].headers.end(),
                    [](const LidarScan::BlockHeader& h) {
                        return !(h.status & 0x01);
                    });
                // retry until we receive a full set of valid measurements
                // (accounting for azimuth_window settings if any)
                if (n_invalid <= (int)w - column_window_length) i++;
            }
        }

        // check if IMU data is available (but don't do anything with it)
        if (st & sensor::IMU_DATA) {
            sensor::read_imu_packet(*handle, packet_buf.get(), pf);
        }
    }
    std::cerr << "ok" << std::endl;

    /*
     * The example code includes functions for efficiently and accurately
     * computing point clouds from range measurements. LidarScan data can also
     * be accessed directly using the Eigen[0] linear algebra library.
     *
     * [0] http://eigen.tuxfamily.org
     */
    std::cerr << "Computing point clouds... " << std::endl;

    // pre-compute a table for efficiently calculating point clouds from range
    XYZLut lut = ouster::make_xyz_lut(info);
    std::vector<LidarScan::Points> clouds;

    for (const LidarScan& scan : scans) {
        // compute a point cloud using the lookup table
        clouds.push_back(ouster::cartesian(scan, lut));

        // channel fields can be queried as well
        auto n_returns = (scan.field(sensor::RANGE) != 0).count();

        std::cerr << "  Frame no. " << scan.frame_id << " with " << n_returns
                  << " returns" << std::endl;
    }

    /*
     * Write output to CSV files. The output can be viewed in a point cloud
     * viewer like CloudCompare:
     *
     * [0] https://github.com/cloudcompare/cloudcompare
     */
    std::cerr << "Writing files... " << std::endl;

    int file_ind = 0;
    std::string file_base{"cloud_"};
    for (const LidarScan::Points& cloud : clouds) {
        std::string filename = file_base + std::to_string(file_ind++) + ".csv";
        std::ofstream out;
        out.open(filename);
        out << std::fixed << std::setprecision(4);

        // write each point, filtering out points without returns
        for (int i = 0; i < cloud.rows(); i++) {
            auto xyz = cloud.row(i);
            if (!xyz.isApproxToConstant(0.0))
                out << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
        }

        out.close();
        std::cerr << "  Wrote " << filename << std::endl;
    }

    return EXIT_SUCCESS;
}

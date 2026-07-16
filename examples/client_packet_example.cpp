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
#include <string>
#include <vector>

#include "ouster/core/impl/build.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"
#include "ouster/sensor/client.h"
#include "ouster/sensor/sensor_packet_source.h"

// NOLINTNEXTLINE(google-build-using-namespace)
using namespace ouster::sdk;

const size_t N_FRAMES = 5;

void fatal(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Version: " << SDK_VERSION_FULL << " (" << BUILD_SYSTEM << ")"
                  << "\n\nUsage: client_example <sensor_hostname> "
                     "[<sensor_hostname>]..."
                  << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    // Limit ouster_client log statements to "info"
    sensor::init_logger("info");

    std::cout << "Ouster client example " << SDK_VERSION << std::endl;
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
    std::vector<sensor::Sensor> sensors;
    std::vector<int> count;
    for (int arg = 1; arg < argc; arg++) {
        const std::string sensor_hostname = argv[arg];

        std::cout << "Connecting to \"" << sensor_hostname << "\"...\n";

        core::SensorConfig config;
        config.udp_dest = "@auto";
        sensor::Sensor sensor(sensor_hostname, config);

        sensors.push_back(sensor);
        count.push_back(0);
    }
    //! [doc-stag-frame-batching]
    sensor::SensorPacketSource client(sensors);

    std::cout << "Connection to sensors succeeded" << std::endl;

    // Since the client has fetched and cached the metadata, we can now print
    // info about each sensor and build objects necessary for batching.
    std::vector<core::FrameBatcher> batch_to_frame;
    std::vector<std::vector<core::LidarFrame>> frames;

    std::vector<core::XYZLut> luts;
    for (const auto& info : client.sensor_info()) {
        size_t w = info->format.columns_per_frame;
        size_t h = info->format.pixels_per_column;

        ouster::sdk::core::ColumnWindow column_window = info->format.column_window;

        std::cout << "  Firmware version:  " << info->image_rev
                  << "\n  Serial number:     " << info->sn
                  << "\n  Product line:      " << info->prod_line << "\n  Frame dimensions:   " << w
                  << " x " << h << "\n  Column window:     [" << column_window.first << ", "
                  << column_window.second << "]" << std::endl;
        batch_to_frame.emplace_back(*info);
        frames.emplace_back(N_FRAMES, core::LidarFrame{info});
        // pre-compute a table for efficiently calculating point clouds from
        // range
        luts.emplace_back(*info, true);
    }

    /*
     * The network client provides some convenience wrappers around socket APIs
     * to facilitate reading lidar and IMU data from the network. It is also
     * possible to configure the sensor offline and read data directly from a
     * UDP socket.
     */
    std::cout << "Capturing points... ";

    while (true) {
        auto event = client.get_packet(1.0);
        if (event.type == sensor::ClientEvent::PACKET) {
            if (event.packet().type() == core::PacketType::Lidar) {
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
                // Safe: type verified above with event.packet().type() check
                auto& lidar_packet = static_cast<core::LidarPacket&>(event.packet());
                if (count[event.source] == N_FRAMES) {
                    continue;
                }
                //  batcher will return "true" when the current frame is
                //  complete
                if (batch_to_frame[event.source].batch(lidar_packet,
                                                       frames[event.source][count[event.source]])) {
                    // retry until we receive a full set of valid measurements
                    // (accounting for azimuth_window settings if any)
                    if (frames[event.source][count[event.source]].complete(
                            client.sensor_info()[event.source]->format.column_window)) {
                        count[event.source]++;
                    }
                }
            } else if (event.packet().type() == core::PacketType::Imu) {
                // got an IMU packet
            }
        }
        //! [doc-etag-frame-batching]
        if (event.type == sensor::ClientEvent::ERR) {
            fatal("Sensor client returned error state!");
        }

        if (event.type == sensor::ClientEvent::POLL_TIMEOUT) {
            fatal("Sensor client returned poll timeout state!");
        }

        // exit when we got all our frames
        bool all = true;
        for (const auto& cnt : count) {
            if (cnt != N_FRAMES) {
                all = false;
            }
        }
        if (all) {
            break;
        }
    }
    std::cout << "ok" << std::endl;

    /*
     * The example code includes functions for efficiently and accurately
     * computing point clouds from range measurements. LidarFrame data can
     * also be accessed directly using the Eigen[0] linear algebra library.
     *
     * [0] http://eigen.tuxfamily.org
     */
    std::cout << "Computing point clouds... " << std::endl;

    std::vector<std::vector<core::PointCloudXYZd>> clouds;
    clouds.resize(frames.size());
    for (size_t i = 0; i < frames.size(); i++) {
        for (const core::LidarFrame& frame : frames[i]) {
            // compute a point cloud using the lookup table
            clouds[i].push_back(luts[i](frame));

            // channel fields can be queried as well
            auto n_valid_first_returns =
                (frame.field<uint32_t>(core::ChanField::RANGE) != 0).count();

            // LidarFrame also provides access to header information such as
            // status and timestamp
            auto status = frame.status();
            auto it = std::find_if(status.data(), status.data() + status.size(),
                                   [](const uint32_t status_val) {
                                       return (status_val & 0x01);
                                   });  // find first valid status
            if (it != status.data() + status.size()) {
                auto ts_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::nanoseconds(
                        frame.timestamp()(it - status.data())));  // get corresponding timestamp

                std::cout << "  Frame no. " << frame.frame_id << " with " << n_valid_first_returns
                          << " valid first returns at " << ts_ms.count() << " ms" << std::endl;
            }
        }
    }

    /*
     * Write output to CSV files. The output can be viewed in a point cloud
     * viewer like CloudCompare:
     *
     * [0] https://github.com/cloudcompare/cloudcompare
     */
    std::cout << "Writing files... " << std::endl;

    for (size_t i = 0; i < clouds.size(); i++) {
        std::string file_base{"cloud_"};
        file_base += std::to_string(i) + "_";
        int file_ind = 0;
        for (const core::PointCloudXYZd& cloud : clouds[i]) {
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
        }
    }

    return EXIT_SUCCESS;
}

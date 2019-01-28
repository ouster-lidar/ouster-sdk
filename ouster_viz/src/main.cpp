#include <unistd.h>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>

#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster/viz.h"

namespace OS1 = ouster::OS1;
namespace viz = ouster::viz;

/**
 * Print usage
 */
void print_help() {
    std::cout << "Usage:\n"
              << "Required arguments: ./viz hostname udp_destination_host\n"
              << "Optional Arguments:\n"
              << "  -m <512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10> "
                 "lidar mode, default: 1024x10"
              << std::endl;
}

int main(int argc, char** argv) {
    int W = 1024;
    int H = OS1::pixels_per_column;
    OS1::lidar_mode mode = OS1::MODE_1024x10;

    try {
        int c = 0;
        while ((c = getopt(argc, argv, "hm:")) != -1) {
            switch (c) {
                case 'h':
                    print_help();
                    return 1;
                    break;
                case 'm':
                    mode = OS1::lidar_mode_of_string(optarg);
                    if (mode) {
                        W = OS1::n_cols_of_lidar_mode(mode);
                    } else {
                        std::cout << "Lidar Mode must be 512x10, 512x20, "
                                     "1024x10, 1024x20, or 2048x10"
                                  << std::endl;
                        print_help();
                        std::exit(EXIT_FAILURE);
                    }
                    break;
                case '?':
                    std::cout << "Invalid Argument Format" << std::endl;
                    print_help();
                    std::exit(EXIT_FAILURE);
                    break;
            }
        }
    } catch (const std::exception& ex) {
        std::cout << "Invalid Argument Format: " << ex.what() << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

    if (argc != optind + 2) {
        std::cerr << "Expected 2 arguments after options" << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Sensor: " << argv[optind]
              << " UDP Destination:" << argv[optind + 1] << std::endl;
    std::shared_ptr<OS1::client> cli;
    cli = OS1::init_client(argv[optind], argv[optind + 1], mode);
    if (!cli) {
        std::cerr << "Failed to connect to client at: " << argv[optind]
                  << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

    uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    uint8_t imu_buf[OS1::imu_packet_bytes + 1];

    auto ls = std::unique_ptr<ouster::LidarScan>(new ouster::LidarScan(W, H));

    auto vh = viz::init_viz(W, H);

    auto info = OS1::parse_metadata(get_metadata(*cli));

    auto xyz_lut = OS1::make_xyz_lut(W, H, info.beam_azimuth_angles,
                                     info.beam_altitude_angles);

    // Use to signal termination
    std::atomic_bool end_program{false};

    auto it = std::back_inserter(*ls);

    // callback that calls update with filled lidar scan
    auto batch_and_update =
        OS1::batch_to_iter<std::back_insert_iterator<ouster::LidarScan>,
                           ouster::LidarScan::value_type>(
            xyz_lut, W, H, [&](uint64_t) {
                viz::update(*vh, ls);
                ls->clear();
                it = std::back_inserter(*ls);
            });

    // Start poll thread
    std::thread poll([&] {
        while (!end_program) {
            // Poll the client for data and add to our lidar scan
            OS1::client_state st = OS1::poll_client(*cli);
            if (st & OS1::client_state::ERROR) {
                std::cerr << "Client returned error state" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (st & OS1::client_state::LIDAR_DATA) {
                if (OS1::read_lidar_packet(*cli, lidar_buf))
                    batch_and_update(lidar_buf, it);
            }
            if (st & OS1::client_state::IMU_DATA) {
                OS1::read_imu_packet(*cli, imu_buf);
            }
        }
    });

    // Start render loop
    viz::run_viz(*vh);
    end_program = true;

    // clean up
    poll.join();
    return EXIT_SUCCESS;
}

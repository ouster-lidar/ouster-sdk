#include <unistd.h>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"
#include "ouster/viz.h"

namespace OS1 = ouster::OS1;
namespace viz = ouster::viz;

/**
 * Generate a pre-computed matrix of unit vectors pointing radially outwards,
 * for easily computing a point cloud from a lidar scan
 */
std::vector<double> make_xyz_lut(const viz::SensorSpecifics& ss) {
    int n = ss.H * ss.col_per_rev;
    std::vector<double> xyz = std::vector<double>(3 * n, 0);
    double* x(xyz.data());
    double* y(xyz.data() + n);
    double* z(xyz.data() + 2 * n);

    for (int icol = 0; icol < ss.col_per_rev; icol++) {
        for (int ipx = 0; ipx < OS1::pixels_per_column; ipx++) {
            double h_angle_0 =
                (2.0 * M_PI * (double)icol) / ((double)ss.col_per_rev);
            double h_angle = std::sin((double)ss.beam_azimuth_angles.at(ipx) *
                                      2 * M_PI / 360.0) +
                             h_angle_0;

            x[(ipx * ss.col_per_rev) + icol] =
                -std::cos((double)ss.beam_altitude_angles.at(ipx) * 2 * M_PI /
                          360.0) *
                std::cos(h_angle);
            y[(ipx * ss.col_per_rev) + icol] =
                std::cos((double)ss.beam_altitude_angles.at(ipx) * 2 * M_PI /
                         360.0) *
                std::sin(h_angle);
            z[(ipx * ss.col_per_rev) + icol] = std::sin(
                (double)ss.beam_altitude_angles.at(ipx) * 2 * M_PI / 360.0);
        }
    }
    return xyz;
}

/**
 * Add a LiDAR UDP column to the lidar scan
 */
int add_col_to_lidar_scan(const uint8_t* col_buf,
                          std::unique_ptr<ouster::LidarScan>& lidar_scan) {
    const int ticks = OS1::col_h_encoder_count(col_buf);
    // drop packets with invalid encoder counts
    if (ticks < 0 or ticks > OS1::encoder_ticks_per_rev) {
        return 0;
    }
    const int azimuth = ticks * lidar_scan->W / OS1::encoder_ticks_per_rev;

    for (size_t row = 0; row < OS1::pixels_per_column; row++) {
        const uint8_t* px_buf = OS1::nth_px(row, col_buf);
        size_t index = row * lidar_scan->W + azimuth;
        lidar_scan->range.at(index) = OS1::px_range(px_buf);
        lidar_scan->intensity.at(index) = OS1::px_signal_photons(px_buf);
        lidar_scan->reflectivity.at(index) = OS1::px_reflectivity(px_buf);
        lidar_scan->noise.at(index) = OS1::px_noise_photons(px_buf);
    }
    return azimuth;
}

/**
 * Add a LiDAR UDP packet to the Point OS1 cloud
 */
template <typename F>
void add_packet_to_lidar_scan(const uint8_t* buf, int& last_azimuth,
                              std::unique_ptr<ouster::LidarScan>& lidar_scan,
                              F&& f) {
    for (int icol = 0; icol < OS1::columns_per_buffer; icol++) {
        const uint8_t* col_buf = OS1::nth_col(icol, buf);
        // individual column gets dropped if the column is invalid
        if (!OS1::col_valid(col_buf)) continue;
        int azimuth =
            (add_col_to_lidar_scan(col_buf, lidar_scan) + 1) % (lidar_scan->W);

        if (azimuth < last_azimuth) f();
        last_azimuth = azimuth;
    }
}

/**
 * Print usage
 */
void print_help() {
    std::cout << "Usage:\n"
              << "Required arguments: ./viz hostname udp_destination_host\n"
              << "Optional Arguments:"
              << "\n\t-m <512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10> "
                 "lidar mode, default: 1024x10"
              << "\n\t-r <float> range image visualization scaling factor"
              << "\n\t-i <float> intensity image visualization scaling factor"
              << "\n\t-n <float> noise image visualization scaling factor"
              << std::endl;
}

int main(int argc, char** argv) {
    viz::UserConfig uc;
    viz::SensorSpecifics ss = {
        1024, OS1::pixels_per_column,
        std::vector<float>(OS1::beam_azimuth_angles.begin(),
                           OS1::beam_azimuth_angles.end()),
        std::vector<float>(OS1::beam_altitude_angles.begin(),
                           OS1::beam_altitude_angles.end())};

    std::cout << "Sensor: " << argv[1] << " UDP Destination:" << argv[2]
              << std::endl;
    std::shared_ptr<OS1::client> cli;

    try {
        cli = OS1::init_client(argv[1], argv[2], 7502, 7503);
        if (!cli) {
            std::cerr << "Failed to connect to client at: " << argv[1]
                      << std::endl;
            print_help();
            return 1;
        }

        int c = 0;
        int temp = 0;
        while ((c = getopt(argc, argv, "hr:n:i:m:")) != -1) {
            switch (c) {
                case 'h':
                    print_help();
                    return 1;
                    break;
                case 'r':
                    uc.range_scale = std::stod(optarg);
                    break;
                case 'n':
                    uc.noise_scale = std::stod(optarg);
                    break;
                case 'i':
                    uc.intensity_scale = std::stod(optarg);
                    break;
                case 'm':
                    temp = std::stod(optarg);
                    if (temp == 512 || temp == 1024 || temp == 2048) {
                        ss.col_per_rev = temp;
                    } else {
                        std::cout << "Lidar Mode must be 512, 1024, or 2048"
                                  << std::endl;
                        return -1;
                    }
                    break;
                case '?':
                    std::cout << "Incorrect Argument Format" << std::endl;
                    print_help();
                    return -1;
                    break;
            }
        }
    } catch (const std::exception& ex) {
        std::cout << "Invalid format : " << ex.what() << std::endl;
        print_help();
        return 1;
    }

    uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    uint8_t imu_buf[OS1::imu_packet_bytes + 1];

    auto ls_poll = std::unique_ptr<ouster::LidarScan>(
        new ouster::LidarScan(ss.col_per_rev, ss.H));

    int ls_counter = ss.col_per_rev;

    std::vector<double> xyz_lut = make_xyz_lut(ss);
    auto vh = viz::init_viz(xyz_lut, uc, ss);

    // Use to signal termination
    std::atomic_bool end_program{false};

    // Start render loop
    std::thread render([&] {
        viz::run_viz(*vh);
        end_program = true;
    });

    // Poll the client for data and add to our lidar scan
    while (!end_program) {
        OS1::client_state st = OS1::poll_client(*cli);

        if (st & OS1::client_state::ERROR) {
            std::cerr << "Client returned error state" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        if (st & OS1::client_state::LIDAR_DATA) {
            if (OS1::read_lidar_packet(*cli, lidar_buf)) {
                add_packet_to_lidar_scan(lidar_buf, ls_counter, ls_poll, [&] {
                    viz::update_poll(*vh, ls_poll);
                });
            }
        }
        if (st & OS1::client_state::IMU_DATA) {
            OS1::read_imu_packet(*cli, imu_buf);
        }
    }

    // clean up
    render.join();
    return 0;
}

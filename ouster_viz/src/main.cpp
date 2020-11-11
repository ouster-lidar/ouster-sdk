#include <tclap/CmdLine.h>

#include <Eigen/Eigen>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
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
    std::cout
        << "Usage: viz [options] [hostname] [udp_destination]\n"
        << "Options:\n"
        << "  -m <512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10> : lidar mode, "
           "default 1024x10\n"
        << "  -l <port> : use specified port for lidar data\n"
        << "  -i <port> : use specified port for imu data \n"
        << "  -f <path> : use provided metadata file; do not configure via TCP"
        << std::endl;
}

int main(int argc, char** argv) {
    OS1::lidar_mode mode = OS1::MODE_1024x10;
    bool do_config = true;  // send tcp commands to configure sensor
    int lidar_port = 0;
    int imu_port = 0;

    std::string meta_file{};
    std::string hostname;
    std::string udp_dest;

    try {
        TCLAP::CmdLine cmd("Ouster Visualizer");

        TCLAP::UnlabeledValueArg<std::string> hostname_arg(
            "hostname", "hostname of the sensor", true, "", "string");
        cmd.add(hostname_arg);

        TCLAP::UnlabeledValueArg<std::string> udp_destination_arg(
            "udp_destination", "destination host for the udp packets", true, "",
            "string");
        cmd.add(udp_destination_arg);

        TCLAP::ValueArg<std::string> mode_arg("m", "mode",
                                              "<512x10 | 512x20 | 1024x10 | "
                                              "1024x20 | 2048x10> : lidar mode "
                                              "default 1024x10",
                                              false, "1024x10", "string");
        cmd.add(mode_arg);

        TCLAP::ValueArg<int> lidar_port_arg("l", "lidar_port",
                                            "use specified port for lidar data",
                                            false, 0, "int");
        cmd.add(lidar_port_arg);

        TCLAP::ValueArg<int> imu_port_arg("i", "imu_port",
                                          "use specified port for imu data",
                                          false, 0, "int");
        cmd.add(imu_port_arg);

        TCLAP::ValueArg<std::string> meta_file_arg(
            "f", "metadata_path",
            "use provided metadata file; do not configure via TCP", false, "",
            "string");
        cmd.add(meta_file_arg);

        cmd.parse(argc, argv);

        if (meta_file_arg.getValue() != "") {
            do_config = false;
            meta_file = meta_file_arg.getValue();
        }

        mode = OS1::lidar_mode_of_string(mode_arg.getValue());
        if (!mode) {
            std::cout << "Lidar Mode must be 512x10, 512x20, "
                         "1024x10, 1024x20, or 2048x10"
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }

        lidar_port = lidar_port_arg.getValue();
        imu_port = imu_port_arg.getValue();

        hostname = hostname_arg.getValue();
        udp_dest = udp_destination_arg.getValue();
    } catch (const std::exception& ex) {
        std::cout << "Invalid Argument Format: " << ex.what() << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

    std::shared_ptr<OS1::client> cli;
    socket_init();
    if (do_config) {
        std::cout << "Configuring sensor: " << hostname
                  << " UDP Destination: " << udp_dest << std::endl;
        cli = OS1::init_client(hostname, udp_dest, mode, lidar_port, imu_port);
    } else {
        if (lidar_port == 0) lidar_port = 7502;
        if (imu_port == 0) imu_port = 7503;
        std::cout << "Listening for sensor data on udp ports " << lidar_port
                  << " and " << imu_port << std::endl;
        cli = OS1::init_client("", lidar_port, imu_port);
    }

    if (!cli) {
        std::cerr << "Failed to initialize client" << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

    std::string metadata{};
    if (do_config) {
        metadata = OS1::get_metadata(*cli);
    } else {
        std::stringstream buf{};
        std::ifstream ifs{};
        ifs.open(meta_file);
        buf << ifs.rdbuf();
        ifs.close();

        if (!ifs)
            std::cerr << "Failed to read metadata file: " << meta_file
                      << std::endl;

        metadata = buf.str();
    }

    auto info = OS1::parse_metadata(metadata);
    int H = info.format.pixels_per_column;
    int W = info.format.columns_per_frame;

    auto pf = OS1::get_format(info.format);

    const auto xyz_lut =
        ouster::make_xyz_lut(W, H, OS1::range_unit,
                             info.lidar_origin_to_beam_origin_mm,
                             info.beam_azimuth_angles, info.beam_altitude_angles);
    auto vh = viz::init_viz(&xyz_lut, info.format, info.prod_line);

    // Use to signal termination
    std::atomic_bool end_program{false};

    std::unique_ptr<uint8_t[]> lidar_buf(new uint8_t[pf.lidar_packet_size + 1]);
    std::unique_ptr<uint8_t[]> imu_buf(new uint8_t[pf.imu_packet_size + 1]);

    auto ls = std::unique_ptr<ouster::LidarScan>(new ouster::LidarScan(W, H));
    auto it = ls->begin();

    // callback that calls update with filled lidar scan
    auto batch_and_update = OS1::batch_to_iter<ouster::LidarScan::iterator>(
        W, pf, ouster::LidarScan::Pixel::empty_val(), &ouster::LidarScan::pixel,
        [&](std::chrono::nanoseconds) {
            // swap lidar scan and point it to new buffer
            viz::update(*vh, ls);
            it = ls->begin();
        });

    // Start poll thread
    std::thread poll([&] {
        while (!end_program) {
            // Poll the client for data and add to our lidar scan
            OS1::client_state st = OS1::poll_client(*cli);
            if (st & OS1::client_state::CLIENT_ERROR) {
                std::cerr << "Client returned error state" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (st & OS1::client_state::LIDAR_DATA) {
                if (OS1::read_lidar_packet(*cli, lidar_buf.get(), pf))
                    batch_and_update(lidar_buf.get(), it);
            }
            if (st & OS1::client_state::IMU_DATA) {
                OS1::read_imu_packet(*cli, imu_buf.get(), pf);
            }
        }
    });

    // Start render loop
    viz::run_viz(*vh);
    end_program = true;

    // clean up
    poll.join();
    socket_quit();
    return EXIT_SUCCESS;
}

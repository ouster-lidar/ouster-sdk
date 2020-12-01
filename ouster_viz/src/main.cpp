#include <tclap/CmdLine.h>

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ouster/build.h"
#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/lidar_scan_viz.h"
#include "ouster/point_viz.h"
#include "ouster/types.h"

namespace sensor = ouster::sensor;
namespace viz = ouster::viz;

int main(int argc, char** argv) {
    sensor::lidar_mode mode = sensor::MODE_UNSPEC;
    bool do_config = true;  // send tcp commands to configure sensor
    int lidar_port = 0;
    int imu_port = 0;

    std::string meta_file{};
    std::string hostname;
    std::string udp_dest;

    try {
        TCLAP::CmdLine cmd("Ouster Visualizer", ' ',
                           ouster::CLIENT_VERSION_FULL);

        TCLAP::UnlabeledValueArg<std::string> hostname_arg(
            "hostname", "hostname of the sensor", true, "", "string");
        cmd.add(hostname_arg);

        TCLAP::UnlabeledValueArg<std::string> udp_destination_arg(
            "udp_destination", "destination host for the udp packets", true, "",
            "string");
        cmd.add(udp_destination_arg);

        TCLAP::ValueArg<std::string> mode_arg(
            "m", "mode",
            "<512x10 | 512x20 | 1024x10 | "
            "1024x20 | 2048x10> : lidar mode ",
            false, "", "string");
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

        if (mode_arg.getValue().size()) {
            mode = sensor::lidar_mode_of_string(mode_arg.getValue());
            if (!mode) {
                std::cout << "Lidar Mode must be 512x10, 512x20, "
                             "1024x10, 1024x20, or 2048x10"
                          << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }

        lidar_port = lidar_port_arg.getValue();
        imu_port = imu_port_arg.getValue();

        hostname = hostname_arg.getValue();
        udp_dest = udp_destination_arg.getValue();
    } catch (const std::exception& ex) {
        std::cout << "Unexpected error: " << ex.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::shared_ptr<sensor::client> cli;

    if (do_config) {
        std::cout << "Connecting to " << hostname << "; sending data to "
                  << udp_dest << std::endl;
        cli =
            sensor::init_client(hostname, udp_dest, mode,
                                sensor::TIME_FROM_UNSPEC, lidar_port, imu_port);
    } else {
        if (lidar_port == 0) lidar_port = 7502;
        if (imu_port == 0) imu_port = 7503;
        std::cout << "Listening for sensor data on udp ports " << lidar_port
                  << " and " << imu_port << std::endl;
        cli = sensor::init_client("", lidar_port, imu_port);
    }

    if (!cli) {
        std::cerr << "Failed to initialize client" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    const sensor::sensor_info info = [&]() {
        if (do_config) {
            return sensor::parse_metadata(sensor::get_metadata(*cli));
        } else {
            return sensor::metadata_from_json(meta_file);
        }
    }();

    std::cout << "Using lidar_mode: " << sensor::to_string(info.mode)
              << std::endl;
    std::cout << info.prod_line << " sn: " << info.sn
              << " firmware rev: " << info.fw_rev << std::endl;

    const uint32_t H = info.format.pixels_per_column;
    const uint32_t W = info.format.columns_per_frame;

    auto packet_format = sensor::get_format(info);

    const auto xyz_lut = ouster::make_xyz_lut(info);

    viz::PointViz point_viz(
        {viz::CloudSetup{xyz_lut.direction.data(), xyz_lut.offset.data(), H * W,
                         W, info.extrinsic.data()}},
        "Ouster Viz", false);

    /**
     * Calling lidar_scan_viz->draw() has a small overhead that blocks for
     * about 0.01 to 0.03 seconds, a fraction of a scan.
     * To avoid dropping packets, we run the sensor client polling in a separate
     * thread, and use the following condition variable to inform the main
     * thread when data is ready to be displayed.
     */
    viz::LidarScanViz lidar_scan_viz(info, point_viz);
    std::condition_variable cv;
    std::mutex swap_mtx;
    bool lidar_scan_ready = false;

    std::unique_ptr<ouster::LidarScan> ls_read(new ouster::LidarScan(W, H));
    std::unique_ptr<ouster::LidarScan> ls_write(new ouster::LidarScan(W, H));

    auto batch = ouster::ScanBatcher(W, packet_format);

    std::unique_ptr<uint8_t[]> lidar_buf(
        new uint8_t[packet_format.lidar_packet_size + 1]);
    std::unique_ptr<uint8_t[]> imu_buf(
        new uint8_t[packet_format.imu_packet_size + 1]);

    std::thread poll([&] {
        while (!point_viz.quit) {
            // Poll the client for data and add to our lidar scan
            sensor::client_state st = sensor::poll_client(*cli);
            if (st & sensor::client_state::CLIENT_ERROR) {
                std::cerr << "Client returned error state" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (st & sensor::client_state::LIDAR_DATA) {
                if (sensor::read_lidar_packet(*cli, lidar_buf.get(),
                                              packet_format)) {
                    if (batch(lidar_buf.get(), *ls_write)) {
                        std::lock_guard<std::mutex> lk(swap_mtx);
                        std::swap(ls_read, ls_write);
                        lidar_scan_ready = true;
                        cv.notify_one();
                    }
                }
            }
            if (st & sensor::client_state::IMU_DATA) {
                sensor::read_imu_packet(*cli, imu_buf.get(), packet_format);
            }
            if (st & sensor::EXIT) {
                point_viz.quit = true;
                break;
            }
        }
    });

    std::thread update_draw([&]() {
        while (!point_viz.quit) {
            std::unique_lock<std::mutex> lk2(swap_mtx);
            cv.wait(lk2, [&]() { return lidar_scan_ready || point_viz.quit; });

            if (point_viz.quit) break;

            lidar_scan_ready = false;
            lidar_scan_viz.draw(*ls_read);
        }
    });

    point_viz.drawLoop();
    cv.notify_one();  // wake up update_draw thread for exit
    poll.join();
    update_draw.join();

    return EXIT_SUCCESS;
}

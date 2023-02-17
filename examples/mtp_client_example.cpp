#include <iostream>

#include "ouster/client.h"

using namespace ouster;

const size_t UDP_BUF_SIZE = 65536;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "\n\nUsage: <sensor_hostname> <main secondary>"
                  << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string sensor_hostname = argv[1];
    // sensor::sensor_config config;

    bool main = false;

    if (std::string(argv[2]) == "main") {
        main = true;
    } else if (std::string(argv[2]) == "secondary") {
        main = false;
    } else {
        std::cerr << "Invalid second argument: " << argv[2]
                  << " only values of main or secondary are valid" << std::endl;
        return EXIT_FAILURE;
    }

    sensor::sensor_config config;
    if (!sensor::get_config(sensor_hostname, config)) {
        std::cerr << "Failed to get sensor config" << std::endl;
        return EXIT_FAILURE;
    }
    config.udp_dest = "239.201.201.201";
    if (sensor::in_multicast(config.udp_dest.value())) {
        std::cerr << "In multicast" << std::endl;
    } else {
        std::cerr << "Not a multicast address" << std::endl;
        return -1;
    }
    std::shared_ptr<ouster::sensor::client> cli;
    if (main) {
        // This assumes a change to subscribe to IPADDR_ANY if not specified.
        // Othewise you need to specify your own IP address such as
        // cli = sensor::mtp_init_client_main(sensor_hostname, config,
        // "192.168.1.11");
        cli = sensor::mtp_init_client_main(sensor_hostname, config);
    } else {
        cli = sensor::mtp_init_client_secondary(sensor_hostname, config);
    }
    if (!cli) {
        std::cerr << "Failed to initialize sensor" << std::endl;
        return EXIT_FAILURE;
    }

    auto metadata = sensor::get_metadata(*cli);
    sensor::sensor_info info = sensor::parse_metadata(metadata);
    sensor::packet_format pf = sensor::get_format(info);
    auto packet_buf = std::make_unique<uint8_t[]>(UDP_BUF_SIZE);

    while (true) {
        sensor::client_state st = sensor::poll_client(*cli);

        if (st == sensor::TIMEOUT) {
            std::cerr << "Timed out" << std::endl;
            continue;
        }
        if (st & sensor::LIDAR_DATA) {
            if (sensor::read_lidar_packet(*cli, packet_buf.get(), pf)) {
                std::cerr << "Read Lidar Packet" << std::endl;
            }
        }

        if (st & sensor::IMU_DATA) {
            if (sensor::read_imu_packet(*cli, packet_buf.get(), pf)) {
                std::cerr << "Read IMU packet" << std::endl;
            }
        }
    }
}
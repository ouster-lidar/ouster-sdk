#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"

namespace OS1 = ouster::OS1;

uint64_t n_lidar_packets = 0;
uint64_t n_imu_packets = 0;

uint64_t lidar_col_0_ts = 0;
uint64_t imu_ts = 0;

float lidar_col_0_h_angle = 0.0;
float imu_av_z = 0.0;
float imu_la_y = 0.0;

void handle_lidar(uint8_t* buf) {
    n_lidar_packets++;
    lidar_col_0_ts = OS1::col_timestamp(OS1::nth_col(0, buf));
    lidar_col_0_h_angle = OS1::col_h_angle(OS1::nth_col(0, buf));
}

void handle_imu(uint8_t* buf) {
    n_imu_packets++;
    imu_ts = OS1::imu_sys_ts(buf);
    imu_av_z = OS1::imu_av_z(buf);
    imu_la_y = OS1::imu_la_y(buf);
}

void print_headers() {
    std::cout << std::setw(15) << "n_lidar_packets" << std::setw(15)
              << "col_0_azimuth" << std::setw(15) << "col_0_ts" << std::setw(15)
              << "n_imu_packets" << std::setw(15) << "im_av_z" << std::setw(15)
              << "im_la_y" << std::setw(15) << "imu_ts" << std::endl;
}

void print_stats() {
    std::cout << "\r" << std::setw(15) << n_lidar_packets << std::setw(15)
              << lidar_col_0_h_angle << std::setw(15) << lidar_col_0_ts
              << std::setw(15) << n_imu_packets << std::setw(15) << imu_av_z
              << std::setw(15) << imu_la_y << std::setw(15) << imu_ts;
    std::flush(std::cout);
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: ouster_client_example <os1_hostname> "
                     "<data_destination_ip>"
                  << std::endl;
        return 1;
    }

    auto cli = OS1::init_client(argv[1], argv[2]);
    if (!cli) {
        std::cerr << "Failed to connect to client at: " << argv[1] << std::endl;
        return 1;
    }

    uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    uint8_t imu_buf[OS1::imu_packet_bytes + 1];

    print_headers();

    while (true) {
        OS1::client_state st = OS1::poll_client(*cli);
        if (st & OS1::ERROR) {
            return 1;
        } else if (st & OS1::LIDAR_DATA) {
            if (OS1::read_lidar_packet(*cli, lidar_buf))
                handle_lidar(lidar_buf);
        } else if (st & OS1::IMU_DATA) {
            if (OS1::read_imu_packet(*cli, imu_buf)) handle_imu(imu_buf);
        }

        if (n_imu_packets % 50 == 0) print_stats();
    }

    return 0;
}

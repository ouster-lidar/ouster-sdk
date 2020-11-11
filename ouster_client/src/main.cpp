#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "ouster/client.h"
#include "ouster/compat.h"

namespace sensor = ouster::sensor;

uint64_t n_lidar_packets = 0;
uint64_t n_imu_packets = 0;

uint64_t lidar_col_0_ts = 0;
uint64_t imu_ts = 0;

float lidar_col_0_h_angle = 0.0;
float imu_av_z = 0.0;
float imu_la_y = 0.0;

void handle_lidar(uint8_t* buf, const sensor::packet_format& pf) {
    n_lidar_packets++;
    lidar_col_0_ts = pf.col_timestamp(pf.nth_col(0, buf));
    lidar_col_0_h_angle = pf.col_h_angle(pf.nth_col(0, buf));
}

void handle_imu(uint8_t* buf, const sensor::packet_format& pf) {
    n_imu_packets++;
    imu_ts = pf.imu_gyro_ts(buf);
    imu_av_z = pf.imu_av_z(buf);
    imu_la_y = pf.imu_la_y(buf);
}

void print_headers() {
    std::cout << std::setw(15) << "n_lidar_packets" << std::setw(15)
              << "col_0_azimuth" << std::setw(20) << "col_0_ts" << std::setw(15)
              << "n_imu_packets" << std::setw(15) << "im_av_z" << std::setw(15)
              << "im_la_y" << std::setw(20) << "imu_ts" << std::endl;
}

void print_stats() {
    std::cout << "\r" << std::setw(15) << n_lidar_packets << std::setw(15)
              << lidar_col_0_h_angle << std::setw(20) << lidar_col_0_ts
              << std::setw(15) << n_imu_packets << std::setw(15) << imu_av_z
              << std::setw(15) << imu_la_y << std::setw(20) << imu_ts;
    std::flush(std::cout);
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: ouster_client_example <sensor_hostname> "
                     "<data_destination_ip>"
                  << std::endl;
        return 1;
    }
    socket_init();
    auto cli = sensor::init_client(argv[1], argv[2]);
    if (!cli) {
        std::cerr << "Failed to connect to client at: " << argv[1] << std::endl;
        return 1;
    }

    auto metadata = sensor::get_metadata(*cli);
    auto info = sensor::parse_metadata(metadata);
    auto pf = sensor::get_format(info.format);

    std::unique_ptr<uint8_t[]> lidar_buf(new uint8_t[pf.lidar_packet_size + 1]);
    std::unique_ptr<uint8_t[]> imu_buf(new uint8_t[pf.imu_packet_size + 1]);

    print_headers();

    while (true) {
        sensor::client_state st = sensor::poll_client(*cli);
        if (st & sensor::CLIENT_ERROR) {
            return 1;
        } else if (st & sensor::LIDAR_DATA) {
            if (sensor::read_lidar_packet(*cli, lidar_buf.get(), pf))
                handle_lidar(lidar_buf.get(), pf);
        } else if (st & sensor::IMU_DATA) {
            if (sensor::read_imu_packet(*cli, imu_buf.get(), pf))
                handle_imu(imu_buf.get(), pf);
        }

        if (n_imu_packets % 50 == 0) print_stats();
    }
    socket_quit();
    return 0;
}

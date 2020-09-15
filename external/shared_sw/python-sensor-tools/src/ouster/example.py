import argparse
import sys

import ouster.client._sensor as sensor
import ouster.client.lidardata as osl
from typing import List

n_lidar_packets: int = 0
n_imu_packets: int = 0

lidar_col_0_ts: int = 0
imu_ts: int = 0

lidar_col_0_h_angle: float = 0.0
imu_av_z: float = 0.0
imu_la_y: float = 0.0


def handle_lidar(col_timestamps, col_encoders,
                 pf: sensor.PacketFormat) -> None:
    global n_lidar_packets
    global lidar_col_0_ts
    global lidar_col_0_h_angle
    n_lidar_packets += 1
    lidar_col_0_ts = col_timestamps[0]
    lidar_col_0_h_angle = col_encoders[0] / pf.encoder_ticks_per_rev


def handle_imu(buf: bytearray, pf: sensor.PacketFormat) -> None:
    global n_imu_packets
    global imu_ts
    global imu_av_z
    global imu_la_y
    n_imu_packets += 1
    imu_ts = pf.imu_sys_ts(buf)
    imu_av_z = pf.imu_av_z(buf)
    imu_la_y = pf.imu_la_y(buf)


def print_headers() -> None:
    sys.stdout.write(
        "{:>15} {:>15} {:>15} {:>15} {:>15} {:>15} {:>15}\n".format(
            "n_lidar_packets", "col_0_azimuth", "col_0_ts", "n_imu_packets",
            "im_av_z", "im_la_y", "imu_ts"))


def print_stats() -> None:
    sys.stdout.write(
        "\r{:15} {:15.3f} {:15.6f} {:15} {:15.3f} {:15.3f} {:15.6f}".format(
            n_lidar_packets, lidar_col_0_h_angle, lidar_col_0_ts / 1e9,
            n_imu_packets, imu_av_z, imu_la_y, imu_ts / 1e9))


def run() -> None:
    argParser = argparse.ArgumentParser(
        description='Barebones OS client written in Python')
    argParser.add_argument("remote_ip",
                           type=str,
                           help="IP address of OS device")
    argParser.add_argument(
        "local_ip",
        type=str,
        help="IP address of the local data-receiving interface")

    args = argParser.parse_args()
    cli = sensor.init_client(args.remote_ip, args.local_ip)

    if cli is None:
        sys.stderr.write("Failed to connect to client at: {}\n".format(
            sys.argv[1]))
        sys.exit(1)
    print_headers()

    meta = sensor.parse_metadata(sensor.get_metadata(cli))
    pf = sensor.get_format(meta)

    try:
        lidar_buf = bytearray(pf.lidar_packet_size + 1)
        imu_buf = bytearray(pf.imu_packet_size + 1)
        os_data = osl.OsLidarData(lidar_buf, pf)
        col_timestamps = os_data.make_col_timestamp_view()
        col_encoders = os_data.make_col_encoder_count_view()
        while True:
            st = sensor.poll_client(cli)
            if st & sensor.ERROR:
                sys.exit(1)
            elif st & sensor.LIDAR_DATA:
                if (sensor.read_lidar_packet(cli, lidar_buf, pf)):
                    handle_lidar(col_timestamps, col_encoders, pf)
            elif st & sensor.IMU_DATA:
                if (sensor.read_imu_packet(cli, imu_buf, pf)):
                    handle_imu(imu_buf, pf)
            if n_imu_packets % 11 == 0:
                print_stats()
    except KeyboardInterrupt:
        pass
    sys.exit(0)

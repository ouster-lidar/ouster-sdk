import argparse
import sys

import ouster.client as client

n_lidar_packets: int = 0
n_imu_packets: int = 0

lidar_col_0_ts: int = 0
imu_ts: int = 0

lidar_col_0_h_angle: float = 0.0
imu_av_z: float = 0.0
imu_la_y: float = 0.0


def handle_lidar(p: client.LidarPacket) -> None:
    global n_lidar_packets
    global lidar_col_0_ts
    global lidar_col_0_h_angle
    n_lidar_packets += 1
    lidar_col_0_ts = p.view(client.ColHeader.TIMESTAMP)[0]
    lidar_col_0_h_angle = p.view(
        client.ColHeader.ENCODER_COUNT)[0] / p._pf.encoder_ticks_per_rev


def handle_imu(p: client.ImuPacket) -> None:
    global n_imu_packets
    global imu_ts
    global imu_av_z
    global imu_la_y
    n_imu_packets += 1
    imu_ts = p.sys_ts
    imu_av_z = p.angular_vel[2]
    imu_la_y = p.accel[1]


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

    print(f"Connecting to {args.remote_ip}; sending data to {args.local_ip}")
    cli = client.init_client(args.remote_ip, args.local_ip)

    if cli is None:
        print(f"Failed to connect to client at: {args.remote_ip}")
        sys.exit(1)

    meta = client.parse_metadata(client.get_metadata(cli))
    pf = client.get_format(meta)

    print(f"Connected to {meta.sn} running {meta.fw_rev}")

    meta = client.parse_metadata(client.get_metadata(cli))
    print_headers()
    try:
        for p in client.packets(pf, cli):
            if isinstance(p, client.LidarPacket):
                handle_lidar(p)
            elif isinstance(p, client.ImuPacket):
                handle_imu(p)
            if n_lidar_packets % 64 == 0:
                print_stats()
    except KeyboardInterrupt:
        print("Caught signal, exiting...")

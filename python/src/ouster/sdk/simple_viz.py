"""
Copyright (c) 2022, Ouster, Inc.
All rights reserved.
"""

import argparse
import numpy as np

from ouster import client
from .viz import SimpleViz
from .util import resolve_metadata


def main() -> None:
    descr = """Visualize pcap or sensor data using simple viz bindings."""

    epilog = """When reading data from a sensor, this will autoconfigure the udp
        destination unless -x is used."""

    parser = argparse.ArgumentParser(description=descr, epilog=epilog)

    required = parser.add_argument_group('one of the following is required')
    group = required.add_mutually_exclusive_group(required=True)
    group.add_argument('--sensor', metavar='HOST', help='sensor hostname')
    group.add_argument('--pcap', metavar='PATH', help='path to pcap file')
    parser.add_argument('--meta', metavar='PATH', help='path to metadata json')
    parser.add_argument('--lidar-port', type=int,
            help='configure sensor to use specified port for lidar data if running against sensor')
    parser.add_argument('-x',
                        '--no-auto-dest',
                        action='store_true',
                        help='do not auto configure udp destination if running against sensor')
    parser.add_argument('--extrinsics',
                        metavar="F",
                        type=float,
                        required=False,
                        nargs=16,
                        help='lidar sensor extrinsics in homogenous matrix given'
                        'in row-major order to use in 3D viz')

    args = parser.parse_args()

    if args.sensor:
        hostname = args.sensor
        if args.lidar_port or (not args.no_auto_dest):
            config = client.SensorConfig()
            if args.lidar_port:
                config.udp_port_lidar = args.lidar_port
            print("Configuring sensor...")
            client.set_config(hostname,
                              config,
                              udp_dest_auto=(not args.no_auto_dest))
        config = client.get_config(hostname)

        print("Initializing...")
        scans = client.Scans.stream(hostname,
                                    config.udp_port_lidar or 7502,
                                    complete=False)
        rate = None

    elif args.pcap:
        import ouster.pcap as pcap

        metadata_path = resolve_metadata(args.pcap, args.meta)
        if not metadata_path:
            print("Metadata file not found, please specify "
                  "a valid metadata file with `--meta`")
            return
        with open(metadata_path) as json:
            print(f"Reading metadata from: {metadata_path}")
            info = client.SensorInfo(json.read())
        scans = client.Scans(pcap.Pcap(args.pcap, info))
        rate = 1.0

    if args.extrinsics:
        scans.metadata.extrinsic = np.array(args.extrinsics).reshape((4, 4))
        print(f"Using sensor extrinsics:\n{scans.metadata.extrinsic}")

    SimpleViz(scans.metadata, rate).run(scans)

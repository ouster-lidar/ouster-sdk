#!/usr/bin/python3

import argparse
import socket
import sys
import os
from datetime import datetime

import ouster.sensor as ouster_sensor
import ouster.pycap as ouster_pcap

parser = argparse.ArgumentParser(
    description=
    'Replay ouster pcap files or record pcap files from a connected ouster sensor')

subparsers = parser.add_subparsers(dest='command', help='sub-command help')

replay = subparsers.add_parser('replay', help='replay help')
replay.add_argument('-l',
                    '--loop',
                    metavar='count',
                    type=int,
                    nargs='?',
                    default=1,
                    help="loop playback")
replay.add_argument('-d',
                    '--dest',
                    metavar='ip',
                    type=str,
                    help="override destination ip")
replay.add_argument('-r',
                    '--rate',
                    metavar='mult',
                    type=float,
                    default=1.0,
                    help="rate multiplier")
replay.add_argument('pcap_file', type=str, help="replay pcap filename")

record = subparsers.add_parser('record', help='record help')
record.add_argument('-o',
                    '--outfile',
                    metavar='pcap_file',
                    type=str,
                    help="record pcap filepath")
record.add_argument('-p',
                    '--passive',
                    action='store_true',
                    help="Recording from a sensor already running")
record.add_argument('-d',
                    '--udp_dest',
                    type=str,
                    help="IP address of the local data-receiving interface")
record.add_argument('hostname', type=str, help="hostname of OUSTER device")

metadata = subparsers.add_parser('metadata', help='metadata help')
metadata.add_argument('-o',
                      '--output',
                      metavar='file',
                      type=str,
                      help="file to write")
metadata.add_argument('hostname', type=str, help="hostname of OUSTER device")

# Dump sensor configuration json
# write_config = subparsers.add_parser('write_config', help='write_config help')
# write_config.add_argument('-o', '--output', metavar='file', type=str, help="file to write")
# write_config.add_argument('hostname', type=str, help="hostname of OUSTER device")

# Configure sensor
# configure = subparsers.add_parser('configure', help='configure help')
# configure.add_argument('-s', '--save', type=bool, default=False, help="save configuration persistently")
# configure.add_argument('config_file', type=str, help="configuration json")
# configure.add_argument('hostname', type=str, help="hostname of OUSTER device")

args = parser.parse_args()

# Replay file
if args.command == 'replay':
    dest = 0
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)

        # loop forever without arg
        if args.loop is None:
            args.loop = -1

        while args.loop != 0:
            print("Starting Replay")
            ouster_pcap.replay_pcap(args.pcap_file, socket.gethostbyname(socket.gethostname()), args.dest,
                                    args.rate)

            args.loop -= 1

    except Exception as e:
        print(e)
        sys.exit(1)

    finally:
        s.close()

elif args.command == 'record':
    if args.outfile is None:
        args.outfile = "./{}_{}.pcap".format(
            args.hostname,
            datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))

    if args.passive:
        if args.udp_dest is not None:
            sys.stderr.write(
                "Warning: udp destination ignored in passive mode")
    else:
        if args.udp_dest is None:
            sys.stderr.write("Please provide a udp data destination\n")
            sys.exit(1)

        cli = ouster_sensor.init_client(args.hostname, args.udp_dest)
        if cli is None:
            sys.stderr.write("Failed to connect to client at: {}\n".format(
                args.hostname))
            sys.exit(1)

    # Start the pcap record
    ouster_pcap.record_pcap(args.outfile, args.hostname)

elif args.command == 'metadata':
    cli = ouster_sensor.init_client(args.hostname)
    if cli is None:
        sys.stderr.write("Failed to connect to client at: {}\n".format(
            args.hostname))
        sys.exit(1)

    meta_string = ouster_sensor.get_metadata(cli)
    if args.output is None:
        import json
        metadata = json.loads(meta_string)
        print(json.dumps(metadata))
    else:
        with open(args.output, 'w') as ofile:
            ofile.write(meta_string)
        print("Metadata saved in : {}".format(args.output))

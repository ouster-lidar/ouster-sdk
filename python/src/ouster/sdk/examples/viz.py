import argparse
import os
import threading

import ouster.client as client
from ouster.sdk._viz import PyViz


def main() -> None:
    descr = """Visualize pcap or sensor data using simple viz bindings."""

    epilog = """When reading data from a sensor, this will currently not
    configure the sensor or query it for the port to listen on. You will need to
    set the sensor port and distination settings separately.
    """

    parser = argparse.ArgumentParser(
        description=descr, epilog=epilog)

    required = parser.add_argument_group('one of the following is required')
    group = required.add_mutually_exclusive_group(required=True)
    group.add_argument('--sensor', metavar='HOST', help='sensor hostname')
    group.add_argument('--pcap', metavar='PATH', help='path to pcap file')

    parser.add_argument('--meta', metavar='PATH', help='path to metadata json')
    parser.add_argument('--lidar-port', type=int, default=7502)

    args = parser.parse_args()

    if args.sensor:
        print("Initializing...")
        scans = client.Scans.stream(args.sensor,
                                    args.lidar_port,
                                    complete=False)
    elif args.pcap:
        import ouster.pcap as pcap

        if args.meta:
            metadata_path = args.meta
        else:
            print("Deducing metadata based on pcap name. "
                  "To provide a different metadata path, use --meta")
            metadata_path = os.path.splitext(args.pcap)[0] + ".json"

        with open(metadata_path) as json:
            info = client.SensorInfo(json.read())
        scans = client.Scans(
            pcap.Pcap(args.pcap, info, rate=1.0, lidar_port=args.lidar_port))

    viz = PyViz(scans.metadata)

    def run() -> None:
        try:
            for scan in scans:
                viz.draw(scan)
        finally:
            # signal main thread to exit
            viz.quit()

    try:
        print("Starting client thread...")
        client_thread = threading.Thread(target=run, name="Client")
        client_thread.start()

        print("Starting rendering loop...")
        viz.loop()
    finally:
        scans.close()
        client_thread.join()

    print("Done")


if __name__ == "__main__":
    main()

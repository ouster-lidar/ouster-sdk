import argparse
import os
import threading

import ouster.client as client
from ouster.sdk._viz import PyViz


def sensor_viz(hostname: str, lidar_port: int) -> None:
    """Listen for data on the specified lidar port and run the visualizer.

    Note: this will currently not configure the sensor or query the sensor for
    the port to listen on. You will need to set the sensor port and distination
    settings separately.
    """

    print("Initializing...")
    quit = threading.Event()
    scans = client.Scans.stream(hostname,
                                lidar_port)

    viz = PyViz(scans.metadata)

    def run() -> None:
        try:
            for scan in scans:
                viz.draw(scan)

                if quit.is_set():
                    break

        finally:
            # signal main thread to exit on error
            viz.quit()

    try:
        print("Starting client thread...")
        client_thread = threading.Thread(target=run, name="Client")
        client_thread.start()

        print("Starting rendering loop...")
        viz.loop()
    finally:
        quit.set()
        client_thread.join()
        scans.close()

    print("Done")


def pcap_viz(file: str, meta_path: str) -> None:
    """Visualize data from a pcap file.

    To correctly visualize a pcap containing multiple UDP streams, you must
    specify a destination port. All packets recorded with a different
    destination port will be filtered out.
    """

    import ouster.pcap as pcap

    with open(meta_path) as json:
        info = client.SensorInfo(json.read())
    viz = PyViz(info)

    quit = threading.Event()
    source = pcap.Pcap(file,
                       info, rate=1.0)

    # pcap replay thread
    def run() -> None:
        try:
            print("Replaying...")
            for scan in client.Scans(source):
                viz.draw(scan)
                if quit.is_set():
                    raise StopIteration()
            source.reset()
        except StopIteration:
            pass
        finally:
            # signal main thread to exit
            viz.quit()

    try:
        print("Starting replay thread...")
        replay_thread = threading.Thread(target=run, name="Replay")
        replay_thread.start()

        print("Starting rendering loop...")
        viz.loop()
    finally:
        quit.set()
        replay_thread.join()
        source.close()

    print("Done")


def main() -> None:
    descr = """

    Visualize pcap or sensor data using python bindings of simple viz."""

    parser = argparse.ArgumentParser(description=descr)

    required = parser.add_argument_group('one of the following is required')
    group = required.add_mutually_exclusive_group(required=True)
    group.add_argument('--sensor', metavar='HOST', help='sensor hostname')
    group.add_argument('--pcap', metavar='PATH', help='path to pcap file')

    parser.add_argument('--meta', metavar='PATH', help='path to metadata json')
    parser.add_argument('--lidar-port', type=int, default=7502, help='Lidar port')

    args = parser.parse_args()

    if args.sensor:
        sensor_viz(args.sensor, args.lidar_port)
    elif args.pcap:
        if args.meta:
            metadata_path = args.meta
        else:
            print('Deducing metadata based on pcap name. To provide a different metadata path, use --meta')
            metadata_path = os.path.splitext(args.pcap)[0] + ".json"
        pcap_viz(args.pcap, metadata_path)


if __name__ == "__main__":
    main()

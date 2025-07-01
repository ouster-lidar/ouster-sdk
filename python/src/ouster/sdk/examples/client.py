"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Executable examples for using the sensor client APIs.

This module has a rudimentary command line interface. For usage, run::

    $ python -m ouster.sdk.examples.client -h
"""

import argparse
from contextlib import closing

import numpy as np

from ouster.sdk import core, sensor
from ouster.sdk.core import LidarMode


def configure_dual_returns(hostname: str) -> None:
    """Configure sensor to use dual returns profile given hostname

    Args:
        hostname: hostname of the sensor
    """
    config = sensor.get_config(hostname)
    if (config.lidar_mode in {LidarMode.MODE_2048x10, core.LidarMode.MODE_1024x20, core.LidarMode.MODE_4096x5}):
        print(
            f"Changing lidar_mode from {str(config.lidar_mode)} to 1024x10 to"
            " enable to dual returns on FW < 2.5. Will not persist change.")
        config.lidar_mode = core.LidarMode.MODE_1024x10

    # [doc-stag-config-udp-profile]
    config.udp_profile_lidar = core.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL
    # [doc-etag-config-udp-profile]

    try:
        sensor.set_config(hostname, config, persist=False, udp_dest_auto=False)
    except ValueError:
        print("error: Your sensor does not support dual returns. Please"
              " check the hardware revision and firmware version vs release"
              " notes.")
        return

    print("Retrieving sensor metadata..")
    with closing(sensor.SensorScanSource(hostname)) as source:
        # print some useful info from
        print(
            f"udp profile lidar: {str(source.sensor_info[0].format.udp_profile_lidar)}"
        )


def configure_sensor_params(hostname: str) -> None:
    """Configure sensor params given hostname

    Args:
        hostname: hostname of the sensor
    """

    # [doc-stag-configure]
    # create empty config
    config = core.SensorConfig()

    # set the values that you need: see sensor documentation for param meanings
    config.operating_mode = core.OperatingMode.OPERATING_NORMAL
    config.lidar_mode = core.LidarMode.MODE_1024x10
    config.udp_port_lidar = 7502
    config.udp_port_imu = 7503

    # set the config on sensor, using appropriate flags
    sensor.set_config(hostname, config, persist=True, udp_dest_auto=True)
    # [doc-etag-configure]

    # if you like, you can view the entire set of parameters
    config = sensor.get_config(hostname)
    print(f"sensor config of {hostname}:\n{config}")


def fetch_metadata(hostname: str) -> None:
    """Fetch metadata from a sensor and write it to disk.

    Accurately reconstructing point clouds from a sensor data stream
    requires access to sensor calibration and per-run configuration
    like the operating mode and azimuth window.

    The client API makes it easy to read metadata and write it to disk
    for use with recorded data streams.

    Args:
        hostname: hostname of the sensor
    """
    # [doc-stag-fetch-metadata]
    with closing(sensor.SensorScanSource(hostname)) as source:
        metadata = source.sensor_info[0]
        # print some useful info from metadata
        print("Retrieved metadata:")
        print(f"  serial no:        {metadata.sn}")
        print(f"  firmware version: {metadata.fw_rev}")
        print(f"  product line:     {metadata.prod_line}")
        print(f"  lidar mode:       {metadata.config.lidar_mode}")
        print(f"Writing to: {hostname}.json")

        # write metadata to disk
        with open(f"{hostname}.json", "w") as f:
            f.write(metadata.to_json_string())
    # [doc-etag-fetch-metadata]


def filter_3d_by_range_and_azimuth(hostname: str,
                                   lidar_port: int = 7502,
                                   range_min: int = 2) -> None:
    """Easily filter 3D Point Cloud by Range and Azimuth Using the 2D Representation

    Args:
        hostname: hostname of sensor
        lidar_port: UDP port to listen on for lidar data
        range_min: range minimum in meters
    """
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except ModuleNotFoundError:
        print("This example requires matplotlib and an appropriate Matplotlib "
            "GUI backend such as TkAgg or Qt5Agg.")
        exit(1)
    import math

    # set up figure
    plt.figure()
    ax = plt.axes(projection='3d')
    r = 3
    ax.set_xlim3d([-r, r])  # type: ignore
    ax.set_ylim3d([-r, r])  # type: ignore
    ax.set_zlim3d([-r, r])  # type: ignore

    plt.title("Filtered 3D Points from {}".format(hostname))
    # [doc-stag-filter-3d]
    source = sensor.SensorScanSource(hostname, lidar_port=lidar_port)
    metadata = source.sensor_info[0]
    scan = next(iter(source))[0]
    assert scan is not None
    source.close()

    # obtain destaggered range
    range_destaggered = core.destagger(metadata,
                                       scan.field(core.ChanField.RANGE))

    # obtain destaggered xyz representation
    xyzlut = core.XYZLut(metadata)
    xyz_destaggered = core.destagger(metadata, xyzlut(scan))

    # select only points with more than min range using the range data
    xyz_filtered = xyz_destaggered * (range_destaggered[:, :, np.newaxis] >
                                      (range_min * 1000))

    # get first 3/4 of scan
    to_col = math.floor(metadata.format.columns_per_frame * 3 / 4)
    xyz_filtered = xyz_filtered[:, 0:to_col, :]
    # [doc-etag-filter-3d]

    [x, y, z] = [c.flatten() for c in np.dsplit(xyz_filtered, 3)]
    ax.scatter(x, y, z, c=z / max(z), s=0.2)  # type: ignore
    plt.show()


def live_plot_reflectivity(hostname: str, lidar_port: int = 7502) -> None:
    """Display reflectivity from live sensor

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data

    """
    import cv2  # type: ignore

    print("press ESC from visualization to exit")

    # [doc-stag-live-plot-reflectivity]
    # establish sensor connection
    from ouster.sdk import sensor
    with closing(sensor.SensorScanSource(hostname, lidar_port=lidar_port)) as stream:
        show = True
        while show:
            for scan, *_ in stream:
                if scan is None:
                    continue
                # uncomment if you'd like to see frame id printed
                # print("frame id: {} ".format(scan.frame_id))
                reflectivity = core.destagger(stream.sensor_info[0],
                                          scan.field(core.ChanField.REFLECTIVITY))
                reflectivity = (reflectivity / np.max(reflectivity) * 255).astype(np.uint8)
                cv2.imshow("scaled reflectivity", reflectivity)
                key = cv2.waitKey(1) & 0xFF
                # [doc-etag-live-plot-reflectivity]
                # 27 is esc
                if key == 27:
                    show = False
                    break
        cv2.destroyAllWindows()


def plot_xyz_points(hostname: str, lidar_port: int = 7502) -> None:
    """Display range from a single scan as 3D points

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data
    """
    import matplotlib.pyplot as plt  # type: ignore

    # get single scan
    source = sensor.SensorScanSource(hostname, lidar_port=lidar_port)
    scan = next(iter(source))[0]
    assert scan is not None
    metadata = source.sensor_info[0]
    source.close()

    # set up figure
    plt.figure()
    ax = plt.axes(projection='3d')
    r = 3
    ax.set_xlim3d([-r, r])  # type: ignore
    ax.set_ylim3d([-r, r])  # type: ignore
    ax.set_zlim3d([-r, r])  # type: ignore

    plt.title("3D Points from {}".format(hostname))

    # [doc-stag-plot-xyz-points]
    # transform data to 3d points
    xyzlut = core.XYZLut(metadata)
    xyz = xyzlut(scan.field(core.ChanField.RANGE))
    # [doc-etag-plot-xyz-points]

    # graph xyz
    [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    ax.scatter(x, y, z, c=z / max(z), s=0.2)  # type: ignore
    plt.show()


def record_pcap(hostname: str,
                lidar_port: int = 7502,
                imu_port: int = 7503,
                n_seconds: int = 10) -> None:
    """Record data from live sensor to pcap file.

    Note that pcap files recorded this way only preserve the UDP data stream and
    not networking information, unlike capturing packets directly from a network
    interface with tools like tcpdump or wireshark.

    See the API docs of :py:func:`.pcap.record` for additional options for
    writing pcap files.

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data
        imu_port: UDP port to listen on for imu data
        n_seconds: max seconds of time to record. (Ctrl-Z correctly closes
                   streams)
    """
    # [doc-stag-pcap-record]
    from ouster.sdk import sensor
    import ouster.sdk.pcap as pcap
    from datetime import datetime
    from more_itertools import time_limited
    # connect to sensor and record lidar/imu packets
    with closing(sensor.SensorPacketSource(hostname, lidar_port=lidar_port, imu_port=imu_port,
                               buffer_time_sec=1.0)) as source:

        # make a descriptive filename for metadata/pcap files
        time_part = datetime.now().strftime("%Y%m%d_%H%M%S")
        meta = source.sensor_info[0]
        fname_base = f"{meta.prod_line}_{meta.sn}_{meta.config.lidar_mode}_{time_part}"

        print(f"Saving sensor metadata to: {fname_base}.json")
        with open(f"{fname_base}.json", "w") as f:
            f.write(source.sensor_info[0].to_json_string())

        print(f"Writing to: {fname_base}.pcap (Ctrl-C to stop early)")
        source_it = time_limited(n_seconds, source)

        def to_packet():
            for idx, packet in source_it:
                yield packet
        n_packets = pcap.record(to_packet(), f"{fname_base}.pcap")

        print(f"Captured {n_packets} packets")
    # [doc-etag-pcap-record]


def main() -> None:
    examples = {
        "configure-dual-returns": configure_dual_returns,
        "configure-sensor": configure_sensor_params,
        "fetch-metadata": fetch_metadata,
        "filter-3d-by-range-and-azimuth": filter_3d_by_range_and_azimuth,
        "live-plot-reflectivity": live_plot_reflectivity,
        "plot-xyz-points": plot_xyz_points,
        "record-pcap": record_pcap,
    }

    description = "Ouster Python SDK examples. The EXAMPLE must be one of:\n  " + str.join(
        '\n  ', examples.keys())

    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('hostname',
                        metavar='HOSTNAME',
                        type=str,
                        help='Sensor hostname, e.g. "os-122033000087"')
    parser.add_argument('example',
                        metavar='EXAMPLE',
                        choices=examples.keys(),
                        type=str,
                        help='Name of the example to run')

    args = parser.parse_args()

    try:
        example = examples[args.example]
    except KeyError:
        print(f"No such example: {args.example}")
        exit(1)

    print(f"example: {args.example}")
    example(args.hostname)  # type: ignore


if __name__ == "__main__":
    main()

# flake8: noqa: E303 (E303 too many blank lines)
"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Executable examples for using the core sensor APIs.

This module has a rudimentary command line interface. For usage, run::

    $ python -m ouster.sdk.examples.core -h
"""

import argparse
from contextlib import closing

import numpy as np

# [doc-stag-config-imports]
# Other imports
from ouster.sdk import core
from ouster.sdk import sensor
# [doc-etag-config-imports]


def configure_dual_returns(hostname: str) -> None:
    """Configure sensor to use dual returns profile given hostname

    Args:
        hostname: hostname of the sensor
    """
    config = sensor.get_config(hostname)
    if (config.lidar_mode in {core.LidarMode._2048x10, core.LidarMode._1024x20, core.LidarMode._4096x5}):
        print(
            f"Changing lidar_mode from {str(config.lidar_mode)} to 1024x10 to"
            " enable to dual returns on FW < 2.5. Will not persist change.")
        config.lidar_mode = core.LidarMode._1024x10

    # [doc-stag-config-udp-profile]
    config.udp_profile_lidar = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    # [doc-etag-config-udp-profile]

    # [doc-stag-config-timing]
    # or TIME_FROM_PTP_1588, TIME_FROM_SYNC_PULSE_IN
    config.timestamp_mode = core.TimestampMode.TIME_FROM_INTERNAL_OSC
    config.multipurpose_io_mode = core.MultipurposeIOMode.OUTPUT_FROM_INTERNAL_OSC
    config.nmea_baud_rate = core.NMEABaudRate.BAUD_9600
    # [doc-etag-config-timing]

    try:
        config.udp_dest = "@auto"
        sensor.set_config(hostname, config, persist=False)
    except ValueError:
        print("error: Your sensor does not support dual returns. Please"
              " check the hardware revision and firmware version vs release"
              " notes.")
        return

    print("Retrieving sensor info..")
    with closing(sensor.SensorFrameSetSource(hostname)) as source:
        # print some useful info from
        print(
            f"udp profile lidar: {str(source.sensor_info[0].format.udp_profile_lidar)}"
        )


def configure_sensor_params(sensor_hostname: str) -> None:
    """Configure sensor params given hostname

    Args:
        hostname: hostname of the sensor
    """
    # [doc-stag-get-config]
    config: core.SensorConfig = sensor.get_config(sensor_hostname)
    print(f"Sensor config of {sensor_hostname}:\n" +
          f"{str(config)}")
    # [doc-etag-get-config]
    # create empty config
    # [doc-stag-make-config]
    # set the values that you need: see sensor documentation for param meanings
    config.operating_mode = core.OperatingMode.NORMAL
    config.lidar_mode = core.LidarMode._1024x10
    config.udp_dest = "@auto"
    # [doc-etag-make-config]
    config.udp_port_lidar = 7502
    config.udp_port_imu = 7503
    # [doc-stag-set-config]
    persist = True

    sensor.set_config(sensor_hostname, config, persist)
    # [doc-etag-set-config]
    print("Configuration set successfully. Handle failure")

    config_updated = sensor.get_config(sensor_hostname)
    print(f"Sensor config of {sensor_hostname}:\n" +
          f"{str(config_updated)}")


def fetch_sensor_info(sensor_hostname: str) -> None:
    """Fetch sensor info from a sensor and write it to disk.

    Accurately reconstructing point clouds from a sensor data stream
    requires access to sensor calibration and per-run configuration
    like the operating mode and azimuth window.

    The client API makes it easy to read sensor info and write it to disk
    for use with recorded data streams.

    Args:
        hostname: hostname of the sensor
    """

    # [doc-stag-fetch-sensor-info]
    with closing(sensor.SensorFrameSetSource(sensor_hostname)) as source:
        sensor_info = source.sensor_info[0]
        print("Retrieved sensor info:\n")
        print(f"  serial no:        {sensor_info.sn}")
        print(f"  firmware version: {sensor_info.fw_rev}")
        print(f"  product line:     {sensor_info.prod_line}")
        print(f"  lidar mode:       {sensor_info.config.lidar_mode}")
        print(f"  columns/frame:    {sensor_info.format.columns_per_frame}")
        print(f"  beam angles:      {len(sensor_info.beam_altitude_angles)}")
        print(f"  altitude:         {len(sensor_info.beam_azimuth_angles)}")
        print(f"Writing to: {sensor_hostname}.json")
        with open(f"{sensor_hostname}.json", "w") as f:
            f.write(sensor_info.to_json_string())
    # [doc-etag-fetch-sensor-info]


# [doc-stag-filter-3d-full]
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
    source = sensor.SensorFrameSetSource(hostname, lidar_port=lidar_port)
    info = source.sensor_info[0]
    frame = next(iter(source))[0]
    assert frame is not None
    source.close()

    # [doc-stag-filter-3d-setup]
    azimuth_fraction = 0.75
    # destagger RANGE so each column maps to a fixed azimuth.
    range_field = frame.field(core.ChanField.RANGE)
    range_destaggered = core.destagger(info,
                                       range_field)

    # obtain destaggered xyz representation
    xyzlut = core.XYZLut(info, use_extrinsics=True)
    cloud = xyzlut(frame)
    # [doc-etag-filter-3d-setup]
    # [doc-stag-filter-3d-destagger]
    # cloud shape is (H, W, 3)
    # SDK python wrapper applies the destaggering
    # logic to each channel
    xyz_destaggered = core.destagger(info, cloud)

    # [doc-etag-filter-3d-destagger]
    # [doc-stag-filter-3d-mask]
    min_range_mm = range_min * 1000.0  # corrected variable name
    # 1. Create mask: True if range > min, else False
    # NumPy arrays support element-wise comparison
    mask = (range_destaggered[:, :, np.newaxis] > min_range_mm)

    # 2. Apply mask via multiplication
    # Element-wise multiplication zeros out invalid points
    xyz_filtered = xyz_destaggered * mask


    # [doc-etag-filter-3d-mask]
    # [doc-stag-filter-3d]
    # 3. Slicing: Limit to the first azimuth_fraction
    col_limit = math.floor(info.format.columns_per_frame * azimuth_fraction)
    # Standard NumPy slicing [row, col, channel]
    xyz_filtered = xyz_filtered[:, 0:col_limit, :]



    # [doc-etag-filter-3d]

    [x, y, z] = [c.flatten() for c in np.dsplit(xyz_filtered, 3)]
    ax.scatter(x, y, z, c=z / max(z), s=0.2)  # type: ignore
    plt.show()
# [doc-etag-filter-3d-full]


def live_plot_reflectivity(hostname: str, lidar_port: int = 7502) -> None:
    """Display reflectivity from live sensor

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data

    """
    import cv2  # type: ignore

    print("press ESC from visualization to exit")

    # [doc-stag-live-plot-reflectivity]
    # establish sensor connection via open_source
    from ouster.sdk import open_source
    with closing(open_source(hostname,
                             collate=False,
                             sensor_idx=0,
                             lidar_port=lidar_port)) as stream:
        show = True
        while show:
            for frames in stream:
                frame = frames[0]
                if frame is None:
                    continue
                # uncomment if you'd like to see frame id printed
                # print("frame id: {} ".format(frame.frame_id))
                reflectivity = core.destagger(stream.sensor_info[0],
                                              frame.field(core.ChanField.REFLECTIVITY))
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
    """Display range from a single frame as 3D points

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data
    """
    import matplotlib.pyplot as plt  # type: ignore

    # get single frame
    source = sensor.SensorFrameSetSource(hostname, lidar_port=lidar_port)
    frame = next(iter(source))[0]
    assert frame is not None
    sensor_info = source.sensor_info[0]
    source.close()

    # set up figure
    plt.figure()
    ax = plt.axes(projection='3d')
    r = 3
    ax.set_xlim3d([-r, r])  # type: ignore
    ax.set_ylim3d([-r, r])  # type: ignore
    ax.set_zlim3d([-r, r])  # type: ignore

    plt.title("3D Points from {}".format(hostname))
    # transform data to 3d points
    # [doc-stag-plot-xyz-imports]
    # Other imports
    # Other imports
    # Other imports
    from ouster.sdk import core
    # [doc-etag-plot-xyz-imports]
    # [doc-stag-plot-xyz-points]
    # source = sensor.SensorFrameSetSource(sensor_hostname)
    # sensor_info = source.sensor_info[0]
    # frame = next(iter(source))[0]
    xyzlut = core.XYZLut(sensor_info)
    range = frame.field(core.ChanField.RANGE)
    cloud = xyzlut(range)
    # [doc-etag-plot-xyz-points]

    # graph xyz
    [x, y, z] = [c.flatten() for c in np.dsplit(cloud, 3)]
    ax.scatter(x, y, z, c=z / max(z), s=0.2)  # type: ignore
    plt.show()


def get_fname_base(info: core.SensorInfo) -> str:
    from datetime import datetime
    time_part = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{info.prod_line}_{info.sn}_{info.config.lidar_mode}_{time_part}"


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
    # [doc-stag-pcap-write-imports]
    # Other imports
    from more_itertools import time_limited
    from ouster.sdk import open_packet_source
    from ouster.sdk import pcap
    # [doc-etag-pcap-write-imports]
    # [doc-stag-pcap-write-setup]
    # connect to sensor and record lidar/imu packets
    with closing(open_packet_source(hostname,
                                    lidar_port=lidar_port,
                                    imu_port=imu_port,
                                    buffer_time_sec=1.0
                                    )) as source:

        info = source.sensor_info[0]

        # make a descriptive filename for metadata/pcap files
        fname_base = get_fname_base(info)

        print(f"Saving sensor info to: {fname_base}.json")
        with open(f"{fname_base}.json", "w") as f:
            f.write(source.sensor_info[0].to_json_string())
        print(f"Writing to: {fname_base}.pcap (Ctrl-C to stop early)")
    # [doc-etag-pcap-write-setup]
    # [doc-stag-pcap-write]
        # int n_seconds = 10
        # Yield items from iterable until n_seconds have passed.
        source_it = time_limited(n_seconds, source)

        def to_packet():
            for idx, packet in source_it:
                yield packet

        # Python API exposes pcap.record(), a convenience helper that accepts an
        # iterable of packets and handles buffer management/timestamping. See:
        #   ouster-sdk/python/src/ouster/sdk/pcap/pcap.py (record())
        # C++ performs the same work via pcap::record_* helpers. See:
        #   ouster_pcap/include/ouster/os_pcap.h.
        n_packets = pcap.record(
            to_packet(),
            f"{fname_base}.pcap"
        )

        print(f"Captured {n_packets} packets")
    # [doc-etag-pcap-write]


def main() -> None:
    examples = {
        "configure-dual-returns": configure_dual_returns,
        "configure-sensor": configure_sensor_params,
        "fetch-sensor-info": fetch_sensor_info,
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

"""Example code for Ouster Python SDK

All examples commented out from main. Feel free to uncomment to try.

Note: if you want to run matplotlib within docker you will need tkinter
"""

import argparse
from more_itertools import time_limited
from contextlib import closing

import numpy as np
import cv2

import matplotlib.pyplot as plt

from ouster import client

def configure_sensor_params(hostname: str) -> None:
    """Configure sensor params given hostname

    Args:
        hostname: hostname of the sensor
    """
    print("\nexample: configure_sensor_params")

    # create empty config
    config = client.SensorConfig()

    # set the values that you need: see sensor docs for param meanings
    config.phase_lock_enable = True
    config.ld_mode = client.LidarMode.MODE_1024x10

    # set the config on sensor, using persist bool if desired
    client.set_config(hostname, config, persist=True)

    # if you like, you can view the entire set of parameters
    config = client.get_config(hostname)
    print("sensor config of {}\n: ".format(hostname), config)


def get_metadata(hostname: str) -> client.SensorInfo:
    """Print metadata given hostname

    Args:
        hostname: hostname of the sensor
    """

    return client.Sensor(hostname).metadata


def display_range_2d(hostname: str, lidar_port: int) -> None:
    """Display range data taken live from sensor as an image

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data
    """
    print("example: display_range_2d")

    # get single scan [doc-stag-single-scan]
    sample = client.Scans.sample(hostname, 1, lidar_port)
    scan = next(sample)[0]
    # [doc-etag-single-scan]

    # initialize plot
    fig, ax = plt.subplots()
    fig.canvas.set_window_title("example: display_range_2d")

    # plot using imshow
    plt.imshow(client.destagger(
        client.Sensor(hostname).metadata, scan.field(client.ChanField.RANGE)),
               resample=False)

    # configure and show plot
    plt.title("Range Data from {}".format(hostname))
    plt.axis('off')
    plt.show()


def display_all_2d(hostname: str, lidar_port: int, n_scans: int = 5) -> None:
    """Display all channels of n consecutive lidar scans taken live from sensor

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data
        n_scans: number of scans to show
    """
    print("example: display_all_2d")

    # [doc-stag-display-all-2d]
    # take sample of n scans from sensor
    sample = client.Scans.sample(hostname, n_scans, lidar_port)
    metadata = client.Sensor(hostname).metadata

    # initialize and configure subplots
    fig, axarr = plt.subplots(n_scans,
                              4,
                              sharex=True,
                              sharey=True,
                              figsize=(12.0, n_scans * .75),
                              tight_layout=True)
    fig.suptitle("{} consecutive scans from {}".format(n_scans, hostname))
    fig.canvas.set_window_title("example: display_all_2D")

    # set row and column titles of subplots
    column_titles = ["range", "reflectivity", "ambient", "intensity"]
    row_titles = ["Scan {}".format(i) for i in list(range(n_scans))]
    for ax, column_title in zip(axarr[0], column_titles):
        ax.set_title(column_title)
    for ax, row_title in zip(axarr[:, 0], row_titles):
        ax.set_ylabel(row_title)

    # plot 2D scans
    for count, scan in enumerate(next(sample)):
        axarr[count, 0].imshow(
            client.destagger(metadata, scan.field(client.ChanField.RANGE)))
        axarr[count, 1].imshow(
            client.destagger(metadata,
                             scan.field(client.ChanField.REFLECTIVITY)))
        axarr[count, 2].imshow(
            client.destagger(metadata, scan.field(client.ChanField.AMBIENT)))
        axarr[count, 3].imshow(
            client.destagger(metadata, scan.field(client.ChanField.INTENSITY)))

    # configure and show plot
    [ax.get_xaxis().set_visible(False) for ax in axarr.ravel()]
    [ax.set_yticks([]) for ax in axarr.ravel()]
    [ax.set_yticklabels([]) for ax in axarr.ravel()]
    plt.show()
    # [doc-etag-display-all-2d]


def display_intensity_live(hostname: str, lidar_port: int) -> None:
    """
    Display intensity from live sensor

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data

    """
    print("example: display_intensity_scaled_2d")
    print("\tpress ESC from visualization to exit")

    # establish sensor connection
    with closing(client.Scans.stream(hostname, lidar_port,
                                     complete=False)) as stream:
        show = True
        while show:
            for scan in stream:
                # uncomment if you'd like to see frame id printed
                # print("frame id: {} ".format(scan.frame_id))
                signal = client.destagger(
                    stream.metadata, scan.field(client.ChanField.INTENSITY))
                signal = (signal / np.max(signal) * 255).astype(np.uint8)
                cv2.imshow("scaled intensity", signal)
                key = cv2.waitKey(1) & 0xFF
                # 27 is esc
                if key == 27:
                    show = False
                    break
        cv2.destroyAllWindows()


def display_xyz_points(hostname: str, lidar_port: int) -> None:
    """Display range from a single scan as 3D points

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data
    """

    print("example: display_xyz_points")

    # get single scan
    metadata = client.Sensor(hostname).metadata
    xyzlut = client.XYZLut(metadata)
    sample = client.Scans.sample(hostname, 1, lidar_port, metadata=metadata)
    scan = next(sample)[0]

    # set up figure
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    fig.canvas.set_window_title("example: display_xyz_points")
    plt.title("3D Points from {}".format(hostname))

    # transform data to 3d points and graph
    xyz = xyzlut(scan)
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2])
    plt.show()


def write_xyz_to_csv(hostname: str,
                     lidar_port: int,
                     cloud_prefix: str,
                     n_scans: int = 5) -> None:
    """Write xyz sample from live sensor to csv

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data
        cloud_prefix: prefix to written out csvs
        n_scans: number of scans to write
    """

    print("example: write_xyz_to_csv")
    metadata = client.Sensor(hostname).metadata
    xyzlut = client.XYZLut(metadata)
    sample = client.Scans.sample(hostname,
                                 n_scans,
                                 lidar_port,
                                 metadata=metadata)

    for count, scan in enumerate(next(sample)):
        out_name = "{}_{}.txt".format(cloud_prefix, count)
        print("writing {}..".format(out_name))
        np.savetxt(out_name, xyzlut(scan), delimiter=" ")


def plot_imu_z_acc_over_time(hostname: str,
                             lidar_port: int,
                             imu_port: int,
                             n_seconds: int = 10) -> None:
    """Plot the z acceleration from the IMU over time

    Args:
        hostname: hostname of the sensor
        imu_port: UDP port to listen on for imu data
        n_seconds: seconds of time to take a sample over
    """
    print("example: plot_imu_z_acc_over_time")

    # connect to sensor and get imu packets within n_seconds
    source = client.Sensor(hostname, lidar_port, imu_port)
    with closing(source):
        ts, z_accel = zip(*[(p.sys_ts, p.accel[2])
                            for p in list(time_limited(n_seconds, source))
                            if isinstance(p, client.ImuPacket)])

    # initialize plot
    fig, ax = plt.subplots(figsize=(12.0, 2))
    ax.plot(ts, z_accel)

    fig.canvas.set_window_title("example: plot_imu_z_acc_over_time")
    plt.title("Z Accel from IMU over {} Seconds".format(n_seconds))
    ax.set_xticks(np.arange(min(ts), max(ts), step=((max(ts) - min(ts)) / 5)))
    # add end ticker to x axis
    ax.set_xticks(list(ax.get_xticks()) + [max(ts)])

    ax.set_xlim([min(ts), max(ts)])
    ax.set_ylabel("z accel")
    ax.set_xlabel("timestamp (ns)")

    ax.ticklabel_format(useOffset=False, style="plain")
    plt.show()

def main():
    "Parse arguments and pass them to various examples"
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--hostname',
                        dest='hostname',
                        type=str,
                        help='sensor hostname',
                        required=True)
    parser.add_argument('--lidar_port',
                        dest='lidar_port',
                        default=7502,
                        type=int,
                        help='UDP port to listen on for lidar data')
    parser.add_argument('--imu_port',
                        dest='imu_port',
                        default=7503,
                        type=int,
                        help='UDP port to listen on for imu data')

    args = parser.parse_args()

    # configure_sensor_params(args.hostname)
    # print(get_metadata(args.hostname))
    # display_range_2d(args.hostname, args.lidar_port)
    # display_all_2d(args.hostname, args.lidar_port, n_scans = 20)
    # display_intensity_live(args.hostname, args.lidar_port)
    # display_xyz_points(args.hostname, args.lidar_port)
    # write_xyz_to_csv(args.hostname, args.lidar_port,
    #                         cloud_prefix = 'xyz',
    #                         n_scans = 5)
    # plot_imu_z_acc_over_time(args.hostname,
    #                         args.lidar_port,
    #                         args.imu_port,
    #                         n_seconds=6)

if __name__ == "__main__":
    main()

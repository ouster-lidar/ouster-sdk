"""Executable examples for using the pcap APIs.

This module has a rudimentary command line interface. For usage, run::

    $ python -m ouster.sdk.examples.pcap -h
"""
import os
import argparse
from contextlib import closing
from itertools import islice
import time

import numpy as np
from more_itertools import nth

from ouster import client, pcap
from .colormaps import normalize


def pcap_display_xyz_points(source: client.PacketSource,
                            metadata: client.SensorInfo,
                            num: int = 0) -> None:
    """Plot point cloud using matplotlib."""
    import matplotlib.pyplot as plt  # type: ignore

    # [doc-stag-pcap-plot-xyz-points]
    scan = nth(client.Scans(source), num)
    if not scan:
        print(f"ERROR: Scan # {num} in not present in pcap file")
        exit(1)

    # set up figure
    plt.figure()
    ax = plt.axes(projection='3d')
    r = 6
    ax.set_xlim3d([-r, r])
    ax.set_ylim3d([-r, r])
    ax.set_zlim3d([-r, r])

    plt.title("3D Points XYZ for scan")

    # transform data to 3d points and graph
    xyzlut = client.XYZLut(metadata)
    xyz = xyzlut(scan)

    key = scan.field(client.ChanField.SIGNAL)

    [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    ax.scatter(x, y, z, c=normalize(key.flatten()), s=0.2)
    plt.show()
    # [doc-etag-pcap-plot-xyz-points]


def pcap_2d_viewer(source: client.PacketSource,
                   metadata: client.SensorInfo,
                   num: int = 0) -> None:
    """Visualize channel fields in 2D using opencv."""
    import cv2  # type: ignore

    # [doc-stag-pcap-display-live]
    print("press ESC from visualization to exit")

    quit = False
    paused = False
    destagger = True
    num = 0
    for scan in client.Scans(source):
        print("frame id: {}, num = {}".format(scan.frame_id, num))

        fields = [scan.field(ch) for ch in client.ChanField]
        if destagger:
            fields = [client.destagger(metadata, f) for f in fields]

        combined_images = np.vstack(
            [np.pad(normalize(f), 2, constant_values=1.0) for f in fields])

        cv2.imshow("4 channels: ", combined_images)

        # handle keys presses
        while True:
            key = cv2.waitKey(1) & 0xFF
            # 100 is d
            if key == 100:
                destagger = not destagger
            # 32 is SPACE
            if key == 32:
                paused = not paused
            # 27 is ESC
            elif key == 27:
                quit = True

            if not paused:
                break
            time.sleep(0.1)

        if quit:
            break
        num += 1

    cv2.destroyAllWindows()
    # [doc-etag-pcap-display-live]


def pcap_read_packets(
        source: client.PacketSource,
        metadata: client.SensorInfo,
        num: int = 0  # not used in this examples
) -> None:
    """Basic read packets example from pcap file. """
    # [doc-stag-pcap-read-packets]
    for packet in source:
        if isinstance(packet, client.LidarPacket):
            # Now we can process the LidarPacket. In this case, we access
            # the encoder_counts, timestamps, and ranges
            encoder_counts = packet.header(client.ColHeader.ENCODER_COUNT)
            timestamps = packet.header(client.ColHeader.TIMESTAMP)
            ranges = packet.field(client.ChanField.RANGE)
            print(f'  encoder counts = {encoder_counts.shape}')
            print(f'  timestamps = {timestamps.shape}')
            print(f'  ranges = {ranges.shape}')

        elif isinstance(packet, client.ImuPacket):
            # and access ImuPacket content
            print(f'  acceleration = {packet.accel}')
            print(f'  angular_velocity = {packet.angular_vel}')
    # [doc-etag-pcap-read-packets]


def pcap_show_one_scan(source: client.PacketSource,
                       metadata: client.SensorInfo,
                       num: int = 0,
                       destagger: bool = True) -> None:
    """Plot all channels of one scan in 2D using matplotlib."""
    import matplotlib.pyplot as plt  # type: ignore

    scan = nth(client.Scans(source), num)
    if not scan:
        print(f"ERROR: Scan # {num} in not present in pcap file")
        exit(1)

    # [doc-stag-pcap-show-one]
    fig = plt.figure(constrained_layout=True)
    axs = fig.subplots(len(client.ChanField), 1, sharey=True)

    for ax, field in zip(axs, client.ChanField):
        img = normalize(scan.field(field))
        if destagger:
            img = client.destagger(metadata, img)

        ax.set_title(str(field), fontdict={'fontsize': 10})
        ax.imshow(img, cmap='gray', resample=False)
        ax.set_yticklabels([])
        ax.set_yticks([])
        ax.set_xticks([0, scan.w])
    plt.show()
    # [doc-etag-pcap-show-one]


def pcap_to_csv(source: client.PacketSource,
                metadata: client.SensorInfo,
                num: int = 0,
                csv_dir: str = ".",
                csv_base: str = "pcap_out",
                csv_ext: str = "csv") -> None:
    """Write scans from a pcap to csv files (one per lidar scan).

    The number of saved lines per csv file is always H x W, which corresponds to
    a full 2D image representation of a lidar scan.

    Each line in a csv file is:

        TIMESTAMP, RANGE (mm), SIGNAL, NEAR_IR, REFLECTIVITY, X (mm), Y (mm), Z (mm)

    If ``csv_ext`` ends in ``.gz``, the file is automatically saved in
    compressed gzip format. :func:`.numpy.loadtxt` can be used to read gzipped
    files transparently back to :class:`.numpy.ndarray`.

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        num: number of scans to save from pcap to csv files
        csv_dir: path to the directory where csv files will be saved
        csv_base: string to use as the base of the filename for pcap output
        csv_ext: file extension to use, "csv" by default
    """

    # ensure that base csv_dir exists
    if not os.path.exists(csv_dir):
        os.makedirs(csv_dir)

    field_names = 'TIMESTAMP (ns), RANGE (mm), SIGNAL, NEAR_IR, REFLECTIVITY, X (mm), Y (mm), Z (mm)'
    field_fmts = ['%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d']

    # [doc-stag-pcap-to-csv]
    # precompute xyzlut to save computation in a loop
    xyzlut = client.XYZLut(metadata)

    # create an iterator of LidarScans from pcap and bound it if num is specified
    scans = iter(client.Scans(source))
    if num:
        scans = islice(scans, num)

    for idx, scan in enumerate(scans):

        # copy per-column timestamps for each channel
        col_timestamps = scan.header(client.ColHeader.TIMESTAMP)
        timestamps = np.tile(col_timestamps, (scan.h, 1))

        # grab channel data
        fields_values = [scan.field(ch) for ch in client.ChanField]

        # use integer mm to avoid loss of precision casting timestamps
        xyz = (xyzlut(scan) * 1000).astype(np.int64)

        # get all data as one H x W x 8 int64 array for savetxt()
        frame = np.dstack((timestamps, *fields_values, xyz))

        # not necessary, but output points in "image" vs. staggered order
        frame = client.destagger(metadata, frame)

        # write csv out to file
        csv_path = os.path.join(csv_dir, f'{csv_base}_{idx:06d}.{csv_ext}')
        print(f'write frame #{idx}, to file: {csv_path}')

        header = '\n'.join([f'frame num: {idx}', field_names])

        np.savetxt(csv_path,
                   frame.reshape(-1, frame.shape[2]),
                   fmt=field_fmts,
                   delimiter=',',
                   header=header)
    # [doc-etag-pcap-to-csv]


def pcap_3d_one_scan(source: client.PacketSource,
                     metadata: client.SensorInfo,
                     num: int = 0) -> None:
    """Render one scan from a pcap file in the Open3D viewer.

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        num: scan number in a given pcap file (satrs from *0*)
    """
    import open3d as o3d

    # get single scan by index
    scan = nth(client.Scans(source), num)

    if not scan:
        print(f"ERROR: Scan # {num} in not present in pcap file")
        exit(1)

    # [doc-stag-open3d-one-scan]
    # compute point cloud using client.SensorInfo and client.LidarScan
    xyz = client.XYZLut(metadata)(scan)

    # create point cloud and coordinate axes geometries
    cloud = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(xyz.reshape((-1, 3))))
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(1.0)
    # [doc-etag-open3d-one-scan]

    # initialize visualizer and rendering options
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(cloud)
    vis.add_geometry(axes)
    ropt = vis.get_render_option()
    ropt.point_size = 1.0
    ropt.background_color = np.asarray([0, 0, 0])

    # initialize camera settings
    ctr = vis.get_view_control()
    ctr.set_zoom(0.1)
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([1, 0, 0])

    # run visualizer main loop
    print("Press Q or Excape to exit")
    vis.run()
    vis.destroy_window()


def main():
    """Pcap examples runner."""
    examples = {
        "plot-xyz-points": pcap_display_xyz_points,
        "2d-viewer": pcap_2d_viewer,
        "read-packets": pcap_read_packets,
        "plot-one-scan": pcap_show_one_scan,
        "pcap-to-csv": pcap_to_csv,
        "open3d-one-scan": pcap_3d_one_scan,
    }

    description = "Ouster Python SDK Pcap examples. The EXAMPLE must be one of:\n" + str.join(
        '\n  ', examples.keys())

    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('pcap_path', metavar='PCAP', help='path to pcap file')
    parser.add_argument('metadata_path',
                        metavar='METADATA',
                        help='path to metadata json')
    parser.add_argument('example',
                        metavar='EXAMPLE',
                        choices=examples.keys(),
                        help='name of the example to run')
    parser.add_argument('--scan-num',
                        type=int,
                        default=1,
                        help='index of scan to use')
    args = parser.parse_args()

    try:
        example = examples[args.example]
    except KeyError:
        print(f"No such example: {args.example}")
        print(description)
        exit(1)

    if not args.metadata_path or not os.path.exists(args.metadata_path):
        print(f"Metadata file does not exist: {args.metadata_path}")
        exit(1)

    print(f'example: {args.example}')

    with open(args.metadata_path, 'r') as f:
        metadata = client.SensorInfo(f.read())
    source = pcap.Pcap(args.pcap_path, metadata)

    with closing(source):
        example(source, metadata, args.scan_num)  # type: ignore


if __name__ == "__main__":
    main()

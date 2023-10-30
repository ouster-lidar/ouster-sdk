"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Executable examples for using the pcap APIs.

This module has a rudimentary command line interface. For usage, run::

    $ python -m ouster.sdk.examples.pcap -h
"""
import os
import argparse
from contextlib import closing
import numpy as np

from ouster import client, pcap
from .colormaps import normalize


def pcap_3d_one_scan(source: client.PacketSource,
                     metadata: client.SensorInfo,
                     num: int = 0) -> None:
    """Render one scan from a pcap file in the Open3D viewer.

    Args:
        source: PacketSource from pcap
        metadata: associated SensorInfo for PacketSource
        num: scan number in a given pcap file (satrs from *0*)
    """
    try:
        import open3d as o3d  # type: ignore
    except ModuleNotFoundError:
        print(
            "This example requires open3d, which may not be available on all "
            "platforms. Try running `pip3 install open3d` first.")
        exit(1)

    from more_itertools import nth
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
        o3d.utility.Vector3dVector(xyz.reshape((-1, 3))))  # type: ignore
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
        1.0)  # type: ignore

    # [doc-etag-open3d-one-scan]

    # initialize visualizer and rendering options
    vis = o3d.visualization.Visualizer()  # type: ignore

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
    print("Press Q or Escape to exit")
    vis.run()
    vis.destroy_window()


def pcap_display_xyz_points(source: client.PacketSource,
                            metadata: client.SensorInfo,
                            num: int = 0) -> None:
    """Plot point cloud using matplotlib."""
    import matplotlib.pyplot as plt  # type: ignore

    # [doc-stag-pcap-plot-xyz-points]
    from more_itertools import nth
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
    xyz = xyzlut(scan.field(client.ChanField.RANGE))

    key = scan.field(client.ChanField.REFLECTIVITY)

    [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    ax.scatter(x, y, z, c=normalize(key.flatten()), s=0.2)
    plt.show()
    # [doc-etag-pcap-plot-xyz-points]


def pcap_to_las(source: client.PacketSource,
                metadata: client.SensorInfo,
                num: int = 0,
                las_dir: str = ".",
                las_base: str = "las_out",
                las_ext: str = "las") -> None:
    "Write scans from a pcap to las files (one per lidar scan)."

    if (metadata.format.udp_profile_lidar ==
            client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL):
        print("Note: You've selected to convert a dual returns pcap to LAS. "
              "Second returns are ignored in this conversion by this example "
              "for clarity reasons.  You can modify the code as needed by "
              "accessing it through Github or the SDK documentation.")

    from itertools import islice
    import laspy  # type: ignore

    # precompute xyzlut to save computation in a loop
    xyzlut = client.XYZLut(metadata)

    # create an iterator of LidarScans from pcap and bound it if num is specified
    scans = iter(client.Scans(source))
    if num:
        scans = islice(scans, num)

    for idx, scan in enumerate(scans):

        xyz = xyzlut(scan.field(client.ChanField.RANGE))

        las = laspy.create()
        las.x = xyz[:, :, 0].flatten()
        las.y = xyz[:, :, 1].flatten()
        las.z = xyz[:, :, 2].flatten()

        las_path = os.path.join(las_dir, f'{las_base}_{idx:06d}.{las_ext}')
        print(f'write frame #{idx} to file: {las_path}')

        las.write(las_path)


def pcap_to_pcd(source: client.PacketSource,
                metadata: client.SensorInfo,
                num: int = 0,
                pcd_dir: str = ".",
                pcd_base: str = "pcd_out",
                pcd_ext: str = "pcd") -> None:
    "Write scans from a pcap to pcd files (one per lidar scan)."

    if (metadata.format.udp_profile_lidar ==
            client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL):
        print("Note: You've selected to convert a dual returns pcap. Second "
              "returns are ignored in this conversion by this example "
              "for clarity reasons.  You can modify the code as needed by "
              "accessing it through github or the SDK documentation.")

    from itertools import islice
    try:
        import open3d as o3d  # type: ignore
    except ModuleNotFoundError:
        print(
            "This example requires open3d, which may not be available on all "
            "platforms. Try running `pip3 install open3d` first.")
        exit(1)

    if not os.path.exists(pcd_dir):
        os.makedirs(pcd_dir)

    # precompute xyzlut to save computation in a loop
    xyzlut = client.XYZLut(metadata)

    # create an iterator of LidarScans from pcap and bound it if num is specified
    scans = iter(client.Scans(source))
    if num:
        scans = islice(scans, num)

    for idx, scan in enumerate(scans):

        xyz = xyzlut(scan.field(client.ChanField.RANGE))

        pcd = o3d.geometry.PointCloud()  # type: ignore

        pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1,
                                                            3))  # type: ignore

        pcd_path = os.path.join(pcd_dir, f'{pcd_base}_{idx:06d}.{pcd_ext}')
        print(f'write frame #{idx} to file: {pcd_path}')

        o3d.io.write_point_cloud(pcd_path, pcd)  # type: ignore


def pcap_to_ply(source: client.PacketSource,
                metadata: client.SensorInfo,
                num: int = 0,
                ply_dir: str = ".",
                ply_base: str = "ply_out",
                ply_ext: str = "ply") -> None:
    "Write scans from a pcap to ply files (one per lidar scan)."

    # Don't need to print warning about dual returns since this leverages pcap_to_pcd

    # We are reusing the same Open3d File IO function to write the PLY file out
    pcap_to_pcd(source,
                metadata,
                num=num,
                pcd_dir=ply_dir,
                pcd_base=ply_base,
                pcd_ext=ply_ext)


def pcap_query_scan(source: client.PacketSource,
                    metadata: client.SensorInfo,
                    num: int = 0) -> None:
    """
    Example: Query available fields in LidarScan

    Args:
        source: PacketSource from pcap
        metadata: associated SensorInfo for PacketSource
        num: scan number in a given pcap file (satrs from *0*)
    """
    scans = iter(client.Scans(source))

    # [doc-stag-pcap-query-scan]
    scan = next(scans)
    print("Available fields and corresponding dtype in LidarScan")
    for field in scan.fields:
        print('{0:15} {1}'.format(str(field), scan.field(field).dtype))
    # [doc-etag-pcap-query-scan]


def pcap_read_packets(
        source: client.PacketSource,
        metadata: client.SensorInfo,
        num: int = 0  # not used in this example
) -> None:
    """Basic read packets example from pcap file. """
    # [doc-stag-pcap-read-packets]
    for packet in source:
        if isinstance(packet, client.LidarPacket):
            # Now we can process the LidarPacket. In this case, we access
            # the measurement ids, timestamps, and ranges
            measurement_ids = packet.measurement_id
            timestamps = packet.timestamp
            ranges = packet.field(client.ChanField.RANGE)
            print(f'  encoder counts = {measurement_ids.shape}')
            print(f'  timestamps = {timestamps.shape}')
            print(f'  ranges = {ranges.shape}')

        elif isinstance(packet, client.ImuPacket):
            # and access ImuPacket content
            print(f'  acceleration = {packet.accel}')
            print(f'  angular_velocity = {packet.angular_vel}')
    # [doc-etag-pcap-read-packets]


def pcap_to_csv(
        source: client.PacketSource,
        metadata: client.SensorInfo,
        num: int = 0) -> None:
    # leave comment directing users to ouster-cli
    print("NOTICE: The pcap-to-csv example has been retired in favor of "
          "the ouster-cli utility installed with the Python Ouster SDK.\n"
          "To try: ouster-cli source <PCAP> convert <OUT.CSV>")


def main():
    """Pcap examples runner."""
    examples = {
        "open3d-one-scan": pcap_3d_one_scan,
        "plot-xyz-points": pcap_display_xyz_points,
        "pcap-to-las": pcap_to_las,
        "pcap-to-pcd": pcap_to_pcd,
        "pcap-to-ply": pcap_to_ply,
        "pcap-to-csv": pcap_to_csv,
        "query-scan": pcap_query_scan,
        "read-packets": pcap_read_packets,
    }

    description = "Ouster Python SDK Pcap examples. The EXAMPLE must be one of:\n  " + str.join(
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

    with open(args.metadata_path, 'r') as f:
        metadata = client.SensorInfo(f.read())

    print(f'example: {args.example}')

    source = pcap.Pcap(args.pcap_path, metadata)

    with closing(source):
        example(source, metadata, args.scan_num)  # type: ignore


if __name__ == "__main__":
    main()

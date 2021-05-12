"""Ouster Open3D visualizations examples."""

import time
import threading
import numpy as np
from contextlib import closing
from more_itertools import nth

from ouster import client

from .pcap import read_metadata, ae, colorize

def sensor_viewer_3d(hostname: str,
                     lidar_port: int = 7502,
                     imu_port: int = 7503) -> None:
    """Open3D viewer of real-time data from sensor.

    Args:
        hostname: hostname of the sensor
        lidar_port: UDP port to listen on for lidar data
        imu_port: UDP port to listen on for imu data
    """
    import open3d as o3d
    from .open3d_utils import view_from

    with closing(client.Sensor(hostname, lidar_port, imu_port,
                               buf_size=640)) as source:

        xyzlut = client.XYZLut(source.metadata)
        # Shared data from sensor to animation loop
        data_ready = False
        data_xyz = np.array([])
        data_key_img_color = np.array([])

        data_lock = threading.Lock()

        def fetch_scans() -> None:
            nonlocal data_lock, data_ready, data_xyz, data_key_img_color
            for scan in client.Scans(source):
                xyz = xyzlut(scan)
                with data_lock:
                    data_xyz = xyz
                    key = scan.field(client.ChanField.SIGNAL)
                    key_img = ae(key)
                    data_key_img_color = colorize(key_img)
                    data_ready = True

        # Initialize point cloud with default data (needed for open3d to
        # render correctly and setup scaling parameter internally)
        pcd = o3d.geometry.PointCloud()

        w = source.metadata.format.columns_per_frame
        h = source.metadata.format.pixels_per_column

        pcd.points = o3d.utility.Vector3dVector(30.0 * np.random.rand(h * w, 3))
        pcd.colors = o3d.utility.Vector3dVector(np.zeros((h * w, 3)))

        initialized = False
        def animation_callback(vis):
            nonlocal initialized, pcd
            nonlocal data_lock, data_ready, data_xyz, data_key_img_color
            z_near = 1.0
            if not initialized:
                ropt = vis.get_render_option()
                ropt.point_size = 1.0
                ropt.background_color = np.asarray([0, 0, 0])
                ropt.light_on = False
                ctr = vis.get_view_control()
                ctr.set_zoom(0.2)
                ctr.set_constant_z_near(z_near)
                view_from(vis, np.array([2, 1, 2]), np.array([0, 0, 0]))
                initialized = True
                return True

            with data_lock:
                if data_ready:
                    pcd.points = o3d.utility.Vector3dVector(
                        data_xyz.reshape((-1, 3)))
                    pcd.colors = o3d.utility.Vector3dVector(
                        data_key_img_color.reshape((-1, 3)))
                    data_ready = False
                    return True

            # No need to update geometries if scan wasn't changed
            return False

        # Fetching sensor scans and preparing data in a separate thread
        t = threading.Thread(target=fetch_scans, name="Sensor", daemon=True)
        t.start()

        # helper coordinate axes
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(0.2)

        # use with animation callback to setup render/view control options
        o3d.visualization.draw_geometries_with_animation_callback(
            [pcd, axes], animation_callback)


def pcap_3d_one_scan(pcap_path: str, metadata_path: str, num: int = 0) -> None:
    """Render one scan in Open3D viewer from pcap file.

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        num: scan number in a given pcap file (satrs from *0*)
    """
    import ouster.pcap as pcap

    metadata = read_metadata(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

    # get single scan by num
    scans = client.Scans(source)
    scan = nth(scans, num)

    if not scan:
        print(f'ERROR: Scan # {num} in not present in pcap file: {pcap_path}')
        return    

    # [doc-stag-open3d-one-scan]
    import open3d as o3d
    from .open3d_utils import view_from

    # ... ``scan``` is a client.LidarScan object read previously from pcap file

    xyzlut = client.XYZLut(metadata)
    xyz = xyzlut(scan)
    key = scan.field(client.ChanField.SIGNAL)

    key_range = scan.field(client.ChanField.RANGE)
    key[key_range == 0] = 0

    # apply Ouster ``spezia`` colormap to field values
    key_img = ae(key)
    key_img_color = colorize(key_img)

    # prepare point cloud for Open3d Visualiser
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.reshape((-1, 3)))
    pcd.colors = o3d.utility.Vector3dVector(key_img_color.reshape((-1, 3)))

    # helper coordinate axes
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(1.0)

    scans_it = iter(scans)

    initialized = False

    def animation_callback(vis):
        nonlocal initialized, pcd, scans_it, scan
        if not initialized:
            ropt = vis.get_render_option()
            ropt.point_size = 1.0
            ropt.background_color = np.asarray([0, 0, 0])
            ropt.light_on = False
            # view point optimized for frame 84 of OS1_128.pcap sample data
            # change it for your data or use 'R' key to reset view if you see
            # nothing in a window
            ctr = vis.get_view_control()
            ctr.set_zoom(0.03)
            view_from(vis, np.array([2, 1, 2]), np.array([-3, 1.5, 0]))
            initialized = True
            return True

        # No need to update all geometries if we are not changing them
        return False

    # use with animation callback to setup render/view control options
    o3d.visualization.draw_geometries_with_animation_callback(
        [pcd, axes], animation_callback)
    # [doc-etag-open3d-one-scan]


def pcap_3d_one_scan_canvas(pcap_path: str,
                            metadata_path: str,
                            num: int = 0) -> None:
    """Render one scan in Open3D viewer from pcap file with 2d image.

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        num: scan number in a given pcap file (satrs from *0*)
    """
    import ouster.pcap as pcap

    metadata = read_metadata(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

    # get single scan by num
    scans = client.Scans(source)
    scan = nth(scans, num)

    if not scan:
        print(f'ERROR: Scan # {num} in not present in pcap file: {pcap_path}')
        return

    # [doc-stag-open3d-one-scan-canvas]
    import open3d as o3d
    from .open3d_utils import (view_from, create_canvas, canvas_set_image_data,
                               canvas_set_viewport)

    # ... ``scan``` is a client.LidarScan object read previously from pcap file

    xyzlut = client.XYZLut(metadata)
    xyz = xyzlut(scan)
    key = scan.field(client.ChanField.SIGNAL)

    key_range = scan.field(client.ChanField.RANGE)
    key[key_range == 0] = 0

    # apply Ouster ``spezia`` colormap to field values
    key_img = ae(key)
    key_img_color = colorize(key_img)
    key_img_gray = np.dstack([key_img, key_img, key_img])

    # key_img_rgb = np.dstack([ae(key), ae(key), ae(key)])

    # prepare point cloud for Open3d Visualiser
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.reshape((-1, 3)))
    pcd.colors = o3d.utility.Vector3dVector(key_img_color.reshape((-1, 3)))

    # helper coordinate axes
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(1.0)

    # prepare canvas for 2d image
    pict = create_canvas(scan.w, scan.h)
    canvas_set_image_data(pict, client.destagger(metadata, key_img_gray))

    scans_it = iter(scans)

    initialized = False

    def animation_callback(vis):
        nonlocal initialized, pcd, pict, scans_it, scan
        z_near = 1.0
        if not initialized:
            ropt = vis.get_render_option()
            ropt.point_size = 1.0
            ropt.background_color = np.asarray([0, 0, 0])
            ropt.light_on = False
            ctr = vis.get_view_control()
            ctr.set_zoom(0.03)
            ctr.set_constant_z_near(z_near)
            # view point optimized for frame 84 of OS1_128.pcap sample data
            view_from(vis, np.array([2, 1, 2]), np.array([-3, 1.5, 0]))
            initialized = True
            return True

        ctr = vis.get_view_control()

        # slowly rotate for better look-n-feel
        ctr.rotate(0.1, 0.0)

        # calculate and update canvas position using current camera params
        camera_params = ctr.convert_to_pinhole_camera_parameters()

        screen_width = camera_params.intrinsic.width

        canvas_height = scan.h
        canvas_width = scan.w * canvas_height / scan.h

        viewport = ((screen_width - canvas_width) / 2, 0, canvas_width,
                    canvas_height)

        # canvas position and size on a current screen
        canvas_set_viewport(pict, viewport,
                            camera_params.intrinsic.intrinsic_matrix,
                            camera_params.extrinsic)

        # Always re-render because we are moving canvas along with camera
        return True

    # use with animation callback to setup render/view control options
    o3d.visualization.draw_geometries_with_animation_callback(
        [pcd, axes, pict], animation_callback)
    # [doc-etag-open3d-one-scan-canvas]

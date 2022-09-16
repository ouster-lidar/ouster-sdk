"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Ouster Open3D visualizer
"""
import time

from more_itertools import consume, nth
import numpy as np

try:
    import open3d as o3d  # type: ignore
except ModuleNotFoundError:
    print("This example requires open3d, which may not be available on all "
          "platforms. Try running `pip3 install open3d` first.")
    exit(1)

from ouster import client
from ouster.client import _utils
from .colormaps import colorize

Z_NEAR = 1.0


def view_from(vis: o3d.visualization.Visualizer,
              from_point: np.ndarray,
              to_point: np.ndarray = np.array([0, 0, 0])):
    """Helper to setup view direction for Open3D Visualiser.

    Args:
        from_point: camera location in 3d space as ``[x,y,z]``
        to_point: camera view direction in 3d space as ``[x,y,z]``
    """
    ctr = vis.get_view_control()
    up_v = np.array([0, 0, 1])
    dir_v = to_point - from_point
    left_v = np.cross(up_v, dir_v)
    up = np.cross(dir_v, left_v)
    ctr.set_lookat(to_point)
    ctr.set_front(-dir_v)
    ctr.set_up(up)


def create_canvas(w: int, h: int) -> o3d.geometry.TriangleMesh:
    """Create canvas for 2D image.

    Args:
        w: width of the 2D image in screen coords (pixels)
        h: height of the 2D image in screen coords (pixels)
    """
    pic = o3d.geometry.TriangleMesh()
    pic.vertices = o3d.utility.Vector3dVector(
        np.array([[0, 1, 1], [0, -1, 1], [0, -1, -1], [0, 1, -1]]))
    pic.triangles = o3d.utility.Vector3iVector(
        np.array([
            [0, 3, 2],
            [2, 1, 0],
        ]))
    pic.triangle_uvs = o3d.utility.Vector2dVector(
        np.array([[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 1.0], [1.0, 0.0],
                  [0.0, 0.0]]))
    im = np.zeros((h, w, 3), dtype=np.float32)
    pic.textures = [o3d.geometry.Image(im)]
    pic.triangle_material_ids = o3d.utility.IntVector(
        np.array([0, 0], dtype=np.int32))
    return pic


def canvas_set_viewport(
        pic: o3d.geometry.TriangleMesh,
        camera_params: o3d.camera.PinholeCameraParameters) -> None:
    """Set the position of the 2D image in space so it seems as static.

    The method should be called on every animation update (animation callback)
    before rendering so the 2D mesh with texture always appear in the position
    that would seem "static" for the observer of the scene through the current
    camera parameters.

    Args:
        pic: canvas with 2D image, created with :func:`.create_canvas`
        camera_params: current open3d camera parameters
    """

    # calculate viewport
    screen_width = camera_params.intrinsic.width

    pic_img_data = np.asarray(pic.textures[0])
    canvas_height, canvas_width, _ = pic_img_data.shape

    viewport = ((screen_width - canvas_width) / 2, 0, canvas_width,
                canvas_height)

    # grab camera parameters
    intrinsics = camera_params.intrinsic.intrinsic_matrix
    extrinsics = camera_params.extrinsic

    x_pos, y_pos, width, height = viewport
    pict_pos = np.array([[x_pos, y_pos], [x_pos + width, y_pos],
                         [x_pos + width, y_pos + height],
                         [x_pos, y_pos + height]])

    # put canvas in correct camera view (img frame -> camera frame)
    assert intrinsics[0, 0] == intrinsics[1, 1]
    pict_pos = np.append((pict_pos - intrinsics[:2, 2]) / intrinsics[0, 0],
                         np.ones((pict_pos.shape[0], 1)),
                         axis=1)
    pict_pos = pict_pos * (Z_NEAR + 0.001)

    # move canvas to world coords (camera frame -> world frame)

    # invert camera extrinsics: inv([R|T]) = [R'|-R'T]
    tr = np.eye(extrinsics.shape[0])
    tr[:3, :3] = np.transpose(extrinsics[:3, :3])
    tr[:3, 3] = -np.transpose(extrinsics[:3, :3]) @ extrinsics[:3, 3]

    pict_pos = tr @ np.transpose(
        np.append(pict_pos, np.ones((pict_pos.shape[0], 1)), axis=1))
    pict_pos = np.transpose(pict_pos / pict_pos[-1])

    # set canvas position
    pict_vertices = np.asarray(pic.vertices)
    np.copyto(pict_vertices, pict_pos[:, :3])


def canvas_set_image_data(pic: o3d.geometry.TriangleMesh,
                          img_data: np.ndarray) -> None:
    """Set 2D image data to 2D canvas.

    Args:
        pic: 2D canvas creates with :func:`.create_canvas`
        img_data: image data RGB (i.e. shape ``[h, w, 3]``)
    """
    pic_img_data = np.asarray(pic.textures[0])
    assert pic_img_data.shape == img_data.shape
    np.copyto(pic_img_data, img_data)


def range_for_field(f: client.ChanField) -> client.ChanField:
    if f in (client.ChanField.RANGE2, client.ChanField.SIGNAL2,
             client.ChanField.REFLECTIVITY2):
        return client.ChanField.RANGE2
    else:
        return client.ChanField.RANGE


def viewer_3d(scans: client.Scans, paused: bool = False) -> None:
    """Render one scan in Open3D viewer from pcap file with 2d image.

    Args:
        pcap_path: path to the pcap file
        metadata_path: path to the .json with metadata (aka :class:`.SensorInfo`)
        num: scan number in a given pcap file (satrs from *0*)
    """

    # visualizer state
    scans_iter = iter(scans)
    scan = next(scans_iter)
    metadata = scans.metadata
    xyzlut = client.XYZLut(metadata)

    fields = list(scan.fields)
    aes = {}
    for field_ind, field in enumerate(fields):
        if field in (client.ChanField.SIGNAL, client.ChanField.SIGNAL2):
            aes[field_ind] = _utils.AutoExposure(0.02, 0.1, 3)
        else:
            aes[field_ind] = _utils.AutoExposure()

    field_ind = 2

    def next_field(vis):
        nonlocal field_ind
        field_ind = (field_ind + 1) % len(fields)
        update_data(vis)
        print(f"Visualizing: {fields[field_ind].name}")

    def toggle_pause(vis):
        nonlocal paused
        paused = not paused
        print(f"Paused: {paused}")

    def right_arrow(vis, action, mods):
        nonlocal scan
        if action == 1:
            print("Skipping forward 10 frames")
            scan = nth(scans_iter, 10)
            if scan is None:
                raise StopIteration
            update_data(vis)

    # create geometries
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(0.2)
    cloud = o3d.geometry.PointCloud()
    image = create_canvas(metadata.format.columns_per_frame,
                          metadata.format.pixels_per_column)

    def update_data(vis: o3d.visualization.Visualizer):
        xyz = xyzlut(scan.field(range_for_field(fields[field_ind])))
        key = scan.field(fields[field_ind]).astype(float)

        # apply colormap to field values
        aes[field_ind](key)
        color_img = colorize(key)

        # prepare point cloud for Open3d Visualiser
        cloud.points = o3d.utility.Vector3dVector(xyz.reshape((-1, 3)))
        cloud.colors = o3d.utility.Vector3dVector(color_img.reshape((-1, 3)))

        # prepare canvas for 2d image
        gray_img = np.dstack([key] * 3)
        canvas_set_image_data(image, client.destagger(metadata, gray_img))

        # signal that point cloud and needs to be re-rendered
        vis.update_geometry(cloud)

    # initialize vis
    vis = o3d.visualization.VisualizerWithKeyCallback()
    try:
        vis.create_window()

        ropt = vis.get_render_option()
        ropt.point_size = 1.0
        ropt.background_color = np.asarray([0, 0, 0])
        ropt.light_on = False

        # populate point cloud before adding geometry to avoid camera issues
        update_data(vis)

        vis.add_geometry(cloud)
        vis.add_geometry(image)
        vis.add_geometry(axes)

        # initialize camera settings
        ctr = vis.get_view_control()
        ctr.set_constant_z_near(Z_NEAR)
        ctr.set_zoom(0.2)
        view_from(vis, np.array([2, 1, 2]), np.array([0, 0, 0]))

        # register keys
        vis.register_key_callback(ord(" "), toggle_pause)
        vis.register_key_callback(ord("M"), next_field)
        vis.register_key_action_callback(262, right_arrow)

        # main loop
        last_ts = 0.0
        while vis.poll_events():
            ts = time.monotonic()

            # update data at scan frequency to avoid blocking the rendering thread
            if not paused and ts - last_ts >= 1 / metadata.mode.frequency:
                scan = next(scans_iter)
                update_data(vis)
                last_ts = ts

            # always update 2d image to follow camera
            canvas_set_viewport(image,
                                ctr.convert_to_pinhole_camera_parameters())
            vis.update_geometry(image)
            vis.update_renderer()

    finally:
        # open3d 0.13.0 segfaults on macos during teardown without this
        o3d.visualization.Visualizer.clear_geometries(vis)
        vis.destroy_window()


def main() -> None:
    import argparse
    import os
    import ouster.pcap as pcap

    descr = """Example visualizer using the open3d library.

    Visualize either pcap data (specified using --pcap) or a running sensor
    (specified using --sensor). If no metadata file is specified, this will look
    for a file with the same name as the pcap with the '.json' extension, or
    query it directly from the sensor.

    Visualizing a running sensor requires the sensor to be configured and
    sending lidar data to the default UDP port (7502) on the host machine.
    """

    parser = argparse.ArgumentParser(description=descr)
    parser.add_argument('--pause', action='store_true', help='start paused')
    parser.add_argument('--start', type=int, help='skip to frame number')
    parser.add_argument('--meta', metavar='PATH', help='path to metadata json')

    required = parser.add_argument_group('one of the following is required')
    group = required.add_mutually_exclusive_group(required=True)
    group.add_argument('--sensor', metavar='HOST', help='sensor hostname')
    group.add_argument('--pcap', metavar='PATH', help='path to pcap file')

    args = parser.parse_args()

    if args.sensor:
        scans = client.Scans.stream(args.sensor, metadata=args.meta)
    elif args.pcap:
        pcap_path = args.pcap
        metadata_path = args.meta or os.path.splitext(pcap_path)[0] + ".json"

        with open(metadata_path, 'r') as f:
            metadata = client.SensorInfo(f.read())

        source = pcap.Pcap(pcap_path, metadata)
        scans = client.Scans(source)
        consume(scans, args.start or 0)

    try:
        viewer_3d(scans, paused=args.pause)
    except (KeyboardInterrupt, StopIteration):
        pass
    finally:
        scans.close()


if __name__ == "__main__":
    main()

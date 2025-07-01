import argparse
import logging
import os

import numpy as np
from PIL import Image as PILImage
from plyfile import PlyData  # type: ignore  # noqa

import ouster.sdk.viz as viz

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('PNG_Maker')

# if beyond max points, random delete points to prevent memory overflow
# 50000000 points is around 30 mins recording with the default PLY maker voxel
# map size. You can increase the voxel map size in OSF save command for
# large maps
max_plot_points = 50000000


def get_doll_dist(points):
    # to linearize the plot camera zoon in distance
    lidar_default_range = 150
    min_camera_dist = 2
    # bigger number make camera more zoom in
    camera_magic_n = 20
    # because ouster.viz view_distance() uses std::exp(log_distance_ * 0.01)
    # to linearize the view_distance(). I time 100 for input
    linearize_n = 100

    max_values = np.max(points, axis=0)
    min_values = np.min(points, axis=0)

    max_dist = max(((max(abs(max_values - min_values)) -
                     lidar_default_range) / camera_magic_n), min_camera_dist)

    doll_dist = np.log(max_dist) * linearize_n

    return doll_dist


def peak_start_point(ply_files):
    if not ply_files:
        logger.error("PLY files list is empty\nExit!")
        return None

    # Read the first PLY file
    file = ply_files[0]
    try:
        with open(file, 'rb') as f:
            ply_data = PlyData.read(f)
        logger.info("PLY file successfully read.")
    # Now you can access the data in ply_data
    except IOError as e:
        logger.error("Error reading PLY file:", e)

    # Check if the PLY data is not empty
    if len(ply_data['vertex']) == 0:
        logger.error("PLY file is empty\nExit!")
        return None

    # Extract the first point as start point
    x = ply_data['vertex']['x'][0]
    y = ply_data['vertex']['y'][0]
    z = ply_data['vertex']['z'][0]

    logger.info(f"start origin point ({x}, {y}, {z})")
    return x, y, z


def output_ext_verify(filename):
    if not filename.endswith(".png"):
        raise argparse.ArgumentTypeError("Expect output file end with .png")
    return filename


def screenshot_and_save_png(viz, screenshot_path: str):
    pixels = viz.get_screenshot()
    PILImage.fromarray(pixels).convert(
            "RGB").save(screenshot_path)
    logger.info(f"Screenshot is saved at {screenshot_path}")
    return screenshot_path


def convert_ply_to_png(ply_files, screenshot_path, is_plot):
    start_point = peak_start_point(ply_files)

    if not start_point:
        return

    points = np.empty((0, 3))
    keys = np.empty(0)

    for file in ply_files:
        logger.info(f"processing file : {file}")
        # Read the PLY file
        ply_data = PlyData.read(file)
        x = ply_data['vertex']['x'] - start_point[0]
        y = ply_data['vertex']['y'] - start_point[1]
        z = ply_data['vertex']['z'] - start_point[2]

        possible_keys = ['REFLECTIVITY', 'SIGNAL', 'NEAR_IR']
        key_field = None

        for key in possible_keys:
            if key in ply_data['vertex'].data.dtype.names:
                key_field = key
                break

        if key_field is None:
            logger.error("No valid key field found in PLY file.")
            return

        k = ply_data['vertex'][key_field] * 2

        points = np.concatenate((points, np.stack((x, y, z), axis=-1)), axis=0)
        keys = np.concatenate((keys, k))

    current_size = points.shape[0]
    if current_size > max_plot_points:
        logger.info(
                f"current points {current_size} is greater than"
                f"maximum {max_plot_points}")
        points_to_delete = current_size - max_plot_points
        indices_to_delete = np.random.choice(
            current_size, points_to_delete, replace=False)
        points = np.delete(points, indices_to_delete, axis=0)
        keys = np.delete(keys, indices_to_delete)
        logger.info(f"random delete {points_to_delete} points")

    doll_dist = get_doll_dist(points)

    point_viz = viz.PointViz("Viz", True, 3840, 2160)
    viz.add_default_controls(point_viz)
    cloud = viz.Cloud(points.shape[0])

    cloud.set_xyz(np.asfortranarray(points))
    cloud.set_key(np.asfortranarray(keys))
    cloud.set_palette(viz.magma_palette)
    point_viz.add(cloud)
    point_viz.camera.set_dolly(doll_dist)
    point_viz.update()
    screenshot_and_save_png(point_viz, screenshot_path)
    if is_plot:
        point_viz.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot Point Cloud")
    parser.add_argument(
        "PLY_files",
        metavar='file',
        nargs='+',
        help="PLY file as input")
    parser.add_argument(
        "--dir",
        help="PNG output dir")
    parser.add_argument(
        "--out",
        type=output_ext_verify,
        help="PNG output name")
    parser.add_argument(
        '--plot',
        action='store_true',
        default=False,
        help='Enable plotting')

    args = parser.parse_args()
    file_path = ''

    if args.dir:
        if not os.path.exists(args.dir):
            logger.error(f"The given output dir {args.dir} not exist")
            exit(1)

        file_path = args.dir
        if not file_path.endswith(os.path.sep):
            file_path += os.path.sep
    if file_path and not args.out:
        logger.error(f"Given the output dir {file_path} without the output "
                     f"filename {args.out}. Use --out to specify the out filename")
        exit(1)
    if args.out:
        file_path += args.out

    convert_ply_to_png(args.PLY_files, file_path, args.plot)

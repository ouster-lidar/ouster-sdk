"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Example of custom visualizer use.

Intended to run with `python -m ouster.sdk.examples.viz`

"""

import argparse
from ouster import client, pcap, viz
from ouster.sdk.util import resolve_metadata
import os
import sys
import numpy as np
import random


def make_checker_board(square_size, reps):
    """Makes a test checker board image."""
    img_data = np.full((square_size, square_size), 0)
    img_data = np.hstack([img_data, np.logical_xor(img_data, 1)])
    img_data = np.vstack([img_data, np.logical_xor(img_data, 1)])
    img_data = np.tile(img_data, reps)
    return img_data


# Helper method to remove list of objects from point_viz
def remove_objs(point_viz, objs):
    for obj in objs:
        point_viz.remove(obj)
    # currently we need to always call update()/run_once() after some object removes
    point_viz.update()
    point_viz.run_once()


def main():
    """PointViz visualizer examples."""

    parser = argparse.ArgumentParser(
        description=main.__doc__,
        formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('pcap_path',
                        nargs='?',
                        metavar='PCAP',
                        help='path to pcap file')
    parser.add_argument('meta_path',
                        nargs='?',
                        metavar='METADATA',
                        help='path to metadata json')

    args = parser.parse_args()

    pcap_path = os.getenv("SAMPLE_DATA_PCAP_PATH", args.pcap_path)
    meta_path = os.getenv("SAMPLE_DATA_JSON_PATH", args.meta_path)

    # try to find the metadata json file near the pcap file by common prefix
    meta_path = resolve_metadata(pcap_path, meta_path)

    if not pcap_path or not meta_path:
        print(
            "ERROR: Please add SAMPLE_DATA_PCAP_PATH and SAMPLE_DATA_JSON_PATH to"
            + " environment variables or pass <pcap_path> and <meta_path>")
        sys.exit()

    print(f"Using:\n\tjson: {meta_path}\n\tpcap: {pcap_path}")

    # Getting data sources
    meta = client.SensorInfo(open(meta_path).read())
    packets = pcap.Pcap(pcap_path, meta)
    scans = iter(client.Scans(packets))

    # ==============================
    print("Ex 0: Empty Point Viz")

    # [doc-stag-empty-pointviz]
    # Creating a point viz instance
    point_viz = viz.PointViz("Example Viz")
    viz.add_default_controls(point_viz)

    # ... add objects here

    # update internal objects buffers and run visualizer
    point_viz.update()
    point_viz.run()
    # [doc-etag-empty-pointviz]

    # =========================================================================
    print("Ex 1.0:\tImages and Labels: the Image object and 2D Image "
          "set_position() - height-normalized screen coordinates")

    label_top = viz.Label("[0, 1]", 0.5, 0.0, align_top=True)
    label_top.set_scale(2)
    point_viz.add(label_top)

    label_bot = viz.Label("[0, -1]", 0.5, 1, align_top=False)
    label_bot.set_scale(2)
    point_viz.add(label_bot)

    # [doc-stag-image-pos-center]
    img = viz.Image()
    img.set_image(np.full((10, 10), 0.5))
    img.set_position(-0.5, 0.5, -0.5, 0.5)
    point_viz.add(img)
    # [doc-etag-image-pos-center]

    # visualize
    point_viz.update()
    point_viz.run()

    # =========================================================================
    print("Ex 1.1:\tImages and Labels: Window-aligned images with 2D Image "
          "set_hshift() - width-normalized [-1, 1] horizontal shift")

    # [doc-stag-image-pos-left]
    # move img to the left
    img.set_position(0, 1, -0.5, 0.5)
    img.set_hshift(-1)
    # [doc-etag-image-pos-left]

    # visualize
    point_viz.update()
    point_viz.run()

    # [doc-stag-image-pos-right]
    # move img to the right
    img.set_position(-1, 0, -0.5, 0.5)
    img.set_hshift(1)
    # [doc-etag-image-pos-right]

    # visualize
    point_viz.update()
    point_viz.run()

    # [doc-stag-image-pos-right-bottom]
    # move img to the right bottom
    img.set_position(-1, 0, -1, 0)
    img.set_hshift(1)
    # [doc-etag-image-pos-right-bottom]

    # visualize
    point_viz.update()
    point_viz.run()

    # remove_objs(point_viz, [label_top, label_mid, label_bot, img])
    remove_objs(point_viz, [label_top, label_bot, img])

    # =======================================
    print("Ex 1.2:\tImages and Labels: Lidar Scan Fields as Images")

    # [doc-stag-scan-fields-images]
    scan = next(scans)

    img_aspect = (meta.beam_altitude_angles[0] -
                  meta.beam_altitude_angles[-1]) / 360.0
    img_screen_height = 0.4  # [0..2]
    img_screen_len = img_screen_height / img_aspect

    # prepare field data
    ranges = scan.field(client.ChanField.RANGE)
    ranges = client.destagger(meta, ranges)
    ranges = np.divide(ranges, np.amax(ranges), dtype=np.float32)

    signal = scan.field(client.ChanField.REFLECTIVITY)
    signal = client.destagger(meta, signal)
    signal = np.divide(signal, np.amax(signal), dtype=np.float32)

    # creating Image viz elements
    range_img = viz.Image()
    range_img.set_image(ranges)
    # top center position
    range_img.set_position(-img_screen_len / 2, img_screen_len / 2,
                           1 - img_screen_height, 1)
    point_viz.add(range_img)

    signal_img = viz.Image()
    signal_img.set_image(signal)
    img_aspect = (meta.beam_altitude_angles[0] -
                  meta.beam_altitude_angles[-1]) / 360.0
    img_screen_height = 0.4  # [0..2]
    img_screen_len = img_screen_height / img_aspect
    # bottom center position
    signal_img.set_position(-img_screen_len / 2, img_screen_len / 2, -1,
                            -1 + img_screen_height)
    point_viz.add(signal_img)
    # [doc-etag-scan-fields-images]

    # visualize
    point_viz.update()
    point_viz.run()

    print("Ex 1.3:\tImages and Labels: Adding labels")

    # [doc-stag-scan-fields-images-labels]
    range_label = viz.Label(str(client.ChanField.RANGE),
                            0.5,
                            0,
                            align_top=True)
    range_label.set_scale(1)
    point_viz.add(range_label)

    signal_label = viz.Label(str(client.ChanField.REFLECTIVITY),
                             0.5,
                             1 - img_screen_height / 2,
                             align_top=True)
    signal_label.set_scale(1)
    point_viz.add(signal_label)
    # [doc-etag-scan-fields-images-labels]

    # visualize
    point_viz.update()
    point_viz.run()

    # ===============================================================
    print("Ex 2.0:\tPoint Clouds: As Structured Points")

    # [doc-stag-scan-structured]
    cloud_scan = viz.Cloud(meta)
    cloud_scan.set_range(scan.field(client.ChanField.RANGE))
    cloud_scan.set_key(signal)
    point_viz.add(cloud_scan)
    # [doc-etag-scan-structured]

    # visualize
    point_viz.update()
    point_viz.run()

    remove_objs(point_viz, [cloud_scan])

    # ===============================================================
    print("Ex 2.1:\tPoint Clouds: As Unstructured Points")

    # [doc-stag-scan-unstructured]
    # transform scan data to 3d points
    xyzlut = client.XYZLut(meta)
    xyz = xyzlut(scan.field(client.ChanField.RANGE))

    cloud_xyz = viz.Cloud(xyz.shape[0] * xyz.shape[1])
    cloud_xyz.set_xyz(np.reshape(xyz, (-1, 3)))
    cloud_xyz.set_key(signal.ravel())
    point_viz.add(cloud_xyz)
    # [doc-etag-scan-unstructured]

    point_viz.camera.dolly(150)

    # visualize
    point_viz.update()
    point_viz.run()

    # =======================================================
    print("Ex 2.2:\tPoint Clouds: Custom Axes Helper as Unstructured Points")

    # [doc-stag-axes-helper]
    # basis vectors
    x_ = np.array([1, 0, 0]).reshape((-1, 1))
    y_ = np.array([0, 1, 0]).reshape((-1, 1))
    z_ = np.array([0, 0, 1]).reshape((-1, 1))

    axis_n = 100
    line = np.linspace(0, 1, axis_n).reshape((1, -1))

    # basis vector to point cloud
    axis_points = np.hstack((x_ @ line, y_ @ line, z_ @ line)).transpose()

    # colors for basis vectors
    axis_color_mask = np.vstack((np.full(
        (axis_n, 4), [1, 0.1, 0.1, 1]), np.full((axis_n, 4), [0.1, 1, 0.1, 1]),
                                 np.full((axis_n, 4), [0.1, 0.1, 1, 1])))

    cloud_axis = viz.Cloud(axis_points.shape[0])
    cloud_axis.set_xyz(axis_points)
    cloud_axis.set_key(np.full(axis_points.shape[0], 0.5))
    cloud_axis.set_mask(axis_color_mask)
    cloud_axis.set_point_size(3)
    point_viz.add(cloud_axis)
    # [doc-etag-axes-helper]

    point_viz.camera.dolly(50)

    # visualize
    point_viz.update()
    point_viz.run()

    remove_objs(point_viz, [
        range_img, range_label, signal_img, signal_label, cloud_axis, cloud_xyz
    ])

    # ===============================================================
    print("Ex 2.3:\tPoint Clouds: the LidarScanViz class")

    # [doc-stag-lidar-scan-viz]
    # Creating LidarScan visualizer (3D point cloud + field images on top)
    ls_viz = viz.LidarScanViz(meta, point_viz)

    # adding scan to the lidar scan viz
    ls_viz.update(scan)

    # refresh viz data
    ls_viz.draw()

    # visualize
    # update() is not needed for LidatScanViz because it's doing it internally
    point_viz.run()
    # [doc-etag-lidar-scan-viz]

    # ===================================================
    print("Ex 3.0:\tAugmenting point clouds with 3D Labels")

    # [doc-stag-lidar-scan-viz-labels]
    # Adding 3D Labels
    label1 = viz.Label("Label1: [1, 2, 4]", 1, 2, 4)
    point_viz.add(label1)

    label2 = viz.Label("Label2: [2, 1, 4]", 2, 1, 4)
    label2.set_scale(2)
    point_viz.add(label2)

    label3 = viz.Label("Label3: [4, 2, 1]", 4, 2, 1)
    label3.set_scale(3)
    point_viz.add(label3)
    # [doc-etag-lidar-scan-viz-labels]

    point_viz.camera.dolly(-100)

    # visualize
    point_viz.update()
    point_viz.run()

    # ===============================================
    print("Ex 4.0:\tOverlay 2D Images and 2D Labels")

    # [doc-stag-overlay-images-labels]
    # Adding image 1 with aspect ratio preserved
    img = viz.Image()
    img_data = make_checker_board(10, (2, 4))
    mask_data = np.zeros((30, 30, 4))
    mask_data[:15, :15] = np.array([1, 0, 0, 1])
    img.set_mask(mask_data)
    img.set_image(img_data)
    ypos = (0, 0.5)
    xlen = (ypos[1] - ypos[0]) * img_data.shape[1] / img_data.shape[0]
    xpos = (0, xlen)
    img.set_position(*xpos, *ypos)
    img.set_hshift(-0.5)
    point_viz.add(img)

    # Adding Label for image 1: positioned at bottom left corner
    img_label = viz.Label("ARRrrr!", 0.25, 0.5)
    img_label.set_rgba((1.0, 1.0, 0.0, 1))
    img_label.set_scale(2)
    point_viz.add(img_label)

    # Adding image 2: positioned to the right of the window
    img2 = viz.Image()
    img_data2 = make_checker_board(10, (4, 2))
    mask_data2 = np.zeros((30, 30, 4))
    mask_data2[15:25, 15:25] = np.array([0, 1, 0, 0.5])
    img2.set_mask(mask_data2)
    img2.set_image(img_data2)
    ypos2 = (0, 0.5)
    xlen2 = (ypos2[1] - ypos2[0]) * img_data2.shape[1] / img_data2.shape[0]
    xpos2 = (-xlen2, 0)
    img2.set_position(*xpos2, *ypos2)
    img2.set_hshift(1.0)
    point_viz.add(img2)

    # Adding Label for image 2: positioned at top left corner
    img_label2 = viz.Label("Second",
                           1.0,
                           0.25,
                           align_top=True,
                           align_right=True)
    img_label2.set_rgba((0.0, 1.0, 1.0, 1))
    img_label2.set_scale(1)
    point_viz.add(img_label2)
    # [doc-etag-overlay-images-labels]

    # visualize
    point_viz.update()
    point_viz.run()

    # ===============================================================
    print("Ex 5.0:\tAdding key handlers: 'R' for random camera dolly")

    # [doc-stag-key-handlers]
    def handle_dolly_random(ctx, key, mods) -> bool:
        if key == 82:  # key R
            dolly_num = random.randrange(-15, 15)
            print(f"Random Dolly: {dolly_num}")
            point_viz.camera.dolly(dolly_num)
            point_viz.update()
        return True

    point_viz.push_key_handler(handle_dolly_random)
    # [doc-etag-key-handlers]

    # visualize
    point_viz.update()
    point_viz.run()

    # That's all folks! Happy hacking!


if __name__ == "__main__":
    main()

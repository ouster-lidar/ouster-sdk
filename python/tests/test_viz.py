"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

import weakref
from typing import TYPE_CHECKING, Tuple

import pytest
import numpy as np

import random

from ouster import client

# test env may not have opengl, but all test modules are imported during
# collection. Import is still needed to typecheck
if TYPE_CHECKING:
    from ouster.sdk import viz
else:
    viz = pytest.importorskip('ouster.sdk.viz')

# mark all tests in this module so they only run with the --interactive flag
pytestmark = pytest.mark.interactive


def make_checker_board(square_size: int, reps:Tuple[int, int]) -> np.ndarray:
    img_data = np.full((square_size, square_size), 0)
    img_data = np.hstack([img_data, np.logical_xor(img_data, 1)])
    img_data = np.vstack([img_data, np.logical_xor(img_data, 1)])
    img_data = np.tile(img_data, reps)
    return img_data


@pytest.fixture
def point_viz() -> viz.PointViz:
    point_viz = viz.PointViz("Test Viz",
                             fix_aspect=True,
                             window_width=640,
                             window_height=480)

    weakself = weakref.ref(point_viz)

    def handle_keys(ctx, key, mods) -> bool:
        self = weakself()
        if self is not None and key == 256:  # ESC
            self.running(False)
        return True

    point_viz.push_key_handler(handle_keys)
    return point_viz


def test_point_viz_image(point_viz: viz.PointViz) -> None:
    """Test displaying a full-window image."""
    img = viz.Image()
    img.set_position(-4 / 3, 4 / 3, -1, 1)
    img.set_image(
        np.tile(np.linspace(0.0, 1.0, 640, dtype=np.float32), (480, 1)))
    point_viz.add(img)

    point_viz.update()
    point_viz.run()


def test_point_viz_image_with_labels_aligned(point_viz: viz.PointViz) -> None:
    """Test displaying a set of images aligned to the corners."""

    red_rgba = (1.0, 0.3, 0.3, 1)
    blue_rgba = (0.3, 0.3, 1.0, 1)
    green_rgba = (0.3, 1.0, 0.3, 1)
    yellow_rgba = (1.0, 1.0, 0.3, 1)
    cyan_rgba = (1.0, 0.3, 1.0, 1)
    magenta_rgba = (0.3, 1.0, 1.0, 1)
    white_rgba = (1.0, 1.0, 1.0, 1)
    gray_rgba = (0.5, 0.5, 0.5, 1)

    colors = [red_rgba, blue_rgba, green_rgba, yellow_rgba, cyan_rgba, magenta_rgba, white_rgba,
              gray_rgba]

    ylen = 0.15
    xlen_calc = lambda vlen, shape: shape[1] * vlen / shape[0]
    label_posx = lambda posx: (posx + 1) / 2
    label_posy = lambda posy: 1 - (posy + 1) / 2

    def gen_rand_img():
        return make_checker_board(10, (random.randrange(2, 5), random.randrange(2, 5)))

    def gen_rand_color():
        return random.choice(colors)

    def add_image(im_data, xpos, ypos, hshift = 0):
        img = viz.Image()
        img.set_position(*xpos, *ypos)
        img.set_image(im_data)
        img.set_hshift(hshift)
        point_viz.add(img)

    def add_label(text, xpos, ypos, align_right=False, align_top=False, rgba=None, scale=2):
        xpos = label_posx(xpos)
        ypos = label_posy(ypos)
        label = viz.Label(text, xpos, ypos, align_right=align_right, align_top=align_top)
        label.set_rgba(rgba if rgba else gen_rand_color())
        label.set_scale(scale)
        point_viz.add(label)

    # center
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [- ylen / 2, ylen / 2] # center
    xpos = [- xlen / 2, xlen / 2]
    add_image(img_data, xpos, ypos, 0)
    add_label("Center", 0, ypos[1])

    # top left
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [1 - ylen, 1] # top
    xpos = [0, xlen]
    add_image(img_data, xpos, ypos, -1.0)
    add_label("Top Left - top", -1.0, ypos[1], align_top=True)
    add_label("Top Left - bottom", -1.0, ypos[0], align_top=False)

    # top right
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [1 - ylen, 1] # top
    xpos = [- xlen, 0]
    add_image(img_data, xpos, ypos, 1.0)
    add_label("Top Right - top", 1.0, ypos[1], align_right=True, align_top=True)
    add_label("Top Right - bottom", 1.0, ypos[0], align_right=True, align_top=False)

    # bottom left
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [-1, -1 + ylen] # bottom
    xpos = [0, xlen]
    add_image(img_data, xpos, ypos, -1.0)
    add_label("Bottom Left - top", -1.0, ypos[1], align_right=False, align_top=True)
    add_label("Bottom Left - bottom", -1.0, ypos[0], align_right=False, align_top=False)

    # bottom right
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [-1, -1 + ylen] # bottom
    xpos = [-xlen, 0]
    add_image(img_data, xpos, ypos, 1.0)
    add_label("Bottom Right - top", 1.0, ypos[1], align_right=True, align_top=True)
    add_label("Bottom Right - bottom", 1.0, ypos[0], align_right=True, align_top=False)

    point_viz.update()
    point_viz.run()


def test_point_viz_labels(point_viz: viz.PointViz) -> None:
    """Smoke test rendering labels."""

    label1 = viz.Label("Foobar", 0, 0, 1)
    label1.set_scale(0.75)
    label2 = viz.Label("Baz\nQux", 0, 0, 0)
    label3 = viz.Label("Quux", 1, 1, align_right=True)
    label3.set_scale(2.5)
    point_viz.add(label1)
    point_viz.add(label2)
    point_viz.add(label3)
    point_viz.camera.dolly(250)
    point_viz.update()
    point_viz.run()


def test_point_viz_cloud_unstructured(point_viz: viz.PointViz) -> None:
    """Smoke test rendering unstructured clouds."""
    import math
    import threading
    import time

    cloud = viz.Cloud(1024)
    point_viz.add(cloud)
    cloud.set_xyz(np.random.rand(1024, 3).astype(np.float32) * 30 - 15)
    cloud.set_key(np.random.rand(1024).astype(np.float32))

    quit = threading.Event()

    def animate() -> None:
        ticks = 0
        while not quit.is_set():
            t = (ticks % 300) / 150.0 * math.pi
            pose = np.array(
                [
                    [math.cos(t), math.sin(-t), 0, 0],  # rotate about z
                    [math.sin(t), math.cos(t), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ],
                order='F')

            cloud.set_pose(pose)
            point_viz.update()
            ticks += 1
            time.sleep(0.0333)

    thread = threading.Thread(target=animate)
    thread.start()

    point_viz.run()
    quit.set()
    thread.join()


def test_point_viz_destruction() -> None:
    """Check that PointViz is destroyed deterministically."""
    point_viz = viz.PointViz("Test Viz")
    ref = weakref.ref(point_viz)

    del point_viz
    assert ref() is None


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_scan_viz_destruction(meta: client.SensorInfo,
                              point_viz: viz.PointViz) -> None:
    """Check that LidarScan is destroyed deterministically."""
    ls_viz = viz.LidarScanViz(meta, point_viz)
    ref = weakref.ref(ls_viz)

    del ls_viz
    assert ref() is None


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_viz_multiple_instances(meta: client.SensorInfo,
                                scan: client.LidarScan) -> None:
    """Check that destructing a visualizer doesn't break other instances."""
    point_viz = viz.PointViz("Test Viz")

    # will call destructor, make sure it doesn't do anything silly like terminate glfw
    point_viz2 = viz.PointViz("Test Viz2")
    del point_viz2

    ls_viz = viz.LidarScanViz(meta, point_viz)

    ls_viz.scan = scan
    ls_viz.draw()
    point_viz.run()


def test_scan_viz_smoke(meta: client.SensorInfo,
                        scan: client.LidarScan) -> None:
    """Smoke test LidarScan visualization."""
    ls_viz = viz.LidarScanViz(meta)

    ls_viz.scan = scan
    ls_viz.draw()
    ls_viz.run()


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_scan_viz_extras(meta: client.SensorInfo,
                         scan: client.LidarScan) -> None:
    """Check rendering of labels, cuboids, clouds and images together."""
    point_viz = viz.PointViz("Test Viz")
    ls_viz = viz.LidarScanViz(meta, point_viz)

    cube1 = viz.Cuboid(np.identity(4), (1.0, 0, 0))

    # scaled in y and translated in x
    pose2 = np.array([
        [1, 0, 0, 5],
        [0, 2, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    cube2 = viz.Cuboid(pose2, (0, 1, 0, 0.5))

    label1 = viz.Label("Baz\nQux", 0.0, 0.0, 2.0)
    point_viz.add(label1)
    point_viz.add(cube1)
    point_viz.add(cube2)

    ls_viz.scan = scan

    point_viz.camera.dolly(150)
    ls_viz.draw()
    point_viz.run()

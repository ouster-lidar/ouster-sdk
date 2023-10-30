"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

import weakref
from typing import TYPE_CHECKING, Tuple, Optional, Callable

import pytest
import numpy as np

import random

from ouster import client

# test env may not have opengl, but all test modules are imported during
# collection. Import is still needed to typecheck
if TYPE_CHECKING:
    import ouster.viz as viz
else:
    viz = pytest.importorskip('ouster.viz')

# mark all tests in this module so they only run with the --interactive flag
pytestmark = pytest.mark.interactive


def make_checker_board(square_size: int, reps: Tuple[int, int]) -> np.ndarray:
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
    viz.add_default_controls(point_viz)
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


def test_point_viz_rgb_image(point_viz: viz.PointViz) -> None:
    """Test displaying a rgb image."""
    img = viz.Image()
    img.set_position(-4 / 3, 4 / 3, -1, 1)
    point_viz.add(img)

    label = viz.Label("", 0, 0, align_top=True)
    point_viz.add(label)

    def show_viz():
        point_viz.update()
        point_viz.run()

    label.set_text("Image RGB: set..image(mono), 2dim")
    image_data_mono = np.array(
        [[0.1, 0.3, 0.7], [0.2, 0.4, 0.8], [0.3, 0.5, 0.9]], dtype=float)
    img.set_image(image_data_mono)
    show_viz()

    label.set_text("Image RGB: set..image(mono), 3dim")
    img.set_image(image_data_mono[:, :, np.newaxis])
    show_viz()

    # 2 elements on 3rd dimension is an error
    with pytest.raises(ValueError):
        img.set_image(np.dstack((image_data_mono, image_data_mono)))

    label.set_text("Image RGB: set..image(rgb), 3dim")
    image_data_rgb = np.array(
        [[[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[1, 1, 0], [0, 0, 1], [1, 0, 1]],
         [[0, 1, 1], [1, 1, 1], [0, 0, 0]]],
        dtype=float)
    img.set_image(image_data_rgb)
    show_viz()

    label.set_text(
        "Image RGB: set..image(rgba), 3dim, two right columns has 0.5 alpha")
    image_data_rgba = np.dstack(
        (image_data_rgb, np.full((*image_data_rgb.shape[:2], 1), 0.5)))
    image_data_rgba[:, 0, 3] = 1.0
    img.set_image(image_data_rgba)
    show_viz()

    label.set_text(
        "Image RGB: set..mask(rgba), 4 pixels with different alphas, image the same"
    )
    mask_data_rgba = np.array([[[0, 0, 1.0, 0.2], [0, 0, 1.0, 0.4]],
                               [[0, 0, 1.0, 0.6], [0, 0, 1.0, 0.8]]],
                              dtype=float)
    img.set_mask(mask_data_rgba)
    show_viz()

    label.set_text(
        "Image RGB: set..mask(rgba), +3 columns added to mask, image the same")
    mask_data_rgba = np.hstack(
        (mask_data_rgba,
         np.array([[[1.0, 0, 0, 0.3], [1.0, 0, 0, 0.5], [1.0, 0, 0, 0.7]],
                   [[1.0, 0, 0, 0.3], [1.0, 0, 0, 0.5], [1.0, 0, 0, 0.7]]],
                  dtype=float)))
    img.set_mask(mask_data_rgba)
    show_viz()

    label.set_text(
        "Image RGB: set..mask(rgba), last row alpha 0.8, image the same")
    mask_data_rgba[-1, :, 3] = 0.8
    img.set_mask(mask_data_rgba)
    show_viz()

    label.set_text(
        "Image RGB: set..mask(rgba), last row alpha 1.0, image the same")
    mask_data_rgba[-1, :, 3] = 1.0
    img.set_mask(mask_data_rgba)
    show_viz()

    label.set_text("Image RGB: set..image(mono), mask the same")
    img.set_image(image_data_mono)
    show_viz()


def test_point_viz_image_palette(point_viz: viz.PointViz) -> None:
    """Test displaying a full-window image."""

    palettes = [
        ('no..palette', lambda i: i.clear_palette()),
        ('spezia..palette', lambda i: i.set_palette(viz.spezia_palette)),
        ('calref..palette', lambda i: i.set_palette(viz.calref_palette)),
        ('viridis..palette', lambda i: i.set_palette(viz.viridis_palette)),
        ('magma..palette', lambda i: i.set_palette(viz.magma_palette)),
        ('grey..palette', lambda i: i.set_palette(viz.grey_palette)),
    ]

    num_palettes = len(palettes)

    images = []
    labels = []

    key = np.tile(np.linspace(0.0, 1.0, 1024, dtype=np.float32), (128, 1))

    for idx, (palette_name, palette_enable) in enumerate(palettes):
        img = viz.Image()
        img_y0 = -1 + idx * 2 / num_palettes
        img_y1 = -1 + (idx + 1) * 2 / num_palettes
        img.set_position(-4 / 3, 4 / 3, img_y0, img_y1)
        palette_enable(img)
        img.set_image(key)
        point_viz.add(img)
        images.append(img)

        label = viz.Label(palette_name, 0.5, (1 - img_y1) / 2, align_top=True)
        point_viz.add(label)
        labels.append(label)

    def show_viz():
        point_viz.update()
        point_viz.run()

    show_viz()


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

    colors = [
        red_rgba, blue_rgba, green_rgba, yellow_rgba, cyan_rgba, magenta_rgba,
        white_rgba, gray_rgba
    ]

    ylen = 0.15
    xlen_calc = lambda vlen, shape: shape[1] * vlen / shape[0]
    label_posx = lambda posx: (posx + 1) / 2
    label_posy = lambda posy: 1 - (posy + 1) / 2

    def gen_rand_img():
        return make_checker_board(
            10, (random.randrange(2, 5), random.randrange(2, 5)))

    def gen_rand_color():
        return random.choice(colors)

    def add_image(im_data, xpos, ypos, hshift=0):
        img = viz.Image()
        img.set_position(*xpos, *ypos)
        img.set_image(im_data)
        img.set_hshift(hshift)
        point_viz.add(img)

    def add_label(text,
                  xpos,
                  ypos,
                  align_right=False,
                  align_top=False,
                  rgba=None,
                  scale=2):
        xpos = label_posx(xpos)
        ypos = label_posy(ypos)
        label = viz.Label(text,
                          xpos,
                          ypos,
                          align_right=align_right,
                          align_top=align_top)
        label.set_rgba(rgba if rgba else gen_rand_color())
        label.set_scale(scale)
        point_viz.add(label)

    # center
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [-ylen / 2, ylen / 2]  # center
    xpos = [-xlen / 2, xlen / 2]
    add_image(img_data, xpos, ypos, 0)
    add_label("Center", 0, ypos[1])

    # top left
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [1 - ylen, 1]  # top
    xpos = [0, xlen]
    add_image(img_data, xpos, ypos, -1.0)
    add_label("Top Left - top", -1.0, ypos[1], align_top=True)
    add_label("Top Left - bottom", -1.0, ypos[0], align_top=False)

    # top right
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [1 - ylen, 1]  # top
    xpos = [-xlen, 0]
    add_image(img_data, xpos, ypos, 1.0)
    add_label("Top Right - top",
              1.0,
              ypos[1],
              align_right=True,
              align_top=True)
    add_label("Top Right - bottom",
              1.0,
              ypos[0],
              align_right=True,
              align_top=False)

    # bottom left
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [-1, -1 + ylen]  # bottom
    xpos = [0, xlen]
    add_image(img_data, xpos, ypos, -1.0)
    add_label("Bottom Left - top",
              -1.0,
              ypos[1],
              align_right=False,
              align_top=True)
    add_label("Bottom Left - bottom",
              -1.0,
              ypos[0],
              align_right=False,
              align_top=False)

    # bottom right
    img_data = gen_rand_img()
    xlen = xlen_calc(ylen, img_data.shape)
    ypos = [-1, -1 + ylen]  # bottom
    xpos = [-xlen, 0]
    add_image(img_data, xpos, ypos, 1.0)
    add_label("Bottom Right - top",
              1.0,
              ypos[1],
              align_right=True,
              align_top=True)
    add_label("Bottom Right - bottom",
              1.0,
              ypos[0],
              align_right=True,
              align_top=False)

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


def test_point_viz_rgb_cloud(point_viz: viz.PointViz) -> None:
    """Test rgb coloring of clouds."""

    cloud = viz.Cloud(1024)
    point_viz.add(cloud)

    points = np.random.rand(3, 1024).astype(np.float32) * 30 - 15

    # should be r, g, b from top to bottom
    key = np.zeros(1024, dtype=float)

    key[5.0 < points[2, :]] = 0.2
    key[(-5.0 < points[2, :]) & (points[2, :] <= 5.0)] = 0.5
    key[points[2, :] < -5.0] = 0.8

    cloud.set_xyz(points)

    cloud.set_point_size(10)
    point_viz.camera.pitch(-45.0)

    label = viz.Label("", 0, 0, align_top=True)
    point_viz.add(label)

    def show_viz():
        point_viz.update()
        point_viz.run()

    with pytest.raises(ValueError):
        cloud.set_key(key.reshape((1, 1024, 1, 1)))

    with pytest.raises(ValueError):
        cloud.set_key(key[:100])

    with pytest.raises(ValueError):
        cloud.set_key(np.dstack((key, key)))

    # 1dim key, mono, (HxW) size ===========================
    label.set_text("Cloud RGB: MONO set..key()")
    cloud.set_key(key)
    show_viz()

    # + set_key_alpha() ====================================
    label.set_text("Cloud RGB: add set..key..alpha() to parts")
    key_alpha = np.full(1024, 1.0, dtype=float)
    key_alpha[points[1, :] > 0] = 0.3
    cloud.set_key_alpha(key_alpha)
    show_viz()

    # + set_mask(rgba) =====================================
    label.set_text("Cloud RGB: add set..mask() 1/3 of cloud "
                   "in RED with transparency")
    ones = np.ones([1024, 1], dtype=float)
    mask_rgba = np.hstack((0.1 * ones, 0.1 * ones, 0.1 * ones, 0.5 * ones))
    mask_rgba[points[0, :] > 5.0] = np.array([0.8, 0.1, 0.1, 0.8])
    cloud.set_mask(mask_rgba)
    show_viz()

    # remove mask
    zeros = np.zeros([1024, 4], dtype=float)
    cloud.set_mask(zeros)

    # 2dim key, mono, (HxW) size ===========================
    label.set_text("Cloud RGB: MONO, set..key() 2dim, no mask, but key alpha "
                   "left")
    key_2dim = key.reshape((1, 1024))
    cloud.set_key(key_2dim)
    show_viz()

    # 3dim key, mono, (HxW) size ===========================
    label.set_text("Cloud RGB: MONO, set..key() 3dim, x 0.5, no mask, but key "
                   "alpha left")
    key_3dim = key.reshape((1, 1024, 1)) * 0.5
    cloud.set_key(key_3dim)
    show_viz()

    # 3dim key, color 3 channel RGB, (HxWx3) size ==========
    label.set_text("Cloud RGB: RGB, set..key() 3dim, no mask, but key alpha "
                   "left")
    key_3dim_rgb = np.dstack(
        (key, np.full(1024, 0.2, dtype=float), np.full(1024, 0.2, dtype=float)))
    cloud.set_key(key_3dim_rgb)
    show_viz()

    # 3dim key, color 4 channel RGBA, (HxWx4) size =========
    label.set_text("Cloud RGB: RGBA, set..key() 3dim, no mask, key alpha "
                   "should be overwritten by key")
    key_3dim_rgba = np.dstack(
        (np.full(1024, 0.1, dtype=float), np.full(1024, 1.0, dtype=float),
         np.full(1024, 0.1, dtype=float), key))
    cloud.set_key(key_3dim_rgba)
    show_viz()

    # millions of points ===================================
    label.set_text("Cloud RGB: Add big cloud")
    many_points_num = 5 * 10**6
    big_cloud = viz.Cloud(many_points_num)
    big_points = np.random.rand(3, many_points_num).astype(np.float32)

    big_key = big_points[0]
    big_points = big_points * 300 - 14

    big_cloud.set_xyz(big_points)
    big_cloud.set_key(big_key)
    point_viz.add(big_cloud)
    show_viz()


def test_point_viz_destruction() -> None:
    """Check that PointViz is destroyed deterministically."""
    point_viz = viz.PointViz("Test Viz")
    ref = weakref.ref(point_viz)

    del point_viz
    assert ref() is None


@pytest.mark.parametrize('test_key', ['single-2.3'])
def test_scan_viz_destruction(meta: client.SensorInfo,
                              point_viz: viz.PointViz) -> None:
    """Check that LidarScan is destroyed deterministically."""
    ls_viz = viz.LidarScanViz(meta, point_viz)
    ref = weakref.ref(ls_viz)

    del ls_viz
    assert ref() is None


@pytest.mark.parametrize('test_key', ['single-2.3'])
def test_viz_multiple_instances(meta: client.SensorInfo,
                                scan: client.LidarScan) -> None:
    """Check that destructing a visualizer doesn't break other instances."""
    point_viz = viz.PointViz("Test Viz")

    # will call destructor, make sure it doesn't do anything silly like terminate glfw
    point_viz2 = viz.PointViz("Test Viz2")
    del point_viz2

    ls_viz = viz.LidarScanViz(meta, point_viz)

    ls_viz.update(scan)
    ls_viz.draw()
    point_viz.run()


def test_scan_viz_smoke(meta: client.SensorInfo,
                        scan: client.LidarScan) -> None:
    """Smoke test LidarScan visualization."""
    ls_viz = viz.LidarScanViz(meta)

    ls_viz.update(scan)
    ls_viz.draw()
    ls_viz.run()


@pytest.mark.parametrize('test_key', ['single-2.3'])
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

    ls_viz.update(scan)

    point_viz.camera.dolly(150)
    ls_viz.draw()
    point_viz.run()


class LidarScanVizWithCallbacks(viz.LidarScanViz):
    """Add callbacks for pre-draw and post-draw"""

    def __init__(
        self,
        meta: client.SensorInfo,
        viz: Optional[viz.PointViz] = None,
        pre_draw_callback: Optional[Callable[[client.LidarScan],
                                             client.LidarScan]] = None,
        post_draw_callback: Optional[Callable[['LidarScanVizWithCallbacks'],
                                              None]] = None
    ) -> None:
        super().__init__(meta, viz)

        self._pre_draw_callback = pre_draw_callback
        self._post_draw_callback = post_draw_callback

    def _draw(self) -> None:
        """Overriding the draw and setting pre/post callbacks"""

        # pre draw callbacl takes LidarScan and returns LidarScan
        if self._pre_draw_callback:
            self._scan = self._pre_draw_callback(self._scan)

        # call original draw
        super()._draw()

        # post draw callbacks takes LidarScanViz object so it can get access
        # to _clouds and _images within it and set additional masks, colors, etc.
        if self._post_draw_callback:
            self._post_draw_callback(self)


@pytest.mark.parametrize('test_key', ['single-2.3'])
def test_simple_viz_with_callbacks(meta: client.SensorInfo,
                                   scan: client.LidarScan) -> None:
    """Call the callback on pre/post draw example."""

    from itertools import repeat
    from copy import deepcopy

    start_range = 1    # in meters
    num_steps = 100

    # this can be any scan source (just a repeater as an example)
    scans = repeat(scan, num_steps)

    point_viz = viz.PointViz("Test Viz")

    scan_cnt = 0

    def pre_draw(ls: client.LidarScan) -> client.LidarScan:
        nonlocal scan_cnt

        nls = deepcopy(ls)
        ratio = scan_cnt / num_steps

        # modifying range in some way
        range = nls.field(client.ChanField.RANGE)
        range = start_range * 1000 + (range - start_range * 1000) * ratio
        nls.field(client.ChanField.RANGE)[:] = range

        scan_cnt += 1
        # don't forget to return it back
        return nls

    def post_draw(scan_viz: LidarScanVizWithCallbacks) -> None:

        # currently discplayed LidarScan
        ls = scan_viz._scan

        ratio = scan_cnt / num_steps
        img_mask = np.zeros((ls.h, ls.w, 4))
        col_idx = int(ls.w * ratio)
        img_mask[:, col_idx - 5:col_idx + 5] = np.array([1.0, 0.3, 0.3, 1.0])

        # there are 2 images - single return and second
        # can safely skip the second, but here we draw on both smth
        for img in scan_viz._images:
            # set some mask
            img.set_mask(img_mask)

        # same for clouds, first and second return
        for cloud in scan_viz._clouds:
            # set some mask
            cloud.set_mask(img_mask)

    # regular LidarScanViz
    # ls_viz = viz.LidarScanViz(meta, point_viz)

    ls_viz = LidarScanVizWithCallbacks(meta,
                                       point_viz,
                                       pre_draw_callback=pre_draw,
                                       post_draw_callback=post_draw)

    viz.SimpleViz(ls_viz, rate=1.0).run(scans)

    print("Done")

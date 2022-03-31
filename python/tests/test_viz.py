import weakref
from typing import TYPE_CHECKING

import pytest
import numpy as np

from ouster import client

# test env may not have opengl, but all test modules are imported during
# collection. Import is still needed to typecheck
if TYPE_CHECKING:
    from ouster.sdk import viz
else:
    viz = pytest.importorskip('ouster.sdk.viz')

# mark all tests in this module so they only run with the --interactive flag
pytestmark = pytest.mark.interactive


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

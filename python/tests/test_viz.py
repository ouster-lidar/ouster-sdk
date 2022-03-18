import weakref

import pytest
import numpy as np

from ouster import client

# test env may not have opengl, but all test modules are imported during
# collection. Note that to avoid errors when viz isn't imported, all type
# annotations from the viz module have to be strings
try:
    from ouster.sdk import viz
except ImportError:
    pass

# mark all tests in this module so they only run with the --interactive flag
pytestmark = pytest.mark.interactive


@pytest.fixture
def point_viz() -> 'viz.PointViz':
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


def test_point_viz_image(point_viz: 'viz.PointViz') -> None:
    """Test displaying a full-window image."""
    img = viz.Image()
    img.set_position(-4 / 3, 4 / 3, -1, 1)
    img.set_image(
        np.tile(np.linspace(0.0, 1.0, 640, dtype=np.float32), (480, 1)))
    point_viz.add(img)

    point_viz.update()
    point_viz.run()


def test_point_viz_labels(point_viz: 'viz.PointViz') -> None:
    """Smoke test rendering labels."""

    label1 = viz.Label3d(np.array([0, 0, 1]), "Foobar")
    label2 = viz.Label3d(np.array([0, 0, 0]), "Baz\nQux")
    point_viz.add(label1)
    point_viz.add(label2)
    point_viz.camera.dolly(250)
    point_viz.update()
    point_viz.run()


def test_point_viz_destruction() -> None:
    """Check that PointViz is destroyed deterministically."""
    point_viz = viz.PointViz("Test Viz")
    ref = weakref.ref(point_viz)

    del point_viz
    assert ref() is None


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_scan_viz_destruction(meta: client.SensorInfo,
                              point_viz: 'viz.PointViz') -> None:
    """Check that LidarScan is destroyed deterministically."""
    # point_viz = viz.PointViz("Test Viz")
    ls_viz = viz.LidarScanViz(point_viz, meta)
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

    ls_viz = viz.LidarScanViz(point_viz, meta)

    ls_viz.scan = scan
    ls_viz.update()
    point_viz.run()


def test_scan_viz_smoke(meta: client.SensorInfo,
                        scan: client.LidarScan) -> None:
    """Smoke test LidarScan visualization."""
    point_viz = viz.PointViz("Test Viz")
    ls_viz = viz.LidarScanViz(point_viz, meta)

    ls_viz.scan = scan
    ls_viz.update()
    point_viz.run()


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_scan_viz_extras(meta: client.SensorInfo,
                         scan: client.LidarScan) -> None:
    """Check rendering of labels, cuboids, clouds and images together."""
    point_viz = viz.PointViz("Test Viz")
    ls_viz = viz.LidarScanViz(point_viz, meta)

    pose1 = np.identity(4, np.float32)
    cube1 = viz.Cuboid(pose1, np.array([1, 0, 0, 1], np.float32))

    pose2 = np.diag([1, 2, 1, 1]) * pose1
    pose2[3, 0] += 5
    cube2 = viz.Cuboid(pose2, np.array([0, 1, 0, 1], np.float32))

    label1 = viz.Label3d(np.array([0, 0, 2]), "Baz\nQux")
    point_viz.add(label1)
    point_viz.add(cube1)
    point_viz.add(cube2)

    ls_viz.scan = scan

    point_viz.camera.dolly(150)
    ls_viz.update()
    point_viz.run()

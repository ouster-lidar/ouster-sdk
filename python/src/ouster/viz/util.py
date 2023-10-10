from typing import Optional, TypeVar, Callable, List

import weakref
import numpy as np

import ouster.sdk.pose_util as pu

from ._viz import PointViz, Cloud, Label, WindowCtx


T = TypeVar('T')


def push_point_viz_handler(
        viz: PointViz, arg: T, handler: Callable[[T, WindowCtx, int, int],
                                                 bool]) -> None:
    """Add a key handler with extra context without keeping it alive.

    It's often useful to add a key callback that calls a method of an object
    that wraps a PointViz instance. In this case it's necessary to take some
    extra care to avoid a reference cycle; holding onto self in the callback
    passed to native code would cause a memory leak.

    Args:
        viz: The PointViz instance.
        arg: The extra context to pass to handler; often `self`.
        handler: Key handler callback taking an extra argument
    """
    weakarg = weakref.ref(arg)

    def handle_keys(ctx: WindowCtx, key: int, mods: int) -> bool:
        arg = weakarg()
        if arg is not None:
            return handler(arg, ctx, key, mods)
        return True

    viz.push_key_handler(handle_keys)


def push_point_viz_fb_handler(
        viz: PointViz, arg: T, handler: Callable[[T, List, int, int],
                                                 bool]) -> None:
    """Add a frame buffer handler with extra context without keeping it alive.

    See docs for `push_point_viz_handler()` method above for details.

    Args:
        viz: The PointViz instance.
        arg: The extra context to pass to handler; often `self`.
        handler: Frame buffer handler callback taking an extra argument
    """
    weakarg = weakref.ref(arg)

    def handle_fb_data(fb_data: List, fb_width: int, fb_height: int) -> bool:
        arg = weakarg()
        if arg is not None:
            return handler(arg, fb_data, fb_width, fb_height)
        return True

    viz.push_frame_buffer_handler(handle_fb_data)


def _cloud_axis_points(axis_length: float = 1.0) -> np.ndarray:
    """Generate coordinate axis point cloud."""

    # basis vectors
    x_ = np.array([1, 0, 0]).reshape((-1, 1))
    y_ = np.array([0, 1, 0]).reshape((-1, 1))
    z_ = np.array([0, 0, 1]).reshape((-1, 1))

    axis_n = 100
    line = np.linspace(0, axis_length, axis_n).reshape((1, -1))

    # basis vector to point cloud
    axis_points = np.hstack(
        (x_ @ line,
        y_ @ line,
        z_ @ line)).transpose()

    return axis_points


def _make_cloud_axis(axis_points) -> Cloud:
    """Create viz.Cloud object with colors from coordinate axis points"""

    axis_n = int(axis_points.shape[0] / 3)

    # colors for basis vectors
    axis_color_mask = np.vstack((np.full(
        (axis_n, 4), [1, 0.1, 0.1, 1]), np.full((axis_n, 4), [0.1, 1, 0.1, 1]),
                                 np.full((axis_n, 4), [0.1, 0.1, 1, 1])))

    cloud_axis = Cloud(axis_points.shape[0])
    cloud_axis.set_xyz(axis_points)
    cloud_axis.set_key(np.full(axis_points.shape[0], 0.5))
    # TODO[pb]: To use set_key(rgb) instead of set_mask() for colors
    cloud_axis.set_mask(axis_color_mask)
    cloud_axis.set_point_size(3)
    return cloud_axis


class AxisWithLabel:
    """Coordinate axis with a text label."""

    def __init__(self,
                 point_viz: PointViz,
                 *,
                 pose: pu.PoseH = np.eye(4),
                 label: str = "",
                 length: float = 1.0,
                 thickness: int = 3,
                 label_scale: Optional[float] = None,
                 enabled: bool = True):
        self._viz = point_viz
        self._pose = pose
        self._label = label

        self._axis_cloud = _make_cloud_axis(_cloud_axis_points(length))
        self._axis_cloud.set_point_size(thickness)
        self._axis_cloud.set_pose(self._pose)
        self._axis_label = Label(self._label, *pose[:3, 3])
        if label_scale:
            self._axis_label.set_scale(label_scale)

        self._enabled = False
        if enabled:
            self.enable()

    @property
    def enabled(self) -> bool:
        """True if label is added to the viz"""
        return self._enabled

    def enable(self) -> None:
        """Enable the label and make it added to the viz"""
        if not self._enabled:
            self._viz.add(self._axis_cloud)
            self._viz.add(self._axis_label)
            self._enabled = True

    def disable(self) -> None:
        """Disable the label and remove it from the viz"""
        if self._enabled:
            self._viz.remove(self._axis_cloud)
            self._viz.remove(self._axis_label)
            self._enabled = False

    def toggle(self) -> bool:
        """Toggle the label visibility (i.e. presence in the viz)"""
        if not self._enabled:
            self.enable()
        else:
            self.disable()
        return self._enabled

    @property
    def pose(self) -> np.ndarray:
        """Label pose, 4x4 matrix"""
        return self._pose

    @pose.setter
    def pose(self, pose: np.ndarray):
        """Set label pose, 4x4 matrix, and update internal states"""
        self._pose = pose
        self.update()

    @property
    def label(self) -> str:
        """Label text, 4x4 matrix"""
        return self._label

    @label.setter
    def label(self, label_text: str):
        """Set label text, and update internal states"""
        self._label = label_text
        self.update()

    def update(self) -> None:
        """Update label component viz states."""
        self._axis_cloud.set_pose(self._pose)
        self._axis_label.set_position(*self._pose[:3, 3])
        self._axis_label.set_text(self._label)

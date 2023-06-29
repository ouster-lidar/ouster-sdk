from typing import Optional

import numpy as np

import ouster.sdk.pose_util as pu

import ouster.sdk.viz as viz


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


def _make_cloud_axis(axis_points) -> viz.Cloud:
    """Create viz.Cloud object with colors from coordinate axis points"""

    axis_n = int(axis_points.shape[0] / 3)

    # colors for basis vectors
    axis_color_mask = np.vstack((np.full(
        (axis_n, 4), [1, 0.1, 0.1, 1]), np.full((axis_n, 4), [0.1, 1, 0.1, 1]),
                                 np.full((axis_n, 4), [0.1, 0.1, 1, 1])))

    cloud_axis = viz.Cloud(axis_points.shape[0])
    cloud_axis.set_xyz(axis_points)
    cloud_axis.set_key(np.full(axis_points.shape[0], 0.5))
    # TODO[pb]: To use set_key(rgb) instead of set_mask() for colors
    cloud_axis.set_mask(axis_color_mask)
    cloud_axis.set_point_size(3)
    return cloud_axis


class AxisWithLabel:
    """Coordinate axis with a text label."""

    def __init__(self,
                 point_viz: viz.PointViz,
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
        self._axis_label = viz.Label(self._label, *pose[:3, 3])
        if label_scale:
            self._axis_label.set_scale(label_scale)

        self._enabled = False
        if enabled:
            self.enable()

    @property
    def enabled(self):
        return self._enabled

    def enable(self):
        if not self._enabled:
            self._viz.add(self._axis_cloud)
            self._viz.add(self._axis_label)
            self._enabled = True

    def disable(self):
        if self._enabled:
            self._viz.remove(self._axis_cloud)
            self._viz.remove(self._axis_label)
            self._enabled = False

    def toggle(self):
        if not self._enabled:
            self.enable()
        else:
            self.disable()

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, pose: np.ndarray):
        self._pose = pose
        self.update()

    @property
    def label(self):
        return self._label

    @label.setter
    def label(self, label_text: str):
        self._label = label_text
        self.update()

    def update(self) -> None:
        """Update viz component states."""
        self._axis_cloud.set_pose(self._pose)
        self._axis_label.set_position(*self._pose[:3, 3])
        self._axis_label.set_text(self._label)

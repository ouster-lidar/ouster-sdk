import numpy as np

from ouster.sdk._bindings.viz import PointViz, Cloud, Label


def _cloud_axis_points(axis_length: float = 1.0, axis_n: int = 100) -> np.ndarray:
    """Generate coordinate axis point cloud."""

    # basis vectors
    x_ = np.array([1, 0, 0], dtype=np.float32).reshape((-1, 1))
    y_ = np.array([0, 1, 0], dtype=np.float32).reshape((-1, 1))
    z_ = np.array([0, 0, 1], dtype=np.float32).reshape((-1, 1))

    line = np.linspace(0, axis_length, axis_n, dtype=np.float32).reshape((1, -1))

    # basis vector to point cloud
    axis_points = np.hstack((x_ @ line, y_ @ line, z_ @ line)).transpose()

    return np.ascontiguousarray(axis_points)


def _make_cloud_axis(axis_points) -> Cloud:
    """Create viz.Cloud object with colors from coordinate axis points"""

    axis_n = int(axis_points.shape[0] / 3)
    axes_rgba = np.vstack((np.full((axis_n, 4), [1.0, 0.1, 0.1, 1.0], np.float32),
                           np.full((axis_n, 4), [0.1, 1.0, 0.1, 1.0], np.float32),
                           np.full((axis_n, 4), [0.1, 0.1, 1.0, 1.0], np.float32)))

    cloud_axis = Cloud(axis_points.shape[0])
    cloud_axis.set_xyz(axis_points)
    cloud_axis.set_key(axes_rgba)
    cloud_axis.set_point_size(3)
    return cloud_axis


class AxisWithLabel:
    """Coordinate axis with a text label."""

    def __init__(self,
                 point_viz: PointViz,
                 *,
                 pose: np.ndarray = np.eye(4),
                 label: str = "",
                 length: float = 1.0,
                 thickness: int = 3,
                 label_scale: float = 1.0,
                 axis_n: int = 100,
                 enabled: bool = True):
        self._viz = point_viz
        self._pose = pose
        self._label = label

        self._axis_cloud = _make_cloud_axis(_cloud_axis_points(length, axis_n))
        self._axis_cloud.set_point_size(thickness)
        self._axis_cloud.set_pose(self._pose)
        if label:
            self._axis_label = Label(self._label, *pose[:3, 3])
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
            if self._label:
                self._viz.add(self._axis_label)
            self._enabled = True

    def disable(self) -> None:
        """Disable the label and remove it from the viz"""
        if self._enabled:
            self._viz.remove(self._axis_cloud)
            if self._label:
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
        if self._label:
            self._axis_label.set_position(*self._pose[:3, 3])
            self._axis_label.set_text(self._label)

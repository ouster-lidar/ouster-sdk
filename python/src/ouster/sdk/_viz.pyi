"""Type annotations for viz python bindings."""

from typing import Callable, overload

import numpy as np

from ..client import SensorInfo

calref_palette: np.ndarray
spezia_palette: np.ndarray


class WindowCtx:

    @property
    def lbutton_down(self) -> bool:
        ...

    @property
    def mbutton_down(self) -> bool:
        ...

    @property
    def mouse_x(self) -> float:
        ...

    @property
    def mouse_y(self) -> float:
        ...

    @property
    def window_width(self) -> int:
        ...

    @property
    def window_height(self) -> int:
        ...


class Camera:

    def reset(self) -> None:
        ...

    def yaw(self, degrees: float) -> None:
        ...

    def pitch(self, degrees: float) -> None:
        ...

    def dolly(self, amount: int) -> None:
        ...

    def dolly_xy(self, x: float, y: float) -> None:
        ...

    def set_fov(self, degrees: float) -> None:
        ...

    def set_orthographic(self, state: bool) -> None:
        ...

    def set_proj_offset(self, x: float, y: float) -> None:
        ...


class TargetDisplay:

    def enable_rings(self, state: bool) -> bool:
        ...

    def set_ring_size(self, n: int) -> None:
        ...


class Cloud:

    def __init__(self, si: SensorInfo) -> None:
        ...

    def set_range(self, range: np.ndarray) -> None:
        ...

    def set_key(self, key: np.ndarray) -> None:
        ...

    def set_point_size(self, size: float) -> None:
        ...

    def set_palette(self, palette: np.ndarray) -> None:
        ...


class Image:

    def __init__(self) -> None:
        ...

    def set_image(self, image: np.ndarray) -> None:
        ...

    def set_mask(self, image: np.ndarray) -> None:
        ...

    def set_position(self, x0: float, x1: float, y0: float, y1: float) -> None:
        ...


class Cuboid:

    def __init__(self, pose: np.ndarray, rgba: np.ndarray) -> None:
        ...

    def set_pose(self, pose: np.ndarray) -> None:
        ...

    def set_rgba(self, rgba: np.ndarray) -> None:
        ...


class Label3d:

    def __init__(self, xyz: np.ndarray, text: str) -> None:
        ...

    def set_position(self, xyz: np.ndarray) -> None:
        ...

    def set_text(self, text: str) -> None:
        ...


class PointViz:

    def __init__(self,
                 name: str,
                 fix_aspect: bool = ...,
                 window_width: int = ...,
                 window_height: int = ...) -> None:
        ...

    def run(self) -> None:
        ...

    @overload
    def running(self) -> bool:
        ...

    @overload
    def running(self, state: bool) -> None:
        ...

    def update(self) -> bool:
        ...

    def push_key_handler(self, f: Callable[[WindowCtx, int, int],
                                           bool]) -> None:
        ...

    @property
    def camera(self) -> Camera:
        ...

    @property
    def target_display(self) -> TargetDisplay:
        ...

    @overload
    def add(self, cloud: Cloud) -> None:
        ...

    @overload
    def add(self, image: Image) -> None:
        ...

    @overload
    def add(self, cuboid: Cuboid) -> None:
        ...

    @overload
    def add(self, label: Label3d) -> None:
        ...

    @overload
    def remove(self, cloud: Cloud) -> None:
        ...

    @overload
    def remove(self, image: Image) -> None:
        ...

    @overload
    def remove(self, cuboid: Cuboid) -> None:
        ...

    @overload
    def remove(self, label: Label3d) -> None:
        ...


def add_default_controls(viz: PointViz) -> None:
    ...

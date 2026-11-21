import numpy as np
import json
import weakref
from enum import Enum
from typing import List, Dict, Tuple, Callable, Any
from functools import partial
from ouster.sdk._bindings.viz import PointViz, Cloud, Label, WindowCtx


class VizOption:
    def __init__(self, name: str, default, key, on_change,
                 min=None, max=None, decrement=None, help=None, save=True,
                 items=None, get_value=None):
        ty = type(default)
        assert name != "camera"
        assert help is not None
        self.type = ty
        self.name = name
        if ty is bool:
            assert min is None
            assert max is None
            self.value = default
        elif ty is int:
            self.value = default
            assert min is not None
            assert max is not None
            self.min = min
            self.max = max
        elif isinstance(default, Enum):
            assert min is None
            assert max is None
            self.value = default
        elif ty is str:
            assert items is not None
            self.value = default
        else:
            raise RuntimeError("Unsupported option type")
        self.on_change = on_change
        self.key = key[0]
        self.mod = key[1] if len(key) > 1 else 0
        self.decrement = decrement
        self.on_change2 = None
        self.help = help
        self.save = save
        self.get_items = items
        self.get_value = get_value

    def load_value(self, arg, string: str):
        if isinstance(self.value, Enum):
            new_value = self.type[string]
        elif self.type is bool:
            new_value = string == "True"
        else:
            new_value = self.type(string)
        if new_value != self.value:
            self.value = new_value
            self.on_change(arg, self.value)
            if self.on_change2:
                self.on_change2(arg)

    def serialize_value(self, arg) -> str:
        if self.get_value is not None:
            value = self.get_value(arg)
        else:
            value = self.value
        if isinstance(self.value, Enum):
            return value.name
        return str(value)

    def on_key_toggle(self, arg):
        if self.get_value:
            self.value = self.get_value(arg)
        if isinstance(self.value, Enum):
            modes = list(self.type)
            current_index = modes.index(self.value)
            next_index = (current_index + 1) % len(modes)
            new_value = modes[next_index]
        elif self.type is int:
            new_value = self.min + (self.value - self.min + 1) % (self.max - self.min + 1)
        elif isinstance(self.value, str):
            modes = self.get_items(arg)
            try:
                current_index = modes.index(self.value)
            except ValueError:
                current_index = 0
            next_index = (current_index + 1) % len(modes)
            new_value = modes[next_index]
        else:
            # bool!
            new_value = not self.value

        if new_value != self.value:
            self.value = new_value
            self.on_change(arg, self.value)
            if self.on_change2:
                self.on_change2(arg)

    def on_key_increment(self, amt, arg):
        if self.get_value:
            self.value = self.get_value(arg)
        if isinstance(self.value, Enum):
            modes = list(self.type)
            current_index = modes.index(self.value)
            if amt > 0:
                next_index = (current_index + amt) % len(modes)
            else:
                next_index = current_index + amt
                while next_index < 0:
                    next_index += len(modes)
            new_value = modes[next_index]
        elif isinstance(self.value, str):
            modes = self.get_items(arg)
            try:
                current_index = modes.index(self.value)
            except ValueError:
                current_index = 0
            if amt > 0:
                next_index = (current_index + amt) % len(modes)
            else:
                next_index = current_index + amt
                while next_index < 0:
                    next_index += len(modes)
            new_value = modes[next_index]
        else:
            new_value = max(self.min, min(self.value + amt, self.max))
        if new_value != self.value:
            self.value = new_value
            self.on_change(arg, self.value)
            if self.on_change2:
                self.on_change2(arg)


class VizConfig:
    def save(self) -> str:
        out: Dict[str, Any] = {}
        arg = self.get_arg()
        for option in self.options:
            if not option.save:
                continue
            out[option.name] = option.serialize_value(arg)
        viz = self.weak_viz()
        assert viz
        cam = {}
        cam["yaw"] = viz.camera.get_yaw()
        cam["pitch"] = viz.camera.get_pitch()
        cam["roll"] = viz.camera.get_roll()
        cam["dolly"] = viz.camera.get_dolly()
        cam["ortho"] = viz.camera.is_orthographic()
        cam["fov"] = viz.camera.get_fov()
        offset = viz.camera.get_view_offset()
        cam["offset_x"] = offset[0]
        cam["offset_y"] = offset[1]
        cam["offset_z"] = offset[2]
        out["camera"] = cam
        return json.dumps(out)

    def key_to_str(self, char, mod) -> str:
        key = str(char).lower()
        mod = mod or 0
        if (mod & 1) != 0:
            key = "SHIFT+" + key
        if (mod & 2) != 0:
            key = "CTRL+" + key
        if (mod & 4) != 0:
            key = "ALT+" + key
        return key

    def key_definitions(self) -> Dict[str, str]:
        out: Dict[str, str] = {}
        for option in self.options:
            key = self.key_to_str(option.key, option.mod)
            if option.decrement:
                key += " / " + self.key_to_str(option.decrement[0], option.decrement[1])
            out[key] = option.help or option.name
        return out

    def load(self, cfg: str):
        data = json.loads(cfg)
        arg = self.get_arg()
        # load options
        for option in self.options:
            if option.name not in data:
                continue
            option.load_value(arg, data[option.name])
        cam = data.get("camera", {})
        viz = self.weak_viz()
        assert viz
        if "yaw" in cam:
            viz.camera.set_yaw(cam["yaw"])
        if "pitch" in cam:
            viz.camera.set_pitch(cam["pitch"])
        if "roll" in cam:
            viz.camera.set_roll(cam["roll"])
        if "dolly" in cam:
            viz.camera.set_dolly(cam["dolly"])
        if "ortho" in cam:
            viz.camera.set_orthographic(cam["ortho"])
        if "fov" in cam:
            viz.camera.set_fov(cam["fov"])
        offset = (cam.get("offset_x", 0), cam.get("offset_y", 0), cam.get("offset_z", 0))
        viz.camera.set_view_offset(offset)
        viz.update()

    def __init__(self, viz: PointViz, get_arg, on_change=None):
        self.options: List[VizOption] = []
        self.weak_viz = weakref.ref(viz)
        self.on_change = on_change

        self.key_bindings: Dict[Tuple[int, int], Callable[[Any], None]] = {}

        def handle_keys(get_arg, ctx: WindowCtx, key: int, mods: int) -> bool:
            if (key, mods) in self.key_bindings:
                arg = get_arg()
                self.key_bindings[key, mods](arg)
            return True

        self.get_arg = get_arg
        viz.push_key_handler(partial(handle_keys, get_arg))

    def option(self, *args, **kwargs):
        opt = VizOption(*args, **kwargs)
        # add to bindings
        if opt.decrement is not None:
            self.key_bindings[(ord(opt.key), opt.mod)] = partial(opt.on_key_increment, 1)
            self.key_bindings[(ord(opt.decrement[0]), opt.decrement[1])] = partial(opt.on_key_increment, -1)
        else:
            self.key_bindings[(ord(opt.key), opt.mod)] = opt.on_key_toggle

        if self.on_change:
            opt.on_change2 = self.on_change
        self.options.append(opt)


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

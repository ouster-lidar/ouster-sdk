"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Sensor data visualization tools.

Visualize lidar data using OpenGL.
"""

from collections import deque
from functools import partial
from enum import Enum, auto
import itertools
import threading
import queue
import time
from datetime import datetime, timezone
from typing import (Callable, ClassVar, cast, Deque, Dict, Iterable, Iterator, List,
                    Optional, Tuple, TypeVar, Union)
import logging

import numpy as np

from ouster.sdk._deprecation import warn_deprecated
from ouster.sdk import core
from ouster.sdk.core import LidarFrame, ChanField, FrameSet, FrameSetSource, SensorInfo
from ouster.sdk._bindings.viz import (PointVizNotRunningError, PointViz, Cloud, Label, WindowCtx,
                   add_default_controls, MouseButton, MouseButtonEvent, EventModifierKeys,
                   distinct_palette, MouseEventType)
from ouster.sdk._bindings.viz import Image, Cuboid, Camera, TargetDisplay, ObjectOverlay  # noqa: F401
from ouster.sdk.viz.model import VizExtraMode, LidarFrameVizModel, \
        ObjectColorMode, ObjectViewMode, discrete_rainbow_palette  # noqa: F401
from . import util as vizu
from ouster.sdk.viz.view_mode import ImageMode, CloudMode, CloudPaletteItem  # noqa: F401
from ouster.sdk.viz.accumulators import LidarFrameVizAccumulators
from ouster.sdk.viz.accumulators_config import LidarFrameVizAccumulatorsConfig, MAP_MAX_POINTS_NUM, MAP_SELECT_RATIO
from ouster.sdk.viz.model import SensorModel, ZoneSelectionMode, ZoneRenderMode
import weakref

import PIL.Image as PILImage

logger = logging.getLogger("viz-logger")

T = TypeVar('T')


class ImuVisualizationConfig:
    imu_plot_width_pixels = 1000
    imu_plot_height_pixels = 28
    downsample_factor = 2
    field_colors = {
        'IMU_ACC': (1.0, 0.0, 0.0, 1.0),
        'IMU_GYRO': (1.0, 1.0, 1.0, 1.0),
    }

    def __init__(self, options='only_gyro') -> None:
        if options == 'only_gyro':
            self._fields = ['IMU_GYRO']
        elif options == 'only_acc':
            self._fields = ['IMU_ACC']
        elif options == 'both':
            self._fields = ['IMU_ACC', 'IMU_GYRO']
        elif options == 'none':
            self._fields = []


def _mouse_button_handler(weak_self, ctx: WindowCtx,
    button: MouseButton, event: MouseButtonEvent, mods: EventModifierKeys) -> bool:
    self = weak_self()
    with self._lock:
        ret = self._model.mouse_button_handler(ctx, button, event, mods)
        if not ret:
            self._flags_mode = LidarFrameViz.FlagsMode.NONE
            self._update_multi_viz_osd()
            self._viz.update()
            self._model.update_aoi_label(self._frame_set)
        return ret


def _mouse_pos_handler(weak_self, ctx: WindowCtx, x: float, y: float) -> bool:
    self = weak_self()

    # TODO: law of Demeter
    if self._model._mouse_down_image is not None and self._model._current_selection:
        selection = self._model._current_selection[self._model._mouse_down_image]
        if selection.finalized:
            # if selection is finalized, exit early
            return True
    else:
        # if we're not making a selection, then exit early
        if not self._model._pick_start:
            return True

    with self._lock:
        ret = self._model.mouse_pos_handler(ctx, x, y)
        if not ret:
            self._model.update_aoi_label(self._frame_set)
            self._viz.update()
        return ret


def _frame_buffer_resize_handler(weak_self, ctx: WindowCtx):
    self = weak_self()
    with self._lock:
        self._model.update_aoi(ctx)
        self._update_multi_viz_osd()
        self._viz.update()
        return True


def _valid_or_zero_ts(frame: LidarFrame):
    # TODO: this may need addressing or refactoring
    try:
        return frame.get_first_valid_packet_timestamp()
    except RuntimeError:
        return 0


def _get_timestamp(frames: FrameSet):
    return min([_valid_or_zero_ts(s) for s in frames if s])


class LidarFrameViz:
    """Multi LidarFrame clouds visualizer

    Uses the supplied PointViz instance to easily display the contents of a
    LidarFrame. Sets up key bindings to toggle which channel fields and returns
    are displayed, and change 2D image and point size.
    """
    class OsdState(Enum):
        NONE = auto(),
        DEFAULT = auto(),
        HELP = auto()

    class FlagsMode(Enum):
        NONE = 0
        HIGHLIGHT_SECOND = 1
        HIGHLIGHT_BLOOM = 2
        HIDE_BLOOM = 3
        HIGHLIGHT_ZONES = 4
        HIGHLIGHT_OBJECTS = 5

    class ImagesLayout(Enum):
        HORIZONTAL = 0
        VERTICAL = 1

    class CameraMode(Enum):
        FOLLOW = 0
        FOLLOW_ROTATION_LOCKED = 1
        FIXED = 2
        FOLLOW_SMOOTH = 3
        FOLLOW_SMOOTH_ROTATION_LOCK = 4

    class TrackViewMode(Enum):
        OFF = 0
        TRACK_ONLY = 1
        TRACK_PLUS_PER_FRAME_TRAJECTORY = 2

    class ImageViewMode(Enum):
        ALL = 0
        ALL_FLIPPED = 1
        ONE = 2
        ONE_FLIPPED = 3

    class FlagsPalette(Enum):
        RAINBOW = 0
        DISTINCT = 1

    def set_interpolant(self, interpolant: float) -> None:
        with self._lock:
            self._interpolant = interpolant

    def clear_pose_histories(self) -> None:
        """Clear frame/camera pose histories used for interpolation."""
        with self._lock:
            self._frame_pose_history.clear()
            self._camera_pose_history.clear()
            self._interpolant = 1.0

    def __init__(
            self,
            metas: List[SensorInfo],
            point_viz: Optional[PointViz] = None,
            accumulators_config: Optional[LidarFrameVizAccumulatorsConfig] = None,
            imu_viz_config: ImuVisualizationConfig = ImuVisualizationConfig(),
            *,
            _img_aspect_ratio: float = 0,
            _buflen = 50,
            _add_default_controls = True) -> None:
        """
        Args:
            metas: sensor metadata used to interpret frames
            viz: use an existing PointViz instance instead of creating one
        """
        self._viz = point_viz or PointViz("Ouster Viz")
        self._add_default_controls = _add_default_controls

        # used to synchronize key handlers and _draw()
        self._lock = threading.RLock()
        self._model = LidarFrameVizModel(self._viz, metas, _img_aspect_ratio=_img_aspect_ratio)
        self._imu_acc_scale = [{'IMU_ACC': 1, 'IMU_GYRO': 1} for m in metas]
        self._imu_viz_num_frames = _buflen
        self.update_on_input = True

        # extension point for the OSD text, inserts before the "axes" line
        self._osd_text_extra: Callable[[], str] = lambda: ""

        # set initial image sizes
        self._image_size_initialized = False
        self._image_view_mode = LidarFrameViz.ImageViewMode.ALL

        # initial point size in frame clouds
        for label in self._model._aoi_labels:
            self._viz.add(label)

        self._frames_accum: Optional[LidarFrameVizAccumulators] = None
        if accumulators_config:
            self._frames_accum = LidarFrameVizAccumulators(
                self._model,
                self._viz,
                accumulators_config,
                self._lock
            )

            # TODO[tws]: decide the fate of FramesAccumulator's OSD
            self._frames_accum.toggle_osd(False)
            if hasattr(self, "_osd_text_extra"):
                self._osd_text_extra = self._frames_accum.osd_text
                # if we add the extra text to the _frame_viz we also want to
                # draw(w/o update) its state when frames accum keys cause the
                # re-draw of FramesAccumulator state (osd included)
                self._frames_accum._key_press_pre_draw = (
                    lambda: self.draw(update=False))

        self._imu_viz_config = imu_viz_config
        self._osd_scale = 1

        # add images and clouds to viz
        for sensor in self._model._sensors:
            [self._viz.add(image) for image in sensor._images]
            [self._viz.add(cloud) for cloud in sensor._clouds]
            self._viz.add(sensor._object_overlay)

        self._images_layout = LidarFrameViz.ImagesLayout.HORIZONTAL

        # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
        # initialize masks to just green with zero opacity
        self._flags_mode = LidarFrameViz.FlagsMode.NONE
        self._flags_palette = self.FlagsPalette.RAINBOW

        # initialize rings
        self._ring_size = 1
        self._ring_line_width = 2
        self._viz.target_display.set_ring_line_width(self._ring_line_width)
        self._viz.target_display.set_ring_size(self._ring_size)
        self._viz.target_display.enable_rings(True)

        # initialize osd
        self._osd_state = LidarFrameViz.OsdState.DEFAULT
        self._previous_osd_state = LidarFrameViz.OsdState.NONE
        self._osd = Label("", 0, 1)
        self._viz.add(self._osd)

        # initialize frame axis helpers
        self._frame_axis_enabled = True
        self._frame_axis = []
        # sensors axis
        for idx, sensor in enumerate(self._model._sensors):
            self._frame_axis.append(
                vizu.AxisWithLabel(self._viz,
                                   pose=sensor._meta.sensor_to_body,
                                   label=str(idx + 1),
                                   thickness=3))
        # map origin axis
        self._map_origin_axis = vizu.AxisWithLabel(self._viz,
                                                   label="M",
                                                   thickness=5,
                                                   label_scale=0.4)

        # base link axis
        self._base_link_axis = vizu.AxisWithLabel(self._viz,
                                                  label="O",
                                                  thickness=5,
                                                  label_scale=0.4)

        self._track_view_mode: LidarFrameViz.TrackViewMode = LidarFrameViz.TrackViewMode.TRACK_ONLY
        self._per_frame_pose_trajectory: Deque[Cloud] = deque(maxlen=100)
        self._per_frame_pose_trajectory_axis: Deque[vizu.AxisWithLabel] = deque(maxlen=15)

        self._camera_mode = LidarFrameViz.CameraMode.FOLLOW
        # TODO[UN]: assign a key to select a sensor to be tracked
        # also what to do in case tracked sensor is hidden?
        self._tracked_sensor = 0
        self._frame_pose_history: Deque[np.ndarray] = deque(maxlen=2)
        self._camera_pose_history: Deque[np.ndarray] = deque(maxlen=20)
        self._camera_pose = np.eye(4)
        self._interpolant = 1.0

        self._on_osd_scale: Optional[Callable[[], None]] = None

        self._frame_set: FrameSet = FrameSet()
        self._frame_num = -1
        self._first_frame_ts = None
        self._setup_controls()
        self._zone_selection_mode = ZoneSelectionMode.LIVE
        self._zone_render_mode = ZoneRenderMode.STL
        self.set_zone_selection_and_render_modes(self._zone_selection_mode, self._zone_render_mode)

        if hasattr(self, "_key_definitions") and self._frames_accum:
            self._key_definitions.update(
                getattr(self._frames_accum, "_key_definitions", {}))

    def imu_plot(self, frames, imu_viz_config):
        img_h = self._imu_viz_config.imu_plot_height_pixels // self._imu_viz_config.downsample_factor
        img_w = self._imu_viz_config.imu_plot_width_pixels // self._imu_viz_config.downsample_factor

        def add_field(field_name, img, color, sensor_idx):
            # Find first valid frame to get measurements_per_frame
            # (handles case where frames[0] is None due to collation gaps or missing data)
            first_valid = None
            for s in frames:
                if s[sensor_idx] is not None and s[sensor_idx].has_field(field_name):
                    first_valid = s[sensor_idx]
                    break
            if first_valid is None:
                return  # No valid frames with this field in the buffer
            measurements_per_frame = first_valid.field(field_name).shape[0]
            # Use NaN for missing frames so gaps appear instead of false zeros
            all_frame_values = np.full((measurements_per_frame * len(frames),), np.nan)
            self._imu_acc_scale[sensor_idx][field_name] = 1.0
            for idx, frame in enumerate(frames):
                frame = frames[idx][sensor_idx]
                if frame is None:
                    continue  # NaN values left for missing frames create visual gaps
                field_values = frame.field(field_name)[::-1]
                field_value_magnitudes = np.linalg.norm(field_values, axis=1)
                # Mark invalid IMU measurements as missing.
                if frame.has_field('IMU_STATUS'):
                    imu_status = frame.field('IMU_STATUS')[::-1]
                    field_value_magnitudes = field_value_magnitudes.astype(float)
                    field_value_magnitudes[(imu_status & 0x1) == 0] = np.nan
                valid = field_value_magnitudes[~np.isnan(field_value_magnitudes)]
                max_magnitude = np.max(valid) if len(valid) > 0 else 0
                if max_magnitude > 0:
                    new_scale_amt = 1.0 / max_magnitude
                    if self._imu_acc_scale[sensor_idx][field_name] > new_scale_amt:
                        self._imu_acc_scale[sensor_idx][field_name] = new_scale_amt
                all_frame_values[
                    idx * measurements_per_frame:(idx + 1) * measurements_per_frame
                ] = field_value_magnitudes

            chunk_w = img_w * len(frames) // self._imu_viz_num_frames
            x_axis_values = np.linspace(0, len(all_frame_values) - 1, chunk_w)
            # Interpolate over valid data only (np.interp can't handle NaN)
            valid_mask = ~np.isnan(all_frame_values)
            valid_indices = np.where(valid_mask)[0]
            if len(valid_indices) == 0:
                return
            resampled_field = np.interp(x_axis_values, valid_indices, all_frame_values[valid_mask])
            # Restore NaN for pixels that map into gap regions
            nearest_orig = np.clip(np.round(x_axis_values).astype(int), 0, len(all_frame_values) - 1)
            resampled_field[~valid_mask[nearest_orig]] = np.nan
            # Only plot pixels that have valid data
            valid = ~np.isnan(resampled_field)
            scaled = (img_h - 1 - resampled_field[valid] * (img_h - 1)
                * self._imu_acc_scale[sensor_idx][field_name]).astype(int)
            pixel_x = np.arange(chunk_w)[valid]
            indices = np.column_stack((scaled, pixel_x)).T
            img[indices[0], img_w - indices[1] - 1] = color

        for sensor_idx, sensor in enumerate(self._model._sensors):
            img = np.zeros((img_h, img_w, 4), np.float32)
            assert img.shape[2] == 4
            for field in imu_viz_config._fields:
                add_field(field, img, imu_viz_config.field_colors[field], sensor_idx)
            sensor._imu_plot.set_image(img)

    # TODO[tws] likely remove
    @property
    def metadata(self) -> List[SensorInfo]:
        """Metadatas for the displayed sensors."""
        return self._model._metas

    def set_class_maps(self, class_maps: core.ClassMapSet) -> None:
        """Set the class maps for the visualizer."""
        with self._lock:
            self._model.set_class_maps(class_maps)

    @property
    def osd_state(self) -> "LidarFrameViz.OsdState":
        """Returns the state of the on screen display."""
        return self._osd_state

    def cycle_osd_scale(self):
        self._osd_scale = max(0, (self._osd_scale + 1) % 5)
        if self._on_osd_scale:
            self._on_osd_scale()

    def set_view(self, view: str):
        if view == 'top':
            self._viz.camera.set_pitch(0)
            self._viz.camera.set_roll(0)
            self._viz.camera.set_yaw(0)
        elif view == 'left':
            self._viz.camera.set_pitch(-90)
            self._viz.camera.set_roll(0)
            self._viz.camera.set_yaw(0)
        elif view == "forward":
            self._viz.camera.set_pitch(-90)
            self._viz.camera.set_roll(0)
            self._viz.camera.set_yaw(90)
        self._viz.update()

    def toggle_sky(self) -> None:
        """Toggle showing the color the sky in the pointcloud where there is no return"""
        with self._lock:
            for idx, sensor in enumerate(self._model._sensors):
                sensor._show_sky = not sensor._show_sky
                sensor.update_clouds(self._model._cloud_mode_name, self._frame_set[idx])
            self._viz.update()

    def _setup_controls(self) -> None:
        # key bindings. will be called from rendering thread, must be synchronized
        key_bindings: Dict[Tuple[int, int], Callable[[LidarFrameViz], None]] = {
            (ord('I'), 0): partial(LidarFrameViz.update_image_size, amount=+1),
            (ord('I'), 1): partial(LidarFrameViz.update_image_size, amount=-1),
            (ord('I'), 2): LidarFrameViz.cycle_img_view_mode,
            (ord('P'), 0): partial(LidarFrameViz.update_point_size, amount=+1),
            (ord('P'), 1): partial(LidarFrameViz.update_point_size, amount=-1),
            (ord('1'), 0): partial(LidarFrameViz.toggle_cloud, i=0),
            (ord('2'), 0): partial(LidarFrameViz.toggle_cloud, i=1),
            (ord('B'), 0): partial(LidarFrameViz.cycle_img_mode, i=0, direction=+1),
            (ord('B'), 1): partial(LidarFrameViz.cycle_img_mode, i=0, direction=-1),
            (ord('N'), 0): partial(LidarFrameViz.cycle_img_mode, i=1, direction=+1),
            (ord('N'), 1): partial(LidarFrameViz.cycle_img_mode, i=1, direction=-1),
            (ord('M'), 0): partial(LidarFrameViz.cycle_cloud_mode, direction=+1),
            (ord('M'), 1): partial(LidarFrameViz.cycle_cloud_mode, direction=-1),
            (ord('F'), 0): partial(LidarFrameViz.cycle_cloud_palette, direction=+1),
            (ord('F'), 1): partial(LidarFrameViz.cycle_cloud_palette, direction=-1),
            (ord("'"), 0): partial(LidarFrameViz.update_ring_size, amount=+1),
            (ord("'"), 1): partial(LidarFrameViz.update_ring_size, amount=-1),
            (ord("'"), 2): LidarFrameViz.cicle_ring_line_width,
            (ord("O"), 0): LidarFrameViz.toggle_osd,
            (ord('U'), 0): partial(LidarFrameViz.cycle_camera_mode, direction=+1),
            (ord('U'), 1): partial(LidarFrameViz.cycle_camera_mode, direction=-1),
            (ord("O"), 2): LidarFrameViz.cycle_osd_scale,
            (ord('C'), 0): partial(LidarFrameViz.update_flags_mode, direction=+1),
            (ord('C'), 1): partial(LidarFrameViz.update_flags_mode, direction=-1),
            (ord('C'), 2): LidarFrameViz.cycle_flags_palette,
            (ord('C'), 3): LidarFrameViz.cycle_object_color_mode,
            (ord('5'), 0): LidarFrameViz.cycle_object_view_mode,
            (ord('8'), 0): LidarFrameViz.toggle_track_view_mode,
            (ord('9'), 0): LidarFrameViz.toggle_axis_markers,
            (ord('/'), 1): LidarFrameViz.toggle_help,
            (ord('Y'), 0): LidarFrameViz.cycle_zone_selection_mode,
            (ord('Y'), 2): LidarFrameViz.cycle_zone_render_mode,
            (ord('1'), 1): partial(LidarFrameViz.set_view, view='top'),
            (ord('2'), 1): partial(LidarFrameViz.set_view, view='forward'),
            (ord('3'), 1): partial(LidarFrameViz.set_view, view='left'),
            (ord('S'), 2): LidarFrameViz.toggle_sky,
        }

        self._key_definitions: Dict[str, str] = {
            'w': "Camera pitch down",
            's': "Camera pitch up",
            'a': "Camera yaw right",
            'd': "Camera yaw left",
            'q': "Camera roll left",
            'e': "Camera roll right",
            "i / SHIFT+i": "Increase/decrease size of displayed 2D images",
            "p / SHIFT+p": "Increase/decrease point size",
            "SHIFT+r": "Reset camera orientation",
            "SHIFT+1": "Top-down view",
            "SHIFT+2": "Front-facing view",
            "SHIFT+3": "Left-facing view",
            "CTRL+r": "Camera bird-eye view",
            "0": "Toggle orthographic camera",
            "CTRL+- / CTRL+=": "Increase/decrease field of view (perspective only)",
            "1": "Toggle first return point cloud",
            "2": "Toggle second return point cloud",
            "b": "Cycle top 2D image",
            "n": "Cycle bottom 2D image",
            'm': "Cycle through point cloud coloring mode",
            'f': "Cycle through point cloud color palette",
            'c': "Cycle current highlight mode",
            'CTRL+c': "Cycle object palette",
            'CTRL+SHIFT+c': "Cycle object color attribute",
            'y': "Cycle displayed zone set.",
            'CTRL+y': "Cycle zone render mode.",
            'u': "Cycle camera mode",
            "5": "Cycle object display mode",
            "8": "Toggle track view mode",
            "9": "Toggle axis helpers at frame origin",
            '?': "Print keys to standard out",
            "= / -": "Dolly in and out",
            "\" / SHIFT+\"": "Increase/decrease spacing in range markers",
            "CTRL+\"": "Cycle through thickness of range markers or hide",
            "CTRL+i": "Flip or hide 2D images",
            "CTRL+o": "Cycle on-screen display scale",
            "CTRL+s": "Toggle sky visualization",
            'SHIFT': "Camera Translation with mouse drag",
            'ESC': "Exit the application",
        }

        self._setup_sensor_toggle_keys(self._model._sensors, key_bindings)

        def handle_keys(weak_self, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            if (key, mods) in key_bindings:
                local_self = weak_self()
                key_bindings[key, mods](local_self)
                local_self.draw(update=local_self.update_on_input)
            return True

        if isinstance(self._viz, PointViz):
            self._viz.push_key_handler(partial(handle_keys, weakref.ref(self)))
            self._viz.push_mouse_button_handler(partial(_mouse_button_handler, weakref.ref(self)), MouseEventType.RIGHT)
            self._viz.push_mouse_pos_handler(partial(_mouse_pos_handler, weakref.ref(self)), MouseEventType.RIGHT)
            self._viz.push_frame_buffer_resize_handler(partial(_frame_buffer_resize_handler, weakref.ref(self)))
            # push default controls last so they are on "top" and take input priority
            # this avoids unnecessary thread contention for input only they handle
            if self._add_default_controls:
                add_default_controls(self._viz)
            self._viz.notifications_enabled = True

        print("Press \'?\' while viz window is focused to print key bindings")

    MAX_SENSOR_TOGGLE_KEYS = 9

    def _setup_sensor_toggle_keys(self, sensors: List[SensorModel],
        key_bindings: Dict[Tuple[int, int],
        Callable[['LidarFrameViz'], None]]) -> None:

        for sensor_index in range(min(len(self._model._sensors), LidarFrameViz.MAX_SENSOR_TOGGLE_KEYS)):
            key_bindings[(ord(str(sensor_index + 1)), 2)] = \
                partial(LidarFrameViz.toggle_sensor, sensor_index=sensor_index)
            self._key_definitions[f"CTRL+{str(sensor_index + 1)}"] = f"Toggle sensor {sensor_index + 1} point cloud(s)"

    def select_img_mode(self, i: int, mode: str) -> bool:
        """Change the displayed field of the i'th image."""
        with self._lock:
            res = self._model.select_image_mode(i, mode)
            self._model.update_aoi()
            self._model.update_aoi_label(self._frame_set)

            # Note, updating the image mode needs the current frame
            # because ImageMode.enabled requires it.
            if len(self._frame_set) == len(self._model._sensors):
                self._model.update(self._frame_set, False)
            return res

    def cycle_img_mode(self, i: int, *, direction: int = 1) -> None:
        """Change the displayed field of the i'th image."""
        with self._lock:
            self._model.cycle_image_mode(i, direction)
            self._model.update_aoi()
            self._model.update_aoi_label(self._frame_set)

            # Note, updating the image mode needs the current frame
            # because ImageMode.enabled requires it.
            if len(self._frame_set) == len(self._model._sensors):
                self._model.update(self._frame_set, False)

    def select_cloud_mode(self, mode: str) -> bool:
        """Change the coloring mode of the 3D point cloud."""
        with self._lock:
            res = self._model.select_cloud_mode(mode)

            # Note, updating the cloud mode needs the current frame
            # because CloudMode.enabled requires it.
            if len(self._frame_set) == len(self._model._sensors):
                self._model.update(self._frame_set, False)
            return res

    def cycle_cloud_mode(self, direction: int = 1) -> None:
        """Change the coloring mode of the 3D point cloud."""
        with self._lock:
            self._model.cycle_cloud_mode(direction)

            self._model.update_aoi()

            # Note, updating the cloud mode needs the current frame
            # because CloudMode.enabled requires it.
            if len(self._frame_set) == len(self._model._sensors):
                self._model.update(self._frame_set, False)
            self._viz.set_notification(f"Cloud mode: {self._model._cloud_mode_name.replace('_', ' ')}")

    def cycle_cloud_palette(self, *, direction: int = 1) -> None:
        """Change the color palette of the 3D point cloud."""
        with self._lock:
            # move to LidarFrameVizModel
            self._model._palettes.cycle_cloud_palette(direction)
            self._model.update_cloud_palettes()
            self._viz.set_notification(f"Cloud palette: {self._model._palettes.get_current_palette().name}")

    def toggle_cloud(self, i: int) -> None:
        """Toggle whether the i'th return is displayed."""
        with self._lock:
            if i >= len(self._model._cloud_enabled):
                return
            if self._model._cloud_enabled[i]:
                self._model._cloud_enabled[i] = False
                for sensor in self._model._sensors:
                    # TODO[tws] encapsulate
                    if i < len(sensor._clouds):
                        self._viz.remove(sensor._clouds[i])
            else:
                self._model._cloud_enabled[i] = True
                for sensor in self._model._sensors:
                    # TODO[tws] encapsulate
                    if i < len(sensor._clouds) and sensor._enabled:
                        self._viz.add(sensor._clouds[i])

    def toggle_sensor(self, sensor_index: int) -> None:
        """Toggle whether the i'th sensor data is displayed."""
        with self._lock:
            sensor = self._model._sensors[sensor_index]
            sensor._enabled = not sensor._enabled
            if sensor._enabled:
                for i, cld in enumerate(sensor._clouds):
                    if i < len(self._model._cloud_enabled) and self._model._cloud_enabled[i]:
                        self._viz.add(cld)
                for img in sensor._images:
                    self._viz.add(img)
                self._viz.add(sensor._object_overlay)
                self._frame_axis[sensor_index].enable()
            else:
                for cld in sensor._clouds:
                    self._viz.remove(cld)
                for img in sensor._images:
                    self._viz.remove(img)
                self._viz.remove(sensor._object_overlay)
                self._frame_axis[sensor_index].disable()
            sensor.update_zones()

            #  clear the selection if we're hiding the sensor
            if any([selection._sensor_index == sensor_index for selection in self._model._current_selection]):
                self._model.clear_aoi()

            if self._frames_accum:
                self._frames_accum.toggle_sensor(sensor_index, sensor._enabled)

        self.update_image_size(0)

    def update_point_size(self, amount: int) -> None:
        """Change the point size of the 3D cloud."""
        with self._lock:
            # TODO refactor add to model
            self._model._cloud_pt_size = min(10.0,
                                      max(1.0, self._model._cloud_pt_size + amount))
            self._model._set_cloud_pt_size(self._model._cloud_pt_size)
            self._viz.set_notification(f"Point size: {int(self._model._cloud_pt_size)}")

    def update_image_size(self, amount: int) -> None:
        """Change the size of the 2D image and position image labels."""
        with self._lock:
            self._model.update_image_size(amount)

    def toggle_flip_images(self) -> None:
        """Toggle if 2D images should be flipped or not."""
        with self._lock:
            self._model.toggle_flip_images()

    def cycle_img_view_mode(self) -> None:
        with self._lock:
            modes = list(LidarFrameViz.ImageViewMode)
            current_index = modes.index(self._image_view_mode)
            next_index = (current_index + 1) % len(modes)
            self._image_view_mode = modes[next_index]
            self._viz.set_notification(
                f"Image view mode: {self._image_view_mode.name.replace('_', ' ')}")

            if self._image_view_mode == LidarFrameViz.ImageViewMode.ALL:
                self._model.flip_images(False)
                self._model.show_one_image(False)
            elif self._image_view_mode == LidarFrameViz.ImageViewMode.ALL_FLIPPED:
                self._model.flip_images(True)
                self._model.show_one_image(False)
            elif self._image_view_mode == LidarFrameViz.ImageViewMode.ONE:
                self._model.flip_images(False)
                self._model.show_one_image(True)
            elif self._image_view_mode == LidarFrameViz.ImageViewMode.ONE_FLIPPED:
                self._model.flip_images(True)
                self._model.show_one_image(True)

    def update_ring_size(self, amount: int) -> None:
        """Change distance ring size."""
        with self._lock:
            self._ring_size = min(3, max(-2, self._ring_size + amount))
            self._viz.target_display.set_ring_size(self._ring_size)
            self._viz.set_notification(
                f"Ring size: {self._viz.target_display.get_ring_size_m()} meters")

    def cicle_ring_line_width(self) -> None:
        """Change rings line width."""
        with self._lock:
            self._ring_line_width = max(0, (self._ring_line_width + 1) % 10)
            self._viz.target_display.set_ring_line_width(self._ring_line_width)
            self._viz.target_display.enable_rings(self._ring_line_width != 0)
            self._viz.set_notification(f"Ring width: {self._ring_line_width} pixels")

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._lock:
            if self._osd_state != LidarFrameViz.OsdState.DEFAULT:
                self._osd_state = LidarFrameViz.OsdState.DEFAULT
            else:
                self._osd_state = LidarFrameViz.OsdState.NONE

    def cycle_camera_mode(self, direction: int = 1) -> None:
        """Toggle the camera follow mode."""
        with self._lock:
            modes = list(LidarFrameViz.CameraMode)
            current_index = modes.index(self._camera_mode)
            next_index = (current_index + direction) % len(modes)
            self._camera_mode = modes[next_index]
            self._camera_pose_history.clear()
            self._interpolant = 1.0
            self._viz.set_notification(
                f"Camera mode: {self._camera_mode.name.replace('_', ' ')}")

    # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
    def update_flags_mode(self,
                          mode: 'Optional[LidarFrameViz.FlagsMode]' = None, direction: int = 1) -> None:
        with self._lock:
            # cycle between flag mode enum values
            if mode is None:
                self._flags_mode = LidarFrameViz.FlagsMode(
                    (self._flags_mode.value + direction) %
                    len(LidarFrameViz.FlagsMode.__members__))
            else:
                self._flags_mode = mode

            self._viz.set_notification(
                f"Flags mode: {self._flags_mode.name.replace('_', ' ')}")
            if self._flags_mode != LidarFrameViz.FlagsMode.NONE:
                self._model.clear_aoi()

            # if no frame is set yet, skip updating the cloud masks
            if not self._frame_set:
                return

            # reset the cloud range field (since HIDE_BLOOM will modify it)
            for frame, sensor in zip(self._frame_set, self._model._sensors):
                if not frame:
                    continue
                for i, range_field in ((0, ChanField.RANGE),
                                       (1, ChanField.RANGE2)):
                    if range_field in frame.fields:
                        sensor._clouds[i].set_range(frame.field(range_field))

            # TODO[UN]: need to be done as part of the combined lidar mode
            # set mask on all points in the second cloud, clear other mask
            for i, sensor in enumerate(self._model._sensors):
                if self._flags_mode == LidarFrameViz.FlagsMode.HIGHLIGHT_SECOND:
                    sensor._cloud_masks[0][:, :, 3] = 0
                    sensor._cloud_masks[1][:, :, 3] = 255
                else:
                    sensor._cloud_masks[0][:, :, 3] = 0
                    sensor._cloud_masks[1][:, :, 3] = 0
                sensor._clouds[0].set_mask(sensor._cloud_masks[0])
                sensor._clouds[1].set_mask(sensor._cloud_masks[1])

            # update masks based on AOI if one is set
            if self._flags_mode == LidarFrameViz.FlagsMode.NONE:
                self._model.clear_masks()
                self._model.update_aoi()

    def _draw_update_flags_mode(self) -> None:
        """Apply selected FlagsMode to the cloud"""
        # set a cloud mask based where first flags bit is set
        for frame, sensor in zip(self._frame_set, self._model._sensors):
            if not frame:
                continue
            if self._flags_mode == LidarFrameViz.FlagsMode.HIGHLIGHT_BLOOM:
                for i, flag_field in ((0, ChanField.FLAGS), (1, ChanField.FLAGS2)):
                    if flag_field in frame.fields:
                        mask_opacity = (frame.field(flag_field) & 0x1) * 255
                        sensor._cloud_masks[i][:, :, 3] = mask_opacity
                        sensor._clouds[i].set_mask(sensor._cloud_masks[i])
            # set range to zero where first flags bit is set
            elif self._flags_mode == LidarFrameViz.FlagsMode.HIDE_BLOOM:
                for i, flag_field, range_field in ((0, ChanField.FLAGS,
                                                    ChanField.RANGE),
                                                   (1, ChanField.FLAGS2,
                                                    ChanField.RANGE2)):
                    if flag_field in frame.fields and range_field in frame.fields:
                        # modifying the frame in-place would break cycling modes while paused
                        rng = frame.field(range_field).copy()
                        rng[frame.field(flag_field) & 0x1 == 0x1] = 0
                        sensor._clouds[i].set_range(rng)
            # zone masks need to be updated every frame because they depend on the frame data (including which zones are
            # live or triggered.)
            elif self._flags_mode == LidarFrameViz.FlagsMode.HIGHLIGHT_ZONES:
                sensor.update_zone_occupancy_cloud_and_image_masks(frame, self._model._palettes)
                sensor._clouds[0].set_mask(sensor._cloud_masks[0])
            elif self._flags_mode == LidarFrameViz.FlagsMode.HIGHLIGHT_OBJECTS:
                sensor.update_object_id_cloud_and_image_masks(frame, self._model._flags_palette)
                sensor._clouds[0].set_mask(sensor._cloud_masks[0])

    def cycle_flags_palette(self) -> None:
        """Cycle the palette used for flags modes."""
        with self._lock:
            self._flags_palette = self.FlagsPalette(
                (self._flags_palette.value + 1) %
                len(self.FlagsPalette.__members__))
            # TODO create a mapping
            palette = distinct_palette \
                if self._flags_palette == self.FlagsPalette.DISTINCT \
                else discrete_rainbow_palette
            self._model.update_overlay_colors(palette)
            self._viz.set_notification(
                f"Flags palette: {self._flags_palette.name.replace('_', ' ')}")

    def cycle_object_color_mode(self) -> None:
        """Cycle the mode used for object highlighting."""
        with self._lock:
            self._model._object_color_mode = ObjectColorMode(
                (self._model._object_color_mode.value + 1) %
                len(ObjectColorMode.__members__))
            palette = distinct_palette \
                if self._flags_palette == self.FlagsPalette.DISTINCT \
                else discrete_rainbow_palette
            self._model.update_overlay_colors(palette)
            self._viz.set_notification(
                f"Object color mode: {self._model._object_color_mode.name.replace('_', ' ')}")

    def cycle_object_view_mode(self) -> None:
        with self._lock:
            self._model._object_view_mode = ObjectViewMode(
                (self._model._object_view_mode.value + 1) %
                len(ObjectViewMode.__members__))
            self._model.update_objects(self._frame_set)
            self._viz.set_notification(
                f"Object visibility: {self._model._object_view_mode.name.replace('_', ' ')}")

    def toggle_track_view_mode(self) -> None:
        """Cycle through the track view modes"""
        with self._lock:
            if self._track_view_mode == LidarFrameViz.TrackViewMode.TRACK_PLUS_PER_FRAME_TRAJECTORY:
                if self._frames_accum:
                    self._frames_accum.track_accumulator.toggle_visibility(False)
                self._track_view_mode = LidarFrameViz.TrackViewMode.OFF
                for trajectory in self._per_frame_pose_trajectory:
                    self._viz.remove(trajectory)
                for axis in self._per_frame_pose_trajectory_axis:
                    axis.disable()
            elif self._track_view_mode == LidarFrameViz.TrackViewMode.OFF:
                self._track_view_mode = LidarFrameViz.TrackViewMode.TRACK_ONLY
                if self._frames_accum:
                    self._frames_accum.track_accumulator.toggle_visibility(True)
            elif self._track_view_mode == LidarFrameViz.TrackViewMode.TRACK_ONLY:
                self._track_view_mode = LidarFrameViz.TrackViewMode.TRACK_PLUS_PER_FRAME_TRAJECTORY
                for trajectory in self._per_frame_pose_trajectory:
                    self._viz.add(trajectory)
                for axis in self._per_frame_pose_trajectory_axis:
                    axis.enable()
            self._viz.set_notification(f"Track: {self._track_view_mode.name.replace('_', ' ')}")

    def toggle_axis_markers(self) -> None:
        """Toggle the helper axis of a frame ON/OFF"""
        with self._lock:
            if self._frame_axis_enabled:
                self._frame_axis_enabled = False
                self._map_origin_axis.disable()
                self._base_link_axis.disable()
                for axis in self._frame_axis:
                    axis.disable()
            else:
                self._frame_axis_enabled = True
                self._map_origin_axis.enable()
                self._base_link_axis.enable()
                for axis, sensor in zip(self._frame_axis, self._model._sensors):
                    if sensor._enabled:
                        axis.enable()
            self._viz.set_notification(
                f"Axis markers: {'ON' if self._frame_axis_enabled else 'OFF'}")

    @property
    def frame(self) -> FrameSet:
        """The currently displayed frame."""
        return self._frame_set

    @property
    def frame_num(self) -> int:
        """The currently displayed frame number"""
        return self._frame_num

    def update(self,
               frames: FrameSet,
               frame_num: Optional[int] = None,
               last_n_frame_sets: List[FrameSet] = []) -> None:
        """Update the LidarFrameViz state with the provided frames."""
        with self._lock:
            self._frame_set = frames
            if frame_num is not None:
                self._frame_num = frame_num
            else:
                self._frame_num += 1

            self._model.update(self._frame_set)
            if len(last_n_frame_sets) > 0:
                self.imu_plot(last_n_frame_sets, self._imu_viz_config)
            if self._frames_accum:
                self._frames_accum.update(frames, self._frame_num)

            self._update_fine_frame_trajectory()

            self._model.update_aoi_label(self._frame_set)
            frame = self._frame_set[self._tracked_sensor]
            if frame is not None:
                self._frame_pose_history.append(core.last_valid_column_pose(frame))
            self._interpolant = 1.0

    def draw(self, update: bool = True, update_camera: bool = True) -> None:
        """Process and draw the latest state to the screen."""
        with self._lock:
            self._draw(update_camera=update_camera)
            if self._frames_accum:
                self._frames_accum._draw()

        if not self._image_size_initialized:
            self.update_image_size(0)
            self._image_size_initialized = True

        if update:
            self._viz.update()

    def run(self) -> None:
        """Run the rendering loop of the visualizer.

        See :py:meth:`.PointViz.run`
        """
        self._viz.run()

    # i/o and processing, called from client thread
    # usually need to synchronize with key handlers, which run in render thread
    def _draw(self, update_camera: bool = True) -> None:
        if update_camera:
            self._draw_update_camera_pose(update_camera_history=True)
        else:
            self._draw_update_scene_poses()
        self._draw_update_flags_mode()
        self._update_multi_viz_osd()

    @staticmethod
    def _format_version(version: core.Version) -> str:
        result = f'v{version.major}.{version.minor}.{version.patch}'
        if version.prerelease:
            result += f'-{version.prerelease}'
        return result

    def show_all_imu_plots(self):
        [s._imu_plot.show() for s in self._model._sensors]

    def hide_all_imu_plots(self):
        [s._imu_plot.hide() for s in self._model._sensors]

    def show_all_sensor_labels(self):
        [s._sensor_label.show() for s in self._model._sensors]

    def hide_all_sensor_labels(self):
        [s._sensor_label.hide() for s in self._model._sensors]

    def text_scale(self):
        return self._osd_scale / 2 + 0.5

    def _update_multi_viz_osd(self):
        text_scale = self.text_scale()
        self._osd.set_scale(text_scale)

        # handle OSD modes "NONE" and "HELP"
        if self._osd_state == LidarFrameViz.OsdState.NONE:
            self.hide_all_sensor_labels()
            self.hide_all_imu_plots()
            self._osd.set_text("")
            return
        elif self._osd_state == LidarFrameViz.OsdState.HELP:
            self.hide_all_sensor_labels()
            self.hide_all_imu_plots()
            on_screen_help_text = []
            for key_binding in self._key_definitions:
                on_screen_help_text.append(f"{key_binding:^7}: {self._key_definitions[key_binding]}")
            self._osd.set_text('\n'.join(on_screen_help_text))
            return

        self.show_all_sensor_labels()
        self.show_all_imu_plots()

        # create the OSD text
        osd_str = ""
        cloud_idxs_str = str([i + 1 for i in range(len(self._model._cloud_enabled))])
        cloud_states_str = ", ".join(
            ["ON" if e else "OFF" for e in self._model._cloud_enabled])

        img_keys = 'B, N'
        cld_keys = 'M'
        img_modes = self._model._image_mode_names[0]
        img_modes += ", " + self._model._image_mode_names[1]
        cld_modes = self._model._cloud_mode_name
        osd_str += f"image [{img_keys}]: {img_modes}\n" \
            f"cloud {cloud_idxs_str}: {cloud_states_str}\n" \
            f"        cloud mode [{cld_keys}]: {cld_modes}\n" \
            f"        palette [F]: {self._model._cloud_palette_name}\n" \

        osd_str_extra = self._osd_text_extra()
        if osd_str_extra:
            osd_str_extra += "\n"

        valid_frames = [s for s in self._frame_set if s]
        if not valid_frames:
            return  # Skip update if no frames available yet

        first_frame_ts = _get_timestamp(self._frame_set) * 1e-9
        if self._first_frame_ts is None:
            self._first_frame_ts = first_frame_ts
        frame_ts = first_frame_ts - self._first_frame_ts    # show relative time
        first_frame_datetime = datetime.fromtimestamp(first_frame_ts, tz=timezone.utc)
        osd_str += f"{osd_str_extra}" \
            f"time: {first_frame_datetime.strftime('%Y-%m-%d %H:%M:%S.%f')} UTC\n" \
            f"frame # : {self._frame_num}, frame ts: {frame_ts:0.3f} s"
        self._osd.set_text(osd_str)

        view_h = self._viz.viewport_height
        img_height = self._imu_viz_config.imu_plot_height_pixels / view_h * text_scale
        img_width = self._imu_viz_config.imu_plot_width_pixels / view_h * text_scale
        label_height_accum = self._osd.text_height / view_h
        img_height_accum = 0

        # add the individual sensor labels and imu images from bottom to top
        # one at a time positioned based on the heights of the previously added items
        # so use a reverse index to index into the sensor models
        for idx in range(len(self._model._sensors)):
            rev_idx = len(self._model._sensors) - idx - 1
            sensor = self._model._sensors[rev_idx]
            img_bottom = -(1 - label_height_accum) * 2 + 1 + (img_height_accum * 2)
            if sensor._has_imu_data:
                sensor._imu_plot.show()
                img_height_accum += img_height
            else:
                sensor._imu_plot.hide()
            label_bottom = 1 - label_height_accum - img_height_accum
            sensor._sensor_label.position = (0.0, label_bottom)
            sensor._sensor_label.scale = text_scale

            img_left = 75 * text_scale / view_h
            img_top = img_bottom + img_height * 2
            sensor._imu_plot.hshift = -1
            sensor._imu_plot.position = (img_left, img_width - img_left, img_bottom, img_top)
            meta = sensor._meta
            cloud_state = "ON" if sensor._enabled else "OFF"
            enabled_clouds_str = f"sensor [CTRL+{rev_idx + 1}]: {cloud_state}"
            frame = self._frame_set[rev_idx]
            if frame:
                frame_ts = _valid_or_zero_ts(frame) * 1e-9 - self._first_frame_ts    # show relative time
                enabled_clouds_str += f"   frame id: {frame.frame_id}   ts: {frame_ts:0.2f}\n"
            else:
                enabled_clouds_str += "\n"
            profile_str = str(meta.format.udp_profile_lidar).replace('_', '..')
            version_str = LidarFrameViz._format_version(meta.get_version())
            enabled_clouds_str += f"        {profile_str}   {meta.config.lidar_mode}"
            enabled_clouds_str += f"\n        {meta.prod_line}   {meta.sn}   {version_str}"
            default_color = (1.0, 1.0, 1.0, 1.0)
            if frame is not None:
                if frame.shot_limiting() != core.ShotLimitingStatus.NORMAL:
                    enabled_clouds_str += f"\n        WARNING: Shot limiting status: {frame.shot_limiting()}"
                    default_color = (1.0, 0.6, 0.0, 1.0)
                if frame.thermal_shutdown() != core.ThermalShutdownStatus.NORMAL:
                    enabled_clouds_str += f"\n        WARNING: Thermal shutdown status: {frame.thermal_shutdown()}"
                    default_color = (1.0, 0.0, 0.0, 1.0)
            sensor._sensor_label.text = enabled_clouds_str
            if self._model._cloud_mode_name == "SENSOR":
                color = sensor._color
                sensor._sensor_label.rgba = (color[0] / 255, color[1] / 255, color[2] / 255, 1)
            else:
                sensor._sensor_label.rgba = default_color
            label_height_accum += sensor._sensor_label.text_height / view_h

    def _get_pose_with_min_ts(self) -> np.ndarray:
        first_frame = min([s for s in self._frame_set if s], key=lambda s: _valid_or_zero_ts(s))
        return core.last_valid_column_pose(first_frame)

    def _interpolate_camera_pose(self):
        if not self._frame_pose_history:
            if not self._frame_set:
                return np.eye(4)
            frame = self._frame_set[self._tracked_sensor]
            if frame is not None:
                return core.last_valid_column_pose(frame)
            return self._get_pose_with_min_ts()
        n = len(self._frame_pose_history)
        if n == 1:
            return self._frame_pose_history[0]

        p1 = self._frame_pose_history[-2]
        p2 = self._frame_pose_history[-1]
        t = self._interpolant
        if t >= 1.0:
            return p2

        return core.interp_pose(np.array([t]), np.array([0.0, 1.0]),
                                np.stack([p1, p2]))[0]

    def _apply_scene_poses(self) -> None:
        if not self._frame_set:
            return

        # frame with the minimal timestamp determines the
        # center of the system (by it's frame pose)
        pose = self._get_pose_with_min_ts()
        self._base_link_axis.pose = pose
        for axis, sensor in zip(self._frame_axis, self._model._sensors):
            axis.pose = pose @ sensor._meta.sensor_to_body

    def _draw_update_scene_poses(self) -> None:
        """Update axis poses from the current frames."""
        with self._lock:
            self._apply_scene_poses()

    def _draw_update_camera_pose(self, update_camera_history: bool = True) -> None:
        """Apply poses from the frames to the scene"""
        with self._lock:
            if not self._frame_set:
                return

            self._apply_scene_poses()
            frame_pose = self._interpolate_camera_pose()
            if update_camera_history:
                self._camera_pose_history.append(frame_pose)
            if self._camera_mode == LidarFrameViz.CameraMode.FOLLOW:
                self._camera_pose = frame_pose
            elif self._camera_mode == LidarFrameViz.CameraMode.FOLLOW_ROTATION_LOCKED:
                # only change the position and keep the rotation part
                self._camera_pose[:3, 3] = frame_pose[:3, 3]
            elif self._camera_mode == LidarFrameViz.CameraMode.FIXED:
                pass
            elif self._camera_mode == LidarFrameViz.CameraMode.FOLLOW_SMOOTH:
                avg_t = np.mean([p[:3, 3] for p in self._camera_pose_history], axis=0)
                R_sum = sum(p[:3, :3] for p in self._camera_pose_history)
                U, _, Vt = np.linalg.svd(R_sum)
                self._camera_pose = np.eye(4)
                self._camera_pose[:3, :3] = U @ Vt
                self._camera_pose[:3, 3] = avg_t
            elif self._camera_mode == LidarFrameViz.CameraMode.FOLLOW_SMOOTH_ROTATION_LOCK:
                avg_t = np.mean([p[:3, 3] for p in self._camera_pose_history], axis=0)
                self._camera_pose[:3, 3] = avg_t

            self._viz.camera.set_target(np.linalg.inv(self._camera_pose))

    def clear_trajectory(self) -> None:
        for cloud in self._per_frame_pose_trajectory:
            self._viz.remove(cloud)
        self._per_frame_pose_trajectory.clear()

        for axis in self._per_frame_pose_trajectory_axis:
            axis.disable()
        self._per_frame_pose_trajectory_axis.clear()

        if self._frames_accum is not None:
            self._frames_accum.clear_track()

    def _update_fine_frame_trajectory(self) -> None:
        """Draw markers for each pose in the frame."""

        frame = self._frame_set[self._tracked_sensor]
        if frame is None:
            return

        valid_poses = (np.bitwise_and(frame.status, 1) == 1)
        xyz = frame.body_to_world[:, :3, -1].astype(np.float32)
        xyz = xyz[valid_poses]
        rgb = np.ones_like(xyz)
        cloud = Cloud(xyz.shape[0])
        cloud.set_xyz(np.ascontiguousarray(xyz))
        cloud.set_key(rgb)
        cloud.set_point_size(2.0)

        def deque_saturated(deq: Deque) -> bool:
            return deq.maxlen is not None and len(deq) == deq.maxlen

        # before adding a new pose trajectory, remove the oldest one if deque is saturated
        if deque_saturated(self._per_frame_pose_trajectory):
            oldest = self._per_frame_pose_trajectory[0]
            if self._track_view_mode == LidarFrameViz.TrackViewMode.TRACK_PLUS_PER_FRAME_TRAJECTORY:
                self._viz.remove(oldest)
        self._per_frame_pose_trajectory.append(cloud)
        if self._track_view_mode == LidarFrameViz.TrackViewMode.TRACK_PLUS_PER_FRAME_TRAJECTORY:
            self._viz.add(cloud)

        step = max(1, frame.w // 4)
        poses = frame.body_to_world[0:frame.w:step]
        last_pose = frame.body_to_world[frame.w - 1]
        poses = np.concatenate([poses, last_pose[None, ...]], axis=0)
        for pose in poses:
            new_axis = vizu.AxisWithLabel(
                self._viz,
                pose=pose,
                length=0.01,
                thickness=2,
                axis_n=15,
                label="",
                enabled=self._track_view_mode == LidarFrameViz.TrackViewMode.TRACK_PLUS_PER_FRAME_TRAJECTORY
            )
            # before adding a new axis, remove the oldest one if deque is saturated
            if deque_saturated(self._per_frame_pose_trajectory_axis):
                oldest_axis = self._per_frame_pose_trajectory_axis[0]
                oldest_axis.disable()
            self._per_frame_pose_trajectory_axis.append(new_axis)

    def print_key_bindings(self) -> None:
        print(">---------------- Key Bindings --------------<")
        for key_binding in self._key_definitions:
            print(f"{key_binding:^7}: {self._key_definitions[key_binding]}")
        print(">--------------------------------------------<")

    def toggle_help(self) -> None:
        with self._lock:
            if self._osd_state != LidarFrameViz.OsdState.HELP:
                self._previous_osd_state = self._osd_state
                self._osd_state = LidarFrameViz.OsdState.HELP
                self.print_key_bindings()
            else:
                self._osd_state = self._previous_osd_state

    def set_zone_selection_and_render_modes(self, sel_mode: ZoneSelectionMode, render_mode: ZoneRenderMode) -> None:
        for sensor in self._model._sensors:
            sensor.set_zone_selection_and_render_mode(sel_mode, render_mode)

    def cycle_zone_selection_mode(self) -> None:
        with self._lock:
            self._zone_selection_mode = ZoneSelectionMode(
                (self._zone_selection_mode.value + 1) %
                len(ZoneSelectionMode.__members__))
            self._viz.set_notification(
                f"Zone Selection Mode: {self._zone_selection_mode.name.replace('_', ' ')}")
            self.set_zone_selection_and_render_modes(self._zone_selection_mode, self._zone_render_mode)

    def cycle_zone_render_mode(self) -> None:
        with self._lock:
            self._zone_render_mode = ZoneRenderMode(
                (self._zone_render_mode.value + 1) %
                len(ZoneRenderMode.__members__))
            self._viz.set_notification(
                f"Zone Render Mode: {self._zone_render_mode.name.replace('_', ' ')}")
            self.set_zone_selection_and_render_modes(self._zone_selection_mode, self._zone_render_mode)


class _Seekable:
    """Wrap an iterable to support seeking by index.

    Similar to `more_itertools.seekable` but keeps indexes stable even values
    are evicted from the cache.

    The :meth:`seek` and :meth:`__next__` methods maintain the invariant:

        (read_ind - len(cache)) < next_ind <= read_ind + 1
    """

    # FIXME[tws] somehow simplify typing
    def __init__(self, it: Iterable[FrameSet], maxlen=50) -> None:
        self._next_ind = 0  # index of next value to be returned
        self._read_ind = -1  # index of most recent (leftmost) value in cache
        self._iterable = it
        self._it: Iterator[FrameSet] = \
            cast(Iterator[FrameSet], iter(it))
        self._cache: Deque[FrameSet] = deque([], maxlen)

    def __iter__(self):
        return self

    def __next__(self) -> FrameSet:
        # next value already read, is in cache
        t: FrameSet
        if self._next_ind <= self._read_ind:
            t = self._cache[self._read_ind - self._next_ind]
            self._next_ind += 1
            return t
        # next value comes from iterator
        elif self._next_ind == self._read_ind + 1:
            t = next(self._it)
            # detect a loop signaled by empty collation
            if len(t) == 0:
                self._next_ind = 0
                self._read_ind = -1
                return FrameSet()
            self._cache.appendleft(t)
            self._next_ind += 1
            self._read_ind += 1

            return t
        else:
            raise AssertionError("Violated: next_ind <= read_ind + 1")

    def last_n(self, n: int) -> List[FrameSet]:
        """Returns up to the last n frames."""
        # TODO[tws] annoyingly, the result depends on whether this is called before or after next()
        start = self._read_ind - self._next_ind + 1
        stop = start + n
        res = list(itertools.islice(
            self._cache,
            start, stop
        ))
        return res  # type: ignore

    @property
    def frame_num(self) -> int:
        """Returns the most-recently read frame number, starting at zero."""
        return self._next_ind

    @property
    def scan_num(self) -> int:
        """Deprecated: use frame_num."""
        warn_deprecated("scan_num is deprecated, use frame_num")
        return self.frame_num

    def seek(self, ind: int) -> bool:
        """Update iterator position to index `ind`.

        Args:
            ind: the desired index to be read on the subsequent call to
                 :meth:`__next__`

        Returns:
            True if seeking succeeded, False otherwise. Seeking may fail if the
            desired index has been evicted from the cache.

        Raises:
            StopIteration if seeking beyond the end of the iterator
        """

        # Disallow seeking before the first frame
        if ind < 0:
            return False

        # seek forward until ind is next to be read
        while ind > self._next_ind:
            next(self)

        # here ind <= _read_ind + 1. Left to check whether value is in the cache
        if ind > (self._read_ind - len(self._cache)):
            self._next_ind = ind
            return True
        else:
            # value not in cache, seek failed
            return False

    def close(self) -> None:
        """Close the underlying iterable, if supported."""
        if hasattr(self._iterable, 'close'):
            self._iterable.close()


class LiveConsumer:
    """
    Spawns a thread to consume frames as soon as they're available.
    This is a work-around to deal with situations where the visualizer is too slow to keep up with a live sensor.
    """

    def __init__(self, iterable, should_count_dropped_frame_method):
        self._stopped = threading.Event()
        self._queue = queue.Queue(1)
        self._iterable = iterable
        self._consumer_thread = threading.Thread(target=partial(self.__consume))
        self._dropped_frames = 0
        self._should_count_dropped_frame_method = should_count_dropped_frame_method

    def __consume(self):
        for frames in self._iterable:
            try:
                if self._stopped.is_set():
                    break
                self._queue.put_nowait(frames)
            except queue.Full:
                if self._should_count_dropped_frame_method():
                    self._dropped_frames += 1
        self._stopped.set()

    def __iter__(self):
        return self

    def __next__(self):
        while True:
            if self._stopped.is_set():
                raise StopIteration()
            try:
                return self._queue.get(timeout=1)
            except queue.Empty:
                pass

    def start(self):
        assert not self._stopped.is_set()
        self._consumer_thread.start()

    def shutdown(self):
        self._stopped.set()
        self._consumer_thread.join()

    @property
    def dropped_frames(self):
        return self._dropped_frames


class SimpleViz:
    """Visualize a stream of LidarFrames.

    Handles controls for playback speed, pausing and stepping."""

    _playback_rates: ClassVar[Tuple[float, ...]]
    _playback_rates = (0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 0.0)
    _images: List[Tuple[PILImage.Image, float]] = []

    def __init__(self,
                 metadata: Union[List[SensorInfo], SensorInfo],
                 *,
                 rate: Optional[float] = None,
                 pause_at: int = -1,
                 on_eof: str = 'exit',
                 accum_max_num: int = 0,
                 accum_min_dist_meters: float = 0,
                 accum_min_dist_num: int = 1,
                 map_enabled: bool = False,
                 map_select_ratio: float = MAP_SELECT_RATIO,
                 map_max_points: int = MAP_MAX_POINTS_NUM,
                 imu_viz_config: ImuVisualizationConfig = ImuVisualizationConfig(),
                 title: str = "Ouster Viz",
                 maximized: bool = False,
                 fullscreen: bool = False,
                 resolution: Optional[Tuple[int, int]] = None,
                 screenshot_resolution: Optional[Union[Tuple[int, int], float]] = None,
                 _override_pointviz: Optional[PointViz] = None,
                 _override_lidar_frame_viz: Optional[LidarFrameViz] = None,
                 _buflen: int = 50,
                 video_format: str = "png",
                 clear_on_loop: bool = False, subframes = 0) -> None:
        """
        Args:
            arg: Metadata associated with the frames to be visualized or a
                 LidarFrameViz instance to use.
            rate: Playback rate. One of 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0 or
                  None for "live" playback (the default).
            pause_at: frame number to pause at, default (-1) - no auto pause, to
                      stop after the very first frame use 0
            on_eof: What to do when the source ends. One of 'exit', 'stop' or 'loop'
            video_format: Format for saving continuous screenshots. One of 'png' or 'gif'
        Raises:
            ValueError: if the specified rate isn't one of the options
        """
        valid = ["exit", "stop", "loop"]
        if on_eof not in valid:
            raise ValueError(f"on_eof must be one of {valid}, was '{on_eof}'")

        self._metadata = [metadata] if isinstance(metadata, SensorInfo) else metadata
        if _override_pointviz and _override_lidar_frame_viz and _override_lidar_frame_viz._viz != _override_pointviz:
            raise ValueError('SimpleViz cannot use two different PointViz instances simultaneously.')
        if _override_lidar_frame_viz:
            self._viz = _override_lidar_frame_viz._viz
        else:
            if _override_pointviz:
                self._viz = _override_pointviz
            else:
                # maximized/fullscreen take priority; resolution is ignored when either is active
                if maximized or fullscreen or resolution is None:
                    self._viz = PointViz(title, maximized=maximized, fullscreen=fullscreen)
                else:
                    _window_w, _window_h = resolution
                    self._viz = PointViz(title, window_width=_window_w, window_height=_window_h,
                                         maximized=maximized, fullscreen=fullscreen)
        accum_config = LidarFrameVizAccumulatorsConfig(
            accum_max_num=accum_max_num,
            accum_min_dist_meters=accum_min_dist_meters,
            accum_min_dist_num=accum_min_dist_num,
            map_enabled=map_enabled,
            map_select_ratio=map_select_ratio,
            map_max_points=map_max_points
        )
        self._frame_viz = _override_lidar_frame_viz if _override_lidar_frame_viz \
            else LidarFrameViz(
                self._metadata,
                self._viz,
                accum_config,
                imu_viz_config,
                _buflen=_buflen,
                _add_default_controls=False
            )

        self._lock = threading.Lock()
        self._live = (rate is None)
        self._rate_ind = SimpleViz._playback_rates.index(rate or 0.0)
        if subframes < 0:
            raise ValueError(f"subframes must be non-negative, got {subframes}")
        self._subframes = subframes
        self._buflen = _buflen
        self._pause_at = pause_at
        self._on_eof = on_eof
        self._last_draw_period = 0.0
        self._video_format = video_format
        self._clear_on_loop = clear_on_loop

        # pausing and stepping
        self._cv = threading.Condition()
        self._paused = False
        self._step = 0
        self._proc_exit = False

        # playback status display
        # TODO[tws] probably move playback osd to LidarFrameViz...
        # We've had to define extra key handlers here to support it for no good reason.
        self._right_osd = Label("", 1, 1, align_right=True)
        self._viz.add(self._right_osd)

        def on_osd_scale(weak_self):
            local_self = weak_self()
            if local_self is not None:
                local_self._update_right_osd()
        self._frame_viz._on_osd_scale = partial(on_osd_scale, weakref.ref(self))

        # continuous screenshots recording
        self._viz_img_recording = False
        self._screenshot_resolution = screenshot_resolution

        self._update_right_osd()

        key_bindings: Dict[Tuple[int, int], Callable[[SimpleViz], None]] = {
            (ord(','), 0): partial(SimpleViz.seek_relative, n_frames=-1),
            (ord(','), 2): partial(SimpleViz.seek_relative, n_frames=-10),
            (ord('.'), 0): partial(SimpleViz.seek_relative, n_frames=1),
            (ord('.'), 2): partial(SimpleViz.seek_relative, n_frames=10),
            (ord(' '), 0): SimpleViz.toggle_pause,
            (ord('O'), 0): SimpleViz.toggle_osd,
            (ord('/'), 1): SimpleViz.toggle_help,
            (ord('X'), 1): SimpleViz.toggle_img_recording,
            (ord('Z'), 1): SimpleViz.screenshot,
            (ord('V'), 0): partial(SimpleViz.cycle_screenshot_res_factor, direction=1),
            (ord('V'), 1): partial(SimpleViz.cycle_screenshot_res_factor, direction=-1),
            (ord('X'), 2): SimpleViz.toggle_video_format,
            (ord('H'), 0): partial(SimpleViz.adjust_subframes, direction=1),
            (ord('H'), 1): partial(SimpleViz.adjust_subframes, direction=-1)
        }

        key_definitions: Dict[str, str] = {
            'o': "Toggle information overlay",
            'SHIFT+x': "Toggle a continuous saving of screenshots",
            'CTRL+x': "Toggle continuous screenshot format.",
            'SHIFT+z': "Take a screenshot",
            'v / SHIFT+v': "Cycle screenshot resolution factor",
            ". / ,": "Step forward one frame",
            "> / <": "Increase/decrease playback rate (during replay)",
            'SPACE': "Pause and unpause",
            'H / SHIFT+H': "Increase/decrease number of camera subframes",
        }
        if hasattr(self._frame_viz, "_key_definitions"):
            self._frame_viz._key_definitions.update(key_definitions)

        # only allow changing rate when not in "live" mode
        if not self._live:
            key_bindings.update({
                (ord(','), 1):
                partial(SimpleViz.modify_rate, amount=-1),
                (ord('.'), 1):
                partial(SimpleViz.modify_rate, amount=1),
            })

        def handle_keys(weak_self, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            local_self = weak_self()
            if (key, mods) in key_bindings:
                prop = key_bindings[key, mods](local_self)
                # override rather than add bindings on no prop
                return False if prop is None else prop
            return True
        self._viz.push_key_handler(partial(handle_keys, weakref.ref(self)))
        # push default controls last so they are on "top" and take input priority
        # this avoids unnecessary thread contention for input only they handle
        # this is a duplicate of what happens in LidarFrameViz, but its fine
        add_default_controls(self._viz)

    def toggle_help(self) -> None:
        self._frame_viz.toggle_help()
        self._update_right_osd()
        self._frame_viz.draw()

    def _update_right_osd(self) -> None:
        if self._frame_viz.osd_state != LidarFrameViz.OsdState.DEFAULT:
            self._right_osd.set_text("")
            return

        screenshot_str: str = ""
        if self._screenshot_resolution:
            if isinstance(self._screenshot_resolution, float):
                # screenshot resolution is a scale factor
                res_str = (
                    str(int(self._screenshot_resolution))
                    if self._screenshot_resolution.is_integer()
                    else str(self._screenshot_resolution)
                )
                screenshot_str = f"screenshot res: {res_str}x\n"
            elif isinstance(self._screenshot_resolution, tuple):
                # screenshot resolution is a size
                screenshot_str = f"screenshot res: {self._screenshot_resolution[0]}x{self._screenshot_resolution[1]}\n"

        if self._paused:
            playback_str = "playback: paused"
        elif self._live:
            playback_str = "playback: live"
        else:
            rate = SimpleViz._playback_rates[self._rate_ind]
            playback_str = f"playback: {str(rate) + 'x' if rate else 'max'}"

        playback_str = f"fps: {self.frames_per_sec:.1f}   {playback_str}"

        self._right_osd.set_text(f"{screenshot_str}{playback_str}")
        self._right_osd.set_scale(self._frame_viz.text_scale())

    def toggle_video_format(self) -> None:
        if self._video_format == "gif":
            self._video_format = "png"
        else:
            self._video_format = "gif"

        self._viz.set_notification(f"Set continuous screenshot format to {self._video_format}")

    def toggle_pause(self) -> None:
        """Pause or unpause the visualization."""
        with self._cv:
            self._paused = not self._paused
            self._update_right_osd()
            if not self._paused:
                self._cv.notify()

    def seek_relative(self, n_frames: int) -> None:
        """Seek forward of backwards in the stream."""
        with self._cv:
            self._paused = True
            self._step = n_frames
            self._update_right_osd()
            self._cv.notify()

    def modify_rate(self, amount: int) -> None:
        """Switch between preset playback rates."""
        n_rates = len(SimpleViz._playback_rates)
        with self._cv:
            self._rate_ind = max(0, min(n_rates - 1, self._rate_ind + amount))
            self._update_right_osd()
            new_rate: Union[str, float] = SimpleViz._playback_rates[self._rate_ind]
            if new_rate == 0.0:
                new_rate = "max"
            self._viz.set_notification(f"Playback rate set to {new_rate}")

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._cv:
            self._frame_viz.toggle_osd()
            self._update_right_osd()
            self._frame_viz.draw()

    def toggle_img_recording(self) -> None:
        if self._viz_img_recording:
            self._viz_img_recording = False
            if len(self._images):
                self._viz.set_notification("Recording Stopped. Saving...")
            else:
                self._viz.set_notification("Recording Stopped")
            self._save_gif()
        else:
            self._viz_img_recording = True
            self._viz.set_notification("Recording Started")

    def screenshot(self, file_path: Optional[str] = None) -> None:
        file_name: str
        try:
            if self._screenshot_resolution is None:
                # Handle the 'None' case (default)
                file_name = self._viz.save_screenshot(file_path or "")
            elif isinstance(self._screenshot_resolution, float):
                # Handle the scale factor case
                scale_factor = self._screenshot_resolution
                file_name = self._viz.save_screenshot(file_path or "", scale_factor)
            else:
                # Handle the tuple case (width, height)
                width, height = self._screenshot_resolution
                file_name = self._viz.save_screenshot(file_path or "", width, height)
            if file_name:
                print(f"Saved screenshot to: {file_name}")
        except PointVizNotRunningError:
            pass

    _screenshot_resolutions: List[float] = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]

    def cycle_screenshot_res_factor(self, direction: int):
        with self._lock:
            if not self._screenshot_resolution:
                self._screenshot_resolution = 2.0 if direction >= 0 else 0.5
            elif isinstance(self._screenshot_resolution, tuple):
                # Handle the tuple case
                self._screenshot_resolution = 1.0
            else:
                # _screenshot_resolution is already a float
                res = np.floor(self._screenshot_resolution)
                if res > self._screenshot_resolutions[-1]:
                    res = self._screenshot_resolutions[-1]
                elif res < self._screenshot_resolutions[0]:
                    res = self._screenshot_resolutions[0]
                index = self._screenshot_resolutions.index(res)
                if direction > 0:
                    # Next integer above
                    next_up = index + 1
                    if next_up >= len(self._screenshot_resolutions):
                        next_up = 0
                    self._screenshot_resolution = self._screenshot_resolutions[next_up]
                elif direction < 0:
                    # Next integer below
                    next_down = index - 1
                    if next_down < 0:
                        next_down = len(self._screenshot_resolutions) - 1
                    self._screenshot_resolution = self._screenshot_resolutions[next_down]
                # If direction == 0, do nothing
            self._update_right_osd()
            self._frame_viz.draw()

    def set_class_maps(self, class_maps: core.ClassMapSet) -> None:
        """Set the class maps for the visualizer."""
        self._frame_viz.set_class_maps(class_maps)

    def adjust_subframes(self, direction: int):
        with self._lock:
            if direction > 0 and self._subframes < 60:
                self._subframes += 1
            elif direction < 0 and self._subframes > 0:
                self._subframes -= 1
            self._viz.set_notification(f"Set subframes to {self._subframes}")

    def _sleep_playback_interval(self, sleep_time: float,
                                 max_sleep_interval: float = 0.1) -> bool:
        """Sleep up to sleep_time. Returns True if paused during sleep."""
        time_remaining = sleep_time
        while time_remaining > 0 and not self._paused:
            interval = min(max_sleep_interval, time_remaining)
            time.sleep(interval)
            time_remaining -= interval
        return self._paused

    def _cancel_frame_on_pause(self, seekable) -> bool:
        """Seek back if paused mid-frame. Returns True if the frame was canceled."""
        if not self._paused:
            return False
        if self._step > 0:
            seekable.seek(seekable.frame_num - 1)
        else:
            self._step -= 1
        return True

    def _render_camera_frame(self, interpolant: float,
                             update_camera_history: bool) -> None:
        self._frame_viz.set_interpolant(interpolant)
        self._frame_viz._draw_update_camera_pose(
            update_camera_history=update_camera_history)
        self._frame_viz._viz.update()

    def _recording_data_time(self, frame_ts: float, frame_duration: float,
                             interpolant: float,
                             prev_frame_time: Optional[float]) -> float:
        start_ts = (prev_frame_time if prev_frame_time is not None
                    else frame_ts - frame_duration)
        return start_ts + frame_duration * interpolant

    def _append_recording_frame(self, frame_ts: float, frame_duration: float,
                                interpolant: float,
                                prev_frame_time: Optional[float]) -> None:
        if not self._viz_img_recording:
            return
        if self._video_format == "png":
            self.screenshot("")
            return
        try:
            if self._screenshot_resolution is None:
                pix = self._viz.get_screenshot()
            elif isinstance(self._screenshot_resolution, float):
                pix = self._viz.get_screenshot(self._screenshot_resolution)
            else:
                width, height = self._screenshot_resolution
                pix = self._viz.get_screenshot(width, height)
            frame_time = self._recording_data_time(
                frame_ts, frame_duration, interpolant, prev_frame_time)
            self._images.append((PILImage.fromarray(pix), frame_time))
        except PointVizNotRunningError:
            pass

    def _lidar_frame_period(self) -> float:
        if isinstance(self._frame_viz, LidarFrameViz):
            # TODO[UN]: per sensor frame period
            return 1.0 / (self._metadata[0].format.fps)
        else:
            # if some other frame viz that is not derived from LidarFrameViz
            # we default to 10 Hz
            return 1.0 / 10.0

    def _process(self, seekable: _Seekable) -> None:
        frame_idx = -1

        # times for the last sim time since pause or skip
        # when we playback:
        # sim_time = (time.now() - last_time_update)*rate + last_sim_time
        last_sim_time = None
        last_time_update = None

        # packet time (or fake time) of last frame we received
        last_frame_time = None
        # used to detect changes in rate to reset the sim time origin to avoid glitches
        last_rate = None

        # monotonic time of last frame display, used to show frame display rate
        last_play_time = None

        # flag that a loop was signaled
        looped = False

        try:
            while True:
                # wait until unpaused, step, or quit
                try:
                    with self._cv:
                        if self._paused:
                            self._cv.wait_for(lambda: not self._paused or self._step or
                                              self._proc_exit)
                            last_time_update = time.monotonic()
                            last_sim_time = last_frame_time
                            if not self._paused:
                                # update the OSD to show unpaused
                                self._update_right_osd()
                                self._viz.update()
                        if self._proc_exit:
                            break
                        if self._step:
                            seek_ind = seekable.frame_num + self._step - 1
                            self._step = 0
                            if not seekable.seek(seek_ind):
                                continue

                    rate = SimpleViz._playback_rates[self._rate_ind]
                    if rate != last_rate:
                        last_rate = rate
                        last_time_update = time.monotonic()
                        last_sim_time = last_frame_time

                    frame = next(seekable)
                    frames = seekable.last_n(self._frame_viz._imu_viz_num_frames)
                    frame_idx = seekable.frame_num
                    # an empty array indicates that a loop occurred
                    if len(frame) == 0:
                        if self._clear_on_loop:
                            self._frame_viz.clear_trajectory()
                        self._frame_viz.clear_pose_histories()
                        last_frame_time = None
                        looped = True
                        self._step = 1  # skip to next frame, frame 0
                        continue
                    frame_ts = _get_timestamp(frame) / 1e9

                    # fallback if we have no valid ts in the frame
                    if frame_ts == 0:
                        if last_frame_time is None:
                            frame_ts = 0
                        else:
                            frame_ts = last_frame_time + self._lidar_frame_period()

                    # reset playback on loop
                    if looped:
                        last_time_update = time.monotonic()
                        last_sim_time = frame_ts - self._lidar_frame_period()
                        looped = False
                    frame_duration = (frame_ts - last_frame_time if last_frame_time
                                     else self._lidar_frame_period())
                    prev_frame_time = last_frame_time
                    last_frame_time = frame_ts

                    if last_sim_time is None or last_time_update is None:
                        last_sim_time = frame_ts
                        last_time_update = time.monotonic()

                    # Queue up the frames to be viewed
                    self._frame_viz.update(frame, frame_idx - 1, frames)
                    self._frame_viz.draw(update=False, update_camera=False)

                    if self._pause_at == frame_idx - 1:
                        self._paused = True

                    self._update_right_osd()

                    # Sleep until time to "play" this frame if necessary
                    sim_time = (time.monotonic() - last_time_update) * rate + last_sim_time
                    if rate != 0 and not self._paused and not self._live:
                        total_sleep_time = (frame_ts - sim_time) / rate
                        has_update_on_input = hasattr(self._frame_viz, "update_on_input")
                        if has_update_on_input:
                            self._frame_viz.update_on_input = False

                        paused_mid_frame = False
                        if self._subframes == 0:
                            if total_sleep_time > 0:
                                paused_mid_frame = self._sleep_playback_interval(
                                    total_sleep_time)
                            if not paused_mid_frame:
                                self._render_camera_frame(1.0, update_camera_history=True)
                                self._append_recording_frame(
                                    frame_ts, frame_duration, 1.0, prev_frame_time)
                        else:
                            subframe_sleep = (0 if self._viz_img_recording else
                                              total_sleep_time / (self._subframes + 1))
                            for subframe in range(self._subframes + 1):
                                if subframe_sleep > 0:
                                    paused_mid_frame = self._sleep_playback_interval(
                                        subframe_sleep)
                                    if paused_mid_frame:
                                        break
                                interpolant = subframe / (self._subframes + 1)
                                is_last = subframe == self._subframes
                                self._render_camera_frame(
                                    interpolant, update_camera_history=is_last)
                                # Skip subframe 0 after the first frame; it
                                # duplicates the previous frame's final subframe.
                                if subframe > 0 or prev_frame_time is None:
                                    self._append_recording_frame(
                                        frame_ts, frame_duration, interpolant,
                                        prev_frame_time)
                            if self._paused:
                                paused_mid_frame = True

                        if has_update_on_input:
                            self._frame_viz.update_on_input = True
                        if paused_mid_frame and self._cancel_frame_on_pause(seekable):
                            continue
                    else:  # if live, pause, or rate is 0
                        self._render_camera_frame(1.0, update_camera_history=True)
                        self._append_recording_frame(
                            frame_ts, frame_duration, 1.0, prev_frame_time)

                    # Update frame rate display calculation
                    now_ts = time.monotonic()
                    last = last_play_time or now_ts
                    dt = now_ts - last
                    last_play_time = now_ts
                    self._last_draw_period = dt
                except StopIteration:
                    if not self._paused and not self._on_eof == "stop":
                        break

                    # Pause after we get a StopIteration in eof "stop"
                    if self._on_eof == "stop":
                        self._paused = True
                        self._update_right_osd()
                        self._viz.update()

        finally:
            # signal rendering (main) thread to exit, with a delay
            # because the viz in main thread may not have been started
            # and on Mac it was observed that it fails to set a flag if
            # _process fails immediately after start
            time.sleep(0.5)
            self._viz.running(False)

    @property
    def frames_per_sec(self) -> float:
        """Frames per second processing rate."""
        if self._last_draw_period > 0:
            return 1.0 / self._last_draw_period
        else:
            return 0.0

    @property
    def scans_per_sec(self) -> float:
        """Deprecated: use frames_per_sec."""
        warn_deprecated("scans_per_sec is deprecated, use frames_per_sec")
        return self.frames_per_sec

    def _save_gif(self, filename=None) -> None:
        if len(self._images) == 0:
            return

        if filename is None:
            filename = f"viz_gif_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.gif"

        print("Saving gif to " + filename)
        start = self._images[0][0]
        rate = SimpleViz._playback_rates[self._rate_ind]
        # live => rate of 1.0
        if rate <= 0.0:
            rate = 1.0

        # calculate durations in ms between frames
        default_dt = (self._lidar_frame_period() / (self._subframes + 1)
                      if self._subframes else self._lidar_frame_period())
        images = []
        durations = []
        for i in range(1, len(self._images)):
            images.append(self._images[i][0])
            dt = self._images[i][1] - self._images[i - 1][1]
            if dt <= 0:
                dt = default_dt
            durations.append(dt * 1000 / rate)
        # assume last frame has same duration as the one before (or some default if single frame)
        durations.append(durations[-1] if len(durations) else 100)

        # save in a background thread to avoid freezing the viz
        def run_in_thread():
            start.save(filename, save_all=True, append_images=images, duration=durations, loop=0)
            self._viz.set_notification("Finished Saving GIF")
            print("Finished Saving GIF")

        thread = threading.Thread(target=run_in_thread)
        thread.start()

        self._images = []

    def run(self, frames: Union[Iterable[FrameSet], Iterable[LidarFrame], Iterable[List[Optional[LidarFrame]]]]
            ) -> None:
        """Start reading frames and visualizing the stream.

        Must be called from the main thread on macOS. Will close the provided
        frame source before returning.

        Args:
            frames: A stream of frames to visualize.

        Returns:
            When the stream is consumed or the visualizer window is closed.
        """

        # loop the source if we want loop
        if self._on_eof == "loop":
            def looper(frames):
                while True:
                    for frame in frames:
                        yield frame
                    yield FrameSet()  # signal a loop occurred
            frames = looper(frames)

        def shim(frames) -> Iterable[FrameSet]:
            for frame in frames:
                if isinstance(frame, FrameSet):
                    yield frame
                elif isinstance(frame, LidarFrame):
                    yield FrameSet([frame])
                else:
                    yield FrameSet(frame)
        frames = shim(frames)

        if self._live:
            # If live, create a "LiveConsumer" to drop frames if the viz is too slow
            # to keep up with the source. The lambda indicates whether a dropped frame should be counted.
            frames = LiveConsumer(frames, lambda: not self._paused)

        seekable = _Seekable(frames, maxlen=self._buflen)

        try:
            logger.info("Starting processing thread...")
            self._proc_exit = False
            proc_thread = threading.Thread(name="Viz processing",
                                           target=self._process,
                                           args=(seekable, ))
            proc_thread.start()

            logger.info("Starting rendering loop...")
            if isinstance(frames, LiveConsumer):
                frames.start()
            self._viz.run()
            logger.info("Done rendering loop")
        except KeyboardInterrupt:
            print("Termination requested, shutting down...")
        finally:
            if isinstance(frames, LiveConsumer):
                frames.shutdown()
                if frames.dropped_frames:
                    logging.warning("The visualizer dropped %d frames during playback - "
                        "please make sure to specify a playback rate if working with "
                        "a file-based source!", frames.dropped_frames)

            # save a gif if we have one pending
            self._save_gif()

            # processing thread will still be running if e.g. viz window was closed
            with self._cv:
                self._proc_exit = True
                self._cv.notify()

            logger.info("Joining processing thread")
            proc_thread.join()
            self._frame_viz._model.finish()


def lf_show(frames: Union[FrameSetSource, LidarFrame, FrameSet, List[Optional[LidarFrame]],
                         List[List[Optional[LidarFrame]]]],
            *,
            title: Optional[str] = None) -> None:
    """[BETA] Display a set of LidarFrames in an interactive window.

    Args:
        frames: A set of LidarFrames to visualize.

    Optional Args:
        title: Title of the visualization window. If not provided,
               it will be composed from the sensor serial numbers.

    Note:
        This is a beta feature and its API may change in future releases.
    """

    def compose_title(sensor_infos: List[SensorInfo]) -> str:
        return ",".join([str(si.sn) for si in sensor_infos])

    def get_sensor_infos(frames: Union[FrameSetSource, List[FrameSet]]) -> List[SensorInfo]:
        return [s.sensor_info for s in frames[0] if s is not None]

    def normalize(frames: Union[LidarFrame, FrameSet, List[Optional[LidarFrame]], List[List[Optional[LidarFrame]]]]
                  ) -> List[FrameSet]:
        if isinstance(frames, LidarFrame):
            return cast(List[FrameSet], [FrameSet([frames])])
        elif isinstance(frames, FrameSet):
            return [frames]
        elif isinstance(frames, list) and len(frames) > 0 and isinstance(frames[0], list) and \
                len(frames[0]) > 0:
            return cast(List[FrameSet], [FrameSet(ss) for ss in frames])  # type: ignore
        elif isinstance(frames, list) and len(frames) > 0:
            return cast(List[FrameSet], [FrameSet(frames)])  # type: ignore
        else:
            raise ValueError("frames must be a LidarFrame, a list of LidarFrames or a list of "
                             "lists of LidarFrames and should not be empty")

    frames_it: Union[FrameSetSource, List[FrameSet]]
    frames_it = frames if isinstance(frames, FrameSetSource) else normalize(frames)
    sensors_infos = get_sensor_infos(frames_it)
    title = title or f"Ouster Viz: {compose_title(sensors_infos)}"
    SimpleViz(sensors_infos, title=title, pause_at=0,
              on_eof='loop', rate=1.0).run(cast(Iterable[FrameSet], frames_it))

"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Sensor data visualization tools.

Visualize lidar data using OpenGL.
"""

import collections.abc
from collections import deque
from functools import partial
from enum import Enum, auto
import itertools
import threading
import queue
import time
from datetime import datetime, timezone
from typing import (Callable, ClassVar, cast, Deque, Dict, Iterable, Iterator, List,
                    Optional, Sequence, Tuple, TypeVar, Union)
import logging

import numpy as np

from ouster.sdk import core
from ouster.sdk.core import (Frame, LidarFrame, CameraFrame, ChanField, FrameSet, FrameSetSource,
                             SensorInfo)
from ouster.sdk._bindings.viz import (PointVizNotRunningError, PointViz, Cloud, Label, WindowCtx,
                   add_default_controls, MouseButton, MouseButtonEvent, EventModifierKeys,
                   distinct_palette, MouseEventType)
from ouster.sdk._bindings.viz import Image, Cuboid, Camera, TargetDisplay, ObjectOverlay  # noqa: F401
from ouster.sdk.io import GifWriter
from ouster.sdk.viz.model import VizExtraMode, LidarFrameVizModel, \
        ObjectColorMode, ObjectViewMode, discrete_rainbow_palette  # noqa: F401
from . import util as vizu
from ouster.sdk.viz.view_mode import ImageMode, CloudMode, CloudPaletteItem  # noqa: F401
from ouster.sdk.viz.accumulators import LidarFrameVizAccumulators
from ouster.sdk.viz.accumulators_config import LidarFrameVizAccumulatorsConfig, MAP_MAX_POINTS_NUM, MAP_SELECT_RATIO
from ouster.sdk.viz.model import (SensorModel, CameraModel, ZoneSelectionMode, ZoneRenderMode,
                                  DEFAULT_CAMERA_FRUSTUM_FAR, DEFAULT_CAMERA_RANGE_MM)
from ouster.sdk.viz.util import VizConfig
import weakref

logger = logging.getLogger("viz-logger")

T = TypeVar('T')


def _resize_nearest(image: np.ndarray, height: int, width: int) -> np.ndarray:
    rows = ((2 * np.arange(height) + 1) * image.shape[0]) // (2 * height)
    cols = ((2 * np.arange(width) + 1) * image.shape[1]) // (2 * width)
    return image[rows[:, None], cols]


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


def _valid_or_zero_ts(frame: core.Frame):
    # TODO: this may need addressing or refactoring
    if isinstance(frame, LidarFrame):
        try:
            return frame.get_first_valid_receive_timestamp()
        except RuntimeError:
            return 0
    # Non-lidar frames (e.g. CameraFrame) have no packets but still carry
    # per-frame timing via receive_timestamp; use the first nonzero entry.
    pts = frame.receive_timestamp
    nonzero = pts[pts != 0]
    return int(nonzero[0]) if nonzero.size else 0


def _frame_set_has_content(frames: FrameSet) -> bool:
    """True when the FrameSet contains lidar or camera data."""
    return len(frames) > 0


def _frame_set_is_loop_sentinel(frames: FrameSet) -> bool:
    """True for the empty FrameSet yielded to signal playback loop."""
    return len(frames) == 0


def _get_timestamp(frames: FrameSet):
    ts = [t for t in (_valid_or_zero_ts(s) for s in frames if s is not None) if t]
    receive_timestamp = getattr(frames, "receive_timestamp", 0)
    if receive_timestamp:
        ts.append(receive_timestamp)
    if not ts:
        return 0
    return min(ts)


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

    class CameraMode(Enum):
        FOLLOW = 0
        FOLLOW_ROTATION_LOCKED = 1
        FOLLOW_WORLD_ROTATION_LOCKED = 2
        FIXED = 3
        FOLLOW_SMOOTH = 4
        FOLLOW_SMOOTH_ROTATION_LOCK = 5
        FOLLOW_SMOOTH_WORLD_ROTATION_LOCK = 6

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
            metas: Sequence[SensorInfo],
            point_viz: Optional[PointViz] = None,
            accumulators_config: Optional[LidarFrameVizAccumulatorsConfig] = None,
            imu_viz_config: ImuVisualizationConfig = ImuVisualizationConfig(),
            *,
            _img_aspect_ratio: float = 0,
            _buflen = 50,
            _add_default_controls = True,
            camera_frustum_far: float = DEFAULT_CAMERA_FRUSTUM_FAR,
            camera_default_range_mm: int = DEFAULT_CAMERA_RANGE_MM,
            background_color: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> None:
        """
        Args:
            metas: sensor metadata used to interpret frames. May be a combined
                   collection of LidarInfo and CameraInfo.
            viz: use an existing PointViz instance instead of creating one
            camera_frustum_far: distance (m) from each camera's origin to the
                   far plane of its 3D FOV frustum wireframe.
            camera_default_range_mm: distance (mm) at which to draw the pixels
                   of a camera frame that has no RANGE field, so its images show
                   as a 3D point cloud; 0 draws no cloud for such frames. It is
                   a forward distance, so the image forms a flat plane.
        """
        self._viz = point_viz or PointViz("Ouster Viz")
        self._add_default_controls = _add_default_controls

        # used to synchronize key handlers and _draw()
        self._lock = threading.RLock()
        self._model = LidarFrameVizModel(self._viz, metas,
                                         _img_aspect_ratio=_img_aspect_ratio,
                                         camera_frustum_far=camera_frustum_far,
                                         camera_default_range_mm=camera_default_range_mm)
        # One scale entry per stream (lidars first, then cameras), matching the
        # frame positions used by imu_plot(). Cameras with IMU data are plotted
        # alongside lidars.
        self._imu_acc_scale = [{'IMU_ACC': 1, 'IMU_GYRO': 1}
                               for _ in self._model._stream_models()]
        self._imu_viz_num_frames = _buflen
        self.update_on_input = True

        # extension point for the OSD text, inserts before the "axes" line
        self._osd_text_extra: Callable[[], str] = lambda: ""

        # set initial image sizes
        self._image_size_initialized = False
        self._image_view_mode = LidarFrameViz.ImageViewMode.ALL

        self._viz.set_background_color(background_color)
        self._background_color = background_color

        def on_change(real_self):
            real_self.draw(update=real_self.update_on_input)
        self.config = VizConfig(self._viz, weakref.ref(self), on_change=on_change)

        # initial point size in frame clouds
        for label in self._model._aoi_labels:
            self._viz.add(label)

        self._frames_accum: Optional[LidarFrameVizAccumulators] = None
        if accumulators_config and metas:
            self._frames_accum = LidarFrameVizAccumulators(
                self._model,
                self._viz,
                accumulators_config,
                self._lock,
                self.config
            )

            # TODO[tws]: decide the fate of FramesAccumulator's OSD
            self._frames_accum.toggle_osd(False)
            if hasattr(self, "_osd_text_extra"):
                self._osd_text_extra = self._frames_accum.osd_text

        self._imu_viz_config = imu_viz_config
        self._osd_scale = 1

        # add images and clouds to viz
        for sensor in self._model._sensors:
            [self._viz.add(image) for image in sensor._images]
            [self._viz.add(cloud) for cloud in sensor._clouds]
            assert sensor._object_overlay is not None
            self._viz.add(sensor._object_overlay)
        for camera in self._model._cameras:
            [self._viz.add(image) for image in camera._images]
            if camera._object_overlay is not None:
                self._viz.add(camera._object_overlay)
            # Frustum follows the axis-marker toggle (initially shown).
            self._set_frustum_visible(camera, True)
            # camera clouds are added lazily once frame dimensions are known

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

        self._eye_dome = False

        # initialize frame axis helpers
        self._frame_axis_enabled = True
        # One scene axis per stream, in _stream_models() order (lidar and camera
        # streams may be interleaved); each stream supplies its own initial pose.
        self._stream_axis: List[vizu.AxisWithLabel] = []
        for idx, stream in enumerate(self._model._stream_models()):
            self._stream_axis.append(
                vizu.AxisWithLabel(self._viz,
                                   pose=stream.axis_pose(),
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

        self._camera_mode = LidarFrameViz.CameraMode.FOLLOW_WORLD_ROTATION_LOCKED
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
        self._zone_selection_mode = ZoneSelectionMode.LIVE
        self._zone_render_mode = ZoneRenderMode.STL
        self.set_zone_selection_and_render_modes(self._zone_selection_mode, self._zone_render_mode)

        self._setup_controls()

    def imu_plot(self, frames, imu_viz_config):
        img_h = self._imu_viz_config.imu_plot_height_pixels // self._imu_viz_config.downsample_factor
        img_w = self._imu_viz_config.imu_plot_width_pixels // self._imu_viz_config.downsample_factor

        stream_frame_indices = self._model._stream_frame_indices()

        def add_field(field_name, img, color, sensor_idx):
            # Map this stream's index to its position in the unified FrameSet
            # (lidar and camera streams may be interleaved).
            frame_idx = (stream_frame_indices[sensor_idx]
                         if sensor_idx < len(stream_frame_indices) else sensor_idx)
            # Find first valid frame to get measurements_per_frame
            # (handles case where a frame is None due to collation gaps or missing data)
            first_valid = None
            for s in frames:
                if s[frame_idx] is not None and s[frame_idx].has_field(field_name):
                    first_valid = s[frame_idx]
                    break
            if first_valid is None:
                return  # No valid frames with this field in the buffer
            measurements_per_frame = first_valid.field(field_name).shape[0]
            # Use NaN for missing frames so gaps appear instead of false zeros
            all_frame_values = np.full((measurements_per_frame * len(frames),), np.nan)
            self._imu_acc_scale[sensor_idx][field_name] = 1.0
            for idx, frame in enumerate(frames):
                frame = frames[idx][frame_idx]
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

        # Iterate all streams (lidars then cameras). add_field() maps each
        # stream index to its FrameSet position via _stream_frame_indices(), so
        # it works for both lidar and camera streams in any (interleaved) order.
        for sensor_idx, sensor in enumerate(self._model._stream_models()):
            img = np.zeros((img_h, img_w, 4), np.float32)
            assert img.shape[2] == 4
            for field in imu_viz_config._fields:
                add_field(field, img, imu_viz_config.field_colors[field], sensor_idx)
            sensor._imu_plot.set_image(img)

    # TODO[tws] likely remove
    @property
    def metadata(self) -> Sequence[SensorInfo]:
        """Metadata for every displayed sensor, in FrameSet order."""
        return self._model.metadata

    def set_class_maps(self, class_maps: core.ClassMapSet) -> None:
        """Set the class maps for the visualizer."""
        with self._lock:
            self._model.set_class_maps(class_maps)

    @property
    def osd_state(self) -> "LidarFrameViz.OsdState":
        """Returns the state of the on screen display."""
        return self._osd_state

    def cycle_osd_scale(self):
        self.set_osd_scale(max(0, (self._osd_scale + 1) % 5))

    def set_osd_scale(self, scale):
        self._osd_scale = scale
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

    def toggle_sky(self, value: Optional[bool] = None) -> None:
        """Toggle showing the color the sky in the pointcloud where there is no return"""
        with self._lock:
            for idx, sensor in enumerate(self._model._sensors):
                if value is not None:
                    sensor._show_sky = value
                else:
                    sensor._show_sky = not sensor._show_sky
                frame_idx = self._model._sensor_frame_indices[idx]
                if frame_idx < len(self._frame_set):
                    sensor.update_clouds(self._model._cloud_mode_name,
                                         cast(Optional[LidarFrame], self._frame_set[frame_idx]))
            self._viz.update()

    def toggle_eye_dome(self, value: Optional[bool] = None) -> None:
        with self._lock:
            if value is not None:
                self._eye_dome = value
            else:
                self._eye_dome = not self._eye_dome
            self._viz.enable_eye_dome_lighting(self._eye_dome)
            if self._eye_dome:
                if self._background_color == (0.0, 0.0, 0.0, 1.0):
                    self._viz.set_background_color((0.5, 0.5, 0.5, 1.0))
                self._viz.set_notification("Enabled eye-dome lighting")
            else:
                self._viz.set_background_color(self._background_color)
                self._viz.set_notification("Disabled eye-dome lighting")

    def _setup_controls(self) -> None:
        # key bindings. will be called from rendering thread, must be synchronized
        key_bindings: Dict[Tuple[int, int], Callable[[LidarFrameViz], None]] = {
            (ord('/'), 1): LidarFrameViz.toggle_help,
            (ord('1'), 1): partial(LidarFrameViz.set_view, view='top'),
            (ord('2'), 1): partial(LidarFrameViz.set_view, view='forward'),
            (ord('3'), 1): partial(LidarFrameViz.set_view, view='left'),
        }

        self._key_definitions: Dict[str, str] = {
            'w': "Camera pitch down",
            's': "Camera pitch up",
            'a': "Camera yaw right",
            'd': "Camera yaw left",
            'q': "Camera roll left",
            'e': "Camera roll right",
            "SHIFT+r": "Reset camera orientation",
            "SHIFT+1": "Top-down view",
            "SHIFT+2": "Front-facing view",
            "SHIFT+3": "Left-facing view",
            "CTRL+r": "Camera bird-eye view",
            "0": "Toggle orthographic camera",
            "CTRL+- / CTRL+=": "Increase/decrease field of view (perspective only)",
            '?': "Print keys to standard out",
            "= / -": "Dolly in and out",
            'SHIFT': "Camera Translation with mouse drag",
            'ESC': "Exit the application",
        }

        self.config.option("show_return_1", True, ('1', 0),
                           lambda arg, val: arg.toggle_cloud(0, val),
                           help="Toggle first return point cloud")
        self.config.option("show_return_2", True, ('2', 0),
                           lambda arg, val: arg.toggle_cloud(1, val),
                           help="Toggle second return point cloud")
        self.config.option("track_view_mode", self._track_view_mode, ('8', 0),
                           lambda arg, val: arg.set_track_view_mode(val),
                           help="Toggle track view mode")
        self.config.option("object_view_mode", self._model._object_view_mode, ('5', 0),
                           lambda arg, val: arg.set_object_view_mode(val),
                           help="Cycle object display mode")
        self.config.option("image_mode", self._image_view_mode, ('I', 2),
                           lambda arg, val: arg.set_img_view_mode(val),
                           help="Flip or hide 2D images")
        self.config.option("flags_mode", self._flags_mode, ('C', 0),
                           lambda arg, val: arg.update_flags_mode(val, 0),
                           decrement=('C', 1),
                           help="Cycle current highlight mode")
        self.config.option("camera_mode", self._camera_mode, ('U', 0),
                           lambda arg, val: arg.cycle_camera_mode(0, val),
                           decrement=('U', 1), help="Cycle camera mode")
        self.config.option("point_size", 2, ('P', 0),
                           lambda arg, val: arg.set_point_size(val), min=1, max=10,
                           decrement=('P', 1),
                           help="Increase/decrease point size")
        self.config.option("osd_size", self._osd_scale, ('O', 2),
                           lambda arg, val: arg.set_osd_scale(val), min=0, max=4,
                           help="Cycle on-screen display scale")
        self.config.option("show_osd", True, ('O',),
                           lambda arg, val: arg.toggle_osd(val),
                           help="Toggle OSD")
        self.config.option("ring_width", 2, ("'", 2),
                           lambda arg, val: arg.cicle_ring_line_width(val),
                           min=0, max=9,
                           help="Cycle through thickness of range markers or hide")
        self.config.option("ring_size", self._ring_size, ("'", 0),
                           lambda arg, val: arg.update_ring_size(0, val),
                           min=-2, max=3, decrement=("'", 1),
                           help="Increase/decrease spacing in range markers")
        self.config.option("image_size", 4, ('I', 0),
                           lambda arg, val: arg.set_image_size(val),
                           min=0, max=20, decrement=("I", 1),
                           help="Increase/decrease size of displayed 2D images")
        self.config.option("show_sky", False, ('S', 2),
                           lambda arg, val: arg.toggle_sky(val),
                           help="Toggle sky visualization")
        self.config.option("eye_dome", False, ('E', 2),
                           lambda arg, val: arg.toggle_eye_dome(val),
                           help="Toggle eye-dome lighting")
        self.config.option("flags_palette", self._flags_palette, ('C', 2),
                           lambda arg, val: arg.cycle_flags_palette(val),
                           help="Toggle flags palette")
        self.config.option("object_color_mode", self._model._object_color_mode, ('C', 3),
                           lambda arg, val: arg.cycle_object_color_mode(val),
                           help="Cycle object color attribute")
        self.config.option("axis_markers", True, ('9',),
                           lambda arg, val: arg.toggle_axis_markers(val),
                           help="Toggle axis helpers at frame origin")
        self.config.option("zone_selection_mode", self._zone_selection_mode, ('Y', 0),
                           lambda arg, val: arg.set_zone_selection_mode(val),
                           help="Cycle displayed zone set.")
        self.config.option("zone_render_mode", self._zone_render_mode, ('Y', 2),
                           lambda arg, val: arg.set_zone_render_mode(val),
                           help="Cycle zone render mode.")
        self.config.option("cloud_mode", "", ('M', 0),
                           lambda arg, val: arg.select_cloud_mode(val, True),
                           decrement=('M', 1),
                           items=lambda arg: arg._model.sorted_cloud_mode_names(),
                           get_value=lambda arg: arg._model._cloud_mode_name,
                           help="Cycle through point cloud coloring mode")
        self.config.option("image_mode_0", "", ('B', 0),
                           lambda arg, val: arg.select_img_mode(0, val, True),
                           decrement=('B', 1),
                           items=lambda arg: arg._model.sorted_image_mode_names(),
                           get_value=lambda arg: arg._model._image_mode_names[0],
                           help="Cycle top 2D image")
        self.config.option("image_mode_1", "", ('N', 0),
                           lambda arg, val: arg.select_img_mode(1, val, True),
                           decrement=('N', 1),
                           items=lambda arg: arg._model.sorted_image_mode_names(),
                           get_value=lambda arg: arg._model._image_mode_names[1],
                           help="Cycle bottom 2D image")
        self.config.option("cloud_palette", "", ('F', 0),
                           lambda arg, val: arg.set_cloud_palette(val),
                           decrement=('F', 1),
                           items=lambda arg: [p.name for p in arg._model._palettes._cloud_palettes],
                           get_value=lambda arg: arg._model._palettes.get_current_palette().name,
                           help="Cycle through point cloud color palette")

        # add sensor/camera toggle keys
        self._setup_sensor_toggle_keys(self._model._stream_models())

        conf_keys = self.config.key_definitions()
        for key in conf_keys:
            self._key_definitions[key] = conf_keys[key]

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

    def _setup_sensor_toggle_keys(self, streams: List[Union[SensorModel, CameraModel]]) -> None:
        for stream_index in range(min(len(streams), LidarFrameViz.MAX_SENSOR_TOGGLE_KEYS)):
            cb = lambda arg, val, idx=stream_index: arg.toggle_sensor(idx, enabled=val)
            self.config.option("toggle_sensor_" + str(stream_index), True,
                               (str(stream_index + 1), 2), cb,
                               help=f"Toggle stream {stream_index + 1} point cloud(s)")

    def select_img_mode(self, i: int, mode: str, force: bool = False) -> bool:
        """Change the displayed field of the i'th image."""
        with self._lock:
            res = self._model.select_image_mode(i, mode, force)
            self._model.update_aoi()
            self._model.update_aoi_label(self._frame_set)

            # Note, updating the image mode needs the current frame
            # because ImageMode.enabled requires it.
            if len(self._frame_set) == len(self._model._stream_models()):
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
            if len(self._frame_set) == len(self._model._stream_models()):
                self._model.update(self._frame_set, False)

    def select_cloud_mode(self, mode: str, force: bool = False) -> bool:
        """Change the coloring mode of the 3D point cloud."""
        with self._lock:
            res = self._model.select_cloud_mode(mode, force)

            # Note, updating the cloud mode needs the current frame
            # because CloudMode.enabled requires it.
            if len(self._frame_set) == len(self._model._stream_models()):
                self._model.update(self._frame_set, False)
            return res

    def cycle_cloud_mode(self, direction: int = 1) -> None:
        """Change the coloring mode of the 3D point cloud."""
        with self._lock:
            self._model.cycle_cloud_mode(direction)

            self._model.update_aoi()

            # Note, updating the cloud mode needs the current frame
            # because CloudMode.enabled requires it.
            if len(self._frame_set) == len(self._model._stream_models()):
                self._model.update(self._frame_set, False)
            self._viz.set_notification(f"Cloud mode: {self._model._cloud_mode_name.replace('_', ' ')}")

    def cycle_cloud_palette(self, *, direction: int = 1) -> None:
        """Change the color palette of the 3D point cloud."""
        with self._lock:
            # move to LidarFrameVizModel
            self._model._palettes.cycle_cloud_palette(direction)
            self._model.update_cloud_palettes()
            self._viz.set_notification(f"Cloud palette: {self._model._palettes.get_current_palette().name}")

    def set_cloud_palette(self, name: str) -> None:
        with self._lock:
            self._model._palettes.set_palette_by_name(name)
            self._model.update_cloud_palettes()
            self._viz.set_notification(f"Cloud palette: {self._model._palettes.get_current_palette().name}")

    def toggle_cloud(self, i: int, enable: Optional[bool] = None) -> None:
        """Toggle whether the i'th return is displayed."""
        with self._lock:
            if i >= len(self._model._cloud_enabled):
                return
            if enable is not None:
                self._model._cloud_enabled[i] = enable
            else:
                self._model._cloud_enabled[i] = not self._model._cloud_enabled[i]
            if not self._model._cloud_enabled[i]:
                for stream in self._model._stream_models():
                    if i < len(stream._clouds):
                        self._viz.remove(stream._clouds[i])
            else:
                for stream in self._model._stream_models():
                    if i < len(stream._clouds) and stream._enabled:
                        self._viz.add(stream._clouds[i])

    def toggle_sensor(self, sensor_index: int, enabled: Optional[bool] = None) -> None:
        """Toggle whether the i'th sensor or camera stream is displayed."""
        with self._lock:
            streams = self._model._stream_models()
            if sensor_index >= len(streams):
                return
            stream = streams[sensor_index]
            stream._enabled = (not stream._enabled) if enabled is None else enabled
            if stream._enabled:
                for i, cld in enumerate(stream._clouds):
                    if i < len(self._model._cloud_enabled) and self._model._cloud_enabled[i]:
                        self._viz.add(cld)
                for img in stream._images:
                    self._viz.add(img)
                if stream._object_overlay is not None:
                    self._viz.add(stream._object_overlay)
                # Frustum is shown only when axis markers are also on.
                self._set_frustum_visible(stream, self._frame_axis_enabled)
                if sensor_index < len(self._stream_axis):
                    self._stream_axis[sensor_index].enable()
            else:
                for cld in stream._clouds:
                    self._viz.remove(cld)
                for img in stream._images:
                    self._viz.remove(img)
                if stream._object_overlay is not None:
                    self._viz.remove(stream._object_overlay)
                self._set_frustum_visible(stream, False)
                if sensor_index < len(self._stream_axis):
                    self._stream_axis[sensor_index].disable()
                # a no-op for streams without zones (cameras)
                stream.update_zones()

            #  clear the selection if we're hiding the stream
            if any([selection._sensor_index == sensor_index for selection in self._model._current_selection]):
                self._model.clear_aoi()

            if self._frames_accum and isinstance(stream, SensorModel):
                self._frames_accum.toggle_sensor(sensor_index, stream._enabled)

        self.update_image_size(0)

    def set_point_size(self, size: int) -> None:
        """Change the point size of the 3D cloud."""
        with self._lock:
            # TODO refactor add to model
            self._model._set_cloud_pt_size(size)
            self._viz.set_notification(f"Point size: {int(self._model._cloud_pt_size)}")

    def set_image_size(self, amount: int) -> None:
        """Set the size of the 2D image and position image labels."""
        with self._lock:
            self._model.set_image_size(amount)

    def update_image_size(self, amount: int) -> None:
        """Change the size of the 2D image and position image labels."""
        with self._lock:
            self._model.update_image_size(amount)

    def toggle_flip_images(self) -> None:
        """Toggle if 2D images should be flipped or not."""
        with self._lock:
            self._model.toggle_flip_images()

    def set_img_view_mode(self, mode):
        with self._lock:
            self._image_view_mode = mode
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

    def update_ring_size(self, amount: int, setpoint: Optional[int] = None) -> None:
        """Change distance ring size."""
        with self._lock:
            if setpoint is not None:
                self._ring_size = setpoint
            else:
                self._ring_size = min(3, max(-2, self._ring_size + amount))
            self._viz.target_display.set_ring_size(self._ring_size)
            self._viz.set_notification(
                f"Ring size: {self._viz.target_display.get_ring_size_m()} meters")

    def cicle_ring_line_width(self, width: Optional[int] = None) -> None:
        """Change rings line width."""
        with self._lock:
            if width is not None:
                self._ring_line_width = width
            else:
                self._ring_line_width = max(0, (self._ring_line_width + 1) % 10)
            self._viz.target_display.set_ring_line_width(self._ring_line_width)
            self._viz.target_display.enable_rings(self._ring_line_width != 0)
            self._viz.set_notification(f"Ring width: {self._ring_line_width} pixels")

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._lock:
            show = self._osd_state != LidarFrameViz.OsdState.DEFAULT if state is None else state
            if show:
                self._osd_state = LidarFrameViz.OsdState.DEFAULT
            else:
                self._osd_state = LidarFrameViz.OsdState.NONE

    def cycle_camera_mode(self, direction: int = 1, mode: 'Optional[LidarFrameViz.CameraMode]' = None) -> None:
        """Toggle the camera follow mode."""
        with self._lock:
            if mode is not None:
                self._camera_mode = mode
            else:
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
            for frame, sensor in zip(self._model._lidar_frames(self._frame_set),
                                     self._model._sensors):
                if not frame:
                    continue
                for i, range_field in ((0, ChanField.RANGE),
                                       (1, ChanField.RANGE2)):
                    if range_field in frame.fields:
                        sensor._clouds[i].set_range(frame.field(range_field))

            # TODO[UN]: need to be done as part of the combined lidar mode
            # set mask on all points in the second cloud, clear other mask
            # todo fix this not working on startup if there are no clouds
            for i, sensor in enumerate(self._model._sensors):
                sensor._cloud_masks[0][:, :, 3] = 0
                sensor._clouds[0].set_mask(sensor._cloud_masks[0])
                if len(sensor._cloud_masks) > 1:
                    if self._flags_mode == LidarFrameViz.FlagsMode.HIGHLIGHT_SECOND:
                        sensor._cloud_masks[1][:, :, 3] = 255
                    else:
                        sensor._cloud_masks[1][:, :, 3] = 0
                    sensor._clouds[1].set_mask(sensor._cloud_masks[1])

            # update masks based on AOI if one is set
            if self._flags_mode == LidarFrameViz.FlagsMode.NONE:
                self._model.clear_masks()
                self._model.update_aoi()

    def _draw_update_flags_mode(self) -> None:
        """Apply selected FlagsMode to the cloud"""
        # set a cloud mask based where first flags bit is set
        for frame, sensor in zip(self._model._lidar_frames(self._frame_set),
                                 self._model._sensors):
            # Frames are matched to lidar sensors by recorded FrameSet position;
            # flags masks are lidar-specific.
            frame = cast(Optional[LidarFrame], frame)
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

    def cycle_flags_palette(self, value: 'Optional[LidarFrameViz.FlagsPalette]' = None) -> None:
        """Cycle the palette used for flags modes."""
        with self._lock:
            if value is not None:
                self._flags_palette = value
            else:
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

    def cycle_object_color_mode(self, value: 'Optional[ObjectColorMode]' = None) -> None:
        """Cycle the mode used for object highlighting."""
        with self._lock:
            if value is not None:
                self._model._object_color_mode = value
            else:
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

    def set_object_view_mode(self, mode: ObjectViewMode) -> None:
        with self._lock:
            self._model._object_view_mode = mode
            if self._frame_set:
                self._model.update_objects(self._frame_set)
            self._viz.set_notification(
                f"Object visibility: {self._model._object_view_mode.name.replace('_', ' ')}")

    def toggle_track_view_mode(self) -> None:
        """Cycle through the track view modes"""
        with self._lock:
            self.set_track_view_mode(LidarFrameViz.TrackViewMode(
                (self._track_view_mode.value + 1) %
                len(LidarFrameViz.TrackViewMode.__members__)))

    def set_track_view_mode(self, mode: "LidarFrameViz.TrackViewMode") -> None:
        """Set the track view mode"""
        with self._lock:
            self._track_view_mode = mode
            if self._track_view_mode == LidarFrameViz.TrackViewMode.OFF:
                if self._frames_accum:
                    self._frames_accum.toggle_mode_track(False)
                for trajectory in self._per_frame_pose_trajectory:
                    self._viz.remove(trajectory)
                for axis in self._per_frame_pose_trajectory_axis:
                    axis.disable()
            elif self._track_view_mode == LidarFrameViz.TrackViewMode.TRACK_ONLY:
                if self._frames_accum:
                    self._frames_accum.toggle_mode_track(True)
            elif self._track_view_mode == LidarFrameViz.TrackViewMode.TRACK_PLUS_PER_FRAME_TRAJECTORY:
                for trajectory in self._per_frame_pose_trajectory:
                    self._viz.add(trajectory)
                for axis in self._per_frame_pose_trajectory_axis:
                    axis.enable()
            self._viz.set_notification(f"Track: {self._track_view_mode.name.replace('_', ' ')}")

    def _set_frustum_visible(self, stream: Union[SensorModel, CameraModel],
                             visible: bool) -> None:
        """Add/remove a stream's FOV frustum from the scene, idempotently."""
        if stream._frustum is None:
            return
        if visible and not stream._frustum_in_viz:
            self._viz.add(stream._frustum)
            stream._frustum_in_viz = True
        elif not visible and stream._frustum_in_viz:
            self._viz.remove(stream._frustum)
            stream._frustum_in_viz = False

    def toggle_axis_markers(self, show: Optional[bool] = None) -> None:
        """Toggle the helper axis of a frame ON/OFF"""
        with self._lock:
            do_show = show if show is not None else self._frame_axis_enabled
            if not do_show:
                self._frame_axis_enabled = False
                self._map_origin_axis.disable()
                self._base_link_axis.disable()
                for axis in self._stream_axis:
                    axis.disable()
                # Camera FOV frustums follow the axis markers.
                for stream in self._model._stream_models():
                    self._set_frustum_visible(stream, False)
            else:
                self._frame_axis_enabled = True
                self._map_origin_axis.enable()
                self._base_link_axis.enable()
                for axis, stream in zip(self._stream_axis, self._model._stream_models()):
                    if stream._enabled:
                        axis.enable()
                # Camera FOV frustums follow the axis markers (only for
                # currently-enabled streams).
                for stream in self._model._stream_models():
                    self._set_frustum_visible(stream, stream._enabled)
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
            frame = (self._frame_set[self._tracked_sensor]
                     if len(self._frame_set) > self._tracked_sensor else None)
            if isinstance(frame, LidarFrame):
                self._frame_pose_history.append(core.last_valid_column_pose(frame))
            elif any(f is not None for f in self._camera_frames()):
                # camera-only: follow the camera body pose
                self._frame_pose_history.append(self._get_pose_with_min_ts())
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
        [s._imu_plot.show() for s in self._model._stream_models()]

    def hide_all_imu_plots(self):
        [s._imu_plot.hide() for s in self._model._stream_models()]

    def show_all_sensor_labels(self):
        self.show_all_stream_labels()

    def hide_all_sensor_labels(self):
        self.hide_all_stream_labels()

    def show_all_stream_labels(self):
        for stream in self._model._stream_models():
            stream._sensor_label.show()

    def hide_all_stream_labels(self):
        for stream in self._model._stream_models():
            stream._sensor_label.hide()

    def text_scale(self):
        return self._osd_scale / 2 + 0.5

    def _update_multi_viz_osd(self) -> None:
        text_scale = self.text_scale()
        self._osd.set_scale(text_scale)

        # handle OSD modes "NONE" and "HELP"
        if self._osd_state == LidarFrameViz.OsdState.NONE:
            self.hide_all_stream_labels()
            self.hide_all_imu_plots()
            self._osd.set_text("")
            return
        elif self._osd_state == LidarFrameViz.OsdState.HELP:
            self.hide_all_stream_labels()
            self.hide_all_imu_plots()
            on_screen_help_text = []
            for key_binding in self._key_definitions:
                on_screen_help_text.append(f"{key_binding:^7}: {self._key_definitions[key_binding]}")
            self._osd.set_text('\n'.join(on_screen_help_text))
            return

        self.show_all_stream_labels()
        self.show_all_imu_plots()

        # create the OSD text
        osd_str = ""
        num_clouds = len(self._model._cloud_present)
        cloud_idxs_str = str([i + 1 for i in range(num_clouds)])
        cloud_states_str = ", ".join(
            ["ON" if self._model._cloud_enabled[idx] else "OFF" for idx in range(num_clouds)])

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

        if not _frame_set_has_content(self._frame_set):
            self._osd.set_text(osd_str)
            return

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

        streams = self._model._stream_models()
        # Frames aligned to _stream_models() order, so lidar and camera streams
        # may be interleaved in any registration order.
        stream_frames = self._model._stream_frames(self._frame_set)

        # add stream labels and IMU plots from bottom to top
        for idx in range(len(streams)):
            rev_idx = len(streams) - idx - 1
            stream = streams[rev_idx]
            frame = stream_frames[rev_idx] if rev_idx < len(stream_frames) else None
            img_bottom = -(1 - label_height_accum) * 2 + 1 + (img_height_accum * 2)
            # Both lidar and camera streams may carry IMU data and get a plot.
            if stream._has_imu_data:
                stream._imu_plot.show()
                img_height_accum += img_height
            else:
                stream._imu_plot.hide()
            img_left = 75 * text_scale / view_h
            img_top = img_bottom + img_height * 2
            stream._imu_plot.hshift = -1
            stream._imu_plot.position = (img_left, img_width - img_left, img_bottom, img_top)

            label_bottom = 1 - label_height_accum - img_height_accum
            stream._sensor_label.position = (0.0, label_bottom)
            stream._sensor_label.scale = text_scale

            # relative display timestamp, computed once here so the streams
            # don't need to know about playback state
            valid_ts: Optional[float] = None
            if frame is not None:
                ts = _valid_or_zero_ts(frame) * 1e-9
                if self._first_frame_ts is not None and ts > 0:
                    valid_ts = ts - self._first_frame_ts
            ctrl_index = rev_idx + 1 if rev_idx < LidarFrameViz.MAX_SENSOR_TOGGLE_KEYS else None
            label_str, rgba = stream.osd_label(
                ctrl_index, frame,
                cloud_mode_name=self._model._cloud_mode_name,
                valid_ts=valid_ts)
            stream._sensor_label.text = label_str
            stream._sensor_label.rgba = rgba

            label_height_accum += stream._sensor_label.text_height / view_h

    def _camera_frames(self) -> List:
        """Camera frames from the unified FrameSet, one entry per known camera.

        Uses each camera's recorded FrameSet position, so lidar and camera
        streams may be interleaved in any order. Returns None for a camera whose
        frame is missing.
        """
        if not isinstance(self._frame_set, FrameSet):
            return []
        return [self._frame_set[i] if i < len(self._frame_set) else None
                for i in self._model._camera_frame_indices]

    def _get_pose_with_min_ts(self) -> np.ndarray:
        lidar_frames = [s for s in self._frame_set if isinstance(s, LidarFrame)]
        if lidar_frames:
            first_frame = min(lidar_frames, key=lambda s: _valid_or_zero_ts(s))
            return core.last_valid_column_pose(first_frame)
        # camera-only: use the first available camera body pose
        for frame in self._camera_frames():
            if frame is not None:
                return np.asarray(frame.body_to_world[0], dtype=np.float64)
        return np.eye(4)

    def _interpolate_camera_pose(self):
        if not self._frame_pose_history:
            if not _frame_set_has_content(self._frame_set):
                return np.eye(4)
            if len(self._frame_set) > self._tracked_sensor:
                frame = self._frame_set[self._tracked_sensor]
                if isinstance(frame, LidarFrame):
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
        if not _frame_set_has_content(self._frame_set):
            return

        # frame with the minimal timestamp determines the
        # center of the system (by its frame pose)
        pose = self._get_pose_with_min_ts()
        self._base_link_axis.pose = pose
        # Each stream computes its own axis pose for the current frame: lidars
        # use the base pose composed with their extrinsics, cameras project
        # their own body pose into the world. ``None`` leaves the axis unchanged.
        stream_frames = self._model._stream_frames(self._frame_set)
        for idx, stream in enumerate(self._model._stream_models()):
            frame = stream_frames[idx] if idx < len(stream_frames) else None
            new_pose = stream.scene_axis_pose(pose, frame)
            if new_pose is not None:
                self._stream_axis[idx].pose = new_pose

    def _draw_update_scene_poses(self) -> None:
        """Update axis poses from the current frames."""
        with self._lock:
            self._apply_scene_poses()

    def _draw_update_camera_pose(self, update_camera_history: bool = True) -> None:
        """Apply poses from the frames to the scene"""
        with self._lock:
            if not _frame_set_has_content(self._frame_set):
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
            elif self._camera_mode == LidarFrameViz.CameraMode.FOLLOW_WORLD_ROTATION_LOCKED:
                # only change the position and keep rotation aligned with the world
                self._camera_pose[:3, 3] = frame_pose[:3, 3]
                self._camera_pose[:3, :3] = np.eye(3)
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
            elif self._camera_mode == LidarFrameViz.CameraMode.FOLLOW_SMOOTH_WORLD_ROTATION_LOCK:
                avg_t = np.mean([p[:3, 3] for p in self._camera_pose_history], axis=0)
                self._camera_pose[:3, 3] = avg_t
                self._camera_pose[:3, :3] = np.eye(3)

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
        if len(self._frame_set) <= self._tracked_sensor:
            return

        frame = self._frame_set[self._tracked_sensor]
        if not isinstance(frame, LidarFrame):
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

    def set_zone_selection_mode(self, sel_mode: ZoneSelectionMode) -> None:
        with self._lock:
            self._zone_selection_mode = sel_mode
            self._viz.set_notification(
                f"Zone Selection Mode: {self._zone_selection_mode.name.replace('_', ' ')}")
            self.set_zone_selection_and_render_modes(self._zone_selection_mode, self._zone_render_mode)

    def set_zone_render_mode(self, render_mode: ZoneRenderMode) -> None:
        with self._lock:
            self._zone_render_mode = render_mode
            self._viz.set_notification(
                f"Zone Render Mode: {self._zone_render_mode.name.replace('_', ' ')}")
            self.set_zone_selection_and_render_modes(self._zone_selection_mode, self._zone_render_mode)

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
            if _frame_set_is_loop_sentinel(t):
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
    _images: List[Tuple[np.ndarray, float]] = []

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
                 clear_on_loop: bool = False, subframes = 0,
                 background_color: Tuple[float, float, float, float] = (0, 0, 0, 1),
                 camera_default_range_mm: int = DEFAULT_CAMERA_RANGE_MM) -> None:
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
            camera_default_range_mm: distance (mm) at which to draw the pixels
                  of a camera frame that has no RANGE field, as a forward
                  distance; 0 draws no cloud for such frames. Ignored with
                  ``_override_lidar_frame_viz``.
        Raises:
            ValueError: if the specified rate isn't one of the options
        """
        valid = ["exit", "stop", "loop"]
        if on_eof not in valid:
            raise ValueError(f"on_eof must be one of {valid}, was '{on_eof}'")

        self._metadata = metadata
        if not self._metadata:
            raise ValueError("SimpleViz requires at least one LidarInfo or CameraInfo")
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
        if isinstance(self._metadata, SensorInfo):
            self._metadata = [self._metadata]
        self._frame_viz = _override_lidar_frame_viz if _override_lidar_frame_viz \
            else LidarFrameViz(
                self._metadata,
                self._viz,
                accum_config,
                imu_viz_config,
                _buflen=_buflen,
                _add_default_controls=False,
                camera_default_range_mm=camera_default_range_mm,
                background_color=background_color
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
            (ord('/'), 1): SimpleViz.toggle_help,
            (ord('X'), 1): SimpleViz.toggle_img_recording,
            (ord('Z'), 1): SimpleViz.screenshot,
            (ord('V'), 0): partial(SimpleViz.cycle_screenshot_res_factor, direction=1),
            (ord('V'), 1): partial(SimpleViz.cycle_screenshot_res_factor, direction=-1),
            (ord('X'), 2): SimpleViz.toggle_video_format,
            (ord('H'), 0): partial(SimpleViz.adjust_subframes, direction=1),
            (ord('H'), 1): partial(SimpleViz.adjust_subframes, direction=-1)
        }

        # replace the scan viz on change with ours
        found = False
        for opt in self._frame_viz.config.options:
            if opt.name == "show_osd":
                weak_self = weakref.ref(self)

                def on_change(arg, val):
                    this = weak_self()
                    if this is not None:
                        this.toggle_osd(val)
                opt.on_change = on_change
                found = True
                break
        assert found

        key_definitions: Dict[str, str] = {
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
            self._frame_viz.toggle_osd(state)
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
            self._images.append((pix, frame_time))
        except PointVizNotRunningError:
            pass

    def _frame_period(self) -> float:
        if isinstance(self._frame_viz, LidarFrameViz):
            infos = (list(self._metadata)
                     if isinstance(self._metadata, collections.abc.Sequence)
                     else [self._metadata])
            # A sensor reports 0 when its metadata carries no frame rate, as a
            # camera recorded before fps was serialized does. Take the rate from
            # the first sensor that has one.
            for info in infos:
                if info.fps:
                    return 1.0 / info.fps
            return 1.0 / 30.0
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
                    if _frame_set_is_loop_sentinel(frame):
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
                            frame_ts = last_frame_time + self._frame_period()

                    # reset playback on loop
                    if looped:
                        last_time_update = time.monotonic()
                        last_sim_time = frame_ts - self._frame_period()
                        looped = False
                    frame_duration = (frame_ts - last_frame_time if last_frame_time
                                     else self._frame_period())
                    prev_frame_time: Optional[float] = last_frame_time
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

    def _save_gif(self, filename=None) -> None:
        if len(self._images) == 0:
            return

        if filename is None:
            filename = f"viz_gif_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.gif"

        print("Saving gif to " + filename)
        images = [image for image, _ in self._images]
        rate = SimpleViz._playback_rates[self._rate_ind]
        # live => rate of 1.0
        if rate <= 0.0:
            rate = 1.0

        # calculate durations in ms between frames
        default_dt = (self._frame_period() / (self._subframes + 1)
                      if self._subframes else self._frame_period())
        durations = []
        for i in range(1, len(self._images)):
            dt = self._images[i][1] - self._images[i - 1][1]
            if dt <= 0:
                dt = default_dt
            durations.append(dt * 1000 / rate)
        # assume last frame has same duration as the one before (or some default if single frame)
        durations.append(durations[-1] if len(durations) else 100)

        # save in a background thread to avoid freezing the viz
        def run_in_thread():
            # a GIF has a single size, so frames captured after a resolution
            # change are scaled to the size of the first frame
            height, width = images[0].shape[:2]
            try:
                with GifWriter(filename, width, height) as writer:
                    for image, duration in zip(images, durations):
                        if image.shape[:2] != (height, width):
                            image = _resize_nearest(image, height, width)
                        writer.add_frame(image, duration)
            except (RuntimeError, ValueError) as e:
                self._viz.set_notification("Failed Saving GIF")
                print(f"Failed Saving GIF: {e}")
                return
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


AnyFrames = Union[FrameSetSource, CameraFrame, LidarFrame, FrameSet,
                  List[Optional[Union[Frame, LidarFrame, CameraFrame]]],
                  List[List[Optional[LidarFrame]]]]


def lf_show(frames: AnyFrames,
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
        names = [str(si.sn) for si in sensor_infos if si.sn]
        return ",".join(names) if names else "camera"

    def get_sensor_infos(frames: Union[FrameSetSource, List[FrameSet]]) -> List[SensorInfo]:
        # Return the combined sensor infos (LidarInfo for lidars, CameraInfo
        # for cameras) using the unified accessors.
        if isinstance(frames, FrameSetSource):
            return list(frames.sensor_info)
        infos: List[SensorInfo] = []
        for s in frames[0]:
            if isinstance(s, (LidarFrame, CameraFrame)) and s.sensor_info is not None:
                infos.append(s.sensor_info)
        return infos

    def normalize(frames: AnyFrames
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

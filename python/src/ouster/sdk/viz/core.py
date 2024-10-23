"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Sensor data visualization tools.

Visualize lidar data using OpenGL.
"""

from collections import deque
from functools import partial
from enum import Enum, auto
import os
import threading
import time
from datetime import datetime
from typing import (Callable, ClassVar, cast, Deque, Dict, Iterable, Iterator, List,
                    Optional, Tuple, TypeVar, Union)
import logging

import numpy as np
from PIL import Image as PILImage

from ouster.sdk import client
from ouster.sdk.client import (first_valid_packet_ts, LidarScan, ChanField)
from ouster.sdk._bindings.viz import (PointViz, Cloud, Image, Cuboid, Label, WindowCtx, Camera,
                   TargetDisplay, add_default_controls)
from .util import push_point_viz_handler, push_point_viz_fb_handler
from .model import VizExtraMode
from . import util as vizu
from .view_mode import (ImageMode, CloudMode, LidarScanVizMode, CloudPaletteItem)
from .accumulators import LidarScanVizAccumulators
from .accumulators_config import LidarScanVizAccumulatorsConfig, MAP_MAX_POINTS_NUM, MAP_SELECT_RATIO
from .model import LidarScanVizModel


logger = logging.getLogger("viz-logger")

T = TypeVar('T')


class LidarScanViz:
    """Multi LidarScan clouds visualizer

    Uses the supplied PointViz instance to easily display the contents of a
    LidarScan. Sets up key bindings to toggle which channel fields and returns
    are displayed, and change 2D image and point size.
    """
    class OsdState(Enum):
        NONE = auto(),
        DEFAULT = auto(),
        HELP = auto()

    class FlagsMode(Enum):
        NONE = 0
        HIGHLIGHT_SECOND = 1    # TODO[UN]: probably unneccessary
        HIGHLIGHT_BLOOM = 2
        HIDE_BLOOM = 3

    class ImagesLayout(Enum):
        HORIZONTAL = 0
        VERTICAL = 1

    def __init__(
            self,
            metas: List[client.SensorInfo],
            point_viz: Optional[PointViz] = None,
            accumulators_config: Optional[LidarScanVizAccumulatorsConfig] = None,
            *,
            _img_aspect_ratio: float = 0,
            _ext_modes: Optional[List[LidarScanVizMode]] = None,    # TODO[UN]: disabled for now
            _ext_palettes: Optional[List[CloudPaletteItem]] = None) -> None:
        """
        Args:
            metas: sensor metadata used to interpret scans
            viz: use an existing PointViz instance instead of creating one
        """

        # used to synchronize key handlers and _draw()
        self._lock = threading.Lock()
        self._model = LidarScanVizModel(metas, _img_aspect_ratio=_img_aspect_ratio)
        self.update_on_input = True

        # extension point for the OSD text, inserts before the "axes" line
        self._osd_text_extra: Callable[[], str] = lambda: ""

        # set initial image sizes
        self._img_size_fraction = 4
        self._image_size_initialized = False

        # initial point size in scan clouds
        self._viz = point_viz or PointViz("Ouster Viz")

        self._scans_accum: Optional[LidarScanVizAccumulators] = None
        if accumulators_config:
            self._scans_accum = LidarScanVizAccumulators(
                self._model,
                self._viz,
                accumulators_config,
                self._lock
            )

            # TODO[tws]: decide the fate of ScansAccumulator's OSD
            self._scans_accum.toggle_osd(False)
            if hasattr(self, "_osd_text_extra"):
                self._osd_text_extra = self._scans_accum.osd_text
                # if we add the extra text to the _scan_viz we also want to
                # draw(w/o update) its state when scans accum keys cause the
                # re-draw of ScansAccumulator state (osd included)
                self._scans_accum._key_press_pre_draw = (
                    lambda: self.draw(update=False))

            if hasattr(self, "_key_definitions") and self._scans_accum:
                self._key_definitions.update(
                    getattr(self._scans_accum, "_key_definitions", {}))

        # add images and clouds to viz
        for sensor in self._model._sensors:
            [self._viz.add(image) for image in sensor._images]
            [self._viz.add(cloud) for cloud in sensor._clouds]

        self._images_layout = LidarScanViz.ImagesLayout.HORIZONTAL

        # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
        # initialize masks to just green with zero opacity
        self._flags_mode = LidarScanViz.FlagsMode.NONE

        # initialize rings
        self._ring_size = 1
        self._ring_line_width = 2
        self._viz.target_display.set_ring_line_width(self._ring_line_width)
        self._viz.target_display.set_ring_size(self._ring_size)
        self._viz.target_display.enable_rings(True)

        # initialize osd
        self._osd_state = LidarScanViz.OsdState.DEFAULT
        self._previous_osd_state = LidarScanViz.OsdState.NONE
        self._osd = Label("", 0, 1)
        self._viz.add(self._osd)

        # initialize scan axis helpers
        self._scan_axis_enabled = True
        self._scan_axis = []
        # sensors axis
        for idx, sensor in enumerate(self._model._sensors):
            self._scan_axis.append(
                vizu.AxisWithLabel(self._viz,
                                   pose=sensor._meta.extrinsic,
                                   label=str(idx + 1),
                                   thickness=3))
        # system center axis
        self._scan_axis_origin = vizu.AxisWithLabel(self._viz,
                                                    label="O",
                                                    thickness=5,
                                                    label_scale=0.4)

        self._camera_follow_enabled = True
        # TODO[UN]: assign a key to select a sensor to be tracked
        # also what to do in case tracked sensor is hidden?
        self._tracked_sensor = 0
        self._scan_pose = np.eye(4)

        self._scans: List[Optional[LidarScan]] = []
        self._scan_num = -1
        self._first_frame_ts = None

        self._setup_controls()

    # TODO[tws] likely remove
    @property
    def metadata(self) -> List[client.SensorInfo]:
        """Metadatas for the displayed sensors."""
        return self._model._metas

    @property
    def osd_state(self) -> "LidarScanViz.OsdState":
        """Returns the state of the on screen display."""
        return self._osd_state

    def _setup_controls(self) -> None:
        # key bindings. will be called from rendering thread, must be synchronized
        key_bindings: Dict[Tuple[int, int], Callable[[LidarScanViz], None]] = {
            (ord('E'), 0): partial(LidarScanViz.update_image_size, amount=+1),
            (ord('E'), 1): partial(LidarScanViz.update_image_size, amount=-1),
            (ord('P'), 0): partial(LidarScanViz.update_point_size, amount=+1),
            (ord('P'), 1): partial(LidarScanViz.update_point_size, amount=-1),
            (ord('1'), 0): partial(LidarScanViz.toggle_cloud, i=0),
            (ord('2'), 0): partial(LidarScanViz.toggle_cloud, i=1),
            (ord('B'), 0): partial(LidarScanViz.cycle_img_mode, i=0, direction=+1),
            (ord('B'), 1): partial(LidarScanViz.cycle_img_mode, i=0, direction=-1),
            (ord('N'), 0): partial(LidarScanViz.cycle_img_mode, i=1, direction=+1),
            (ord('N'), 1): partial(LidarScanViz.cycle_img_mode, i=1, direction=-1),
            (ord('M'), 0): partial(LidarScanViz.cycle_cloud_mode, direction=+1),
            (ord('M'), 1): partial(LidarScanViz.cycle_cloud_mode, direction=-1),
            (ord('F'), 0): partial(LidarScanViz.cycle_cloud_palette, direction=+1),
            (ord('F'), 1): partial(LidarScanViz.cycle_cloud_palette, direction=-1),
            (ord("'"), 0): partial(LidarScanViz.update_ring_size, amount=+1),
            (ord("'"), 1): partial(LidarScanViz.update_ring_size, amount=-1),
            (ord("'"), 2): LidarScanViz.cicle_ring_line_width,
            (ord("O"), 0): LidarScanViz.toggle_osd,
            (ord('U'), 0): LidarScanViz.toggle_camera_follow,
            # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
            (ord('C'), 0): LidarScanViz.update_flags_mode,
            (ord('9'), 0): LidarScanViz.toggle_scan_axis,
            (ord('/'), 1): LidarScanViz.toggle_help,
        }

        self._key_definitions: Dict[str, str] = {
            'w': "Camera pitch down",
            's': "Camera pitch up",
            'a': "Camera yaw right",
            'd': "Camera yaw left",
            "e / E": "Increase/decrease size of displayed 2D images",
            "p / P": "Increase/decrease point size",
            "R": "Reset camera orientation",
            "CTRL+r": "Camera bird-eye view",
            "0": "Toggle orthographic camera",
            "1": "Toggle first return point cloud",
            "2": "Toggle second return point cloud",
            "b": "Cycle top 2D image",
            "n": "Cycle bottom 2D image",
            'm': "Cycle through point cloud coloring mode",
            'f': "Cycle through point cloud color palette",
            'c': "Cycle current highlight mode",
            'u': "Toggle camera mode FOLLOW/FIXED",
            "9": "Toggle axis helpers at scan origin",
            '?': "Print keys to standard out",
            "= / -": "Dolly in and out",
            "' / \"": "Increase/decrease spacing in range markers",
            "CTRL+'": "Cycle through thickness of range markers or hide",
            'SHIFT': "Camera Translation with mouse drag",
            'ESC': "Exit the application",
        }

        # dynamically assign sensor toggle keys
        # TODO[tws]: there should be a sane limit
        for sensor_index in range(len(self._model._sensors)):
            key_bindings[(ord(str(sensor_index + 1)), 2)] = \
                partial(LidarScanViz.toggle_sensor, sensor_index=sensor_index)
            self._key_definitions[f"CTRL+{str(sensor_index + 1)}"] = f"Toggle sensor {sensor_index + 1} point cloud(s)"

        def handle_keys(self: LidarScanViz, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            if (key, mods) in key_bindings:
                key_bindings[key, mods](self)
                self.draw(update=self.update_on_input)
            return True

        push_point_viz_handler(self._viz, self, handle_keys)
        if isinstance(self._viz, PointViz):
            add_default_controls(self._viz)
        print("Press \'?\' while viz window is focused to print key bindings")

    def cycle_img_mode(self, i: int, *, direction: int = 1) -> None:
        """Change the displayed field of the i'th image."""
        with self._lock:
            self._model.cycle_image_mode(i, direction)

            # Note, updating the image mode needs the current scan
            # because ImageMode.enabled requires it.
            self._model.update(self._scans)

    def cycle_cloud_mode(self, direction: int = 1) -> None:
        """Change the coloring mode of the 3D point cloud."""
        with self._lock:
            self._model.cycle_cloud_mode(direction)

            # Note, updating the cloud mode needs the current scan
            # because CloudMode.enabled requires it.
            self._model.update(self._scans)

    def cycle_cloud_palette(self, *, direction: int = 1) -> None:
        """Change the color palette of the 3D point cloud."""
        with self._lock:
            # move to LidarScanVizModel
            self._model._palettes.cycle_cloud_palette(direction)
            self._model.update_cloud_palettes()

    def toggle_cloud(self, i: int) -> None:
        """Toggle whether the i'th return is displayed."""
        with self._lock:
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
                    if self._model._cloud_enabled[i]:
                        self._viz.add(cld)
                for img in sensor._images:
                    self._viz.add(img)
                self._scan_axis[sensor_index].enable()
            else:
                for cld in sensor._clouds:
                    self._viz.remove(cld)
                for img in sensor._images:
                    self._viz.remove(img)
                self._scan_axis[sensor_index].disable()
            if self._scans_accum:
                self._scans_accum.toggle_sensor(sensor_index, sensor._enabled)
        self.update_image_size(0)

    def update_point_size(self, amount: int) -> None:
        """Change the point size of the 3D cloud."""
        with self._lock:
            # TODO refactor add to model
            self._model._cloud_pt_size = min(10.0,
                                      max(1.0, self._model._cloud_pt_size + amount))
            self._model._set_cloud_pt_size(self._model._cloud_pt_size)

    def update_image_size(self, amount: int) -> None:
        """Change the size of the 2D image and position image labels."""
        with self._lock:
            size_fraction_max = 20
            self._img_size_fraction = (self._img_size_fraction + amount +
                                       (size_fraction_max + 1)) % (
                                           size_fraction_max + 1)
        enabled_sensors = [sensor for sensor in self._model._sensors if sensor._enabled]
        image_h = self._img_size_fraction / size_fraction_max

        # compute the total width
        total_image_w = 0.0
        for sensor in enabled_sensors:
            image_w = image_h / sensor._img_aspect_ratio
            total_image_w += image_w

        # set image positions
        center_x = -total_image_w / 2
        last_image_right = 0.0
        for sensor in enabled_sensors:
            image_w = image_h / sensor._img_aspect_ratio
            image_left = last_image_right
            image_right = last_image_right + image_w
            last_image_right = image_right
            for image_idx, image in enumerate(sensor._images):
                image_top = 1.0 - image_h * image_idx
                image_bottom = 1.0 - image_h * (image_idx + 1)
                image.set_position(image_left + center_x, image_right + center_x, image_bottom, image_top)

    def update_ring_size(self, amount: int) -> None:
        """Change distance ring size."""
        with self._lock:
            self._ring_size = min(3, max(-2, self._ring_size + amount))
            self._viz.target_display.set_ring_size(self._ring_size)

    def cicle_ring_line_width(self) -> None:
        """Change rings line width."""
        with self._lock:
            self._ring_line_width = max(0, (self._ring_line_width + 1) % 10)
            self._viz.target_display.set_ring_line_width(self._ring_line_width)
            self._viz.target_display.enable_rings(self._ring_line_width != 0)

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._lock:
            if self._osd_state != LidarScanViz.OsdState.DEFAULT:
                self._osd_state = LidarScanViz.OsdState.DEFAULT
            else:
                self._osd_state = LidarScanViz.OsdState.NONE

    def toggle_camera_follow(self) -> None:
        """Toggle the camera follow mode."""
        with self._lock:
            self._camera_follow_enabled = not self._camera_follow_enabled

    # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
    def update_flags_mode(self,
                          mode: 'Optional[LidarScanViz.FlagsMode]' = None) -> None:
        with self._lock:
            # cycle between flag mode enum values
            if mode is None:
                self._flags_mode = LidarScanViz.FlagsMode(
                    (self._flags_mode.value + 1) %
                    len(LidarScanViz.FlagsMode.__members__))
            else:
                self._flags_mode = mode

            # if no scan is set yet, skip updating the cloud masks
            if not self._scans:
                return

            # reset the cloud range field (since HIDE_BLOOM will modify it)
            for scan, sensor in zip(self._scans, self._model._sensors):
                if not scan:
                    continue
                for i, range_field in ((0, ChanField.RANGE),
                                       (1, ChanField.RANGE2)):
                    if range_field in scan.fields:
                        sensor._clouds[i].set_range(scan.field(range_field))

            # TODO[UN]: need to be done as part of the combined lidar mode
            # set mask on all points in the second cloud, clear other mask
            for i, sensor in enumerate(self._model._sensors):
                if self._flags_mode == LidarScanViz.FlagsMode.HIGHLIGHT_SECOND:
                    sensor._cloud_masks[0][:, :, 3] = 0.0
                    sensor._cloud_masks[1][:, :, 3] = 1.0
                # clear masks on both clouds, expected to be set dynamically in _draw()
                else:
                    sensor._cloud_masks[0][:, :, 3] = 0.0
                    sensor._cloud_masks[1][:, :, 3] = 0.0
                sensor._clouds[0].set_mask(sensor._cloud_masks[0])
                sensor._clouds[1].set_mask(sensor._cloud_masks[1])

    def _draw_update_flags_mode(self) -> None:
        """Apply selected FlagsMode to the cloud"""
        # set a cloud mask based where first flags bit is set
        for scan, sensor in zip(self._scans, self._model._sensors):
            if not scan:
                continue
            if self._flags_mode == LidarScanViz.FlagsMode.HIGHLIGHT_BLOOM:
                for i, flag_field in ((0, ChanField.FLAGS), (1, ChanField.FLAGS2)):
                    if flag_field in scan.fields:
                        mask_opacity = (scan.field(flag_field) & 0x1) * 1.0
                        sensor._cloud_masks[i][:, :, 3] = mask_opacity
                        sensor._clouds[i].set_mask(sensor._cloud_masks[i])
            # set range to zero where first flags bit is set
            elif self._flags_mode == LidarScanViz.FlagsMode.HIDE_BLOOM:
                for i, flag_field, range_field in ((0, ChanField.FLAGS,
                                                    ChanField.RANGE),
                                                   (1, ChanField.FLAGS2,
                                                    ChanField.RANGE2)):
                    if flag_field in scan.fields and range_field in scan.fields:
                        # modifying the scan in-place would break cycling modes while paused
                        rng = scan.field(range_field).copy()
                        rng[scan.field(flag_field) & 0x1 == 0x1] = 0
                        sensor._clouds[i].set_range(rng)

    def toggle_scan_axis(self) -> None:
        """Toggle the helper axis of a scan ON/OFF"""
        with self._lock:
            if self._scan_axis_enabled:
                self._scan_axis_enabled = False
                self._scan_axis_origin.disable()
                for axis in self._scan_axis:
                    axis.disable()
            else:
                self._scan_axis_enabled = True
                self._scan_axis_origin.enable()
                for axis, sensor in zip(self._scan_axis, self._model._sensors):
                    if sensor._enabled:
                        axis.enable()

    @property
    # NOTE[BREAKING]
    def scan(self) -> List[Optional[LidarScan]]:
        """The currently displayed scan."""
        return self._scans

    @property
    def scan_num(self) -> int:
        """The currently displayed scan number"""
        return self._scan_num

    def update(self,
               scans: List[Optional[LidarScan]],
               scan_num: Optional[int] = None) -> None:
        """Update the LidarScanViz state with the provided scans."""
        self._scans = scans
        if scan_num is not None:
            self._scan_num = scan_num
        else:
            self._scan_num += 1

        self._model.update(self._scans)
        if self._scans_accum:
            self._scans_accum.update(scans, self._scan_num)

    def draw(self, update: bool = True) -> None:
        """Process and draw the latest state to the screen."""
        with self._lock:
            self._draw()
            if self._scans_accum:
                self._scans_accum._draw()

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
    def _draw(self) -> None:
        self._draw_update_scan_poses()
        self._draw_update_flags_mode()
        self._update_multi_viz_osd()

    @staticmethod
    def _format_version(version: client.Version) -> str:
        result = f'v{version.major}.{version.minor}.{version.patch}'
        if version.prerelease:
            result += f'-{version.prerelease}'
        return result

    def _update_multi_viz_osd(self):
        if self._osd_state == LidarScanViz.OsdState.NONE:
            self._osd.set_text("")
            return
        elif self._osd_state == LidarScanViz.OsdState.HELP:
            on_screen_help_text = []
            for key_binding in self._key_definitions:
                on_screen_help_text.append(f"{key_binding:^7}: {self._key_definitions[key_binding]}")
            self._osd.set_text('\n'.join(on_screen_help_text))
            return

        enabled_clouds_str = ""
        for idx, sensor in enumerate(self._model._sensors):
            meta = sensor._meta
            cloud_state = "ON" if sensor._enabled else "OFF"
            enabled_clouds_str += f"sensor [CTRL+{idx + 1}]: {cloud_state}"
            scan = self._scans[idx]
            if scan:
                enabled_clouds_str += f" frame id: {scan.frame_id}\n"
            else:
                enabled_clouds_str += "\n"
            profile_str = str(meta.format.udp_profile_lidar).replace('_', '..')
            version_str = LidarScanViz._format_version(meta.get_version())
            enabled_clouds_str += f"        {profile_str} {meta.prod_line} {version_str} {meta.config.lidar_mode}\n"

        osd_str = f"{enabled_clouds_str}"

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
            f"        flags mode[c]: {self._flags_mode.name}\n" \
            f"        point size [P]: {int(self._model._cloud_pt_size)}\n"

        osd_str_extra = self._osd_text_extra()
        if osd_str_extra:
            osd_str_extra += "\n"

        frame_ts = min([first_valid_packet_ts(s) for s in self._scans if s]) * 1e-9
        if self._first_frame_ts is None:
            self._first_frame_ts = frame_ts
        frame_ts -= self._first_frame_ts    # show relative time
        osd_str += f"{osd_str_extra}" \
            f"axes [9]: {'ON' if self._scan_axis_enabled else 'OFF'}\n" \
            f"camera mode [U]: {'FOLLOW' if self._camera_follow_enabled else 'FIXED'}\n" \
            f"frame # : {self._scan_num}, frame ts: {frame_ts:0.3f} s"
        self._osd.set_text(osd_str)

    def _get_pose_with_min_ts(self) -> np.ndarray:
        first_scan = min([s for s in self._scans if s], key=lambda s: first_valid_packet_ts(s))
        return client.first_valid_column_pose(first_scan)

    def _draw_update_scan_poses(self) -> None:
        """Apply poses from the Scans to the scene"""
        # handle Axis and Camera poses
        # scan with the minimal timestamp determines the
        # center of the system (by it's scan pose)
        pose = self._get_pose_with_min_ts()
        self._viz.camera.set_target(np.linalg.inv(pose))
        self._scan_axis_origin.pose = pose
        # update all sensor axis positions
        for axis, sensor in zip(self._scan_axis, self._model._sensors):
            axis.pose = pose @ sensor._meta.extrinsic

        if self._camera_follow_enabled:
            scan = self._scans[self._tracked_sensor]
            if scan:    # if picked scan is None don't update camera
                self._scan_pose = client.first_valid_column_pose(scan)
        self._viz.camera.set_target(np.linalg.inv(self._scan_pose))

    def print_key_bindings(self) -> None:
        print(">---------------- Key Bindings --------------<")
        for key_binding in self._key_definitions:
            print(f"{key_binding:^7}: {self._key_definitions[key_binding]}")
        print(">--------------------------------------------<")

    def toggle_help(self) -> None:
        with self._lock:
            if self._osd_state != LidarScanViz.OsdState.HELP:
                self._previous_osd_state = self._osd_state
                self._osd_state = LidarScanViz.OsdState.HELP
                self.print_key_bindings()
            else:
                self._osd_state = self._previous_osd_state


def _first_scan_ts(scans: Union[List[Optional[LidarScan]], LidarScan]):
    scans = [scans] if isinstance(scans, client.LidarScan) else scans  # TODO[tws] fix these banes of our existence
    for scan in scans:
        if scan:
            return client.first_valid_column_ts(scan)


class _Seekable:
    """Wrap an iterable to support seeking by index.

    Similar to `more_itertools.seekable` but keeps indexes stable even values
    are evicted from the cache.

    The :meth:`seek` and :meth:`__next__` methods maintain the invariant:

        (read_ind - len(cache)) < next_ind <= read_ind + 1
    """

    # FIXME[tws] somehow simplify typing
    def __init__(self, it: Union[Iterable[List[Optional[LidarScan]]], Iterable[LidarScan]], maxlen=50) -> None:
        self._next_ind = 0  # index of next value to be returned
        self._read_ind = -1  # index of most recent (leftmost) value in cache
        self._iterable = it
        self._it: Union[Iterator[List[Optional[LidarScan]]], Iterator[LidarScan]] = \
            cast(Union[Iterator[List[Optional[LidarScan]]], Iterator[LidarScan]], iter(it))
        self._maxlen = maxlen
        self._cache: Deque[Union[List[Optional[LidarScan]], LidarScan]] = deque([], maxlen)
        self._first_scan_ts = None

    def __iter__(self):
        return self

    def __next__(self) -> Union[List[Optional[LidarScan]], LidarScan]:
        # next value already read, is in cache
        t: Union[List[Optional[LidarScan]], LidarScan]
        if self._next_ind <= self._read_ind:
            t = self._cache[self._read_ind - self._next_ind]
            self._next_ind += 1
            return t
        # next value comes from iterator
        elif self._next_ind == self._read_ind + 1:
            t = cast(Union[List[Optional[LidarScan]], LidarScan], next(self._it))
            self._cache.appendleft(t)
            if len(self._cache) > self._maxlen:
                self._cache.pop()

            self._next_ind += 1
            self._read_ind += 1

            # TODO[tws] improve loop handling.
            # This is a massive kludge to handle looping the source with a _Seekable, which was supposed to be a
            # generic. A better solution will probably rely on a better conceptulization of ScanSource/MultiScanSource
            # as Iterables, where the user (SimpleViz, in this case,) waits for StopIteration and resets the source.
            if self._first_scan_ts:
                if self._first_scan_ts == _first_scan_ts(t):
                    self._next_ind = 1
                    self._read_ind = 0
            else:
                self._first_scan_ts = _first_scan_ts(t)
            return t
        else:
            raise AssertionError("Violated: next_ind <= read_ind + 1")

    @property
    def scan_num(self) -> int:
        """Returns the most-recently read scan number, starting at zero."""
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


def _save_fb_to_png(fb_data: List,
                    fb_width: int,
                    fb_height: int,
                    action_name: Optional[str] = "screenshot",
                    file_path: Optional[str] = None):
    img_arr = np.array(fb_data,
                       dtype=np.uint8).reshape([fb_height, fb_width, 3])
    img_fname = datetime.now().strftime(
        f"viz_{action_name}_%Y%m%d_%H%M%S.%f")[:-3] + ".png"
    if file_path:
        img_fname = os.path.join(file_path, img_fname)
    PILImage.fromarray(np.flip(img_arr, axis=0)).convert("RGB").save(img_fname)
    return img_fname


class SimpleViz:
    """Visualize a stream of LidarScans.

    Handles controls for playback speed, pausing and stepping."""

    _playback_rates: ClassVar[Tuple[float, ...]]
    _playback_rates = (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 0.0)

    def __init__(self,
                 metadata: Union[List[client.SensorInfo], client.SensorInfo],
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
                 _override_pointviz: Optional[PointViz] = None,
                 _override_lidarscanviz: Optional[LidarScanViz] = None,
                 _buflen: int = 50) -> None:
        """
        Args:
            arg: Metadata associated with the scans to be visualized or a
                 LidarScanViz instance to use.
            rate: Playback rate. One of 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0 or
                  None for "live" playback (the default).
            pause_at: scan number to pause at, default (-1) - no auto pause, to
                      stop after the very first scan use 0

        Raises:
            ValueError: if the specified rate isn't one of the options
        """
        self._metadata = [metadata] if isinstance(metadata, client.SensorInfo) else metadata
        self._viz = _override_pointviz if _override_pointviz else PointViz("Ouster Viz")
        accum_config = LidarScanVizAccumulatorsConfig(
            accum_max_num=accum_max_num,
            accum_min_dist_meters=accum_min_dist_meters,
            accum_min_dist_num=accum_min_dist_num,
            map_enabled=map_enabled,
            map_select_ratio=map_select_ratio,
            map_max_points=map_max_points
        )
        self._scan_viz = _override_lidarscanviz if _override_lidarscanviz \
            else LidarScanViz(self._metadata, self._viz, accum_config)

        self._lock = threading.Lock()
        self._live = (rate is None)
        self._rate_ind = SimpleViz._playback_rates.index(rate or 0.0)
        self._buflen = _buflen
        self._pause_at = pause_at
        self._on_eof = on_eof
        self._last_draw_period = 0.0

        # pausing and stepping
        self._cv = threading.Condition()
        self._paused = False
        self._step = 0
        self._proc_exit = False

        # playback status display
        # TODO[tws] probably move playback osd to LidarScanViz...
        # We've had to define extra key handlers here to support it for no good reason.
        self._playback_osd = Label("", 1, 1, align_right=True)
        self._viz.add(self._playback_osd)
        self._update_playback_osd()

        # continuous screenshots recording
        self._viz_img_recording = False

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
        }

        key_definitions: Dict[str, str] = {
            'o': "Toggle information overlay",
            'SHIFT+x': "Toggle a continuous saving of screenshots",
            'SHIFT+z': "Take a screenshot!",
            ". / ,": "Step forward one frame",
            "> / <": "Increase/decrease playback rate (during replay)",
            'SPACE': "Pause and unpause",
        }
        if hasattr(self._scan_viz, "_key_definitions"):
            self._scan_viz._key_definitions.update(key_definitions)

        # only allow changing rate when not in "live" mode
        if not self._live:
            key_bindings.update({
                (ord(','), 1):
                partial(SimpleViz.modify_rate, amount=-1),
                (ord('.'), 1):
                partial(SimpleViz.modify_rate, amount=1),
            })

        def handle_keys(self: SimpleViz, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            if (key, mods) in key_bindings:
                prop = key_bindings[key, mods](self)
                # override rather than add bindings on no prop
                return False if prop is None else prop
            return True

        push_point_viz_handler(self._viz, self, handle_keys)

    def toggle_help(self) -> None:
        self._scan_viz.toggle_help()
        self._update_playback_osd()
        self._scan_viz.draw()

    def _update_playback_osd(self) -> None:
        if self._scan_viz.osd_state != LidarScanViz.OsdState.DEFAULT:
            self._playback_osd.set_text("")
            return

        if self._paused:
            playback_str = "playback: paused"
        elif self._live:
            playback_str = "playback: live"
        else:
            rate = SimpleViz._playback_rates[self._rate_ind]
            playback_str = f"playback: {str(rate) + 'x' if rate else 'max'}"

        fps_rates = f"sps: {self.scans_per_sec:.1f}   fps: {self._viz.fps:.1f}"
        playback_str = f"{fps_rates}   {playback_str}"

        self._playback_osd.set_text(f"{playback_str}")

    def toggle_pause(self) -> None:
        """Pause or unpause the visualization."""
        with self._cv:
            self._paused = not self._paused
            self._update_playback_osd()
            if not self._paused:
                self._cv.notify()

    def seek_relative(self, n_frames: int) -> None:
        """Seek forward of backwards in the stream."""
        with self._cv:
            self._paused = True
            self._step = n_frames
            self._update_playback_osd()
            self._cv.notify()

    def modify_rate(self, amount: int) -> None:
        """Switch between preset playback rates."""
        n_rates = len(SimpleViz._playback_rates)
        with self._cv:
            self._rate_ind = max(0, min(n_rates - 1, self._rate_ind + amount))
            self._update_playback_osd()

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._cv:
            self._scan_viz.toggle_osd()
            self._update_playback_osd()
            self._scan_viz.draw()

    def toggle_img_recording(self) -> None:
        if self._viz_img_recording:
            self._viz_img_recording = False
            self._viz.pop_frame_buffer_handler()
            print("Key SHIFT-X: Img Recording STOPPED")
        else:
            self._viz_img_recording = True

            def record_fb_imgs(fb_data: List, fb_width: int, fb_height: int):
                saved_img_path = _save_fb_to_png(fb_data,
                                                 fb_width,
                                                 fb_height,
                                                 action_name="recording")
                print(f"Saving recordings to: {saved_img_path}")
                # continue to other fb_handlers
                return True
            self._viz.push_frame_buffer_handler(record_fb_imgs)
            print("Key SHIFT-X: Img Recording STARTED")

    def screenshot(self, file_path: Optional[str] = None) -> None:
        def handle_fb_once(viz: PointViz, fb_data: List, fb_width: int,
                           fb_height: int):
            saved_img_path = _save_fb_to_png(fb_data,
                                             fb_width,
                                             fb_height,
                                             file_path=file_path)
            viz.pop_frame_buffer_handler()
            print(f"Saved screenshot to: {saved_img_path}")
        push_point_viz_fb_handler(self._viz, self._viz, handle_fb_once)

    def _lidar_frame_period(self) -> float:
        if isinstance(self._scan_viz, LidarScanViz):
            # TODO[UN]: per sensor frame period
            return 1.0 / (self._metadata[0].format.fps)
        else:
            # if some other scan viz that is not derived from LidarScanViz
            # we default to 10 Hz
            return 1.0 / 10.0

    def _get_timestamp(self, scan: List[Optional[LidarScan]]):
        return min([first_valid_packet_ts(s) for s in scan if s])

    def _process(self, seekable: _Seekable) -> None:
        scan_idx = -1

        # times for the last sim time since pause or skip
        # when we playback:
        # sim_time = (time.now() - last_time_update)*rate + last_sim_time
        last_sim_time = None
        last_time_update = None

        # packet time (or fake time) of last scan we received
        last_scan_time = None
        # used to detect changes in rate to reset the sim time origin to avoid glitches
        last_rate = None

        # monotonic time of last scan display, used to show scan display rate
        last_play_time = None

        try:
            while True:
                # wait until unpaused, step, or quit
                try:
                    with self._cv:
                        if self._paused:
                            self._cv.wait_for(lambda: not self._paused or self._step or
                                              self._proc_exit)
                            last_time_update = time.monotonic()
                            last_sim_time = last_scan_time
                        if self._proc_exit:
                            break
                        if self._step:
                            seek_ind = seekable.scan_num + self._step - 1
                            self._step = 0
                            if not seekable.seek(seek_ind):
                                continue

                    rate = SimpleViz._playback_rates[self._rate_ind]
                    if rate != last_rate:
                        last_rate = rate
                        last_time_update = time.monotonic()
                        last_sim_time = last_scan_time

                    scan = next(seekable)
                    scan_idx = seekable.scan_num
                    scan = [scan] if isinstance(scan, client.LidarScan) else scan
                    scan_ts = self._get_timestamp(scan) / 1e9

                    # fallback if we have no valid ts in the scan
                    if scan_ts == 0:
                        if last_scan_time is None:
                            scan_ts = 0
                        else:
                            scan_ts = last_scan_time + self._lidar_frame_period()

                    # detect loops
                    if last_scan_time is not None and last_scan_time > scan_ts:
                        last_time_update = time.monotonic()
                        last_sim_time = scan_ts - self._lidar_frame_period()
                    last_scan_time = scan_ts

                    if last_sim_time is None:
                        last_sim_time = scan_ts
                        last_time_update = time.monotonic()

                    # Queue up the scans to be viewed
                    self._scan_viz.update(scan, scan_idx)
                    self._scan_viz.draw(update=False)

                    if self._pause_at == scan_idx:
                        self._paused = True

                    self._update_playback_osd()

                    # Sleep until time to "play" this scan if necessary
                    sim_time = (time.monotonic() - last_time_update) * rate + last_sim_time
                    if rate != 0 and not self._paused and not self._live:
                        time_remaining = (scan_ts - sim_time) / rate
                        if time_remaining > 0:
                            # Dont update based on input while we are sleeping or we skip ahead
                            has_update_on_input = hasattr(self._scan_viz, "update_on_input")
                            if has_update_on_input:
                                self._scan_viz.update_on_input = False
                            time.sleep(time_remaining)
                            if has_update_on_input:
                                self._scan_viz.update_on_input = True

                    # Update scan rate display calculation
                    now_ts = time.monotonic()
                    last = last_play_time or now_ts
                    dt = now_ts - last
                    last_play_time = now_ts
                    self._last_draw_period = dt
                    self._viz.update()

                except StopIteration:
                    if not self._paused and not self._on_eof == "stop":
                        break

                    # Pause after we get a StopIteration in eof "stop"
                    if self._on_eof == "stop":
                        self._paused = True
                        self._update_playback_osd()
                        self._viz.update()

        finally:
            # signal rendering (main) thread to exit, with a delay
            # because the viz in main thread may not have been started
            # and on Mac it was observed that it fails to set a flag if
            # _process fails immediately after start
            time.sleep(0.5)
            self._viz.running(False)

    @property
    def scans_per_sec(self) -> float:
        """Scans per second processing rate."""
        if self._last_draw_period > 0:
            return 1.0 / self._last_draw_period
        else:
            return 0.0

    def run(self, scans: Iterable[client.LidarScan]) -> None:
        """Start reading scans and visualizing the stream.

        Must be called from the main thread on macOS. Will close the provided
        scan source before returning.

        Args:
            scans: A stream of scans to visualize.

        Returns:
            When the stream is consumed or the visualizer window is closed.
        """

        seekable = _Seekable(scans, maxlen=self._buflen)
        try:
            logger.info("Starting processing thread...")
            self._proc_exit = False
            proc_thread = threading.Thread(name="Viz processing",
                                           target=self._process,
                                           args=(seekable, ))
            proc_thread.start()

            logger.info("Starting rendering loop...")
            self._viz.run()
            logger.info("Done rendering loop")
        except KeyboardInterrupt:
            print("Termination requested, shutting down...")
        finally:
            # processing thread will still be running if e.g. viz window was closed
            with self._cv:
                self._proc_exit = True
                self._cv.notify()

            logger.info("Joining processing thread")
            proc_thread.join()


__all__ = [
    'PointViz', 'Cloud', 'Image', 'Cuboid', 'Label', 'WindowCtx', 'Camera',
    'TargetDisplay', 'add_default_controls', 'ImageMode',
    'CloudMode', 'CloudPaletteItem', 'VizExtraMode', 'LidarScanViz',
    'push_point_viz_handler',
]

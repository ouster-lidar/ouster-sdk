"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Sensor data visualization tools.

Visualize lidar data using OpenGL.
"""

from collections import deque
from dataclasses import dataclass
from functools import partial
from enum import Enum
import os
import threading
import time
from datetime import datetime
from typing import (Callable, ClassVar, Deque, Dict, Generic, Iterable, List,
                    Optional, Tuple, TypeVar, Union, Any)
import logging
import click

import numpy as np
from PIL import Image as PILImage

from ouster import client
from ouster.client import ChanField, ShotLimitingStatus, ThermalShutdownStatus
from ._viz import (PointViz, Cloud, Image, Cuboid, Label, WindowCtx, Camera,
                   TargetDisplay, add_default_controls, calref_palette,
                   spezia_palette, grey_palette, viridis_palette, magma_palette)
from .util import push_point_viz_handler, push_point_viz_fb_handler
from . import util as vizu

from .view_mode import (ImageMode, CloudMode, LidarScanVizMode, ReflMode,
                        SimpleMode, is_norm_reflectivity_mode, CloudPaletteItem)

from .scans_accum import ScansAccumulator

from ouster.sdkx.util import img_aspect_ratio  # type:ignore
import platform

logger = logging.getLogger("viz-logger")
client_log_location = None
if platform.system() == "Windows":
    client_log_dir = os.getenv("LOCALAPPDATA")

    if not client_log_dir:
        client_log_dir = os.getenv("TMP")
        if not client_log_dir:
            client_log_dir = "C:"
    client_log_dir = os.path.join(client_log_dir, "ouster-cli")
    client_log_location = os.path.join(client_log_dir, "ouster-sdk.log")

else:
    client_log_dir = os.path.join(os.path.expanduser("~"), ".ouster-cli")
    client_log_location = os.path.join(client_log_dir, "ouster-sdk.log")

logging_enabled = True
# TODO[pb]: Consider the dependency on `click`` here, and why we are using the
#           click.echo() and not prints for example?
if not os.path.exists(client_log_dir):
    try:
        os.makedirs(client_log_dir)
    except Exception as e:
        click.echo(f"Can't enable logging: {e}")
        logging_enabled = False
if not os.access(client_log_dir, os.W_OK):
    click.echo("Can't enable logging")
    logging_enabled = False
if logging_enabled:
    client.init_logger("info", client_log_location)


T = TypeVar('T')


@dataclass
class ImgModeItem:
    """Image mode for specific return with explicit name."""
    mode: ImageMode
    name: str
    return_num: int = 0


@dataclass
class VizExtraMode:
    """Image/Cloud mode factory func

    Used to embed viz modes from external plugins.
    """
    func: Callable[[], LidarScanVizMode]

    def create(self,
               info: Optional[client.SensorInfo] = None) -> LidarScanVizMode:
        extra_mode = self.func()
        if info and hasattr(extra_mode, "_info") and extra_mode._info is None:
            extra_mode._info = info
        return extra_mode


# Viz modes added externally
_viz_extra_modes: List[VizExtraMode]
_viz_extra_modes = []

# Viz palettes added externally
_viz_extra_palettes: List[CloudPaletteItem]
_viz_extra_palettes = []


class LidarScanViz:
    """Visualize LidarScan data.

    Uses the supplied PointViz instance to easily display the contents of a
    LidarScan. Sets up key bindings to toggle which channel fields and returns
    are displayed, and change 2D image and point size.
    """

    class FlagsMode(Enum):
        NONE = 0
        HIGHLIGHT_SECOND = 1
        HIGHLIGHT_BLOOM = 2
        HIDE_BLOOM = 3

    _cloud_palette: Optional[CloudPaletteItem]

    def __init__(
            self,
            meta: client.SensorInfo,
            viz: Optional[PointViz] = None,
            *,
            _img_aspect_ratio: float = 0,
            _ext_modes: Optional[List[LidarScanVizMode]] = None,
            _ext_palettes: Optional[List[CloudPaletteItem]] = None) -> None:
        """
        Args:
            meta: sensor metadata used to interpret scans
            viz: use an existing PointViz instance instead of creating one
        """

        # used to synchronize key handlers and _draw()
        self._lock = threading.Lock()

        # cloud display state
        self._cloud_mode_ind = 0  # index into _cloud_mode_channels
        self._cloud_enabled = [True, True]
        self._cloud_pt_size = 2.0

        self._cloud_refl_mode_prev = False
        self._cloud_refl_mode = False

        self._cloud_palettes: List[CloudPaletteItem]
        self._cloud_palettes = [
            CloudPaletteItem("Ouster Colors", spezia_palette),
            CloudPaletteItem("Greyscale", grey_palette),
            CloudPaletteItem("Viridis", viridis_palette),
            CloudPaletteItem("Magma", magma_palette),
            CloudPaletteItem("Cal. Ref", calref_palette),
        ]

        # Add extra color palettes, usually inserted through plugins
        self._cloud_palettes.extend(_viz_extra_palettes)

        self._cloud_palettes.extend(_ext_palettes or [])

        self._cloud_palette_ind = 0
        self._cloud_palette = self._cloud_palettes[self._cloud_palette_ind]
        self._cloud_palette_name = self._cloud_palette.name

        # image display state
        self._img_ind = [0, 1]  # index of field to display
        self._img_refl_mode = [False, False]
        self._img_size_fraction = 4
        self._img_aspect = _img_aspect_ratio or img_aspect_ratio(meta)

        # misc display state
        self._ring_size = 1
        self._ring_line_width = 2
        self._osd_enabled = True
        self._scan_poses_enabled = True
        self._camera_follow_enabled = True

        self._modes: List[LidarScanVizMode]
        self._modes = [
            ReflMode(info=meta),
            SimpleMode(ChanField.NEAR_IR, info=meta, use_ae=True, use_buc=True),
            SimpleMode(ChanField.SIGNAL, info=meta),
            SimpleMode(ChanField.RANGE, info=meta),
        ]

        # Add extra viz mode, usually inserted through plugins
        self._modes.extend([vm.create(meta) for vm in _viz_extra_modes])

        self._modes.extend(_ext_modes or [])

        self._image_modes: List[ImgModeItem]
        self._image_modes = [
            ImgModeItem(mode, name, num) for mode in self._modes
            if isinstance(mode, ImageMode)
            for num, name in enumerate(mode.names)
        ]

        self._cloud_modes: List[CloudMode]
        self._cloud_modes = [m for m in self._modes if isinstance(m, CloudMode)]

        # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
        # initialize masks to just green with zero opacity
        mask = np.zeros(
            (meta.format.pixels_per_column, meta.format.columns_per_frame, 4),
            dtype=np.float32)
        mask[:, :, 1] = 1.0
        self._cloud_masks = (mask, mask.copy())
        self._flags_mode = LidarScanViz.FlagsMode.NONE

        self._viz = viz or PointViz("Ouster Viz")

        # initialize scan's first and second return clouds
        self._metadata = meta
        self._clouds = (Cloud(meta), Cloud(meta))
        self._viz.add(self._clouds[0])
        self._viz.add(self._clouds[1])

        # initialize images
        self._images = (Image(), Image())
        self._viz.add(self._images[0])
        self._viz.add(self._images[1])
        self.update_image_size(0)

        # initialize rings
        self._viz.target_display.set_ring_line_width(self._ring_line_width)
        self._viz.target_display.set_ring_size(self._ring_size)
        self._viz.target_display.enable_rings(True)

        # initialize osd
        self._osd = Label("", 0, 1)
        self._viz.add(self._osd)

        # initialize scan axis helper
        self._scan_axis = vizu.AxisWithLabel(self._viz,
                                             thickness=3,
                                             length=1.0,
                                             enabled=False)

        self._scan_num = -1

        # scan identity poses to re-use
        self._scan_column_poses_identity = np.array(
            [np.eye(4) for _ in range(self._metadata.format.columns_per_frame)],
            dtype=np.float32)

        # extension point for the OSD text, inserts before the "axes" line
        self._osd_text_extra: Callable[[], str] = lambda: ""

        # key bindings. will be called from rendering thread, must be synchronized
        key_bindings: Dict[Tuple[int, int], Callable[[LidarScanViz], None]] = {
            (ord('E'), 0): partial(LidarScanViz.update_image_size, amount=1),
            (ord('E'), 1): partial(LidarScanViz.update_image_size, amount=-1),
            (ord('P'), 0): partial(LidarScanViz.update_point_size, amount=1),
            (ord('P'), 1): partial(LidarScanViz.update_point_size, amount=-1),
            (ord('1'), 0): partial(LidarScanViz.toggle_cloud, i=0),
            (ord('2'), 0): partial(LidarScanViz.toggle_cloud, i=1),
            (ord('B'), 0): partial(LidarScanViz.cycle_img_mode, i=0, direction=1),
            (ord('B'), 1): partial(LidarScanViz.cycle_img_mode, i=0, direction=-1),
            (ord('N'), 0): partial(LidarScanViz.cycle_img_mode, i=1, direction=1),
            (ord('N'), 1): partial(LidarScanViz.cycle_img_mode, i=1, direction=-1),
            (ord('M'), 0): partial(LidarScanViz.cycle_cloud_mode, direction=1),
            (ord('M'), 1): partial(LidarScanViz.cycle_cloud_mode, direction=-1),
            (ord('F'), 0): partial(LidarScanViz.cycle_cloud_palette, direction=1),
            (ord('F'), 1): partial(LidarScanViz.cycle_cloud_palette, direction=-1),
            (ord("'"), 0): partial(LidarScanViz.update_ring_size, amount=1),
            (ord("'"), 1): partial(LidarScanViz.update_ring_size, amount=-1),
            (ord("'"), 2): LidarScanViz.cicle_ring_line_width,
            (ord("O"), 0): LidarScanViz.toggle_osd,
            # NOTE[pb]: Left for future consideration whether we want it back or no
            # (ord('T'), 0): LidarScanViz.toggle_scan_poses,
            (ord('U'), 0): LidarScanViz.toggle_camera_follow,
            # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
            (ord('C'), 0): LidarScanViz.update_flags_mode,
            (ord('9'), 0): LidarScanViz.toggle_scan_axis,
            (ord('/'), 1): LidarScanViz.print_keys,
        }

        def handle_keys(self: LidarScanViz, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            if (key, mods) in key_bindings:
                key_bindings[key, mods](self)
                self.draw()
            return True

        key_definitions: Dict[str, str] = {
            'w': "Camera pitch up",
            's': "Camera pitch down",
            'a': "Camera yaw left",
            'd': "Camera yaw right",
            "e / E": "Increase/decrease size of displayed 2D images",
            "p / P": "Increase/decrease point size",
            "R": "Reset camera orientation",
            "ctr+r": "Camera bird-eye view",
            "0": "Toggle orthographic camera",
            "1": "Toggle point cloud 1 visibility",
            "2": "Toggle point cloud 2 visibility",
            "b": "Cycle top 2D image",
            "n": "Cycle bottom 2D image",
            'm': "Cycle through point cloud coloring mode",
            'f': "Cycle through point cloud color palette",
            # 't': "Toggle scan poses",
            'u': "Toggle camera mode FOLLOW/FIXED",
            "9": "Toggle axis helpers at scan origin",
            '?': "Print keys to standard out",
            "= / -": "Dolly in and out",
            "' / \"": "Increase/decrease spacing in range markers",
            'SHIFT': "Camera Translation with mouse drag",
            'ESC': "Exit the application",
        }
        self._key_definitions = key_definitions

        # TODO[pb]: Consider moving the ? handling to the SimpleViz, since
        #           it's the most common driver of the LidarScanViz. However
        #           because LidarScanViz can be alone on top of PointViz
        #           it's good to have the key help there as well .... need to
        #           think/discuss more SimpleViz/<Others>Viz objects interplay
        print("Press \'?\' while viz window is focused to print key bindings")

        push_point_viz_handler(self._viz, self, handle_keys)
        add_default_controls(self._viz)

    def cycle_img_mode(self, i: int, *, direction: int = 1) -> None:
        """Change the displayed field of the i'th image."""
        with self._lock:
            self._img_ind[i] += direction

    def cycle_cloud_mode(self, *, direction: int = 1) -> None:
        """Change the coloring mode of the 3D point cloud."""
        with self._lock:
            self._cloud_mode_ind = (self._cloud_mode_ind + direction)
            self._cloud_refl_mode = False

    def cycle_cloud_palette(self, *, direction: int = 1) -> None:
        """Change the color palette of the 3D point cloud."""
        with self._lock:
            npalettes = len(self._cloud_palettes)
            self._cloud_palette_ind = (self._cloud_palette_ind + npalettes +
                                       direction) % npalettes
            self._cloud_palette = self._cloud_palettes[self._cloud_palette_ind]
            self._cloud_refl_mode = False

    def toggle_cloud(self, i: int) -> None:
        """Toggle whether the i'th return is displayed."""
        with self._lock:
            if self._cloud_enabled[i]:
                self._cloud_enabled[i] = False
                self._viz.remove(self._clouds[i])
            else:
                self._cloud_enabled[i] = True
                self._viz.add(self._clouds[i])

    def update_point_size(self, amount: int) -> None:
        """Change the point size of the 3D cloud."""
        with self._lock:
            self._cloud_pt_size = min(10.0,
                                      max(1.0, self._cloud_pt_size + amount))
            for cloud in self._clouds:
                cloud.set_point_size(self._cloud_pt_size)

    def update_image_size(self, amount: int) -> None:
        """Change the size of the 2D image."""
        with self._lock:
            size_fraction_max = 20
            self._img_size_fraction = (self._img_size_fraction + amount +
                                       (size_fraction_max + 1)) % (
                                           size_fraction_max + 1)

            vfrac = self._img_size_fraction / size_fraction_max
            hfrac = vfrac / 2 / self._img_aspect

            self._images[0].set_position(-hfrac, hfrac, 1 - vfrac, 1)
            self._images[1].set_position(-hfrac, hfrac, 1 - vfrac * 2,
                                         1 - vfrac)

            # center camera target in area not taken up by image
            self._viz.camera.set_proj_offset(0, vfrac)

    def update_ring_size(self, amount: int) -> None:
        """Change distance ring size."""
        with self._lock:
            self._ring_size = min(3, max(-2, self._ring_size + amount))
            self._viz.target_display.set_ring_size(self._ring_size)

    def cicle_ring_line_width(self) -> None:
        """Change rings line width."""
        with self._lock:
            self._ring_line_width = max(1, (self._ring_line_width + 1) % 10)
            self._viz.target_display.set_ring_line_width(self._ring_line_width)

    def toggle_osd(self, state: Optional[bool] = None) -> None:
        """Show or hide the on-screen display."""
        with self._lock:
            self._osd_enabled = not self._osd_enabled if state is None else state

    # NOTE: Left for future consideration
    # def toggle_scan_poses(self) -> None:
    #     """Toggle the per column poses use."""
    #     with self._lock:
    #         if self._scan_poses_enabled:
    #             self._scan_poses_enabled = False
    #             print("LidarScanViz: Key T: Scan Poses: OFF")
    #         else:
    #             self._scan_poses_enabled = True
    #             print("LidarScanViz: Key T: Scan Poses: ON")

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

            # set mask on all points in the second cloud, clear other mask
            if self._flags_mode == LidarScanViz.FlagsMode.HIGHLIGHT_SECOND:
                self._cloud_masks[0][:, :, 3] = 0.0
                self._cloud_masks[1][:, :, 3] = 1.0
                self._clouds[0].set_mask(self._cloud_masks[0])
                self._clouds[1].set_mask(self._cloud_masks[1])
            # clear masks on both clouds, expected to be set dynamically in _draw()
            else:
                self._cloud_masks[0][:, :, 3] = 0.0
                self._cloud_masks[1][:, :, 3] = 0.0
                self._clouds[0].set_mask(self._cloud_masks[0])
                self._clouds[1].set_mask(self._cloud_masks[1])

            # not bothering with OSD
            # TODO[pb]: hmm, probably should bother with OSD somehow
            print("Flags mode:", self._flags_mode.name)

    def toggle_scan_axis(self) -> None:
        """Toggle the helper axis of a scan ON/OFF"""
        with self._lock:
            self._scan_axis.toggle()

    def print_keys(self) -> None:
        with self._lock:
            print(">---------------- Key Bindings --------------<")
            for key_binding in self._key_definitions:
                print(f"{key_binding:^7}: {self._key_definitions[key_binding]}")
            print(">--------------------------------------------<")

    @property
    def metadata(self) -> client.SensorInfo:
        """The sensor metadata of the scans."""
        return self._metadata

    @property
    def scan(self) -> client.LidarScan:
        """The currently displayed scan."""
        return self._scan

    @property
    def scan_num(self) -> int:
        """The currently displayed scan number"""
        return self._scan_num

    def update(self,
               scan: client.LidarScan,
               scan_num: Optional[int] = None) -> None:
        self._scan = scan
        if scan_num is not None:
            self._scan_num = scan_num
        else:
            self._scan_num += 1

    def draw(self, update: bool = True) -> bool:
        """Process and draw the latest state to the screen."""
        with self._lock:
            self._draw()

        if update:
            return self._viz.update()
        else:
            return False

    def run(self) -> None:
        """Run the rendering loop of the visualizer.

        See :py:meth:`.PointViz.run`
        """
        self._viz.run()

    # i/o and processing, called from client thread
    # usually need to synchronize with key handlers, which run in render thread
    def _draw(self) -> None:

        # figure out what to draw based on current viz state
        scan = self._scan

        # available display modes
        img_modes = list(
            filter(lambda m: m.mode.enabled(scan, m.return_num),
                   self._image_modes))
        cloud_modes = list(filter(lambda m: m.enabled(scan),
                                  self._cloud_modes))

        # update 3d display
        self._cloud_mode_ind = (self._cloud_mode_ind +
                                len(cloud_modes)) % len(cloud_modes)
        cloud_mode = cloud_modes[self._cloud_mode_ind]

        refl_mode = is_norm_reflectivity_mode(cloud_mode)
        if refl_mode:
            if not self._cloud_refl_mode_prev:
                self._cloud_palette = CloudPaletteItem("Cal. Ref", calref_palette)
                self._cloud_refl_mode = True
            elif not self._cloud_refl_mode:
                self._cloud_palette = self._cloud_palettes[
                    self._cloud_palette_ind]
        else:
            if self._cloud_refl_mode_prev:
                self._cloud_palette = self._cloud_palettes[
                    self._cloud_palette_ind]
        self._cloud_refl_mode_prev = refl_mode

        for i, range_field in ((0, ChanField.RANGE), (1, ChanField.RANGE2)):
            if range_field in scan.fields:
                range_data = scan.field(range_field)
            else:
                range_data = np.zeros((scan.h, scan.w), dtype=np.uint32)

            self._clouds[i].set_range(range_data)

            if self._cloud_palette is not None:
                self._clouds[i].set_palette(self._cloud_palette.palette)

            if cloud_mode.enabled(scan, i):
                cloud_modes[self._cloud_mode_ind].set_cloud_color(
                    self._clouds[i], scan, return_num=i)
            else:
                cloud_modes[self._cloud_mode_ind].set_cloud_color(
                    self._clouds[i], scan, return_num=0)

        if self._cloud_palette is not None:
            self._cloud_palette_name = self._cloud_palette.name

        # palette is set only on the first _draw when it's changed
        self._cloud_palette = None

        # update 2d images
        for i in (0, 1):
            self._img_ind[i] = (self._img_ind[i] +
                                len(img_modes)) % len(img_modes)
            img_mode_item = img_modes[self._img_ind[i]]
            img_mode = img_mode_item.mode

            refl_mode = is_norm_reflectivity_mode(img_mode)
            if refl_mode and not self._img_refl_mode[i]:
                self._images[i].set_palette(calref_palette)
            if not refl_mode and self._img_refl_mode[i]:
                self._images[i].clear_palette()
            self._img_refl_mode[i] = refl_mode

            img_mode.set_image(self._images[i], scan, img_mode_item.return_num)

        # update osd
        meta = self._metadata
        first_ts_s = client.first_valid_column_ts(scan) / 1e9

        cloud_idxs_str = str([i + 1 for i in range(len(self._cloud_enabled))])
        cloud_states_str = ", ".join(["ON" if e else "OFF" for e in self._cloud_enabled])

        osd_str_extra = self._osd_text_extra()
        if osd_str_extra:
            osd_str_extra += "\n"

        if self._osd_enabled:
            osd_str = f"image [B, N]: {img_modes[self._img_ind[0]].name}, {img_modes[self._img_ind[1]].name}\n" \
                      f"cloud {cloud_idxs_str}: {cloud_states_str}\n"
            osd_str += f"        mode [M]: {cloud_modes[self._cloud_mode_ind].name}\n" \
                       f"        palette [F]: {self._cloud_palette_name}\n" \
                       f"        point size [P]: {int(self._cloud_pt_size)}\n"
            osd_str += f"{osd_str_extra}" \
                       f"axes [9]: {'ON' if self._scan_axis.enabled else 'OFF'}\n" \
                       f"camera mode [U]: {'FOLLOW' if self._camera_follow_enabled else 'FIXED'}\n" \
                       f"frame: {scan.frame_id}, sensor ts: {first_ts_s:.3f}s\n" \
                       f"profile: {str(meta.format.udp_profile_lidar).replace('_', '..')}\n" \
                       f"{meta.prod_line} {meta.fw_rev} {meta.mode}"
        else:
            osd_str = ""

        self._osd.set_text(osd_str)

        if scan.shot_limiting() != ShotLimitingStatus.SHOT_LIMITING_NORMAL:
            print(f"WARNING: Shot limiting status: {scan.shot_limiting()} "
                  f"(sensor ts: {first_ts_s:.3f})")
        if scan.thermal_shutdown() != ThermalShutdownStatus.THERMAL_SHUTDOWN_NORMAL:
            print(f"WARNING: Thermal shutdown status: {scan.thermal_shutdown()} "
                  f"(sensor ts: {first_ts_s:.3f})")

        self._draw_update_scan_poses()

        # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
        self._draw_update_flags_mode()

    def _draw_update_scan_poses(self) -> None:
        """Apply poses from the scan to the cloud"""

        column_poses = (self._scan.pose if self._scan_poses_enabled else
                        self._scan_column_poses_identity)

        for cloud in self._clouds:
            cloud.set_column_poses(column_poses)

        scan_pose = column_poses[client.first_valid_column(self._scan)]

        # with poses camera tracks the scan position
        camera_target = (np.linalg.inv(scan_pose)
                         if self._scan_poses_enabled else np.eye(4))

        # TODO[pb]: We probably shouldn't have the camera settings to
        # FOLLOW/FIXED in the LidarScan viz, but move it to the SimpleViz.
        # Potential complications of such move is the OSD display that happens
        # in the LidarScanViz and combines already the ScansAccumulator display.
        if self._camera_follow_enabled:
            self._viz.camera.set_target(camera_target)

        # update scan axis helper
        self._scan_axis.pose = scan_pose @ self._metadata.extrinsic

    def _draw_update_flags_mode(self) -> None:
        """Apply selected FlagsMode to the cloud"""
        # set a cloud mask based where first flags bit is set
        if self._flags_mode == LidarScanViz.FlagsMode.HIGHLIGHT_BLOOM:
            for i, flag_field in ((0, ChanField.FLAGS), (1, ChanField.FLAGS2)):
                if flag_field in self._scan.fields:
                    mask_opacity = (self._scan.field(flag_field) & 0x1) * 1.0
                    self._cloud_masks[i][:, :, 3] = mask_opacity
                    self._clouds[i].set_mask(self._cloud_masks[i])

        # set range to zero where first flags bit is set
        elif self._flags_mode == LidarScanViz.FlagsMode.HIDE_BLOOM:
            for i, flag_field, range_field in ((0, ChanField.FLAGS,
                                                ChanField.RANGE),
                                               (1, ChanField.FLAGS2,
                                                ChanField.RANGE2)):
                if flag_field in self._scan.fields and range_field in self._scan.fields:
                    # modifying the scan in-place would break cycling modes while paused
                    range = self._scan.field(range_field).copy()
                    range[self._scan.field(flag_field) & 0x1 == 0x1] = 0
                    self._clouds[i].set_range(range)


class _Seekable(Generic[T]):
    """Wrap an iterable to support seeking by index.

    Similar to `more_itertools.seekable` but keeps indexes stable even values
    are evicted from the cache.

    The :meth:`seek` and :meth:`__next__` methods maintain the invariant:

        (read_ind - len(cache)) < next_ind <= read_ind + 1
    """

    def __init__(self, it: Iterable[T], maxlen=50) -> None:
        self._next_ind = 0  # index of next value to be returned
        self._read_ind = -1  # index of most recent (leftmost) value in cache
        self._iterable = it
        self._it = iter(it)
        self._maxlen = maxlen
        self._cache: Deque[T] = deque([], maxlen)

    def __iter__(self) -> '_Seekable[T]':
        return self

    def __next__(self) -> T:
        # next value already read, is in cache
        if self._next_ind <= self._read_ind:
            t = self._cache[self._read_ind - self._next_ind]
            self._next_ind += 1
            return t
        # next value comes from iterator
        elif self._next_ind == self._read_ind + 1:
            t = next(self._it)
            self._cache.appendleft(t)
            if len(self._cache) > self._maxlen:
                self._cache.pop()
            self._next_ind += 1
            self._read_ind += 1
            return t
        else:
            raise AssertionError("Violated: next_ind <= read_ind + 1")

    @property
    def next_ind(self) -> int:
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
            self._iterable.close()  # type: ignore


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


# TODO: Make/Define a better ScanViz interface
# not a best way to describe interface, yeah duck typing danger, etc ...
# but ScanViz object should have a write property 'scan' and underlying
# Point viz member at '_viz'
AnyScanViz = Union[LidarScanViz, Any]


class SimpleViz:
    """Visualize a stream of LidarScans.

    Handles controls for playback speed, pausing and stepping."""

    _playback_rates: ClassVar[Tuple[float, ...]]
    _playback_rates = (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 0.0)

    def __init__(self,
                 arg: Union[client.SensorInfo, AnyScanViz],
                 *,
                 rate: Optional[float] = None,
                 pause_at: int = -1,
                 on_eof: str = 'exit',
                 scans_accum: Optional[Union[bool, ScansAccumulator]] = None,
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
        if isinstance(arg, client.SensorInfo):
            self._metadata = arg
            self._viz = PointViz("Ouster Viz")
            self._scan_viz = LidarScanViz(arg, self._viz)
        elif isinstance(arg, LidarScanViz):
            self._metadata = arg._metadata
            self._viz = arg._viz
            self._scan_viz = arg
        else:
            # we continue, so custom ScanVizs can be used with the same
            # SimpleViz class and basic controls
            self._viz = arg._viz
            self._scan_viz = arg

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
        self._playback_osd = Label("", 1, 1, align_right=True)
        self._viz.add(self._playback_osd)
        self._osd_enabled = True
        self._update_playback_osd()

        # continuous screenshots recording
        self._viz_img_recording = False

        metas = ([self._scan_viz.metadata] if isinstance(
            self._scan_viz.metadata, client.SensorInfo) else
                 self._scan_viz.metadata)

        self._scans_accum = None
        # TODO[pb]: Here we consider ScansAccumulator separately for
        # first tests and ease of use, however its behaviour is very close
        # (fully aligned?) with the `arg: AnyScanViz` option. Which means that
        # we might consider a refactor when SimpleViz accepts not a single
        # `arg: AnyScanViz` but a list `List[AnyScanViz]` type.
        # to be discussed and refactored later ....
        if scans_accum:
            if isinstance(scans_accum, ScansAccumulator):
                self._scans_accum = scans_accum
                if self._scans_accum.viz is None:
                    self._scans_accum.set_point_viz(self._viz)
            else:
                self._scans_accum = ScansAccumulator(metas, point_viz=self._viz)
            self._scans_accum.toggle_osd(False)

            if hasattr(self._scan_viz, "_osd_text_extra"):
                self._scan_viz._osd_text_extra = self._scans_accum.osd_text
                # if we add the extra text to the _scan_viz we also want to
                # draw(w/o update) its state when scans accum keys cause the
                # re-draw of ScansAccumulator state (osd included)
                self._scans_accum._key_press_pre_draw = (
                    lambda: self._scan_viz.draw(update=False))

        if hasattr(self._scan_viz, "_key_definitions") and self._scans_accum:
            self._scan_viz._key_definitions.update(
                getattr(self._scans_accum, "_key_definitions", {}))

        key_bindings: Dict[Tuple[int, int], Callable[[SimpleViz], None]] = {
            (ord(','), 0): partial(SimpleViz.seek_relative, n_frames=-1),
            (ord(','), 2): partial(SimpleViz.seek_relative, n_frames=-10),
            (ord('.'), 0): partial(SimpleViz.seek_relative, n_frames=1),
            (ord('.'), 2): partial(SimpleViz.seek_relative, n_frames=10),
            (ord(' '), 0): SimpleViz.toggle_pause,
            (ord('O'), 0): SimpleViz.toggle_osd,
            (ord('X'), 1): SimpleViz.toggle_img_recording,
            (ord('Z'), 1): SimpleViz.screenshot,
        }

        key_definitions: Dict[str, str] = {
            'o': "Toggle information overlay",
            'shift+x': "Toggle a continuous saving of screenshots",
            'shift+z': "Take a screenshot!",
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

    def _update_playback_osd(self) -> None:
        if not self._osd_enabled:
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
            self._osd_enabled = not self._osd_enabled if state is None else state
            self._scan_viz.toggle_osd(self._osd_enabled)
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

    def _frame_period(self) -> float:
        rate = SimpleViz._playback_rates[self._rate_ind]
        if rate and not self._paused:
            if isinstance(self._scan_viz, LidarScanViz):
                return 1.0 / (self._metadata.format.fps * rate)
            else:
                # if some other scan viz that is not derived from LidarScanViz
                # we default to 10 Hz
                return 1.0 / (10 * rate)
        else:
            return 0.0

    def _process(self, seekable: _Seekable[client.LidarScan]) -> None:

        last_ts = time.monotonic()
        scan_idx = -1
        try:
            while True:
                # wait until unpaused, step, or quit
                try:
                    with self._cv:
                        self._cv.wait_for(lambda: not self._paused or self._step or
                                          self._proc_exit)
                        if self._proc_exit:
                            break
                        if self._step:
                            seek_ind = seekable.next_ind + self._step - 1
                            self._step = 0
                            if not seekable.seek(seek_ind):
                                continue
                        period = self._frame_period()

                    # process new data
                    scan_idx = seekable.next_ind
                    scan = next(seekable)
                    # TODO[pb]: Now scan_idx keeps increasing if looped source
                    # is presented, thus there is a need to keep track the lapsed
                    # scan_idx and pass it always as a valid scan number starting
                    # from 0 of the scans source.
                    self._scan_viz.update(scan, scan_idx)
                    if self._scans_accum:
                        self._scans_accum.update(scan, scan_idx)

                    self._scan_viz.draw(update=False)
                    if self._scans_accum:
                        self._scans_accum.draw(update=False)

                    if self._pause_at == scan_idx:
                        self._paused = True

                    self._update_playback_osd()

                    # sleep for remainder of scan period
                    to_sleep = max(0.0, period - (time.monotonic() - last_ts))
                    if scan_idx > 0:
                        time.sleep(to_sleep)

                    now_t = time.monotonic()
                    # track process/draw period
                    self._last_draw_period = now_t - last_ts
                    last_ts = now_t

                    # show new data
                    self._viz.update()
                except StopIteration:
                    if self._on_eof == 'exit':
                        break

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
            pass
        finally:
            try:
                # some scan sources may be waiting on IO, blocking the
                # processing thread
                seekable.close()
            except Exception as e:
                logger.warn(f"Data source closed with error: '{e}'")

            # processing thread will still be running if e.g. viz window was closed
            with self._cv:
                self._proc_exit = True
                self._cv.notify()

            logger.info("Joining processing thread")
            proc_thread.join()


def scans_accum_for_cli(metas: Union[client.SensorInfo,
                                     List[client.SensorInfo]],
                        *,
                        accum_num: int = 0,
                        accum_every: Optional[int] = None,
                        accum_every_m: Optional[float] = None,
                        accum_map: bool = False,
                        accum_map_ratio: float = 0.001) -> ScansAccumulator:
    """Create ScansAccumulator using its CLI params."""
    a_every = 1
    a_every_m = 0.0
    if accum_every is None and accum_every_m is not None:
        a_every = 0
        a_every_m = accum_every_m
    elif accum_every is not None and accum_every_m is None:
        a_every = accum_every
        a_every_m = 0.0
    elif accum_every is not None and accum_every_m is not None:
        a_every = accum_every
        a_every_m = accum_every_m

    return ScansAccumulator(metas,
                            accum_max_num=accum_num,
                            accum_min_dist_num=a_every,
                            accum_min_dist_meters=a_every_m,
                            map_enabled=accum_map,
                            map_select_ratio=accum_map_ratio)


__all__ = [
    'PointViz', 'Cloud', 'Image', 'Cuboid', 'Label', 'WindowCtx', 'Camera',
    'TargetDisplay', 'add_default_controls', 'calref_palette', 'spezia_palette',
    'grey_palette', 'viridis_palette', 'magma_palette', 'ImageMode',
    'CloudMode', 'CloudPaletteItem', 'VizExtraMode'
]

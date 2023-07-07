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
                    Optional, Tuple, TypeVar, Union, Any, cast)
from typing_extensions import Protocol, runtime_checkable
import weakref
import logging
import click

import numpy as np
from PIL import Image as PILImage

from ouster import client
from ouster.client import _utils, ChanField, first_valid_column_pose
from ..client._client import Version
from ._viz import (PointViz, Cloud, Image, Cuboid, Label, WindowCtx, Camera,
                   TargetDisplay, add_default_controls, calref_palette,
                   spezia_palette, grey_palette, viridis_palette, magma_palette)

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


@runtime_checkable
class FieldViewMode(Protocol):
    """LidarScan field processor

    View modes define the process of getting the key data for
    the scan and return number as well as checks the possibility
    of showing data in that mode, see `enabled()`.
    """

    _info: Optional[client.SensorInfo]

    @property
    def name(self) -> str:
        """Name of the view mode"""
        ...

    @property
    def names(self) -> List[str]:
        """Name of the view mode per return number"""
        ...

    def _prepare_data(self,
                      ls: client.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:
        """Prepares data for visualization given the scan and return number"""
        ...

    def enabled(self, ls: client.LidarScan, return_num: int = 0) -> bool:
        """Checks the view mode availability for a scan and return number"""
        ...


@runtime_checkable
class ImageMode(FieldViewMode, Protocol):
    """Applies the view mode key to the viz.Image"""

    def set_image(self,
                  img: Image,
                  ls: client.LidarScan,
                  return_num: int = 0) -> None:
        """Prepares the key data and sets the image key to it."""
        ...


@runtime_checkable
class CloudMode(FieldViewMode, Protocol):
    """Applies the view mode key to the viz.Cloud"""

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: client.LidarScan,
                        *,
                        return_num: int = 0) -> None:
        """Prepares the key data and sets the cloud key to it."""
        ...


class ImageCloudMode(ImageMode, CloudMode, Protocol):
    """Applies the view mode to viz.Cloud and viz.Image"""
    pass


def _second_chan_field(field: client.ChanField) -> Optional[client.ChanField]:
    """Get the second return field name."""
    # yapf: disable
    second_fields = dict({
        client.ChanField.RANGE: client.ChanField.RANGE2,
        client.ChanField.SIGNAL: client.ChanField.SIGNAL2,
        client.ChanField.REFLECTIVITY: client.ChanField.REFLECTIVITY2,
        client.ChanField.FLAGS: client.ChanField.FLAGS2
    })
    # yapf: enable
    return second_fields.get(field, None)


class SimpleMode(ImageCloudMode):
    """Basic view mode with AutoExposure and BeamUniformityCorrector

    Handles single and dual returns scans.

    When AutoExposure is enabled its state updates only for return_num=0 but
    applies for both returns.
    """
    def __init__(self,
                 field: client.ChanField,
                 *,
                 info: Optional[client.SensorInfo] = None,
                 prefix: Optional[str] = "",
                 suffix: Optional[str] = "",
                 use_ae: bool = True,
                 use_buc: bool = False) -> None:
        """
        Args:
            info: sensor metadata used mainly for destaggering here
            field: ChanField to process, second return is handled automatically
            prefix: name prefix
            suffix: name suffix
            use_ae: if True, use AutoExposure for the field
            use_buc: if True, use BeamUniformityCorrector for the field
        """
        self._info = info
        self._fields = [field]
        field2 = _second_chan_field(field)
        if field2:
            self._fields.append(field2)
        self._ae = _utils.AutoExposure() if use_ae else None
        self._buc = _utils.BeamUniformityCorrector() if use_buc else None
        self._prefix = f"{prefix}: " if prefix else ""
        self._suffix = f" ({suffix})" if suffix else ""
        self._wrap_name = lambda n: f"{self._prefix}{n}{self._suffix}"

    @property
    def name(self) -> str:
        return self._wrap_name(str(self._fields[0]))

    @property
    def names(self) -> List[str]:
        return [self._wrap_name(str(f)) for f in self._fields]

    def _prepare_data(self,
                      ls: client.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None

        f = self._fields[return_num]
        key_data = ls.field(f).astype(np.float32)

        if self._buc:
            self._buc(key_data)

        if self._ae:
            self._ae(key_data, update_state=(return_num == 0))
        else:
            key_max = np.max(key_data)
            if key_max:
                key_data = key_data / key_max

        return key_data

    def set_image(self,
                  img: Image,
                  ls: client.LidarScan,
                  return_num: int = 0) -> None:
        if self._info is None:
            raise ValueError(
                f"VizMode[{self.name}] requires metadata to make a 2D image")
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            img.set_image(client.destagger(self._info, key_data))

    def set_cloud_color(self,
                        cloud: Cloud,
                        ls: client.LidarScan,
                        return_num: int = 0) -> None:
        key_data = self._prepare_data(ls, return_num)
        if key_data is not None:
            cloud.set_key(key_data)

    def enabled(self, ls: client.LidarScan, return_num: int = 0):
        return (self._fields[return_num] in ls.fields
                if return_num < len(self._fields) else False)


class ReflMode(SimpleMode, ImageCloudMode):
    """Prepares image/cloud data for REFLECTIVITY channel"""

    def __init__(self, *, info: Optional[client.SensorInfo] = None) -> None:
        super().__init__(client.ChanField.REFLECTIVITY, info=info, use_ae=True)
        # used only for uncalibrated reflectivity in FW prior v2.1.0
        # TODO: should we check for calibrated reflectivity status from
        # metadata too?
        if info is not None:
            self._info = cast(client.SensorInfo, self._info)
            self._normalized_refl = (Version.from_string(self._info.fw_rev) >=
                                     Version.from_string("v2.1.0"))
        else:
            # NOTE/TODO[pb]: ReflMode added through viz extra mode mechanism
            # may not have a correct normalized_refl set ... need a refactor.
            self._normalized_refl = True

    def _prepare_data(self,
                      ls: client.LidarScan,
                      return_num: int = 0) -> Optional[np.ndarray]:
        if not self.enabled(ls, return_num):
            return None

        f = self._fields[return_num]
        refl_data = ls.field(f).astype(np.float32)
        if self._normalized_refl:
            refl_data /= 255.0
        else:
            # mypy doesn't recognize that we always should have _ae here
            # so we have explicit check
            if self._ae:
                self._ae(refl_data, update_state=(return_num == 0))
        return refl_data


def is_norm_reflectivity_mode(mode: FieldViewMode) -> bool:
    """Checks whether the image/cloud mode is a normalized REFLECTIVITY mode

    NOTE[pb]: This is highly implementation specific and doesn't look nicely,
    i.e. it's more like duck/duct plumbing .... but suits the need.
    """
    return (isinstance(mode, ReflMode) and mode._normalized_refl)


@dataclass
class ImgModeItem:
    """Image mode for specific return with explicit name."""
    mode: ImageMode
    name: str
    return_num: int = 0


@dataclass
class CloudPaletteItem:
    """Palette with a name"""
    name: str
    palette: np.ndarray


LidarScanVizMode = Union[ImageCloudMode, ImageMode, CloudMode]
"""Field view mode types"""


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

        self._cloud_refl_mode = False

        self._cloud_palettes: List[CloudPaletteItem]
        self._cloud_palettes = [
            CloudPaletteItem("Ouster Colors", spezia_palette),
            CloudPaletteItem("Greyscale", grey_palette),
            CloudPaletteItem("Viridis", viridis_palette),
            CloudPaletteItem("Magma", magma_palette),
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
        self._ring_line_width = 1
        self._osd_enabled = True
        self._scan_poses_enabled = False

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
        self._viz.target_display.set_ring_size(self._ring_size)
        self._viz.target_display.enable_rings(True)

        # initialize osd
        self._osd = Label("", 0, 1)
        self._viz.add(self._osd)

        self._scan_ind = -1

        # scan identity poses to re-use
        self._scan_column_poses_identity = np.array(
            [np.eye(4) for _ in range(self._metadata.format.columns_per_frame)],
            dtype=np.float32)

        # key bindings. will be called from rendering thread, must be synchronized
        key_bindings: Dict[Tuple[int, int], Callable[[LidarScanViz], None]] = {
            (ord('E'), 0): partial(LidarScanViz.update_image_size, amount=1),
            (ord('E'), 1): partial(LidarScanViz.update_image_size, amount=-1),
            (ord('P'), 0): partial(LidarScanViz.update_point_size, amount=1),
            (ord('P'), 1): partial(LidarScanViz.update_point_size, amount=-1),
            (ord('1'), 0): partial(LidarScanViz.toggle_cloud, i=0),
            (ord('2'), 0): partial(LidarScanViz.toggle_cloud, i=1),
            (ord('B'), 0): partial(LidarScanViz.cycle_img_mode, i=0),
            (ord('N'), 0): partial(LidarScanViz.cycle_img_mode, i=1),
            (ord('M'), 0): LidarScanViz.cycle_cloud_mode,
            (ord('F'), 0): LidarScanViz.cycle_cloud_palette,
            (ord("'"), 0): partial(LidarScanViz.update_ring_size, amount=1),
            (ord("'"), 1): partial(LidarScanViz.update_ring_size, amount=-1),
            (ord("'"), 2): LidarScanViz.cicle_ring_line_width,
            (ord("O"), 0): LidarScanViz.toggle_osd,
            (ord('T'), 0): LidarScanViz.toggle_scan_poses,
            # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
            (ord('C'), 0): LidarScanViz.update_flags_mode,
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
            "0": "Toggle orthographic camera",
            "1": "Toggle point cloud 1 visibility",
            "2": "Toggle point cloud 2 visibility",
            "b": "Cycle top 2D image",
            "n": "Cycle bottom 2D image",
            'm': "Cycle through point cloud coloring mode",
            'f': "Cycle through point cloud color palette",
            't': "Toggle scan poses",
            '?': "Print keys to standard out",
            "= / -": "Dolly in and out",
            "' / \"": "Increase/decrease spacing in range markers",
            'SHIFT': "Camera Translation with mouse drag",
            'ESC': "Exit the application",
        }
        self._key_definitions = key_definitions
        print("Press \'?\' while viz window is focused to print key bindings")

        push_point_viz_handler(self._viz, self, handle_keys)
        add_default_controls(self._viz)

    def cycle_img_mode(self, i: int) -> None:
        """Change the displayed field of the i'th image."""
        with self._lock:
            self._img_ind[i] += 1

    def cycle_cloud_mode(self) -> None:
        """Change the coloring mode of the 3D point cloud."""
        with self._lock:
            self._cloud_mode_ind = (self._cloud_mode_ind + 1)

    def cycle_cloud_palette(self) -> None:
        """Change the color palette of the 3D point cloud."""
        with self._lock:
            self._cloud_palette_ind = (self._cloud_palette_ind + 1) % len(
                self._cloud_palettes)
            self._cloud_palette = self._cloud_palettes[self._cloud_palette_ind]

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

    def toggle_scan_poses(self) -> None:
        """Toggle the per column poses use."""
        with self._lock:
            if self._scan_poses_enabled:
                self._scan_poses_enabled = False
                print("LidarScanViz: Key T: Scan Poses: OFF")
            else:
                self._scan_poses_enabled = True
                print("LidarScanViz: Key T: Scan Poses: ON")

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

    def print_keys(self) -> None:
        with self._lock:
            print(">---------------- Key Bindings --------------<")
            for key_binding in self._key_definitions:
                print(f"{key_binding:^7}: {self._key_definitions[key_binding]}")
            print(">--------------------------------------------<")

    @property
    def scan(self) -> client.LidarScan:
        """The currently displayed scan."""
        return self._scan

    @scan.setter
    def scan(self, scan: client.LidarScan) -> None:
        """Set the scan to display"""
        self._scan = scan

    @property
    def scan_ind(self) -> int:
        """The currently displayed scan number (-1 if was never set)"""
        return self._scan_ind

    @scan_ind.setter
    def scan_ind(self, scan_ind: int) -> None:
        """Set the scan number of the currently displayed scan"""
        self._scan_ind = scan_ind

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
        self._cloud_mode_ind %= len(cloud_modes)
        cloud_mode = cloud_modes[self._cloud_mode_ind]

        refl_mode = is_norm_reflectivity_mode(cloud_mode)
        if refl_mode:
            self._cloud_palette = (CloudPaletteItem("Cal. Ref", calref_palette)
                                   if not self._cloud_refl_mode else None)
        else:
            if self._cloud_refl_mode:
                self._cloud_palette = self._cloud_palettes[
                    self._cloud_palette_ind]
        self._cloud_refl_mode = refl_mode

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
            self._img_ind[i] %= len(img_modes)
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
        enable_ind = [i + 1 for i, b in enumerate(self._cloud_enabled) if b]

        nonzeros = np.flatnonzero(scan.timestamp)
        first_ts = scan.timestamp[nonzeros[0]] if len(nonzeros) > 0 else 0

        if self._osd_enabled:
            self._osd.set_text(
                f"image: {img_modes[self._img_ind[0]].name}/{img_modes[self._img_ind[1]].name}\n"
                f"cloud{enable_ind}: {cloud_modes[self._cloud_mode_ind].name}\n"
                f"palette: {self._cloud_palette_name}\n"
                f"frame: {scan.frame_id}\n"
                f"sensor ts: {first_ts / 1e9:.3f}s\n"
                f"profile: {str(meta.format.udp_profile_lidar)}\n"
                f"{meta.prod_line} {meta.fw_rev} {meta.mode}\n"
                f"shot limiting status: {str(scan.shot_limiting())}\n"
                f"thermal shutdown status: {str(scan.thermal_shutdown())}")
        else:
            self._osd.set_text("")

        self._draw_update_scan_poses()

        # TODO[pb]: Extract FlagsMode to custom processor (TBD the whole thing)
        self._draw_update_flags_mode()

    def _draw_update_scan_poses(self) -> None:
        """Apply poses from the scan to the cloud"""

        column_poses = (self._scan.pose if self._scan_poses_enabled else
                        self._scan_column_poses_identity)

        for cloud in self._clouds:
            cloud.set_column_poses(column_poses)

        # with poses camera tracks the scan position
        camera_target = (np.linalg.inv(first_valid_column_pose(self._scan))
                         if self._scan_poses_enabled else np.eye(4))

        self._viz.camera.set_target(camera_target)

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
# but ScanViz object shoud have a write property 'scan' and underlying
# Point viz member at '_viz'
AnyScanViz = Union[LidarScanViz, Any]


class SimpleViz:
    """Visualize a stream of LidarScans.

    Handles controls for playback speed, pausing and stepping."""

    _playback_rates: ClassVar[Tuple[float, ...]]
    _playback_rates = (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 0.0)

    def __init__(self,
                 arg: Union[client.SensorInfo, AnyScanViz],
                 rate: Optional[float] = None,
                 pause_at: int = -1,
                 on_eof: str = 'exit',
                 _buflen: int = 50) -> None:
        """
        Args:
            arg: Metadata associated with the scans to be visualized or a
                 LidarScanViz instance to use.
            rate: Playback rate. One of 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0 or
                  None for "live" playback (the default).
            pause_at: scan number to pause at, dafault (-1) - no auto pause, to
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
                key_bindings[key, mods](self)
                # override rather than add bindings
                return False
            return True

        push_point_viz_handler(self._viz, self, handle_keys)

    def _update_playback_osd(self) -> None:
        if not self._osd_enabled:
            self._playback_osd.set_text("")
        elif self._paused:
            self._playback_osd.set_text("playback: paused")
        elif self._live:
            self._playback_osd.set_text("playback: live")
        else:
            rate = SimpleViz._playback_rates[self._rate_ind]
            self._playback_osd.set_text(
                f"playback: {str(rate) + 'x' if rate else  'max'}")

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
                    self._scan_viz.scan = next(seekable)
                    self._scan_viz.scan_ind = scan_idx
                    self._scan_viz.draw(update=False)

                    if self._pause_at == scan_idx:
                        self._paused = True
                        self._update_playback_osd()

                    # sleep for remainder of scan period
                    to_sleep = max(0.0, period - (time.monotonic() - last_ts))
                    if scan_idx > 0:
                        time.sleep(to_sleep)

                    last_ts = time.monotonic()

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

    def run(self, scans: Iterable[client.LidarScan]) -> None:
        """Start reading scans and visualizing the stream.

        Must be called from the main thread on macos. Will close the provided
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


__all__ = [
    'PointViz', 'Cloud', 'Image', 'Cuboid', 'Label', 'WindowCtx', 'Camera',
    'TargetDisplay', 'add_default_controls', 'calref_palette', 'spezia_palette',
    'grey_palette', 'viridis_palette', 'magma_palette', 'ImageMode',
    'CloudMode', 'ImageCloudMode', 'CloudPaletteItem', 'VizExtraMode'
]

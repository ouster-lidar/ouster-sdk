"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.

Ouster Scan Accumulator Visualizer
"""

from typing import (Optional, Dict, Tuple, List, Callable, Union, Iterable,
                    no_type_check, Any)

import os
from dataclasses import dataclass
import threading
from functools import partial
from itertools import chain
import numpy as np
import time
import logging

from ._viz import (PointViz, WindowCtx, Label, Cloud, grey_palette,
                   spezia_palette, magma_palette, viridis_palette,
                   calref_palette)
from .util import push_point_viz_handler
import ouster.client as client
from ouster.client import ChanField
import ouster.sdk.pose_util as pu

from .view_mode import (CloudMode, ReflMode, SimpleMode,
                        is_norm_reflectivity_mode, CloudPaletteItem)

logger = logging.getLogger("viz-accum-logger")


@dataclass
class ScanRecord:
    poses: List[Optional[np.ndarray]]
    scan: Optional[List[Optional[client.LidarScan]]] = None
    cloud_mode_keys: Optional[Dict[str, Optional[np.ndarray]]] = None


TRACK_INIT_POINTS_NUM: int = 100
TRACK_MAX_POINTS_NUM: int = 100000
MAP_INIT_POINTS_NUM: int = 10000
MAP_MAX_POINTS_NUM: int = 1500000  # 1.5 M default
MAP_SELECT_RATIO: float = 0.001
TRACK_MAP_GROWTH_RATE: float = 1.5

ACCUM_DEBUG = 0
try:
    ACCUM_DEBUG = int(os.getenv("OUSTER_SDK_ACCUM_DEBUG", 0))
except Exception:
    pass

if not logger.hasHandlers():
    logger.addHandler(logging.StreamHandler())
if ACCUM_DEBUG:
    logger.setLevel(logging.DEBUG)


class ScansAccumulator:
    """Accumulate scans, track poses and overall map view

    Could be used with or without ``PointViz`` immediate visualization. With
    ``PointViz`` it acts similarly to ``LidarScanViz`` and can be as a first
    argument passed to ``SimpleViz`` (or as a separate ``scans_accum`` parameter)

    Every new scan (``LidarScan`` or ``Tuple[Optional[LidarScan]]``) is passed
    through ``update(scan, num)``.

    Available visualization depends on whether poses are present or not
    and params set on init. View modes are combination of:

       * **TRACK** - scan poses (i.e. trajectories) of every scan "seen" (poses
         required)
       * **ACCUM** - set of accumulated scans (key frames) picked according to
         params
       * **MAP** - overall map with select ratio of random points from every
         scan passed through ``update()``
    """
    def __init__(self,
                 metas: Union[client.SensorInfo, List[client.SensorInfo]],
                 *,
                 point_viz: Optional[PointViz] = None,
                 accum_max_num: int = 0,
                 accum_min_dist_meters: float = 0,
                 accum_min_dist_num: int = 1,
                 map_enabled: bool = False,
                 map_select_ratio: float = MAP_SELECT_RATIO,
                 map_max_points: int = MAP_MAX_POINTS_NUM,
                 map_overflow_from_start: bool = False):
        """
        Args:
            metas: one or many sensor metadatas for scans that will be passed
              on update
            point_viz: if present then the resulting ScansAccumulator could be
              used similar to LidarScanViz as first argument in SimpleViz
              or with ``scans_accum`` parameter.

              Without ``point_viz`` ScansAccumulator is intended to be used as
              a separate standalone component to accumulate points and get the
              resulting point clouds/tracks with color keys back.

              It's possible to add `PointViz` later using the function
              ``set_point_viz(point_viz)``
            accum_max_num: aka, ``--accum-num``, the maximum number of accumulated
              (ACCUM) scans to keep
            accum_min_dist_meters: aka, ``--accum-every-m``, the minimum distance
              between accumulated (ACCUM) key frames
            accum_min_dist_num: aka, ``--accum-every``, the minimum distance in
              scans between accumulated (ACCUM) key frames
            map_enabled: enable overall map accumulation (MAP) (``--accum-map``)
            map_select_ratio: percent of points to select from the scans to the
              overall map (MAP), default 0.001
            map_max_points: maximum number of points to keep in overall map (MAP)
            map_overflow_from_start: if True, on map overflow continue writing
              points from the beginning (as in ring buffer), if False, overwrite
              points randomly in the existing map
        """
        self._viz: Optional[PointViz] = None
        self._metas = [metas] if isinstance(metas, client.SensorInfo) else metas

        self._kf_min_dist_m = accum_min_dist_meters
        self._kf_min_dist_n = accum_min_dist_num
        self._kf_max_num = accum_max_num

        self._map_enabled = map_enabled
        self._map_select_ratio = map_select_ratio
        self._map_max_points = map_max_points
        self._map_overflow_start = map_overflow_from_start

        # sets to True if non-identity poses were detected in processed scans
        self._poses_detected = False

        self._accum_mode_track = True
        self._accum_mode_accum = False
        self._accum_mode_map = False

        # sensor index used for all map/accum operations
        self._sensor_idx = 0

        self._cloud_pt_size = 1

        self._xyzlut = [
            client.XYZLut(m, use_extrinsics=True) for m in self._metas
        ]

        self._lock = threading.Lock()

        # initialize TRACK structs
        # NOTE[pb]: default zeros are not working great due to how alpha
        #           compositing is implemented (if I guess correctly, but
        #           not 100% sure :() )
        self._track_xyz_init = np.array([10000000, 10000000, 10000000],
                                        dtype=np.float32)
        self._track_xyz = np.full((TRACK_INIT_POINTS_NUM, 3),
                                  self._track_xyz_init,
                                  dtype=np.float32,
                                  order='F')
        self._track_key = np.zeros((TRACK_INIT_POINTS_NUM, 4),
                                   dtype=np.float32)
        self._track_key_color = np.array([0.9, 0.9, 0.9, 1.0],
                                         dtype=np.float32)
        self._track_idx = 0
        self._track_overflow = False

        # initialize the cloud coloration modes
        self._cloud_modes: List[CloudMode] = [
            ReflMode(info=self._metas[self._sensor_idx]),
            SimpleMode(ChanField.NEAR_IR,
                       info=self._metas[self._sensor_idx],
                       use_ae=True,
                       use_buc=True),
            SimpleMode(ChanField.SIGNAL, info=self._metas[self._sensor_idx]),
            SimpleMode(ChanField.RANGE, info=self._metas[self._sensor_idx]),
        ]

        # index of available modes "pens" to get colors for point clouds
        # start with all _cloud_modes and then check on every seen scan
        # that is used for map/accum that we can still use available
        # modes on such scans
        self._available_modes: List[int] = list(range(len(self._cloud_modes)))

        # init view cloud mode toggle
        self._cloud_mode_ind = 0
        self._cloud_mode_ind_prev = (self._cloud_mode_ind + 1) % len(self._cloud_modes)

        # cloud color palettes to use
        self._cloud_palettes: List[CloudPaletteItem] = [
            CloudPaletteItem("Ouster Colors", spezia_palette),
            CloudPaletteItem("Greyscale", grey_palette),
            CloudPaletteItem("Viridis", viridis_palette),
            CloudPaletteItem("Magma", magma_palette),
            CloudPaletteItem("Cal. Ref", calref_palette),
        ]

        # Cal. Ref. is separate because we explicitly set it on REFLECTIVITY
        # color mode and restrict rotations of palettes when it's reflectivity
        self._cloud_calref_palette = CloudPaletteItem("Cal. Ref", calref_palette)

        # init cloud palette toggle
        self._cloud_palette_ind = 0
        self._cloud_palette_ind_prev = self._cloud_palette_ind

        # whether it's currently "snapped" to the Cal.Ref. palette
        self._cloud_palette_refl_mode = False

        # trigger the palette check, so it initializes the _cloud_palette_refl_mode
        # variable correctly for situations when OSD text is drawn before the
        # call to draw()
        # TODO[pb]: Make it less convoluted with palettes toggling and Cal.Ref.
        # snapping
        self._update_cloud_palette()

        # initialize MAP structs
        map_init_points_num = MAP_INIT_POINTS_NUM if self._map_enabled else 0
        self._map_xyz = np.zeros((map_init_points_num, 3),
                                 dtype=np.float32,
                                 order='F')
        # calculated color keys for every map point, indexed by cloud mode name
        self._map_keys: Dict[str, np.ndarray] = {
            mode.name: np.zeros(map_init_points_num, dtype=np.float32)
            for mode in self._cloud_modes
        }
        self._map_idx = 0
        self._map_overflow = False

        # viz.Cloud for map points
        self._cloud_map: Optional[Cloud] = None

        # initialize osd (on screen display text)
        self._osd: Optional[Label] = None
        self._osd_enabled = False

        self._scan_num = -1
        self._scan_records: List[Optional[ScanRecord]] = []

        # key frames is an index to self._scan_records data
        self._key_frames: List[Optional[int]] = [None] * (self._kf_max_num + 1)
        self._key_frames_dirty: List[int] = [0] * (self._kf_max_num + 1)
        self._key_frames_head = 0
        self._key_frames_tail = 0

        # viz.Clouds for accumulated key frames scans (+1 to match the
        # key_frames layout 1-to-1)
        self._clouds_accum: List[Optional[Cloud]] = [None] * (self._kf_max_num + 1)

        # pool of viz.Clouds, to not re-create on often change of key frames num
        self._clouds_accum_pool: List[Cloud] = []

        # accum key frames track (i.e. trajectory points)
        self._kf_track_xyz = np.full((self._kf_max_num + 1, 3),
                                     self._track_xyz_init,
                                     dtype=np.float32,
                                     order='F')
        self._kf_track_key = np.zeros((self._kf_max_num + 1, 4),
                                      dtype=np.float32)
        self._kf_track_key_color = np.array([0.9, 0.9, 0.2, 1.0])

        # scan poses track
        self._cloud_track: Optional[Cloud] = None

        self._last_draw_dt: float = 0
        self._last_update_dt: float = 0

        # callback for any external vizs that need to hookup into the draw update
        # (currently used by LidarScanViz to update the OSD text)
        self._key_press_pre_draw: Callable[[], Any] = lambda: None

        if point_viz:
            self.set_point_viz(point_viz)

    def set_point_viz(self, point_viz: PointViz):
        """Initialize point viz and cloud components."""
        assert self._viz is None, "Can't set viz to ScansAccumulator again"

        self._viz = point_viz

        self._osd_enabled = True
        self._osd = Label("", 0, 1, align_right=False)
        self._viz.add(self._osd)

        self._ensure_cloud_map()

        self._cloud_kf_track = Cloud(self._kf_max_num + 1)
        self._cloud_kf_track.set_point_size(10)
        self._cloud_kf_track.set_xyz(self._kf_track_xyz)
        self._cloud_kf_track.set_key(self._kf_track_key[np.newaxis, ...])
        self._ensure_cloud_track()

        self._initialize_accum_mode()
        self._initialize_key_bindings()

    @property
    def viz(self) -> Optional[PointViz]:
        return self._viz

    def _initialize_accum_mode(self) -> None:
        """Set initial state of accum mode."""
        if self._viz is None:
            return

        # switch accum mode and view for better UX
        if self._map_enabled:
            self.toggle_mode_map(True)
        elif self._kf_max_num > 0:
            self.toggle_mode_accum(True)

        self.toggle_mode_track(self._accum_mode_track)

    def _initialize_key_bindings(self) -> None:
        """Initialize key bindings and key definitions."""
        if self._viz is None:
            return

        key_bindings: Dict[Tuple[int, int], Callable[[ScansAccumulator], bool]] = {
            (ord('J'), 0): partial(ScansAccumulator.update_point_size, amount=1),
            (ord('J'), 1): partial(ScansAccumulator.update_point_size, amount=-1),
            (ord('K'), 0): partial(ScansAccumulator.cycle_cloud_mode, direction=1),
            (ord('K'), 1): partial(ScansAccumulator.cycle_cloud_mode, direction=-1),
            (ord('G'), 0): partial(ScansAccumulator.cycle_cloud_palette, direction=1),
            (ord('G'), 1): partial(ScansAccumulator.cycle_cloud_palette, direction=-1),
            (ord('6'), 0): ScansAccumulator.toggle_mode_accum,
            (ord('7'), 0): ScansAccumulator.toggle_mode_map,
            (ord('8'), 0): ScansAccumulator.toggle_mode_track,
        }

        key_definitions: Dict[str, str] = {
            'j / J': "Increase/decrease point size of accumulated clouds or map",
            'k / K': "Cycle point cloud coloring mode of accumulated clouds or map",
            'g / G': "Cycle point cloud color palette of accumulated clouds or map",
            '6': "Toggle scans accumulation view mode (ACCUM)",
            '7': "Toggle overall map view mode (MAP)",
            '8': "Toggle poses/trajectory view mode (TRACK)",
        }
        self._key_definitions = key_definitions

        def handle_keys(self: ScansAccumulator, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            if (key, mods) in key_bindings:
                draw = key_bindings[key, mods](self)
                if draw:
                    # notify neighbor vizs that need to update on key press re-draw
                    self._key_press_pre_draw()

                    # draw + _viz.update() inside
                    self.draw()
                else:
                    self._viz.update()  # type: ignore
            return True

        push_point_viz_handler(self._viz, self, handle_keys)

    def _ensure_structs_map(self, xyz_size: int = 1) -> None:
        """Check map idx and array sizes and increase if needed"""
        if (not self._map_overflow and
                self._map_idx + xyz_size > self._map_xyz.shape[0]):
            new_size = min(
                self._map_max_points,
                int((self._map_xyz.shape[0] + xyz_size) *
                    TRACK_MAP_GROWTH_RATE))
            logger.debug("RESIZE: map_xyz: %d, max: %d", new_size,
                         self._map_max_points)
            map_xyz = np.zeros((new_size, 3), dtype=np.float32, order='F')
            map_xyz[:self._map_xyz.shape[0]] = self._map_xyz
            del self._map_xyz
            self._map_xyz = map_xyz

            for map_key_name, map_key in self._map_keys.items():
                new_map_key = np.zeros(new_size, dtype=np.float32)
                new_map_key[:map_key.shape[0]] = map_key
                self._map_keys[map_key_name] = new_map_key
                logger.debug("  RESIZE: map_keys[%s]: %d", map_key_name,
                             new_size)
                del map_key

            self._map_overflow = (self._map_idx + xyz_size > new_size)

        # reset map_idx from the beginning if there is no space for all
        # incoming points and map_overflow_start is enabled
        if (self._map_overflow_start and
                self._map_idx + xyz_size > self._map_xyz.shape[0]):
            self._map_idx = 0

    def _ensure_structs_track(self) -> None:
        """Check track idx and array sizes and increase if needed"""
        if (self._track_idx >= self._track_xyz.shape[0] and
                self._track_xyz.shape[0] < TRACK_MAX_POINTS_NUM):
            new_size = min(
                TRACK_MAX_POINTS_NUM,
                int((self._track_key.shape[0] + 1) * TRACK_MAP_GROWTH_RATE))
            logger.debug("RESIZE track_xyz: %d", new_size)
            track_xyz = np.full((new_size, 3),
                                self._track_xyz_init,
                                dtype=np.float32,
                                order='F')
            track_xyz[:self._track_xyz.shape[0]] = self._track_xyz
            self._track_xyz = track_xyz
            track_key = np.zeros((new_size, 4), dtype=np.float32)
            track_key[:self._track_key.shape[0]] = self._track_key
            self._track_key = track_key

        # overflow of the max track size
        if self._track_idx >= self._track_key.shape[0]:
            self._track_idx = 0
            self._track_overflow = True

    def _ensure_cloud_map(self) -> None:
        """Create/re-create the cloud MAP object"""
        def make_cloud_map(n: int):
            self._cloud_map = Cloud(n)
            self._cloud_map.set_point_size(self._cloud_pt_size)
            self._cloud_map.set_palette(self.active_cloud_palette.palette)
            if self.map_visible:
                self._viz.add(self._cloud_map)  # type: ignore

        pnum = self._map_xyz.shape[0]
        if not self._cloud_map:
            make_cloud_map(pnum)
        elif self._cloud_map.size < pnum:
            self._viz.remove(self._cloud_map)  # type: ignore
            del self._cloud_map
            make_cloud_map(pnum)

    def _ensure_cloud_track(self) -> None:
        """Create/re-create the cloud TRACK object"""
        pnum = self._track_xyz.shape[0]
        init_cloud_track = not self._cloud_track
        if init_cloud_track or (self._cloud_track and
                                self._cloud_track.size < pnum):
            self._viz.remove(self._cloud_track)  # type: ignore
            self._viz.remove(self._cloud_kf_track)  # type: ignore
            del self._cloud_track
            self._cloud_track = Cloud(pnum)
            self._cloud_track.set_point_size(5)
            if init_cloud_track:
                self._cloud_track.set_xyz(self._track_xyz)
                self._cloud_track.set_key(self._track_key[np.newaxis, ...])
            if self.track_visible:
                self._viz.add(self._cloud_track)  # type: ignore
                self._viz.add(self._cloud_kf_track)  # type: ignore

    def update_point_size(self, amount: int) -> bool:
        """Change the point size of the MAP/ACCUM point cloud."""
        with self._lock:
            self._cloud_pt_size = int(min(10.0,
                                      max(1.0, self._cloud_pt_size + amount)))
            for acloud in filter(lambda c: c is not None, self._clouds_accum):
                acloud.set_point_size(self._cloud_pt_size)  # type: ignore
            if self._map_enabled and self._cloud_map is not None:
                self._cloud_map.set_point_size(self._cloud_pt_size)
        return True

    def _draw_track(self) -> None:
        """Update the poses TRACK cloud"""
        self._ensure_cloud_track()

        if self._cloud_track is None:
            return

        self._cloud_track.set_xyz(self._track_xyz)
        self._cloud_track.set_key(self._track_key[np.newaxis, ...])

    def _draw_map(self) -> None:
        """Update the MAP cloud"""
        self._ensure_cloud_map()

        if self._cloud_map is None:
            return

        self._cloud_map.set_xyz(self._map_xyz)
        self._cloud_map.set_key(
            self._map_keys[self.active_cloud_mode.name][np.newaxis, ...])

        if ACCUM_DEBUG:
            update_palette = self._update_cloud_palette()
            if update_palette:
                palette_name = update_palette.name
            else:
                palette_name = "None"
            logger.debug("UPDATE CLOUD PALETTE (MAP): %s", palette_name)
            logger.debug("ACTIVE CLOUD PALETTE (MAP): %s",
                        self.active_cloud_palette.name)
            logger.debug("ACTIVE CLOUD MODE (MAP): %s", self.active_cloud_mode.name)

        update_palette = self._update_cloud_palette()
        if update_palette is not None:
            self._cloud_map.set_palette(update_palette.palette)

    @no_type_check
    def _draw_accum(self) -> None:
        """Update the ACCUM clouds"""
        if self._viz is None:
            return

        for i, dirty in enumerate(self._key_frames_dirty):
            if not dirty:
                continue
            key_frame_idx = self._key_frames[i]
            if key_frame_idx is None:
                # remove cloud to pool if it's present
                if self._clouds_accum[i] is not None:
                    self._viz.remove(self._clouds_accum[i])
                    self._clouds_accum_pool.append(
                        self._clouds_accum[i])
                    self._clouds_accum[i] = None
            else:
                # add/update the cloud
                sr = self._scan_records[key_frame_idx]
                ls = sr.scan[self._sensor_idx]
                # add cloud
                if self._clouds_accum[i] is None:
                    if self._clouds_accum_pool:
                        self._clouds_accum[i] = self._clouds_accum_pool.pop()
                        self._clouds_accum[i].set_point_size(
                            self._cloud_pt_size)
                    else:
                        # create new Cloud
                        self._clouds_accum[i] = Cloud(
                            self._metas[self._sensor_idx])
                        self._clouds_accum[i].set_point_size(
                            self._cloud_pt_size)
                    if self.accum_visible:
                        self._viz.add(self._clouds_accum[i])
                    self._clouds_accum[i].set_palette(
                        self.active_cloud_palette.palette)
                # update cloud
                logger.debug("clouds: updated idx: %d, for key_frame_idx: %d",
                             i, key_frame_idx)
                self._clouds_accum[i].set_range(ls.field(
                    client.ChanField.RANGE))

                mode_name = self.active_cloud_mode.name
                if sr.cloud_mode_keys.get(mode_name) is None:
                    sr.cloud_mode_keys[
                        mode_name] = self.active_cloud_mode._prepare_data(
                            ls, return_num=0)
                self._clouds_accum[i].set_key(sr.cloud_mode_keys[mode_name])

                self._clouds_accum[i].set_column_poses(ls.pose)
            self._key_frames_dirty[i] = 0

        if not self.key_frames_num:
            # no accumulated clouds present
            return

        self._cloud_kf_track.set_xyz(self._kf_track_xyz)
        self._cloud_kf_track.set_key(self._kf_track_key[np.newaxis, ...])

        update_palette = self._update_cloud_palette()
        logger.debug("Update palette (draw_accum): %s",
                     (update_palette.name if update_palette else None))
        if (update_palette is not None or
                self._cloud_mode_ind_prev != self._active_cloud_mode_ind):
            logger.debug("Update colors to: %s", self.active_cloud_mode.name)
            mode_name = self.active_cloud_mode.name
            for acloud, kf_idx in zip(self._clouds_accum, self._key_frames):
                if acloud and kf_idx is not None:
                    sr = self._scan_records[kf_idx]
                    ls = sr.scan[self._sensor_idx]
                    # generate color keys if they are not yet cached in
                    # mode_keys
                    if sr.cloud_mode_keys.get(mode_name) is None:
                        sr.cloud_mode_keys[
                            mode_name] = self.active_cloud_mode._prepare_data(
                                ls, return_num=0)

                    if self._cloud_mode_ind_prev != self._active_cloud_mode_ind:
                        acloud.set_key(sr.cloud_mode_keys[mode_name])

                    if update_palette is not None:
                        acloud.set_palette(update_palette.palette)

    @no_type_check
    def toggle_mode_accum(self, state: Optional[bool] = None) -> bool:
        """Toggle ACCUM view"""
        with self._lock:
            new_state = (not self._accum_mode_accum
                         if state is None else state)
            if self._accum_mode_accum and not new_state:
                for acloud in self._clouds_accum:
                    if acloud:
                        self._viz.remove(acloud)
            elif not self._accum_mode_accum and new_state:
                for acloud in self._clouds_accum:
                    if acloud:
                        self._viz.add(acloud)
            self._accum_mode_accum = new_state
        return True

    @no_type_check
    def toggle_mode_track(self, state: Optional[bool] = None) -> bool:
        """Toggle TRACK view"""
        with self._lock:
            new_state = (not self._accum_mode_track if state is None else state)
            if self._accum_mode_track and not new_state:
                self._viz.remove(self._cloud_track)
                self._viz.remove(self._cloud_kf_track)
            elif (not self._accum_mode_track and new_state):
                self._viz.add(self._cloud_track)
                self._viz.add(self._cloud_kf_track)
            self._accum_mode_track = new_state
        return True

    @no_type_check
    def toggle_mode_map(self, state: Optional[bool] = None) -> bool:
        """Toggle MAP view"""
        with self._lock:
            new_state = (not self._accum_mode_map if state is None else state)
            if self._accum_mode_map and not new_state:
                self._viz.remove(self._cloud_map)
            elif not self._accum_mode_map and new_state:
                self._viz.add(self._cloud_map)
            self._accum_mode_map = new_state
        return True

    def cycle_cloud_mode(self, *, direction: int = 1) -> bool:
        """Change the coloring mode of the point cloud for MAP/ACCUM clouds"""
        with self._lock:
            self._cloud_mode_ind = (self._cloud_mode_ind + direction)
            self._cloud_palette_refl_mode = False
            # update internal states immediately so the OSD text of scans accum
            # is switched already to a good state (needed for LidarScanViz osd
            # update)
            self._update_cloud_palette()
        return True

    def cycle_cloud_palette(self, *, direction: int = 1) -> bool:
        """Change the color palette of the point cloud for MAP/ACCUM clouds"""
        with self._lock:
            npalettes = len(self._cloud_palettes)
            self._cloud_palette_ind = (self._cloud_palette_ind + direction +
                                       npalettes) % npalettes
            self._cloud_palette_refl_mode = False
            # update internal states immediately so the OSD text of scans accum
            # is switched already to a good state (needed for LidarScanViz osd
            # update)
            self._update_cloud_palette()
        return True

    def toggle_osd(self, state: Optional[bool] = None) -> bool:
        """Show or hide the on-screen display."""
        with self._lock:
            self._osd_enabled = (not self._osd_enabled
                                 if state is None else state)
        return True

    def osd_text(self) -> str:
        """Prepare OSD text for use in draw_osd or elsewhere."""
        osd_text = ""

        def append_with_nl(s1: str, s2: str):
            if s1 and s2:
                return f"{s1}\n{s2}"
            return s1 or s2

        # Lines like:
        # scan, map accum [6, 7]: ON, OFF
        #     mode [K]: REFLECTIVITY
        #     palette [G]: Cal Ref
        accum_str = ""
        accum_states = []
        if self._kf_max_num > 0:
            accum_str_debug = ""
            if ACCUM_DEBUG:
                accum_str_debug = f" ({self.key_frames_num}/{self._kf_max_num})"
            accum_states.append(
                ("scan", "6",
                 f"ON{accum_str_debug}" if self.accum_visible else "OFF"))
        if self._map_enabled:
            map_str_debug = ""
            if ACCUM_DEBUG:
                map_points = (self._map_xyz.shape[0]
                              if self._map_overflow else self._map_idx)
                map_str_debug = f" ({map_points}{', o' if self._map_overflow else ''})"
            accum_states.append(("map", "7", f"ON{map_str_debug}" if self.map_visible else "OFF"))
        if accum_states:
            accum_names_str = ", ".join([s[0] for s in accum_states])
            accum_keys_str = ", ".join([s[1] for s in accum_states])
            accum_status_str = ", ".join([s[2] for s in accum_states])
            accum_str = f"{accum_names_str} accum [{accum_keys_str}]: {accum_status_str}\n"
            accum_str += f"        mode [K]: {self.active_cloud_mode.name}\n"
            accum_str += f"        palette [G]: {self.active_cloud_palette.name}\n"
            accum_str += f"        point size [J]: {int(self._cloud_pt_size)}"

        osd_text = append_with_nl(osd_text, accum_str)

        # Line like:
        # poses [8]: ON (12, o)
        poses_str = f"poses [8]: {'ON' if self.track_visible else 'OFF'}"
        track_points = (self._track_xyz.shape[0]
                        if self._track_overflow else self._track_idx)
        if ACCUM_DEBUG:
            poses_str += f" ({track_points}{', o' if self._track_overflow else ''})"

        # Lines like (debug only):
        # update_dt: 0.0032 s
        # draw_dt: 0.0002 s
        osd_text = append_with_nl(osd_text, poses_str)
        if ACCUM_DEBUG:
            osd_text = append_with_nl(
                osd_text, f"update dt: {self._last_update_dt:.4f} s")
            osd_text = append_with_nl(osd_text,
                                      f"draw dt: {self._last_draw_dt:.4f} s")

        return osd_text

    def _draw_osd(self):
        """Update on screen display label text"""
        if self._osd_enabled:
            self._osd.set_text(self.osd_text())
        else:
            self._osd.set_text("")

    @no_type_check
    def update(self,
               scan: Union[client.LidarScan,
                           Tuple[Optional[client.LidarScan]]],
               scan_num: Optional[int] = None) -> None:
        """Register the new scan and update the states of TRACK, ACCUM and MAP"""
        t = time.monotonic()
        self._scan: List[Optional[client.LidarScan]] = [scan] if isinstance(
            scan, client.LidarScan) else scan
        if scan_num is not None:
            self._scan_num = scan_num
        else:
            self._scan_num += 1

        logger.debug("update_scan: scan_num = %d", self._scan_num)

        # NOTE: copy() is very very important, without it the whole LidarScan
        #       if captured and not GCed because it's all views all the way down
        scans_pose = [
            client.first_valid_column_pose(s).copy() if s is not None else None
            for s in self._scan
        ]

        if len(self._scan_records) <= self._scan_num:
            self._scan_records.extend(
                [None] * (self._scan_num - len(self._scan_records) + 1))

        if (self._scan_num < len(self._scan_records) and
                self._scan_records[self._scan_num] is not None):
            # skip all processing/updates if we've already seen this scan num
            return

        scan_record = ScanRecord(poses=scans_pose)
        self._scan_records[self._scan_num] = scan_record

        # refine available modes based on the current scan
        ls = self._scan[self._sensor_idx]
        self._available_modes = list(
            filter(lambda midx: self._cloud_modes[midx].enabled(ls),
                   self._available_modes))
        assert self._available_modes, f"No view mode can be selected that" \
            f" works for all scans for the sensor idx: {self._sensor_idx}"

        if not self._poses_detected:
            self._poses_detected = client.poses_present(ls)

        # ====================================================================
        # extract/update scans track data
        self._update_track()

        # ====================================================================
        # extract/update accumulated scans (i.e. key frames) data
        if self._kf_max_num > 0:
            self._update_accum()

        # ====================================================================
        # extract/update map data
        if self._map_enabled:
            self._update_map()

        if ACCUM_DEBUG:
            logger.debug("scan_records: " + ", ".join([
                f"{i}:{sr.scan[0].w}" for i, sr in enumerate(self._scan_records)
                if sr is not None and sr.scan is not None and
                sr.scan[self._sensor_idx] is not None
            ]))

        self._last_update_dt = time.monotonic() - t

    @no_type_check
    def _update_track(self) -> None:
        """Extract and update the scans poses TRACK"""
        with self._lock:
            self._ensure_structs_track()

        sr = self._scan_records[self._scan_num]
        if sr.poses[self._sensor_idx] is not None:
            self._track_xyz[self._track_idx] = sr.poses[
                self._sensor_idx][:3, 3]
            self._track_key[self._track_idx] = self._track_key_color

        self._track_idx += 1

    @no_type_check
    def _update_accum(self) -> None:
        """Update accumulated key frames scans (ACCUM) states"""

        # check is it a key frame
        if not self._is_key_frame():
            return

        logger.debug("== KEY FRAME ======: scan_num = %d", self._scan_num)

        with self._lock:
            # add new key frame
            self._key_frames[self._key_frames_head] = self._scan_num
            self._key_frames_dirty[self._key_frames_head] = 1

            # save scan to the scan_record
            self._scan_records[self._scan_num].scan = self._scan
            self._scan_records[self._scan_num].cloud_mode_keys = dict()

            # add pose to the key frame track
            sr = self._scan_records[self._scan_num]
            self._kf_track_xyz[self._key_frames_head] = sr.poses[
                self._sensor_idx][:3, 3]
            self._kf_track_key[
                self._key_frames_head] = self._kf_track_key_color

            # advance head
            self._key_frames_head = ((self._key_frames_head + 1) %
                                    (self._kf_max_num + 1))

            # if we moved to the tail, clean up old key frame data and advance tail
            if self._key_frames_head == self._key_frames_tail:
                # evict tail
                sr_tail_idx = self._key_frames[self._key_frames_tail]
                if sr_tail_idx is not None:
                    # clean ScanRecords at: self._scan_records[sr_tail_idx]
                    logger.debug("kf remove scan for sr: %d tail: %d eviction",
                                 sr_tail_idx, self._key_frames_tail)
                    self._scan_records[sr_tail_idx].scan = None
                    self._scan_records[sr_tail_idx].cloud_mode_keys = None

                    self._key_frames[self._key_frames_tail] = None
                    self._key_frames_dirty[self._key_frames_tail] = 1

                    self._kf_track_xyz[
                        self._key_frames_tail] = self._track_xyz_init
                    self._kf_track_key[self._key_frames_tail] = np.zeros(4)

                    # advance tail to repare room for the next write to head
                    self._key_frames_tail = ((self._key_frames_tail + 1) %
                                             (self._kf_max_num + 1))

        if ACCUM_DEBUG:
            logger.debug(
                "kframes: head = %d, tail = %d, kf_num = %d, kf_max+1 = %d",
                self._key_frames_head, self._key_frames_tail,
                self.key_frames_num, self._kf_max_num + 1)
            logger.debug("kframes: " + ", ".join(
                ([f"{i}:{v}" for i, v in enumerate(self._key_frames)])))
            logger.debug("key_frames_idx: %s", str(list(self.key_frames_idxs)))
            logger.debug("key_frames_dirty: %s", str(self._key_frames_dirty))

    @no_type_check
    def _is_key_frame(self) -> bool:
        """Key frames selection logic for ACCUM modes

        Keys frames are selected using 2 params:
            _kf_min_dist_m (aka ``--accum-every-m``) - key frame if distance to
              the previous key frame gte than the value
            _kf_min_dist_n (aka ``--accum-every``) - key frame if distance in
              scan number to the previous key frame gte than value

        NOTE: Key frames are not used for overall map view (MAP)
        """

        # any scan is a key frame if it's the first key frame to be added
        if not self.key_frames_num:
            return True

        prev_kf_idx = (self._kf_max_num +
                       self._key_frames_head) % (self._kf_max_num + 1)
        prev_kf_scan_num = self._key_frames[prev_kf_idx]

        # accum every num scans
        if self._kf_min_dist_n > 0 and (abs(self._scan_num - prev_kf_scan_num)
                                        >= self._kf_min_dist_n):
            return True

        # accum every m meters
        if self._kf_min_dist_m > 0:
            prev_kf_sr = self._scan_records[prev_kf_scan_num]
            sr = self._scan_records[self._scan_num]

            dist_to_prev = np.linalg.norm(
                (sr.poses[self._sensor_idx][:3, 3] -
                prev_kf_sr.poses[self._sensor_idx][:3, 3]))
            if (dist_to_prev >= self._kf_min_dist_m):
                return True

        return False

    @no_type_check
    def _update_map(self) -> None:
        """Update the map (MAP) data.

        Extract the select ratio of random points from the current scan.

        The map size if bounded by ``map_max_points``, selected random points
        defined by ratio ``map_select_ratio``, flag that triggers the map
        overwrite from the beginning rather than randomly is
        ``map_overflow_from_start``.
        """
        if self._scan is None:
            return

        ls = self._scan[self._sensor_idx]
        if ls is None:
            return

        # get random xyz points using map select ratio
        sel_flag = ls.field(client.ChanField.RANGE) != 0
        nzi, nzj = np.nonzero(sel_flag)
        nzc = np.random.choice(len(nzi),
                               int(self._map_select_ratio * len(nzi)),
                               replace=False)
        row_sel, col_sel = nzi[nzc], nzj[nzc]
        xyz = self._xyzlut[0](ls.field(client.ChanField.RANGE))
        xyz = pu.dewarp(xyz, column_poses=ls.pose)[row_sel, col_sel]
        xyz_num = xyz.shape[0]

        with self._lock:
            self._ensure_structs_map(xyz_num)

        if not self._map_overflow or self._map_overflow_start:
            idxs = list(range(self._map_idx, self._map_idx + xyz_num))
        else:
            idxs = np.random.choice(self._map_xyz.shape[0],
                                    xyz_num,
                                    replace=False)
        self._map_xyz[idxs] = xyz

        for i in self._available_modes:
            key = self._cloud_modes[i]._prepare_data(ls, return_num=0)[row_sel,
                                                                       col_sel]
            self._map_keys[self._cloud_modes[i].name][idxs] = key

        # remove no longer available modes from map keys
        if len(self._available_modes) < len(self._map_keys):
            d = self._map_keys.keys() - [
                self._cloud_modes[i].name for i in self._available_modes
            ]
            for mk in d:
                del self._map_keys[mk]
        assert len(self._available_modes) == len(self._map_keys), \
            "Scans field types are not uniform" \
            f" for the map/accum sensor idx: {self._sensor_idx}"

        self._map_idx += xyz_num

    @property
    def key_frames_num(self) -> int:
        """Current number of accumulated ACCUM key frames"""
        return (self._key_frames_head - self._key_frames_tail +
                self._kf_max_num + 1) % (self._kf_max_num + 1)

    @property
    @no_type_check
    def key_frames_idxs(self) -> Iterable[int]:
        """Indices of accumulated frames (ACCUM) in ScanRecords list"""
        if self._key_frames_head >= self._key_frames_tail:
            return self._key_frames[self._key_frames_tail:self.
                                    _key_frames_head]
        else:
            return chain(self._key_frames[self._key_frames_tail:],
                         self._key_frames[:self._key_frames_head])

    @property
    def track_visible(self) -> bool:
        """Whether TRACK is visible"""
        return self._accum_mode_track

    @property
    def accum_visible(self) -> bool:
        """Whether accumulated key frames (ACCUM) is visible"""
        return self._accum_mode_accum

    @property
    def map_visible(self) -> bool:
        """Whether overall map (MAP) is visible"""
        return self._accum_mode_map

    @property
    def active_cloud_palette(self) -> CloudPaletteItem:
        """Cloud palette used for ACCUM/MAP clouds"""
        return (self._cloud_palettes[self._active_cloud_palette_ind]
                if not self._cloud_palette_refl_mode else
                self._cloud_calref_palette)

    @property
    def _active_cloud_palette_ind(self) -> int:
        """Cloud palette index used for ACCUM/MAP clouds"""
        self._cloud_palette_ind = self._cloud_palette_ind % len(
            self._cloud_palettes)
        return self._cloud_palette_ind

    def _update_cloud_palette(self) -> Optional[CloudPaletteItem]:
        """Switch cloud palettes states for ACCUM/MAP clouds but not sets it.

        Returns what cloud palette should be set to ACCUM/MAP clouds, None if
        no changes are needed to the cloud palettes to match the active cloud
        mode and coloring options.
        """
        refl_mode = is_norm_reflectivity_mode(self.active_cloud_mode)
        if self._cloud_mode_ind_prev != self._active_cloud_mode_ind:
            refl_mode_prev = is_norm_reflectivity_mode(
                self._cloud_modes[self._cloud_mode_ind_prev])

            if refl_mode_prev and not refl_mode:
                return self.active_cloud_palette
            elif not refl_mode_prev and refl_mode:
                # snap to the Cal.Ref. palette until cloud mode or palette cycled
                self._cloud_palette_refl_mode = True
                return self._cloud_calref_palette

        if (self._cloud_palette_ind_prev != self._active_cloud_palette_ind):
            return self.active_cloud_palette

        return None

    @property
    def active_cloud_mode(self) -> CloudMode:
        """Current color mode of point ACCUM/MAP point clouds"""
        return self._cloud_modes[self._active_cloud_mode_ind]

    @property
    def _active_cloud_mode_ind(self) -> int:
        """Current color mode index of point ACCUM/MAP point clouds"""
        nmodes = len(self._available_modes)
        self._cloud_mode_ind = (self._cloud_mode_ind + nmodes) % nmodes
        return self._available_modes[self._cloud_mode_ind]

    @property
    def metadata(self) -> List[client.SensorInfo]:
        """Metadatas for the displayed sensors."""
        return self._metas

    def _draw(self) -> None:
        t = time.monotonic()
        self._draw_track()
        if self._map_enabled:
            self._draw_map()
        self._draw_accum()
        self._draw_osd()

        # saving the "pen" and palette that we drew everything with
        self._cloud_mode_ind_prev = self._active_cloud_mode_ind
        self._cloud_palette_ind_prev = self._active_cloud_palette_ind

        self._last_draw_dt = time.monotonic() - t

    @no_type_check
    def draw(self, update: bool = True) -> bool:
        """Process and draw the latest state to the screen."""
        with self._lock:
            self._draw()

        if update:
            return self._viz.update()
        else:
            return False

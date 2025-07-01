"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.

Ouster scan accumulation for LidarScanViz
"""

from typing import (Optional, Dict, List, Tuple, Callable, Any)

import threading
from functools import partial

from ouster.sdk._bindings.viz import PointViz, WindowCtx, Label
from .util import push_point_viz_handler
from .model import LidarScanVizModel
from .scans_accumulator import ScansAccumulator
from .map_accumulator import MapAccumulator
from .accumulators_config import LidarScanVizAccumulatorsConfig
from .track import MultiTrack
from .tracks_accumulator import TracksAccumulator
from ouster.sdk.core import LidarScan


class LidarScanVizAccumulators:
    """Accumulate scans, track poses and overall map view
    Every new scan (``LidarScan`` or ``List[Optional[LidarScan]]``) is passed
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
                 model: LidarScanVizModel,
                 point_viz: PointViz,
                 config: LidarScanVizAccumulatorsConfig,
                 lock: threading.Lock):
        """
        Args:
            model: a LidarScanVizModel instance.
            point_viz: the PointViz instance to use for rendering clouds.
            config: a LidarScanVizAccumulatorsConfig that accepts the following keyword args:
                accum_max_num: aka, ``--accum-num``, the maximum number of accumulated
                  (ACCUM) scans to keep
                accum_min_dist_meters: aka, ``--accum-every-m``, the minimum distance
                  between accumulated (ACCUM) key frames
                accum_min_dist_num: aka, ``--accum-every``, the minimum distance in
                  scans between accumulated (ACCUM) key frames
                map_enabled: enable overall map accumulation (MAP) (``--map``)
                map_select_ratio: percent of points to select from the scans to the
                  overall map (MAP), default 0.001
                map_max_points: maximum number of points to keep in overall map (MAP)
                map_overflow_from_start: if True, on map overflow continue writing
                  points from the beginning (as in ring buffer), if False, overwrite
                  points randomly in the existing map
        """
        self._lock = lock
        self._viz = point_viz
        track = MultiTrack(model, config)
        self._track = track
        self._config = config
        self._ma = MapAccumulator(model, point_viz, track._tracks[0], config)
        self._sa = ScansAccumulator(model, point_viz, track)
        self._ta = TracksAccumulator(model, point_viz, track._tracks[0])

        if self._ma._map_enabled:
            self._ma.toggle_visibility(True)
        elif config._accum_max_num > 0:
            self._sa.toggle_visibility(True)
        self._ta.toggle_visibility(True)

        # initialize osd (on screen display text)
        self._osd: Optional[Label] = None
        self._osd_enabled = False

        # callback for any external vizs that need to hookup into the draw update
        # (currently used by LidarScanViz to update the OSD text)
        self._key_press_pre_draw: Callable[[], Any] = lambda: None

        self._osd_enabled = True
        self._osd = Label("", 0, 1, align_right=False)
        self._viz.add(self._osd)
        self._initialize_key_bindings()

        self._cloud_pt_size: float = 1

    def cycle_cloud_mode(self, direction) -> bool:
        with self._lock:
            self._ma.cycle_cloud_mode(direction=direction)
            self._sa.cycle_cloud_mode(direction=direction)
        return True

    def cycle_cloud_palette(self, direction) -> bool:
        with self._lock:
            self._ma.cycle_cloud_palette(direction=direction)
            self._sa.cycle_cloud_palette(direction=direction)
        return True

    def _initialize_key_bindings(self) -> None:
        """Initialize key bindings and key definitions."""

        # TODO[tws] typing
        key_bindings: Dict[Tuple[int, int], Callable[[], bool]] = {
            (ord('J'), 0): partial(self.update_point_size, amount=1),
            (ord('J'), 1): partial(self.update_point_size, amount=-1),
            (ord('K'), 0): partial(self.cycle_cloud_mode, direction=1),
            (ord('K'), 1): partial(self.cycle_cloud_mode, direction=-1),
            (ord('G'), 0): partial(self.cycle_cloud_palette, direction=1),
            (ord('G'), 1): partial(self.cycle_cloud_palette, direction=-1),
            # TODO[UN]: replace with other keys
            (ord('6'), 0): partial(self.toggle_mode_accum),
            (ord('7'), 0): partial(self.toggle_mode_map),
            (ord('8'), 0): partial(self.toggle_mode_track),
        }

        key_definitions: Dict[str, str] = {
            'j / SHIFT+j': "Increase/decrease point size of accumulated clouds or map",
            'k / SHIFT+k': "Cycle point cloud coloring mode of accumulated clouds or map",
            'g / SHIFT+g': "Cycle point cloud color palette of accumulated clouds or map",
            '6': "Toggle scans accumulation view mode (ACCUM)",
            '7': "Toggle overall map view mode (MAP)",
            '8': "Toggle poses/trajectory view mode (TRACK)",
        }
        self._key_definitions = key_definitions

        def handle_keys(self: LidarScanVizAccumulators, ctx: WindowCtx, key: int,
                        mods: int) -> bool:
            if (key, mods) in key_bindings:
                draw = key_bindings[key, mods]()
                if draw:
                    # notify neighbor vizs that need to update on key press re-draw
                    if self._key_press_pre_draw is not None:
                        self._key_press_pre_draw()

                    # draw + _viz.update() inside
                    self.draw()
                else:
                    self._viz.update()
            return True

        push_point_viz_handler(self._viz, self, handle_keys)

    def update_point_size(self, amount: int) -> bool:
        """Change the point size of the MAP/ACCUM point cloud."""
        # TODO[tws] generalize
        with self._lock:
            self._cloud_pt_size = min(10.0, max(1.0, self._cloud_pt_size + amount))
            self._ma.update_point_size(self._cloud_pt_size)
            self._sa.update_point_size(self._cloud_pt_size)
        return True

    def toggle_sensor(self, sensor_idx, state: bool):
        self._sa.toggle_sensor(sensor_idx, state)

    def toggle_mode_accum(self, state: Optional[bool] = None) -> bool:
        """Toggle ACCUM view"""
        with self._lock:
            self._sa.toggle_visibility(state)
        return True

    def toggle_mode_map(self, state: Optional[bool] = None) -> bool:
        """Toggle MAP view"""
        with self._lock:
            self._ma.toggle_visibility(state)
        return True

    def toggle_mode_track(self, state: Optional[bool] = None) -> bool:
        with self._lock:
            self._ta.toggle_visibility(state)
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
        if self._config._accum_max_num > 0:
            accum_states.append(
                ("scan", "6",
                 "ON" if self._sa.accum_visible else "OFF"))
        if self._ma._map_enabled:
            accum_states.append(("map", "7", "ON" if self._ma.map_visible else "OFF"))
        if accum_states:
            active_cloud_mode = self._ma.active_cloud_mode
            active_cloud_palette = self._ma._get_palette_from_active_mode_name(active_cloud_mode)
            accum_names_str = ", ".join([s[0] for s in accum_states])
            accum_keys_str = ", ".join([s[1] for s in accum_states])
            accum_status_str = ", ".join([s[2] for s in accum_states])
            accum_str = f"{accum_names_str} accum [{accum_keys_str}]: {accum_status_str}\n"
            accum_str += f"        mode [K]: {active_cloud_mode}\n"    # TODO[tws] generalize
            accum_str += f"        palette [G]: {active_cloud_palette.name}\n"  # TODO[tws] generalize
            accum_str += f"        point size [J]: {int(self._cloud_pt_size)}"

        osd_text = append_with_nl(osd_text, accum_str)

        poses_str = f"poses [8]: {'ON' if self._ta.track_visible else 'OFF'}"
        osd_text = append_with_nl(osd_text, poses_str)
        return osd_text

    def _draw_osd(self):
        """Update on screen display label text"""
        if self._osd_enabled:
            self._osd.set_text(self.osd_text())
        else:
            self._osd.set_text("")

    def update(self,
               scans: List[Optional[LidarScan]],
               scan_num: Optional[int] = None) -> None:
        """
        Updates the accumulation state from the current scan. Locking is necessary here because the accumulation state
        depends on the current view mode, which might change in a separate thread than the thread that calls update().
        """
        with self._lock:
            self._ma._use_default_view_modes()
            self._sa._use_default_view_modes()
            self._track.update(scans, scan_num)
            if self._ma._map_enabled:
                self._ma.update(scans, scan_num)
            self._sa.update(scans, scan_num)
            self._ta.update(scans, scan_num)

    def _draw(self) -> None:
        self._ta._draw_track()
        if self._ma.map_visible:
            self._ma._draw_map()
        self._draw_osd()

    # TODO[tws] likely remove; realistically we only need one lock and LidarScanViz should manage it
    def draw(self, update: bool = True) -> None:
        """Process and draw the latest state to the screen."""
        with self._lock:
            self._draw()

        if update:
            self._viz.update()

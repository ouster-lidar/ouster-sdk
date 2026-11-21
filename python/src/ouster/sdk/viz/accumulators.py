"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.

Ouster frame accumulation for LidarFrameViz
"""

from typing import Optional, List

import threading

from ouster.sdk._bindings.viz import PointViz, Label
from ouster.sdk.viz.model import LidarFrameVizModel
from ouster.sdk.viz.frames_accumulator import FramesAccumulator
from ouster.sdk.viz.map_accumulator import MapAccumulator
from ouster.sdk.viz.accumulators_config import LidarFrameVizAccumulatorsConfig
from ouster.sdk.viz.track import MultiTrack
from ouster.sdk.viz.tracks_accumulator import TracksAccumulator
from ouster.sdk.core import FrameSet
from ouster.sdk.viz.util import VizConfig


class LidarFrameVizAccumulators:
    """Accumulate frames, track poses and overall map view
    Every new frame (``LidarFrame`` or ``FrameSet``) is passed
    through ``update(frame, num)``.

    Available visualization depends on whether poses are present or not
    and params set on init. View modes are combination of:

       * **TRACK** - frame poses (i.e. trajectories) of every frame "seen" (poses
         required)
       * **ACCUM** - set of accumulated frames (key frames) picked according to
         params
       * **MAP** - overall map with select ratio of random points from every
         frame passed through ``update()``
    """

    def __init__(self,
                 model: LidarFrameVizModel,
                 point_viz: PointViz,
                 config: LidarFrameVizAccumulatorsConfig,
                 lock: threading.RLock,
                 options: Optional[VizConfig]):
        """
        Args:
            model: a LidarFrameVizModel instance.
            point_viz: the PointViz instance to use for rendering clouds.
            config: a LidarFrameVizAccumulatorsConfig that accepts the following keyword args:
                accum_max_num: aka, ``--accum-num``, the maximum number of accumulated
                  (ACCUM) frames to keep
                accum_min_dist_meters: aka, ``--accum-every-m``, the minimum distance
                  between accumulated (ACCUM) key frames
                accum_min_dist_num: aka, ``--accum-every``, the minimum distance in
                  frames between accumulated (ACCUM) key frames
                map_enabled: enable overall map accumulation (MAP) (``--map``)
                map_select_ratio: percent of points to select from the frames to the
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
        self._ma = MapAccumulator(model, point_viz, track.primary_track, config)
        self._sa = FramesAccumulator(model, point_viz, track)
        self._track_accums: List[TracksAccumulator] = [
            TracksAccumulator(model, point_viz, t) for t in track.display_tracks()
        ]
        self._ta = self._track_accums[0]
        self._options = options

        if self._ma._map_enabled:
            self._ma.toggle_visibility(True)
        elif config._accum_max_num > 0:
            self._sa.toggle_visibility(True)
        self._ta.toggle_visibility(True)

        # initialize osd (on screen display text)
        self._osd: Optional[Label] = None
        self._osd_enabled = False

        self._osd_enabled = True
        self._osd = Label("", 0, 1, align_right=False)
        self._viz.add(self._osd)

        self._cloud_pt_size: float = 1

        self._initialize_key_bindings()

    @property
    def track_accumulator(self) -> TracksAccumulator:
        """Get the TracksAccumulator instance."""
        return self._ta

    def set_cloud_mode(self, mode: str):
        with self._lock:
            self._ma.set_cloud_mode(mode)
            self._sa.set_cloud_mode(mode)

    def set_cloud_palette(self, palette: str):
        with self._lock:
            self._ma.set_cloud_palette(palette)
            self._sa.set_cloud_palette(palette)

    def _initialize_key_bindings(self) -> None:
        """Initialize key bindings and key definitions."""
        if self._options is None:
            return
        self._options.option("map_point_size", self._cloud_pt_size, ('J', 0),
                             lambda arg, val: arg._frames_accum.set_point_size(val),
                             decrement=('J', 1), min=1, max=10,
                             help="Increase/decrease point size of accumulated clouds or map")
        self._options.option("view_accum", self._config._accum_max_num > 0, ('6',),
                             lambda arg, val: arg._frames_accum.toggle_mode_accum(val),
                             help="Toggle frames accumulation view mode (ACCUM)")
        self._options.option("map_accum", self._ma._map_enabled, ('7',),
                             lambda arg, val: arg._frames_accum.toggle_mode_map(val),
                             help="Toggle overall map view mode (MAP)")
        self._options.option("frames_accum_return_0", True, ('1', 4),
                             lambda arg, val: arg._frames_accum.toggle_return(0, val),
                             help="Toggle frames accumulation first return (ACCUM)")
        self._options.option("frames_accum_return_1", True, ('2', 4),
                             lambda arg, val: arg._frames_accum.toggle_return(1, val),
                             help="Toggle frames accumulation second return (ACCUM)")
        self._options.option("frame_accum_mode", "", ('K', 0),
                             lambda arg, val: arg._frames_accum.set_cloud_mode(val),
                             decrement=('K', 1),
                             items=lambda arg: arg._model.sorted_cloud_mode_names(),
                             get_value=lambda arg: arg._frames_accum._ma._cloud_mode_name,
                             help="Cycle point cloud coloring mode of accumulated clouds or map")

        def get_palette_value(arg):
            index = arg._frames_accum._ma._cloud_palette_ind
            return arg._model._palettes._cloud_palettes[index].name

        def get_palette_list(arg):
            return [p.name for p in arg._model._palettes._cloud_palettes]

        self._options.option("frame_accum_palette", "", ('G', 0),
                             lambda arg, val: arg._frames_accum.set_cloud_palette(val),
                             decrement=('G', 1),
                             items=get_palette_list,
                             get_value=get_palette_value,
                             help="Cycle point cloud color palette of accumulated clouds or map")

    def set_point_size(self, size: int) -> bool:
        """Change the point size of the MAP/ACCUM point cloud."""
        # TODO[tws] generalize
        with self._lock:
            self._cloud_pt_size = size
            self._ma.update_point_size(self._cloud_pt_size)
            self._sa.update_point_size(self._cloud_pt_size)
        return True

    def toggle_sensor(self, sensor_idx, state: bool):
        self._sa.toggle_sensor(sensor_idx, state)

    def toggle_return(self, return_idx, state: Optional[bool] = None) -> bool:
        with self._lock:
            self._sa.toggle_return(return_idx, state)
        return True

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
            for ta in self._track_accums:
                ta.toggle_visibility(state)
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
        # frame, map accum [6, 7]: ON, OFF
        #     mode [K]: REFLECTIVITY
        #     palette [G]: Cal Ref
        accum_str = ""
        accum_states = []
        if self._config._accum_max_num > 0:
            stats = ("ON" if self._sa.accum_visible else "OFF")
            accum_states.append(("frame", "6", stats))
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
        if self._config._accum_max_num:
            enabled = self._sa._return_enabled
            accum_str += "\n        frame return ["
            accum_str += ", ".join(["ALT+" + str(i + 1) for i in range(len(enabled))]) + "] "
            accum_str += ", ".join([("ON" if enabled[k] else "OFF") for k in enabled])

        return append_with_nl(osd_text, accum_str)

    def _draw_osd(self):
        """Update on screen display label text"""
        if self._osd_enabled:
            self._osd.set_text(self.osd_text())
        else:
            self._osd.set_text("")

    def clear_track(self) -> None:
        self._track.clear()

    def update(self,
               frames: FrameSet,
               frame_num: Optional[int] = None) -> None:
        """
        Updates the accumulation state from the current frame. Locking is necessary here because the accumulation state
        depends on the current view mode, which might change in a separate thread than the thread that calls update().
        """
        with self._lock:
            self._ma._use_default_view_modes()
            self._sa._use_default_view_modes()
            self._track.update(frames, frame_num)
            if self._ma._map_enabled:
                self._ma.update(frames, frame_num)
            self._sa.update(frames, frame_num)
            self._ta.update(frames, frame_num)

    def _draw(self) -> None:
        for ta in self._track_accums:
            ta._draw_track()
        if self._ma.map_visible:
            self._ma._draw_map()
        self._draw_osd()

    # TODO[tws] likely remove; realistically we only need one lock and LidarFrameViz should manage it
    def draw(self, update: bool = True) -> None:
        """Process and draw the latest state to the screen."""
        with self._lock:
            self._draw()

        if update:
            self._viz.update()

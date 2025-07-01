import numpy as np
from typing import Dict, Optional, List, Union
from ouster.sdk.core import LidarScan, ChanField
from ouster.sdk.core import dewarp
from ouster.sdk._bindings.viz import Cloud, PointViz
from ouster.sdk.viz.accum_base import AccumulatorBase
from ouster.sdk.viz.model import LidarScanVizModel
from ouster.sdk.viz.track import Track, TRACK_MAP_GROWTH_RATE
from ouster.sdk.viz.view_mode import CloudPaletteItem
from ouster.sdk.viz.accumulators_config import LidarScanVizAccumulatorsConfig

MAP_INIT_POINTS_NUM: int = 10000


class MapAccumulator(AccumulatorBase):
    """Used by LidarScanVizAccumulators to display a point cloud that is produced by all scans in the source data."""
    def __init__(self, model: LidarScanVizModel, point_viz: PointViz, track: Track, config:
    LidarScanVizAccumulatorsConfig):

        super().__init__(model, point_viz, track)

        # TODO[tws] consider refactoring or removing _map_enabled, either by making it an attribute of base or by
        # using optional instances of MapAccumulator and ScansAccumulator to indicate
        # enabled status
        self._map_enabled = config._map_enabled
        self._accum_mode_map = self._map_enabled  # TODO[tws] kind of redundant
        self._map_select_ratio = config._map_select_ratio
        self._map_max_points = config._map_max_points
        self._map_overflow_start = config._map_overflow_from_start

        # initialize MAP structs
        map_init_points_num = MAP_INIT_POINTS_NUM if self._map_enabled else 0
        map_init_points_num = min(self._map_max_points, map_init_points_num)
        self._map_xyz = np.zeros((map_init_points_num, 3),
                                 dtype=np.float32,
                                 order='F')
        # calculated color keys for every map point, indexed by cloud mode name
        self._map_keys: Dict[str, np.ndarray] = {}
        self._map_idx = 0
        self._map_overflow = False

        # viz.Cloud for map points
        self._cloud_map: Optional[Cloud] = None
        self._ensure_cloud_map()

    def toggle_visibility(self, state: Optional[bool] = None):
        new_state = (not self._accum_mode_map if state is None else state)
        if self._accum_mode_map and not new_state:
            assert self._cloud_map
            self._viz.remove(self._cloud_map)
        elif not self._accum_mode_map and new_state:
            assert self._cloud_map
            self._viz.add(self._cloud_map)
        self._accum_mode_map = new_state

    def _ensure_cloud_map(self) -> None:
        """Create/re-create the cloud MAP object"""
        def make_cloud_map(n: int):
            self._cloud_map = Cloud(n)
            self._cloud_map.set_point_size(self._cloud_pt_size)
            current_palette = self._get_palette_from_active_mode_name(self.active_cloud_mode)
            if current_palette:
                self._cloud_map.set_palette(current_palette.palette)
            if self.map_visible:
                self._viz.add(self._cloud_map)

        pnum = self._map_xyz.shape[0]
        if not self._cloud_map:
            make_cloud_map(pnum)
        elif self._cloud_map.size < pnum:
            self._viz.remove(self._cloud_map)
            make_cloud_map(pnum)

    def _ensure_structs_map(self, xyz_size: int = 1) -> None:
        """Check map idx and array sizes and increase if needed"""
        if (not self._map_overflow and
                self._map_idx + xyz_size > self._map_xyz.shape[0]):
            new_size = min(
                self._map_max_points,
                int((self._map_xyz.shape[0] + xyz_size) *
                    TRACK_MAP_GROWTH_RATE))
            map_xyz = np.zeros((new_size, 3), dtype=np.float32, order='F')
            map_xyz[:self._map_xyz.shape[0]] = self._map_xyz
            self._map_xyz = map_xyz

            for map_key_name, map_key in self._map_keys.items():
                new_map_key = np.zeros(new_size, dtype=np.float32)
                new_map_key[:map_key.shape[0]] = map_key
                self._map_keys[map_key_name] = new_map_key
                del map_key

            self._map_overflow = (self._map_idx + xyz_size > new_size)

        # reset map_idx from the beginning if there is no space for all
        # incoming points and map_overflow_start is enabled
        if (self._map_overflow_start and
                self._map_idx + xyz_size > self._map_xyz.shape[0]):
            self._map_idx = 0

    @property
    def map_visible(self) -> bool:
        """Whether overall map (MAP) is visible"""
        return self._accum_mode_map

    def _update_map(self) -> None:
        """Update the map (MAP) data.

        Extract the select ratio of random points from the current scan.

        The map size if bounded by ``map_max_points``, selected random points
        defined by ratio ``map_select_ratio``, flag that triggers the map
        overwrite from the beginning rather than randomly is
        ``map_overflow_from_start``.
        """
        assert len(self._scan) == len(self._model._sensors)
        for sensor, scan in zip(self._model._sensors, self._scan):
            if scan is None:
                return

            # get random xyz points using map select ratio
            range_field = scan.field(ChanField.RANGE)
            sel_flag = range_field != 0
            nzi, nzj = np.nonzero(sel_flag)
            nzc = np.random.choice(len(nzi),
                                   min(int(self._map_select_ratio * len(nzi)), self._map_max_points),
                                   replace=False)
            row_sel, col_sel = nzi[nzc], nzj[nzc]
            xyz = sensor._xyzlut(range_field)
            xyz = dewarp(xyz, scan.pose)[row_sel, col_sel]
            xyz_num = xyz.shape[0]

            # TODO[tws] previously protected by "with self._lock" - do we still need this?
            self._ensure_structs_map(xyz_num)

            idxs: Union[List[int], np.ndarray]
            if not self._map_overflow or self._map_overflow_start:
                idxs = list(range(self._map_idx, self._map_idx + xyz_num))
            else:
                idxs = np.random.choice(self._map_xyz.shape[0],
                                        xyz_num,
                                        replace=False)
            self._map_xyz[idxs] = xyz

            for mode in sensor._cloud_modes.values():
                field_colors = mode._prepare_data(scan, return_num=0)
                if field_colors is not None:
                    key = field_colors[row_sel, col_sel]
                    if mode.name not in self._map_keys:
                        self._map_keys[mode.name] = np.zeros(self._map_xyz.shape[0], dtype=np.float32)
                    self._map_keys[mode.name][idxs] = key

            self._map_idx += xyz_num

    def update(self,
               scan: List[Optional[LidarScan]],
               scan_num: Optional[int] = None) -> None:
        super().update(scan, scan_num)
        self._update_map()

    def _update_cloud_mode(self):
        self._draw_map()

    def _get_palette_from_active_mode_name(self, mode_name: str) -> CloudPaletteItem:
        if mode_name == ChanField.REFLECTIVITY or \
            mode_name == ChanField.REFLECTIVITY2:
            current_palette = self._model._palettes._refl_cloud_palettes[self._cloud_palette_ind]
        else:
            current_palette = self._model._palettes._cloud_palettes[self._cloud_palette_ind]
        return current_palette

    def _update_cloud_palette(self) -> None:
        # FIXME[tws] the CloudMode only applies to the sensor that was used to it
        # which probably means we need to use separate clouds per sensor anyway
        # This is wrong because
        # 1. the sensor may not have the mode
        # 2. the is_norm_reflectivity_mode may be inconsistent across sensors for the REFLECTIVITY mode
        # So for now we're just assuming that all sensors have valid calibrated reflectivity.
        current_palette = self._get_palette_from_active_mode_name(self.active_cloud_mode)
        if not current_palette:
            return

        self._cloud_palette_prev: Optional[CloudPaletteItem]
        if self._cloud_palette_prev is None or self._cloud_palette_prev.name != current_palette.name:
            self._cloud_palette_prev = current_palette
            if self._cloud_map:
                self._cloud_map.set_palette(current_palette.palette)

    def update_point_size(self, point_size: float):
        self._cloud_pt_size = point_size
        if self._cloud_map:
            self._cloud_map.set_point_size(self._cloud_pt_size)

    def _draw_map(self) -> None:
        """Update the map cloud"""
        self._ensure_cloud_map()

        if self._cloud_map is None:
            return

        self._cloud_map.set_xyz(self._map_xyz)
        if self.active_cloud_mode in self._map_keys:
            self._cloud_map.set_key(
                self._map_keys[self.active_cloud_mode][np.newaxis, ...])
        self._update_cloud_palette()

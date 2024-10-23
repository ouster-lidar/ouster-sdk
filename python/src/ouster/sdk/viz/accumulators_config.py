"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.

Ouster scan accumulation for LidarScanViz
"""

MAP_MAX_POINTS_NUM: int = 1500000  # 1.5 M default
MAP_SELECT_RATIO: float = 0.001


class LidarScanVizAccumulatorsConfig:
    """Represents configuration for AccumulatorBase implementations used within
    LidarScanVizAccumulators."""
    def __init__(self,
        accum_max_num: int = 0,
        accum_min_dist_meters: float = 0,
        accum_min_dist_num: int = 0,
        map_enabled: bool = False,
        map_select_ratio: float = MAP_SELECT_RATIO,
        map_max_points: int = MAP_MAX_POINTS_NUM,
        map_overflow_from_start: bool = False):

        self._accum_max_num = accum_max_num
        self._accum_min_dist_meters = accum_min_dist_meters
        self._accum_min_dist_num = accum_min_dist_num
        self._map_enabled = map_enabled
        self._map_select_ratio = map_select_ratio
        self._map_max_points = map_max_points
        self._map_overflow_from_start = map_overflow_from_start

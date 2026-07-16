from typing import Any, Iterable, Optional

import numpy as np
from ouster.sdk.viz import Cloud

from ouster.cli.plugins.source_util import SourceCommandContext


class SlamLocalMapViz:
    """Render SLAM's diagnostic local map alongside the frame visualizer."""

    def __init__(self, ctx: SourceCommandContext, simple_viz: Any) -> None:
        self._ctx = ctx
        self._simple_viz = simple_viz
        self._cloud: Optional[Cloud] = None
        self._cloud_size = -1
        self._last_frame_id = -1
        self._point_size = 1.0

        self._install_key_bindings()

    def wrap(self, frame_sets: Iterable[Any]) -> Iterable[Any]:
        for frame_set in frame_sets:
            self.update()
            yield frame_set

    def update(self) -> None:
        local_map_update = self._ctx.slam_local_map_viz_points
        if local_map_update is None:
            return

        frame_id, points = local_map_update
        if frame_id == self._last_frame_id:
            return
        self._last_frame_id = frame_id

        point_count = len(points)
        if point_count == 0:
            self._remove_cloud()
            self._cloud_size = 0
            return

        cloud = self._cloud
        if self._cloud_size != point_count or cloud is None:
            self._remove_cloud()
            cloud = Cloud(point_count)
            cloud.set_point_size(self._point_size)
            cloud.set_key(self._color_key(point_count))
            self._simple_viz._viz.add(cloud)
            self._cloud = cloud
            self._cloud_size = point_count

        cloud.set_xyz(points)

    def _remove_cloud(self) -> None:
        if self._cloud is not None:
            self._simple_viz._viz.remove(self._cloud)
            self._cloud = None

    def _color_key(self, point_count: int) -> np.ndarray:
        return np.full((point_count, 3), (0, 230, 255), dtype=np.uint8)

    def _install_key_bindings(self) -> None:
        if hasattr(self._simple_viz._frame_viz, "_key_definitions"):
            self._simple_viz._frame_viz._key_definitions['j / SHIFT+j'] = (
                "Increase/decrease point size of accumulated clouds, map, or "
                "SLAM local map")

        self._simple_viz._viz.push_key_handler(self._handle_keys)

    def _handle_keys(self, _ctx: object, key: int, mods: int) -> bool:
        if (key, mods) == (ord('J'), 0):
            self._update_point_size(1)
        elif (key, mods) == (ord('J'), 1):
            self._update_point_size(-1)
        return True

    def _update_point_size(self, amount: int) -> None:
        self._point_size = min(10.0, max(1.0, self._point_size + amount))
        if self._cloud is not None:
            self._cloud.set_point_size(self._point_size)
        self._simple_viz._viz.set_notification(
            f"SLAM local map point size: {self._point_size:g}")
        self._simple_viz._viz.update()

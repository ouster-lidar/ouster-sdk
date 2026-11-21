from typing import Optional, Sequence
from ouster.sdk.viz import Cloud, PointViz
from ouster.sdk.viz.model import LidarFrameVizModel
from ouster.sdk.viz.track import Track
import ouster.sdk.core as core
from ouster.sdk.core import ChanField, FrameSet
from ouster.sdk.viz.view_mode import (CloudMode, CloudPaletteItem, is_norm_reflectivity_mode)


class AccumulatorBase:
    """The base of any class used in LidarFrameVizAccumulators that displays data accumulated from frames."""
    def __init__(self, model: LidarFrameVizModel, point_viz: PointViz, track: Track):
        self._frame_num = 1
        self._viz = point_viz
        self._model = model
        self._track = track
        self._metas = model.metadata

        # init view cloud mode toggle
        self._cloud_mode_name: str = ''

        # init cloud palette toggle
        self._cloud_palette_ind = 0
        self._cloud_palette_prev: Optional[CloudPaletteItem] = \
            self._model._palettes._refl_cloud_palettes[self._cloud_palette_ind]

        self._cloud_pt_size: float = 1.0

        # frame poses track
        self._cloud_track: Optional[Cloud] = None

    @property
    def metadata(self) -> Sequence[core.SensorInfo]:
        """Metadata for every displayed sensor, in FrameSet order."""
        return self._metas

    def get_palette(self, cloud_mode: CloudMode) -> CloudPaletteItem:
        # TODO[tws] deduplicate the palette selection code with that in Palettes
        refl_mode = is_norm_reflectivity_mode(cloud_mode)
        if refl_mode:
            return self._model._palettes._refl_cloud_palettes[self._cloud_palette_ind]
        else:
            return self._model._palettes._cloud_palettes[self._cloud_palette_ind]

    @property
    def active_cloud_mode(self) -> str:
        """Name of current color mode of point ACCUM/MAP point clouds"""
        return self._cloud_mode_name

    def _update_cloud_mode(self):
        pass

    def _update_cloud_palette(self):
        pass

    def set_cloud_mode(self, mode: str):
        """Change the coloring mode of the point cloud for MAP/ACCUM clouds"""
        self._cloud_mode_name = mode
        self._update_cloud_mode()

    def set_cloud_palette(self, palette: str):
        """Change the color palette of the point cloud for MAP/ACCUM clouds"""
        names = [p.name for p in self._model._palettes._cloud_palettes]
        self._cloud_palette_ind = names.index(palette)
        self._update_cloud_palette()

    # TODO: likely refactor to de-duplicate logic (but not state) with LidarFrameVizModel
    def _use_default_view_modes(self):
        """Try to set the image and cloud view modes to sensible defaults, depending on what
        fields have been discovered via _amend_view_modes_all.

        If no sensible default can be found, the method will
        pick the view modes associated with the first fields.
        """
        if self._cloud_mode_name:
            # exit early if the view modes are already set
            return

        sorted_cloud_mode_names = self._model.sorted_cloud_mode_names()
        if sorted_cloud_mode_names == ["RING", "SENSOR", "TIMESTAMP"]:
            return

        if "RGB" in sorted_cloud_mode_names:
            preferred_field = "RGB"
        elif ChanField.REFLECTIVITY in sorted_cloud_mode_names:
            preferred_field = ChanField.REFLECTIVITY
        else:
            preferred_field = sorted_cloud_mode_names[0]
        self._cloud_mode_name = preferred_field

        self._update_cloud_palette()

    def update(self,
               frame: FrameSet,
               frame_num: Optional[int] = None) -> None:
        """Register the new frame and update the states of TRACK, ACCUM and MAP"""
        self._frame = frame
        if frame_num is not None:
            self._frame_num = frame_num
        else:
            self._frame_num += 1

    def toggle_visibility(self, state: Optional[bool] = None):
        pass

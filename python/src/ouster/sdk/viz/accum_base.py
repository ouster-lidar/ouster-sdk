from typing import List, Optional
from ouster.sdk.viz import Cloud, PointViz
from ouster.sdk.viz.model import LidarScanVizModel
from ouster.sdk.viz.track import Track
import ouster.sdk.core as core
from ouster.sdk.core import ChanField
from .view_mode import (CloudMode, CloudPaletteItem, is_norm_reflectivity_mode)


class AccumulatorBase:
    """The base of any class used in LidarScanVizAccumulators that displays data accumulated from scans."""
    def __init__(self, model: LidarScanVizModel, point_viz: PointViz, track: Track):
        self._scan_num = 1
        self._viz = point_viz
        self._model = model
        self._track = track
        self._metas = model.metadata

        # init view cloud mode toggle
        self._cloud_mode_ind = 0
        self._cloud_mode_name: str = ''

        # init cloud palette toggle
        self._cloud_palette_ind = 0
        self._cloud_palette_prev: Optional[CloudPaletteItem] = \
            self._model._palettes._refl_cloud_palettes[self._cloud_palette_ind]

        self._cloud_pt_size: float = 1.0

        # scan poses track
        self._cloud_track: Optional[Cloud] = None

    @property
    def metadata(self) -> List[core.SensorInfo]:
        """Metadatas for the displayed sensors."""
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

    def cycle_cloud_mode(self, *, direction: int = 1):
        """Change the coloring mode of the point cloud for MAP/ACCUM clouds"""
        all_cloud_mode_names = self._model.sorted_cloud_mode_names()
        self._cloud_mode_ind = (self._cloud_mode_ind + direction) % len(all_cloud_mode_names)
        self._cloud_mode_name = all_cloud_mode_names[self._cloud_mode_ind]
        self._update_cloud_mode()

    def cycle_cloud_palette(self, *, direction: int = 1):
        """Change the color palette of the point cloud for MAP/ACCUM clouds"""
        npalettes = len(self._model._palettes._cloud_palettes)
        self._cloud_palette_ind = (self._cloud_palette_ind + direction +
                                   npalettes) % npalettes
        # update internal states immediately so the OSD text of scans accum
        # is switched already to a good state (needed for LidarScanViz osd
        # update)
        self._update_cloud_palette()

    # TODO: likely refactor to de-duplicate logic (but not state) with LidarScanVizModel
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
        if sorted_cloud_mode_names == ["RING"]:
            return

        try:
            preferred_fields = [ChanField.REFLECTIVITY, ChanField.REFLECTIVITY2]
            if ChanField.REFLECTIVITY2 not in self._model._known_fields:
                preferred_fields[1] = ChanField.NEAR_IR
            self._cloud_mode_ind = sorted_cloud_mode_names.index(preferred_fields[0])
        except ValueError:
            # handle a situation where no reflectivity or near ir channels are present
            # which may happen with customized datasets
            self._cloud_mode_ind = 0

        self._cloud_mode_name = sorted_cloud_mode_names[self._cloud_mode_ind]
        self._update_cloud_palette()

    def update(self,
               scan: List[Optional[core.LidarScan]],
               scan_num: Optional[int] = None) -> None:
        """Register the new scan and update the states of TRACK, ACCUM and MAP"""
        self._scan: List[Optional[core.LidarScan]] = [scan] if isinstance(
            scan, core.LidarScan) else scan
        if scan_num is not None:
            self._scan_num = scan_num
        else:
            self._scan_num += 1

    def toggle_visibility(self, state: Optional[bool] = None):
        pass

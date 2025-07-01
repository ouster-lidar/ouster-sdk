"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.
"""

from typing import List, Optional
import ouster.sdk.core as core
from ouster.sdk._bindings.viz import Cloud, PointViz
from ouster.sdk.viz.accum_base import AccumulatorBase
from ouster.sdk.viz.model import LidarScanVizModel, SensorModel
from ouster.sdk.viz.track import ScanRecord, MultiTrack, Track
from ouster.sdk.viz.view_mode import CloudPaletteItem


class SensorClouds:
    """Encapsulates the render state for a single sensor in ScansAccumulator."""
    def __init__(self, viz: PointViz, sensor: SensorModel, track: Track):
        self._viz = viz
        self._sensor = sensor
        self._track = track
        self._cloud_pt_size: float = 1
        # viz.Clouds for accumulated key frames scans (+1 to match the
        # key_frames layout 1-to-1)
        self._clouds_accum: List[Optional[Cloud]] = [None] * (self._track._kf_max_num + 1)
        self._cloud_palette_prev: Optional[CloudPaletteItem] = None

    def remove_cloud(self, i) -> None:
        if self._clouds_accum[i] is not None:
            self._viz.remove(self._clouds_accum[i])
            self._clouds_accum[i] = None

    def add_cloud(self, i, accum_visible: bool, active_cloud_palette: CloudPaletteItem, sr: ScanRecord) -> bool:
        # add cloud
        if self._clouds_accum[i] is None:
            # create new Cloud
            self._clouds_accum[i] = Cloud(self._sensor._meta)
            self._clouds_accum[i].set_point_size(
                self._cloud_pt_size)

            # set cloud range data and pose
            self._clouds_accum[i].set_range(sr.scan.field(
                core.ChanField.RANGE))
            self._clouds_accum[i].set_column_poses(sr.scan.pose)

            if accum_visible:
                self._viz.add(self._clouds_accum[i])
            return True
        return False

    def update_cloud(self, i, active_cloud_mode_name: str, sr: ScanRecord):
        mode = self._sensor._cloud_modes.get(active_cloud_mode_name)
        if not mode:
            return

        if sr.cloud_mode_keys.get(mode.name) is None:
            sr.cloud_mode_keys[
                mode.name] = mode._prepare_data(
                    sr.scan, return_num=0)
        self._clouds_accum[i].set_key(sr.cloud_mode_keys[mode.name])

    def show_clouds(self):
        for acloud in self._clouds_accum:
            if acloud:
                self._viz.add(acloud)

    def hide_clouds(self):
        for acloud in self._clouds_accum:
            if acloud:
                self._viz.remove(acloud)

    def update(self, accum_visible: bool, active_cloud_mode_name: str, active_cloud_palette: CloudPaletteItem,
       force_update: bool = False):
        for i, key_frame_idx in enumerate(self._track._key_frames):
            if key_frame_idx is None or not self._sensor._enabled:
                self.remove_cloud(i)
            else:
                # add/update the cloud
                sr = self._track._scan_records[key_frame_idx]
                if sr:
                    if self.add_cloud(i, accum_visible, active_cloud_palette, sr):
                        self.update_cloud(i, active_cloud_mode_name, sr)
                        self._clouds_accum[i].set_palette(  # type: ignore
                            active_cloud_palette.palette)
                else:
                    # TODO[tws]: maybe unnecessary?
                    self.remove_cloud(i)

    def set_palette(self, active_cloud_palette: CloudPaletteItem):
        if active_cloud_palette != self._cloud_palette_prev:
            for cloud in self._clouds_accum:
                if cloud:
                    cloud.set_palette(active_cloud_palette.palette)
            self._cloud_palette_prev = active_cloud_palette  # TODO[tws] encapsulate at the cloud level?

    def update_point_size(self, point_size: float):
        self._cloud_pt_size = point_size
        for cloud in self._clouds_accum:
            if cloud:
                cloud.set_point_size(self._cloud_pt_size)

    def _update_cloud_mode(self, active_cloud_mode_name: str, palette: CloudPaletteItem):
        for i, key_frame_idx in enumerate(self._track._key_frames):
            if key_frame_idx is None:
                continue
            sr = self._track._scan_records[key_frame_idx]
            cloud = self._clouds_accum[i]
            if sr and cloud is not None:
                self.update_cloud(i, active_cloud_mode_name, sr)
                cloud.set_palette(palette.palette)


class ScansAccumulator(AccumulatorBase):
    """Used by LidarScanVizAccumulators to display every Nth scan or a scan at every K meters."""
    def __init__(self, model: LidarScanVizModel,
            point_viz: PointViz,
            track: MultiTrack):

        self._previous_cloud_mode_name: Optional[str] = None
        super().__init__(model, point_viz, track._tracks[0])  # TODO multitrack
        self._accum_mode_accum = True

        self._sensor_clouds = [
            SensorClouds(point_viz, sensor, track) for sensor, track in zip(model._sensors, track._tracks)
        ]

    def update_point_size(self, point_size: float):
        self._cloud_pt_size = point_size
        for sensor_clouds in self._sensor_clouds:
            sensor_clouds.update_point_size(self._cloud_pt_size)

    def update(self,
               scan: List[Optional[core.LidarScan]],
               scan_num: Optional[int] = None, force_update: bool = False) -> None:
        super().update(scan, scan_num)
        for sensor_clouds in self._sensor_clouds:
            mode = sensor_clouds._sensor._cloud_modes.get(self.active_cloud_mode)
            if mode:
                palette = self.get_palette(mode)
                sensor_clouds.update(
                    self.accum_visible, self.active_cloud_mode, palette, force_update=force_update
                )

    @property
    def accum_visible(self) -> bool:
        """Whether accumulated key frames (ACCUM) is visible"""
        return self._accum_mode_accum

    def _update_cloud_mode(self):
        if self.active_cloud_mode == self._previous_cloud_mode_name:
            return
        for sensor, sensor_clouds in zip(self._model._sensors, self._sensor_clouds):
            mode = sensor._cloud_modes.get(self.active_cloud_mode)
            if mode:
                palette = self.get_palette(mode)
                sensor_clouds._update_cloud_mode(self.active_cloud_mode, palette)
        self._previous_cloud_mode_name = self.active_cloud_mode

    def _update_cloud_palette(self):  # TODO don't return anything
        """Switch the palette for the accumulated scans."""
        # TODO[tws] deduplicate this logic with map accum somehow
        for (sensor, sensor_clouds) in zip(self._model._sensors, self._sensor_clouds):
            cloud_mode = sensor._cloud_modes.get(self.active_cloud_mode)
            if cloud_mode:
                sensor_clouds.set_palette(
                    self.get_palette(cloud_mode)
                )

    def toggle_sensor(self, sensor_idx: int, state: bool):
        sensor_cloud = self._sensor_clouds[sensor_idx]
        if state:
            sensor_cloud.show_clouds()
        else:
            sensor_cloud.hide_clouds()

    def toggle_visibility(self, state: Optional[bool] = None):
        """Toggle or set the visibility of the accumulated scans."""
        new_state = (not self._accum_mode_accum
                     if state is None else state)
        if self._accum_mode_accum and not new_state:
            for sensor_cloud in self._sensor_clouds:
                sensor_cloud.hide_clouds()
        elif not self._accum_mode_accum and new_state:
            for sensor_cloud in self._sensor_clouds:
                sensor_cloud.show_clouds()
        self._accum_mode_accum = new_state

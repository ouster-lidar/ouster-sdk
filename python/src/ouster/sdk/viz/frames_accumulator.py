"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.
"""

from typing import List, Optional
from ouster.sdk.core import CameraFrame, Frame, FrameSet, LidarFrame, dewarp
from ouster.sdk._bindings.viz import Cloud, PointViz
from ouster.sdk.viz.accum_base import AccumulatorBase
from ouster.sdk.viz.model import CameraModel, LidarFrameVizModel, SensorModel
from ouster.sdk.viz.track import FrameRecord, MultiTrack, Track
from ouster.sdk.viz.view_mode import CloudPaletteItem
import numpy as np


class SensorClouds:
    """Encapsulates the render state for a single sensor in FramesAccumulator."""
    def __init__(self, viz: PointViz, sensor: SensorModel, track: Track):
        self._viz = viz
        self._sensor = sensor
        self._track = track
        self._cloud_pt_size: float = 1
        # viz.Clouds for accumulated key frames (+1 to match the
        # key_frames layout 1-to-1)
        self._clouds_accum: List[Optional[List[Cloud]]] = [None] * (self._track._kf_max_num + 1)
        self._cloud_palette_prev: Optional[CloudPaletteItem] = None
        self._return_enabled = [True] * len(sensor._clouds)
        self._shown = True

    def remove_cloud(self, i) -> None:
        if self._clouds_accum[i] is not None:
            for cloud in self._clouds_accum[i]:
                self._viz.remove(cloud)
            self._clouds_accum[i] = None

    def add_cloud(self, i, accum_visible: bool, active_cloud_palette: CloudPaletteItem, frame: Frame) -> bool:
        # add cloud
        if self._clouds_accum[i] is None:
            # create new Clouds
            clouds = []
            if not isinstance(frame, LidarFrame):
                return False
            for ci in range(len(self._sensor._clouds)):
                field_name = "RANGE" if ci < 1 else f"RANGE{ci + 1}"
                if not frame.has_field(field_name):
                    continue

                # Downselect to just valid points to save compute/memory in most cases
                # Also apply dewarping once, rather than every single render
                pts = self._sensor._xyzlut(frame.field(field_name))
                dewarped_pts = dewarp(pts, frame.body_to_world)
                dewarped_pts = dewarped_pts[frame.field(field_name) != 0]

                # maybe do this heuristically? e.g. based on number of valid points
                cloud = Cloud(dewarped_pts.shape[0])
                cloud.set_point_size(self._cloud_pt_size)
                cloud.set_xyz(dewarped_pts)

                if accum_visible and self._return_enabled[ci]:
                    self._viz.add(cloud)
                clouds.append(cloud)

            self._clouds_accum[i] = clouds
            return True
        return False

    def update_cloud(self, i, active_cloud_mode_name: str, frame: Frame):
        mode = self._sensor._cloud_modes.get(active_cloud_mode_name)
        if not mode:
            return
        if not isinstance(frame, LidarFrame):
            return

        for return_num in range(len(self._clouds_accum[i])):
            if mode.enabled(frame, return_num):
                data = mode._prepare_data(frame, return_num=return_num)
            else:
                data = mode._prepare_data(frame, return_num=0)
            assert data is not None
            field_name = "RANGE" if return_num < 1 else f"RANGE{return_num + 1}"
            self._clouds_accum[i][return_num].set_key(data[frame.field(field_name) != 0])

    def show_clouds(self):
        self._shown = True
        for clouds in self._clouds_accum:
            if clouds:
                for ret_idx, cloud in enumerate(clouds):
                    if self._return_enabled[ret_idx]:
                        self._viz.add(cloud)

    def hide_clouds(self):
        self._shown = False
        for clouds in self._clouds_accum:
            if clouds:
                for cloud in clouds:
                    self._viz.remove(cloud)

    def update(self, accum_visible: bool, active_cloud_mode_name: str, active_cloud_palette: CloudPaletteItem,
       force_update: bool = False):
        for i, key_frame_idx in enumerate(self._track._key_frames):
            if key_frame_idx is None or not self._sensor._enabled:
                self.remove_cloud(i)
            else:
                # add/update the cloud
                sr = self._track._frame_records[key_frame_idx]
                if sr and sr.frame:
                    frame = sr.frame
                    if self.add_cloud(i, accum_visible, active_cloud_palette, frame):
                        self.update_cloud(i, active_cloud_mode_name, frame)
                        clouds = self._clouds_accum[i]
                        assert clouds is not None
                        for cloud in clouds:
                            cloud.set_palette(  # type: ignore
                                active_cloud_palette.palette)
                else:
                    # TODO[tws]: maybe unnecessary?
                    self.remove_cloud(i)

    def toggle_return(self, return_idx: int, state: bool):
        if return_idx >= len(self._return_enabled):
            return

        self._return_enabled[return_idx] = state
        for clouds in self._clouds_accum:
            for idx, cloud in enumerate(clouds or []):
                if idx != return_idx or cloud is None:
                    continue
                if state and self._shown:
                    self._viz.add(cloud)
                else:
                    self._viz.remove(cloud)

    def set_palette(self, active_cloud_palette: CloudPaletteItem):
        if active_cloud_palette != self._cloud_palette_prev:
            for clouds in self._clouds_accum:
                if clouds:
                    for cloud in clouds:
                        cloud.set_palette(active_cloud_palette.palette)
            self._cloud_palette_prev = active_cloud_palette  # TODO[tws] encapsulate at the cloud level?

    def update_point_size(self, point_size: float):
        self._cloud_pt_size = point_size
        for clouds in self._clouds_accum:
            if clouds:
                for cloud in clouds:
                    cloud.set_point_size(self._cloud_pt_size)

    def _update_cloud_mode(self, active_cloud_mode_name: str, palette: CloudPaletteItem):
        for i, key_frame_idx in enumerate(self._track._key_frames):
            if key_frame_idx is None:
                continue
            sr = self._track._frame_records[key_frame_idx]
            clouds = self._clouds_accum[i]
            if sr and clouds is not None:
                assert sr.frame is not None
                self.update_cloud(i, active_cloud_mode_name, sr.frame)
                for cloud in clouds:
                    cloud.set_palette(palette.palette)


class CameraClouds:
    """Encapsulates the accumulated key-frame clouds for a single camera stream.

    Camera RANGE frames are projected with the shared ``XYZLut`` (in the camera
    frame) and placed in the world with a per-cloud pose, so accumulated key
    frames line up along the estimated trajectory.
    """
    def __init__(self, viz: PointViz, camera: CameraModel, track: Track):
        self._viz = viz
        self._camera = camera
        self._track = track
        self._cloud_pt_size: float = 1
        self._clouds_accum: List[Optional[Cloud]] = [None] * (self._track._kf_max_num + 1)
        self._black_palette = CloudPaletteItem("Black", np.zeros((256, 3), dtype=np.float32))
        self._shown = True

    def remove_cloud(self, i) -> None:
        if self._clouds_accum[i] is not None:
            self._viz.remove(self._clouds_accum[i])
            self._clouds_accum[i] = None

    def add_cloud(self, i, accum_visible: bool, sr: FrameRecord) -> bool:
        if self._clouds_accum[i] is not None:
            return False
        frame = sr.frame
        if not isinstance(frame, CameraFrame) or not frame.has_field("RANGE"):
            return False
        world_xyz = self._camera.world_xyz(frame)
        if world_xyz is None:
            return False
        cloud = Cloud(world_xyz.shape[0])
        cloud.set_point_size(self._cloud_pt_size)
        cloud.set_xyz(world_xyz)
        if accum_visible:
            self._viz.add(cloud)
        self._clouds_accum[i] = cloud
        return True

    def update_cloud(self, i, active_cloud_mode_name: str, active_cloud_palette: CloudPaletteItem,
                     sr: FrameRecord) -> None:
        cloud = self._clouds_accum[i]
        if cloud is None:
            return
        frame = sr.frame
        mode = self._camera._cloud_modes.get(active_cloud_mode_name)
        if mode is not None and mode.enabled(frame, 0):  # type: ignore[arg-type]
            mode.set_cloud_color(cloud, frame, return_num=0)  # type: ignore[arg-type]
            cloud.set_palette(active_cloud_palette.palette)
        else:
            cloud.set_palette(self._black_palette.palette)

    def update(self, accum_visible: bool, active_cloud_mode_name: str,
               active_cloud_palette: CloudPaletteItem) -> None:
        for i, key_frame_idx in enumerate(self._track._key_frames):
            if key_frame_idx is None or not self._camera._enabled:
                self.remove_cloud(i)
                continue
            sr = self._track._frame_records[key_frame_idx]
            if not sr:
                self.remove_cloud(i)
                continue
            if self.add_cloud(i, accum_visible, sr):
                self.update_cloud(i, active_cloud_mode_name, active_cloud_palette, sr)

    def show_clouds(self) -> None:
        self._shown = True
        for cloud in self._clouds_accum:
            if cloud is not None:
                self._viz.add(cloud)

    def hide_clouds(self) -> None:
        self._shown = False
        for cloud in self._clouds_accum:
            if cloud is not None:
                self._viz.remove(cloud)

    def set_palette(self, active_cloud_palette: CloudPaletteItem) -> None:
        for cloud in self._clouds_accum:
            if cloud is not None:
                cloud.set_palette(active_cloud_palette.palette)

    def update_point_size(self, point_size: float) -> None:
        self._cloud_pt_size = point_size
        for cloud in self._clouds_accum:
            if cloud is not None:
                cloud.set_point_size(self._cloud_pt_size)

    def _update_cloud_mode(self, active_cloud_mode_name: str, palette: CloudPaletteItem) -> None:
        for i, key_frame_idx in enumerate(self._track._key_frames):
            if key_frame_idx is None or self._clouds_accum[i] is None:
                continue
            sr = self._track._frame_records[key_frame_idx]
            if sr:
                self.update_cloud(i, active_cloud_mode_name, palette, sr)


class FramesAccumulator(AccumulatorBase):
    """Used by LidarFrameVizAccumulators to display every Nth frame or a frame at every K meters."""
    def __init__(self, model: LidarFrameVizModel,
            point_viz: PointViz,
            track: MultiTrack):

        self._previous_cloud_mode_name: Optional[str] = None
        super().__init__(model, point_viz, track.primary_track)  # TODO multitrack
        self._accum_mode_accum = True
        self._black_palette = CloudPaletteItem("Black", np.zeros((256, 3), dtype=np.float32))
        self._return_enabled = {}
        self._return_enabled[0] = True

        self._sensor_clouds = [
            SensorClouds(point_viz, sensor, track) for sensor, track in zip(model._sensors, track._tracks)
        ]
        self._camera_clouds = [
            CameraClouds(point_viz, camera, track)
            for camera, track in zip(model._cameras, track._camera_tracks)
        ]

    def update_point_size(self, point_size: float):
        self._cloud_pt_size = point_size
        for sensor_clouds in self._sensor_clouds:
            sensor_clouds.update_point_size(self._cloud_pt_size)
        for camera_clouds in self._camera_clouds:
            camera_clouds.update_point_size(self._cloud_pt_size)

    def update(self,
               frame: FrameSet,
               frame_num: Optional[int] = None, force_update: bool = False) -> None:
        super().update(frame, frame_num)
        for sensor_clouds in self._sensor_clouds:
            mode = sensor_clouds._sensor._cloud_modes.get(self.active_cloud_mode)
            if mode:
                palette = self.get_palette(mode)
                sensor_clouds.update(
                    self.accum_visible, self.active_cloud_mode, palette, force_update=force_update
                )
        for camera_clouds in self._camera_clouds:
            mode = camera_clouds._camera._cloud_modes.get(self.active_cloud_mode)
            palette = self.get_palette(mode) if mode else camera_clouds._black_palette
            camera_clouds.update(self.accum_visible, self.active_cloud_mode, palette)
        for fr in frame:
            if fr is not None and fr.has_field("RANGE2"):
                if 1 not in self._return_enabled:
                    self._return_enabled[1] = True

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
            else:
                # show the cloud as black if chanfield is missing
                sensor_clouds._update_cloud_mode(self.active_cloud_mode, self._black_palette)
        for camera_clouds in self._camera_clouds:
            mode = camera_clouds._camera._cloud_modes.get(self.active_cloud_mode)
            palette = self.get_palette(mode) if mode else self._black_palette
            camera_clouds._update_cloud_mode(self.active_cloud_mode, palette)
        self._previous_cloud_mode_name = self.active_cloud_mode

    def _update_cloud_palette(self):
        """Switch the palette for the accumulated frames."""
        # TODO[tws] deduplicate this logic with map accum somehow
        for (sensor, sensor_clouds) in zip(self._model._sensors, self._sensor_clouds):
            cloud_mode = sensor._cloud_modes.get(self.active_cloud_mode)
            if cloud_mode:
                sensor_clouds.set_palette(
                    self.get_palette(cloud_mode)
                )
            else:
                # show the cloud as black if chanfield is missing
                sensor_clouds.set_palette(self._black_palette)
        for camera_clouds in self._camera_clouds:
            cloud_mode = camera_clouds._camera._cloud_modes.get(self.active_cloud_mode)
            if cloud_mode:
                camera_clouds.set_palette(self.get_palette(cloud_mode))
            else:
                camera_clouds.set_palette(self._black_palette)

    def toggle_sensor(self, sensor_idx: int, state: bool):
        sensor_cloud = self._sensor_clouds[sensor_idx]
        if state:
            sensor_cloud.show_clouds()
        else:
            sensor_cloud.hide_clouds()

    def toggle_return(self, return_idx: int, state: Optional[bool] = None):
        if return_idx not in self._return_enabled:
            return
        new_state = state if state is not None else (not self._return_enabled[return_idx])
        self._return_enabled[return_idx] = new_state
        for sensor_cloud in self._sensor_clouds:
            sensor_cloud.toggle_return(return_idx, new_state)

    def toggle_visibility(self, state: Optional[bool] = None):
        """Toggle or set the visibility of the accumulated frames."""
        new_state = (not self._accum_mode_accum
                     if state is None else state)
        if self._accum_mode_accum and not new_state:
            for sensor_cloud in self._sensor_clouds:
                sensor_cloud.hide_clouds()
            for camera_cloud in self._camera_clouds:
                camera_cloud.hide_clouds()
        elif not self._accum_mode_accum and new_state:
            for sensor_cloud in self._sensor_clouds:
                sensor_cloud.show_clouds()
            for camera_cloud in self._camera_clouds:
                camera_cloud.show_clouds()
        self._accum_mode_accum = new_state

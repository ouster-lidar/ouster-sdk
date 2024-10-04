"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.
"""

import numpy as np
from typing import Optional
from ouster.sdk._bindings.viz import Cloud, PointViz
from ouster.sdk.viz.model import LidarScanVizModel
from ouster.sdk.viz.accum_base import AccumulatorBase
from ouster.sdk.viz.track import Track


class TracksAccumulator(AccumulatorBase):
    """Encapsulates render state for tracks (scan positions) and key frames
    (which are "interesting" scan positions according to the LidarScanVizAccumulatorsConfig.)
    Cloud objects (from the ouster_viz library) are used to render the tracks."""
    DEFAULT_PT_SIZE = 5
    DEFAULT_KF_PT_SIZE = 10

    # TODO[tws] support MultiTrack?
    def __init__(self, model: LidarScanVizModel, point_viz: PointViz, track: Track):
        super().__init__(model, point_viz, track)
        self._accum_mode_track = True

        pnum = self._track._xyz.shape[0]
        self._cloud_track = Cloud(pnum)
        self._cloud_track.set_point_size(self.DEFAULT_PT_SIZE)
        self._cloud_kf_track = Cloud(track._kf_max_num + 1)
        self._cloud_kf_track.set_point_size(self.DEFAULT_KF_PT_SIZE)
        self._cloud_kf_track.set_xyz(track._kf_xyz)
        self._cloud_kf_track.set_key(track._kf_key[np.newaxis, ...])
        self._viz.add(self._cloud_track)
        self._viz.add(self._cloud_kf_track)
        self._ensure_cloud_track()

    @property
    def track_visible(self) -> bool:
        """Returns true if the track is visible."""
        return self._accum_mode_track

    def toggle_visibility(self, state: Optional[bool] = None):
        """Toggle track visibility by adding or removing the Clouds used to render it
        to / from the viz."""
        new_state = (not self._accum_mode_track if state is None else state)
        if self._accum_mode_track and not new_state:
            self._viz.remove(self._cloud_track)
            self._viz.remove(self._cloud_kf_track)
        elif (not self._accum_mode_track and new_state):
            self._viz.add(self._cloud_track)
            self._viz.add(self._cloud_kf_track)
        self._accum_mode_track = new_state

    def _ensure_cloud_track(self) -> None:
        """Create/re-create the Clouds used to render the track in the viz."""
        pnum = self._track._xyz.shape[0]
        if self._cloud_track.size < pnum:
            self._viz.remove(self._cloud_track)
            self._viz.remove(self._cloud_kf_track)
            self._cloud_track = Cloud(pnum)
            self._cloud_track.set_point_size(self.DEFAULT_PT_SIZE)
            self._cloud_track.set_xyz(self._track._xyz)
            self._cloud_track.set_key(self._track._key[np.newaxis, ...])
            if self.track_visible:
                self._viz.add(self._cloud_track)
                self._viz.add(self._cloud_kf_track)

    # TODO[tws] generalize draw method interface
    def _draw_track(self) -> None:
        """Updates the values in the Clouds used to render the track."""
        self._ensure_cloud_track()
        self._cloud_track.set_xyz(self._track._xyz)
        self._cloud_track.set_key(self._track._key[np.newaxis, ...])
        self._cloud_kf_track.set_xyz(self._track._kf_xyz)
        self._cloud_kf_track.set_key(self._track._kf_key[np.newaxis, ...])

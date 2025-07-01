"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.
"""

import numpy as np
import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import ouster.sdk.core as core
from .model import LidarScanVizModel
from .accumulators_config import LidarScanVizAccumulatorsConfig


TRACK_INIT_POINTS_NUM: int = 100
TRACK_MAX_POINTS_NUM: int = 100000
TRACK_MAP_GROWTH_RATE: float = 1.5

logger = logging.getLogger("viz-accum-logger")
if not logger.hasHandlers():
    logger.addHandler(logging.StreamHandler())


@dataclass
class ScanRecord:
    """Represents a scan, its pose, and its color (aka keys) for each available cloud mode."""
    pose: np.ndarray
    scan: core.LidarScan
    cloud_mode_keys: Dict[str, Optional[np.ndarray]] = field(default_factory = lambda: dict())

    @property
    def position(self):
        return self.pose[:3, 3]


class Track:
    """Represents a sequence of scans with their poses and "key frames", which are scans
    representing every Nth scan or every few meters."""
    # TODO[tws] consider moving "key" (e.g. color) arrays to TracksAccumulator
    def __init__(self, config: LidarScanVizAccumulatorsConfig, extrinsics: np.ndarray = np.eye(4)):
        self._scan_num = -1
        self._scan_records: List[Optional[ScanRecord]] = []

        self._kf_min_dist_m = config._accum_min_dist_meters
        self._kf_min_dist_n = config._accum_min_dist_num

        # key frames is an index to self._scan_records data
        self._kf_max_num = config._accum_max_num
        self._key_frames: List[Optional[int]] = [None] * (self._kf_max_num + 1)
        self._key_frames_head: int = 0
        self._key_frames_tail: int = 0

        # initialize TRACK structs
        self._xyz_init = np.array([10000000, 10000000, 10000000], dtype=np.float32)
        self._xyz = np.full((TRACK_INIT_POINTS_NUM, 3),
                          self._xyz_init,
                          dtype=np.float32,
                          order='F')
        self._key = np.zeros((TRACK_INIT_POINTS_NUM, 4),
                          dtype=np.float32)
        self._key_color = np.array([0.9, 0.9, 0.9, 1.0],
                          dtype=np.float32)
        self._track_idx = 0
        self._track_overflow = False

        # accum key frames track (i.e. trajectory points)
        self._kf_xyz = np.full((self._kf_max_num + 1, 3),
                          self._xyz_init,
                          dtype=np.float32,
                          order='F')
        self._kf_key = np.zeros((self._kf_max_num + 1, 4),
                           dtype=np.float32)
        self._kf_key_color = np.array([0.9, 0.9, 0.2, 1.0])
        self._extrinsics = extrinsics

    def _update_track(self) -> None:
        """Extract and update the scans poses TRACK"""
        self._ensure_structs_track()

        sr = self._scan_records[self._scan_num]
        if sr:
            self._xyz[self._track_idx] = sr.position
            self._key[self._track_idx] = self._key_color
        self._track_idx += 1

    def _ensure_structs_track(self) -> None:
        """Check track idx and array sizes and increase if needed"""
        if (self._track_idx >= self._xyz.shape[0] and
                self._xyz.shape[0] < TRACK_MAX_POINTS_NUM):
            new_size = min(
                TRACK_MAX_POINTS_NUM,
                int((self._key.shape[0] + 1) * TRACK_MAP_GROWTH_RATE))
            xyz = np.full((new_size, 3),
                        self._xyz_init,
                        dtype=np.float32,
                        order='F')
            xyz[:self._xyz.shape[0]] = self._xyz
            self._xyz = xyz
            key = np.zeros((new_size, 4), dtype=np.float32)
            key[:self._key.shape[0]] = self._key
            self._key = key

        # overflow of the max track size
        if self._track_idx >= self._key.shape[0]:
            self._track_idx = 0
            self._track_overflow = True

    def _update_accum(self) -> None:
        """Update accumulated key frames scans (ACCUM) states"""

        # check is it a key frame
        if not self._is_key_frame():
            return

        # add new key frame
        self._key_frames[self._key_frames_head] = self._scan_num

        # add pose to the key frame track
        sr = self._scan_records[self._scan_num]
        if sr:
            self._kf_xyz[self._key_frames_head] = sr.position
            self._kf_key[self._key_frames_head] = self._kf_key_color

        # advance head
        self._key_frames_head = ((self._key_frames_head + 1) %
                                (self._kf_max_num + 1))

        # if we moved to the tail, clean up old key frame data and advance tail
        if self._key_frames_head == self._key_frames_tail:
            # evict tail
            sr_tail_idx = self._key_frames[self._key_frames_tail]
            if sr_tail_idx is not None:
                # clean ScanRecords at: self._scan_records[sr_tail_idx]
                self._scan_records[sr_tail_idx] = None
                self._key_frames[self._key_frames_tail] = None

                self._kf_xyz[
                    self._key_frames_tail] = self._xyz_init
                self._kf_key[self._key_frames_tail] = np.zeros(4)

                # advance tail to repare room for the next write to head
                self._key_frames_tail = ((self._key_frames_tail + 1) %
                                         (self._kf_max_num + 1))

    def _is_key_frame(self) -> bool:
        """Returns true if the current scan should be considered a key frame."""
        # any scan is a key frame if it's the first key frame to be added
        if not self.key_frames_num:
            return True

        prev_kf_idx = (self._kf_max_num +
                       self._key_frames_head) % (self._kf_max_num + 1)

        prev_kf_scan_num = self._key_frames[prev_kf_idx]
        assert prev_kf_scan_num is not None

        # accum every num scans
        if self._kf_min_dist_n > 0 and (abs(self._scan_num - prev_kf_scan_num)
                                        >= self._kf_min_dist_n):
            return True

        # accum every m meters
        if self._kf_min_dist_m > 0:
            prev_kf_sr = self._scan_records[prev_kf_scan_num]
            assert prev_kf_sr is not None
            sr = self._scan_records[self._scan_num]
            if sr:
                dist_to_prev = np.linalg.norm(
                    (sr.position -
                    prev_kf_sr.position))
                if (dist_to_prev >= self._kf_min_dist_m):
                    return True

        return False

    @property
    def key_frames_num(self) -> int:
        """Current number of key frames"""
        return (self._key_frames_head - self._key_frames_tail +
                self._kf_max_num + 1) % (self._kf_max_num + 1)

    def update(self,
               scan: Optional[core.LidarScan],
               scan_num: int) -> None:
        """Register the new scan and update the track and key frames"""
        self._scan_num = scan_num
        if len(self._scan_records) <= self._scan_num:
            self._scan_records.extend(
                [None] * (self._scan_num - len(self._scan_records) + 1))

        if (self._scan_num < len(self._scan_records) and
                self._scan_records[self._scan_num] is not None):
            # skip all processing/updates if we've already seen this scan num
            return

        if scan:
            pose = core.last_valid_column_pose(scan) @ self._extrinsics
            self._scan_records[self._scan_num] = ScanRecord(pose=pose, scan=scan)
        self._update_track()
        self._update_accum()


class MultiTrack:
    """Represents scan positions and key frames for multiple sensors."""
    # TODO[tws] can probably just the number of sensors instead of a LidarScanVizModel
    def __init__(self, model: LidarScanVizModel, config: LidarScanVizAccumulatorsConfig):
        self._tracks = [
            Track(config, m.extrinsic) for m in model.metadata
        ]
        self._kf_max_num = config._accum_max_num
        self._scan_num = -1

    def update(self,
               scans: List[Optional[core.LidarScan]],
               scan_num: Optional[int] = None) -> None:
        """Update the Track for each sensor."""
        if scan_num is not None:
            self._scan_num = scan_num
        else:
            self._scan_num += 1

        assert len(scans) == len(self._tracks)
        for scan, track in zip(scans, self._tracks):
            track.update(scan, self._scan_num)

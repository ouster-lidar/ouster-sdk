# type: ignore
"""
# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

Module ouster_kiss_icp

Description:
    This module was a copy from the KissICP repository https://github.com/PRBonn/kiss-icp
    We edit it to fit our use cases better

Author:
    Hao Yuan

Last modified date: Jan 29th 2024

Modifcation History:
    1/29/24 Copy KissICP module and handle the frame drop issue
"""


from collections import deque
import numpy as np
import logging

from kiss_icp.config import KISSConfig
from kiss_icp.deskew import get_motion_compensator
from kiss_icp.mapping import get_voxel_hash_map
from kiss_icp.preprocess import get_preprocessor
from kiss_icp.registration import register_frame
from kiss_icp.threshold import get_threshold_estimator
from kiss_icp.voxelization import voxel_down_sample

import ouster.sdk.util.pose_util as pu

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()


class KissICP:

    def __init__(self, config: KISSConfig):
        self.poses = []
        self.config = config
        self.compensator = get_motion_compensator(config)
        self.adaptive_threshold = get_threshold_estimator(self.config)
        self.local_map = get_voxel_hash_map(self.config)
        self.preprocess = get_preprocessor(self.config)
        self.last_two_ts = deque([], maxlen=2)
        self.frame_drop_ratio_prev = 1

    def _cal_ts_diff_ratio(self, ts) -> float:
        if len(self.last_two_ts) < 2:
            self.last_two_ts.append(ts)
            return 1.0

        ts_diff_prev = self.last_two_ts[-1] - self.last_two_ts[-2]
        if ts_diff_prev == 0 or ts < self.last_two_ts[-1]:
            return 1.0

        ts_diff_cur = ts - self.last_two_ts[-1]
        # calculate frame drop ratio using real timestamps
        frame_drop_ratio = ts_diff_cur / ts_diff_prev

        self.last_two_ts.append(ts)
        return frame_drop_ratio

    def register_frame(self, frame, timestamps, scan_ts):
        # drop out of ordered frame
        if len(self.poses) >= 1 and scan_ts <= self.last_two_ts[-1]:
            logger.info(f"Kiss ICP drops the out of order frame at ts {scan_ts}")
            return

        frame_drop_ratio_curr = self._cal_ts_diff_ratio(scan_ts)

        if len(self.poses) >= 2:
            corrected_poses = [pu.pose_interp(self.poses[-2], self.poses[-1],
                                              t=1 - (1 / self.frame_drop_ratio_prev)), self.poses[-1]]
        else:
            corrected_poses = []

        # Apply motion compensation
        frame = self.compensator.deskew_scan(
            frame, corrected_poses, timestamps)

        # Preprocess the input cloud
        frame = self.preprocess(frame)

        # Voxelize
        source, frame_downsample = self.voxelize(frame)

        # Get motion prediction and adaptive_threshold
        sigma = self.get_adaptive_threshold()

        # Compute initial_guess for ICP
        prediction = self.get_prediction_model(frame_drop_ratio_curr)
        last_pose = self.poses[-1] if self.poses else np.eye(4)
        initial_guess = last_pose @ prediction
        self.frame_drop_ratio_prev = frame_drop_ratio_curr

        # Run ICP
        new_pose = register_frame(
            points=source,
            voxel_map=self.local_map,
            initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma,
            kernel=sigma / 3,
        )

        self.adaptive_threshold.update_model_deviation(
            np.linalg.inv(initial_guess) @ new_pose)
        self.local_map.update(frame_downsample, new_pose)
        self.poses.append(new_pose)

    def voxelize(self, iframe):
        frame_downsample = voxel_down_sample(
            iframe, self.config.mapping.voxel_size * 0.5)
        source = voxel_down_sample(
            frame_downsample,
            self.config.mapping.voxel_size * 1.5)
        return source, frame_downsample

    def get_adaptive_threshold(self):
        return (
            self.config.adaptive_threshold.initial_threshold
            if not self.has_moved()
            else self.adaptive_threshold.get_threshold()
        )

    def get_prediction_model(self, frame_drop_ratio_curr):
        if len(self.poses) < 2:
            return np.eye(4)
        pose_diff_exp = np.linalg.inv(self.poses[-2]) @ self.poses[-1]

        curr_prev_drop_ratio = frame_drop_ratio_curr / self.frame_drop_ratio_prev
        # Skip matrix manipulation if frames' drop ratio is less than 5%
        if abs(curr_prev_drop_ratio - 1) < 0.05:
            return pose_diff_exp

        pose_diff_log = pu.log_pose(pose_diff_exp) * curr_prev_drop_ratio

        return pu.exp_pose6(pose_diff_log)

    def has_moved(self):
        if len(self.poses) < 1:
            return False

        def compute_motion(T1, T2):
            return np.linalg.norm(
                (np.linalg.inv(T1) @ T2)[:3, -1])
        motion = compute_motion(self.poses[0], self.poses[-1])
        return motion > 5 * self.config.adaptive_threshold.min_motion_th

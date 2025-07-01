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
"""


import numpy as np
import logging

from kiss_icp.config import KISSConfig  # type: ignore[import-untyped]
from kiss_icp.mapping import get_voxel_hash_map  # type: ignore[import-untyped]
from kiss_icp.registration import get_registration  # type: ignore[import-untyped]
from kiss_icp.threshold import get_threshold_estimator  # type: ignore[import-untyped]
from kiss_icp.voxelization import voxel_down_sample  # type: ignore[import-untyped]

import ouster.sdk.util.pose_util as pu
from ouster.sdk._bindings.mapping import _Preprocessor      # type: ignore[attr-defined]
from ouster.sdk._bindings.mapping import _Vector3dVector    # type: ignore[attr-defined]

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()


class Preprocessor:
    def __init__(self, max_range, min_range, deskew, max_num_threads):
        self._preprocessor = _Preprocessor(
            max_range, min_range, deskew, max_num_threads
        )

    def preprocess(self, frame: np.ndarray, timestamps: np.ndarray, relative_motion: np.ndarray):
        return np.asarray(
            self._preprocessor._preprocess(
                _Vector3dVector(frame),
                timestamps.ravel(),
                relative_motion,
            )
        )


class KissICP:

    def __init__(self, config: KISSConfig, initial_pose: np.ndarray = np.eye(4)):
        self.last_pose = initial_pose if initial_pose is not None else np.eye(4)
        self.last_delta_pose = np.eye(4)
        self.config = config
        self.adaptive_threshold = get_threshold_estimator(self.config)
        self.local_map = get_voxel_hash_map(self.config)
        self.registration = get_registration(self.config)
        self.preprocessor = Preprocessor(max_range=self.config.data.max_range,
                                         min_range=self.config.data.min_range,
                                         deskew=self.config.data.deskew,
                                         max_num_threads=0)

    def register_frame(self, frame, timestamps, delta_ratio=1):
        # Preprocess the input cloud
        frame = self.preprocessor.preprocess(frame, timestamps, self.last_delta_pose)

        # Voxelize
        source, frame_downsample = self.voxelize(frame)

        sigma = self.adaptive_threshold.get_threshold()

        if delta_ratio == 1:
            initial_guess = self.last_pose @ self.last_delta_pose
        else:
            delta_pose_log = pu.log_pose(self.last_delta_pose) * delta_ratio
            delta_pose_exp = pu.exp_pose6(delta_pose_log)
            initial_guess = self.last_pose @ delta_pose_exp

        # Run ICP
        new_pose = self.registration.align_points_to_map(
            points=source,
            voxel_map=self.local_map,
            initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma,
            kernel=sigma / 3,
        )

        model_deviation = np.linalg.inv(initial_guess) @ new_pose

        self.adaptive_threshold.update_model_deviation(model_deviation)

        if delta_ratio == 1:
            self.last_delta_pose = np.linalg.inv(self.last_pose) @ new_pose
        elif delta_ratio <= 0:
            raise ValueError("frame delta_ratio must be greater than zero.")
        else:
            delta_pose_exp = np.linalg.inv(self.last_pose) @ new_pose
            delta_pose_log = pu.log_pose(delta_pose_exp) / delta_ratio
            self.last_delta_pose = pu.exp_pose6(delta_pose_log)

        self.last_pose = new_pose
        self.local_map.update(frame_downsample, new_pose)

    def voxelize(self, iframe):
        frame_downsample = voxel_down_sample(
            iframe, self.config.mapping.voxel_size * 0.5)
        source = voxel_down_sample(
            frame_downsample,
            self.config.mapping.voxel_size * 1.5)
        return source, frame_downsample

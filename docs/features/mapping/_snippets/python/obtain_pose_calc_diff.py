from __future__ import annotations

from typing import Iterator, Tuple

import numpy as np

from ouster.sdk import open_source
from ouster.sdk.mapping import SlamConfig, SlamEngine


"""Return rotation and translation deltas between two poses."""
# [doc-stag-pose-diff]
def pose_differences(last_frame_pose: np.ndarray,
                     frame_pose: np.ndarray ) -> Tuple[np.ndarray, np.ndarray]:
    pose_diff = np.linalg.inv(last_frame_pose) @ frame_pose
    rotation_diff = pose_diff[:3, :3]
    translation_diff = pose_diff[:3, 3]
    return rotation_diff, translation_diff
# [doc-etag-pose-diff]

def iterate_pose_differences(
    data_source, slam: SlamEngine
) -> Iterator[Tuple[int, int, np.ndarray, np.ndarray, np.ndarray]]:
    """Yield pose metadata and deltas for each frame in ``data_source``."""

    last_frame_pose = np.eye(4)
    # [doc-stag-pose-diff-call]
    for idx, frame_set in enumerate(data_source):
        frames_w_poses = slam.update(frame_set)
        if not frames_w_poses:
            continue

        try:
            first_frame = frames_w_poses[0]
            if first_frame is None:
                continue
            col = first_frame.get_first_valid_column()
        except RuntimeError:
            continue

        frame_pose = first_frame.body_to_world[col]
        frame_ts = frame_set[0].timestamp[col]
        rotation_diff, translation_diff = pose_differences(last_frame_pose, frame_pose)
        last_frame_pose = frame_pose
        # [doc-etag-pose-diff-call]
        yield idx, frame_ts, frame_pose, rotation_diff, translation_diff


def build_default_slam(source_file_path: str) -> Tuple[SlamEngine, object]:
    """Construct a SLAM engine and its backing data source."""

    data_source = open_source(source_file_path, sensor_idx=-1)
    config = SlamConfig.create("lio")
    config.min_range = 1
    config.max_range = 50
    config.voxel_size = 0.5
    slam = SlamEngine.create(data_source.sensor_info, config)
    return slam, data_source


def run_pose_difference_report(source_file_path: str) -> None:
    """Emit pose drift information for the provided recording."""

    slam, data_source = build_default_slam(source_file_path)
    for idx, frame_ts, frame_pose, rotation_diff, translation_diff in iterate_pose_differences(
        data_source, slam
    ):
        print(f"idx = {idx} at timestamp {frame_ts} has the pose {frame_pose}")
        print(
            "idx = {idx} and Rotation Difference: {rotation_diff}, "
            "Translation Difference: {translation_diff}".format(
                idx=idx,
                rotation_diff=rotation_diff,
                translation_diff=translation_diff,
            )
        )


if __name__ == "__main__":
    run_pose_difference_report("/PATH_TO_THE_FILE")

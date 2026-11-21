# mypy: ignore-errors
import numpy as np

# [doc-stag-slam-imports]
from ouster.sdk import open_source
from ouster.sdk import mapping
from ouster.sdk.core import matrix_to_euler, LidarFrame
# [doc-etag-slam-imports]


def run_slam_once(source_file: str) -> None:
    """Open a source and run SLAM printing pose for each frame."""
    # collate=True, sensor_idx=-1 are defaults
    # [doc-stag-slam-open]
    slam_config = mapping.SlamConfig.create("lio")
    slam_config.min_range = 0.5
    slam_config.max_range = 100.0
    slam_config.deskew_method = "auto"
    # [doc-etag-slam-open]
    # [doc-stag-slam-engine]
    source = open_source(source_file)
    slam_engine = mapping.SlamEngine.create(
        source.sensor_info,
        slam_config)
    # [doc-etag-slam-engine]

    # [doc-stag-slam-loop]
    # [doc-stag-slam-loop-update]
    for frame_set in source:
        frame_set = slam_engine.update(frame_set)
        # [doc-etag-slam-loop-update]
        # [doc-stag-slam-loop-printpose]
        frame = frame_set[0]
        # Last valid column (scan end for lidar; column 0 for camera).
        col = frame.get_last_valid_column()
        frame_pose = frame.body_to_world[col]
        frame_ts = frame.timestamp[col]
        # [doc-etag-slam-loop-printpose]
        # Extract translation (top-right 3x1)
        t = frame_pose[:3, 3]
        # ZYX euler (roll, pitch, yaw) in degrees
        rot = frame_pose[:3, :3]
        angles = np.degrees(matrix_to_euler(rot))
        roll = angles[0]
        pitch = angles[1]
        yaw = angles[2]
        # frame_id is a LidarFrame attribute; a camera frame has no id to print.
        frame_id = frame.frame_id if isinstance(frame, LidarFrame) else -1
        print(f"idx = {frame_id}; frame_ts = {frame_ts};"
              f"; Translation: "
              f"{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f} "
              f" (Roll: {roll:.1f} "
              f", Pitch: {pitch:.1f} "
              f", Yaw: {yaw:.1f})")
    # [doc-etag-slam-loop]


__all__ = ["run_slam_once"]

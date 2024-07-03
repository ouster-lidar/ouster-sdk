from typing import Tuple

import numpy as np

import ouster.sdk.util.pose_util as pu
from ouster.sdk import client


def getNormalizedTimestamps(scan: client.LidarScan):
    ts = np.linspace(0, 1, scan.w, endpoint=True)
    # If there are jumps in the timestamps due to poor sensor time-syncing, fall back to defaults
    if np.any(np.diff(scan.timestamp[scan.status == 1]) <= 0):
        return ts
    else:
        start_col = client.first_valid_column(scan)
        stop_col = client.last_valid_column(scan)
        ts_actual = scan.timestamp
        start_ts = scan.timestamp[start_col]
        stop_ts = scan.timestamp[stop_col]
        # get 0 col to w col total ts duration
        ts_dur = (stop_ts - start_ts) / (stop_col - start_col) * (scan.w - 1)
        # calculate actual diff normalized ts using ts gap / total ts duration
        ts_actl_diff_norm = np.divide(np.subtract(ts_actual, start_ts), ts_dur)
        # update start_col to stop_col using ts_actl_diff_norm + ts[start_col]
        ts[start_col:stop_col] = ts_actl_diff_norm[start_col:stop_col] + ts[start_col]
        return ts


def getScanColPoseWithTs(start_pose_h, end_pose_h, scan: client.LidarScan) -> np.ndarray:
    ts = getNormalizedTimestamps(scan)
    diff_log = pu.log_pose(np.dot(end_pose_h, np.linalg.inv(start_pose_h)))
    # As end_ts_pose is the kiss-icp results of the scan. end_ts will be the
    # middle ts of the scan. Use end_ts as mid_scan_ts to avoid calculation
    per_col_diff_v = ts[:, np.newaxis] * diff_log[np.newaxis, :]

    per_col_diff_h = pu.exp_pose6(np.array(per_col_diff_v))
    end_pose_h = end_pose_h[np.newaxis, :, :]
    global_col_poses = np.einsum("ijk,ikl->ijl", per_col_diff_h, end_pose_h)

    return global_col_poses


def getScanColPose(start_ts_pose: Tuple[int, np.ndarray],
        end_ts_pose: Tuple[int, np.ndarray],
        scan: client.LidarScan) -> np.ndarray:
    """
    This function return the global_per_col_pose (4x4) of the input LidarScan by using
    the constant pose change rate assumption. This function currently ONLY for KISS-ICP
    Pose interpolaton.
    KISS-ICP registers point cloud based on the middle ts, so thepose timestamp is
    also based on middle ts of a scan.

    scan:              |    start_pose  |    end_pose   |
    ts (beginning):             |                |
                             start_ts         end_ts
    ts (middle):       |                    output_scan
                          funct return-> global_col_poses
    start_ts/end_ts are the middle vaild ts of the start_scan/end_scan.
    * end_pose MUST BE the KISS-ICP output result of the output_scan
    * for example: end_pose = kiss_icp.update(output_scan)
    """
    start_pose_h = start_ts_pose[1]
    end_pose_h = end_ts_pose[1]
    diff_log = pu.log_pose(np.dot(end_pose_h, np.linalg.inv(start_pose_h)))

    start_ts = start_ts_pose[0]
    end_ts = end_ts_pose[0]
    delta_ts = end_ts - start_ts

    if delta_ts < 0:
        raise RuntimeError(
            "end pose ts smaller then start pose ts.\nexit"
        )

    # As end_ts_pose is the kiss-icp results of the scan. end_ts will be the
    # middle ts of the scan. Use end_ts as mid_scan_ts to avoid calculation
    per_col_diff_v = [
        ((ts - end_ts) / delta_ts) * diff_log for ts in scan.timestamp
    ]

    per_col_diff_h = pu.exp_pose6(np.array(per_col_diff_v))

    start_pose_h = start_pose_h[np.newaxis, :, :]
    end_pose_h = end_pose_h[np.newaxis, :, :]
    global_col_poses = np.einsum("ijk,ikl->ijl", per_col_diff_h, end_pose_h)

    return global_col_poses

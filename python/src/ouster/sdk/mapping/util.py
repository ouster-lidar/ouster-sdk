import numpy as np
import ouster.sdk.util.pose_util as pu
from ouster.sdk import client
from typing import List, Optional


def getKissICPInputData(scans: List[Optional[client.LidarScan]], xyz_lut, ts_offsets):
    # processes a list of Lidar scans and prepares the input data required for the KISS ICP
    # return:
    # * total_pts (np.ndarray): The array has a shape of (N, 3), where N is the total
    # number of valid points across all scans. Same with total_ts, they discard the out of ranged (range 0) points
    # * total_ts (np.ndarray): The array has a shape of (N,) which corresponding to each point in the total_pts
    # * raw_ts (List[np.ndarray]): A list of NumPy arrays, which contrains the normalized timestamp
    # for each sensor. Raw_ts keep all normalized ts (keep the out of ranged points).
    # This values will be used in the writeScanColPose. Return to avoid duplicated calculation.
    assert scans

    # total_ts and total_pts discard the out of ranged (range 0) points
    total_ts = []
    total_pts = []
    # raw_ts contains all normalized ts. Keep the out of ranged points
    raw_ts = []

    # Determine overall start and stop timestamps
    total_start_ts = min(scan.timestamp[client.first_valid_column(scan)] + ts_off
                         for scan, ts_off in zip(scans, ts_offsets) if scan)
    total_stop_ts = max(scan.timestamp[client.last_valid_column(scan)] + ts_off
                        for scan, ts_off in zip(scans, ts_offsets) if scan)
    ts_dur = total_stop_ts - total_start_ts

    for idx, scan in enumerate(scans):
        if not scan:
            continue
        start_col = client.first_valid_column(scan)
        stop_col = client.last_valid_column(scan)

        ts = np.zeros(scan.w)
        ts_actual = scan.timestamp + ts_offsets[idx]

        if np.any(np.diff(ts_actual[scan.status == 1]) <= 0):
            # when scan has any out of order timestamp, use the linspaced ts
            ts[start_col:stop_col + 1] = np.linspace(0, 1, (stop_col - start_col + 1),
                                                     endpoint=True)
        else:
            ts_norm = (ts_actual - total_start_ts) / ts_dur
            ts[start_col:stop_col + 1] = ts_norm[start_col:stop_col + 1]

        # Filter out zero returns
        sel_flag = scan.field(client.ChanField.RANGE) != 0
        pts = xyz_lut[idx](scan.field(client.ChanField.RANGE))[sel_flag]
        total_pts.append(pts)

        raw_ts.append(np.array(ts))

        ts = np.tile(ts, (scan.h, 1))[sel_flag]
        total_ts.append(ts)

    # Concatenate all points and timestamps
    total_pts = np.concatenate(total_pts, axis=0) # type: ignore  # noqa
    total_ts = np.concatenate(total_ts, axis=0) # type: ignore  # noqa

    return total_pts, total_ts, raw_ts


def writeScanColPose(slam_prev_pose, slam_curr_pose, scans: List[Optional[client.LidarScan]],
                     normalized_ts_list, slam_frame_diff=1):
    if slam_frame_diff <= 0:
        raise ValueError("frame_diff must greater than zero.")

    if not scans:
        # Handle empty scans
        return

    if slam_prev_pose is None:
        # First scan pose in KISS in identity matrix. not enough poses to do
        # perturbation in every column. Scan col poses will be identity matrix
        # by default
        for scan in scans:
            scan.pose[:] = np.eye(4)  # type: ignore
        return

    diff_log = pu.log_pose(np.dot(slam_curr_pose,
                                  np.linalg.inv(slam_prev_pose))) / slam_frame_diff
    for idx, scan in enumerate(scans):
        if not scan:
            continue
        ts = normalized_ts_list[idx] - 0.5
        per_col_diff_v = ts[:, np.newaxis] * diff_log[np.newaxis, :]

        per_col_diff_h = pu.exp_pose6(np.array(per_col_diff_v))
        slam_curr_pose_reshaped = slam_curr_pose[np.newaxis, :, :]
        col_poses = np.einsum("ijk,ikl->ijl", per_col_diff_h, slam_curr_pose_reshaped)

        scan.pose[:] = col_poses

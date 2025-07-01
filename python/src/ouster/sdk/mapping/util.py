import numpy as np
import logging
import ouster.sdk.util.pose_util as pu
from ouster.sdk import core
from typing import List, Optional, Tuple

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()


def get_frame_ts_range(scans: List[Optional[core.LidarScan]]) -> List[Tuple[np.int64, np.int64]]:
    """
    Returns a list of (first_valid_column_timestamp, last_valid_column_timestamp) for each scan.
    If a scan is None, returns (-1, -1) for that position.
    """
    ranges: List[Tuple[np.int64, np.int64]] = []
    for scan in scans:
        if scan is None:
            ranges.append((np.int64(-1), np.int64(-1)))
        else:
            ranges.append((
                np.int64(scan.get_first_valid_column_timestamp()),
                np.int64(scan.get_last_valid_column_timestamp())
            ))
    return ranges


def get_frame_id_range(scans: List[Optional[core.LidarScan]]) -> List[Tuple[int, int]]:
    """
    Returns a list of (first_valid_column, last_valid_column) for each scan.
    If a scan is None, returns (-1, -1) for that position.
    """
    ranges: List[Tuple[int, int]] = []
    for scan in scans:
        if scan is None:
            ranges.append((-1, -1))
        else:
            ranges.append((
                int(scan.get_first_valid_column()),
                int(scan.get_last_valid_column())
            ))
    return ranges


def get_total_frame_pts_and_ts(scans: List[Optional[core.LidarScan]],
                               xyz_lut, ts_offsets: List[Optional[np.int64]]):
    # processes a list of Lidar scans and prepares the input data required for the KISS ICP
    # scans (List[Optional[core.LidarScan]]): a list of LidarScans with corrected timestamp
    # ts_offsets (List[Optional[np.int64]): If sensors are sync, the ts_offsets will be zero. If sensors not syncs,
    # the timestamp offsets for a list of LidarScans relative to the smallest packet timestamp.
    # Use it to align all scan timestamp within the same packet ts period for slam point cloud deskew
    # return:
    # * total_pts (np.ndarray): The array has a shape of (N, 3), where N is the total
    # number of valid points across all scans. Same with total_ts, they discard the out of ranged (range 0) points
    # * total_ts (np.ndarray): The array has a shape of (N,) which corresponding to each point in the total_pts
    # * raw_ts (List[np.ndarray]): A list of NumPy arrays, which contrains the normalized timestamp
    # for each sensor. Raw_ts keep all normalized ts (keep the out of ranged points).
    # This values will be used in the write_interpolated_poses. Return to avoid duplicated calculation.
    assert len(scans) == len(ts_offsets)

    frame_ts_ranges = get_frame_ts_range(scans)
    frame_id_ranges = get_frame_id_range(scans)

    # Align timestamps with their respective offsets
    adjusted_ts_ranges = [(start + ts_off, end + ts_off)
                          for (start, end), ts_off in zip(frame_ts_ranges, ts_offsets) if ts_off is not None]

    if not adjusted_ts_ranges:
        # This can happen if all scans were None or had no valid ts_offsets
        return np.array([]), np.array([]), []

    total_start_ts = min(start for start, end in adjusted_ts_ranges)
    total_stop_ts = max(end for start, end in adjusted_ts_ranges)
    ts_dur = total_stop_ts - total_start_ts

    # Handle case where duration is zero to avoid division by zero
    if ts_dur == 0:
        raise ValueError("Timestamp duration for the frame is zero. "
                         "All valid timestamps in the scan are identical, which prevents normalization.")

    total_pts = []
    total_ts = []
    normalized_ts = []

    # Create an iterator for valid scans to map them correctly
    valid_scans_info = iter(zip(scans, frame_id_ranges))
    for idx, scan in enumerate(scans):
        ts_off = ts_offsets[idx]
        if scan is None or ts_off is None:
            continue

        # Get the pre-calculated column ranges for the current valid scan
        current_scan, (start_col, stop_col) = next(valid_scans_info)

        ts = np.zeros(scan.w)
        ts_actual = scan.timestamp + np.int64(ts_off)

        ts_norm = (ts_actual - total_start_ts) / ts_dur
        ts[start_col:stop_col + 1] = ts_norm[start_col:stop_col + 1]

        normalized_ts.append(np.array(ts))

        # Filter out points with zero range
        sel_flag = scan.field(core.ChanField.RANGE) != 0
        pts = xyz_lut[idx](scan.field(core.ChanField.RANGE))[sel_flag]
        total_pts.append(pts)

        # Tile timestamps for each point and apply the same filter
        ts_tiled = np.tile(ts, (scan.h, 1))[sel_flag]
        total_ts.append(ts_tiled)

    # Concatenate lists into final numpy arrays
    total_pts_np = np.concatenate(total_pts, axis=0) if total_pts else np.empty((0, 3))
    total_ts_np = np.concatenate(total_ts, axis=0) if total_ts else np.empty((0,))

    return total_pts_np, total_ts_np, normalized_ts


def write_interpolated_poses(slam_prev_pose: np.ndarray, slam_curr_pose: np.ndarray,
                             scans: List[Optional[core.LidarScan]],
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

    # normalized_ts_list needs its own counter for non None scans
    ts_idx = 0
    diff_log = pu.log_pose(
        slam_curr_pose @ np.linalg.inv(slam_prev_pose)) / slam_frame_diff
    slam_curr_pose_reshaped = slam_curr_pose[np.newaxis, :, :]

    for scan in scans:
        if not scan:
            continue
        ts = normalized_ts_list[ts_idx] - 0.5
        ts_idx += 1
        per_col_diff_v = ts[:, np.newaxis] * diff_log[np.newaxis, :]
        per_col_diff_h = pu.exp_pose6(np.array(per_col_diff_v))
        scan.pose[:] = np.einsum("ijk,ikl->ijl", per_col_diff_h, slam_curr_pose_reshaped)


def determine_voxel_size(scans: List[Optional[core.LidarScan]],
                         start_pct: float = 0.92, end_pct: float = 0.96) -> Optional[float]:
    """Average highest 92% to 96% range readings and use this averaged range value
        to calculate the voxel map size """
    selected_ranges = np.array([])

    for idx, scan in enumerate(scans):
        if not scan:
            continue
        sel_flag = scan.field(core.ChanField.RANGE) != 0
        if not np.any(sel_flag):
            continue
        scan_range = scan.field(core.ChanField.RANGE)[sel_flag]
        start_value = np.percentile(scan_range, start_pct * 100)
        end_value = np.percentile(scan_range, end_pct * 100)

        selected_range = scan_range[(scan_range >= start_value) & (scan_range <= end_value)]
        selected_ranges = np.concatenate((selected_ranges, selected_range))

    if np.size(selected_ranges) == 0:
        return None
    # lidar range is in mm. change the unit to meter
    average = 0.001 * np.mean(selected_ranges)
    # use the lidar range readings and a number to land voxel size in a
    # proper range
    return average / 46.0


def _make_ortho(matrix: np.ndarray) -> np.ndarray:
    """Takes a 3x3 rotation matrix and returns an orthonormal matrix of it using SVD."""
    if matrix.shape != (3, 3):
        raise ValueError("Input matrix must be 3x3")
    U, S, Vt = np.linalg.svd(matrix)
    # Calculate the rotation matrix R = U V^T
    r = U @ Vt
    # Check the determinant. If it's -1, we have a reflection, not a pure rotation.
    # We need to flip the sign of the column of U corresponding to the smallest
    # singular value (which is the last one in the standard SVD ordering).
    # This effectively flips the sign of the determinant.
    if np.linalg.det(r) < 0.0:
        U_corrected = U.copy()
        U_corrected[:, -1] *= -1
        r = U_corrected @ Vt
    return r

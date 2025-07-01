from typing import List, Optional

import sys
import logging
import numpy as np
import ouster.sdk.core as core
import ouster.sdk.mapping.ouster_kiss_icp as ouster_kiss_icp
import ouster.sdk.mapping.util as util
from .slam_backend import SlamConfig, SlamBackend
from kiss_icp.config import KISSConfig  # type: ignore  # noqa

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()

try:
    from kiss_icp import __version__ as kiss_icp_version  # type: ignore
    MIN_KISS_ICP_VERSION = "1.2.0"
    if kiss_icp_version < MIN_KISS_ICP_VERSION:
        raise RuntimeError("kiss-icp version is too old. expected at least "
                           f"{MIN_KISS_ICP_VERSION}, got {kiss_icp_version}")
except Exception as e:
    logger.error(f"KISS-ICP {kiss_icp_version if 'kiss_icp_version' in globals() else 'unknown'} is incompatible with "
                 f"ouster-sdk\nError message: {e}")


class KissSlam(SlamBackend):
    """Wraps kiss-icp odometry to use with Ouster pipelines."""

    def __init__(
        self,
        infos: List[core.SensorInfo],
        config: SlamConfig
    ):
        self._xyz_lut = [core.XYZLut(info, use_extrinsics=True) for info in infos]
        self._config = KISSConfig(None)
        self._config.data.deskew = True
        self._config.data.max_range = config.max_range
        self._config.data.min_range = config.min_range
        self._initial_pose = config.initial_pose
        self._voxel_size = config.voxel_size
        # expected time difference between two frames in nanoseconds
        self._frame_durations = [1e9 / info.format.fps for info in infos]
        if isinstance(self._voxel_size, float) and self._voxel_size > 0:
            self._config.mapping.voxel_size = self._voxel_size
            logger.info(f"Kiss-ICP voxel size is {self._voxel_size:.4g} m")
            self._config_kiss_icp(self._config)

        # initialize internal state variables
        self._max_frame_id = max([core.PacketFormat(info).max_frame_id for info in infos])
        self._last_frame_id = [-1 for _ in infos]
        self._scans_ts_offsets: List[Optional[np.int64]] = [np.int64(0)] * len(infos)
        self._use_packet_offset = False
        self._last_frame_ts_range = [(np.int64(-1), np.int64(-1))] * len(infos)
        self._frame_valid_id_range = [(-1, -1)] * len(infos)

    def _config_kiss_icp(self, config):
        self._last_slam_pose = self._initial_pose \
            if self._initial_pose is not None else np.eye(4)
        self._last_slam_pose[:3, :3] = util._make_ortho(self._last_slam_pose[:3, :3])
        self.ouster_kiss_icp = ouster_kiss_icp.KissICP(config, initial_pose=self._last_slam_pose)

    def _correct_scan_ts(self,
                     scan: Optional[core.LidarScan],
                     sensor_id: int) -> Optional[core.LidarScan]:
        """
        Corrects the timestamps of the input Lidar scan based on the previous frame's
        timestamp range and the sensor's frame duration.
        """
        if scan is None:
            logger.error(f"Scan for sensor {sensor_id} is None. Cannot correct timestamps.")
            return None
        # make sure denominator is none zero
        if scan.w == 0:
            raise ValueError("Scan width is 0. Input source file in invalid.")
            return scan

        # determine start timestamp for this scan
        last_start_ts, _ = self._last_frame_ts_range[sensor_id]
        scan_start_id, scan_end_id = self._frame_valid_id_range[sensor_id]

        if last_start_ts == -1:
            # no previous start then use the existing timestamp
            new_start_ts = scan.timestamp[scan_start_id]
        else:
            # normal case: continue from last start + frame duration
            new_start_ts = last_start_ts + self._frame_durations[sensor_id]

        num_valid = scan_end_id - scan_start_id + 1
        col_ts_diff = self._frame_durations[sensor_id] / scan.w
        scan.timestamp[scan_start_id:scan_end_id + 1] = new_start_ts + np.arange(num_valid) * col_ts_diff

        return scan

    def update(self, scans: List[Optional[core.LidarScan]]) -> List[Optional[core.LidarScan]]:
        """
        Update the pose information of each lidar scan based on SLAM pose estimation.

        Workflow:
        • Initialize frame_valid_id_range if not already set.
        • Compute overall frame timestamp range.
        • Check inter-sensor synchronization:
          - If sensors are out of sync, enable packet-offset mode.
        • Monotonicity check (ignoring zero timestamps):
          - For any scan whose timestamps go backwards, correct its timestamps
            and enable packet-offset mode.
        • Packet-offset handling:
          - If offset mode is active, compute fallback timestamp offsets.
          - Abort early if offset computation fails.
        • Dynamic voxel size:
          - If a voxel_size was provided then apply it.
        • Frame-ID handling:
          - Compute per-sensor frame-ID diffs (with overflow protection).
          - Invalidate any out-of-order scans by zeroing their ranges.
          - Track the smallest non-zero diff for SLAM integration.
        • Point-cloud aggregation:
          - Merge valid points and their (possibly offset) timestamps.
          - Abort early if no points remain.
        • SLAM registration:
          - Register the aggregated frame with KISS-ICP.
        • Pose interpolation:
          - Interpolate between the last and new SLAM poses.
          - Write per-column poses back into each scan.
        • Update internal state:
          - Store the new last SLAM pose, last frame-ID array, and timestamp range.

        Returns:
            The same list of scans with the updated per-column poses.
        """
        # only initialize frame_valid_id_range once
        if all(r == (-1, -1) for r in self._frame_valid_id_range):
            self._frame_valid_id_range = [
                (core.first_valid_column(scan), core.last_valid_column(scan))
                if scan is not None else (-1, -1)
                for scan in scans
            ]
        orig_ts_cache: List[Optional[np.ndarray]] = [None] * len(scans)
        frame_ts_range = util.get_frame_ts_range(scans)

        # Sensor sync check. fall back to packet timestamp offsets if sensors are not synchronized.
        is_synced = self._check_sensors_synchronization(frame_ts_range)
        if not is_synced and not self._use_packet_offset:
            logger.warning("Sensors appear unsynchronized. Using estimated clock offsets, "
                           "results may be affected")
            self._use_packet_offset = True

        # Monotonicity check correct any scans timestamp go backwards. If so, correct its ts
        monotonic = self._check_monotonic_increase_ts(scans)
        for sensor_id, sensor_status in enumerate(monotonic):
            scan = scans[sensor_id]
            if scan is not None and not sensor_status:
                # Save original timestamps for non-monotonic sensors to allow later restoration.
                if orig_ts_cache[sensor_id] is None:
                    orig_ts_cache[sensor_id] = scan.timestamp.copy()

                scans[sensor_id] = self._correct_scan_ts(scans[sensor_id], sensor_id)
                if not self._use_packet_offset:
                    logger.warning("Lidarscan timestamp are not monotically increasing. "
                                   "Using estimated clock offsets, results may be affected.")
                    self._use_packet_offset = True

        # If in packet-offset mode, compute offsets
        if self._use_packet_offset:
            self._scans_ts_offsets = self._calculate_fallback_ts_offset(scans)
            if not self._scans_ts_offsets:
                logger.error("Failed to calculate fallback timestamp offsets. Skipping update.")
                return scans

        if callable(self._voxel_size):
            voxel_size = self._voxel_size(scans)
            if not voxel_size:
                return scans
            self._config.mapping.voxel_size = self._voxel_size = voxel_size
            self._config_kiss_icp(self._config)

        # if frames drop, use the slam_update_diff ratio for a more accurate initial pose guess
        slam_update_diff = sys.maxsize
        for idx, scan in enumerate(scans):
            if not scan:
                continue
            curr_frame_id = scan.frame_id
            if self._last_frame_id[idx] == -1:
                frame_id_diff = 1
            else:
                frame_id_diff = curr_frame_id - self._last_frame_id[idx]

            # magic number -65530 is for the 16-bit int overflow case and is considered as for
            # potential missing scans.
            OVERFLOW_PROTECTION_THRESHOLD = -65500
            if self._last_frame_id[idx] != -1 and curr_frame_id <= self._last_frame_id[idx] \
                    and frame_id_diff > OVERFLOW_PROTECTION_THRESHOLD:
                logger.warning("Out-of-order scan detected; invalidating scan")
                scan.field(core.ChanField.RANGE)[:] = 0

            # Prevent integer overflow
            frame_id_diff = frame_id_diff % self._max_frame_id
            slam_update_diff = min(frame_id_diff, slam_update_diff)

            self._last_frame_id[idx] = curr_frame_id

        total_pts, total_ts, norm_ts = util.get_total_frame_pts_and_ts(scans, self._xyz_lut,
                                                                       self._scans_ts_offsets)

        if len(total_pts) == 0:     # no valid points
            logger.warning("No valid points found in the scans. Skipping this update.")
            return scans

        try:
            self.ouster_kiss_icp.register_frame(total_pts, total_ts,
                                                slam_update_diff)
        except Exception as e:
            logger.error(f"Failed to register frame with KISS-ICP: {e}")
            raise

        # Use the SLAM poses to update the column poses of all scans.
        util.write_interpolated_poses(self._last_slam_pose, self.ouster_kiss_icp.last_pose,
                                      scans, norm_ts, slam_update_diff)

        self._last_slam_pose = self.ouster_kiss_icp.last_pose
        self._last_frame_ts_range = frame_ts_range

        # restore only those timestamps we cached
        for sensor_id, orig_ts in enumerate(orig_ts_cache):
            scan = scans[sensor_id]
            if orig_ts is not None and scan is not None:
                scan.timestamp[:] = orig_ts

        # return the updated column pose scans
        return scans

    def _check_sensors_synchronization(self, frame_ts_range) -> bool:
        """
        Determines whether a set of lidar scans are pre-synchronized based on their
        start timestamps.

        This function compares the earliest and latest start timestamps from a range
        of scans and checks if their difference is within the smallest frame duration
        available. If the time difference is less than or equal to the minimum frame
        gap, the scans are considered pre-synchronized.
        """
        min_start_ts = min(frame_ts_range, key=lambda x: x[0])[0]
        max_start_ts = max(frame_ts_range, key=lambda x: x[0])[0]

        min_frame_gap = min(self._frame_durations)

        # Check if all timestamps are within the smallest frame duration
        return max_start_ts - min_start_ts <= min_frame_gap

    def _check_monotonic_increase_ts(self, scans: List[Optional[core.LidarScan]]) -> List[bool]:
        """Check if the timestamps in each lidar scan are strictly increasing
        (ignoring any zeros) and its first non-zero timestamp is greater
        than the previous frame’s end timestamp.
        Returns a list of booleans, true if the scan is monotonically increasing
        """
        results: List[bool] = []

        for idx, scan in enumerate(scans):
            # missing scan, append True as we wont correct it
            if scan is None:
                results.append(True)
                continue

            # grab only the valid columns
            first_id, last_id = self._frame_valid_id_range[idx]
            window = scan.timestamp[first_id: last_id + 1]

            # drop zeros
            filtered_ts = [t for t in window if t != 0]

            # 0 or 1 non-zero timestamp, append true
            if len(filtered_ts) <= 1:
                results.append(True)
                continue

            # first‐nonzero vs previous‐frame end
            first_ts = filtered_ts[0]
            last_frame_end = self._last_frame_ts_range[idx][1]

            # only check inter‐frame ordering if we actually have a previous end
            if last_frame_end >= 0 and first_ts <= last_frame_end:
                results.append(False)
                continue
            # check if timestamps are strictly increasing
            strictly_inc = True
            prev_ts = first_ts

            for ts in filtered_ts[1:]:
                if ts <= prev_ts:
                    strictly_inc = False
                    break
                prev_ts = ts

            results.append(strictly_inc)

        return results

    def _calculate_fallback_ts_offset(self, scans: List[Optional[core.LidarScan]]) -> List[Optional[np.int64]]:
        """Calculate fallback timestamp offsets for a list of LidarScan instances.

        This function computes, for each scan:
          • the packet timestamp offset relative to the earliest packet timestamp across all non‐None scans,
          • then subtracts the scan’s first valid column timestamp,
          • or returns None if the scan itself is None.
        """
        pkt_ts_vals = [
            np.int64(scan.get_first_valid_packet_timestamp())
            for scan in scans
            if scan is not None
        ]
        if not pkt_ts_vals:
            # no valid scans at all return early with a list of None and
            return [None] * len(scans)

        min_pkt_ts = min(pkt_ts_vals)

        # build per‐scan offsets, or None for missing scans
        offsets: List[Optional[np.int64]] = []
        for scan in scans:
            if scan is None:
                offsets.append(None)
            else:
                pkt_ts = np.int64(scan.get_first_valid_packet_timestamp())
                rel_pkt_ts = pkt_ts - min_pkt_ts
                frame_start_ts = np.int64(scan.get_first_valid_column_timestamp())
                offsets.append(rel_pkt_ts - frame_start_ts)

        return offsets

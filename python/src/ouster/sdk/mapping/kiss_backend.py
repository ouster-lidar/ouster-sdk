import math
import logging
import numpy as np
from typing import List, Tuple  # noqa: F401

import ouster.sdk.mapping.ouster_kiss_icp as ouster_kiss_icp
import ouster.sdk.mapping.util as util
import ouster.sdk.client as client

from .slam_backend import SLAMBackend

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()


class KissBackend(SLAMBackend):
    """Wraps kiss-icp odometry to use with Ouster pipelines."""

    def __init__(
        self,
        info: client.SensorInfo,
        use_extrinsics: bool = True,
        max_range: float = 150.0,
        min_range: float = 1.0,
        voxel_size: float = -1.0,
        live_stream: bool = False
    ):
        try:
            from kiss_icp.kiss_icp import KissICP  # type: ignore  # noqa
        except ImportError:
            logger.error("kiss-icp, a package required for slam, is unsupported on "
                         "your platform. ")
            raise
        import kiss_icp.config  # type: ignore  # noqa
        super().__init__(info, use_extrinsics)
        # to store the middle valid timestamp of a scan and middle col pose
        self.ts_pose = list(tuple())  # type: List[Tuple[int, np.ndarray]]
        self.last_id = -1
        self.config = kiss_icp.config.KISSConfig(None)
        self.config.data.deskew = True
        self.config.data.max_range = max_range
        self.config.data.min_range = min_range
        self.voxel_size = voxel_size
        self.live_stream = live_stream
        self.scan_pixels = info.format.pixels_per_column * info.format.columns_per_frame
        if voxel_size and voxel_size > 0:
            self.config.mapping.voxel_size = voxel_size
            logger.info(f"Kiss-ICP voxel map size is {voxel_size:.4g} m")
            self._config_kiss_icp(self.config)

    def _config_kiss_icp(self, config):
        self.kiss_icp = ouster_kiss_icp.KissICP(config)

    """Update the pose (per_column_global_pose) variable in scan and return"""

    def update(self, scan: client.LidarScan) -> client.LidarScan:
        if not self.voxel_size or self.voxel_size <= 0:
            self.voxel_size = self.get_voxel_size(scan)
            self.config.mapping.voxel_size = self.voxel_size
            self._config_kiss_icp(self.config)
            logger.info(f"Auto voxel size calculated based on the first scan "
                        f"which is {self.voxel_size:.4g} m.")

        # Create normalized timetamps with shape (h, w) based on actual
        # per-column timestamps
        ts_norm = np.tile(util.getNormalizedTimestamps(scan), (self.h, 1))

        # accumulate scan timestamps in parallel list to poses
        scan_start_ts = client.first_valid_column_ts(scan)
        scan_end_ts = client.core.last_valid_column_ts(scan)
        scan_mid_ts = int((scan_start_ts + scan_end_ts) / 2)

        # filtering our zero returns makes it substantially faster for kiss-icp
        sel_flag = scan.field(client.ChanField.RANGE) != 0
        xyz = self.xyz_lut(scan.field(client.ChanField.RANGE))[sel_flag]

        try:
            self.kiss_icp.register_frame(xyz, ts_norm[sel_flag], scan_start_ts)
        except Exception as e:
            from kiss_icp import __version__ as kiss_icp_version

            logger.error(f"KISS-ICP {kiss_icp_version} is incompatible with "
                         f"ouster-mapping\n"
                         f"Error message: {e}")
            raise

        if self.last_id != -1 and scan_mid_ts <= self.ts_pose[-1][0]:
            logger.warning("Set the out of order scan invalid")
            scan.field(client.ChanField.RANGE)[:] = 0
            return scan

        self.ts_pose.append((scan_mid_ts, self.kiss_icp.poses[-1]))

        if len(self.ts_pose) >= 2:
            col_global_poses = util.getScanColPoseWithTs(
                self.ts_pose[-2][1], self.ts_pose[-1][1], scan)
            scan.pose[:] = col_global_poses
        elif len(self.ts_pose) < 2:
            # First scan pose in KISS in identity matrix. not enough poses to do
            # perturbation in every column. Juse identity matrix for col poses
            scan.pose[:] = np.tile(np.identity(4), (scan.w, 1, 1))

        self.last_id = scan.frame_id

        return scan

    # average highest 92% to 96% range readings and use this averaged range value
    # to calculate the voxel map size
    def get_voxel_size(self, scan: client.LidarScan, start_pct: float = 0.92,
                       end_pct: float = 0.96) -> float:
        sel_flag = scan.field(client.ChanField.RANGE) != 0
        scan_range = scan.field(client.ChanField.RANGE)[sel_flag]
        start_value = np.percentile(scan_range, start_pct * 100)
        end_value = np.percentile(scan_range, end_pct * 100)
        selected_ranges = scan_range[(scan_range >= start_value) & (scan_range <= end_value)]

        # lidar range is in mm. change the unit to meter
        average = np.mean(selected_ranges) / 1000
        # use the lidar range readings and a number to land voxel size in a
        # proper range
        voxel_size = average / 46.0

        # the above calculation is based on 1024 * 128. Smaller pixels per scan has
        # a smaller voxel size
        pixels_ratio = math.sqrt(math.sqrt(self.scan_pixels / (1024 * 128)))
        voxel_size *= pixels_ratio

        # For live sensor, we use a larger voxel size for better results
        live_voxel_ratio = 2.2
        if self.live_stream:
            voxel_size *= live_voxel_ratio
            logger.warning("Auto voxel sizer uses a larger size voxel for the live "
                           "sensor real-time processing.")

        return voxel_size

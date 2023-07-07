# type: ignore


from typing import List, Tuple  # noqa: F401
import numpy as np
from .slam_backend import SLAMBackend
import ouster.sdkx.mapping.util as util
import ouster.client as client


class KissBackend(SLAMBackend):
    """Wraps kiss-icp odometry to use with Ouster pipelines."""

    def __init__(
        self,
        info: client.SensorInfo,
        use_extrinsics: bool = True,
        max_range: float = 150.0,
        min_range: float = 1.0,
    ):
        try:
            from kiss_icp.kiss_icp import KissICP
        except ImportError:
            print("kiss_icp is not installed. Please run pip install kiss-icp")
        import kiss_icp.config
        super().__init__(info, use_extrinsics)
        config = kiss_icp.config.KISSConfig(config_file=None)
        config.data.deskew = True
        config.data.max_range = max_range
        config.data.min_range = min_range
        config.mapping.voxel_size = config.data.max_range / 100.0
        self.kiss_icp = KissICP(config)
        # to store the middle valid timestamp of a scan and middle col pose
        self.ts_pose = list(tuple())  # type: List[Tuple[int, np.ndarray]]
        self.timestamps = np.tile(
            np.linspace(0, 1.0, self.w, endpoint=False), (self.h, 1)
        )

    """Update the pose (per_column_global_pose) variable in scan and return"""
    def update(self, scan: client.LidarScan) -> client.LidarScan:

        # filtering our zero returns makes it substantially faster for kiss-icp
        sel_flag = scan.field(client.ChanField.RANGE) != 0
        xyz = self.xyz_lut(scan.field(client.ChanField.RANGE))[sel_flag]
        self.kiss_icp.register_frame(xyz, self.timestamps[sel_flag])

        # accumulate scan timestamps in parallel list to poses
        scan_start_ts = scan.timestamp[client.core.first_valid_column(scan)]
        scan_end_ts = scan.timestamp[client.core.last_valid_column(scan)]
        scan_mid_ts = (scan_start_ts + scan_end_ts) / 2
        self.ts_pose.append((scan_mid_ts, self.kiss_icp.poses[-1]))

        if len(self.ts_pose) >= 2:
            col_global_poses = util.getScanColPose(
                self.ts_pose[-2], self.ts_pose[-1], scan
            )
            scan.pose[:] = col_global_poses
        elif len(self.ts_pose) < 2:
            # First scan pose in KISS in identity matrix. not enough poses to do
            # perturbation in every column. Juse identity matrix for col poses

            repeat_num = scan.w
            identity_matrix = np.identity(4)
            scan.pose[:] = np.array(np.tile(identity_matrix, (repeat_num, 1, 1)).tolist())

        return scan

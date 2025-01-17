import time
import logging
from typing import List, Optional
from ouster.sdk import client
from kiss_icp.kiss_icp import KissICP       # type: ignore
import kiss_icp.config                      # type: ignore
import ouster.sdk.mapping.util as util
import numpy as np


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()


class KissLocalization:
    def __init__(self, filename, xyz_lut, max_range, min_range, voxel_size):
        self.config = kiss_icp.config.KISSConfig(None)
        self.config.data.deskew = True
        self.config.data.max_range = max_range
        self.config.data.min_range = min_range
        self.config.mapping.voxel_size = voxel_size
        self.xyz_lut = xyz_lut
        self.last_slam_pose = None

        self.odometry = KissICP(config=self.config)
        self.load_global_map(filename)

    def load_global_map(self, filename):
        import point_cloud_utils as pcu     # type: ignore
        start_time = time.time()
        points = pcu.load_mesh_v(filename)
        end_time = time.time()
        self.odometry.local_map.add_points(points)
        print(f"Took {(end_time - start_time):.4f} seconds to load the map "
              f"{filename} which has {len(points)} points")

    def track(self, scans: List[Optional[client.LidarScan]]) -> List[Optional[client.LidarScan]]:
        pts, ts, ts_raw = util.getKissICPInputData(scans, self.xyz_lut, [0])
        sigma = self.odometry.adaptive_threshold.get_threshold()
        initial_guess = self.odometry.last_pose @ self.odometry.last_delta
        frame = self.odometry.compensator.deskew_scan(pts, ts, self.odometry.last_delta)
        frame = self.odometry.preprocess(frame)
        source, _ = self.odometry.voxelize(frame)
        new_pose = self.odometry.registration.align_points_to_map(
            source,
            self.odometry.local_map,
            initial_guess,
            3.0 * sigma,
            sigma / 3.0)
        model_deviation = np.linalg.inv(initial_guess) @ new_pose
        self.odometry.adaptive_threshold.update_model_deviation(model_deviation)
        self.odometry.last_delta = np.linalg.inv(self.odometry.last_pose) @ new_pose
        self.odometry.last_pose = new_pose
        util.writeScanColPose(self.last_slam_pose,
                self.odometry.last_pose, scans, ts_raw, 1)
        self.last_slam_pose = self.odometry.last_pose
        return scans

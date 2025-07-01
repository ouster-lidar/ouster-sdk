from typing import Union, List, Optional
import time
import logging
import numpy as np
from ouster.sdk.core import SensorInfo, LidarScan, XYZLut, read_pointcloud
from kiss_icp.kiss_icp import KissICP       # type: ignore
import kiss_icp.config                      # type: ignore
import ouster.sdk.mapping.util as util
from .localization_backend import LocalizationConfig, LocalizationBackend


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()


class KissLocalization(LocalizationBackend):

    @classmethod
    def params(cls) -> LocalizationConfig:
        return LocalizationConfig()

    def __init__(self, infos: List[SensorInfo], config: LocalizationConfig, map: Union[str, np.ndarray]):
        self._common_init(infos, config, map if isinstance(map, np.ndarray) else self._load_map(map))

    def _common_init(self, infos: List[SensorInfo], config: LocalizationConfig, map_points: np.ndarray):
        self._xyz_lut = [XYZLut(infos[0], use_extrinsics=True)]
        self._map_points = map_points
        self._config = kiss_icp.config.KISSConfig(None)
        self._config.data.deskew = True
        self._config.data.max_range = config.max_range
        self._config.data.min_range = config.min_range
        self._initial_pose = config.initial_pose
        self._voxel_size = config.voxel_size
        if isinstance(self._voxel_size, float) and self._voxel_size > 0:
            self._config.mapping.voxel_size = self._voxel_size
            self._init_odometry()

    def _init_odometry(self):
        logger.info(f"Using voxel size {self._voxel_size:.4g} m")
        self._odometry = KissICP(config=self._config)
        self._odometry.local_map.add_points(self._map_points)
        self._odometry.last_pose = self._initial_pose \
            if self._initial_pose is not None else np.eye(4)
        self._odometry.last_pose[:3, :3] = util._make_ortho(self._odometry.last_pose[:3, :3])
        self._last_slam_pose = self._odometry.last_pose

    def _load_map(self, map_file: str) -> np.ndarray:
        start_time = time.time()
        points = read_pointcloud(map_file)
        end_time = time.time()
        logger.info(f"Took {(end_time - start_time):.4f} seconds to load the map "
              f"{map_file} which has {len(points)} points")
        return points

    def update(self, scans: List[Optional[LidarScan]]) -> List[Optional[LidarScan]]:

        if callable(self._voxel_size):
            voxel_size = self._voxel_size(scans)
            if not voxel_size:
                return scans
            self._config.mapping.voxel_size = self._voxel_size = voxel_size
            self._init_odometry()

        pts, ts, norm_ts = util.get_total_frame_pts_and_ts(scans, self._xyz_lut, [np.int64(0)])
        sigma = self._odometry.adaptive_threshold.get_threshold()
        initial_guess = self._odometry.last_pose @ self._odometry.last_delta
        frame = self._odometry.preprocessor.preprocess(
            pts, ts, self._odometry.last_delta)
        source, _ = self._odometry.voxelize(frame)
        new_pose = self._odometry.registration.align_points_to_map(
            source,
            self._odometry.local_map,
            initial_guess,
            3.0 * sigma,
            sigma / 3.0)
        model_deviation = np.linalg.inv(initial_guess) @ new_pose
        self._odometry.adaptive_threshold.update_model_deviation(model_deviation)
        self._odometry.last_delta = np.linalg.inv(self._odometry.last_pose) @ new_pose
        self._odometry.last_pose = new_pose
        util.write_interpolated_poses(self._last_slam_pose, self._odometry.last_pose,
                                      scans, norm_ts, 1)
        self._last_slam_pose = self._odometry.last_pose
        return scans

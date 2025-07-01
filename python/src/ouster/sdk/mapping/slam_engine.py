from typing import List, Optional
from ouster.sdk.core import SensorInfo, LidarScan
from ouster.sdk.mapping.slam_backend import SlamConfig
from ouster.sdk.mapping.kiss_slam import KissSlam


class SlamEngine():
    """base class of general slam solutions for SDK cli usage"""

    def __init__(self, infos: List[SensorInfo], config: SlamConfig):
        """SlamEngine constructor"""
        if config.backend not in ["kiss"]:
            raise ValueError(f"Unsupported backend {config.backend}")
        self._backend = KissSlam(infos, config)

    def update(self, scans: List[Optional[LidarScan]]) -> List[Optional[LidarScan]]:
        """
        Update the pose (per_column_global_pose) variable in scan and return
        """
        return self._backend.update(scans)

    @property
    def last_pose(self):
        """Get the current pose"""
        return self._backend._last_slam_pose

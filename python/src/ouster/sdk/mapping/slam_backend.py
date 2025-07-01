from typing import List, Optional
from abc import ABC, abstractmethod
from ouster.sdk.core import SensorInfo, LidarScan
import numpy as np


class SlamConfig:
    min_range: float = 0.0
    max_range: float = 150.0
    voxel_size: float = 1.0
    initial_pose: Optional[np.ndarray] = None
    backend: str = "kiss"


class SlamBackend(ABC):
    """base class of general slam solutions for SDK cli usage"""

    @abstractmethod
    def __init__(self, infos: List[SensorInfo], config: SlamConfig):
        """SlamBackend constructor"""
        pass

    @abstractmethod
    def update(self, scans: List[Optional[LidarScan]]) -> List[Optional[LidarScan]]:
        """Update the pose (per_column_global_pose) variable in scan and return
        """
        pass

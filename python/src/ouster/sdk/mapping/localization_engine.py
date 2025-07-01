from typing import List, Optional, Union
import numpy as np
from ouster.sdk.core import SensorInfo, LidarScan
from ouster.sdk.mapping.localization_backend import LocalizationConfig
from ouster.sdk.mapping.kiss_localization import KissLocalization


class LocalizationEngine():
    """base class of general slam solutions for SDK cli usage"""

    def __init__(self, infos: List[SensorInfo], config: LocalizationConfig,
                 map: Union[str, np.ndarray]):
        """SlamEngine constructor"""
        if config.backend not in ["kiss"]:
            raise ValueError(f"Unsupported backend {config.backend}")
        self._backend = KissLocalization(infos, config, map)

    def update(self, scans: List[Optional[LidarScan]]) -> List[Optional[LidarScan]]:
        """
        Update the pose (per_column_global_pose) variable in scan and return
        """
        return self._backend.update(scans)

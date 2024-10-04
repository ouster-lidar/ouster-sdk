from abc import ABC, abstractmethod

import ouster.sdk.client as client

from typing import List, Optional


class SLAMBackend(ABC):
    """base class of general slam solutions for SDK cli usage"""

    def __init__(self, infos: List[client.SensorInfo], use_extrinsics: bool = True):
        self.xyz_lut = []
        for info in infos:
            self.xyz_lut.append(client.XYZLut(info, use_extrinsics=use_extrinsics))

    """
        Update the pose (per_column_global_pose) variable in scan and return
    """

    @abstractmethod
    def update(self, scans: List[Optional[client.LidarScan]]) -> List[Optional[client.LidarScan]]:
        pass

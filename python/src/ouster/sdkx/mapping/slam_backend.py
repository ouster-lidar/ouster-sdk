import ouster.client as client
from abc import ABC, abstractmethod


class SLAMBackend(ABC):
    """base class of general slam solutions for SDK cli usage"""

    def __init__(self, info: client.SensorInfo, use_extrinsics: bool = True):
        self.info = info
        self.xyz_lut = client.XYZLut(info, use_extrinsics=use_extrinsics)
        self.w = info.format.columns_per_frame
        self.h = info.format.pixels_per_column

    """
        Update the pose (per_column_global_pose) variable in scan and return
    """

    @abstractmethod
    def update(self, scan: client.LidarScan) -> client.LidarScan:
        pass

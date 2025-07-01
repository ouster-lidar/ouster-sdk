# pyi definitions for the exposed classes and methods

from typing import List, Tuple
from numpy import ndarray
import numpy as np
from enum import Enum

class SolverConfig:
    key_frame_distance: float
    traj_rotation_weight: float
    traj_translation_weight: float
    max_num_iterations: int
    function_tolerance: float
    gradient_tolerance: float
    parameter_tolerance: float
    process_printout: bool

    def __init__(self) -> None: ...
    

class SamplingMode(Enum):
    KEY_FRAMES = ...
    COLUMNS    = ...


class LossFunction(Enum):
    HuberLoss     = ...
    CauchyLoss    = ...
    SoftLOneLoss  = ...
    ArctanLoss    = ...
    TrivialLoss   = ...

    @staticmethod
    def from_string(name: str) -> "LossFunction":
        """
        Convert a string (e.g. "HuberLoss") to the corresponding LossFunction enum.
        Raises:
            ValueError: if the string does not match any member.
        """
        ...

class PoseOptimizer:
    def __init__(self, osf_filename: str, key_frame_distance: float, fix_first_node: bool = False) -> None: ...
    
    def add_pose_to_pose_constraint(self, ts1: np.uint64, ts2: np.uint64, diff: np.ndarray,
                                    rotation_weight: float = 1.0, translation_weight: float = 1.0) -> bool: ...
    def add_absolute_pose_constraint(self, ts1: np.uint64, ts2: np.uint64,
                                     rotation_weight: float = 1.0, translation_weight: float = 1.0) -> bool: ...
    def add_point_to_point_constraint(self,
                                      ts1: np.uint64, row1: np.uint32, col1: np.uint32, return_idx1: np.uint32,
                                      ts2: np.uint64, row2: np.uint32, col2: np.uint32, return_idx2: np.uint32,
                                      translation_weight: float = 1.0) -> bool: ...
    def solve(self, n: int = 0) -> None: ...
    def get_timestamps(self, mode: SamplingMode) -> List[np.uint64]: ...
    def get_poses(self, mode: SamplingMode) -> ndarray: ...
    def save(self, osf_name: str) -> bool: ...
    

def save_trajectory(
    filename: str,
    timestamps: List[np.uint64],
    poses: List[ndarray],
    file_type: str = "csv"
) -> bool: ...


class _Preprocessor:
    def __init__(self, max_range: float, min_range: float, deskew: bool, max_num_threads: int) -> None: ...

    def _preprocess(self,
                    points: List[np.ndarray],
                    timestamps: List[float],
                    relative_motion: np.ndarray) -> List[np.ndarray]: ...


def _Vector3dVector(vec: np.ndarray) -> List[np.ndarray]: ...

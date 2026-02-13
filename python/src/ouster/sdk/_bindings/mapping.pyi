from typing import Optional, Union, List, ClassVar, overload, cast
from enum import Enum
import numpy as np
from ouster.sdk.core import SensorInfo, LidarScan, LidarScanSet

class SamplingMode(Enum):
    KEY_FRAMES = cast(SamplingMode, ...)
    COLUMNS = cast(SamplingMode, ...)

class LossFunction(Enum):
    HUBER_LOSS = cast(LossFunction, ...)
    CAUCHY_LOSS = cast(LossFunction, ...)
    SOFT_L_ONE_LOSS = cast(LossFunction, ...)
    ARCTAN_LOSS = cast(LossFunction, ...)
    TRIVIAL_LOSS = cast(LossFunction, ...)

    @staticmethod
    def from_string(name: str) -> 'LossFunction':
        """from_string(name: str) -> ouster.sdk._bindings.mapping.LossFunction


                Convert a string (e.g. "HUBER_LOSS") to the corresponding LossFunction enum.

                Args:
       name: one of "HUBER_LOSS", "CAUCHY_LOSS", "SOFT_L_ONE_LOSS", "ARCTAN_LOSS", "TRIVIAL_LOSS"
            
"""
        ...

class SolverConfig:
    key_frame_distance: float
    traj_rotation_weight: float
    traj_translation_weight: float
    max_num_iterations: int
    function_tolerance: float
    gradient_tolerance: float
    parameter_tolerance: float
    process_printout: bool
    loss_function: LossFunction
    loss_scale: float

    def __init__(self) -> None:
        """__init__(self: ouster.sdk._bindings.mapping.SolverConfig) -> None

Initialize SolverConfig with default values.
"""
        ...

class Constraint:
    translation_weights: np.ndarray

    def get_constraint_id(self) -> int:
        """get_constraint_id(self: ouster.sdk._bindings.mapping.Constraint) -> int


            Get the unique constraint ID. Returns 0 for non-user constraints.
            IDs are assigned when constraint objects are constructed.

            Returns:
                int: The constraint ID, or 0 if not a user-added constraint.
        
"""
        ...

class AbsolutePoseConstraint(Constraint):
    timestamp: np.uint64
    pose: np.ndarray
    rotation_weight: float
    translation_weights: np.ndarray

    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.AbsolutePoseConstraint) -> None

Default constructor

2. __init__(self: ouster.sdk._bindings.mapping.AbsolutePoseConstraint, timestamp: int, pose: numpy.ndarray[numpy.float64[4, 4]], rotation_weight: float = 1.0, translation_weight: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.])) -> None


                              Constructor for AbsolutePoseConstraint.

               Args:
                   timestamp (int): Timestamp of the pose to constrain (nanoseconds)
                   pose: The 4x4 transformation matrix (SE3) to constrain to
                   rotation_weight: Scalar weight applied to the quaternion axis-alignment residual
                   translation_weight: Weight for translation constraints (x, y, z)
               
"""
        ...

    @overload
    def __init__(self, timestamp: np.uint64, pose: np.ndarray, rotation_weight: float=1.0, translation_weight: np.ndarray=np.array([1.0, 1.0, 1.0])) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.AbsolutePoseConstraint) -> None

Default constructor

2. __init__(self: ouster.sdk._bindings.mapping.AbsolutePoseConstraint, timestamp: int, pose: numpy.ndarray[numpy.float64[4, 4]], rotation_weight: float = 1.0, translation_weight: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.])) -> None


                              Constructor for AbsolutePoseConstraint.

               Args:
                   timestamp (int): Timestamp of the pose to constrain (nanoseconds)
                   pose: The 4x4 transformation matrix (SE3) to constrain to
                   rotation_weight: Scalar weight applied to the quaternion axis-alignment residual
                   translation_weight: Weight for translation constraints (x, y, z)
               
"""
        ...

class PoseToPoseConstraint(Constraint):
    timestamp1: np.uint64
    timestamp2: np.uint64
    relative_pose: np.ndarray
    rotation_weight: float
    translation_weights: np.ndarray

    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.PoseToPoseConstraint) -> None

Default constructor

2. __init__(self: ouster.sdk._bindings.mapping.PoseToPoseConstraint, timestamp1: int, timestamp2: int, relative_pose: numpy.ndarray[numpy.float64[4, 4]] = array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]), rotation_weight: float = 1.0, translation_weight: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.])) -> None


               Constructor for PoseToPoseConstraint.

               Args:
                   timestamp1 (int): Timestamp of the first pose (nanoseconds)
                   timestamp2 (int): Timestamp of the second pose (nanoseconds)
                   relative_pose: Expected relative transformation from pose1 to pose2.
                                 Use the identity matrix to let PoseOptimizer auto-estimate it via ICP.
                   rotation_weight: Scalar weight applied to the quaternion axis-alignment residual
                   translation_weight: Weight for translation constraints (x, y, z)
               
"""
        ...

    @overload
    def __init__(self, timestamp1: np.uint64, timestamp2: np.uint64, relative_pose: np.ndarray, rotation_weight: float=1.0, translation_weight: np.ndarray=np.array([1.0, 1.0, 1.0])) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.PoseToPoseConstraint) -> None

Default constructor

2. __init__(self: ouster.sdk._bindings.mapping.PoseToPoseConstraint, timestamp1: int, timestamp2: int, relative_pose: numpy.ndarray[numpy.float64[4, 4]] = array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]), rotation_weight: float = 1.0, translation_weight: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.])) -> None


               Constructor for PoseToPoseConstraint.

               Args:
                   timestamp1 (int): Timestamp of the first pose (nanoseconds)
                   timestamp2 (int): Timestamp of the second pose (nanoseconds)
                   relative_pose: Expected relative transformation from pose1 to pose2.
                                 Use the identity matrix to let PoseOptimizer auto-estimate it via ICP.
                   rotation_weight: Scalar weight applied to the quaternion axis-alignment residual
                   translation_weight: Weight for translation constraints (x, y, z)
               
"""
        ...

class PointToPointConstraint(Constraint):
    timestamp1: np.uint64
    timestamp2: np.uint64
    row1: np.uint32
    col1: np.uint32
    return_idx1: np.uint32
    row2: np.uint32
    col2: np.uint32
    return_idx2: np.uint32
    translation_weights: np.ndarray

    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.PointToPointConstraint) -> None

Default constructor

2. __init__(self: ouster.sdk._bindings.mapping.PointToPointConstraint, timestamp1: int, row1: int, col1: int, return_idx1: int, timestamp2: int, row2: int, col2: int, return_idx2: int, translation_weight: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.])) -> None


               Constructor for PointToPointConstraint.

               Args:
                   timestamp1 (int): Timestamp of the first point's pose (nanoseconds)
                   row1 (int): Row index of the first point
                   col1 (int): Column index of the first point
                   return_idx1 (int): Return index of the first point (1 or 2)
                   timestamp2 (int): Timestamp of the second point's pose (nanoseconds)
                   row2 (int): Row index of the second point
                   col2 (int): Column index of the second point
                   return_idx2 (int): Return index of the second point (1 or 2)
                   translation_weight: Weight for translation constraints (x, y, z)
               
"""
        ...

    @overload
    def __init__(self, timestamp1: np.uint64, row1: np.uint32, col1: np.uint32, return_idx1: np.uint32, timestamp2: np.uint64, row2: np.uint32, col2: np.uint32, return_idx2: np.uint32, translation_weight: np.ndarray=np.array([1.0, 1.0, 1.0])) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.PointToPointConstraint) -> None

Default constructor

2. __init__(self: ouster.sdk._bindings.mapping.PointToPointConstraint, timestamp1: int, row1: int, col1: int, return_idx1: int, timestamp2: int, row2: int, col2: int, return_idx2: int, translation_weight: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.])) -> None


               Constructor for PointToPointConstraint.

               Args:
                   timestamp1 (int): Timestamp of the first point's pose (nanoseconds)
                   row1 (int): Row index of the first point
                   col1 (int): Column index of the first point
                   return_idx1 (int): Return index of the first point (1 or 2)
                   timestamp2 (int): Timestamp of the second point's pose (nanoseconds)
                   row2 (int): Row index of the second point
                   col2 (int): Column index of the second point
                   return_idx2 (int): Return index of the second point (1 or 2)
                   translation_weight: Weight for translation constraints (x, y, z)
               
"""
        ...

class AbsolutePointConstraint(Constraint):
    timestamp: np.uint64
    row: np.uint32
    col: np.uint32
    return_idx: np.uint32
    absolute_position: np.ndarray
    translation_weights: np.ndarray

    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.AbsolutePointConstraint) -> None

Default constructor

2. __init__(self: ouster.sdk._bindings.mapping.AbsolutePointConstraint, timestamp: int, row: int, col: int, return_idx: int, absolute_position: numpy.ndarray[numpy.float64[3, 1]], translation_weight: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.])) -> None


               Constructor for AbsolutePointConstraint.

               Args:
                   timestamp (int): Timestamp of the point's pose (nanoseconds)
                   row (int): Row index of the point
                   col (int): Column index of the point
                   return_idx (int): Return index of the point (1 or 2)
                   absolute_position: Target world position (x, y, z)
                   translation_weight: Weight for translation constraints (x, y, z)
               
"""
        ...

    @overload
    def __init__(self, timestamp: np.uint64, row: np.uint32, col: np.uint32, return_idx: np.uint32, absolute_position: np.ndarray, translation_weight: np.ndarray=np.array([1.0, 1.0, 1.0])) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.AbsolutePointConstraint) -> None

Default constructor

2. __init__(self: ouster.sdk._bindings.mapping.AbsolutePointConstraint, timestamp: int, row: int, col: int, return_idx: int, absolute_position: numpy.ndarray[numpy.float64[3, 1]], translation_weight: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.])) -> None


               Constructor for AbsolutePointConstraint.

               Args:
                   timestamp (int): Timestamp of the point's pose (nanoseconds)
                   row (int): Row index of the point
                   col (int): Column index of the point
                   return_idx (int): Return index of the point (1 or 2)
                   absolute_position: Target world position (x, y, z)
                   translation_weight: Weight for translation constraints (x, y, z)
               
"""
        ...

class PoseOptimizerNode:
    ts: np.uint64
    downsampled_pts: np.ndarray

    def get_pose(self) -> np.ndarray:
        """get_pose(self: ouster.sdk._bindings.mapping.PoseOptimizerNode) -> numpy.ndarray[numpy.float64[4, 4]]
"""
        ...

class PoseOptimizer:

    @overload
    def __init__(self, osf_filename: str, config: SolverConfig, fix_first_node: bool=False) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, options: ouster.sdk._bindings.mapping.SolverConfig) -> None


                 Initialize PoseOptimizer with an OSF file and solver options.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     options (SolverConfig): Solver configuration options. Set options.fix_first_node to True to fix the first node.
                 

2. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, config_filename: str) -> None


                 Initialize PoseOptimizer with an OSF file and solver options loaded from a config file.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     config_filename: Path to the configuration file (JSON) containing solver options. Set fix_first_node in the config file to True to fix the first node.
                 

3. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, key_frame_distance: float) -> None


                 Initialize PoseOptimizer with an OSF file and a node gap.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     key_frame_distance (float): The gap distance between nodes in the trajectory.
                     To fix the first node, set fix_first_node in the SolverConfig after construction.
                 
"""
        ...

    @overload
    def __init__(self, osf_filename: str, key_frame_distance: float, fix_first_node: bool=False) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, options: ouster.sdk._bindings.mapping.SolverConfig) -> None


                 Initialize PoseOptimizer with an OSF file and solver options.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     options (SolverConfig): Solver configuration options. Set options.fix_first_node to True to fix the first node.
                 

2. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, config_filename: str) -> None


                 Initialize PoseOptimizer with an OSF file and solver options loaded from a config file.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     config_filename: Path to the configuration file (JSON) containing solver options. Set fix_first_node in the config file to True to fix the first node.
                 

3. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, key_frame_distance: float) -> None


                 Initialize PoseOptimizer with an OSF file and a node gap.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     key_frame_distance (float): The gap distance between nodes in the trajectory.
                     To fix the first node, set fix_first_node in the SolverConfig after construction.
                 
"""
        ...

    @overload
    def __init__(self, osf_filename: str, constraints_json_file: str, fix_first_node: bool=False) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, options: ouster.sdk._bindings.mapping.SolverConfig) -> None


                 Initialize PoseOptimizer with an OSF file and solver options.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     options (SolverConfig): Solver configuration options. Set options.fix_first_node to True to fix the first node.
                 

2. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, config_filename: str) -> None


                 Initialize PoseOptimizer with an OSF file and solver options loaded from a config file.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     config_filename: Path to the configuration file (JSON) containing solver options. Set fix_first_node in the config file to True to fix the first node.
                 

3. __init__(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str, key_frame_distance: float) -> None


                 Initialize PoseOptimizer with an OSF file and a node gap.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     key_frame_distance (float): The gap distance between nodes in the trajectory.
                     To fix the first node, set fix_first_node in the SolverConfig after construction.
                 
"""
        ...

    def add_constraint(self, constraint: Union[AbsolutePoseConstraint, PoseToPoseConstraint, PointToPointConstraint, AbsolutePointConstraint]) -> int:
        """add_constraint(self: ouster.sdk._bindings.mapping.PoseOptimizer, constraint: ouster.sdk._bindings.mapping.Constraint) -> int


               Add a constraint to the pose optimization problem.

               This is the new unified API for adding constraints. Use the constraint class
               constructors to create constraints, then pass them to this method.
               The constraint must already have a unique ID for later removal.
               Adding a constraint with a duplicate ID will fail.

               Args:
                   constraint: A constraint object created by one of the constraint constructors.

               Returns:
                   int: The unique constraint ID of the added constraint.

               Raises:
                   RuntimeError: If the constraint cannot be added.
               
"""
        ...

    def set_solver_step_callback(self, callback) -> None:
        """set_solver_step_callback(self: ouster.sdk._bindings.mapping.PoseOptimizer, callback: Callable) -> None


                Register a Python callable to be invoked at each solver iteration.

                The callable runs on the same thread as ``solve()``, once per ceres
                iteration. Use this to hook visualization or logging.

                Args:
                    callback (Callable[[], None]): Function to call each iteration.
            
"""
        ...

    def solve(self, steps: int=0) -> float:
        """solve(self: ouster.sdk._bindings.mapping.PoseOptimizer, steps: int = 0) -> float


             Incrementally optimize the trajectory.

             This method performs a fixed number of iterations of the optimization algorithm,
             continuing from the current state. It can be called repeatedly to gradually refine
             the trajectory. The number of iterations to execute is specified by 'steps'.

             Args:
                 steps (int, optional): The number of iterations to run for this incremental
                     optimization. Defaults to 0 (uses whatever max_num_iterations was already set).
	     Returns:
                 float: The cost value from the last solve call.
         
"""
        ...

    def get_cost_value(self) -> float:
        """get_cost_value(self: ouster.sdk._bindings.mapping.PoseOptimizer) -> float


                Get the last solver cost value (final cost from the last solve()).

                Returns:
                    float: The last recorded solver cost value.
            
"""
        ...

    def initialize_trajectory_alignment(self) -> bool:
        """initialize_trajectory_alignment(self: ouster.sdk._bindings.mapping.PoseOptimizer) -> bool


                Initialize trajectory alignment using average absolute constraints.

                Computes a weighted average SE(3) transform from the currently
                loaded absolute pose and absolute point constraints (using their
                weights in Lie algebra space) and left-multiplies the entire
                trajectory by that transform as an initial alignment step before
                optimization.

                Returns:
                    bool: True if an alignment transform was applied, False if
                        skipped (e.g. no absolute constraints or negligible delta).
            
"""
        ...

    def save_config(self, config_filename: str) -> None:
        """save_config(self: ouster.sdk._bindings.mapping.PoseOptimizer, config_filename: str) -> None


        Save the current SolverConfig (including constraints) to a JSON file.

        This method serializes the current solver configuration and all constraints
        to a JSON file. The resulting file can be used later with Pose Optimizer
        construction to restore the exact same optimization setup.

        Args:
            config_filename (str): Path where the JSON file should be saved.

        Raises:
            RuntimeError: If the file cannot be saved.
        
"""
        ...

    def get_timestamps(self, mode: SamplingMode) -> np.ndarray:
        """get_timestamps(self: ouster.sdk._bindings.mapping.PoseOptimizer, type: ouster.sdk._bindings.mapping.SamplingMode) -> numpy.ndarray[numpy.uint64]


                Retrieve timestamps corresponding to the selected sampling mode.

                Args:
                    type (SamplingMode): Sampling strategy to use.
                        - SamplingMode.KEY_FRAMES: Returns timestamps at key-frame poses.
                        - SamplingMode.COLUMNS: Returns timestamps of every lidarscan's columns.

                Returns:
                    numpy.ndarray[np.uint64]: A 1D array of timestamps (nanoseconds).
            
"""
        ...

    def get_poses(self, mode: SamplingMode) -> np.ndarray:
        """get_poses(self: ouster.sdk._bindings.mapping.PoseOptimizer, type: ouster.sdk._bindings.mapping.SamplingMode) -> numpy.ndarray[numpy.float64]


                Retrieve poses as a NumPy array of 4×4 transformation matrices.

                Args:
                    type (SamplingMode): Sampling strategy to use.
                        - SamplingMode.KEY_FRAMES: Returns poses at key-frame timestamps.
                        - SamplingMode.COLUMNS: Returns poses of every lidarscan's columns.

                Returns:
                    numpy.ndarray[np.float64]: An (n, 4, 4) array of 4×4 poses.
            
"""
        ...

    def save(self, osf_filename: str) -> None:
        """save(self: ouster.sdk._bindings.mapping.PoseOptimizer, osf_filename: str) -> None


                 Save the optimized trajectory to an OSF file.

                 This method writes the current state of the optimized trajectory to a new OSF file,
                 preserving all other data from the original file.

                 Args:
                     osf_filename (str): The name of the output OSF file.

                 Returns:
                     bool: True if the file was successfully saved, False otherwise.
             
"""
        ...

    def get_constraints(self) -> List[Union[AbsolutePoseConstraint, PoseToPoseConstraint, PointToPointConstraint, AbsolutePointConstraint]]:
        """get_constraints(*args, **kwargs)
Overloaded function.

1. get_constraints(self: ouster.sdk._bindings.mapping.PoseOptimizer) -> list


               Get all constraints currently added to the pose optimizer.

               Returns:
                   list: A list of constraint objects (copies) currently configured in the optimizer.
               

2. get_constraints(self: ouster.sdk._bindings.mapping.PoseOptimizer) -> list


                 Get all constraints currently configured in the pose optimizer.

                 This method returns a copy of all constraints that are currently
                 configured in the pose optimizer, including both constraints loaded
                 from JSON files during construction and constraints added later via
                 add_constraint().

                 Returns:
                     List[Constraint]: A list of Constraint objects representing all currently
                     configured constraints.
             
"""
        ...

    def set_constraints(self, constraints: List[Union[AbsolutePoseConstraint, PoseToPoseConstraint, PointToPointConstraint, AbsolutePointConstraint]]) -> None:
        """set_constraints(self: ouster.sdk._bindings.mapping.PoseOptimizer, constraints: list) -> None


                 Set all constraints for the pose optimizer.

                 This method replaces all existing constraints with the provided set
                 of constraints. Any constraints previously loaded from JSON files or
                 added via add_constraint() will be removed and replaced with the new
                 constraint set.

                 Args:
                     constraints (List[Constraint]): A list of Constraint objects to set as the
                     complete constraint set.

                 Raises:
                     RuntimeError: If the constraints cannot be set.
             
"""
        ...

    def remove_constraint(self, constraint_id: int) -> None:
        """remove_constraint(self: ouster.sdk._bindings.mapping.PoseOptimizer, constraint_id: int) -> None


               Remove a constraint from the pose optimization problem.

               Args:
                   constraint_id (int): The unique ID of the constraint to remove.

               Raises:
                   RuntimeError: If the constraint ID is not found.
               
"""
        ...

    def get_total_iterations(self) -> int:
        """get_total_iterations(self: ouster.sdk._bindings.mapping.PoseOptimizer) -> int


                Get the cumulative number of solver iterations executed so far.

                Returns:
                    int: Total iterations across all calls to solve().
            
"""
        ...

    def get_sampled_nodes(self, count: int=100) -> List[PoseOptimizerNode]:
        """get_sampled_nodes(self: ouster.sdk._bindings.mapping.PoseOptimizer, count: int = 100) -> list


               Retrieve up to `count` scan nodes evenly sampled across the OSF.

               Each node is guaranteed to have a downsampled point cloud; nodes
               are created on-demand if necessary.
            
"""
        ...

    def get_node(self, timestamp: np.uint64) -> Optional[PoseOptimizerNode]:
        """get_node(self: ouster.sdk._bindings.mapping.PoseOptimizer, timestamp: int) -> ouster.sdk._bindings.mapping.PoseOptimizerNode


               Get the node associated with a given timestamp (first-valid-column ts).
               The node pose is updated before returning. Returns None if not found.
            
"""
        ...

    def get_key_frame_distance(self) -> float:
        """get_key_frame_distance(self: ouster.sdk._bindings.mapping.PoseOptimizer) -> float


                Return the configured key-frame distance (meters) used when constructing the trajectory.
             
"""
        ...

def save_trajectory(filename: str, timestamps: List[np.uint64], poses: List[np.ndarray], file_type: str='csv') -> None:
    """save_trajectory(filename: str, timestamps: numpy.ndarray[numpy.uint64], poses: numpy.ndarray[numpy.float64], file_type: str = 'csv') -> None
"""
    ...

class SlamConfig:
    min_range: float
    max_range: float
    voxel_size: float
    initial_pose: np.ndarray
    backend: str
    deskew_method: str

class SlamEngine:

    def __init__(self, infos: List[SensorInfo], config: SlamConfig) -> None:
        """__init__(self: ouster.sdk._bindings.mapping.SlamEngine, infos: list[ouster.sdk._bindings.client.SensorInfo], config: ouster.sdk._bindings.mapping.SlamConfig) -> None


                SlamEngine constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
                    config (SlamConfig): Configuration options for the SLAM engine.
             
"""
        ...

    def update(self, scans: LidarScanSet) -> LidarScanSet:
        """update(self: ouster.sdk._bindings.mapping.SlamEngine, scans: ouster.sdk._bindings.client.LidarScanSet) -> ouster.sdk._bindings.client.LidarScanSet


                Update the pose (per_column_global_pose) variable in scan and return

                Args:
                    scans (List[LidarScan]): List of scans to update with the latest pose.

                Returns:
                    List[LidarScan]: The updated scans with per_column_global_pose set.
            
"""
        ...

    def get_point_cloud(self) -> np.ndarray:
        """get_point_cloud(self: ouster.sdk._bindings.mapping.SlamEngine) -> numpy.ndarray[numpy.float32[m, 3]]


                Get the current point cloud from the SLAM engine.

                Returns:
                    Nx3: The point cloud generated by the SLAM engine.
            
"""
        ...

class LocalizationConfig:
    min_range: float
    max_range: float
    voxel_size: float
    initial_pose: np.ndarray
    backend: str
    deskew_method: str

class LocalizationEngine:

    def __init__(self, infos: List[SensorInfo], config: LocalizationConfig, map: Union[str, np.ndarray]):
        """__init__(self: ouster.sdk._bindings.mapping.LocalizationEngine, infos: list[ouster.sdk._bindings.client.SensorInfo], config: ouster.sdk._bindings.mapping.LocalizationConfig, map: str) -> None


                LocalizationEngine constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
                    config (LocalizationConfig): Configuration options for the Localization engine.
                    map_path (string): Path to the point cloud map.
             
"""
        ...

    def update(self, scans: LidarScanSet) -> LidarScanSet:
        """update(self: ouster.sdk._bindings.mapping.LocalizationEngine, scans: ouster.sdk._bindings.client.LidarScanSet) -> ouster.sdk._bindings.client.LidarScanSet


                Update the pose (per_column_global_pose) variable in scan and return

                Args:
                    scans (List[LidarScan]): List of scans to update with the latest pose.

                Returns:
                    List[LidarScan]: The updated scans with per_column_global_pose set
            
"""
        ...

class ActiveTimeCorrection:

    def __init__(self, infos: List[SensorInfo]) -> None:
        """__init__(self: ouster.sdk._bindings.mapping.ActiveTimeCorrection, infos: list[ouster.sdk._bindings.client.SensorInfo]) -> None


                ActiveTimeCorrection constructor.
                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             
"""
        ...

    def update(self, scans: LidarScanSet) -> LidarScanSet:
        """update(self: ouster.sdk._bindings.mapping.ActiveTimeCorrection, scans: ouster.sdk._bindings.client.LidarScanSet) -> ouster.sdk._bindings.client.LidarScanSet


                Update the timestamps in the provided LidarScanSet using active time correction.
            
"""
        ...

    def reset(self, scans: LidarScanSet) -> LidarScanSet:
        """reset(self: ouster.sdk._bindings.mapping.ActiveTimeCorrection, scans: ouster.sdk._bindings.client.LidarScanSet) -> ouster.sdk._bindings.client.LidarScanSet


                Restore the timestamps in the provided LidarScanSet to their original values before active time correction was applied.
            
"""
        ...

class DeskewMethod:

    def __init__(self, infos: List[SensorInfo]) -> None:
        """__init__(self: ouster.sdk._bindings.mapping.DeskewMethod, infos: list[ouster.sdk._bindings.client.SensorInfo]) -> None


                ConstantVelocityDeskewMethod constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             
"""
        ...

    def update(self, scans: LidarScanSet) -> LidarScanSet:
        """update(self: ouster.sdk._bindings.mapping.DeskewMethod, scans: ouster.sdk._bindings.client.LidarScanSet) -> ouster.sdk._bindings.client.LidarScanSet


                Update the pose (per_column_global_pose) variable in scan.

                Args:
                    scans (LidarScanSet): a LidarScanSet.

                Returns:
                    LidarScanSet: The updated lidar scans with per_column_global_pose set
            
"""
        ...

    def set_last_pose(self, ts: np.int64, pose: np.ndarray) -> None:
        """Unbound function"""
        ...

class ConstantVelocityDeskewMethod(DeskewMethod):

    def __init__(self, infos: List[SensorInfo]) -> None:
        """__init__(self: ouster.sdk._bindings.mapping.ConstantVelocityDeskewMethod, infos: list[ouster.sdk._bindings.client.SensorInfo]) -> None


                ConstantVelocityDeskewMethod constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             
"""
        ...

    def update(self, scans: LidarScanSet) -> LidarScanSet:
        """update(self: ouster.sdk._bindings.mapping.DeskewMethod, scans: ouster.sdk._bindings.client.LidarScanSet) -> ouster.sdk._bindings.client.LidarScanSet


                Update the pose (per_column_global_pose) variable in scan.

                Args:
                    scans (LidarScanSet): a LidarScanSet.

                Returns:
                    LidarScanSet: The updated lidar scans with per_column_global_pose set
            
"""
        ...

    def set_last_pose(self, ts: np.int64, pose: np.ndarray) -> None:
        """set_last_pose(self: ouster.sdk._bindings.mapping.ConstantVelocityDeskewMethod, ts: int, pose: numpy.ndarray[numpy.float64]) -> None


                Set the current pose to use for deskewing.

                This method allows setting the current pose that will be used
                as a reference for deskewing incoming scans. The current pose
                should represent the sensor's pose at the time of the most recent
                scan.

                Args:
                    ts (int): The timestamp (nanoseconds) associated with the pose.
                    pose (numpy.ndarray): A 4x4 transformation matrix representing
                        the current pose.
            
"""
        ...

class DeskewMethodFactory:

    @staticmethod
    def create(method: str, infos: List[SensorInfo]) -> DeskewMethod:
        """create(method_name: str, infos: list[ouster.sdk._bindings.client.SensorInfo]) -> ouster.sdk._bindings.mapping.DeskewMethod


                    Create a DeskewMethod instance based on the specified method name.

                    Args:
                        method_name (str): The name of the deskew method to create.
                        infos (List[SensorInfo]): List of sensor info objects for each
                            sensor in the system.

                    Returns:
                        DeskewMethod: A new instance of the specified deskew method.
                 
"""
        ...

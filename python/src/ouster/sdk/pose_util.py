from typing_extensions import Protocol
from typing import (Union, Tuple, List, Optional, Iterable, Sequence)

import numpy as np

import bisect

from ouster import client

import logging

try:
    from scipy.spatial.transform import Rotation as R
    _no_scipy = False
except ImportError:
    _no_scipy = True

# show "No scipy ..."" message only once on the first encounter
_no_scipy_warned = False


def no_scipy() -> bool:
    """Checks the scipy availability with a warning message."""
    global _no_scipy_warned
    if _no_scipy:
        if _no_scipy_warned:
            logger = logging.getLogger("ouster-sdk-pose-util")
            logger.warning("No scipy module is found. Please install scipy for "
                           "faster poses calculations: pip3 install scipy.")
            _no_scipy_warned = True
        return True
    else:
        return False


Numeric = Union[int, float, np.number]

# 3D pose (rotation + translation) represented by 6 elements
# of se3 Lie algebra (i.e. exponential coordinates):
#   [wx, wy, wz, x, y, z]
Pose6 = np.ndarray

# 3D pose (rotation + translation) represented by 4x4 matrix
# of SE3 manifold (i.e. homogeneous matrix):
#   4x4 matrix: [R | T]
#               [0 | 1]
#   where: R is 3x3 rotation matrix SO3
#          T is 3x1 translation vector
PoseH = np.ndarray

Pose = Union[Pose6, PoseH]

TrajPoses = Sequence[Tuple[Numeric, Pose]]

# List of pairs (ts, Pose6), where 6 element pose in exponential coordinates.
#   pose : [wx, wy, wz, x, y, z]
PosesList = List[Tuple[int, Pose6]]


# ==========================================================================
# ======== Temp stub functions for no scipy environments ===================
def _no_scipy_exp_rot_vec(vec: np.ndarray) -> np.ndarray:
    """Converts so3 vector(s) to a rotation matrix(s).

    (no scipy version, not optimised)

    Dimensions: [3] -> [3, 3] or [N, 3] -> [N, 3, 3]
    """
    ret_first = False
    if vec.ndim == 1:
        vec = vec[np.newaxis, ...]
        ret_first = True

    res = np.zeros((vec.shape[0], 3, 3))

    for i in range(vec.shape[0]):
        theta = np.linalg.norm(vec[i])
        if abs(theta) < 1e-9:
            res[i] = np.eye(3)
            continue
        rot_axis = vec[i] / theta
        w = np.array([[0, -rot_axis[2], rot_axis[1]],
                    [rot_axis[2], 0, -rot_axis[0]],
                    [-rot_axis[1], rot_axis[0], 0]])
        res[i] = np.eye(3) + np.sin(theta) * w \
            + (1 - np.cos(theta)) * np.dot(w, w)

    return res[0] if ret_first else res


# Lynch and Park: 3.2.3.3, Page 87, Algorithm
def _no_scipy_log_rot_mat(rm) -> np.ndarray:
    """Convert rotation matrix(s) to so3 vector coordinates (log() operator)

    (no scipy version, not optimised)

    Dimensions: [3, 3] -> [3] or [N, 3, 3] -> [N, 3]
    """
    ret_first = False
    if rm.ndim == 2:
        rm = rm[np.newaxis, ...]
        ret_first = True

    res = np.zeros((rm.shape[0], 3))

    for i in range(rm.shape[0]):
        acos = 0.5 * (np.trace(rm[i]) - 1)
        if acos >= 1:
            res[i] = np.zeros(3)
            continue
        if acos <= -1:
            if abs(1 + rm[i, 2, 2]) > 1e-9:
                res[i] = np.pi * (
                    1.0 / np.sqrt(2 + 2 * rm[i, 2, 2])) * np.array(
                        [rm[i, 0, 2], rm[i, 1, 2], 1 + rm[i, 2, 2]])
                continue
            if abs(1 + rm[i, 1, 1]) > 1e-9:
                res[i] = np.pi * (1.0 /
                                  np.sqrt(2 + 2 * rm[i, 1, 1])) * np.array(
                                      [rm[i, 0, 1], 1 + rm[i, 1, 1], rm[i, 2, 1]])
                continue
            res[i] = np.pi * (1.0 / np.sqrt(2 + 2 * rm[i, 0, 0])) * np.array(
                [1 + rm[i, 0, 0], rm[i, 1, 0], rm[i, 2, 0]])
            continue
        theta = np.arccos(acos)
        w = (0.5 / np.sin(theta)) * (rm[i] - rm[i].transpose())
        res[i] = theta * np.array([w[2, 1], w[0, 2], w[1, 0]])

    return res[0] if ret_first else res


def _no_scipy_exp_pose6(vec: np.ndarray) -> np.ndarray:
    """Convert exponential pose6 vector(s) to homogeneous matrix(s).

    (no scipy version, not optimised)

    Dimensions: [6] -> [4, 4] or [N, 6] -> [N, 4, 4]
    """
    ret_first = False
    if vec.ndim == 1:
        vec = vec[np.newaxis, ...]
        ret_first = True

    res = np.zeros((vec.shape[0], 4, 4))

    for i in range(vec.shape[0]):
        # standard implementation
        theta = np.linalg.norm(vec[i, :3])
        if abs(theta) < 1e-9:
            res[i] = np.r_[np.c_[np.eye(3), vec[i, 3:]], [[0, 0, 0, 1]]]
            continue
        rot_axis = vec[i, :3] / theta
        w = np.array([[0, -rot_axis[2], rot_axis[1]],
                    [rot_axis[2], 0, -rot_axis[0]],
                    [-rot_axis[1], rot_axis[0], 0]])
        hom = np.eye(4)

        hom[:3, :3] = _no_scipy_exp_rot_vec(vec[i, :3])

        hom[:3, 3] = np.dot(
            np.eye(3) * theta + (1 - np.cos(theta)) * w
            + (theta - np.sin(theta)) * np.dot(w, w),
            vec[i, 3:]) / theta

        res[i] = hom

    return res[0] if ret_first else res


def _no_scipy_log_pose(hmat: np.ndarray) -> np.ndarray:
    """Convert homogeneous matrix(s) to exp pose6 vector(s) coordinate.

    (no scipy version, not optimised)

    Dimensions: [4, 4] -> [6] or [N, 4, 4] -> [N, 6]
    """
    # standard implementation
    ret_first = False
    if hmat.ndim == 2:
        hmat = hmat[np.newaxis, ...]
        ret_first = True

    res = np.zeros((hmat.shape[0], 6))

    for i in range(hmat.shape[0]):
        sov3 = _no_scipy_log_rot_mat(hmat[i, :3, :3])

        if np.array_equal(sov3, np.zeros(3)):
            res[i] = np.r_[np.zeros(3), hmat[i, :3, 3]]
            continue

        theta = np.linalg.norm(sov3)
        rot_axis = sov3 / theta
        w = np.array([[0, -rot_axis[2],
                    rot_axis[1]], [rot_axis[2], 0, -rot_axis[0]],
                    [-rot_axis[1], rot_axis[0], 0]])

        t = np.dot(
            np.eye(3) / theta - 0.5 * w +
            (1 / theta - 0.5 / np.tan(theta / 2)) * np.dot(w, w), hmat[i, :3,
                                                                       3])

        res[i] = np.r_[sov3, theta * t]

    return res[0] if ret_first else res
# ==========================================================================
# ==========================================================================


def exp_rot_vec(vec: np.ndarray) -> np.ndarray:
    """Converts so3 vector to a rotation matrix.

    Args:
        vec: so3 rotation vector [3] or vectors [N, 3] to rotation
             matrix [3, 3] or matrices [N, 3, 3]
    Return:
        rotation matrix or matrices
    """
    if no_scipy():
        return _no_scipy_exp_rot_vec(vec)
    return R.from_rotvec(vec).as_matrix()


def log_rot_mat(rm: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to so3 coordinates (i.e. log() operator)

    Args:
        rm: rotation matrix [3, 3] or matrices [N, 3, 3]
    Return:
        so3 coordinate rotation vector [3] or [N, 3]
    """
    if no_scipy():
        return _no_scipy_log_rot_mat(rm)
    return R.from_matrix(rm).as_rotvec()


# Lynch and Park: 3.3.3.2, Page 106, Algorithm
def exp_pose6(pose6: Pose6) -> np.ndarray:
    """Convert exponential poses to homogeneous matrix poses.

    Args:
        pose6: vector [6] or matrix [N, 6] of exponential poses
    Return:
        Homogeneous matrix poses of size [4, 4] or [N, 4, 4].
    """
    if no_scipy():
        return _no_scipy_exp_pose6(pose6)

    single_pose = False
    if pose6.ndim > 2:
        raise ValueError(f"ERROR: Expected 1-,2-dim inputs to exp_pose6. But"
                         f" {pose6.ndim} dimensions provided.")
    elif pose6.ndim == 1:
        single_pose = True
        pose6 = pose6[np.newaxis, :]
    theta = np.linalg.norm(pose6[:, :3], axis=1)
    flag = np.abs(theta) < 1e-9
    if np.all(flag):
        hom = np.tile(np.eye(4), (pose6.shape[0], 1, 1))
        hom[:, :3, 3] += pose6[:, 3:]
        return hom[0] if single_pose else hom

    # overwrite close to zero values
    theta[flag] = 1

    rot_axis = pose6[:, :3] / theta[:, np.newaxis]

    w = np.empty((pose6.shape[0], 3, 3), np.float64)
    w[:, 0, 0] = 0
    w[:, 0, 1] = -rot_axis[:, 2]
    w[:, 0, 2] = rot_axis[:, 1]
    w[:, 1, 0] = rot_axis[:, 2]
    w[:, 1, 1] = 0
    w[:, 1, 2] = -rot_axis[:, 0]
    w[:, 2, 0] = -rot_axis[:, 1]
    w[:, 2, 1] = rot_axis[:, 0]
    w[:, 2, 2] = 0

    hom = np.tile(np.eye(4), (pose6.shape[0], 1, 1))
    hom[:, :3, :3] = R.from_rotvec(pose6[:, :3]).as_matrix()

    temp = \
        np.eye(3)[np.newaxis, :, :] * theta[:, np.newaxis, np.newaxis] + \
        (1 - np.cos(theta))[:, np.newaxis, np.newaxis] * w + \
        ((theta - np.sin(theta))[:, np.newaxis, np.newaxis] *
            np.einsum('ijk,ikl->ijl', w, w))

    hom[:, :3, -1] = np.einsum('ijk,ik,i->ij',
                               temp,
                               pose6[:, 3:],
                               1 / theta,
                               optimize=['einsum_path', (0, 1), (0, 1)])
    # Overwrite bad transforms
    hom[flag] = np.eye(4)[np.newaxis, ...]
    hom[flag, :3, 3] += pose6[flag, 3:]

    return hom[0] if single_pose else hom


def log_pose(pose: PoseH) -> np.ndarray:
    """Convert homogeneous matrix(s) to exp pose coordinates.

    Args:
        pose: homogeneous pose [4, 4] or poses [N, 4, 4]
    Return:
        exp pose coordinates [6] or [N, 6]
    """
    if no_scipy():
        return _no_scipy_log_pose(pose)

    if pose.ndim not in [2, 3]:
        raise ValueError(f"ERROR: Expected 2-,3-dim inputs to log_pose. But"
                         f" {pose.ndim} dimensions provided.")
    single_pose = False
    if pose.ndim == 2:
        single_pose = True
        pose = pose[np.newaxis, :]

    sov3 = R.from_matrix(pose[:, :3, :3]).as_rotvec()

    theta = np.linalg.norm(sov3, axis=1)
    flag_def = theta < 1e-9

    # overwrite close to zero values
    theta[flag_def] = 1

    if np.all(flag_def):
        if single_pose:
            return np.concatenate([np.zeros(3), pose[0, :3, 3].ravel()])
        else:
            return np.c_[np.zeros((pose.shape[0], 3)), pose[:, :3, 3]]

    rot_axis = sov3 / theta[:, np.newaxis]

    w = np.empty((rot_axis.shape[0], 3, 3), np.float64)
    w[:, 0, 0] = 0
    w[:, 0, 1] = -rot_axis[:, 2]
    w[:, 0, 2] = rot_axis[:, 1]
    w[:, 1, 0] = rot_axis[:, 2]
    w[:, 1, 1] = 0
    w[:, 1, 2] = -rot_axis[:, 0]
    w[:, 2, 0] = -rot_axis[:, 1]
    w[:, 2, 1] = rot_axis[:, 0]
    w[:, 2, 2] = 0

    t = np.einsum(
        "i,ijk,ik->ij", theta,
        np.eye(3)[np.newaxis, :, :] / theta[:, np.newaxis, np.newaxis] -
        0.5 * w +
        (1 / theta - 0.5 / np.tan(theta / 2))[:, np.newaxis, np.newaxis] *
        np.einsum('ijk,ikl->ijl', w, w), pose[:, :3, 3])

    res = np.c_[sov3, t]

    # overwrite the zero rotation (i.e. identity) since it
    # was left with ones stub to avoid div zero
    res[flag_def] = np.c_[np.zeros((np.count_nonzero(flag_def), 3)),
                          pose[flag_def, :3, 3]]

    return res[0] if single_pose else res


def pose_interp(p1: Pose,
                p2: Pose,
                t: float,
                *,
                delta_pose6: Optional[Pose6] = None) -> np.ndarray:
    """Pose interpolation between pose1 and pose2 at time t as ratio.

    Args:
        p1: starting pose
        p2: ending pose
        t: ratio between pose p1 and p2 at what point to interpolate,
           not restricted between [0, 1] and can be extended for out
           of bounds
        delta_pose6: pre-calculated difference `inv(p1) @ p2`, saves
                     computation if it's available already
    Return:
        pose of the point at time `t` on the line defined by p1 and p2
        on SE3 manifold
    """
    pose1 = np.array(p1)
    if pose1.ndim == 1:
        pose1 = exp_pose6(pose1)
    pose2 = np.array(p2)
    if pose2.ndim == 1:
        pose2 = exp_pose6(pose2)

    if delta_pose6 is None:
        pose_d = np.dot(np.linalg.inv(pose1), pose2)
        log_pose_d = log_pose(pose_d)
    else:
        log_pose_d = delta_pose6

    pose_d_dt = exp_pose6(log_pose_d * t)

    pose_interp = np.dot(pose1, pose_d_dt)
    return pose_interp


def traj_interp(traj_poses: TrajPoses, ts: Union[Sequence[Numeric],
                                                 np.ndarray]) -> np.ndarray:
    """Trajectory interpolation for points in between.

    TODO[pb]: Extend with `time_bounds` args for traj evaluator when needed
    """
    return TrajectoryEvaluator(traj_poses).poses_at(ts)


class Poser(Protocol):
    """Actor that adds poses to LidarScans"""

    def __call__(self,
                 scan: client.LidarScan,
                 *,
                 col_ts: Optional[np.ndarray] = None) -> client.LidarScan:
        """Add poses to the scan, modifying in-place."""
        ...


class TrajectoryEvaluator(Poser):
    """Interpolates trajectory for a set of timestamps from knot poses.

    TODO[pb]: Add function to add/remove knot poses from traj eval.

    TODO: Optionally, we may want to implement these calculations in C++ and
          use bindings to make it faster.
    """
    def __init__(self, poses: TrajPoses, *, time_bounds: Optional[float] = 0):
        """
        Args:
            poses: List of knot poses with timestamps. Every list item is a
                   tuple (ts, pose).
            time_bounds: whether to restrict the pose interpolation to the timestamp
                         range within the poses list:
                         None - no restriction at all on the timestamps that can
                                be used to get pose from the trajectory
                         0    - strict bounds on the timestamp range in the
                                poses list
                         >0   - ratio that is allowed to go over the timestamp
                                bounds. ratio value is applied as the ratio of
                                pose[1].ts - pose[0].ts for the left bound, and
                                pose[N].ts - pose[N-1].ts for the right bound.

        """
        if len(poses) < 2:
            raise ValueError("TrajectoryEvaluator expects at least 2 poses.")

        is_pose_hom = True if poses[0][1].size == 16 else False
        if not is_pose_hom:
            assert poses[0][1].size == 6, "Expect all poses either [6] or [4, 4] size"

        # check time non-decreasing order
        for i, (ts, p) in enumerate(poses[1:], start=1):
            assert ts > poses[i - 1][0], f"Expected increasing trajectories" \
                f" timestamps but found {ts} <= {poses[i - 1][0]}"
            assert poses[i][1].size == 16 if is_pose_hom else poses[i][1].size == 6, \
                "Expected all poses in a trajectory to be of the same type:" \
                "[6] or [4, 4]"

        self._poses = poses
        self._time_bounds = time_bounds

        if len(self._poses) > 0:
            # pre-calculating intermediaries: poses matrix and poses deltas

            # converting all pose knots to homogeneous form
            poses_arr = np.array([p[1] for p in self._poses])
            self._poses_mat = poses_arr if is_pose_hom else exp_pose6(
                poses_arr)

            # precalculating deltas between adjacent poses
            deltas_mat = np.matmul(np.linalg.inv(self._poses_mat[:-1]),
                                   self._poses_mat[1:])
            # we only need vec6 form for deltas to do interpolations
            self._deltas = np.r_[np.zeros((1, 6)), log_pose(deltas_mat)]

        # store keys separately for bisect function
        self._ts_keys = [x[0] for x in self._poses]

    def _check_ts_and_bounds(self, ts: Union[Sequence[Numeric], np.ndarray]):
        """Checks whether `ts` in the trajectory poses timestamps with bounds"""
        # check time non-decreasing order
        if len(ts) > 1:
            for t1, t2 in zip(ts[:-1], ts[1:]):
                assert t1 <= t2, f"Expected non-decreasing timestamps " \
                    f"but found {t1} > {t2}"
        # no check for bounds is needed
        if self._time_bounds is None:
            return

        # get left/right bounds distance
        left_bound = (self._poses[0][0] - ts[0]) / (self._poses[1][0] -
                                                    self._poses[0][0])
        right_bound = (ts[-1] - self._poses[-1][0]) / (self._poses[-1][0] -
                                                       self._poses[-2][0])

        if left_bound > self._time_bounds or right_bound > self._time_bounds:
            raise ValueError(
                f"Timestamps should be in the poses timestamp range: "
                f"from {self._ts_keys[0]} to {self._ts_keys[-1]}, with "
                f"{self._time_bounds} bounds. But got from {ts[0]} to "
                f"{ts[-1]}")

        # time bounds on the edges are fine
        return

    def pose_at(self, ts: Numeric) -> PoseH:
        """Calculates a single pose (4x4 matrix) at a given `ts` timestamp."""

        self._check_ts_and_bounds([ts])

        if ts < self._poses[0][0]:
            # out on the left
            dt = (ts - self._ts_keys[0]) / (self._ts_keys[1] -
                                            self._ts_keys[0])
            pose_d_dt = exp_pose6(self._deltas[1] * dt)
            pose_base = self._poses_mat[0]
        elif ts >= self._poses[-1][0]:
            # out on the right
            dt = (ts - self._ts_keys[-2]) / (self._ts_keys[-1] -
                                             self._ts_keys[-2])
            pose_d_dt = exp_pose6(self._deltas[-1] * dt)
            pose_base = self._poses_mat[-2]
        else:
            # within traj poses timestamps
            curr_idx = bisect.bisect_right(self._ts_keys, ts)
            dt = (ts - self._ts_keys[curr_idx - 1]) / (
                self._ts_keys[curr_idx] - self._ts_keys[curr_idx - 1])
            pose_d_dt = exp_pose6(self._deltas[curr_idx] * dt)
            pose_base = self._poses_mat[curr_idx - 1]

        return np.dot(pose_base, pose_d_dt)

    def poses_at(self, ts: Union[Sequence[Numeric], np.ndarray]) -> np.ndarray:
        """Calculates multiple poses (4x4 matrices) at a given `ts` timestamps."""
        if len(ts) == 0:
            return np.array([])

        self._check_ts_and_bounds(ts)

        ts_size = len(ts)

        pose_d_dts = np.zeros((ts_size, 6))
        dts = np.zeros(ts_size)
        curr_idx_1_poses = np.tile(np.eye(4), (ts_size, 1, 1))

        curr_idx = 0

        for i in range(ts_size):
            if ts[i] < self._poses[0][0]:
                # out on the left
                dts[i] = (ts[i] - self._ts_keys[0]) / (self._ts_keys[1] -
                                                       self._ts_keys[0])
                pose_d_dts[i] = self._deltas[1]
                curr_idx_1_poses[i] = self._poses_mat[0]
            elif ts[i] >= self._poses[-1][0]:
                # out on the right
                dts[i] = (ts[i] - self._ts_keys[-2]) / (self._ts_keys[-1] -
                                                        self._ts_keys[-2])
                pose_d_dts[i] = self._deltas[-1]
                curr_idx_1_poses[i] = self._poses_mat[-2]
            else:
                if curr_idx == 0:
                    # reach the data range by the curr_idx first
                    curr_idx = bisect.bisect_right(self._ts_keys, ts[i])
                elif ts[i] > self._ts_keys[curr_idx]:
                    # current pose knot (to the right) is already behind, need
                    # to advance to the next pose knot

                    if curr_idx + 1 >= len(self._poses):
                        # no more poses
                        break

                    # trying to get just next pose
                    curr_idx += 1

                    if ts[i] > self._ts_keys[curr_idx]:
                        # do bisect again, because it's a discontinuity here
                        # i.e. the current ts[i] is too far from the
                        # previous one [i-1]
                        curr_idx = bisect.bisect_right(self._ts_keys, ts[i])

                dts[i] = (ts[i] - self._ts_keys[curr_idx - 1]) / (
                    self._ts_keys[curr_idx] - self._ts_keys[curr_idx - 1])
                pose_d_dts[i] = self._deltas[curr_idx]
                curr_idx_1_poses[i] = self._poses_mat[curr_idx - 1]

        pose_d_dts = np.einsum("i,ij->ij", dts, pose_d_dts)
        pose_d_dts_mat = exp_pose6(pose_d_dts)
        return np.matmul(curr_idx_1_poses, pose_d_dts_mat)

    def __bool__(self) -> bool:
        return bool(self._poses)

    def __call__(self,
                 scan: client.LidarScan,
                 *,
                 col_ts: Optional[np.ndarray] = None) -> client.LidarScan:
        """Add poses to the scan.

        Use `col_ts` if the column timestamps need to be remapped to a
        different time scale.

        Args:
            col_ts: optional array of [scan.W] remapped timestamps to use
                    instead of scan.timestamp for pose calculations.
        """
        valid_cols = (np.bitwise_and(scan.status, 1) == 1)
        if col_ts is not None and col_ts.ndim == 1 and col_ts.size == scan.w:
            ts = col_ts[valid_cols]
        else:
            ts = scan.timestamp[valid_cols]

        if ts.size == 0:
            return scan

        scan.pose[valid_cols] = self.poses_at(ts)
        return scan

    def __len__(self) -> int:
        return len(self._poses)

    def __getitem__(self, idx):
        return self._poses[idx]


def dewarp(xyz: np.ndarray, *, scan_pose: Optional[PoseH] = None,
           column_poses: Optional[np.ndarray] = None) -> np.ndarray:
    """Returns transformed: xyz_return = scan_pose @ column_poses @ xyz

    Args:
        xyz: is the return of `client.XYZLut` call, which has (H, W, 3) dimensions
             and column major contiguous storage
        scan_pose: optional. (4, 4) homogeneous pose of the scan
        column_poses: optional. (W, 4, 4) homogeneous poses of the scans column

    Returns:
        xyz with applied scan_pose and/or column_poses:
          xyz_return = scan_pose @ column_poses @ xyz
        binary layout of the returned `xyz` is ensure to match the one that is
        returned by call `client.XYZLut` so it can further used with PointViz
        and other functions that expect this specific layout.
    """

    if xyz.ndim != 3 or xyz.shape[2] != 3:
        raise ValueError("Expects xyz to be (H, W, 3) in dewarp")

    h, w = xyz.shape[0], xyz.shape[1]

    if column_poses is not None:
        if not (column_poses.shape[0] == xyz.shape[1]
                and column_poses.shape[1] == 4 and column_poses.shape[2] == 4):
            raise ValueError("Expects column_poses to be (W, 4, 4) in dewarp")
        if scan_pose is not None:
            xyz_poses = np.einsum('ij,ljk->lik', scan_pose, column_poses)
        else:
            xyz_poses = column_poses

        # Angus's version: This one is correct for sure
        xyz_res = np.transpose(
            np.matmul(xyz_poses[:, :3, :3], np.transpose(xyz, axes=(1, 2, 0))),
            axes=(2, 0, 1)) + xyz_poses[np.newaxis, :, :3, -1]
        xyz_res = np.asfortranarray(xyz_res.reshape((-1, 3)))
        return xyz_res.reshape((h, w, -1))

    if scan_pose is None:
        return xyz

    # Angus's version
    xyz_res = np.transpose(np.matmul(scan_pose[np.newaxis, :3, :3],
                                     np.transpose(xyz, axes=(1, 2, 0))),
                           axes=(2, 0, 1)) + scan_pose[np.newaxis, :3, -1]
    xyz_res = np.asfortranarray(xyz_res.reshape((-1, 3)))
    return xyz_res.reshape((h, w, -1))


ScansIterable = Union[Iterable[client.LidarScan],
                      Iterable[List[Optional[client.LidarScan]]]]


def pose_scans(
    source,
    *,
    poses: Optional[Poser] = None
):
    """Add poses to LidarScans stream.

    Args:
        source: one of:
            - Sequence[client.LidarScan] - single scan sources
            - Sequence[List[Optional[client.LidarScan]]] - multi scans sources
    """

    for obj in source:
        if isinstance(obj, client.LidarScan):
            # Iterator[client.LidarScan]
            yield poses(obj) if poses is not None else obj
        elif isinstance(obj, list):
            # collated scans: List[Optional[LidarScan]]
            if poses is not None:
                yield [
                    poses(scan) if scan is not None else None for scan in obj
                ]
            else:
                yield obj
        else:
            raise ValueError(
                "Expected one of types: LidarScan, Tuple[Optional[LidarScan]]"
                "elements. But got:", type(obj))


def load_kitti_poses(file: str) -> np.ndarray:
    """Loads the Kitti poses from the file.

    Returns:
        [N, 4, 4] array of homogeneous poses
    """
    kitti_poses = np.loadtxt(file, delimiter=" ")
    poses_num = kitti_poses.shape[0]
    poses = np.concatenate(
        (kitti_poses, np.tile(np.array([0, 0, 0, 1]), (poses_num, 1))), axis=1)
    poses = poses.reshape((-1, 4, 4))
    return poses


def make_kiss_traj_poses(poses: Union[Sequence[Pose], np.ndarray]) -> TrajPoses:
    """Makes a traj poses from kiss poses.

    Args:
        poses: pose for every scan in the sequence as returned by KissICP

    Returns:
        trajectory poses timestamped by the scan index mid point: 0.5
        For example scan indexes 0, 1, 2 produce timestamps 0.5, 1.5, 2.5
    """
    traj_poses = list([(0.5 + i, p) for i, p in enumerate(poses)])
    return traj_poses


def pose_scans_from_kitti(
    source,
    kitti_poses: str
):
    """Add poses to LidarScans stream using the previously saved per scan poses.

    Every pose is considered to be in the middle of the scan. We assume that
    very first scan starts at t = 0 and ends at t = 1, thus the first pose
    is timestamped as 0.5, second pose is timestamped at 1.5 (middle of the
    second scan), and so on ... to the very last pose N which timestamped at
    N + 0.5 for the last N scan.

    Args:
        source: one of:
            - Sequence[client.LidarScan] - single scan sources
            - Sequence[List[Optional[client.LidarScan]]] - multi scans sources
        kitti_poses: path to the file with in kitti poses format, i.e. every
                     line contains 12 floats of 4x4 homogeneous transformation
                     matrix (``[:3, :]`` in numpy notation, row-major serialized)
    """

    # load one pose per scan
    poses = load_kitti_poses(kitti_poses)

    # make time indexed poses starting from 0.5
    traj_poses = make_kiss_traj_poses(poses)
    traj_eval = TrajectoryEvaluator(traj_poses, time_bounds=1.5)

    norm_col_ts: Optional[np.ndarray]
    norm_col_ts = None

    scan_idx = -1

    start_scan_frame_id = -1
    start_scan_ts = -1

    for obj in source:
        if isinstance(obj, client.LidarScan):
            # Iterator[client.LidarScan]
            scan = obj

            # checking for the source looping (if frame_id and scan_ts was seen)
            if start_scan_frame_id < 0:
                start_scan_frame_id = scan.frame_id
                start_scan_ts = client.first_valid_column_ts(scan)
            elif (start_scan_frame_id == scan.frame_id and
                  start_scan_ts == client.first_valid_column_ts(scan)):
                # loop detected, reset scan_idx
                scan_idx = -1

            scan_idx += 1
            if norm_col_ts is None:
                norm_col_ts = np.linspace(0, 1.0, scan.w, endpoint=False)
            idx_ts = scan_idx + norm_col_ts

            traj_eval(scan, col_ts=idx_ts)
            yield scan

        elif isinstance(obj, list):
            # collated scans: List[Optional[LidarScan]]
            # TODO[pb]: Make it for multi scan sources when we have stable
            #           multi source interfaces
            raise ValueError("Multi scan sources not yet implemented. Got: ",
                             type(obj))
        else:
            raise ValueError(
                "Expected one of types: LidarScan, Tuple[Optional[LidarScan]]"
                "elements. But got:", type(obj))

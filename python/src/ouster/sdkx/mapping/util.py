# type: ignore

from typing import Union, Tuple, Optional
from pathlib import Path
import numpy as np
from ouster import client, pcap
from ouster.sdk.util import resolve_metadata
import ipaddress


def so3VecToRotMat(vec) -> np.ndarray:
    """Converts so3 vector to a rotation matrix."""

    theta = np.linalg.norm(vec)
    if abs(theta) < 1e-9:
        return np.eye(3)
    rot_axis = vec / theta
    w = np.array([[0, -rot_axis[2], rot_axis[1]],
                  [rot_axis[2], 0, -rot_axis[0]],
                  [-rot_axis[1], rot_axis[0], 0]])
    return np.eye(3) + np.sin(theta) * w \
        + (1 - np.cos(theta)) * np.dot(w, w)


def pose6toHomMatrix(vec: np.ndarray) -> np.ndarray:
    """Convert exponential pose of [6] vector to homogeneous matrix [4,4]."""
    if vec.shape == (4, 4):
        # already in matrix form
        return vec

    if vec.ndim == 2:
        # trying to run vectorized
        try:
            return pose6toHomMatrixVectorized(vec)
        except ImportError:
            # scipy is not there, doing loop
            return np.array([pose6toHomMatrix(v) for v in vec])

    # standard implementation
    theta = np.linalg.norm(vec[:3])
    if abs(theta) < 1e-9:
        return np.r_[np.c_[np.eye(3), vec[3:]], [[0, 0, 0, 1]]]
    rot_axis = vec[:3] / theta
    w = np.array([[0, -rot_axis[2], rot_axis[1]],
                  [rot_axis[2], 0, -rot_axis[0]],
                  [-rot_axis[1], rot_axis[0], 0]])
    hom = np.eye(4)

    hom[:3, :3] = so3VecToRotMat(vec[:3])

    hom[:3, 3] = np.dot(
        np.eye(3) * theta + (1 - np.cos(theta)) * w
        + (theta - np.sin(theta)) * np.dot(w, w),
        vec[3:]) / theta

    return hom


def pose6toHomMatrixVectorized(vec: np.ndarray) -> np.ndarray:
    """Convert exponential poses [N, 6] to homogeneous matrices [N, 4, 4].

    NOTE: This method will fail without scipy, and scipy is not a default
    requirement.
    """
    from scipy.spatial.transform import Rotation as R

    theta = np.linalg.norm(vec[:, :3], axis=1)
    flag = np.abs(theta) < 1e-9
    if np.all(flag):
        hom = np.tile(np.eye(4), (vec.shape[0], 1, 1))
        return hom

    # overwrite close to zero values
    theta[flag] = 1

    rot_axis = vec[:, :3] / theta[:, np.newaxis]

    w = np.empty((vec.shape[0], 3, 3), np.float64)
    w[:, 0, 0] = 0
    w[:, 0, 1] = -rot_axis[:, 2]
    w[:, 0, 2] = rot_axis[:, 1]
    w[:, 1, 0] = rot_axis[:, 2]
    w[:, 1, 1] = 0
    w[:, 1, 2] = -rot_axis[:, 0]
    w[:, 2, 0] = -rot_axis[:, 1]
    w[:, 2, 1] = rot_axis[:, 0]
    w[:, 2, 2] = 0

    hom = np.tile(np.eye(4), (vec.shape[0], 1, 1))
    hom[:, :3, :3] = R.from_rotvec(vec[:, :3]).as_matrix()

    temp = \
        np.eye(3)[np.newaxis, :, :] * theta[:, np.newaxis, np.newaxis] + \
        (1 - np.cos(theta))[:, np.newaxis, np.newaxis] * w + \
        ((theta - np.sin(theta))[:, np.newaxis, np.newaxis] *
         np.einsum('ijk,ikl->ijl', w, w))

    hom[:, :3, -1] = np.einsum('ijk,ik,i->ij',
                               temp,
                               vec[:, 3:],
                               1 / theta,
                               optimize=['einsum_path', (0, 1), (0, 1)])
    # Overwrite bad transforms
    hom[flag] = np.eye(4)[np.newaxis, ...]

    return hom


# Lynch and Park: 3.2.3.3, Page 87, Algorithm
def rotMatToSo3Vec(rm) -> np.ndarray:
    """Convert rotation matrix [3,3] to so3 coordinates (i.e. log() operator)"""
    acos = 0.5 * (np.trace(rm) - 1)
    if acos >= 1:
        return np.zeros(3)
    if acos <= -1:
        if abs(1 + rm[2, 2]) > 1e-9:
            return np.pi * (1.0 / np.sqrt(2 + 2 * rm[2, 2])) * np.array(
                [rm[0, 2], rm[1, 2], 1 + rm[2, 2]])
        if abs(1 + rm[1, 1]) > 1e-9:
            return np.pi * (1.0 / np.sqrt(2 + 2 * rm[1, 1])) * np.array(
                [rm[0, 1], 1 + rm[1, 1], [2, 1]])
        return np.pi * (1.0 / np.sqrt(2 + 2 * rm[0, 0])) * np.array(
            [1 + rm[0, 0], rm[1, 0], [2, 0]])
    theta = np.arccos(acos)
    w = (0.5 / np.sin(theta)) * (rm - rm.transpose())
    return theta * np.array([w[2, 1], w[0, 2], w[1, 0]])


# Lynch and Park: 3.3.3.2, Page 106, Algorithm
def homMatToPose6(hmat):
    """Convert homogeneous matrix [4,4] to exp pose coordinates."""
    if hmat.ndim == 3:
        # trying to run vectorized
        try:
            return homMatToPose6Vectorized(hmat)
        except ImportError:
            # scipy is not there, doing loop
            return np.array([homMatToPose6(m) for m in hmat])

    # standard implementation
    sov3 = rotMatToSo3Vec(hmat[:3, :3])

    if np.array_equal(sov3, np.zeros(3)):
        return np.r_[np.zeros(3), hmat[:3, 3]]

    theta = np.linalg.norm(sov3)
    rot_axis = sov3 / theta
    w = np.array([[0, -rot_axis[2], rot_axis[1]],
                  [rot_axis[2], 0, -rot_axis[0]],
                  [-rot_axis[1], rot_axis[0], 0]])

    t = np.dot(np.eye(3) / theta - 0.5 * w
               + (1 / theta - 0.5 / np.tan(theta / 2)) * np.dot(w, w),
               hmat[:3, 3])

    return np.r_[sov3, theta * t]


def homMatToPose6Vectorized(hmat):
    """Convert homogeneous matrices [N,4,4] to exp pose coordinates [N,6].

    NOTE: This method will fail without scipy, and scipy is not a default
    requirement for ouster-sensor-tools.
    """
    from scipy.spatial.transform import Rotation as R

    sov3 = R.from_matrix(hmat[:, :3, :3]).as_rotvec()

    theta = np.linalg.norm(sov3, axis=1)
    flag_def = theta < 1e-9

    # overwrite close to zero values
    theta[flag_def] = 1

    if np.all(flag_def):
        return np.c_[np.zeros((hmat.shape[0], 3)), hmat[:, :3, 3]]

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
        np.einsum('ijk,ikl->ijl', w, w), hmat[:, :3, 3])

    res = np.c_[sov3, t]

    # overwrite the zero rotation (i.e. identity) since it
    # was left with ones stub to avoid div zero
    res[flag_def] = np.c_[np.zeros((np.count_nonzero(flag_def), 3)),
                          hmat[flag_def, :3, 3]]

    return np.c_[sov3, t]


"""
This function return the global_per_col_pose (4x4) of the input LidarScan by using
the constant pose change rate assumption. This function currently ONLY for KISS-ICP
Pose interpolaton.
KISS-ICP registers point cloud based on the middle ts, so thepose timestamp is
also based on middle ts of a scan.

scan:              |    start_pose  |    end_pose   |
ts (beginning):             |                |
                         start_ts         end_ts

ts (middle):       |                    output_scan
                      funct return-> global_col_poses
start_ts/end_ts are the middle vaild ts of the start_scan/end_scan.
* end_pose MUST BE the KISS-ICP output result of the output_scan
* for example: end_pose = kiss_icp.update(output_scan)
"""


def getScanColPose(start_ts_pose: Tuple[int, np.ndarray],
        end_ts_pose: Tuple[int, np.ndarray],
        scan: client.LidarScan) -> np.ndarray:
    start_pose_h = start_ts_pose[1]
    end_pose_h = end_ts_pose[1]
    diff_log = homMatToPose6(np.dot(end_pose_h, np.linalg.inv(start_pose_h)))

    start_ts = start_ts_pose[0]
    end_ts = end_ts_pose[0]
    delta_ts = end_ts - start_ts

    if delta_ts < 0:
        raise RuntimeError(
            "end pose ts smaller then start pose ts.\nexit"
        )

    # As end_ts_pose is the kiss-icp results of the scan. end_ts will be the
    # middle ts of the scan. Use end_ts as mid_scan_ts to avoid calculation
    per_col_diff_v = [
        ((ts - end_ts) / delta_ts) * diff_log for ts in scan.timestamp
    ]

    per_col_diff_h = pose6toHomMatrix(np.array(per_col_diff_v))

    start_pose_h = start_pose_h[np.newaxis, :, :]
    end_pose_h = end_pose_h[np.newaxis, :, :]
    global_col_poses = np.einsum("ijk,ikl->ijl", per_col_diff_h, end_pose_h)

    return global_col_poses


class MetadataNotFound(Exception):
    pass


def _get_pcap_metadata(source_location, meta_location):
    meta_data = resolve_metadata(source_location, meta_location)
    if not meta_data:
        raise MetadataNotFound(
            "Metadata not found. Check they are present or explcitly pass a path to them."
        )
    with open(meta_data, 'r') as meta_file:
        meta_json = meta_file.read()
        info = client.SensorInfo(meta_json)
    return info


def dewarp(xyz: np.ndarray, scan_pose: Optional[np.ndarray] = None,
           column_poses: Optional[np.ndarray] = None) -> np.ndarray:
    """Returns transformed: xyz_return = scan_pose @ column_poses @ xyz"""

    if scan_pose is None and column_poses is None:
        return xyz

    xyz_1 = np.append(xyz, np.ones((xyz.shape[0], xyz.shape[1], 1)), axis=2)

    if column_poses is not None:
        if scan_pose is not None:
            xyz_poses = np.einsum('ij,ljk->lik', scan_pose, column_poses)
        else:
            xyz_poses = column_poses
        xyz_1 = np.einsum('ijk,lik->lij', xyz_poses, xyz_1)
        return xyz_1[:, :, :3]

    assert scan_pose is not None  # Add assertion to indicate scan_pose cannot be None
    xyz_1 = np.einsum('ij,lkj->lki', scan_pose, xyz_1)
    return xyz_1[:, :, :3]


def Source(
    source_location: Union[str, Path, client.Scans],
    *,
    meta: Optional[Union[str, Path]] = None,
    lidar_port: int = 7502,
    imu_port: int = 7503,
) -> client.Scans:
    # If we already have scans just return them back
    if isinstance(source_location, client.Scans):
        return source_location

    # We do not have return source. Let's get it
    # We run str on paths because they might be Path instances
    source_location = str(source_location)
    source_ext = Path(source_location).suffix
    meta_location = meta and str(meta)

    if source_ext == ".pcap":
        info = _get_pcap_metadata(source_location, meta_location)
        # hard-coded variable for process status printout #

        # reopen pcap file as the old iterator point to the end
        pcap_source = pcap.Pcap(
            str(source_location), info, lidar_port=lidar_port, imu_port=imu_port
        )
        return client.Scans(pcap_source, timeout=None, complete=False, _max_latency=2)
    elif source_ext == ".local" or ipaddress.ip_address(source_location):
        sensor_source = client.Sensor(str(source_location), lidar_port, imu_port)
        return client.Scans(sensor_source, timeout=None, complete=False, _max_latency=2)
    else:
        raise Exception(
            "The library only supports running with a live sensor or a recorded pcap")

"""Miscellaneous utilities."""

from typing import cast

from ouster.sdk import core
import numpy as np


def fov_vertical(info: core.SensorInfo) -> float:
    altitude_zeros = np.count_nonzero(info.beam_altitude_angles == 0.0)
    if altitude_zeros > 1:
        altitudes = info.beam_altitude_angles[np.nonzero(
            info.beam_altitude_angles)]  # type: ignore
    else:
        altitudes = info.beam_altitude_angles

    fov_vertical = np.max(altitudes) - np.min(altitudes)
    if fov_vertical < 1e-9 and len(altitudes) > 1:
        print("WARNING: Sensor has vertical field of view of 0. Check beam_altitude_angles values.")
        fov_vertical = 1.0

    return fov_vertical


def fov_horizontal(info: core.SensorInfo) -> float:
    if len(info.beam_azimuth_angles) == info.format.pixels_per_column:
        fov_horizontal = 360.0
    else:
        azimuth_zeros = np.count_nonzero(info.beam_azimuth_angles == 0.0)
        if azimuth_zeros > 1:
            azimuths = info.beam_azimuth_angles[np.nonzero(
                info.beam_azimuth_angles)]  # type: ignore
        else:
            azimuths = info.beam_azimuth_angles
        fov_horizontal = np.max(azimuths) - np.min(azimuths)

    if fov_horizontal < 1e-9:
        print("WARNING: Sensor has horizontal field of view of 0. Check beam_azimuth_angles values.")
        fov_horizontal = 1.0

    return fov_horizontal


def img_aspect_ratio(info: core.SensorInfo) -> float:
    """Returns 2D image aspect ratio based on sensor FOV angles.

    Uses the order:
        img_aspect_ratio = FOV_vertical / FOV_horizontal
    """
    return fov_vertical(info) / fov_horizontal(info)


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Converts Quaternion [w, x, y, z] to Rotation [3x3] matrix."""
    (q0, q1, q2, q3) = q / np.linalg.norm(q)
    # yapf: disable
    # autopep8: off
    return np.array([
        [2 * (q0 * q0 + q1 * q1) - 1, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 2 * (q0 * q0 + q3 * q3) - 1]
    ])
    # autopep8: on
    # yapf: enable


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Converts a Rotation [3x3] matrix to Quaternion [w, x, y, z]."""
    # yapf: disable
    # autopep8: off
    tr = np.trace(R)
    if tr > 0:
        S = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / S
        x = (R[2, 1] - R[1, 2]) * S
        y = (R[0, 2] - R[2, 0]) * S
        z = (R[1, 0] - R[0, 1]) * S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z])
    # autopep8: on
    # yapf: enable


def position_quaternion_to_transform(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Converts Quaternion + Pose [7] vector to homogeneous [4x4] matrix."""
    r = quaternion_to_rotation_matrix(q)
    return np.r_[np.c_[r, p], [[0, 0, 0, 1]]]


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a 3D rotation matrix.

    Parameters:
        roll   : Rotation about the x-axis (rad)
        pitch  : Rotation about the y-axis (rad)
        yaw    : Rotation about the z-axis (rad)

    Returns:
        R : 3x3 rotation matrix
    """
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return R_z @ (R_y @ R_x)


def xyzrpy_to_matrix(px, py, pz, r, p, y):
    """
    A method that takes position + euler angles (rad) and produces an equivalent 4x4 transform.

    Parameters:
        px, py, pz: position
        r, p, y: rotation expressed in euler angles (rad)

    Returns:
        R : 4x4 transformation matrix
    """
    out = np.eye(4)
    out[:3, 3] = np.array([px, py, pz])
    out[:3, :3] = euler_to_rotation_matrix(r, p, y)
    return out


def xyzq_to_matrix(px, py, pz, qx, qy, qz, qw):
    """
    A method that takes position + quaternion (rad) and produces an equivalent 4x4 transform.

    Parameters:
        px, py, pz: position
        qx, qy, qz, qw: rotation expressed in a quaternion

    Returns:
        R : 4x4 transformation matrix
    """
    out = np.eye(4)
    out[:3, 3] = np.array([px, py, pz])
    out[:3, :3] = quaternion_to_rotation_matrix(np.array([qw, qx, qy, qz]))
    return out


def parse_extrinsics_from_string(extrinsics: str, degrees=True) -> np.ndarray:
    """
    A utility method to parse extrinsics in multiple formats.

    Parameters:
        extrinsics: a string representing a file or extrinsics in a supported format
        degrees: whether angles should be parsed as degress (True by default)

        Acceptale extrinsics formats:
        - A json with containing a per sensor extrinsics
        - identity ; Use this to override any stored extrinsics with identity
        - X Y Z R P Y ; 'R P Y' represent euler angles (deg)
        - X Y Z QX QY QZ QW ; (QX, QY QZ, QW) represent a quaternion
        - n1 n2 .. n16 ; 16 floats representing a 2D array in a row-major order

    Returns:
        R : 4x4 transformation matrix or a filename
    """
    sep = ',' if ',' in extrinsics else ' '
    elements = extrinsics.split(sep)
    if len(elements) == 1:
        # treat as a filename if not 'identity' keyword
        return np.eye(4) if elements[0] == "identity" else cast(np.ndarray, elements[0])

    try:
        float_elements = [float(e) for e in elements]
    except Exception:
        raise ValueError(f"extrinsics values: {elements} could not parsed as numbers")

    if len(float_elements) == 6:
        xyz = float_elements[:3]
        rpy = [np.deg2rad(e) for e in float_elements[3:7]] if degrees else float_elements[3:7]
        return xyzrpy_to_matrix(*xyz, *rpy)
    if len(float_elements) == 7:
        return xyzq_to_matrix(*float_elements)
    if len(float_elements) == 16:
        return np.array([*float_elements]).reshape((4, 4))
    raise ValueError("Unsupported extrinsics format, check `ouster-cli source --help`"
                     " for proper usage")

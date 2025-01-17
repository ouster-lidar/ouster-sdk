#  type: ignore
"""Miscellaneous utilities."""

import os
import json
from typing import Optional, Tuple, List

from ouster.sdk import client
import numpy as np
import tarfile
import re


def fov_vertical(info: client.SensorInfo) -> float:
    altitude_zeros = np.count_nonzero(info.beam_altitude_angles == 0.0)
    if altitude_zeros > 1:
        altitudes = info.beam_altitude_angles[np.nonzero(
            info.beam_altitude_angles)]
    else:
        altitudes = info.beam_altitude_angles

    fov_vertical = np.max(altitudes) - np.min(altitudes)
    if fov_vertical < 1e-9:
        print("WARNING: beam_altitudes_angles shouldn't be all zeros")
        fov_vertical = 1.0

    return fov_vertical


def fov_horizontal(info: client.SensorInfo) -> float:
    if len(info.beam_azimuth_angles) == info.format.pixels_per_column:
        fov_horizontal = 360.0
    else:
        azimuth_zeros = np.count_nonzero(info.beam_azimuth_angles == 0.0)
        if azimuth_zeros > 1:
            azimuths = info.beam_azimuth_angles[np.nonzero(
                info.beam_azimuth_angles)]
        else:
            azimuths = info.beam_azimuth_angles
        fov_horizontal = np.max(azimuths) - np.min(azimuths)

    if fov_horizontal < 1e-9:
        print("WARNING: beam_azimuth_angles shouldn't be all zeros")
        fov_horizontal = 1.0

    return fov_horizontal


def img_aspect_ratio(info: client.SensorInfo) -> float:
    """Returns 2D image aspect ratio based on sensor FOV angles.

    Uses the order:
        img_aspect_ratio = FOV_vertical / FOV_horizontal
    """
    return fov_vertical(info) / fov_horizontal(info)


def quatToRotMat(q: np.ndarray) -> np.ndarray:
    """Converts Quaternion [w, x, y, z] to Rotation [3x3] matrix."""
    (q0, q1, q2, q3) = q
    # yapf: disable
    # autopep8: off
    return np.array([
        [2 * (q0 * q0 + q1 * q1) - 1, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 2 * (q0 * q0 + q3 * q3) - 1]
    ])
    # autopep8: on
    # yapf: enable


def quatPoseToHomMat(qp: np.ndarray) -> np.ndarray:
    """Converts Quaternion + Pose [7] vector to homogeneous [4x4] matrix."""
    rotMat = quatToRotMat(qp[:4])
    return np.r_[np.c_[rotMat, qp[4:]], [[0, 0, 0, 1]]]


def _parse_extrinsics_file(ext_file: str,
                           sensor_names: List[str],
                           destination_frame: str = "world"
                           ) -> List[Optional[Tuple[np.ndarray, str]]]:
    """Parsing extrinsics file and looking for sensor names transforms."""
    with open(ext_file) as json_file:
        return _parse_extrinsics_json(json_file.read(),
                                      sensor_names,
                                      destination_frame,
                                      ext_source=ext_file)


def _parse_extrinsics_json(json_data: str,
                           sensor_names: List[str],
                           destination_frame: str = "world",
                           *,
                           ext_source: str = ""
                           ) -> List[Optional[Tuple[np.ndarray, str]]]:
    """Parsing extrinsics json and looking for sensor names transforms."""
    try:
        extrinsics_data = json.loads(json_data)
    except Exception as e:
        # TODO[pb]: Use logging
        print("ERROR: Can't parse extrinsics_parameters.json file: ", str(e))
        return []

    if "transforms" not in extrinsics_data:
        return []

    def transform_to_elem(t: dict) -> Tuple[np.ndarray, str, str]:
        quat_pose = np.array([
            t["q_w"], t["q_x"], t["q_y"], t["q_z"], t["p_x"], t["p_y"],
            t["p_z"]
        ])
        return (quatPoseToHomMat(quat_pose), t["source_frame"],
                t["destination_frame"])

    transforms = map(transform_to_elem, extrinsics_data["transforms"])
    dest_transforms = filter(lambda e: e[2] == destination_frame, transforms)
    # dict with all (sn: mat) items from extrinsics file
    sn_ext = dict([(wt[1], wt[0]) for wt in dest_transforms])
    extrinsics = [(sn_ext[sn], ext_source) if sn in sn_ext else None
                  for sn in sensor_names]
    return extrinsics


def resolve_extrinsics(
    data_path: str,
    infos: List[client.SensorInfo] = [],
    sensor_names: List[str] = []
) -> List[Optional[Tuple[np.ndarray, str]]]:
    """Find the extrinsics for a data_path and metadata sets.

    Currently looking for extrinsics in:
        - perception `extrinsics_parameters.json` in the same directory as a
          .pcap `data_path`.
        - perception `extrinsics_parameters.json` in the tar _configuration.tar
          in the same directory as a .pcap `data_path`.
        - simulated data `extrinsics.json` in the `data_path` directory with
          the destination frame names `base_link`

    TODO[pb]: Update and extend this method in future to also look for a set of
              `extrinsics.json` files when it will be fully defined.
    """
    snames = sensor_names or [str(info.sn) for info in infos]
    if os.path.splitext(data_path)[1] == ".pcap" or os.path.isdir(data_path):
        ext_file = os.path.join(
            os.path.dirname(data_path) if not os.path.isdir(data_path) else
            data_path, "extrinsic_parameters.json")
        if os.path.exists(ext_file):
            # Found perception extrinsics file
            return _parse_extrinsics_file(ext_file,
                                          sensor_names=snames,
                                          destination_frame="world")
        else:
            # try to get an extrinsics file from a tar file_configuration.tar
            # where it can be as one of:
            #   - extrinsic_parameters.json
            #   - settings/extrinsic_parameters.json
            pcap_base = os.path.splitext(data_path)[0]
            # removing possible `-000` suffix from pcap filename
            pcap_base = re.sub(r'\-\d{3}$', '', pcap_base)
            tar_conf_file = f"{pcap_base}_configuration.tar"
            if os.path.exists(tar_conf_file):
                with tarfile.open(tar_conf_file) as tar:
                    tar_names = tar.getnames()
                    for fname in [
                            "extrinsic_parameters.json",
                            "settings/extrinsic_parameters.json"
                    ]:
                        if fname in tar_names:
                            f = tar.extractfile(fname)
                            return _parse_extrinsics_json(
                                f.read(),
                                sensor_names=snames,
                                destination_frame="world",
                                ext_source=f"tar:{fname}")

        ext_file = os.path.join(data_path, "extrinsics.json")
        if os.path.exists(ext_file):
            # Found simulated data extrinsics file
            return _parse_extrinsics_file(ext_file,
                                          sensor_names=snames,
                                          destination_frame="base_link")

    if os.path.isdir(data_path):
        ext_file = os.path.join(data_path, "extrinsics.json")
        if os.path.exists(ext_file):
            # Found simulated data extrinsics file
            return _parse_extrinsics_file(ext_file,
                                          sensor_names=snames,
                                          destination_frame="base_link")
    return []


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
    out[:3, :3] = quatToRotMat(np.array([qw, qx, qy, qz]))
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
        return np.eye(4) if elements[0] == "identity" else elements[0]

    try:
        elements = [float(e) for e in elements]
    except Exception:
        raise ValueError(f"extrinsics values: {elements} could not parsed as numbers")

    if len(elements) == 6:
        xyz = elements[:3]
        rpy = [np.deg2rad(e) for e in elements[3:7]] if degrees else elements[3:7]
        return xyzrpy_to_matrix(*xyz, *rpy)
    if len(elements) == 7:
        return xyzq_to_matrix(*elements)
    if len(elements) == 16:
        return np.array([*elements]).reshape((4, 4))
    raise ValueError("Unsupported extrinsics format, check `ouster-cli source --help`"
                     " for proper usage")

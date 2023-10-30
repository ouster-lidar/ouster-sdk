#  type: ignore
"""Miscellaneous utilities."""

import os
import json
from typing import Optional, Tuple, List

from ouster import client
import numpy as np
import tarfile
import re


def img_aspect_ratio(info: client.SensorInfo) -> float:
    """Returns 2D image aspect ratio based on sensor FOV angles.

    Uses the order:
        img_aspect_ratio = FOV_vertical / FOV_horizontal
    """

    altitude_zeros = np.count_nonzero(info.beam_altitude_angles == 0.0)
    if altitude_zeros > 1:
        altitudes = info.beam_altitude_angles[np.nonzero(
            info.beam_altitude_angles)]
    else:
        altitudes = info.beam_altitude_angles

    fov_vertical = np.max(altitudes) - np.min(altitudes)

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
        print("WARNNING: beam_azimuth_angles shouldn't be all zeros")
        fov_horizontal = 1.0

    if fov_vertical < 1e-9:
        print("WARNNING: beam_altitudes_angles shouldn't be all zeros")
        fov_vertical = 1.0

    return fov_vertical / fov_horizontal


def quatToRotMat(q: np.ndarray) -> np.ndarray:
    """Converts Quaternion [w, x, y, z] to Rotation [3x3] matrix."""
    (q0, q1, q2, q3) = q
    # yapf: disable
    return np.array([
        [2 * (q0 * q0 + q1 * q1) - 1, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 2 * (q0 * q0 + q3 * q3) - 1]
    ])
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
    except Exception:
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
    snames = sensor_names or [info.sn for info in infos]
    if os.path.splitext(data_path)[1] == ".pcap":
        ext_file = os.path.join(os.path.dirname(data_path),
                                "extrinsic_parameters.json")
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

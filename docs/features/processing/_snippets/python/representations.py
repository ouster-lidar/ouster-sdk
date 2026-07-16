from copy import deepcopy
from pathlib import Path
from typing import Tuple

import numpy as np

from ouster.sdk import core, pcap


def _first_frame(source: core.FrameSetSource) -> core.LidarFrame:
    for frame_set in source:
        for frame in frame_set:
            if frame is not None:
                return frame
    raise RuntimeError("No frames found in source")


def load_first_frame(pcap_path: str,
                    metadata_path: str) -> Tuple[core.SensorInfo, core.LidarFrame]:
    """Utility helper used by the documentation tests."""
    sensor_info = core.SensorInfo(Path(metadata_path).read_text())
    source = pcap.PcapFrameSetSource(pcap_path, sensor_info=[sensor_info])
    return sensor_info, _first_frame(source)


def apply_xyz_transform_via_metadata(pcap_path: str,
                                     metadata_path: str) -> np.ndarray:
    sensor_info, frame = load_first_frame(pcap_path, metadata_path)
    # [doc-stag-repr-transform-metadata]
    # Custom pose: flip Z/Y and translate up 20 m and 1.5 m along X.
    transform = np.eye(4, dtype=np.float64)
    transform[2, 2] = -1.0
    transform[1, 1] = -1.0
    transform[2, 3] = 20000.0  # millimetres (20 m)
    transform[0, 3] = 1500.0   # millimetres (1.5 m)
    # Compose it with the sensor’s existing lidar→sensor transform
    adjusted = deepcopy(sensor_info)
    adjusted.lidar_to_sensor_transform = transform @ sensor_info.lidar_to_sensor_transform

    # Build the LUT from the modified metadata and project the frame.
    xyzlut = core.XYZLut(adjusted, use_extrinsics=False)
    cloud_adjusted = xyzlut(frame)
    return cloud_adjusted
    # [doc-etag-repr-transform-metadata]


def apply_xyz_transform(sensor_info: core.SensorInfo,
                        frame: core.LidarFrame) -> np.ndarray:
    """Project RANGE data and then apply a custom 4x4 transform."""
    # [doc-stag-py-repr-transform]
    # Custom pose: flip Z/Y and translate up 20 m and 1.5 m along X.
    transformation = np.eye(4)
    transformation[2, 2] = -1.0
    transformation[1, 1] = -1.0
    transformation[2, 3] = 20.0  # metres (20000 mm)
    transformation[0, 3] = 1.5   # metres (1500 mm)
    base_xyz = core.XYZLut(sensor_info, use_extrinsics=False)(frame)
    points = base_xyz.reshape(-1, 3)
    rotation = transformation[:3, :3]
    translation = transformation[:3, 3]
    rotated = np.empty_like(points)
    rotated[:, 0] = (points[:, 0] * rotation[0, 0] +
                     points[:, 1] * rotation[0, 1] +
                     points[:, 2] * rotation[0, 2])
    rotated[:, 1] = (points[:, 0] * rotation[1, 0] +
                     points[:, 1] * rotation[1, 1] +
                     points[:, 2] * rotation[1, 2])
    rotated[:, 2] = (points[:, 0] * rotation[2, 0] +
                     points[:, 1] * rotation[2, 1] +
                     points[:, 2] * rotation[2, 2])
    adjusted = rotated + translation
    zero_mask = (frame.field(core.ChanField.RANGE).reshape(-1) == 0)
    adjusted[zero_mask] = 0.0
    return adjusted.reshape(base_xyz.shape)
# [doc-etag-py-repr-transform]


def apply_xyz_transform_w_extrinsics(sensor_info: core.SensorInfo,
                                     frame: core.LidarFrame) -> np.ndarray:
    """Project RANGE data using the metadata extrinsic transform."""
    # [doc-stag-transform-extrinsics]
    lut_with_extrinsics = core.XYZLut(sensor_info, use_extrinsics=True)
    return lut_with_extrinsics(frame)
    # [doc-etag-transform-extrinsics]

# [doc-stag-py-xyzf-transform]
def apply_xyzf_transform(sensor_info: core.SensorInfo,
                         frame: core.LidarFrame) -> np.ndarray:
    """Transform before range multiplication into the LUT."""
    extra = np.eye(4)
    extra[2, 2] = -1.0
    extra[1, 1] = -1.0
    extra[2, 3] = 20.0  # metres (20000 mm)
    extra[0, 3] = 1.5   # metres (1500 mm)

    lut_fn = core.XYZLut(sensor_info, use_extrinsics=False)
    h = sensor_info.format.pixels_per_column
    w = sensor_info.format.columns_per_frame

    ones = np.ones((h, w), dtype=np.uint32)
    twos = np.full((h, w), 2, dtype=np.uint32)
    base_one = lut_fn(ones)
    base_two = lut_fn(twos)

    direction = (base_two - base_one).reshape(-1, 3)
    offset = (base_one.reshape(-1, 3) - direction)

    rotation = extra[:3, :3]
    translation = extra[:3, 3]

    rotated_dir = np.empty_like(direction)
    rotated_dir[:, 0] = (direction[:, 0] * rotation[0, 0] +
                         direction[:, 1] * rotation[0, 1] +
                         direction[:, 2] * rotation[0, 2])
    rotated_dir[:, 1] = (direction[:, 0] * rotation[1, 0] +
                         direction[:, 1] * rotation[1, 1] +
                         direction[:, 2] * rotation[1, 2])
    rotated_dir[:, 2] = (direction[:, 0] * rotation[2, 0] +
                         direction[:, 1] * rotation[2, 1] +
                         direction[:, 2] * rotation[2, 2])

    rotated_offset = np.empty_like(offset)
    rotated_offset[:, 0] = (offset[:, 0] * rotation[0, 0] +
                            offset[:, 1] * rotation[0, 1] +
                            offset[:, 2] * rotation[0, 2]) + translation[0]
    rotated_offset[:, 1] = (offset[:, 0] * rotation[1, 0] +
                            offset[:, 1] * rotation[1, 1] +
                            offset[:, 2] * rotation[1, 2]) + translation[1]
    rotated_offset[:, 2] = (offset[:, 0] * rotation[2, 0] +
                            offset[:, 1] * rotation[2, 1] +
                            offset[:, 2] * rotation[2, 2]) + translation[2]

    ranges = frame.field(core.ChanField.RANGE).reshape(-1, 1)
    points = ranges * rotated_dir + rotated_offset
    zero_mask = ranges[:, 0] == 0
    points[zero_mask] = 0.0
    return points.reshape(h, w, 3).astype(np.float32, copy=False)
# [doc-etag-py-xyzf-transform]


def reflectivity_images(sensor_info: core.SensorInfo,
                        frame: core.LidarFrame):
# [doc-stag-destagger]
    """Get staggered and destaggered reflectivity images."""

    reflectivity = frame.field(core.ChanField.REFLECTIVITY)
    reflectivity_destaggered = core.destagger(sensor_info, reflectivity)
# [doc-etag-destagger]
    return reflectivity, reflectivity_destaggered


# [doc-stag-py-repr-x-image]
def get_x_in_image_form(frame: core.LidarFrame,
                        sensor_info: core.SensorInfo,
                        destaggered: bool = False):
    """Return the destaggered X coordinate image."""
    h = sensor_info.format.pixels_per_column
    w = sensor_info.format.columns_per_frame

    # Get the XYZ in (H, W, 3) numpy array form
    xyzlut = core.XYZLut(sensor_info, use_extrinsics=False)
    xyz = xyzlut(frame)

    # Extract X coordinate (equivalent to cloud.col(0) in C++)
    x_image = xyz[:, :, 0].reshape(h, w)
    # Apply destagger if desired
    if not destaggered:
        return x_image

    x_destaggered = core.destagger(sensor_info, x_image)
    return x_destaggered

# [doc-etag-py-repr-x-image]


if __name__ == "__main__":
    raise SystemExit("This module provides documentation snippets and is not executable.")

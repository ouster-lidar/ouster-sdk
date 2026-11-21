"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.
"""

import json

import numpy as np
import pytest

from ouster.sdk import core


def _range_scaling_map(w, h, fx, fy, cx, cy):
    """Range scaling map |o| that converts planar DEPTH into radial RANGE.

    Mirrors the ZED source plugin: each pixel ray maps into the Ouster frame as
    ox=1, oy=-((u-cx)/fx), oz=-((v-cy)/fy); |o| is its magnitude.
    """
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    ox = np.ones_like(u, dtype=np.float64)
    oy = -((u - cx) / fx)
    oz = -((v - cy) / fy)
    return np.sqrt(ox ** 2 + oy ** 2 + oz ** 2)


def _camera_info(w, h, fx, fy, cx, cy):
    """A CameraInfo carrying only pinhole intrinsics + image dimensions.

    The shared XYZLut derives per-pixel direction/offset from the pinhole model,
    so no beam angles are stored on the CameraInfo. Returns the info plus the
    range scaling map used to build a RANGE image.
    """
    camera_info = core.CameraInfo()
    camera_info.model = core.CameraModel(fx, fy, cx, cy)
    camera_info.format.columns_per_frame = w
    camera_info.format.pixels_per_column = h
    return camera_info, _range_scaling_map(w, h, fx, fy, cx, cy)


def test_camera_xyzlut_range_projection():
    fx, fy, cx, cy = 100.0, 200.0, 10.0, 20.0
    w, h = 4, 3
    camera_info, mag = _camera_info(w, h, fx, fy, cx, cy)

    lut = core.XYZLut(camera_info)

    u, v = 2, 1
    z = 2.0
    range_mm = np.zeros((h, w), dtype=np.uint32)
    range_mm[v, u] = int(round(z * mag[v, u] * 1000.0))

    points = lut(range_mm)
    assert points.shape == (h, w, 3)

    # Equivalent to the old pinhole DEPTH projection in the Ouster sensor frame
    # (X forward, Y left, Z up).
    expected_x = z
    expected_y = -((u - cx) * z) / fx
    expected_z = -((v - cy) * z) / fy
    np.testing.assert_allclose(points[v, u], [expected_x, expected_y, expected_z], atol=1e-2)

    zero_pixels = points[range_mm == 0]
    assert np.all(zero_pixels == 0)


def test_camera_frame_range_projection():
    fx, fy, cx, cy = 100.0, 100.0, 0.0, 0.0
    w, h = 2, 2
    camera_info, mag = _camera_info(w, h, fx, fy, cx, cy)

    frame = core.CameraFrame()
    frame.w = w
    frame.h = h
    range_mm = np.zeros((h, w), dtype=np.uint32)
    range_mm[0, 1] = int(round(1.0 * mag[0, 1] * 1000.0))
    range_mm[1, 0] = int(round(2.0 * mag[1, 0] * 1000.0))
    frame.add_field('RANGE', range_mm)

    lut = core.XYZLut(camera_info)
    points = lut(frame)
    # Ouster sensor frame (X forward, Y left, Z up).
    np.testing.assert_allclose(points[0, 1], [1.0, -0.01, 0.0], atol=1e-2)
    np.testing.assert_allclose(points[1, 0], [2.0, 0.0, -0.02], atol=1e-2)


@pytest.mark.parametrize("info_type", [core.CameraInfo, core.LidarInfo])
def test_sensor_info_sensor_to_body_default(info_type):
    np.testing.assert_allclose(info_type().sensor_to_body, np.eye(4))


def test_camera_info_sensor_to_body_defaults_when_absent_from_json():
    camera_info = core.CameraInfo()
    camera_info.sensor_to_body = np.diag([2.0, 2.0, 2.0, 1.0])
    root = json.loads(camera_info.to_json_string())
    del root["camera_info"]["sensor_to_body"]

    restored = core.CameraInfo(json.dumps(root))
    np.testing.assert_allclose(restored.sensor_to_body, np.eye(4))


def test_camera_info_transform_defaults():
    camera_info = core.CameraInfo()
    # A camera's IMU is treated as coincident with the sensor frame by default.
    np.testing.assert_allclose(camera_info.imu_to_sensor_transform, np.eye(4))
    # Cameras do not carry the lidar-specific beam/lidar transforms.
    assert not hasattr(camera_info, "beam_to_lidar_transform")
    assert not hasattr(camera_info, "lidar_to_sensor_transform")
    assert not hasattr(camera_info, "beam_azimuth_angles")
    assert not hasattr(camera_info, "beam_altitude_angles")


def test_camera_info_sensor_to_body_json_round_trip():
    camera_info = core.CameraInfo()
    camera_info.model = core.CameraModel(100.0, 100.0, 0.0, 0.0)

    transform = np.array(
        [
            [1.0, 0.0, 0.0, 0.5],
            [0.0, 1.0, 0.0, -0.25],
            [0.0, 0.0, 1.0, 1.5],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    camera_info.sensor_to_body = transform

    restored = core.CameraInfo(camera_info.to_json_string())
    np.testing.assert_allclose(restored.sensor_to_body, transform)
    assert restored == camera_info


def test_camera_info_model_json_round_trip():
    w, h = 4, 3
    camera_info, _ = _camera_info(w, h, 100.0, 200.0, 10.0, 20.0)

    restored = core.CameraInfo(camera_info.to_json_string())
    assert restored.format.columns_per_frame == w
    assert restored.format.pixels_per_column == h
    model = restored.model
    assert model is not None
    assert model.fx == 100.0
    assert model.fy == 200.0
    assert model.cx == 10.0
    assert model.cy == 20.0
    assert restored == camera_info


def test_camera_frame_timestamp_and_body_to_world():
    frame = core.CameraFrame()
    frame.w, frame.h = 4, 3

    assert frame.timestamp.shape == (1,)
    assert frame.body_to_world.shape == (1, 4, 4)
    np.testing.assert_array_equal(frame.body_to_world[0], np.eye(4))

    frame.timestamp[0] = 123
    assert frame.timestamp[0] == 123

    pose = np.eye(4)
    pose[0, 3] = 1.5
    frame.body_to_world[0] = pose
    np.testing.assert_allclose(frame.body_to_world[0], pose)

    # Image width is independent of timestamp / pose length.
    frame.w = 1920
    assert frame.timestamp.shape == (1,)
    assert frame.body_to_world.shape == (1, 4, 4)


def test_camera_xyzlut_rejects_non_positive_focal_length():
    """The model's constructor guards fx/fy, but its members stay mutable."""
    camera_info = core.CameraInfo()
    camera_info.model = core.CameraModel(100.0, 100.0, 2.0, 1.0)
    camera_info.format.columns_per_frame, camera_info.format.pixels_per_column = 4, 3
    # The unmutated model builds a LUT fine.
    core.XYZLut(camera_info)

    for bad in [0.0, -1.0, float('nan')]:
        for attr in ['fx', 'fy']:
            model = core.CameraModel(100.0, 100.0, 2.0, 1.0)
            setattr(model, attr, bad)
            camera_info.model = model
            with pytest.raises(ValueError):
                core.XYZLut(camera_info)

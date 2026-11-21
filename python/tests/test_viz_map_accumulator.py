from typing import cast

import numpy as np

from .conftest import MockPointViz
from ouster.sdk import core
from ouster.sdk.core import ChanField, LidarMode, LidarFrame, LidarInfo
from ouster.sdk._bindings.viz import PointViz
from ouster.sdk.viz.accumulators_config import LidarFrameVizAccumulatorsConfig
from ouster.sdk.viz.map_accumulator import MapAccumulator
from ouster.sdk.viz.model import LidarFrameVizModel
from ouster.sdk.viz.track import Track


def _make_map_accumulator(meta: LidarInfo, max_points: int = 10) -> MapAccumulator:
    viz = cast(PointViz, MockPointViz())
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    config = LidarFrameVizAccumulatorsConfig(
        map_enabled=True,
        map_select_ratio=1.0,
        map_max_points=max_points
    )
    return MapAccumulator(model, viz, Track(config), config)


def _camera_beam_angles(w, h, fx, fy, cx, cy):
    """Per-pixel beam angles (deg) + range scaling map, mirroring source_zed."""
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    x_cam = (u - cx) / fx
    y_cam = (v - cy) / fy
    z_cam = np.ones_like(x_cam)
    ox = z_cam
    oy = -x_cam
    oz = -y_cam
    mag = np.sqrt(ox ** 2 + oy ** 2 + oz ** 2)
    az = np.degrees(np.arctan2(oy, ox)).flatten()
    alt = np.degrees(np.arcsin(oz / mag)).flatten()
    return az, alt, mag


def _make_camera_info(w: int = 4, h: int = 3) -> core.CameraInfo:
    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(100.0, 100.0, (w - 1) / 2.0, (h - 1) / 2.0)
    cam_info.sensor_to_body = np.eye(4)
    cam_info.format.columns_per_frame = w
    cam_info.format.pixels_per_column = h
    # The shared XYZLut derives per-pixel direction/offset from the pinhole
    # intrinsics, so no beam angles are stored on the CameraInfo.
    return cam_info


def _make_camera_map_accumulator(cam_info: core.CameraInfo, max_points: int = 100):
    viz = cast(PointViz, MockPointViz())
    model = LidarFrameVizModel(viz, [cam_info])
    config = LidarFrameVizAccumulatorsConfig(
        map_enabled=True,
        map_select_ratio=1.0,
        map_max_points=max_points
    )
    return model, MapAccumulator(model, viz, Track(config), config)


def _make_camera_frame(cam_info: core.CameraInfo, w: int = 4, h: int = 3,
                       pose: np.ndarray = np.eye(4)) -> core.CameraFrame:
    frame = core.CameraFrame(cam_info)
    # LidarFrame derives w/h from sensor_info (read-only), but a CameraFrame
    # has no constructor that sizes it, so w/h are writable on CameraFrame
    # (shadowing the read-only base Frame properties).
    frame.w = w
    frame.h = h
    depth = np.zeros((h, w), dtype=np.float64)
    depth[1, 1:3] = 2.0
    _, _, mag = _camera_beam_angles(w, h, 100.0, 100.0, (w - 1) / 2.0, (h - 1) / 2.0)
    range_mm = np.ascontiguousarray(
        np.where(depth > 0.0, depth * mag * 1000.0, 0.0).astype(np.uint32))
    frame.add_field("RANGE", range_mm)
    frame.body_to_world[0] = pose
    return frame


def test_update_map_projects_camera_depth():
    """Camera-only frames should add valid RANGE points (in world) to the map."""
    cam_info = _make_camera_info()
    model, accum = _make_camera_map_accumulator(cam_info)

    frame = _make_camera_frame(cam_info)
    frames = core.FrameSet([frame])
    model.update(frames)
    accum.update(frames, frame_num=0)

    # two non-zero RANGE pixels selected at ratio 1.0
    assert accum._map_idx == 2
    assert "RANGE" in accum._map_keys
    assert np.all(np.isfinite(accum._map_xyz[:2]))
    # Ouster sensor frame: depth of 2m maps to X (forward) ~= 2
    np.testing.assert_allclose(accum._map_xyz[:2, 0], [2.0, 2.0], atol=1e-4)


def test_update_map_applies_camera_pose_translation():
    """The camera body_to_world pose should offset accumulated points."""
    cam_info = _make_camera_info()
    model, accum = _make_camera_map_accumulator(cam_info)

    pose = np.eye(4)
    pose[:3, 3] = [5.0, -3.0, 1.0]
    frame = _make_camera_frame(cam_info, pose=pose)
    frames = core.FrameSet([frame])
    model.update(frames)
    accum.update(frames, frame_num=0)

    assert accum._map_idx == 2
    # Ouster sensor frame: depth (2m) maps to X (forward), so
    # world X = 2 + pose x-translation (5m); world Z = 0 + pose z-translation (1m).
    np.testing.assert_allclose(accum._map_xyz[:2, 0], [7.0, 7.0], atol=1e-4)
    np.testing.assert_allclose(accum._map_xyz[:2, 2], [1.0, 1.0], atol=0.05)


def test_update_map_preserves_vector_key_shape_and_dtype():
    """It should cache vector-valued cloud keys for RGB and normals modes."""
    meta = LidarInfo.from_default(LidarMode._1024x10)
    accum = _make_map_accumulator(meta)
    frame = LidarFrame(meta)
    frame.status[:] = 1
    frame.timestamp[:] = np.arange(frame.w)
    frame.field(ChanField.RANGE)[:] = 0
    frame.field(ChanField.RANGE)[:2, :2] = 1000

    rgb = np.zeros((meta.h, meta.w, 3), dtype=np.uint8)
    rgb[:] = [10, 20, 30]
    frame.add_field("RGB", rgb)

    normals = np.zeros((meta.h, meta.w, 3), dtype=np.float32)
    normals[:] = [1.0, 0.0, -1.0]
    frame.add_field("NORMALS", normals)

    accum._model._amend_view_modes_all([frame])
    accum.update([frame], frame_num=0)

    assert accum._map_idx == 4
    assert accum._map_keys["RGB"].shape == (10, 3)
    assert accum._map_keys["RGB"].dtype == np.uint8
    np.testing.assert_array_equal(accum._map_keys["RGB"][:4], [[10, 20, 30]] * 4)
    assert accum._map_keys["NORMALS"].shape == (10, 3)
    assert accum._map_keys["NORMALS"].dtype == np.float32
    np.testing.assert_allclose(accum._map_keys["NORMALS"][:4], [[1.0, 0.5, 0.0]] * 4)

    accum._cloud_mode_name = "RGB"
    accum._draw_map()


def test_ensure_structs_map_preserves_key_trailing_shape_and_dtype():
    """It should keep scalar and vector key shapes when map storage grows."""
    meta = LidarInfo.from_default(LidarMode._1024x10)
    accum = _make_map_accumulator(meta)
    accum._map_xyz = np.zeros((2, 3), dtype=np.float32)
    accum._map_keys["REFLECTIVITY"] = np.array([0.1, 0.2], dtype=np.float32)
    accum._map_keys["RGB"] = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)
    accum._map_idx = 2

    accum._ensure_structs_map(1)

    grown_size = accum._map_xyz.shape[0]
    assert grown_size > 2
    assert accum._map_keys["REFLECTIVITY"].shape == (grown_size,)
    assert accum._map_keys["REFLECTIVITY"].dtype == np.float32
    np.testing.assert_allclose(accum._map_keys["REFLECTIVITY"][:2], [0.1, 0.2])
    assert accum._map_keys["RGB"].shape == (grown_size, 3)
    assert accum._map_keys["RGB"].dtype == np.uint8
    np.testing.assert_array_equal(accum._map_keys["RGB"][:2], [[1, 2, 3], [4, 5, 6]])

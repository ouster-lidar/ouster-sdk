"""Tests for the ground segmentation engine.

These focus on the generalization of ``GroundSegEngine`` to any ``Frame`` that
carries a ``RANGE`` field. A ``CameraFrame`` (single return, unstaggered image,
single rigid pose) must be segmented just like a ``LidarFrame``.
"""
import numpy as np

from ouster.sdk import core
from ouster.sdk.algorithm import GroundSegEngine

# Synthetic pinhole camera looking forward (Ouster sensor frame: X forward,
# Y left, Z up).
_CAM_W, _CAM_H, _CAM_F = 160, 120, 90.0


def _make_camera_info(sensor_to_body):
    info = core.CameraInfo()
    info.format.columns_per_frame = _CAM_W
    info.format.pixels_per_column = _CAM_H
    info.model = core.CameraModel(_CAM_F, _CAM_F, _CAM_W / 2.0, _CAM_H / 2.0)
    info.sensor_to_body = np.asarray(sensor_to_body, dtype=np.float64)
    return info


def _ray_range_scale():
    """Per-pixel magnitude that converts planar X depth into radial RANGE."""
    u, v = np.meshgrid(np.arange(_CAM_W), np.arange(_CAM_H))
    ox = np.ones_like(u, dtype=np.float64)
    oy = -(u - _CAM_W / 2.0) / _CAM_F
    oz = -(v - _CAM_H / 2.0) / _CAM_F
    return np.sqrt(ox ** 2 + oy ** 2 + oz ** 2)


def _floor_and_wall_range_mm(floor_z=-1.5, wall_x=6.0):
    """Render a RANGE image of a flat floor (Z=floor_z) plus a back wall."""
    u = np.arange(_CAM_W)
    v = np.arange(_CAM_H)
    uu, vv = np.meshgrid(u, v)
    directions = np.stack(
        [
            np.ones_like(uu, dtype=np.float64),
            -(uu - _CAM_W / 2.0) / _CAM_F,
            -(vv - _CAM_H / 2.0) / _CAM_F,
        ],
        axis=-1,
    )
    best = np.full((_CAM_H, _CAM_W), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        t_wall = wall_x / directions[..., 0]
        best = np.where(t_wall > 0.05, np.minimum(best, t_wall), best)
        t_floor = floor_z / directions[..., 2]
        best = np.where(t_floor > 0.05, np.minimum(best, t_floor), best)
    depth = np.where(np.isfinite(best), best, 0.0)
    range_mm = np.where(depth > 0.0, depth * _ray_range_scale() * 1000.0, 0.0)
    return np.ascontiguousarray(range_mm.astype(np.uint32))


def _make_camera_frame(range_mm, info):
    frame = core.CameraFrame()
    frame.w = _CAM_W
    frame.h = _CAM_H
    frame.add_field(core.ChanField.RANGE, range_mm)
    frame.sensor_info = info
    return frame


def test_ground_seg_segments_camera_frame():
    """GroundSegEngine annotates a CameraFrame with a valid GROUND mask.

    The camera's extrinsic is gravity-aligned (identity), so the floor plane's
    normals point along +Z and should be classified as ground. This exercises
    the Frame-base generalization: single return, unstaggered image, and a
    single frame-level status broadcast across all image columns.
    """
    range_mm = _floor_and_wall_range_mm()
    frames = core.FrameSet([_make_camera_frame(range_mm, _make_camera_info(np.eye(4)))])

    GroundSegEngine.create().update(frames)

    ground_mask = frames[0].field(core.ChanField.GROUND)
    assert ground_mask.shape == (_CAM_H, _CAM_W)
    assert ground_mask.dtype == np.uint8
    assert set(np.unique(ground_mask)) <= {0, 1}
    # The floor fills the lower portion of the image, so a substantial number of
    # pixels must be classified as ground (proving the whole frame is processed,
    # not just column 0).
    assert ground_mask.sum() > 1000
    # A camera is single-return, so no GROUND2 field is produced.
    assert not frames[0].has_field(core.ChanField.GROUND2)

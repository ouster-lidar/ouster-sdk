"""Tests for running object detection on camera data.

These focus on the generalization of the classic ``DetectionEngine`` from
``LidarFrame`` to any ``Frame`` carrying a ``RANGE`` field. A ``CameraFrame``
is a single-return, unstaggered image with one global-shutter pose, so it
needs no motion compensation, but its points are still placed in the world
using that pose.
"""
import sys

import numpy as np
import pytest

from ouster.sdk import core
from ouster.sdk.perception import ClassicDetectionConfig, DetectionEngine

# Synthetic pinhole camera looking forward (Ouster sensor frame: X forward,
# Y left, Z up).
_CAM_W, _CAM_H, _CAM_F = 160, 120, 90.0
_CX, _CY = _CAM_W / 2.0, _CAM_H / 2.0
_FLOOR_Z = -1.5
_TIMESTAMP_NS = 1_000_000_000


def _ray_directions():
    """Unnormalized per-pixel rays, scaled so that the X component is 1.

    With this scaling the ray parameter is the planar X depth, which makes the
    box/plane intersections below easy to write.
    """
    u, v = np.meshgrid(np.arange(_CAM_W), np.arange(_CAM_H))
    return np.stack(
        [
            np.ones_like(u, dtype=np.float64),
            -(u - _CX) / _CAM_F,
            -(v - _CY) / _CAM_F,
        ],
        axis=-1,
    )


def _render(boxes, max_depth=25.0):
    """Ray-cast a flat floor plus a set of axis-aligned boxes.

    Returns the radial RANGE image in millimetres and the planar X depth.
    """
    directions = _ray_directions()
    nearest = np.full((_CAM_H, _CAM_W), np.inf)

    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = _FLOOR_Z / directions[..., 2]
    nearest = np.where((t_floor > 0.05) & (t_floor < max_depth),
                       np.minimum(nearest, t_floor), nearest)

    for lower, upper in boxes:
        t_enter = np.full((_CAM_H, _CAM_W), 0.05)
        t_exit = np.full((_CAM_H, _CAM_W), np.inf)
        for axis in range(3):
            with np.errstate(divide="ignore", invalid="ignore"):
                t0 = lower[axis] / directions[..., axis]
                t1 = upper[axis] / directions[..., axis]
            axis_enter = np.nan_to_num(np.minimum(t0, t1), nan=-np.inf)
            axis_exit = np.nan_to_num(np.maximum(t0, t1), nan=np.inf)
            t_enter = np.maximum(t_enter, axis_enter)
            t_exit = np.minimum(t_exit, axis_exit)
        nearest = np.where(t_enter <= t_exit, np.minimum(nearest, t_enter), nearest)

    depth = np.where(np.isfinite(nearest), nearest, 0.0)
    magnitude = np.linalg.norm(directions, axis=-1)
    range_mm = np.ascontiguousarray((depth * magnitude * 1000.0).astype(np.uint32))
    return range_mm, depth


def _make_camera_info(sn=1):
    info = core.CameraInfo()
    info.sn = sn
    info.model = core.CameraModel(_CAM_F, _CAM_F, _CX, _CY)
    info.format.columns_per_frame = _CAM_W
    info.format.pixels_per_column = _CAM_H
    return info


def _make_camera_frame(range_mm, depth, info, body_to_world=None):
    """Build a CameraFrame carrying RANGE and a floor GROUND mask.

    The engine's built-in ground model needs several frames to settle, so
    these single-frame tests supply the ground labels directly, the same way
    the ``ground_seg`` processor would.
    """
    frame = core.CameraFrame()
    frame.w = _CAM_W
    frame.h = _CAM_H
    frame.add_field(core.ChanField.RANGE, range_mm)

    height = depth * _ray_directions()[..., 2]
    ground = ((depth > 0.0) & (np.abs(height - _FLOOR_Z) < 0.05)).astype(np.uint8)
    frame.add_field("GROUND", np.ascontiguousarray(ground))

    frame.sensor_info = info
    np.asarray(frame.timestamp)[0] = _TIMESTAMP_NS
    if body_to_world is not None:
        np.asarray(frame.body_to_world)[0] = body_to_world
    return frame


def _object_positions(frame):
    """World positions of the detected objects, sorted by distance."""
    positions = [
        np.asarray((obj.body_to_world * obj.object_to_body).position)
        for obj in frame.objects["classic"]
    ]
    return sorted(positions, key=np.linalg.norm)


# Two upright boxes standing on the floor, one to the right of the optical
# axis and one to the left.
_TWO_BOXES = [
    ((3.0, -1.2, _FLOOR_Z), (3.6, -0.6, _FLOOR_Z + 1.7)),
    ((4.0, 0.5, _FLOOR_Z), (4.6, 1.1, _FLOOR_Z + 1.7)),
]


@pytest.mark.perception
def test_detect_objects_in_camera_frame():
    """The classic engine finds objects in a CameraFrame and labels pixels."""
    info = _make_camera_info()
    frame = _make_camera_frame(*_render(_TWO_BOXES), info)

    DetectionEngine.create(sensor_infos=[info]).update(frame)

    positions = _object_positions(frame)
    assert len(positions) == 2
    # Only the faces turned towards the camera are visible, so each centroid
    # sits slightly in front of the box centre.
    np.testing.assert_allclose(positions[0], [3.3, -0.9, -0.65], atol=0.35)
    np.testing.assert_allclose(positions[1], [4.3, 0.8, -0.65], atol=0.35)

    # Instance ids are written back onto the camera frame, one per object.
    instance_ids = frame.field("classic::INSTANCE_ID")
    assert instance_ids.shape == (_CAM_H, _CAM_W)
    np.testing.assert_array_equal(np.unique(instance_ids), [0, 1, 2])


@pytest.mark.perception
def test_camera_frame_objects_follow_the_frame_pose():
    """A camera frame is dewarped by its pose even though it is not deskewed.

    Translating the body-to-world pose must translate every detected object by
    the same amount.
    """
    info = _make_camera_info()
    range_mm, depth = _render(_TWO_BOXES)

    offset = np.array([10.0, -4.0, 2.0])
    pose = np.eye(4)
    pose[:3, 3] = offset

    at_origin = _make_camera_frame(range_mm, depth, info)
    moved = _make_camera_frame(range_mm, depth, info, body_to_world=pose)

    engine = DetectionEngine.create(sensor_infos=[info])
    engine.update(at_origin)
    engine.update(moved)

    origin_positions = _object_positions(at_origin)
    moved_positions = _object_positions(moved)
    assert len(origin_positions) == len(moved_positions) == 2
    for before, after in zip(origin_positions, moved_positions):
        np.testing.assert_allclose(after, before + offset, atol=1e-3)


@pytest.mark.perception
def test_camera_image_columns_do_not_wrap():
    """The left and right borders of a camera image are not neighbours.

    A 360 degree lidar's first and last columns are adjacent in space, so the
    clusterer wraps across them. Doing that on a camera would merge unrelated
    objects sitting against opposite borders of the image.
    """
    info = _make_camera_info()
    # Both boxes are at the same depth and run off opposite edges of the image.
    boxes = [
        ((4.0, 3.0, _FLOOR_Z), (4.6, 4.5, _FLOOR_Z + 1.7)),
        ((4.0, -4.5, _FLOOR_Z), (4.6, -3.0, _FLOOR_Z + 1.7)),
    ]
    frame = _make_camera_frame(*_render(boxes), info)

    DetectionEngine.create(sensor_infos=[info]).update(frame)

    positions = _object_positions(frame)
    assert len(positions) == 2
    # One object on each side of the optical axis, rather than a single
    # cluster spanning the whole image.
    assert min(p[1] for p in positions) < -2.5
    assert max(p[1] for p in positions) > 2.5


@pytest.mark.perception
def test_detect_camera_frame_without_reflectivity_or_near_ir():
    """A camera stream usually carries only RANGE; the engine must cope."""
    info = _make_camera_info()
    frame = _make_camera_frame(*_render(_TWO_BOXES), info)
    assert not frame.has_field(core.ChanField.REFLECTIVITY)
    assert not frame.has_field(core.ChanField.NEAR_IR)

    DetectionEngine.create(sensor_infos=[info]).update(frame)

    assert len(frame.objects["classic"]) == 2


def _make_rgb_only_camera_frame(info):
    """A camera frame with pixels but no RANGE, as an RGB-only stream gives."""
    frame = core.CameraFrame()
    frame.w = _CAM_W
    frame.h = _CAM_H
    frame.add_field("RGB", np.zeros((_CAM_H, _CAM_W, 3), dtype=np.uint8))
    frame.sensor_info = info
    np.asarray(frame.timestamp)[0] = _TIMESTAMP_NS
    return frame


@pytest.mark.perception
def test_detect_camera_frame_without_range_is_left_alone():
    """Clustering is driven by RANGE, so a frame without it is passed over.

    An RGB-only camera stream has nothing to cluster. That is a stream the
    engine cannot do anything with, not a malformed frame, so it goes through
    untouched rather than failing.
    """
    info = _make_camera_info()
    frame = _make_rgb_only_camera_frame(info)

    DetectionEngine.create(sensor_infos=[info]).update(frame)

    assert len(frame.objects) == 0
    assert frame.fields == ["RGB"]


@pytest.mark.perception
def test_detect_frame_set_mixing_range_and_rangeless_cameras():
    """One sensor having nothing to cluster does not stop the others."""
    detectable_info = _make_camera_info(sn=1)
    rgb_only_info = _make_camera_info(sn=2)

    detectable = _make_camera_frame(*_render(_TWO_BOXES), detectable_info)
    rgb_only = _make_rgb_only_camera_frame(rgb_only_info)

    frame_set = core.FrameSet([detectable, rgb_only])
    engine = DetectionEngine.create(
        sensor_infos=[detectable_info, rgb_only_info])
    engine.update(frame_set)

    assert len(frame_set.objects["classic"]) == 2
    assert detectable.has_field("classic::INSTANCE_ID")
    assert rgb_only.fields == ["RGB"]


# FIXME ClassicDetect is nondeterministic. We should identify why it isn't and fix the issue.
# @pytest.mark.perception
@pytest.mark.skip(reason="This test fails periodically due to a race condition in the ClassicDetect engine.")
def test_detect_frame_set_with_lidar_and_camera(test_data_dir):
    """A mixed FrameSet is detected on in one pass, not lidar frames only."""
    from copy import deepcopy
    from ouster.sdk import open_source

    with open_source(str(test_data_dir / "osfs" / "single_scan_016.osf")) as source:
        lidar_info = source.sensor_info[0]
        lidar_frame = next(iter(source))[0]

    camera_info = _make_camera_info(sn=lidar_info.sn + 1)
    camera_frame = _make_camera_frame(*_render(_TWO_BOXES), camera_info)

    lidar_only = core.FrameSet([deepcopy(lidar_frame)])
    mixed = core.FrameSet([lidar_frame, camera_frame])

    engine = DetectionEngine.create(sensor_infos=[lidar_info, camera_info])
    engine.update(lidar_only)
    engine.update(mixed)

    # The camera contributes its own objects on top of the lidar's, and both
    # frames come back labelled.
    lidar_object_count = len(lidar_only.objects["classic"])
    assert lidar_object_count > 0
    assert len(mixed.objects["classic"]) == lidar_object_count + 2
    assert lidar_frame.has_field("classic::INSTANCE_ID")
    assert camera_frame.has_field("classic::INSTANCE_ID")


@pytest.mark.perception
def test_detect_rejects_frame_from_unregistered_sensor():
    """A frame whose sensor was not registered is reported, not silently
    skipped."""
    info = _make_camera_info(sn=1)
    frame = _make_camera_frame(*_render(_TWO_BOXES), info)

    engine = DetectionEngine.create(sensor_infos=[_make_camera_info(sn=99)])
    with pytest.raises(ValueError, match="not.*registered"):
        engine.update(frame)


@pytest.mark.perception
@pytest.mark.skipif(sys.platform == "win32", reason="Does not run on Windows")
def test_cli_detect_on_camera_osf(tmp_path):
    """``ouster-cli source <camera osf> detect save`` behaves as it does for a
    lidar OSF: it runs the engine and persists the collations it produced."""
    from click.testing import CliRunner
    from ouster.cli import core as cli_core
    from ouster.cli.plugins.perception import source_detect  # noqa: F401
    from ouster.sdk import open_source, osf

    info = _make_camera_info()
    range_mm, depth = _render(_TWO_BOXES)

    input_osf = str(tmp_path / "camera.osf")
    with osf.Writer(input_osf) as writer:
        writer.add_sensor(info)
        for n in range(2):
            frame = _make_camera_frame(range_mm, depth, info)
            writer.save(0, frame, _TIMESTAMP_NS + n)

    output_osf = str(tmp_path / "detected.osf")
    result = CliRunner().invoke(
        cli_core.cli, ["source", input_osf, "detect", "save", output_osf])
    assert result.exit_code == 0, result.output

    with open_source(output_osf) as source:
        assert source.contains_collations
        frame_sets = list(source)
    assert len(frame_sets) == 2
    for frame_set in frame_sets:
        assert len(frame_set.objects["classic"]) == 2


@pytest.mark.perception
def test_classic_config_is_honoured_for_camera_frames():
    """Cluster filtering applies to camera clusters the same way it does to
    lidar clusters."""
    info = _make_camera_info()
    frame = _make_camera_frame(*_render(_TWO_BOXES), info)

    config = ClassicDetectionConfig()
    # The boxes are ~1.7 m tall, so requiring 3 m rejects both of them.
    config.cluster_filter_min_vertical_size = 3.0
    DetectionEngine.create(sensor_infos=[info], config=config).update(frame)

    assert len(frame.objects["classic"]) == 0

"""Regression tests for the ``--initial-pose`` / ``config.initial_pose`` flag.

Before this was fixed, ``initial_pose`` was plumbed all the way into the C++
``SlamConfig``/``LocalizationConfig`` but never consumed by the engines, so SLAM
always built its map at the origin and localization always started its ICP
search at the map origin. These tests pin the corrected behaviour.

They deliberately test that the configured pose is *applied* to the first
frame, not that ICP subsequently converges the first frame is checked in a
situation where ICP has nothing to register against (an empty SLAM map / an
empty localization map), so the reported pose equals the seed exactly and the
assertions don't depend on scene geometry or convergence.
"""
import numpy as np
import pytest

from ouster.sdk import open_source
from ouster.sdk.mapping import (
    SlamConfig,
    SlamEngine,
    LocalizationConfig,
    LocalizationEngine,
)


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def _pose(x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
    """Build a 4x4 SE(3) pose with a translation and a yaw rotation."""
    pose = np.eye(4)
    theta = np.deg2rad(yaw_deg)
    pose[:2, :2] = [[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]]
    pose[:3, 3] = (x, y, z)
    return pose


def _first_valid_column_pose(frame):
    col = frame.get_first_valid_column()
    if col < 0:
        return None
    return np.array(frame.body_to_world[col])


def _run_slam_poses(source_file, initial_pose):
    """Run SLAM over the whole recording and return the per-frame poses."""
    source = open_source(str(source_file))
    config = SlamConfig.create("lio")
    config.min_range = 1.0
    config.max_range = 50.0
    config.voxel_size = 0.5
    config.deskew_method = "auto"
    config.initial_pose = initial_pose
    engine = SlamEngine.create(source.sensor_info, config)

    poses = []
    for frame_set in source:
        frame_set = engine.update(frame_set)
        pose = _first_valid_column_pose(frame_set[0])
        if pose is not None:
            poses.append(pose)
    return poses


def _run_localization_first_pose(source_file, world_map, initial_pose):
    source = open_source(str(source_file))
    config = LocalizationConfig.create("lio")
    config.min_range = 1.0
    config.max_range = 50.0
    config.voxel_size = 0.5
    config.deskew_method = "auto"
    config.initial_pose = initial_pose
    engine = LocalizationEngine.create(source.sensor_info, world_map, config)

    for frame_set in source:
        frame_set = engine.update(frame_set)
        pose = _first_valid_column_pose(frame_set[0])
        if pose is not None:
            return pose
    return None


def test_slam_initial_pose_offsets_trajectory(input_osf_file):
    """The SLAM trajectory should be anchored at ``config.initial_pose``."""
    seed = _pose(x=100.0, yaw_deg=30.0)
    baseline = _run_slam_poses(input_osf_file, np.eye(4))
    shifted = _run_slam_poses(input_osf_file, seed)

    assert baseline and shifted

    # The first frame is anchored exactly at the initial pose: the SLAM map is
    # still empty, so there is nothing for ICP to correct against.
    assert np.allclose(shifted[0], seed, atol=1e-3)

    # Every subsequent frame is the baseline trajectory rigidly transformed by
    # the seed, i.e. the whole trajectory moved -- not just the first frame.
    for b, s in zip(baseline, shifted):
        assert np.allclose(s, seed @ b, atol=1.0)


def test_slam_initial_pose_defaults_to_origin(input_osf_file):
    """Without an initial pose the first frame stays at the origin."""
    baseline = _run_slam_poses(input_osf_file, np.eye(4))
    assert baseline
    assert np.allclose(baseline[0], np.eye(4), atol=1e-3)


def test_localization_initial_pose_is_applied(input_osf_file):
    """Localization should anchor the first frame at ``config.initial_pose``.

    An empty map is used so ICP has no correspondences and applies no
    correction; the reported first-frame pose therefore equals the seed
    exactly, isolating "is the seed applied" from "does ICP converge".
    """
    empty_map = np.zeros((0, 3), dtype=np.float32)
    seed = _pose(x=100.0, y=-5.0, z=2.0, yaw_deg=30.0)

    baseline = _run_localization_first_pose(input_osf_file, empty_map, np.eye(4))
    seeded = _run_localization_first_pose(input_osf_file, empty_map, seed)

    assert baseline is not None and seeded is not None
    assert np.allclose(baseline, np.eye(4), atol=1e-3)
    assert np.allclose(seeded, seed, atol=1e-3)

"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.
"""

import numpy as np

from ouster.sdk.core import CameraFrame, FrameSet, LidarFrame


def test_lidar_frame_set_bindings() -> None:
    frames_in = [LidarFrame(10, 10, [], 16), LidarFrame(10, 10, [], 16), None, LidarFrame(10, 10, [], 16)]
    collation = FrameSet(frames_in)
    frames_out = [x for x in collation]
    assert frames_in == frames_out
    assert (collation[0].w, collation[0].h) == (10, 10)  # type: ignore
    assert collation[2] is None
    assert len(collation) == 4

    collation.add_field("zz", np.uint32, (10, 10, 10))
    assert collation.has_field("zz")
    collation.del_field("zz")
    assert not collation.has_field("zz")

    collation.add_field("aa", np.uint32, (10, 10, 10))
    collation.add_field("cc", np.uint32, (10, 10, 10))
    collation.add_field("bb", np.uint32, (10, 10, 10))

    assert collation.fields == ["aa", "bb", "cc"]


def test_lidar_frame_set_with_missing_frames() -> None:
    frames_in = [LidarFrame(10, 10, [], 16), None, LidarFrame(10, 10, [], 16)]
    collation = FrameSet(frames_in)
    frames_out = [x for x in collation]
    assert frames_in == frames_out
    assert (collation[0].w, collation[0].h) == (10, 10)  # type: ignore
    assert collation[1] is None
    assert len(collation) == 3


def test_lidar_frame_set_assignment() -> None:
    frames_in = [LidarFrame(10, 10, [], 16), None]
    collation = FrameSet(frames_in)
    collation[0] = None
    assert len(collation) == 2
    assert collation[0] is None


def _mixed_frame_set() -> FrameSet:
    """A FrameSet holding a lidar frame, an empty slot, and a camera frame."""
    camera = CameraFrame()
    camera.w, camera.h = 4, 3
    camera.add_field("RANGE", np.full((3, 4), 2000, dtype=np.uint32))
    return FrameSet([LidarFrame(10, 10, [], 16), None, camera])


def test_frame_set_eq_compares_frames_by_value() -> None:
    """Equality compares the contained frames, lidar and camera alike."""
    assert _mixed_frame_set() == _mixed_frame_set()
    assert not (_mixed_frame_set() != _mixed_frame_set())

    # a difference confined to the camera frame is detected
    changed_camera = _mixed_frame_set()
    changed_camera[2].field("RANGE")[0, 0] = 1  # type: ignore
    assert changed_camera != _mixed_frame_set()

    # as is one confined to the lidar frame
    changed_lidar = _mixed_frame_set()
    changed_lidar[0].frame_id = 7  # type: ignore
    assert changed_lidar != _mixed_frame_set()


def test_frame_set_eq_distinguishes_frame_types() -> None:
    """Swapping a camera frame for a lidar frame is not equal."""
    swapped = _mixed_frame_set()
    swapped[2] = LidarFrame(10, 10, [], 16)
    assert swapped != _mixed_frame_set()


def test_frame_set_eq_missing_frames() -> None:
    """An empty slot is not equal to a populated one."""
    filled = _mixed_frame_set()
    filled[1] = LidarFrame(10, 10, [], 16)
    assert filled != _mixed_frame_set()

    # and comparison against a non-FrameSet is False rather than an error
    assert _mixed_frame_set() != 5

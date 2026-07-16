"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.
"""

import numpy as np

from ouster.sdk.core import FrameSet, LidarFrame


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

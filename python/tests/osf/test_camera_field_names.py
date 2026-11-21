"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.

``field_names`` (``ouster-cli source --fields``) on OSF sources: camera streams
must be filtered exactly like lidar streams, in both uncollated and collated
files, and a requested field a frame does not have is an error for either kind
of stream (reported through the error handler, with the frame dropped).
"""
import os
from typing import List, Optional, Tuple

import numpy as np
import pytest

from ouster.sdk import core, osf
from ouster.sdk.open_source import open_source

from tests.conftest import OSFS_DATA_DIR
from tests.osf.test_camera_collator import _make_camera_info, _make_lidar_frame

LIDAR_OSF = os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf")
NUM_FRAMES = 1


def _make_camera_frame(ts: int) -> core.CameraFrame:
    frame = core.CameraFrame()
    frame.w = 4
    frame.h = 3
    frame.add_field("RANGE", np.full((3, 4), 2000, dtype=np.uint32))
    frame.add_field("RGB", np.full((3, 4, 3), 7, dtype=np.uint8))
    frame.timestamp[0] = ts
    frame.receive_timestamp[0] = ts
    return frame


def _write_osf(path: str, *, with_lidar: bool, collated: bool) -> None:
    infos: List[core.SensorInfo] = []
    template = None
    if with_lidar:
        lidar_src = open_source(LIDAR_OSF, collate=False)
        infos.append(lidar_src.sensor_info[0])
        template = lidar_src[0][0]
    infos.append(_make_camera_info())
    with osf.Writer(path, infos) as writer:
        for n in range(NUM_FRAMES):
            ts = 1_000_000 + n * 100_000
            frames: List[core.Frame] = []
            if template is not None:
                frames.append(_make_lidar_frame(template, ts))
            frames.append(_make_camera_frame(ts))
            if collated:
                writer.save(core.FrameSet(frames))
            else:
                for index, frame in enumerate(frames):
                    writer.save(index, frame, ts)


def _read(path: str, field_names: Optional[List[str]]
          ) -> Tuple[List[core.LidarFrame], List[core.CameraFrame], List[str]]:
    """All lidar and camera frames read with ``field_names``, plus warnings."""
    warnings: List[str] = []

    def error_handler(severity, msg):
        warnings.append(msg)

    src = open_source(path, field_names=field_names, collate=False,
                      error_handler=error_handler)
    lidar_frames: List[core.LidarFrame] = []
    camera_frames: List[core.CameraFrame] = []
    for frame_set in src:
        lidar_frames.extend(frame_set.valid_lidar_frames())
        camera_frames.extend(frame_set.valid_camera_frames())
    src.close()
    return lidar_frames, camera_frames, warnings


@pytest.fixture(params=[False, True], ids=["uncollated", "collated"])
def camera_osf(request, tmp_path):
    path = str(tmp_path / "camera.osf")
    _write_osf(path, with_lidar=False, collated=request.param)
    return path


@pytest.fixture(params=[False, True], ids=["uncollated", "collated"])
def mixed_osf(request, tmp_path):
    path = str(tmp_path / "mixed.osf")
    _write_osf(path, with_lidar=True, collated=request.param)
    return path


@pytest.mark.parametrize("field_names, expected", [
    (None, ["RANGE", "RGB"]),
    (["RGB"], ["RGB"]),
    (["RANGE"], ["RANGE"]),
    (["RGB", "RANGE"], ["RANGE", "RGB"]),
    ([], []),
])
def test_camera_osf_honors_field_names(camera_osf, field_names, expected):
    _, camera_frames, warnings = _read(camera_osf, field_names)
    assert len(camera_frames) == NUM_FRAMES
    for frame in camera_frames:
        assert sorted(frame.fields) == expected
    assert warnings == []


def test_camera_osf_filtered_field_keeps_its_data(camera_osf):
    _, camera_frames, _ = _read(camera_osf, ["RGB"])
    for frame in camera_frames:
        np.testing.assert_array_equal(frame.field("RGB"),
                                      np.full((3, 4, 3), 7, dtype=np.uint8))
        assert frame.timestamp[0] != 0


def test_camera_osf_missing_requested_field_is_an_error(camera_osf):
    """Strict, like lidar: the frame is dropped and the error reported."""
    _, camera_frames, warnings = _read(camera_osf, ["RGB", "NOT_A_FIELD"])
    assert camera_frames == []
    assert len(warnings) == NUM_FRAMES
    assert all("NOT_A_FIELD" in w for w in warnings)


def test_lidar_osf_missing_requested_field_is_an_error():
    lidar_frames, _, warnings = _read(LIDAR_OSF, ["RANGE", "NOT_A_FIELD"])
    assert lidar_frames == []
    assert warnings
    assert all("NOT_A_FIELD" in w for w in warnings)


def test_mixed_osf_filters_camera_and_lidar_streams(mixed_osf):
    lidar_frames, camera_frames, warnings = _read(mixed_osf, ["RANGE"])
    assert len(lidar_frames) == NUM_FRAMES
    assert len(camera_frames) == NUM_FRAMES
    for frame in lidar_frames:
        assert list(frame.fields) == ["RANGE"]
    for frame in camera_frames:
        assert list(frame.fields) == ["RANGE"]
    assert warnings == []

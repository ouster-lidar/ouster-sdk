"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.
"""

import copy
import os
import tempfile

import numpy as np
import pytest

from ouster.sdk import core, osf
from ouster.sdk.open_source import open_source

from tests.conftest import OSFS_DATA_DIR


def test_camera_only_osf_collator_round_trip():
    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(100.0, 100.0, 0.0, 0.0)
    sensor_to_body = np.array(
        [
            [1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.2],
            [0.0, 0.0, 1.0, 0.3],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    cam_info.sensor_to_body = sensor_to_body

    frame = core.CameraFrame()
    frame.w = 4
    frame.h = 3
    frame.add_field("RANGE", np.full((3, 4), 2000, dtype=np.uint32))
    left = np.zeros((3, 4, 3), dtype=np.uint8)
    left[:, :, 2] = 255
    frame.add_field("LEFT_BGR", left)

    with tempfile.NamedTemporaryFile(delete=False, suffix=".osf") as f:
        path = f.name

    try:
        with osf.Writer(path) as writer:
            writer.add_sensor(cam_info)
            writer.save(0, frame, 100)
            writer.save(0, frame, 200)

        uncollated = osf.OsfFrameSetSource(path)
        u_lidar = [i for i in uncollated.sensor_info if isinstance(i, core.LidarInfo)]
        u_cameras = [i for i in uncollated.sensor_info if isinstance(i, core.CameraInfo)]
        assert len(u_lidar) == 0
        assert len(u_cameras) == 1
        np.testing.assert_allclose(u_cameras[0].sensor_to_body, sensor_to_body)

        uncollated_frames = list(uncollated)
        assert len(uncollated_frames) == 2
        first_cam = uncollated_frames[0].valid_camera_frames()[0]
        assert first_cam is not None
        assert first_cam.sensor_info is not None
        np.testing.assert_allclose(first_cam.sensor_info.sensor_to_body, sensor_to_body)
        assert first_cam.timestamp[0] == 100
        np.testing.assert_array_equal(first_cam.body_to_world[0], np.eye(4))
        assert uncollated_frames[1].valid_camera_frames()[0].timestamp[0] == 200

        collated = core.collate(osf.OsfFrameSetSource(path))
        assert len([i for i in collated.sensor_info if isinstance(i, core.CameraInfo)]) == 1
        frames = list(collated)
        assert len(frames) == 2
        assert len(frames[0].valid_camera_frames()) == 1
        assert frames[0].valid_camera_frames()[0] is not None
        assert frames[0].valid_camera_frames()[0].sensor_info is not None
        np.testing.assert_allclose(
            frames[0].valid_camera_frames()[0].sensor_info.sensor_to_body, sensor_to_body
        )
        assert frames[0].valid_camera_frames()[0].has_field("RANGE")
        assert frames[0].valid_camera_frames()[0].has_field("LEFT_BGR")

        opened = open_source(path, collate=True)
        assert len([i for i in opened.sensor_info if isinstance(i, core.CameraInfo)]) == 1
        opened_frames = list(opened)
        assert len(opened_frames) == 2
        assert opened_frames[0].valid_camera_frames()[0] is not None
        assert opened_frames[0].valid_camera_frames()[0].sensor_info is not None
        opened.close()
    finally:
        import os
        try:
            os.unlink(path)
        except OSError:
            pass


def _make_camera_info(fx=100.0):
    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(fx, fx, 0.0, 0.0)
    return cam_info


def _make_camera_frame(ts, range_value=2000):
    frame = core.CameraFrame()
    frame.w = 4
    frame.h = 3
    frame.add_field("RANGE", np.full((3, 4), range_value, dtype=np.uint32))
    frame.timestamp[0] = ts
    frame.receive_timestamp[0] = ts
    return frame


def _make_lidar_frame(template, ts):
    """Copy a real LidarFrame and stamp it with valid status/timestamps."""
    frame = copy.copy(template)
    frame.status[:] = 1
    frame.timestamp[:] = 1
    num_pkts = frame.receive_timestamp.size
    for i in range(num_pkts):
        frame.receive_timestamp[i] = ts + i
    return frame


@pytest.mark.parametrize("camera_first", [False, True])
def test_lidar_camera_collation_registration_order_round_trip(camera_first):
    """save(FrameSet) must honor the sensor registration order, whether the
    camera is registered before or after the lidar (not just 'lidar-first')."""
    lidar_src = open_source(
        os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf"), collate=False)
    lidar_info = lidar_src.sensor_info[0]
    template = lidar_src[0][0]
    assert isinstance(template, core.LidarFrame)

    cam_info = _make_camera_info()

    # Register in the requested order; the saved FrameSet must match it.
    if camera_first:
        infos = [cam_info, lidar_info]
    else:
        infos = [lidar_info, cam_info]

    with tempfile.NamedTemporaryFile(delete=False, suffix=".osf") as f:
        path = f.name

    try:
        with osf.Writer(path, infos) as writer:
            for n in range(3):
                ts = 1_000_000 + n * 100_000
                lidar_frame = _make_lidar_frame(template, ts)
                camera_frame = _make_camera_frame(ts, range_value=1000 * (n + 1))
                if camera_first:
                    writer.save(core.FrameSet([camera_frame, lidar_frame]))
                else:
                    writer.save(core.FrameSet([lidar_frame, camera_frame]))

        src = osf.OsfFrameSetSource(path)
        assert len([i for i in src.sensor_info if isinstance(i, core.LidarInfo)]) == 1
        assert len([i for i in src.sensor_info if isinstance(i, core.CameraInfo)]) == 1

        # sensor_info() must be exposed in the same (registration) order as the
        # frames within each collated FrameSet.
        lidar_pos = 1 if camera_first else 0
        camera_pos = 0 if camera_first else 1
        assert isinstance(src.sensor_info[lidar_pos], core.LidarInfo)
        assert isinstance(src.sensor_info[camera_pos], core.CameraInfo)

        # The OSF already stores collations (written via save(FrameSet)); the
        # source reconstructs them directly, preserving the saved frame order.
        frame_sets = list(src)
        assert len(frame_sets) == 3
        for n, fs in enumerate(frame_sets):
            frames = list(fs)
            assert len(frames) == 2
            # exact positional order is preserved across the round-trip
            assert isinstance(frames[lidar_pos], core.LidarFrame)
            assert isinstance(frames[camera_pos], core.CameraFrame)
            cam = frames[camera_pos]
            np.testing.assert_array_equal(
                cam.field("RANGE"), np.full((3, 4), 1000 * (n + 1), dtype=np.uint32))
            assert cam.sensor_info is not None
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def test_camera_fields_to_write_filters_saved_fields():
    """add_sensor(camera_info, fields_to_write=...) must restrict which
    camera-frame fields are written, mirroring add_sensor() for lidars."""
    cam_info = _make_camera_info()

    def make_multi_field_frame(ts):
        frame = core.CameraFrame()
        frame.w = 4
        frame.h = 3
        frame.add_field("RANGE", np.full((3, 4), 2000, dtype=np.uint32))
        left = np.zeros((3, 4, 3), dtype=np.uint8)
        left[:, :, 2] = 255
        frame.add_field("LEFT_BGR", left)
        frame.timestamp[0] = ts
        frame.receive_timestamp[0] = ts
        return frame

    with tempfile.NamedTemporaryFile(delete=False, suffix=".osf") as f:
        path = f.name

    try:
        # Only RANGE should be persisted; LEFT_BGR must be dropped.
        with osf.Writer(path, [cam_info], ["RANGE"]) as writer:
            writer.save(0, make_multi_field_frame(100), 100)
            writer.save(0, make_multi_field_frame(200), 200)

        src = osf.OsfFrameSetSource(path)
        for frame_set in src:
            cam = frame_set.valid_camera_frames()[0]
            assert cam.has_field("RANGE")
            assert not cam.has_field("LEFT_BGR")
        src.close()
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def test_camera_only_single_sensor_selection():
    """--sensor-idx / .single() must be able to select a camera. Previously the
    OSF source only knew about lidar streams for single-sensor selection, so
    picking a camera (here the only sensor) threw an 'Invalid index'."""
    cam_info = _make_camera_info()

    with tempfile.NamedTemporaryFile(delete=False, suffix=".osf") as f:
        path = f.name

    try:
        with osf.Writer(path) as writer:
            writer.add_sensor(cam_info)
            writer.save(0, _make_camera_frame(100), 100)
            writer.save(0, _make_camera_frame(200), 200)

        selected = open_source(path, collate=False, sensor_idx=0)
        assert len(selected.sensor_info) == 1
        assert isinstance(selected.sensor_info[0], core.CameraInfo)
        frames = list(selected)
        assert len(frames) == 2
        assert frames[0].valid_camera_frames()[0] is not None
        assert frames[1].valid_camera_frames()[0] is not None
        selected.close()
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def test_mixed_lidar_camera_single_sensor_selection():
    """In a mixed lidar+camera file, --sensor-idx can select either kind by its
    unified sensor index (the order exposed by sensor_info())."""
    lidar_src = open_source(
        os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf"), collate=False)
    lidar_info = lidar_src.sensor_info[0]
    template = lidar_src[0][0]
    assert isinstance(template, core.LidarFrame)

    cam_info = _make_camera_info()

    with tempfile.NamedTemporaryFile(delete=False, suffix=".osf") as f:
        path = f.name

    try:
        # Registration order: [lidar (unified index 0), camera (unified index
        # 1)]. Writer.save() takes the unified stream index, so the lidar uses
        # index 0 and the camera uses index 1.
        with osf.Writer(path, [lidar_info, cam_info]) as writer:
            for n in range(3):
                ts = 1_000_000 + n * 100_000
                writer.save(0, _make_lidar_frame(template, ts), ts)
                writer.save(1, _make_camera_frame(ts, range_value=1000 * (n + 1)), ts)

        # Select the lidar (index 0): only lidar frames come out.
        lidar_only = open_source(path, collate=False, sensor_idx=0)
        assert len(lidar_only.sensor_info) == 1
        assert isinstance(lidar_only.sensor_info[0], core.LidarInfo)
        for fs in lidar_only:
            assert fs[0] is not None
            assert isinstance(fs[0], core.LidarFrame)
        lidar_only.close()

        # Select the camera (index 1): only camera frames come out.
        camera_only = open_source(path, collate=False, sensor_idx=1)
        assert len(camera_only.sensor_info) == 1
        assert isinstance(camera_only.sensor_info[0], core.CameraInfo)
        cam_frames = list(camera_only)
        assert len(cam_frames) == 3
        for fs in cam_frames:
            assert fs.valid_camera_frames()[0] is not None
        camera_only.close()
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def test_camera_osf_timestamp_and_pose_round_trip():
    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(100.0, 100.0, 0.0, 0.0)

    frame = core.CameraFrame()
    frame.w = 4
    frame.h = 3
    frame.add_field("RANGE", np.full((3, 4), 2000, dtype=np.uint32))
    frame.timestamp[0] = 12345
    pose = np.eye(4)
    pose[0, 3] = 1.5
    frame.body_to_world[0] = pose

    with tempfile.NamedTemporaryFile(delete=False, suffix=".osf") as f:
        path = f.name

    try:
        with osf.Writer(path) as writer:
            writer.add_sensor(cam_info)
            writer.save(0, frame, 100)

        restored = list(osf.OsfFrameSetSource(path))[0].valid_camera_frames()[0]
        assert restored.timestamp[0] == 12345
        np.testing.assert_allclose(restored.body_to_world[0], pose)
    finally:
        import os
        try:
            os.unlink(path)
        except OSError:
            pass

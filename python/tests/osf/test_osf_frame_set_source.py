# TODO: de-duplicate OSF writers in tests
import copy
import pytest
import tempfile
import ouster.sdk.osf as osf
import ouster.sdk.core as core
from ouster.sdk import open_source
from ouster.sdk.core import (ZoneSet, Zone, Stl, ZoneMode, CoordinateFrame,
                             LidarMode, LidarInfo, FrameSetSourceMetadataSet,
                             LidarFrame)
import os
import sys
import numpy as np
from typing import List, cast


@pytest.fixture
def input_info(test_data_dir):
    filename = test_data_dir / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.json"
    with open(filename, 'r') as f:
        data = f.read()
    return core.LidarInfo(data)


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def test_header_verification_failure():
    """It should raise a RuntimeError if the provided file is not an OSF file."""
    with pytest.raises(RuntimeError, match="EOF reached"):
        with tempfile.NamedTemporaryFile(delete=False) as f:
            f.write(b"abc" * 1000)  # write some bad data to it
            f.close()   # Windows seems to have trouble with the file being open already
            osf.OsfFrameSetSource(f.name)
        try:
            os.unlink(f.name)
        except (OSError, FileNotFoundError):
            pass


def test_frames_num(input_info):
    """New OSF files should always be indexed and have frames_num and len"""
    src = None
    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, [input_info, input_info]) as w:
                frame1 = core.LidarFrame(input_info)
                frame1.status[:] = 0x1
                frame1.field(core.ChanField.REFLECTIVITY)[:] = 100
                w.save(0, frame1)
                w.save(1, frame1)
                frame1.receive_timestamp[:] = 1
                w.save(0, frame1)
                w.save(1, frame1)
        src = osf.OsfFrameSetSource(f.name)
        assert src.frames_num == [2, 2]
        assert len(src) == 4
    finally:
        if src:
            src.close()
        os.unlink(f.name)


def test_osf_open(input_osf_file):
    """Make sure the file opens and has correct metadata in the frames"""
    source = osf.OsfFrameSetSource(str(input_osf_file))
    got_frame = False
    for frame_set in source:
        got_frame = True
        assert frame_set[0].sensor_info == source.sensor_info[0]
    assert got_frame


@pytest.mark.skipif(sys.platform.startswith("win"), reason="Broken on Windows")
def test_missing_streams(input_osf_file):
    """Make sure the OsfFrameSetSource can handle empty streams"""
    data = osf.OsfFrameSetSource(str(input_osf_file))
    frame = next(iter(data))[0]
    data.close()
    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            w = osf.Writer(f.name, [frame.sensor_info, frame.sensor_info, frame.sensor_info])
            frame.frame_id = 0
            w.save(0, frame)
            frame.frame_id = 2
            w.save(2, frame)
            w.close()

        result = core.collate(osf.OsfFrameSetSource(f.name))
        frame = next(iter(result))[0]

        data = None
        for res in result:
            data = res
        assert data[1] is None
        assert data[0] is not None
        assert data[2] is not None
        assert result.frames_num == [1, 0, 1]
    finally:
        if result:
            result.close()
        os.unlink(f.name)


def test_osf_info_modification_bug(input_osf_file):
    """Make sure that modifying the pixel shift by row in the returned metadata doesn't break parsing"""
    source = osf.OsfFrameSetSource(str(input_osf_file))

    def get_first_frame():
        for frame_set in source:
            return frame_set[0]

    frame_before = get_first_frame()
    field_before = frame_before.field(core.ChanField.REFLECTIVITY).astype(np.uint8)

    source.sensor_info[0].format.pixel_shift_by_row = np.zeros((128), np.int32)

    frame_after = get_first_frame()
    field_after = frame_after.field(core.ChanField.REFLECTIVITY).astype(np.uint8)

    assert np.array_equal(field_before, field_after)


def test_osf_frame_set_source_zm_config_thaw(input_info, test_data_dir):
    infos = [copy.deepcopy(input_info), copy.deepcopy(input_info)]
    infos[0].sn = 0
    infos[0].zone_set = None
    infos[1].sn = 1
    zone_set = ZoneSet()
    zone_set.sensor_to_body_transform = np.eye(4)
    zone = Zone()
    zone.point_count = 1
    zone.frame_count = 1
    zone.mode = ZoneMode.OCCUPANCY
    zone.stl = Stl(f'{test_data_dir}/zone_monitor/0.stl')  # dummy STL data
    zone.stl.coordinate_frame = CoordinateFrame.BODY
    zone_set.zones[0] = zone
    infos[1].zone_set = zone_set

    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, infos) as w:
                pass
            w.close()
        src = osf.OsfFrameSetSource(f.name)
        assert src.sensor_info[0].sn == 0
        assert src.sensor_info[0].zone_set is None
        assert src.sensor_info[1].sn == 1
        assert src.sensor_info[1].zone_set == \
            infos[1].zone_set
    finally:
        try:
            os.unlink(f.name)
        except (PermissionError, FileNotFoundError):
            pass


def test_osf_frame_set_source_out_of_bounds_access(input_osf_file):
    source = osf.OsfFrameSetSource(str(input_osf_file))
    assert len(source) == 3
    with pytest.raises(IndexError):
        frames = source[3]

    info = source.sensor_info[0]
    new_infos = [copy.deepcopy(info), copy.deepcopy(info), copy.deepcopy(info)]
    frames = [fs[0] for fs in source]

    frames[0].receive_timestamp[:] = 100
    frames[1].receive_timestamp[:] = 200
    collation_1 = core.FrameSet([frames[0], frames[0], frames[0]])
    collation_2 = core.FrameSet([frames[1], frames[1], frames[1]])

    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, new_infos) as w:
                w.save(collation_1)
                w.save(collation_2)
            w.close()
        collated_src = osf.OsfFrameSetSource(f.name)
        with pytest.raises(IndexError):
            collated_src[2]
    finally:
        try:
            os.unlink(f.name)
        except (PermissionError, FileNotFoundError):
            pass


def test_osf_frame_set_source_collating(input_osf_file):
    input_src = osf.OsfFrameSetSource(str(input_osf_file))
    frames = [fs[0] for fs in input_src]
    assert len(frames) == 3
    info = input_src.sensor_info[0]
    input_src.close()

    frames[0].receive_timestamp[:] = 100
    frames[1].receive_timestamp[:] = 200
    frames[2].receive_timestamp[:] = 300

    new_infos = [copy.deepcopy(info), copy.deepcopy(info), copy.deepcopy(info)]
    new_infos[0].sn = 0
    new_infos[1].sn = 1
    new_infos[2].sn = 2

    collation_1 = core.FrameSet([frames[0], frames[0], frames[0]])
    field_1 = collation_1.add_field("coll1_field", np.float32, (100, 100))
    field_1[:] = 3.1415

    collation_2 = core.FrameSet([frames[2], frames[2], frames[2]])
    field_2 = collation_2.add_field("coll2_field", np.uint8, (100,))
    field_2[:] = 8

    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, new_infos) as w:
                w.save(collation_1)
                w.save(0, frames[1])
                w.save(1, frames[1])
                w.save(2, frames[1])
                w.save(collation_2)
            w.close()
        src = osf.OsfFrameSetSource(f.name)
        assert src.is_collated is True
        assert len(src) == 2
        it = iter(src)
        out_coll_1 = next(it)
        assert out_coll_1 == collation_1
        out_coll_2 = next(it)
        assert out_coll_2 == collation_2
    finally:
        try:
            os.unlink(f.name)
        except (PermissionError, FileNotFoundError):
            pass


def test_object_lists_in_osf(input_osf_file):
    input_src = osf.OsfFrameSetSource(str(input_osf_file))
    frames = [fs[0] for fs in input_src]
    assert len(frames) == 3
    info = input_src.sensor_info[0]
    input_src.close()

    frame = frames[0]
    frame.receive_timestamp[:] = 100

    import ouster.sdk.core as core
    objects = [core.Object(), core.Object()]
    objects[0].id = 1
    objects[0].creation_ts = 99
    objects[0].timestamp = 199
    objects[0].class_id = 1
    objects[0].class_confidence = 0.9
    objects[0].object_to_body.position = np.array([1, 2, 3])
    objects[0].object_to_body.set_rotation(np.array([2, 2, 2]))
    objects[0].body_to_world.position = np.array([10, 20, 30])
    objects[0].body_to_world.set_rotation(np.array([0.1, 0.2, 0.3]))
    objects[0].velocity = np.array([2, 3, 4])
    objects[0].dimensions = np.array([1, 1, 1])
    objects[0].properties["num_points"] = '[100]'
    objects[0].properties["attributes"] = '["eats_icecream", "carries_bag"]'
    objects[1].id = 2
    objects[1].creation_ts = 100
    objects[1].timestamp = 200
    objects[1].class_id = 2
    objects[1].class_confidence = 0.8
    objects[1].object_to_body.position = np.array([3, 2, 1])
    objects[1].object_to_body.set_rotation(np.array([1, 1, 1]))
    objects[1].body_to_world.position = np.array([4, 5, 6])
    objects[1].body_to_world.set_rotation(np.array([0.4, 0.5, 0.6]))
    objects[1].velocity = np.array([4, 3, 2])
    objects[1].dimensions = np.array([2, 2, 2])
    objects[1].properties["num_points"] = '[50]'
    objects[1].properties["attributes"] = '["parked_illegaly"]'
    frame.objects["test_objects"] = objects

    collation = core.FrameSet([frame])
    collation.objects["my_object"] = [objects[0]]
    assert "my_object" in collation.objects

    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, [info]) as w:
                w.save(collation)
            w.close()
        src = osf.OsfFrameSetSource(f.name)
        it = iter(src)
        out_collation = next(it)
        assert "my_object" in out_collation.objects
        assert len(out_collation.objects["my_object"]) == 1
        assert "test_objects" in out_collation[0].objects
        assert len(out_collation[0].objects["test_objects"]) == 2

        assert out_collation.objects["my_object"] == collation.objects["my_object"]
        assert out_collation[0].objects["test_objects"] == collation[0].objects["test_objects"]
    finally:
        try:
            os.unlink(f.name)
        except (PermissionError, FileNotFoundError):
            pass


def test_camera_frame_objects_in_collation_osf(input_info):
    """CameraFrame objects attached to a collated FrameSet survive OSF round-trip."""
    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(500.0, 501.0, 320.0, 240.0)

    lidar_frame = core.LidarFrame(input_info)
    lidar_frame.status[:] = 0x1
    lidar_frame.receive_timestamp[:] = 100

    camera_frame = core.CameraFrame()
    # Camera frames carry a valid timestamp just like lidar frames; a
    # zero-timestamp frame is dropped from a collation regardless of origin.
    camera_frame.timestamp[0] = 100
    camera_frame.receive_timestamp[0] = 100
    obj = core.Object()
    obj.id = 42
    obj.creation_ts = 100
    obj.timestamp = 200
    obj.class_id = 3
    obj.class_confidence = 0.75
    obj.object_to_body.position = np.array([1.0, 2.0, 3.0])
    obj.object_to_body.set_rotation(np.array([0.1, 0.2, 0.3]))
    obj.body_to_world.position = np.array([10.0, 20.0, 30.0])
    obj.body_to_world.set_rotation(np.array([0.4, 0.5, 0.6]))
    obj.velocity = np.array([1.0, 0.0, -1.0])
    obj.dimensions = np.array([2.0, 1.5, 0.5])
    obj.properties["source"] = "camera_test"
    camera_frame.objects["detections"] = [obj]

    collation = core.FrameSet([lidar_frame, camera_frame])

    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, [input_info]) as w:
                w.add_sensor(cam_info)
                w.save(collation)
            w.close()
        src = osf.OsfFrameSetSource(f.name)
        assert src.is_collated is True
        recovered = next(iter(src))
        assert len(recovered.valid_camera_frames()) == 1
        assert recovered.valid_camera_frames()[0] is not None
        assert "detections" in recovered.valid_camera_frames()[0].objects
        assert len(recovered.valid_camera_frames()[0].objects["detections"]) == 1
        assert recovered.valid_camera_frames()[0].objects["detections"] == camera_frame.objects["detections"]
    finally:
        try:
            os.unlink(f.name)
        except (PermissionError, FileNotFoundError):
            pass


def test_frame_set_source_metadata_is_recoverable(tmp_path):
    def truncate_file(path, num_bytes):
        with open(path, 'r+b') as f:
            f.seek(-num_bytes, os.SEEK_END)
            f.truncate()

    osf_path = tmp_path / "test.osf"
    sensor_info = LidarInfo.from_default(LidarMode._1024x10)
    writer = osf.Writer(str(osf_path), [sensor_info])
    metadata = FrameSetSourceMetadataSet()
    metadata['custom_key'] = "custom_value"
    writer.save(metadata)
    writer.close()

    # truncate some bytes to mess up the OSF footer, which should trigger recovery
    truncate_file(str(osf_path), 10)

    src = open_source(str(osf_path))
    assert src.metadata_keys() == {'custom_key'}
    assert src.metadata('custom_key') == "custom_value"


def write_osf_process(input_file, output_file, num_messages):
    src1 = open_source(str(input_file))
    writer = osf.Writer(str(output_file), src1.sensor_info, chunk_size=1)
    frame = src1[0][0]
    for i in range(num_messages):
        writer.save(0, frame)
    os._exit(0)


def test_frame_set_source_unfinalized_recoverable(input_osf_file, tmp_path):
    """Validate that an unfinalized OSF is readable"""
    import multiprocessing

    output_osf_file = tmp_path / "test.osf"
    num_messages = 5
    ctx = multiprocessing.get_context('spawn')
    p = ctx.Process(target=write_osf_process,
                    args=(input_osf_file, output_osf_file, num_messages))
    p.start()
    p.join()

    src = open_source(str(input_osf_file))
    src_copy = open_source(str(output_osf_file))

    # now try and load the file
    # note we lose the last message due to how chunks are written
    expected = num_messages - 1
    assert len(src_copy) == expected
    count = 0
    for frame, in src_copy:
        assert frame == src[0][0]
        count += 1
    assert count == expected


def test_seeking_same_timestamp(tmp_path, input_osf_file) -> None:
    """test that we can seek to the nth scan even if timestamps are all the same"""
    osf_path = tmp_path / "test.osf"
    src1 = open_source(str(input_osf_file))
    writer = osf.Writer(str(osf_path), src1.sensor_info)
    base_frame = src1[0][0]
    assert base_frame is not None
    original_frames: List[LidarFrame] = []
    for i in range(3):
        frame = copy.copy(base_frame)
        assert isinstance(frame, LidarFrame)
        frame.frame_id = i
        writer.save(0, frame)
        original_frames.append(frame)
    writer.close()

    # check we can read all the frames out as expected
    src2 = open_source(str(osf_path))

    for i in range(3):
        new_frame = src2[i][0]
        assert new_frame is not None
        assert isinstance(new_frame, LidarFrame)
        assert new_frame.frame_id == i
        assert new_frame == original_frames[i]


@pytest.mark.parametrize("cutoff", [1, 10, 1000])
def test_frame_set_source_osf_is_recoverable(tmp_path, input_osf_file, cutoff: int):
    def truncate_file(path, num_bytes):
        with open(path, 'r+b') as f:
            f.seek(-num_bytes, os.SEEK_END)
            f.truncate()

    osf_path = tmp_path / "test.osf"
    src1 = open_source(str(input_osf_file))
    writer = osf.Writer(str(osf_path), src1.sensor_info)
    original_frames: List[LidarFrame] = []
    for frame, in src1:
        assert frame is not None
        assert isinstance(frame, LidarFrame)
        writer.save(0, frame)
        original_frames.append(frame)
    writer.close()

    # truncate some bytes to mess up the OSF footer, which should trigger recovery
    truncate_file(str(osf_path), cutoff)

    # check we can read all the frames out
    src2 = open_source(str(osf_path))

    # except for the client_version, the sensor_info should be identical
    # (client_version is a lidar LidarInfo attribute; these are lidar sensors)
    cast(LidarInfo, src2.sensor_info[0]).client_version = \
        cast(LidarInfo, src1.sensor_info[0]).client_version
    assert src2.sensor_info[0] == src1.sensor_info[0]

    # there should be the same number of frames
    assert len(src2) == len(src1)
    looped = 0
    for idx, frame_sets in enumerate(src2):
        looped += 1
        # and the frames should be identical to the original ones
        assert frame_sets[0] == original_frames[idx]
    assert looped == len(src1)

    # also test dump on these files
    osf.dump_metadata(str(osf_path))


def _recovery_camera_info():
    cam_info = core.CameraInfo()
    cam_info.model = core.CameraModel(500.0, 501.0, 320.0, 240.0)
    cam_info.format.columns_per_frame, cam_info.format.pixels_per_column, cam_info.sn = 16, 8, 987654
    cam_info.format.fps = 30
    return cam_info


def _recovery_camera_frame(cam_info, i):
    frame = core.CameraFrame(cam_info)
    w, h = cam_info.format.columns_per_frame, cam_info.format.pixels_per_column
    frame.w, frame.h = w, h
    frame.add_field('RANGE', np.full((h, w), 2000 + i, dtype=np.uint32))
    frame.timestamp[0] = 100000000 * (i + 1)
    frame.receive_timestamp[0] = frame.timestamp[0]
    frame.status[0] = 1
    return frame


def _write_camera_osf(path, frame_count=3):
    """Write a camera-only OSF and return the CameraInfo it was written with."""
    cam_info = _recovery_camera_info()
    writer = osf.Writer(str(path), [cam_info])
    for i in range(frame_count):
        writer.save(core.FrameSet([_recovery_camera_frame(cam_info, i)]))
    writer.close()
    return cam_info


def write_unfinalized_camera_osf_process(output_file, num_messages):
    """Write a camera OSF and die without finalizing, like a crash mid-recording.

    The writer is kept in scope until os._exit so it is never destroyed; letting
    it go out of scope would run its destructor, which finalizes the file and
    writes the very footer this is meant to do without.
    """
    cam_info = _recovery_camera_info()
    writer = osf.Writer(str(output_file), [cam_info], chunk_size=1)
    for i in range(num_messages):
        writer.save(core.FrameSet([_recovery_camera_frame(cam_info, i)]))
    assert not writer.is_closed()
    os._exit(0)


def test_camera_metadata_is_recoverable(tmp_path):
    """A camera OSF whose footer never made it to disk still recovers its
    CameraInfo, the same way a lidar OSF recovers its LidarInfo."""
    osf_path = tmp_path / "camera.osf"
    cam_info = _write_camera_osf(osf_path, frame_count=3)

    # Drop the footer, which is what a crash mid-recording leaves behind.
    with open(str(osf_path), 'r+b') as f:
        f.seek(-10, os.SEEK_END)
        f.truncate()

    src = open_source(str(osf_path))

    # The CameraSensor entry is recovered into the metadata store.
    assert len(src.sensor_info) == 1
    recovered_info = src.sensor_info[0]
    assert isinstance(recovered_info, core.CameraInfo)
    assert recovered_info.to_json_string() == cam_info.to_json_string()

    # And the frames read back carry that CameraInfo.
    frames = [f for f, in src]
    assert len(frames) > 0
    for frame in frames:
        assert isinstance(frame, core.CameraFrame)
        assert frame.sensor_info is not None
        assert frame.sensor_info.to_json_string() == cam_info.to_json_string()


def test_camera_metadata_is_recoverable_unfinalized(tmp_path):
    """Same recovery, but for a file that was never finalized at all, so there
    is no footer to fall back on rather than a damaged one."""
    import multiprocessing

    osf_path = tmp_path / "camera_unfinalized.osf"
    num_messages = 5
    ctx = multiprocessing.get_context('spawn')
    p = ctx.Process(target=write_unfinalized_camera_osf_process,
                    args=(osf_path, num_messages))
    p.start()
    p.join()

    expected_info = _recovery_camera_info()
    src = open_source(str(osf_path))

    assert len(src.sensor_info) == 1
    assert isinstance(src.sensor_info[0], core.CameraInfo)
    assert src.sensor_info[0].to_json_string() == expected_info.to_json_string()

    # As in the lidar case, the last message is lost to the chunk that was
    # still open when the process died.
    frames = [f for f, in src]
    assert len(frames) == num_messages - 1
    for i, frame in enumerate(frames):
        assert isinstance(frame, core.CameraFrame)
        assert frame.sensor_info.to_json_string() == expected_info.to_json_string()
        np.testing.assert_array_equal(frame.field('RANGE'),
                                      np.full((8, 16), 2000 + i, dtype=np.uint32))


def test_mixed_lidar_camera_metadata_is_recoverable(tmp_path, input_info):
    """A mixed file recovers both sensors, each as the right metadata kind and
    wired to the right stream."""
    osf_path = tmp_path / "mixed.osf"
    cam_info = _recovery_camera_info()

    writer = osf.Writer(str(osf_path), [input_info, cam_info])
    for i in range(3):
        lidar_frame = core.LidarFrame(input_info)
        lidar_frame.status[:] = 0x1
        lidar_frame.timestamp[:] = 100000000 * (i + 1)
        lidar_frame.receive_timestamp[:] = lidar_frame.timestamp[0]
        writer.save(core.FrameSet([lidar_frame, _recovery_camera_frame(cam_info, i)]))
    writer.close()

    with open(str(osf_path), 'r+b') as f:
        f.seek(-10, os.SEEK_END)
        f.truncate()

    src = open_source(str(osf_path))

    # Both sensors come back, in registration order, as their concrete types.
    assert len(src.sensor_info) == 2
    assert isinstance(src.sensor_info[0], LidarInfo)
    assert isinstance(src.sensor_info[1], core.CameraInfo)
    assert src.sensor_info[1].to_json_string() == cam_info.to_json_string()

    frame_sets = list(src)
    assert len(frame_sets) > 0
    for frame_set in frame_sets:
        lidar_frames = [f for f in frame_set if isinstance(f, LidarFrame)]
        camera_frames = [f for f in frame_set if isinstance(f, core.CameraFrame)]
        assert len(lidar_frames) == 1
        assert len(camera_frames) == 1
        # Each frame is associated with its own sensor's metadata, not the other's.
        assert isinstance(lidar_frames[0].sensor_info, LidarInfo)
        assert camera_frames[0].sensor_info.to_json_string() == cam_info.to_json_string()

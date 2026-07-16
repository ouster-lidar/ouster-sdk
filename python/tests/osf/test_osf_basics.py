import json
import tempfile
from re import escape
import pytest
import numpy as np
import hashlib
import shutil
import os

from more_itertools import ilen
from ouster.sdk import open_source, SourceURLException

import ouster.sdk._bindings.osf as osf
import ouster.sdk.core as core
from ouster.sdk.core import ChanField, FieldType, LidarMode, LidarFrame, SensorInfo, Severity, init_logger, FrameSet
from ouster.sdk._bindings.osf import LidarFrameStream, Encoder, PngLidarFrameEncoder
from ouster.sdk.osf import OsfDropFrameError


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


@pytest.fixture
def input_info(test_data_dir):
    filename = test_data_dir / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.json"
    with open(filename, 'r') as f:
        data = f.read()
    return core.SensorInfo(data)


# Test that we can save a subset of frame fields and that it errors
# if you try and save a frame missing fields in the metadata
def test_writer_quick(tmp_path, input_info):
    file_name = tmp_path / "test.osf"
    save_fields = [
        core.ChanField.REFLECTIVITY,
        core.ChanField.RANGE
    ]

    error_fields = [core.FieldType(core.ChanField.RANGE, np.uint32)]
    with osf.Writer(str(file_name), input_info, save_fields) as writer:
        frame = core.LidarFrame(input_info)
        frame.field(core.ChanField.REFLECTIVITY)[:] = 123
        frame.field(core.ChanField.RANGE)[:] = 5

        writer.save(0, frame)

        # also try saving an frame with missing fields
        frame2 = core.LidarFrame(input_info, error_fields)
        frame2.field(core.ChanField.RANGE)[:] = 6

        with pytest.raises(ValueError):
            writer.save(0, frame2)

        writer.close()

    # then open it and double check that we only got the fields we needed
    res_reader = osf.Reader(str(file_name))

    messages = [it for it in res_reader.messages()]
    for msg in messages:
        if msg.of(LidarFrameStream):
            ls = msg.decode()
            if ls:
                # validate that it only has the channels we added
                fields = [field for field in ls.fields]
                assert core.ChanField.RANGE in fields
                assert core.ChanField.REFLECTIVITY in fields
                assert len(fields) == 2

    assert len(messages) == 2  # 1 plus the sensor info message


# Test that we can load a subset of fields and that it errors if you try and load missing fields
def test_writer_partial_load(tmp_path, input_info):
    file_name = tmp_path / "test.osf"

    with osf.Writer(str(file_name), input_info) as writer:
        frame = core.LidarFrame(input_info)
        frame.field(core.ChanField.REFLECTIVITY)[:] = 123
        frame.field(core.ChanField.RANGE)[:] = 5
        frame.add_field("test", np.ones((128, 1024, 1)), core.FieldClass.FRAME_FIELD)
        frame.add_field("test2", np.ones((128, 1024, 1)), core.FieldClass.FRAME_FIELD)

        writer.save(0, frame)

        writer.close()

    # then open it and double check that we only got the fields we needed
    def old_error_behavior(severity, msg):
        raise RuntimeError(msg)
    res_reader = osf.Reader(str(file_name), old_error_behavior)

    messages = [it for it in res_reader.messages()]
    for msg in messages:
        if msg.of(LidarFrameStream):
            with pytest.raises(RuntimeError):
                msg.decode(["apples"])
            ls = msg.decode(["RANGE", "test"])
            if ls:
                # validate that it only has the channels we added
                fields = [field for field in ls.fields]
                assert core.ChanField.RANGE in fields
                assert core.ChanField.REFLECTIVITY not in fields
                assert "test" in fields
                assert "test2" not in fields
                assert len(fields) == 2

    assert len(messages) == 2  # 1 plus the sensor info message


def writer_output_handler(writer, output_osf_file, info):
    assert writer.filename() == str(output_osf_file)
    assert len(writer.sensor_info()) == 1
    assert info == writer.sensor_info()[0]

    frame1 = core.LidarFrame(info)
    assert frame1 is not None
    frame1.status[:] = 0x1
    frame1.packet_timestamp[:] = 1
    frame1.field(core.ChanField.REFLECTIVITY)[:] = 100

    frame2 = core.LidarFrame(info)
    assert frame2 is not None
    frame2.status[:] = 0x1
    frame2.packet_timestamp[:] = 1
    frame2.field(core.ChanField.REFLECTIVITY)[:] = 200

    writer.save(0, frame1)
    writer.save(FrameSet([frame2]))

    return (frame1, frame2)


def writer_input_handler(frame1, frame2, output_osf_file):
    assert frame1 is not None
    assert frame2 is not None
    assert frame1 != frame2

    assert output_osf_file.exists()
    res_reader = osf.Reader(str(output_osf_file))

    messages = [it for it in res_reader.messages()]
    assert len(messages) == 4  # 2 plus the sensor info message and collation

    read_frames = []
    for msg in messages:
        frame = msg.decode()
        if frame is not None:
            read_frames.append(frame)

    assert read_frames[0] != read_frames[1]

    assert read_frames[0] == frame1
    assert read_frames[1] == frame2


def test_osf_basic_writer(tmp_path, input_info):
    output_osf_file = tmp_path / "out_basic.osf"

    writer = osf.Writer(str(output_osf_file), input_info)
    frame1, frame2 = writer_output_handler(writer, output_osf_file, input_info)
    assert frame1 is not None
    assert frame2 is not None

    assert not writer.is_closed()
    writer.close()
    assert writer.is_closed()

    assert frame1 is not None
    assert frame2 is not None

    writer_input_handler(frame1, frame2, output_osf_file)


def test_osf_with_writer(tmp_path, input_info):
    output_osf_file = tmp_path / "out_with.osf"

    with osf.Writer(str(output_osf_file), input_info) as writer:
        frame1, frame2 = writer_output_handler(
            writer, output_osf_file, input_info)

    writer_input_handler(frame1, frame2, output_osf_file)


def test_osf_save_message(tmp_path, input_osf_file):
    output_osf_file = tmp_path / "out.osf"

    reader = osf.Reader(str(input_osf_file))
    lidar_meta = reader.meta_store.get(osf.LidarSensor)
    lidar_stream_meta = reader.meta_store.get(osf.LidarFrameStream)

    writer = osf.Writer(str(output_osf_file))
    writer.set_metadata_id(reader.metadata_id)
    lidar_id = writer.add_metadata(lidar_meta)
    lidar_stream_id = writer.add_metadata(lidar_stream_meta)

    # WARNING:
    # This is a test of low-level saveMessage API that directly writes
    # any buffers into the message, users usually really don't want to use it
    # directly. Unless one is creating custom messages.
    writer.save_message(lidar_id, 1, 1, bytes([0, 1, 2, 3, 4]), "")
    writer.save_message(lidar_id, 2, 2, bytearray([0, 1, 2, 3, 4]), "")
    writer.save_message(lidar_id, 3, 3, [0, 1, 2, 3, 4], "")
    writer.save_message(lidar_id, 4, 4, np.array([0, 1, 2, 3, 4], dtype=np.uint8), "")

    # WARNING: It's all not safe to do, but because it's a test we know what
    # we are doing here
    total_ls_cnt = 0
    for msg in reader.messages():
        # pass LidarFrame messages as is to a writer
        if msg.id == lidar_stream_meta.id:
            total_ls_cnt += 1
            writer.save_message(lidar_stream_id, msg.ts, msg.ts, msg.buffer, "")

    writer.close()

    assert output_osf_file.exists()

    res_reader = osf.Reader(str(output_osf_file))

    assert res_reader.start_ts == 1  # smallest message timestamp

    custom_cnt = 0
    ls_cnt = 0
    for msg in res_reader.messages():
        if msg.id == lidar_id:
            custom_cnt += 1
            assert len(msg.buffer) == 5  # we always stored 5 bytes
            assert msg.decode() is None  # unknown message can't be decoded
        elif msg.id == lidar_stream_id:
            ls_cnt += 1
            assert msg.decode()

    assert total_ls_cnt == ls_cnt

    # Checks indexing by message index
    assert res_reader.has_message_idx

    assert res_reader.ts_by_message_idx(lidar_id, 0) == 1
    assert res_reader.ts_by_message_idx(lidar_id, 1) == 2
    assert res_reader.ts_by_message_idx(lidar_id, 2) == 3
    assert res_reader.ts_by_message_idx(lidar_id, 3) == 4

    assert res_reader.ts_by_message_idx(lidar_stream_id, 1) == 991687315250
    assert res_reader.ts_by_message_idx(lidar_stream_id, 2) == 991787323080
    assert res_reader.ts_by_message_idx(lidar_stream_id, 0) == 991587364520


def test_osf_messages(input_osf_file):
    reader = osf.Reader(str(input_osf_file))
    assert reader.has_stream_info
    assert ilen(reader.messages()) == 3

    # reading messages within start_ts, end_ts range
    assert ilen(reader.messages(991587364520, 991687315250)) == 2

    lidar_stream = reader.meta_store.get(osf.LidarFrameStream)
    # reading messages of af first LidarFrameStream
    assert ilen(reader.messages([lidar_stream.id])) == 3

    # reading messages of af first LidarFrameStream between start_ts, end_ts range
    assert ilen(reader.messages([lidar_stream.id], 991587364520,
                                991787323080)) == 3

    assert reader.has_message_idx
    assert reader.ts_by_message_idx(lidar_stream.id, 0) > 0


def expect_error_with_handler(osf_file, expected_severity, expected_message):
    err_handler_called = False

    def err_handler(severity, message):
        nonlocal err_handler_called
        err_handler_called = True
        assert severity == expected_severity
        assert message.startswith(expected_message)
        print(severity, expected_message)
    reader = osf.Reader(str(osf_file), err_handler)
    for m in reader.messages():
        m.decode()
    assert err_handler_called


def expect_error_default(osf_file, message):
    try:
        with tempfile.NamedTemporaryFile(delete=False) as logfile:
            init_logger('info', log_file_path=logfile.name)
            # default behavior - it won't throw, but it'll log a message
            reader = osf.Reader(str(osf_file))
            for m in reader.messages():
                m.decode()
        with open(logfile.name) as out:
            stdout = out.read()
            print(stdout)
            # strip off start of log message
            assert stdout.rsplit(']')[-1].strip().startswith(message)
    finally:
        try:
            os.unlink(logfile.name)
        except OSError:
            pass


def test_reader_error_handler(test_data_dir):
    expect_error_with_handler(test_data_dir / 'osfs' / 'bad_crc32.osf',
                              Severity.OUSTER_WARNING, 'Invalid chunk at file offset')
    expect_error_with_handler(test_data_dir / 'osfs' / 'bad_encoding.osf',
                              Severity.OUSTER_WARNING, 'decodeField: could not decode field')


def test_reader_default_error_handler(test_data_dir, capsys):
    expect_error_default(test_data_dir / 'osfs' / 'bad_crc32.osf', 'Invalid chunk at file offset')
    expect_error_default(test_data_dir / 'osfs' / 'bad_encoding.osf',
                         'decodeField: could not decode field')


def test_graceful_field_decode(test_data_dir):
    osf_file = test_data_dir / 'osfs' / 'bad_16_bit_fields.osf'
    assert os.path.exists(osf_file)
    reader = osf.Reader(str(osf_file))
    for m in reader.messages():
        ls = m.decode()
        assert np.count_nonzero(ls.field('RANGE')) > 0


def test_zpng_decode(test_data_dir):
    osf_file = test_data_dir / 'osfs' / 'single_scan_016.osf'
    reader = osf.Reader(str(osf_file))
    for m in reader.messages():
        m.decode()


def test_major_version_error(test_data_dir):
    with pytest.raises(RuntimeError, match=
        r'The OSF file was created with '
        r'schema version 9999.0.0 but this reader supports up to major '
        r'version (\d+). Major version differences may indicate breaking '
        r'changes. The file will not be read to prevent possible '
            r'misinterpretation or data corruption.'):
        osf_file = test_data_dir / 'osfs' / 'single_scan_major_version.osf'
        osf.Reader(str(osf_file))


def test_minor_version_warning(test_data_dir):
    expect_error_with_handler(
        test_data_dir / 'osfs' / 'single_scan_minor_version.osf',
        Severity.OUSTER_WARNING,
        'The OSF file was created with schema version 2.9999.0, '
        'but this reader only supports up to 2.2. Continuing to read using '
        'best-effort compatibility mode. Some fields introduced in newer '
        'versions may be ignored or unrecognized. For full '
        'feature support, consider updating to the latest '
        'version of the OusterSDK.'
    )


def _get_file_hash(file_name):
    result = hashlib.sha512()
    with open(file_name, "rb") as f:
        result.update(f.read())

    return result.hexdigest()


def test_save_adding_pixel_fields_later(tmp_path, input_osf_file):
    """validate that adding non-frame fields after start is illegal"""
    src = open_source(str(input_osf_file))

    test_path = str(os.path.join(tmp_path, "test.osf"))
    writer = osf.Writer(test_path, src.sensor_info)

    for idx, frame_sets in enumerate(src):
        frame = frame_sets[0]
        if idx % 2 == 1:
            frame.add_field("pixel_field", np.uint8)
            with pytest.raises(ValueError, match="added after"):
                writer.save(0, frame)
        else:
            writer.save(0, frame)
    writer.close()

    # make sure we only saved the 2
    src = open_source(test_path)
    assert len(src) == 2


def test_save_removing_pixel_fields_later(tmp_path, input_osf_file):
    """validate that removing non-frame fields after start is illegal"""
    src = open_source(str(input_osf_file))

    test_path = str(os.path.join(tmp_path, "test.osf"))
    writer = osf.Writer(test_path, src.sensor_info)

    for idx, frame_sets in enumerate(src):
        frame = frame_sets[0]
        if idx % 2 == 0:
            frame.add_field("pixel_field", np.uint8)
            writer.save(0, frame)
        else:
            with pytest.raises(ValueError, match="is missing"):
                writer.save(0, frame)

    writer.close()

    # make sure we only saved the 2
    src = open_source(test_path)
    assert len(src) == 2


def test_save_adding_frame_fields_later(tmp_path, input_osf_file):
    """validate that you can add frame fields later when saving"""
    src = open_source(str(input_osf_file))

    test_path = str(os.path.join(tmp_path, "test.osf"))
    writer = osf.Writer(test_path, src.sensor_info)

    for idx, frame_sets in enumerate(src):
        frame = frame_sets[0]
        if idx % 2 == 1:
            frame.add_field("frame_field", np.array([8]), core.FieldClass.FRAME_FIELD)

        writer.save(0, frame)
    writer.close()

    src = open_source(test_path)
    assert len(src) == 3
    for idx, frame_sets in enumerate(src):
        frame = frame_sets[0]
        if idx % 2 == 1:
            assert "frame_field" in [f.name for f in frame.field_types]


def test_save_removing_frame_fields_later(tmp_path, input_osf_file):
    """validate that you can remove frame fields later when saving"""
    src = open_source(str(input_osf_file))

    test_path = str(os.path.join(tmp_path, "test.osf"))
    writer = osf.Writer(test_path, src.sensor_info)

    for idx, frame_sets in enumerate(src):
        frame = frame_sets[0]
        if idx % 2 == 0:
            frame.add_field("frame_field", np.array([8]), core.FieldClass.FRAME_FIELD)

        writer.save(0, frame)
    writer.close()

    src = open_source(test_path)
    assert len(src) == 3
    for idx, frame_sets in enumerate(src):
        frame = frame_sets[0]
        if idx % 2 == 0:
            assert "frame_field" in [f.name for f in frame.field_types]


def test_osf_metadata_replacement_tools(tmp_path, input_osf_file):
    new_metadata = core.SensorInfo.from_default(
        core.LidarMode._1024x10)

    expected_version = "2.1.0"
    test_path = os.path.join(tmp_path, "test.osf")
    backup_path = os.path.join(tmp_path, "test.bak")

    shutil.copyfile(str(input_osf_file), test_path)
    assert os.path.exists(test_path)
    assert os.stat(test_path).st_size == os.stat(str(input_osf_file)).st_size
    hash1 = _get_file_hash(test_path)

    metadata1 = osf.dump_metadata(test_path)
    assert json.loads(metadata1)['header']['version'] == expected_version

    print(osf.dump_metadata(test_path))
    assert not os.path.exists(backup_path)
    osf.backup_osf_file_metablob(test_path, backup_path)
    assert os.path.exists(backup_path)

    osf.osf_file_modify_metadata(test_path, [new_metadata])
    metadata2 = osf.dump_metadata(test_path)
    assert json.loads(metadata2)['header']['version'] == expected_version

    hash2 = _get_file_hash(test_path)

    assert hash1 != hash2

    osf.restore_osf_file_metablob(test_path, backup_path)
    hash3 = _get_file_hash(test_path)
    assert hash2 != hash3
    metadata3 = osf.dump_metadata(test_path)
    assert json.loads(metadata2)['header']['version'] == expected_version
    assert metadata1 == metadata3


def test_full_custom_frame(tmp_path):
    sensor_info = core.SensorInfo.from_default(core.LidarMode._1024x10)
    w, h = sensor_info.format.columns_per_frame, sensor_info.format.pixels_per_column
    ls = core.LidarFrame(sensor_info, [])
    ls.add_field("floats", np.ones((h, w), np.float64))
    ls.add_field("ints", np.ones((h, w), np.uint8))

    test_path = os.path.join(tmp_path, "test.osf")

    writer = osf.Writer(test_path, [sensor_info])
    writer.save(0, ls)
    writer.close()

    src = open_source(test_path)
    frame = src[0][0]
    assert frame.fields == ["floats", "ints"]


def test_osf_empty_field(tmp_path):
    sensor_info = core.SensorInfo.from_default(core.LidarMode._1024x10)
    ls = core.LidarFrame(sensor_info)
    ls.add_field("temperature", np.array([1.2353]), core.FieldClass.FRAME_FIELD)
    ls.add_field("q", np.ones((128, 1024, 0)), core.FieldClass.FRAME_FIELD)

    ls2 = core.LidarFrame(sensor_info)
    ls2.add_field("temperature", np.array([], np.float64), core.FieldClass.FRAME_FIELD)
    ls2.add_field("q", np.ones((128, 1024, 1)), core.FieldClass.FRAME_FIELD)

    test_path = os.path.join(tmp_path, "test.osf")
    writer = osf.Writer(test_path, [sensor_info])
    writer.save(0, ls, 0)
    writer.save(0, ls2, 1)
    writer.close()

    src = open_source(test_path, index=True)

    assert src[0][0].field("temperature") == [1.2353]
    assert len(src[1][0].field("temperature")) == 0

    assert src[1][0].field("q").shape == (128, 1024, 1)
    assert src[0][0].field("q").shape == (128, 1024, 0)


def test_osf_persists_alerts_thermal_and_shot_limiting_fields(tmp_path) -> None:
    sensor_info = SensorInfo.from_default(core.LidarMode._1024x10)
    frame = LidarFrame(sensor_info)
    frame.alert_flags[:] = range(len(frame.alert_flags))
    frame.frame_status = 0xabcdef0011223344
    frame.shutdown_countdown = 0xab
    frame.shot_limiting_countdown = 0xcd
    test_path = os.path.join(tmp_path, "test.osf")
    with osf.Writer(test_path, [sensor_info]) as writer:
        writer.save(0, frame)
    src = open_source(test_path)
    read_frame = next(iter(src))[0]
    assert read_frame is not None
    assert np.array_equal(read_frame.alert_flags, frame.alert_flags)
    assert read_frame.frame_status == frame.frame_status
    assert read_frame.shutdown_countdown == frame.shutdown_countdown
    assert read_frame.shot_limiting_countdown == frame.shot_limiting_countdown


def test_osf_slice_and_cast() -> None:
    sensor_info = SensorInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(sensor_info)
    frame.alert_flags[:] = range(len(frame.alert_flags))
    frame.frame_status = 0xabcdef0011223344
    frame.shutdown_countdown = 0xab
    frame.shot_limiting_countdown = 0xcd

    # an assumption
    assert frame.field(ChanField.RANGE).dtype == np.uint32
    frame.field(ChanField.RANGE)[:] = 0xffffff01

    sliced_frame = osf.slice_and_cast(frame, [FieldType(core.ChanField.RANGE, np.uint8)])
    assert np.array_equal(sliced_frame.alert_flags, frame.alert_flags)
    assert sliced_frame.shutdown_countdown == frame.shutdown_countdown
    assert sliced_frame.shot_limiting_countdown == frame.shot_limiting_countdown
    assert sliced_frame.frame_status == frame.frame_status
    assert sliced_frame.fields == [ChanField.RANGE]

    casted_range_field = sliced_frame.field(ChanField.RANGE)
    assert casted_range_field.shape == frame.field(ChanField.RANGE).shape
    assert casted_range_field.dtype == np.uint8

    # IMPORTANT: casting may be narrowing without warning or error!
    # Notice that values were truncated to the bottom 8 bits here.
    assert np.array_equal(casted_range_field, np.ones(casted_range_field.shape))


def get_size_for_compression_amount(tmp_path, input_info, compression_amount) -> int:
    file_name = tmp_path / "test.osf"
    with osf.Writer(str(file_name), input_info, [], 0, Encoder(PngLidarFrameEncoder(compression_amount))) as writer:
        frame = core.LidarFrame(input_info)
        writer.save(0, frame)
        writer.close()
        return os.path.getsize(file_name)


def test_writer_with_encoder(tmp_path, input_info) -> None:
    """Files encoded with a higher PNG compression level should be smaller than those encoded with a compression level
    of zero."""
    assert get_size_for_compression_amount(
        tmp_path, input_info, 4) < get_size_for_compression_amount(tmp_path, input_info, 0)


def test_async_writer_exception(tmp_path, input_info) -> None:
    """Calling get() on the future returned from the save method should propagate an exception raised from the save
    thread."""
    file_name = tmp_path / "test.osf"
    assert len(input_info.format.pixel_shift_by_row) == 128
    with osf.AsyncWriter(str(file_name), [input_info], [], 0, Encoder(PngLidarFrameEncoder(4))) as writer:
        frame = core.LidarFrame(4, 1024, [], 16)  # frame size doesn't match sensor info
        future = writer.save(0, frame)
        with pytest.raises(ValueError, match=escape(
            "lidar frame size (1024, 4) does not match the sensor info resolution (1024, 128)")
        ):
            future.get()


def test_writer_enforces_lidar_frame_correct_size(tmp_path, input_info):
    file_name = tmp_path / "test.osf"
    with osf.Writer(str(file_name), [input_info]) as w:
        frame = core.LidarFrame(128, 128, [], 16)  # frame size is wrong
        with pytest.raises(ValueError, match=escape(
            "lidar frame size (128, 128) does not match the sensor info resolution (1024, 128)")
        ):
            w.save(0, frame)


def async_writer_destruction_helper(tmp_path):
    file_name = tmp_path / "test.osf"
    _ = osf.AsyncWriter(str(file_name), [SensorInfo.from_default(LidarMode._1024x10)])
    #  should exit normally


def test_async_writer_destruction(tmp_path):
    import multiprocessing

    p = multiprocessing.Process(
        group=None,
        target=async_writer_destruction_helper,
        args=(tmp_path,)
    )
    p.start()
    p.join()
    assert p.exitcode == 0


def test_error_handler_open_source(test_data_dir):
    def error_handler(severity, msg):
        # open_source will re-raise this as a SourceURLException
        raise RuntimeError('Whoopsie!')

    osf_file = test_data_dir / 'osfs' / 'single_scan_major_version.osf'
    with pytest.raises(SourceURLException, match='Whoopsie!'):
        open_source(str(osf_file), error_handler=error_handler)


def test_writer_save_with_none_frame(test_data_dir, tmp_path):
    """It should allow saving None frame_set to represent dropped frame_set."""
    input_osf_file = test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"
    src = open_source(str(input_osf_file))

    test_path = str(os.path.join(tmp_path, "test.osf"))
    writer = osf.Writer(test_path, [src.sensor_info[0], src.sensor_info[0]])

    for frame_sets in src:
        frame = frame_sets[0]
        frame.packet_timestamp[:] = 1
        frames_in = [frame, None]
        frame_set = FrameSet(frames_in)
        writer.save(frame_set)


def test_writer_drop_frame_error(test_data_dir, tmp_path):
    """It should throw the correct error when timestamps are decreasing or zero in a set"""
    input_osf_file = test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"
    src = open_source(str(input_osf_file))

    test_path = str(os.path.join(tmp_path, "test.osf"))
    writer = osf.Writer(test_path, [src.sensor_info[0]])

    frame_set = next(iter(src))

    writer.save(0, frame_set[0], 100)

    frame_set[0].packet_timestamp[:] = 50
    with pytest.raises(OsfDropFrameError, match="decreasing"):
        writer.save(0, frame_set[0])

    with pytest.raises(OsfDropFrameError, match="decreasing"):
        writer.save(FrameSet(frame_set))

    frame_set[0].packet_timestamp[:] = 0
    with pytest.raises(OsfDropFrameError, match="no valid"):
        writer.save(FrameSet(frame_set))


def test_async_writer_drop_frame_err(tmp_path, input_info) -> None:
    """Saving with illegal timestamps should throw immediately rather than asyncronously."""
    file_name = tmp_path / "test.osf"
    with osf.AsyncWriter(str(file_name), [input_info]) as writer:
        frame = core.LidarFrame(input_info)
        frame.status[:] = 1
        frame.packet_timestamp[:] = 100
        writer.save(0, frame)

        frame.packet_timestamp[:] = 99
        with pytest.raises(OsfDropFrameError, match="decreasing"):
            writer.save(0, frame)

        with pytest.raises(OsfDropFrameError, match="decreasing"):
            writer.save(FrameSet([frame]))

        frame.packet_timestamp[:] = 0
        with pytest.raises(OsfDropFrameError, match="no valid"):
            writer.save(FrameSet([frame]))

import pytest
import numpy as np
import hashlib
import shutil
import os
from typing import cast, Iterator

from more_itertools import ilen
from ouster.sdk import open_source

import ouster.sdk.osf as osf
import ouster.sdk.client as client
from ouster.sdk.client import ChanField, FieldType, LidarMode, LidarScan, SensorInfo
from ouster.sdk._bindings.osf import LidarScanStream


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


@pytest.fixture
def input_info(test_data_dir):
    filename = test_data_dir / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.json"
    with open(filename, 'r') as f:
        data = f.read()
    return client.SensorInfo(data)


def test_osf_scan_source_flags(input_osf_file):
    """TWS 20240614: previously, the flags kwarg would affect whether
    FLAGS/FLAGS2 were added to scans read from OSF. Now it has no effect."""
    from ouster.sdk.client import ChanField
    scan_source = open_source(str(input_osf_file), flags=False)
    assert ChanField.FLAGS not in scan_source.fields
    scan_source = open_source(str(input_osf_file), flags=True)
    assert ChanField.FLAGS not in scan_source.fields


# Test that we can save a subset of scan fields and that it errors
# if you try and save a scan missing fields in the metadata
def test_writer_quick(tmp_path, input_info):
    file_name = tmp_path / "test.osf"
    save_fields = [
        client.ChanField.REFLECTIVITY,
        client.ChanField.RANGE
    ]

    error_fields = [client.FieldType(client.ChanField.RANGE, np.uint32)]
    with osf.Writer(str(file_name), input_info, save_fields) as writer:
        scan = client.LidarScan(128, 1024)
        scan.field(client.ChanField.REFLECTIVITY)[:] = 123
        scan.field(client.ChanField.RANGE)[:] = 5

        writer.save(0, scan)

        # also try saving an scan with missing fields
        scan2 = client.LidarScan(128, 1024, error_fields)
        scan2.field(client.ChanField.RANGE)[:] = 6

        with pytest.raises(ValueError):
            writer.save(0, scan2)

        writer.close()

    # then open it and double check that we only got the fields we needed
    res_reader = osf.Reader(str(file_name))

    messages = [it for it in res_reader.messages()]
    for msg in messages:
        if msg.of(LidarScanStream):
            ls = msg.decode()
            if ls:
                # validate that it only has the channels we added
                fields = [field for field in ls.fields]
                assert client.ChanField.RANGE in fields
                assert client.ChanField.REFLECTIVITY in fields
                assert len(fields) == 2

    assert len(messages) == 1


# Test that we can load a subset of fields and that it errors if you try and load missing fields
def test_writer_partial_load(tmp_path, input_info):
    file_name = tmp_path / "test.osf"

    with osf.Writer(str(file_name), input_info) as writer:
        scan = client.LidarScan(128, 1024)
        scan.field(client.ChanField.REFLECTIVITY)[:] = 123
        scan.field(client.ChanField.RANGE)[:] = 5
        scan.add_field("test", np.ones((128, 1024, 1)), client.FieldClass.SCAN_FIELD)
        scan.add_field("test2", np.ones((128, 1024, 1)), client.FieldClass.SCAN_FIELD)

        writer.save(0, scan)

        writer.close()

    # then open it and double check that we only got the fields we needed
    res_reader = osf.Reader(str(file_name))

    messages = [it for it in res_reader.messages()]
    for msg in messages:
        if msg.of(LidarScanStream):
            with pytest.raises(RuntimeError):
                msg.decode(["apples"])
            ls = msg.decode(["RANGE", "test"])
            if ls:
                # validate that it only has the channels we added
                fields = [field for field in ls.fields]
                assert client.ChanField.RANGE in fields
                assert client.ChanField.REFLECTIVITY not in fields
                assert "test" in fields
                assert "test2" not in fields
                assert len(fields) == 2

    assert len(messages) == 1


def writer_output_handler(writer, output_osf_file, info):
    assert writer.filename() == str(output_osf_file)
    assert writer.sensor_info_count() == 1
    assert info == writer.sensor_info(0)
    assert info == writer.sensor_info()[0]

    scan1 = client.LidarScan(128, 1024)
    assert scan1 is not None
    scan1.status[:] = 0x1
    scan1.field(client.ChanField.REFLECTIVITY)[:] = 100

    scan2 = client.LidarScan(128, 1024)
    assert scan2 is not None
    scan2.status[:] = 0x1
    scan2.field(client.ChanField.REFLECTIVITY)[:] = 200

    writer.save(0, scan1)
    writer.save([scan2])

    return (scan1, scan2)


def writer_input_handler(scan1, scan2, output_osf_file):
    assert scan1 is not None
    assert scan2 is not None
    assert scan1 != scan2

    assert output_osf_file.exists()
    res_reader = osf.Reader(str(output_osf_file))

    messages = [it for it in res_reader.messages()]
    assert len(messages) == 2

    read_scan1 = messages[0].decode()
    assert read_scan1 is not None
    read_scan2 = messages[1].decode()
    assert read_scan2 is not None
    assert read_scan1 != read_scan2

    assert read_scan1 == scan1
    assert read_scan2 == scan2
    count = 0
    for _ in res_reader.messages():
        count += 1
    assert count == 2


def test_osf_basic_writer(tmp_path, input_info):
    output_osf_file = tmp_path / "out_basic.osf"

    writer = osf.Writer(str(output_osf_file), input_info)
    scan1, scan2 = writer_output_handler(writer, output_osf_file, input_info)
    assert scan1 is not None
    assert scan2 is not None

    assert not writer.is_closed()
    writer.close()
    assert writer.is_closed()

    assert scan1 is not None
    assert scan2 is not None

    writer_input_handler(scan1, scan2, output_osf_file)


def test_osf_with_writer(tmp_path, input_info):
    output_osf_file = tmp_path / "out_with.osf"

    with osf.Writer(str(output_osf_file), input_info) as writer:
        scan1, scan2 = writer_output_handler(
            writer, output_osf_file, input_info)

    writer_input_handler(scan1, scan2, output_osf_file)


def test_osf_save_message(tmp_path, input_osf_file):
    output_osf_file = tmp_path / "out.osf"

    reader = osf.Reader(str(input_osf_file))
    lidar_meta = reader.meta_store.get(osf.LidarSensor)
    lidar_stream_meta = reader.meta_store.get(osf.LidarScanStream)

    writer = osf.Writer(str(output_osf_file))
    writer.set_metadata_id(reader.metadata_id)
    lidar_id = writer.add_metadata(lidar_meta)
    lidar_stream_id = writer.add_metadata(lidar_stream_meta)

    # WARNING:
    # This is a test of low-level saveMessage API that directly writes
    # any buffers into the message, users usually really don't want to use it
    # directly. Unless one is creating custom messages.
    writer.save_message(lidar_id, 1, 1, bytes([0, 1, 2, 3, 4]))
    writer.save_message(lidar_id, 2, 2, bytearray([0, 1, 2, 3, 4]))
    writer.save_message(lidar_id, 3, 3, [0, 1, 2, 3, 4])
    writer.save_message(lidar_id, 4, 4, np.array([0, 1, 2, 3, 4], dtype=np.uint8))

    # WARNING: It's all not safe to do, but because it's a test we know what
    # we are doing here
    total_ls_cnt = 0
    for msg in reader.messages():
        # pass LidarScan messages as is to a writer
        if msg.id == lidar_stream_meta.id:
            total_ls_cnt += 1
            writer.save_message(lidar_stream_id, msg.ts, msg.ts, msg.buffer)

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

    lidar_stream = reader.meta_store.get(osf.LidarScanStream)
    # reading messages of af first LidarScanStream
    assert ilen(reader.messages([lidar_stream.id])) == 3

    # reading messages of af first LidarScanStream between start_ts, end_ts range
    assert ilen(reader.messages([lidar_stream.id], 991587364520,
                                991787323080)) == 3

    assert reader.has_message_idx
    assert reader.ts_by_message_idx(lidar_stream.id, 0) > 0


def _get_file_hash(file_name):
    result = hashlib.sha512()
    with open(file_name, "rb") as f:
        result.update(f.read())

    return result.hexdigest()


def test_osf_metadata_replacement_tools(tmp_path, input_osf_file):
    new_metadata = client.SensorInfo.from_default(
        client.LidarMode.MODE_1024x10)

    test_path = os.path.join(tmp_path, "test.osf")
    backup_path = os.path.join(tmp_path, "test.bak")

    shutil.copyfile(str(input_osf_file), test_path)
    assert os.path.exists(test_path)
    assert os.stat(test_path).st_size == os.stat(str(input_osf_file)).st_size
    hash1 = _get_file_hash(test_path)

    metadata1 = osf.dump_metadata(test_path)
    assert not os.path.exists(backup_path)
    osf.backup_osf_file_metablob(test_path, backup_path)
    assert os.path.exists(backup_path)

    osf.osf_file_modify_metadata(test_path, [new_metadata])

    hash2 = _get_file_hash(test_path)

    assert hash1 != hash2

    osf.restore_osf_file_metablob(test_path, backup_path)
    hash3 = _get_file_hash(test_path)
    assert hash2 != hash3
    metadata3 = osf.dump_metadata(test_path)
    assert metadata1 == metadata3


def test_full_custom_scan(tmp_path):
    sensor_info = client.SensorInfo.from_default(client.LidarMode.MODE_1024x10)
    w, h = sensor_info.format.columns_per_frame, sensor_info.format.pixels_per_column
    ls = client.LidarScan(h, w, [])
    ls.add_field("floats", np.ones((h, w), np.float64))
    ls.add_field("ints", np.ones((h, w), np.uint8))

    test_path = os.path.join(tmp_path, "test.osf")

    writer = osf.Writer(test_path, [sensor_info])
    writer.save(0, ls)
    writer.close()

    src = open_source(test_path)
    assert src._stream_idx == 0
    assert src.fields == ["floats", "ints"]


def test_osf_empty_field(tmp_path):
    sensor_info = client.SensorInfo.from_default(client.LidarMode.MODE_1024x10)
    ls = client.LidarScan(64, 1024, sensor_info.format.udp_profile_lidar)
    ls.add_field("temperature", np.array([1.2353]), client.FieldClass.SCAN_FIELD)
    ls.add_field("q", np.ones((128, 1024, 0)), client.FieldClass.SCAN_FIELD)

    ls2 = client.LidarScan(64, 1024, sensor_info.format.udp_profile_lidar)
    ls2.add_field("temperature", np.array([], np.float64), client.FieldClass.SCAN_FIELD)
    ls2.add_field("q", np.ones((128, 1024, 1)), client.FieldClass.SCAN_FIELD)

    test_path = os.path.join(tmp_path, "test.osf")
    writer = osf.Writer(test_path, [sensor_info])
    writer.save(0, ls, 0)
    writer.save(0, ls2, 1)
    writer.close()

    src = open_source(test_path, index=True)

    assert src[0].field("temperature") == [1.2353]
    assert len(src[1].field("temperature")) == 0

    assert src[1].field("q").shape == (128, 1024, 1)
    assert src[0].field("q").shape == (128, 1024, 0)


def test_empty_osf_loop(test_data_dir):
    source = open_source(
        str(test_data_dir / "osfs" / "empty_osf.osf"), cycle=True)

    with pytest.raises(StopIteration):
        next(iter(source))


def test_osf_persists_alerts_thermal_and_shot_limiting_fields(tmp_path) -> None:
    sensor_info = SensorInfo.from_default(client.LidarMode.MODE_1024x10)
    scan = LidarScan(sensor_info.h, sensor_info.w)
    scan.alert_flags[:] = range(len(scan.alert_flags))
    scan.frame_status = 0xabcdef0011223344
    scan.shutdown_countdown = 0xab
    scan.shot_limiting_countdown = 0xcd
    test_path = os.path.join(tmp_path, "test.osf")
    with osf.Writer(test_path, [sensor_info]) as writer:
        writer.save(0, scan)
    src = open_source(test_path)
    read_scan = next(cast(Iterator[LidarScan], iter(src)))
    assert np.array_equal(read_scan.alert_flags, scan.alert_flags)
    assert read_scan.frame_status == scan.frame_status
    assert read_scan.shutdown_countdown == scan.shutdown_countdown
    assert read_scan.shot_limiting_countdown == scan.shot_limiting_countdown


def test_osf_slice_and_cast() -> None:
    sensor_info = SensorInfo.from_default(LidarMode.MODE_1024x10)
    scan = LidarScan(sensor_info.h, sensor_info.w)
    scan.alert_flags[:] = range(len(scan.alert_flags))
    scan.frame_status = 0xabcdef0011223344
    scan.shutdown_countdown = 0xab
    scan.shot_limiting_countdown = 0xcd

    # an assumption
    assert scan.field(ChanField.RANGE).dtype == np.uint32
    scan.field(ChanField.RANGE)[:] = 0xffffff01

    sliced_scan = osf.slice_and_cast(scan, [FieldType(client.ChanField.RANGE, np.uint8)])
    assert np.array_equal(sliced_scan.alert_flags, scan.alert_flags)
    assert sliced_scan.shutdown_countdown == scan.shutdown_countdown
    assert sliced_scan.shot_limiting_countdown == scan.shot_limiting_countdown
    assert sliced_scan.frame_status == scan.frame_status
    assert sliced_scan.fields == [ChanField.RANGE]

    casted_range_field = sliced_scan.field(ChanField.RANGE)
    assert casted_range_field.shape == scan.field(ChanField.RANGE).shape
    assert casted_range_field.dtype == np.uint8

    # IMPORTANT: casting may be narrowing without warning or error!
    # Notice that values were truncated to the bottom 8 bits here.
    assert np.array_equal(casted_range_field, np.ones(casted_range_field.shape))

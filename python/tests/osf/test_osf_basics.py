import pytest
import numpy as np

from more_itertools import ilen

import ouster.osf as osf
import ouster.client as client


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


@pytest.fixture
def input_info(test_data_dir):
    filename = test_data_dir / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.json"
    with open(filename, 'r') as f:
        data = f.read()
    return client.SensorInfo(data)


def writerv2_output_handler(writer, output_osf_file, info):
    assert writer.get_filename() == str(output_osf_file)
    assert writer.sensor_info_count() == 1
    assert info == writer.get_sensor_info(0)
    assert info == writer.get_sensor_info()[0]

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


def writerv2_input_handler(scan1, scan2, output_osf_file):
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


def test_osf_basic_writerv2(tmp_path, input_info):
    output_osf_file = tmp_path / "out_basic.osf"

    writer = osf.WriterV2(str(output_osf_file), input_info)
    scan1, scan2 = writerv2_output_handler(writer, output_osf_file, input_info)
    assert scan1 is not None
    assert scan2 is not None

    assert not writer.is_closed()
    writer.close()
    assert writer.is_closed()

    assert scan1 is not None
    assert scan2 is not None

    writerv2_input_handler(scan1, scan2, output_osf_file)


def test_osf_with_writerv2(tmp_path, input_info):
    output_osf_file = tmp_path / "out_with.osf"

    with osf.WriterV2(str(output_osf_file), input_info) as writer:
        scan1, scan2 = writerv2_output_handler(writer, output_osf_file, input_info)

    writerv2_input_handler(scan1, scan2, output_osf_file)


def test_osf_save_message(tmp_path, input_osf_file):
    output_osf_file = tmp_path / "out.osf"

    reader = osf.Reader(str(input_osf_file))
    lidar_meta = reader.meta_store.get(osf.LidarSensor)
    lidar_stream_meta = reader.meta_store.get(osf.LidarScanStream)

    writer = osf.Writer(str(output_osf_file), reader.id)
    lidar_id = writer.addMetadata(lidar_meta)
    lidar_stream_id = writer.addMetadata(lidar_stream_meta)

    # WARNING:
    # This is a test of low-level saveMessage API that directly writes
    # any buffers into the message, users usually really don't want to use it
    # directly. Unless one is creating custom messages.
    writer.saveMessage(lidar_id, 1, bytes([0, 1, 2, 3, 4]))
    writer.saveMessage(lidar_id, 2, bytearray([0, 1, 2, 3, 4]))
    writer.saveMessage(lidar_id, 3, [0, 1, 2, 3, 4])
    writer.saveMessage(lidar_id, 4, np.array([0, 1, 2, 3, 4], dtype=np.uint8))

    # WARNING: It's all not safe to do, but because it's a test we know what
    # we are doing here
    total_ls_cnt = 0
    for msg in reader.messages():
        # pass LidarScan messages as is to a writer
        if msg.id == lidar_stream_meta.id:
            total_ls_cnt += 1
            writer.saveMessage(lidar_stream_id, msg.ts, msg.buffer)

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

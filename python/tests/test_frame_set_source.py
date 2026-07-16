"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.
"""

import os
import pytest
from ouster.sdk import open_source, osf
from ouster.sdk.core import (SensorInfo, LidarMode, FrameSetSourceMetadataSet,
                             ClassMap, ClassMapSet, LidarFrame, FrameSet)

from tests.conftest import OSFS_DATA_DIR
from tests.conftest import PCAPS_DATA_DIR

paths = [os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf"),
         os.path.join(PCAPS_DATA_DIR, 'OS-1-128_v2.3.0_1024x10_lb_n3.pcap')]

L = 3   # BOTH FILES USED ARE STRICTLY 3


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def test_frame_set_source_index_pcap(test_data_dir) -> None:
    """Validate that the index works correctly for pcap"""
    pcap = test_data_dir / "pcaps" / "OS-1-128_v2.3.0_1024x10_lb_n3.pcap"

    src = open_source(str(pcap), index=True)

    # make sure the length is filled out because we are indexed
    assert len(src) == 3

    # make sure the individual index is filled out and in increasing order
    assert len(src.individual_index) == len(src.sensor_info)
    for sensor in src.individual_index:
        i = 0
        last_ts = 0
        for ts, idx in sensor:
            assert idx == i
            assert ts != 0
            assert ts > last_ts
            last_ts = ts
            i = i + 1

    # make sure the full index is filled out and in increasing order
    assert len(src.full_index) == len(src)
    last_ts = 0
    for ts, sensor_idx in src.full_index:
        assert sensor_idx == 0
        assert ts > last_ts
        last_ts = ts


def test_frame_set_source_index(input_osf_file, tmp_path) -> None:
    """Validate that full and individual index work correctly along with size and len"""
    src = open_source(str(input_osf_file))

    writer = osf.Writer(str(tmp_path / "test.osf"), [src.sensor_info[0]] * 2)
    ts = 1

    num_sensors = 2
    num_frames = len(src)
    for frame, in src:
        assert frame is not None
        for i in range(num_sensors):
            frame.packet_timestamp[:] = ts
            ts += 10
            writer.save(i, frame)
    writer.close()

    src = open_source(str(tmp_path / "test.osf"), collate=False)
    assert len(src.full_index) == num_sensors * num_frames
    assert len(src.individual_index) == num_sensors

    # should contain alternating sensors at 10 ns intervals
    for idx, val in enumerate(src.full_index):
        assert val[0] == idx * 10 + 1
        assert val[1] == idx % num_sensors

    # should contain 3 frames per lidar with expected timestamps
    for idx, sensor in enumerate(src.individual_index):
        assert len(sensor) == num_frames
        for jdx, val in enumerate(sensor):
            assert val[1] == jdx * num_sensors + idx
            assert val[0] == (jdx * num_sensors + idx) * 10 + 1

    # try slicing, it should contain num_sensors samples with correct timestamps
    sliced = src[0:num_sensors]
    assert len(sliced) == num_sensors
    assert len(sliced.full_index) == num_sensors
    assert len(sliced.individual_index) == num_sensors
    for i in range(num_sensors):
        ind = sliced.individual_index
        assert len(ind[i]) == 1
        assert ind[i][0][0] == i * 10 + 1
        assert ind[i][0][1] == i

    # then try singling
    for i in range(num_sensors):
        # it should contain num_frames items and only one sensor
        singled = src.single(i)
        assert len(singled.individual_index) == 1
        assert len(singled.individual_index[0]) == num_frames
        assert len(singled.full_index) == num_frames

        # try it sliced
        # it should only contain one item with the correct timestamps
        sliced = singled[0:1]
        assert len(sliced.individual_index) == 1
        assert len(sliced.individual_index[0]) == 1
        assert sliced.individual_index[0][0][0] == 10 * i + 1
        assert sliced.individual_index[0][0][1] == 0
        assert len(sliced.full_index) == 1
        assert sliced.full_index[0][0] == 10 * i + 1
        assert sliced.full_index[0][1] == 0

    # finally test with a step size to make sure that works correctly
    sliced = src[1::num_sensors]
    assert len(sliced) == num_frames
    assert len(sliced.full_index) == num_frames
    assert len(sliced.individual_index) == num_sensors
    print(sliced.full_index)
    print(sliced.individual_index)
    for idx, val in enumerate(sliced.full_index):
        assert val[0] == (1 + idx * num_sensors) * 10 + 1
        assert val[1] == 1
    for idx, sensor in enumerate(sliced.individual_index):
        if idx != 1:
            assert len(sensor) == 0
        else:
            for jdx, val in enumerate(sliced.individual_index[i]):
                assert val[1] == jdx
                assert val[0] == sliced.full_index[jdx][0]


def test_single_collate(input_osf_file) -> None:
    """Validate that singling a collated source throws the right exception"""
    src = open_source(str(input_osf_file), collate=True)
    with pytest.raises(RuntimeError, match="single stream from an already collated source"):
        src.single(0)


@pytest.mark.parametrize("collated", [True, False])
def test_length(input_osf_file, collated: bool) -> None:
    """Validate that length is correct after singling and slicing both collated and non-collated"""
    src = open_source(str(input_osf_file), collate=collated)
    assert len(src) == 3
    assert len(src[0:1]) == 1
    assert len(src[0:2]) == 2

    if not collated:
        assert len(src.single(0)) == 3
        assert len(src.single(0)[0:1]) == 1


def test_frame_set_source_metadata(tmp_path):
    sensor_info = SensorInfo.from_default(LidarMode._1024x10)
    lidar_frame = LidarFrame(sensor_info)
    lidar_frame.status[:] = 1
    lidar_frame.packet_timestamp[:] = 123456789
    lidar_frame.frame_id = 42

    writer = osf.Writer(str(tmp_path / "test.osf"), [sensor_info])

    class_map1 = ClassMap({
        1: 'dog',
        2: 'cat'
    })
    class_map2 = ClassMap({
        1: 'tree',
        2: 'bush'
    })
    class_maps = ClassMapSet({
        'four_legs': class_map1,
        'zero_legs': class_map2
    })
    metadata = FrameSetSourceMetadataSet()
    metadata['class_maps'] = class_maps
    metadata['additional_info'] = "Test additional info"
    writer.save(metadata)

    assert metadata['additional_info'] == "Test additional info"
    assert metadata['class_maps'] == class_maps
    writer.save(FrameSet([lidar_frame]))
    writer.close()

    src = open_source(str(tmp_path / "test.osf"))
    class_map_set_read = src.metadata('class_maps')
    dog_class = class_map_set_read['four_legs'][1]

    assert dog_class == 'dog'
    assert src.metadata_keys() == {'class_maps', 'additional_info'}
    assert class_map_set_read == class_maps

    assert src.metadata('additional_info') == "Test additional info"
    lidar_frame_2 = next(iter(src))[0]
    assert lidar_frame == lidar_frame_2


def test_frame_set_source_metadata_unsupported_type(tmp_path):
    metadata = FrameSetSourceMetadataSet()
    with pytest.raises(ValueError, match=r"Value must be either a string or ClassMapSet."):
        metadata['custom_int'] = 42

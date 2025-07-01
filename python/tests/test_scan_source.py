"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.
"""

import os
import pytest
from ouster.sdk import open_source, osf

from tests.conftest import OSFS_DATA_DIR
from tests.conftest import PCAPS_DATA_DIR

paths = [os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf"),
         os.path.join(PCAPS_DATA_DIR, 'OS-1-128_v2.3.0_1024x10_lb_n3.pcap')]

L = 3   # BOTH FILES USED ARE STRICTLY 3


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def test_scan_source_index(input_osf_file, tmp_path) -> None:
    """Validate that full and individual index work correctly along with size and len"""
    src = open_source(str(input_osf_file))

    writer = osf.Writer(str(tmp_path / "test.osf"), [src.sensor_info[0]] * 2)
    ts = 0

    num_sensors = 2
    num_scans = len(src)
    for scan, in src:
        assert scan is not None
        for i in range(num_sensors):
            scan.packet_timestamp[:] = ts
            ts += 10
            writer.save(i, scan)
    writer.close()

    src = open_source(str(tmp_path / "test.osf"), collate=False)
    assert len(src.full_index) == num_sensors * num_scans
    assert len(src.individual_index) == num_sensors

    # should contain alternating sensors at 10 ns intervals
    for idx, val in enumerate(src.full_index):
        assert val[0] == idx * 10
        assert val[1] == idx % num_sensors

    # should contain 3 scans per lidar with expected timestamps
    for idx, sensor in enumerate(src.individual_index):
        assert len(sensor) == num_scans
        for jdx, val in enumerate(sensor):
            assert val[1] == jdx * num_sensors + idx
            assert val[0] == (jdx * num_sensors + idx) * 10

    # try slicing, it should contain num_sensors samples with correct timestamps
    sliced = src[0:num_sensors]
    assert len(sliced) == num_sensors
    assert len(sliced.full_index) == num_sensors
    assert len(sliced.individual_index) == num_sensors
    for i in range(num_sensors):
        ind = sliced.individual_index
        assert len(ind[i]) == 1
        assert ind[i][0][0] == i * 10
        assert ind[i][0][1] == i

    # then try singling
    for i in range(num_sensors):
        # it should contain num_scans items and only one sensor
        singled = src.single(i)
        assert len(singled.individual_index) == 1
        assert len(singled.individual_index[0]) == num_scans
        assert len(singled.full_index) == num_scans

        # try it sliced
        # it should only contain one item with the correct timestamps
        sliced = singled[0:1]
        assert len(sliced.individual_index) == 1
        assert len(sliced.individual_index[0]) == 1
        assert sliced.individual_index[0][0][0] == 10 * i
        assert sliced.individual_index[0][0][1] == 0
        assert len(sliced.full_index) == 1
        assert sliced.full_index[0][0] == 10 * i
        assert sliced.full_index[0][1] == 0

    # finally test with a step size to make sure that works correctly
    sliced = src[1::num_sensors]
    assert len(sliced) == num_scans
    assert len(sliced.full_index) == num_scans
    assert len(sliced.individual_index) == num_sensors
    print(sliced.full_index)
    print(sliced.individual_index)
    for idx, val in enumerate(sliced.full_index):
        assert val[0] == (1 + idx * num_sensors) * 10
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

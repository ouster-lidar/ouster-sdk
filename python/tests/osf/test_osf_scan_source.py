# TODO: de-duplicate OSF writers in tests
import pytest
import tempfile
import ouster.sdk.osf as osf
import ouster.sdk.core as core
import os
import sys
import numpy as np


@pytest.fixture
def input_info(test_data_dir):
    filename = test_data_dir / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.json"
    with open(filename, 'r') as f:
        data = f.read()
    return core.SensorInfo(data)


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


def test_bad_filename():
    """It should raise a RuntimeError if the provided file is not an OSF file."""
    with pytest.raises(RuntimeError, match="provided OSF file is not a valid OSF file."):
        with tempfile.NamedTemporaryFile() as f:
            osf.OsfScanSource(f.name)


def test_scans_num(input_info):
    """New OSF files should always be indexed and have scans_num and len"""
    src = None
    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, [input_info, input_info]) as w:
                scan1 = core.LidarScan(128, 1024)
                scan1.status[:] = 0x1
                scan1.field(core.ChanField.REFLECTIVITY)[:] = 100
                w.save(0, scan1)
                w.save(1, scan1)
                scan1.packet_timestamp[:] = 1
                w.save(0, scan1)
                w.save(1, scan1)
        src = osf.OsfScanSource(f.name)
        assert src.scans_num == [2, 2]
        assert len(src) == 4
    finally:
        if src:
            src.close()
        os.unlink(f.name)


def test_osf_open(input_osf_file):
    """Make sure the file opens and has correct metadata in the scans"""
    source = osf.OsfScanSource(str(input_osf_file))
    got_scan = False
    for scan in source:
        got_scan = True
        assert scan[0].sensor_info == source.sensor_info[0]
    assert got_scan


@pytest.mark.skipif(sys.platform.startswith("win"), reason="Broken on Windows")
def test_missing_streams(input_osf_file):
    """Make sure the OsfScanSource can handle empty streams"""
    data = osf.OsfScanSource(str(input_osf_file))
    scan = next(iter(data))[0]
    data.close()
    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            w = osf.Writer(f.name, [scan.sensor_info, scan.sensor_info, scan.sensor_info])
            scan.frame_id = 0
            w.save(0, scan)
            scan.frame_id = 2
            w.save(2, scan)
            w.close()

        result = core.collate(osf.OsfScanSource(f.name))
        scan = next(iter(result))[0]

        data = None
        for res in result:
            data = res
        assert data[1] is None
        assert data[0] is not None
        assert data[2] is not None
        assert result.scans_num == [1, 0, 1]
    finally:
        if result:
            result.close()
        os.unlink(f.name)


def test_osf_info_modification_bug(input_osf_file):
    """Make sure that modifying the pixel shift by row in the returned metadata doesnt break parsing"""
    source = osf.OsfScanSource(str(input_osf_file))

    def get_first_scan():
        for scan in source:
            return scan[0]

    scan_before = get_first_scan()
    field_before = scan_before.field(core.ChanField.REFLECTIVITY).astype(np.uint8)

    source.sensor_info[0].format.pixel_shift_by_row = np.zeros((128), np.int32)

    scan_after = get_first_scan()
    field_after = scan_after.field(core.ChanField.REFLECTIVITY).astype(np.uint8)

    assert np.array_equal(field_before, field_after)

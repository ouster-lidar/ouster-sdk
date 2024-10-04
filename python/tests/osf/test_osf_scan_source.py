# TODO: de-duplicate OSF writers in tests
import pytest
import tempfile
import ouster.sdk.osf as osf
import ouster.sdk.client as client
import os
import numpy as np


@pytest.fixture
def input_info(test_data_dir):
    filename = test_data_dir / "pcaps" / "OS-0-128-U1_v2.3.0_1024x10.json"
    with open(filename, 'r') as f:
        data = f.read()
    return client.SensorInfo(data)


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
                scan1 = client.LidarScan(128, 1024)
                scan1.status[:] = 0x1
                scan1.field(client.ChanField.REFLECTIVITY)[:] = 100
                w.save(0, scan1)
                w.save(1, scan1)
                scan1.packet_timestamp[:] = 1
                w.save(0, scan1)
                w.save(1, scan1)
        src = osf.OsfScanSource(f.name)
        assert src.scans_num == [2, 2]
        assert len(src) == 2
    finally:
        if src:
            src.close()
        os.unlink(f.name)


def test_osf_info_modification_bug(input_osf_file):
    """Make sure that modifying the pixel shift by row in the returned metadata doesnt break parsing"""
    source = osf.OsfScanSource(str(input_osf_file))

    def get_first_scan():
        for scan in source:
            return scan[0]

    scan_before = get_first_scan()
    field_before = scan_before.field(client.ChanField.REFLECTIVITY).astype(np.uint8)

    source.metadata[0].format.pixel_shift_by_row = np.zeros((128), np.int32)

    scan_after = get_first_scan()
    field_after = scan_after.field(client.ChanField.REFLECTIVITY).astype(np.uint8)

    assert np.array_equal(field_before, field_after)

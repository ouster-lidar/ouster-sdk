# TODO: de-duplicate OSF writers in tests
import copy
import pytest
import tempfile
import ouster.sdk.osf as osf
import ouster.sdk.core as core
from ouster.sdk.zone_monitor import ZoneSet, Zone, Stl, ZoneMode, CoordinateFrame
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


def test_header_verification_failure():
    """It should raise a RuntimeError if the provided file is not an OSF file."""
    with pytest.raises(RuntimeError, match="OSF header verification has failed."):
        with tempfile.NamedTemporaryFile(delete=False) as f:
            f.write(b"abc" * 1000)  # write some bad data to it
            f.close()   # Windows seems to have trouble with the file being open already
            osf.OsfScanSource(f.name)
        try:
            os.unlink(f.name)
        except (OSError, FileNotFoundError):
            pass


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


def test_osf_scan_source_zm_config_thaw(input_info, test_data_dir):
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
        src = osf.OsfScanSource(f.name)
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


def test_osf_scan_source_out_of_bounds_access(input_osf_file):
    source = osf.OsfScanSource(str(input_osf_file))
    assert len(source) == 3
    with pytest.raises(IndexError):
        scans = source[3]

    info = source.sensor_info[0]
    new_infos = [copy.deepcopy(info), copy.deepcopy(info), copy.deepcopy(info)]
    scans = [scan[0] for scan in source]

    scans[0].packet_timestamp[:] = 100
    scans[1].packet_timestamp[:] = 200
    collation_1 = core.LidarScanSet([scans[0], scans[0], scans[0]])
    collation_2 = core.LidarScanSet([scans[1], scans[1], scans[1]])

    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, new_infos) as w:
                w.save(collation_1)
                w.save(collation_2)
            w.close()
        collated_src = osf.OsfScanSource(f.name)
        with pytest.raises(IndexError):
            collated_src[2]
    finally:
        try:
            os.unlink(f.name)
        except (PermissionError, FileNotFoundError):
            pass


def test_osf_scan_source_collating(input_osf_file):
    input_src = osf.OsfScanSource(str(input_osf_file))
    scans = [scan[0] for scan in input_src]
    assert len(scans) == 3
    info = input_src.sensor_info[0]
    input_src.close()

    scans[0].packet_timestamp[:] = 100
    scans[1].packet_timestamp[:] = 200
    scans[2].packet_timestamp[:] = 300

    new_infos = [copy.deepcopy(info), copy.deepcopy(info), copy.deepcopy(info)]
    new_infos[0].sn = 0
    new_infos[1].sn = 1
    new_infos[2].sn = 2

    collation_1 = core.LidarScanSet([scans[0], scans[0], scans[0]])
    field_1 = collation_1.add_field("coll1_field", np.float32, (100, 100))
    field_1[:] = 3.1415

    collation_2 = core.LidarScanSet([scans[2], scans[2], scans[2]])
    field_2 = collation_2.add_field("coll2_field", np.uint8, (100,))
    field_2[:] = 8

    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            with osf.Writer(f.name, new_infos) as w:
                w.save(collation_1)
                w.save(0, scans[1])
                w.save(1, scans[1])
                w.save(2, scans[1])
                w.save(collation_2)
            w.close()
        src = osf.OsfScanSource(f.name)
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

import copy
import numpy as np
import os
from re import escape
import pytest
import tempfile
from ouster.sdk.util import resolve_metadata_multi
from ouster.sdk.core import SensorInfo, FieldClass
from ouster.sdk import open_source, open_packet_source, SourceURLException
from tests.conftest import PCAPS_DATA_DIR, BAGS_DATA_DIR, OSFS_DATA_DIR


def test_open_source_empty_source_url():
    """It should raise an error if the src url is the empty string."""
    with pytest.raises(ValueError, match="No valid source specified"):
        open_source('')


def test_open_source_unsupported_source_type():
    """It raises a NotImplementedError if the source type is not supported."""
    with tempfile.NamedTemporaryFile(suffix='.csv') as f:
        with pytest.raises(SourceURLException, match="Could not open scan source. Unhandled source type CSV."):
            open_source(f.name)


def test_open_source_undetermined_source_type():
    """It raises a RuntimeError if the source type couldn't be determined."""
    with pytest.raises(SourceURLException, match=escape("Source type of 'unknown source'"
            " not found. File or host not found.")):
        open_source('unknown source')


def test_open_source_unhandled_source_type():
    """It raises a RuntimeError if the source type is unhandled."""
    with tempfile.NamedTemporaryFile(suffix='.txt') as f:
        with pytest.raises(SourceURLException, match=escape("Could not detect IO type from file"
                " extension. Expecting one of .osf, .pcap, .bag, .mcap, .csv, .png, .ply, .pcd, .stl or .las")):
            open_source(f.name)


def test_open_source_meta_not_supported():
    """It raises a RuntimeError if the meta keyword is provided to an unsupported source type."""
    file_path = os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf")
    with pytest.raises(SourceURLException, match="Parameter 'meta' not supported by OsfScanSource."):
        open_source(file_path, meta=['fake_meta.json'])


def test_open_source_meta_pcap():
    """Providing the meta parameter to open source should override the metadata files used by the PcapScanSource."""
    pcap_file_path = os.path.join(PCAPS_DATA_DIR, 'VLI-16-one-packet.pcap')
    json_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')

    with open(json_file_path, 'r') as f:
        expected = SensorInfo(f.read())

    # the test file is different than what would be resolved ordinarily
    assert json_file_path not in resolve_metadata_multi(pcap_file_path)
    src = open_source(pcap_file_path, meta=(json_file_path,))
    assert src.sensor_info[0] == expected


def test_open_source_field_names_pcap() -> None:
    file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')

    tests = [(["RANGE"], ["RANGE"]), ([], []), (None, ["FLAGS", "NEAR_IR", "RANGE", "REFLECTIVITY"])]
    for parameter, expected in tests:
        src = open_source(file_path, field_names=parameter)
        got_msg = False
        for s, in src:
            got_msg = True
            fields = list(s.fields)  # type: ignore
            assert len(fields) == len(expected)
            for field in expected:
                assert field in fields

        assert got_msg


def test_open_source_field_names_osf() -> None:
    file_path = os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf")

    tests = [(["RANGE"], ["RANGE"]), ([], []), (None, ["NEAR_IR", "RANGE", "REFLECTIVITY"])]
    for parameter, expected in tests:
        src = open_source(file_path, field_names=parameter)
        got_msg = False
        for s, in src:
            got_msg = True
            fields = list(s.fields)  # type: ignore
            assert len(fields) == len(expected)
            for field in expected:
                assert field in fields

        assert got_msg


def test_unindexed_scans_num() -> None:
    """It should return a list of None when index=False (which is the default.)"""
    file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')

    src = open_source(file_path, index=False)
    with pytest.raises(RuntimeError, match="Cannot perform 'scans_num' on an unindexed source."
            " Specify the index parameter as true when creating the source to produce an index."):
        assert src.scans_num == [None]


def test_unindexed_len() -> None:
    """It should raise NotImplemented."""
    file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')

    src = open_source(file_path, index=False)
    with pytest.raises(TypeError, match="Cannot get the length of an unindexed scan source."):
        len(src)


def test_raw_fields() -> None:
    file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')

    src = open_source(file_path, raw_fields=True)
    scan = next(iter(src))[0]
    assert scan is not None
    ft_names = [ft.name for ft in scan.field_types]
    assert "RAW32_WORD1" in ft_names


def test_raw_headers() -> None:
    file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')

    src = open_source(file_path, raw_headers=True)
    scan = next(iter(src))[0]
    assert scan is not None
    ft_names = [ft.name for ft in scan.field_types]
    assert "RAW_HEADERS" in ft_names


def test_open_packet_source_pcap() -> None:
    file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')

    src = open_packet_source(file_path)
    got_packet = False
    for idx, packet in src:
        got_packet = True
        break
    assert got_packet


def test_open_packet_source_bag() -> None:
    file_path = os.path.join(BAGS_DATA_DIR, '512x10_raw.bag')

    src = open_packet_source(file_path)
    got_packet = False
    for idx, packet in src:
        got_packet = True
        break
    assert got_packet


def test_source_no_lidar() -> None:
    """Validate that we can load a source with no lidar data and see no lidar fields"""
    file_path = os.path.join(PCAPS_DATA_DIR, 'imu_zm_no_lidar.pcap')

    src = open_source(file_path)
    scan = next(iter(src))[0]
    assert scan is not None
    ft_names = [ft.name for ft in scan.field_types]
    assert "IMU_PACKET_TIMESTAMP" in ft_names
    assert "ZONE_PACKET_TIMESTAMP" in ft_names
    # assert none are pixel fields
    for ft in scan.field_types:
        assert ft.field_class != FieldClass.PIXEL_FIELD

    # also assert that we see 0 lidar packets per frame
    assert scan.sensor_info.format.lidar_packets_per_frame() == 0


def test_open_source_collating_osf() -> None:
    file_path = os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf")

    input_src = open_source(str(file_path))
    scans = [scan[0] for scan in input_src]
    assert len(scans) == 3
    info = input_src.sensor_info[0]
    input_src.close()

    scans[0].packet_timestamp[:] = 100  # type: ignore
    scans[1].packet_timestamp[:] = 200  # type: ignore
    scans[2].packet_timestamp[:] = 300  # type: ignore

    from ouster.sdk.core import LidarScanSet

    collation = LidarScanSet([scans[0], scans[1], None])
    field = collation.add_field("my_field", np.float32, (100, 100))
    field[:] = 3.1415

    from ouster.sdk.osf import Writer
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".osf") as f:
            new_infos = [copy.deepcopy(info), copy.deepcopy(info), copy.deepcopy(info)]
            new_infos[0].sn = 0
            new_infos[1].sn = 1
            new_infos[2].sn = 2
            with Writer(f.name, new_infos) as w:
                w.save(collation)
            w.close()

        # check we get the same collation out
        src_collated = open_source(f.name, True)
        collation_out = next(iter(src_collated))
        assert len(src_collated) == 1
        assert collation_out == collation
        src_collated.close()

        # check we get single scan if asked to not collate
        src_non_collated = open_source(f.name, False)
        assert len(src_non_collated) == 2
        scans_out = next(iter(src_non_collated))
        assert len(scans_out) == 1

    finally:
        try:
            os.unlink(f.name)
        except (PermissionError, FileNotFoundError):
            pass

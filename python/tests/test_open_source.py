import os
from re import escape
import pytest
import tempfile
from ouster.sdk.util import resolve_metadata_multi
from ouster.sdk.core import SensorInfo
from ouster.sdk import open_source, open_packet_source, SourceURLException
from tests.conftest import PCAPS_DATA_DIR, BAGS_DATA_DIR, OSFS_DATA_DIR


def test_open_source_empty_source_url():
    """It should raise an error if the src url is the empty string."""
    with pytest.raises(ValueError, match="No valid source specified"):
        open_source('')


def test_open_source_unsupported_source_type():
    """It raises a NotImplementedError if the source type is not supported."""
    with tempfile.NamedTemporaryFile(suffix='.csv') as f:
        with pytest.raises(NotImplementedError, match="The io_type:IoType.CSV is not supported!"):
            open_source(f.name)


def test_open_source_undetermined_source_type():
    """It raises a RuntimeError if the source type couldn't be determined."""
    with pytest.raises(SourceURLException, match=escape("Failed to create scan_source for url ['unknown source']\n"
            "more details: Source type of 'unknown source' expected to be a sensor hostname,"
            " ip address, or a .pcap, .osf, or .bag file.")):
        open_source('unknown source')


def test_open_source_meta_not_supported(monkeypatch):
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

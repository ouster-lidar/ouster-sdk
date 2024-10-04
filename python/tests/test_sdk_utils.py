import numpy as np
import pytest
import tempfile
import os
import re
from tests.conftest import PCAPS_DATA_DIR
from ouster.sdk.client import SensorInfo, LidarMode, UDPProfileLidar, PacketWriter, LidarScan, ScanBatcher
from os.path import commonprefix
from pathlib import Path
from ouster.sdk.util import img_aspect_ratio
from ouster.sdk.util.metadata import resolve_metadata, \
    resolve_metadata_multi, data_must_be_a_file_err, meta_must_be_a_file_err
from ouster.sdk.util import scan_to_packets


def test_resolve_metadata_when_data_not_a_file():
    """It should raise an exception if the data path is not a file"""
    with pytest.raises(ValueError, match=data_must_be_a_file_err):
        resolve_metadata('')


def test_resolve_metadata_when_data_not_a_file_2():
    """It should raise an exception if the data path is not a file"""
    with pytest.raises(ValueError, match=data_must_be_a_file_err):
        with tempfile.TemporaryDirectory() as directory:
            resolve_metadata(directory)


def test_resolve_metadata_when_metadata_path_provided_is_not_a_file():
    """It should raise an exception if the provided metadata path is
    not a file."""
    with pytest.raises(ValueError, match=meta_must_be_a_file_err):
        with tempfile.NamedTemporaryFile() as f:
            with tempfile.TemporaryDirectory() as directory:
                resolve_metadata(f.name, directory)


def test_resolve_metadata_min_prefix():
    """When there is no JSON file that has a common prefix with the data file,
    resolve_metadata should return None."""
    with tempfile.TemporaryDirectory() as directory:
        dir_path = Path(directory)
        # create some test files (one representing data, one metadata)

        test_data_filename = 'foo'
        test_meta_filename = 'tmpfile'
        assert not commonprefix([test_data_filename, test_meta_filename])
        open(dir_path / test_data_filename, 'a').close()
        open(dir_path / f'{test_meta_filename}.json', 'a').close()

        assert resolve_metadata(dir_path / test_data_filename) is None


def test_resolve_metadata_min_prefix_2():
    """The minimum common prefix between a data path and a resolved metadata
    file should have a length greater than zero."""
    with tempfile.TemporaryDirectory() as directory:
        dir_path = Path(directory)
        # create some test files (one representing data, one metadata)
        test_filename = 'tmpfile'
        open(dir_path / test_filename, 'a').close()
        open(dir_path / f'{test_filename}.json', 'a').close()

        assert resolve_metadata(dir_path / test_filename) == str(dir_path / f'{test_filename}.json')


def test_resolve_metadata_multi():
    """It should return an empty list if no JSON files match the data path."""
    with tempfile.TemporaryDirectory() as directory:
        dir_path = Path(directory)
        # create some test files (one representing data, one metadata)

        test_data_filename = 'foo'
        test_meta_filename = 'tmpfile'
        assert not commonprefix([test_data_filename, test_meta_filename])
        open(dir_path / test_data_filename, 'a').close()
        open(dir_path / f'{test_meta_filename}.json', 'a').close()

        assert resolve_metadata_multi(dir_path / test_data_filename) == []


def test_resolve_metadata_multi_2():
    """It should return files that exist and share a prefix with the data file."""
    # read files with below prefix from PCAPS_DATA_DIR, the serial numbers for metadata on each is different
    test_data_prefix = 'OS-0-128_v3.0.1_1024x10'
    assert set(resolve_metadata_multi(os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.pcap'))) == set(
        [os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.2.json'),
         os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.json')])


def test_resolve_metadata_multi_exception_raised():
    """It should raise an exception when files that exist and share a prefix with the data file
    also have same sensor serial no."""
    # read files with below prefix from PCAPS_DATA_DIR, the serial numbers for metadata on each is same
    test_data_prefix = 'OS-0-128_v3.0.1_1024x10_20240321_125947'
    test_data_filename = os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.pcap')

    with open(os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.json')) as file:
        meta_content = file.read()
        si = SensorInfo(meta_content)

    metas = set([os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.2.json'),
                 os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.json')])

    expected_exception_msg = "\n".join(["The following metadata files identified for "
                   f"{test_data_filename} contain configuration for the same sensor {si.sn}. Files: "
                   f"{', '.join(sorted((set(metas))))} ",
                   "To resolve this, remove the extra metadata file(s) or specify the metadata "
                   "files manually using the --meta option."])

    with pytest.raises(RuntimeError, match=re.escape(expected_exception_msg)):
        resolve_metadata_multi(test_data_filename)


def test_img_aspect_ratio():
    # aspect ratio is independent of the mode
    assert img_aspect_ratio(SensorInfo.from_default(LidarMode.MODE_1024x10)) == pytest.approx(0.09228333333333334)
    assert img_aspect_ratio(SensorInfo.from_default(LidarMode.MODE_2048x10)) == pytest.approx(0.09228333333333334)
    assert img_aspect_ratio(SensorInfo.from_default(LidarMode.MODE_512x10)) == pytest.approx(0.09228333333333334)


def test_scan_to_packets():
    """It should write out the packet header fields from LidarScan if provided.

    To test this, we'll create a scan, convert it to packets, and then batch
    the packets back into a new scan and compare the scans.
    """
    profile = UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.init_id = 1234
    info.sn = "5678"

    info.format.columns_per_frame = 1024
    info.format.columns_per_packet = 16
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.column_window = (0, 1024)  # ScanBatcher depends on this being set

    writer = PacketWriter.from_profile(profile, info.format.pixels_per_column, info.format.columns_per_packet)
    scan = LidarScan(info)
    scan.frame_status = 0xabcdef0123456789
    scan.alert_flags[:] = range(len(scan.alert_flags))
    assert scan.shot_limiting() == 0x08      # derived from frame_status
    assert scan.thermal_shutdown() == 0x09   # derived from frame_status
    scan.shutdown_countdown = 30
    scan.shot_limiting_countdown = 29
    scan.frame_id = 100
    scan.timestamp[:] = range(len(scan.timestamp))
    scan.measurement_id[:] = range(len(scan.measurement_id))

    # Important: scan_to_packets assumes these are set.
    # Also, scan_to_packets dislikes packets with timestamps equal to zero
    assert len(scan.packet_timestamp) == scan.packet_count
    scan.packet_timestamp[:] = range(1, len(scan.packet_timestamp) + 1)
    scan.status[:] = np.ones(scan.status.shape)

    packets = scan_to_packets(scan, info)
    crcs = [writer.crc(packet.buf) for packet in packets]

    # Check results
    assert len(packets) == len(scan.packet_timestamp)
    batcher = ScanBatcher(info)

    # Batch the packets back into a scan, which we use to test the scan-to-packet results, since methods to get packet
    # column headers aren't bound
    output_scan = LidarScan(info)
    for i, packet in enumerate(packets):
        assert writer.init_id(packet.buf) == info.init_id
        assert writer.prod_sn(packet.buf) == int(info.sn)
        assert writer.frame_id(packet.buf) == scan.frame_id
        assert writer.crc(packet.buf) == writer.calculate_crc(packet.buf)
        assert writer.packet_type(packet.buf) == 0x1
        batcher(packet, output_scan)
    assert np.array_equal(output_scan.packet_timestamp, scan.packet_timestamp)
    assert np.array_equal(output_scan.timestamp, scan.timestamp)
    assert np.array_equal(output_scan.measurement_id, scan.measurement_id)
    assert np.array_equal(output_scan.alert_flags, scan.alert_flags)
    assert output_scan.shutdown_countdown == scan.shutdown_countdown
    assert output_scan.shot_limiting_countdown == scan.shot_limiting_countdown
    assert output_scan.shot_limiting() == scan.shot_limiting()
    assert output_scan.thermal_shutdown() == scan.thermal_shutdown()

    # Ensure that CRCs surive the round trip packet->scan->packet conversion.
    crcs2 = [writer.crc(packet.buf) for packet in scan_to_packets(output_scan, info)]
    assert crcs == crcs2

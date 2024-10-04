"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.
"""

import os
import json
import pytest

from tests.conftest import PCAPS_DATA_DIR


def patch_json_file(file_in, tmp_path, imu_port=None,
                    lidar_port=None, prod_sn=None):
    result_out = os.path.join(tmp_path, os.path.basename(file_in))
    meta = json.loads(open(file_in, 'r').read())

    if imu_port is not None:
        meta['config_params']['udp_port_imu'] = imu_port
    if lidar_port is not None:
        meta['config_params']['udp_port_lidar'] = lidar_port
    if prod_sn is not None:
        meta['sensor_info']['prod_sn'] = prod_sn

    print(meta)
    with open(result_out, 'w') as f:
        f.write(json.dumps(meta))

    return result_out


def calc_path(name):
    return os.path.join(PCAPS_DATA_DIR, name)


@pytest.mark.parametrize("path",
                         ['same_ports',
                          'same_ports_legacy',
                          'same_ports_nonlegacy'])
def test_multiple_scan_source_imu_collsion(tmp_path, path):
    path = calc_path(path)
    file_path = path + ".pcap"

    # new interface
    from ouster.sdk.pcap import PcapScanSource, PcapDuplicatePortException  # type: ignore

    with pytest.raises(PcapDuplicatePortException):
        _ = PcapScanSource(file_path,
                           meta=(path + ".1.json", path + ".2.json"),
                           cycle=False)


def test_multiple_scan_source_pcap(tmp_path) -> None:
    file_path = calc_path('same_ports.pcap')

    # Remove the imu conflict for this test
    meta1_out_path = patch_json_file(calc_path('same_ports.1.json'),
                                     tmp_path,
                                     imu_port=7503)
    meta2_out_path = calc_path('same_ports.2.json')

    # new interface
    from ouster.sdk.client import MultiScanSource
    from ouster.sdk.pcap import PcapScanSource  # type: ignore

    scan_source: MultiScanSource
    scan_source = PcapScanSource(file_path,
                                 meta=[meta1_out_path, meta2_out_path],
                                 cycle=False)

    test_scans = list(scan_source)[0]

    if not test_scans[1] or not test_scans[0]:
        assert False

    assert test_scans[1].frame_id == 883
    assert test_scans[1].h == 64

    assert test_scans[0].frame_id == 1542
    assert test_scans[0].h == 128


def test_bad_id_multiple_with_legacy_and_non_legacy(tmp_path) -> None:
    file_path = calc_path('same_ports.pcap')

    # Remove the imu conflict for this test
    # Change the sn for this test
    meta1_out_path = patch_json_file(calc_path('same_ports.1.json'),
                                     tmp_path,
                                     imu_port=7503,
                                     prod_sn=1234)

    # Change the sn for this test
    meta2_out_path = patch_json_file(calc_path('same_ports.2.json'),
                                     tmp_path,
                                     prod_sn=5678)

    # new interface
    from ouster.sdk.client import MultiScanSource
    from ouster.sdk.pcap import PcapScanSource  # type: ignore

    scan_source: MultiScanSource
    scan_source = PcapScanSource(file_path,
                                 meta=[meta1_out_path, meta2_out_path],
                                 cycle=False)

    test_scans = list(scan_source)[0]

    assert test_scans[0] is None

    if not test_scans[1]:
        assert False

    assert test_scans[1].frame_id == 883
    assert test_scans[1].h == 64


def test_multiple_scan_source_pcap_both_legacy(tmp_path) -> None:
    file_path = calc_path('same_ports_legacy.pcap')

    # new interface
    from ouster.sdk.pcap import PcapScanSource, PcapDuplicatePortException  # type: ignore

    # Remove the imu conflict for this test
    meta1_out_path = patch_json_file(calc_path('same_ports_legacy.1.json'),
                                     tmp_path,
                                     imu_port=7503)
    meta2_out_path = calc_path('same_ports_legacy.2.json')

    with pytest.raises(PcapDuplicatePortException):
        _ = PcapScanSource(file_path,
                           meta=[meta1_out_path, meta2_out_path],
                           cycle=False)


def test_multiple_scan_source_pcap_single_legacy_with_imu(tmp_path) -> None:
    file_path = calc_path('same_ports_legacy.pcap')

    # new interface
    from ouster.sdk.client import MultiScanSource
    from ouster.sdk.pcap import PcapScanSource  # type: ignore

    # Remove the imu conflict for this test
    # Remove the second lidar for this test
    meta1_out_path = patch_json_file(calc_path('same_ports_legacy.1.json'),
                                     tmp_path,
                                     imu_port=7503,
                                     lidar_port=7503)

    meta2_out_path = calc_path('same_ports_legacy.2.json')

    scan_source: MultiScanSource
    scan_source = PcapScanSource(file_path,
                                 meta=[meta1_out_path, meta2_out_path],
                                 cycle=False)
    test_scans = list(scan_source)[0]

    if not test_scans[1]:
        assert False

    assert test_scans[1].frame_id == 185
    assert test_scans[1].h == 64


def test_multiple_scan_source_pcap_both_non_legacy(tmp_path) -> None:
    file_path = calc_path('same_ports_nonlegacy.pcap')

    # new interface
    from ouster.sdk.client import MultiScanSource
    from ouster.sdk.pcap import PcapScanSource  # type: ignore

    # Remove the imu conflict for this test
    meta1_out_path = patch_json_file(calc_path('same_ports_nonlegacy.1.json'),
                                     tmp_path,
                                     imu_port=7503)
    meta2_out_path = calc_path('same_ports_nonlegacy.2.json')

    scan_source: MultiScanSource
    scan_source = PcapScanSource(file_path,
                                 meta=[meta1_out_path, meta2_out_path],
                                 cycle=False)

    test_scans = list(scan_source)[0]

    if not test_scans[1] or not test_scans[0]:
        assert False

    assert test_scans[0].frame_id == 1093
    assert test_scans[0].h == 128

    assert test_scans[1].frame_id == 1535
    assert test_scans[1].h == 64

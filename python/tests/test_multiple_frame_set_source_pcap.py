"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.
"""

import os
import json
import pytest

from tests.conftest import PCAPS_DATA_DIR


def patch_json_file(file_in, tmp_path, imu_port=None,
                    lidar_port=None, prod_sn=None, zm_port=None):
    result_out = os.path.join(tmp_path, os.path.basename(file_in))
    meta = json.loads(open(file_in, 'r').read())

    if imu_port is not None:
        meta['config_params']['udp_port_imu'] = imu_port
    if lidar_port is not None:
        meta['config_params']['udp_port_lidar'] = lidar_port
    if prod_sn is not None:
        meta['sensor_info']['prod_sn'] = prod_sn
    if zm_port is not None:
        meta['config_params']['udp_port_zm'] = zm_port

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
def test_multiple_frame_set_source_imu_collsion(tmp_path, path):
    path = calc_path(path)
    file_path = path + ".pcap"

    # new interface
    from ouster.sdk.pcap import PcapFrameSetSource, PcapDuplicatePortException  # type: ignore

    with pytest.raises(PcapDuplicatePortException):
        _ = PcapFrameSetSource(file_path,
                           meta=[path + ".1.json", path + ".2.json"])


def test_multiple_frame_set_source_pcap(tmp_path) -> None:
    file_path = calc_path('same_ports.pcap')

    # Remove the imu conflict for this test
    meta1_out_path = patch_json_file(calc_path('same_ports.1.json'),
                                     tmp_path,
                                     imu_port=7503)
    meta2_out_path = calc_path('same_ports.2.json')

    # new interface
    from ouster.sdk.core import FrameSetSource, collate
    from ouster.sdk.pcap import PcapFrameSetSource  # type: ignore

    frame_set_source: FrameSetSource
    frame_set_source = PcapFrameSetSource(file_path,
                                 meta=[meta1_out_path, meta2_out_path])

    test_frames = list(iter(collate(frame_set_source)))[0]

    from ouster.sdk.core import LidarFrame
    assert isinstance(test_frames[0], LidarFrame)
    assert isinstance(test_frames[1], LidarFrame)
    assert test_frames[0].sensor_info == frame_set_source.sensor_info[0]

    assert test_frames[1].frame_id == 883
    assert test_frames[1].h == 64

    assert test_frames[0].frame_id == 1542
    assert test_frames[0].h == 128


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
    from ouster.sdk.core import FrameSetSource, collate
    from ouster.sdk.pcap import PcapFrameSetSource  # type: ignore

    frame_set_source: FrameSetSource
    frame_set_source = PcapFrameSetSource(file_path,
                                 meta=[meta1_out_path, meta2_out_path])

    test_frames = list(iter(collate(frame_set_source)))[0]

    assert test_frames[0] is None

    from ouster.sdk.core import LidarFrame
    assert isinstance(test_frames[1], LidarFrame)

    assert test_frames[1].frame_id == 883
    assert test_frames[1].h == 64


def test_multiple_frame_set_source_pcap_both_legacy(tmp_path) -> None:
    file_path = calc_path('same_ports_legacy.pcap')

    # new interface
    from ouster.sdk.pcap import PcapFrameSetSource, PcapDuplicatePortException  # type: ignore

    # Remove the imu conflict for this test
    meta1_out_path = patch_json_file(calc_path('same_ports_legacy.1.json'),
                                     tmp_path,
                                     imu_port=7503)
    meta2_out_path = calc_path('same_ports_legacy.2.json')

    with pytest.raises(PcapDuplicatePortException):
        _ = PcapFrameSetSource(file_path,
                           meta=[meta1_out_path, meta2_out_path])


def test_multiple_frame_set_source_pcap_single_legacy_with_imu(tmp_path) -> None:
    file_path = calc_path('same_ports_legacy.pcap')

    # new interface
    from ouster.sdk.core import FrameSetSource, collate
    from ouster.sdk.pcap import PcapFrameSetSource  # type: ignore

    # Remove the imu conflict for this test
    # Remove the second lidar for this test
    meta1_out_path = patch_json_file(calc_path('same_ports_legacy.1.json'),
                                     tmp_path,
                                     imu_port=7503,
                                     lidar_port=7503)

    meta2_out_path = calc_path('same_ports_legacy.2.json')

    frame_set_source: FrameSetSource
    frame_set_source = PcapFrameSetSource(file_path,
                                 meta=[meta1_out_path, meta2_out_path])
    test_frames = list(iter(collate(frame_set_source)))[0]

    from ouster.sdk.core import LidarFrame
    assert isinstance(test_frames[1], LidarFrame)

    assert test_frames[1].frame_id == 185
    assert test_frames[1].h == 64


def test_multiple_frame_set_source_pcap_both_non_legacy(tmp_path) -> None:
    file_path = calc_path('same_ports_nonlegacy.pcap')

    # new interface
    from ouster.sdk.core import FrameSetSource, collate
    from ouster.sdk.pcap import PcapFrameSetSource  # type: ignore

    # Remove the imu conflict for this test
    meta1_out_path = patch_json_file(calc_path('same_ports_nonlegacy.1.json'),
                                     tmp_path,
                                     imu_port=7503)
    meta2_out_path = calc_path('same_ports_nonlegacy.2.json')

    frame_set_source: FrameSetSource
    frame_set_source = PcapFrameSetSource(file_path,
                                 meta=[meta1_out_path, meta2_out_path])
    test_frames = list(iter(collate(frame_set_source)))[0]
    from ouster.sdk.core import LidarFrame
    assert isinstance(test_frames[0], LidarFrame)
    assert isinstance(test_frames[1], LidarFrame)

    assert test_frames[0].frame_id == 1093
    assert test_frames[0].h == 128

    assert test_frames[1].frame_id == 1535
    assert test_frames[1].h == 64


def test_single_sensor_lidar_and_zm_share_port(tmp_path) -> None:
    """One sensor may send lidar and zone monitoring to the same UDP port."""
    file_path = calc_path('same_ports_nonlegacy.pcap')
    meta_in = calc_path('same_ports_nonlegacy.1.json')
    lidar_port = json.loads(open(meta_in).read())['config_params']['udp_port_lidar']
    meta_path = patch_json_file(meta_in, tmp_path, zm_port=lidar_port)

    from ouster.sdk.core import FrameSetSource, collate, LidarFrame
    from ouster.sdk.pcap import PcapFrameSetSource

    frame_set_source: FrameSetSource
    frame_set_source = PcapFrameSetSource(file_path, meta=[meta_path])
    test_frames = list(iter(collate(frame_set_source)))[0]
    assert isinstance(test_frames[0], LidarFrame)
    assert test_frames[0].frame_id == 1093
    assert test_frames[0].h == 128


def test_two_sensors_same_zm_port_and_sn(tmp_path) -> None:
    """Distinct sensors that share a zone-monitoring port and serial still conflict."""
    file_path = calc_path('same_ports_nonlegacy.pcap')
    meta1_in = calc_path('same_ports_nonlegacy.1.json')
    sn = json.loads(open(meta1_in).read())['sensor_info']['prod_sn']
    meta1_path = patch_json_file(meta1_in, tmp_path, zm_port=7504)
    meta2_path = patch_json_file(calc_path('same_ports_nonlegacy.2.json'),
                                 tmp_path,
                                 lidar_port=7505,
                                 imu_port=7506,
                                 zm_port=7504,
                                 prod_sn=sn)

    from ouster.sdk.pcap import PcapFrameSetSource, PcapDuplicatePortException

    with pytest.raises(PcapDuplicatePortException):
        _ = PcapFrameSetSource(file_path, meta=[meta1_path, meta2_path])

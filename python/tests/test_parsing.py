# type: ignore
import os
import numpy as np
import ouster.pcap._pcap as _pcap
from ouster.client import LidarMode, SensorInfo, UDPProfileLidar, ChanField
import ouster.client as client
import ouster.pcap as pcap
from ouster.sdkx.parsing import FusaDualFormat, PacketFormat, default_scan_fields
from tests.conftest import PCAPS_DATA_DIR
from ouster.sdk.util import resolve_metadata


def test_fusa_parsing_profile():
    dataset_name = 'OS-1-128_767798045_1024x10_20230712_120049'
    meta = open(os.path.join(PCAPS_DATA_DIR, f'{dataset_name}.json')).read()
    si = SensorInfo(meta)
    packet_format = PacketFormat.from_metadata(si)
    assert type(packet_format) is FusaDualFormat


def test_fusa_fields():
    dataset_name = 'OS-1-128_767798045_1024x10_20230712_120049'
    meta = open(os.path.join(PCAPS_DATA_DIR, f'{dataset_name}.json')).read()
    si = SensorInfo(meta)
    pcap = _pcap.replay_initialize(os.path.join(PCAPS_DATA_DIR, f'{dataset_name}.pcap'))
    packet_format = PacketFormat.from_metadata(si)

    # check preconditions
    assert si.format.columns_per_packet == 16
    assert si.format.pixels_per_column == 128
    # Note - because `def_enum` adds a prefix to all symbols in an enum, the
    # profile symbol name doesn't match its C++ equivalent :-/
    assert si.format.udp_profile_lidar == UDPProfileLidar.PROFILE_LIDAR_FUSA_RNG15_RFL8_NIR8_DUAL
    assert si.mode == LidarMode.MODE_1024x10

    buf = bytearray(2**16)
    packet_info = _pcap.packet_info()
    _pcap.next_packet_info(pcap, packet_info)
    _pcap.read_packet(pcap, buf)
    arr = np.frombuffer(buf, dtype=np.uint8)
    assert packet_format.packet_type(arr) == 1
    assert packet_format.frame_id(arr) == 229
    assert packet_format.init_id(arr) == si.init_id
    assert str(packet_format.prod_sn(arr)) == si.sn
    # assert packet_format.countdown_thermal_shutdown(arr) == 0   # Note: doesn't exist in python iface
    # assert packet_format.countdown_shot_limiting(arr) == 0   # Note: doesn't exist in python iface
    # assert packet_format.thermal_shutdown(arr) == 0  # Note: doesn't exist in python iface
    # assert packet_format.shot_limiting(arr) == 0  # Note: doesn't exist in python iface

    # check column header values (from cpp tests)
    '''
    col_timestamps = [
        647839983424,
        647840089656,
        647840187456,
        647840275096,
        647840379896,
        647840469616,
        647840567576,
        647840675576,
        647840768056,
        647840861408,
        647840965048,
        647841059888,
        647841161128,
        647841259968,
        647841351568,
        647841450008,
    ]

    for col in range(si.format.columns_per_packet):
        pass
    '''


def test_fusa_reading_pcap():
    """Check that we can read pcap with FLAGS included."""
    dataset_name = 'OS-1-128_767798045_1024x10_20230712_120049'
    pcap_name = os.path.join(PCAPS_DATA_DIR, f'{dataset_name}.pcap')
    meta = open(resolve_metadata(pcap_name)).read()
    si = SensorInfo(meta)
    source = pcap.Pcap(pcap_name, si)
    field_types = default_scan_fields(source.metadata.format.udp_profile_lidar,
                                      flags=True)
    scans = list(client.Scans(source, fields=field_types))
    assert len(scans) == 1

    ls = scans[0]
    assert np.count_nonzero(ls.field(ChanField.FLAGS)) == 8
    assert np.count_nonzero(ls.field(ChanField.FLAGS2)) == 0

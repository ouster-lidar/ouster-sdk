"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.
"""
import pytest
import numpy as np

from ouster.sdk import client
from ouster.sdk.client import ChanField
from ouster.sdk._bindings.client import FieldInfo
import ouster.sdk._bindings.client as _client


def test_create_field_info() -> None:
    fi = _client.FieldInfo(np.uint16, 1, 0xdeadbeef, 3)

    assert fi.ty_tag == np.dtype('uint16')
    assert fi.offset == 1
    assert fi.mask == 0xdeadbeef
    assert fi.shift == 3


def test_add_custom_profile() -> None:
    profile_nr = 1000
    profile_name = "CUSTOM_PROF"
    fields = [
        (ChanField.RANGE, FieldInfo(np.uint16, 0, 0xdeadbeef, 0)),
        (ChanField.REFLECTIVITY, FieldInfo(np.uint16, 1, 0xff, 2))
    ]
    chan_data_size = 16

    _client.add_custom_profile(profile_nr, profile_name, fields, chan_data_size)

    assert str(_client.UDPProfileLidar(1000)) == profile_name


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_custom_copycat_profile_matches_original(packets: client.PacketSource) -> None:
    """Check that custom profile parses the same as original with the same fields"""
    custom_fields = [
        (ChanField.RANGE, FieldInfo(np.uint32, 0, 0x0007ffff, 0)),
        (ChanField.REFLECTIVITY, FieldInfo(np.uint8, 3, 0, 0)),
        (ChanField.RANGE2, FieldInfo(np.uint32, 4, 0x0007ffff, 0)),
        (ChanField.REFLECTIVITY2, FieldInfo(np.uint8, 7, 0, 0)),
        (ChanField.SIGNAL, FieldInfo(np.uint16, 8, 0, 0)),
        (ChanField.SIGNAL2, FieldInfo(np.uint16, 10, 0, 0)),
        (ChanField.NEAR_IR, FieldInfo(np.uint16, 12, 0, 0)),
        (ChanField.FLAGS2, FieldInfo(np.uint8, 6, 0b11111000, 3)),
        (ChanField.FLAGS, FieldInfo(np.uint8, 2, 0b11111000, 3)),
    ]

    _client.add_custom_profile(11, "DUAL_COPYCAT", custom_fields, 16)

    ls_orig = next(iter(client.Scans(packets)))
    packets.metadata.format.udp_profile_lidar = _client.UDPProfileLidar(11)
    ls_custom = next(iter(client.Scans(packets)))

    # all fields in ls_orig should have corresponding ls_custom fields
    for f in ls_orig.fields:
        assert np.all(ls_orig.field(f) == ls_custom.field(f))

    assert len([x for x in ls_custom.fields]) == len([x for x in ls_orig.fields])

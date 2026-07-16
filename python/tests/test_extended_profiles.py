"""
Copyright (c) 2023, Ouster, Inc.
All rights reserved.
"""
import pytest
import numpy as np

from ouster.sdk import core
from ouster.sdk.core import (ChanField, FieldDecodeInfo, add_custom_profile,
                             UDPProfileLidar)
from tests.multi import Frames


def test_create_field_info() -> None:
    fi = FieldDecodeInfo(np.uint16, 1, 0xdeadbeef, 3)

    assert fi.ty_tag == np.dtype('uint16')
    assert fi.offset == 1
    assert fi.mask == 0xdeadbeef
    assert fi.shift == 3


def test_add_custom_profile() -> None:
    profile_name = "CUSTOM_PROF"
    fields = [
        (ChanField.RANGE, FieldDecodeInfo(np.uint16, 0, 0xdeadbeef, 0)),
        (ChanField.REFLECTIVITY, FieldDecodeInfo(np.uint16, 1, 0xff, 2))
    ]
    chan_data_size = 16

    profile_nr = add_custom_profile(profile_name, fields, chan_data_size)

    assert str(UDPProfileLidar(profile_nr)) == profile_name


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_custom_copycat_profile_matches_original(packets: core.PacketSource) -> None:
    """Check that custom profile parses the same as original with the same fields"""
    custom_fields = [
        (ChanField.RANGE, FieldDecodeInfo(np.uint32, 0, 0x0007ffff, 0)),
        (ChanField.REFLECTIVITY, FieldDecodeInfo(np.uint8, 3, 0, 0)),
        (ChanField.RANGE2, FieldDecodeInfo(np.uint32, 4, 0x0007ffff, 0)),
        (ChanField.REFLECTIVITY2, FieldDecodeInfo(np.uint8, 7, 0, 0)),
        (ChanField.SIGNAL, FieldDecodeInfo(np.uint16, 8, 0, 0)),
        (ChanField.SIGNAL2, FieldDecodeInfo(np.uint16, 10, 0, 0)),
        (ChanField.NEAR_IR, FieldDecodeInfo(np.uint16, 12, 0, 0)),
        (ChanField.FLAGS2, FieldDecodeInfo(np.uint8, 6, 0b11111000, 3)),
        (ChanField.FLAGS, FieldDecodeInfo(np.uint8, 2, 0b11111000, 3)),
        (ChanField.WINDOW, FieldDecodeInfo(np.uint8, 15, 0, 0)),
    ]

    profile_nr = add_custom_profile("DUAL_COPYCAT", custom_fields, 16)

    ls_orig = next(iter(Frames(packets)))[0]
    assert ls_orig is not None
    packets.sensor_info[0].format.udp_profile_lidar = UDPProfileLidar(profile_nr)
    ls_custom = next(iter(Frames(packets)))[0]
    assert ls_custom is not None

    # all fields in ls_orig should have corresponding ls_custom fields
    for f in ls_orig.fields:
        assert np.all(ls_orig.field(f) == ls_custom.field(f))

    assert len([x for x in ls_custom.fields]) == len([x for x in ls_orig.fields])

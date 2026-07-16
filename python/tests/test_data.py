"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Tests for lidar data parsing.

Checks that the output of parsing hasn't changed unexpectedly.
"""
from copy import deepcopy

import numpy as np
import pytest

from ouster.sdk import core
from ouster.sdk._bindings.client import frame_to_packets
from ouster.sdk.core import PacketValidationFailure, PacketFormat, FieldType, LidarFrame


def fake_sensor_info(h: int, w: int, profile = core.UDPProfileLidar.LEGACY):
    info = core.SensorInfo()
    info.format.columns_per_packet = 16
    info.format.pixels_per_column = h
    info.format.columns_per_frame = w
    info.format.udp_profile_lidar = profile
    info.format.udp_profile_imu = core.UDPProfileIMU.LEGACY
    info.image_rev = "3.2.1"
    info.fw_rev = "3.2.1"
    return info


def test_make_packets(meta: core.SensorInfo) -> None:
    pf = PacketFormat.from_info(meta)

    p1 = core.ImuPacket(pf)
    p2 = core.LidarPacket(pf)
    p3 = core.ZonePacket(pf)

    assert p1.validate(meta) == PacketValidationFailure.NONE
    assert p2.validate(meta) == PacketValidationFailure.NONE
    assert p3.validate(meta) == PacketValidationFailure.NONE


def test_imu_packet(meta: core.SensorInfo) -> None:
    pf = PacketFormat.from_info(meta)

    p = core.ImuPacket(pf)

    assert pf.imu_sys_ts(p.buf) == 0
    assert pf.imu_accel_ts(p.buf) == 0
    assert pf.imu_gyro_ts(p.buf) == 0
    assert pf.imu_av_x(p.buf) == 0.0
    assert pf.imu_av_y(p.buf) == 0.0
    assert pf.imu_av_z(p.buf) == 0.0
    assert pf.imu_la_x(p.buf) == 0.0
    assert pf.imu_la_y(p.buf) == 0.0
    assert pf.imu_la_z(p.buf) == 0.0


def test_lidar_packet(meta: core.SensorInfo) -> None:
    """Test reading and writing values from empty packets."""
    pf = PacketFormat(meta)
    p = core.LidarPacket(pf)
    w = pf.columns_per_packet
    h = pf.pixels_per_column

    frame_has_signal = (meta.format.udp_profile_lidar !=
                       core.UDPProfileLidar.RNG15_RFL8_NIR8)

    assert np.array_equal(pf.packet_field(core.ChanField.RANGE, p.buf), np.zeros((h, w)))
    assert np.array_equal(pf.packet_field(core.ChanField.REFLECTIVITY, p.buf),
                          np.zeros((h, w)))
    assert np.array_equal(pf.packet_field(core.ChanField.NEAR_IR, p.buf), np.zeros((h, w)))

    if frame_has_signal:
        assert np.array_equal(pf.packet_field(core.ChanField.SIGNAL, p.buf), np.zeros(
            (h, w)))

    assert len(
        core.ColHeader.__members__) == 5, "Don't forget to update tests!"
    assert np.array_equal(pf.packet_header(core.ColHeader.TIMESTAMP, p.buf), np.zeros(w))
    assert np.array_equal(pf.packet_header(core.ColHeader.MEASUREMENT_ID, p.buf), np.zeros(w))
    assert np.array_equal(pf.packet_header(core.ColHeader.STATUS, p.buf), np.zeros(w))

    assert pf.frame_id(p.buf) == 0


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_read_legacy_packet(packet: core.LidarPacket, packets: core.PacketSource) -> None:
    """Read some arbitrary values from a packet and check header invariants."""
    pf = core.PacketFormat(packets.sensor_info[0])
    assert pf.packet_field(core.ChanField.RANGE, packet.buf)[-1, 0] == 12099
    assert pf.packet_field(core.ChanField.REFLECTIVITY, packet.buf)[-1, 0] == 249
    assert pf.packet_field(core.ChanField.SIGNAL, packet.buf)[-1, 0] == 6
    assert pf.packet_field(core.ChanField.NEAR_IR, packet.buf)[-1, 0] == 13

    assert np.all(np.diff(pf.packet_header(core.ColHeader.TIMESTAMP, packet.buf)) > 0)
    assert np.all(np.diff(pf.packet_header(core.ColHeader.MEASUREMENT_ID, packet.buf)) == 1)
    assert pf.packet_type(packet.buf) == 0
    assert pf.frame_id(packet.buf) == 5424
    assert pf.init_id(packet.buf) == 0
    assert pf.prod_sn(packet.buf) == 0
    assert pf.shot_limiting(packet.buf) == core.ShotLimitingStatus.NORMAL
    assert pf.thermal_shutdown(packet.buf) == core.ThermalShutdownStatus.NORMAL
    # in 1024xN mode, the angle between measurements is exactly 88 encoder ticks
    assert np.all(pf.packet_header(core.ColHeader.STATUS, packet.buf) == 0xffffffff)


@pytest.mark.parametrize('test_key', ['single-2.3'])
def test_read_single_return_packet(packet: core.LidarPacket, packets: core.PacketSource) -> None:
    """Read some arbitrary values from packet and check header invariants."""
    pf = core.PacketFormat(packets.sensor_info[0])
    assert pf.packet_field(core.ChanField.RANGE, packet.buf)[-1, 0] == 11610
    assert pf.packet_field(core.ChanField.REFLECTIVITY, packet.buf)[-1, 0] == 11
    assert pf.packet_field(core.ChanField.SIGNAL, packet.buf)[-1, 0] == 34
    assert pf.packet_field(core.ChanField.NEAR_IR, packet.buf)[-1, 0] == 393

    assert np.all(np.diff(pf.packet_header(core.ColHeader.TIMESTAMP, packet.buf)) > 0)
    assert np.all(np.diff(pf.packet_header(core.ColHeader.MEASUREMENT_ID, packet.buf)) == 1)
    assert pf.packet_type(packet.buf) == 1
    assert pf.frame_id(packet.buf) == 1259
    assert pf.init_id(packet.buf) == 5431293
    assert pf.prod_sn(packet.buf) == 992210000957
    assert pf.shot_limiting(packet.buf) == core.ShotLimitingStatus.NORMAL
    assert pf.thermal_shutdown(packet.buf) == core.ThermalShutdownStatus.NORMAL

    # Changes from LEGACY
    assert np.all(pf.packet_header(core.ColHeader.STATUS, packet.buf) == 0x01)


def test_lidar_frame_init() -> None:
    """If kwargs are used, they should set the frame shape correctly."""
    w, h = 1024, 128
    assert core.LidarFrame(h, w, [], 16).w == w
    assert core.LidarFrame(h, w, [], 16).h == h
    assert core.LidarFrame(w=w, h=h, field_types=[], columns_per_packet=16).w == w
    assert core.LidarFrame(w=w, h=h, field_types=[], columns_per_packet=16).h == h
    assert core.LidarFrame(h=h, w=w, field_types=[], columns_per_packet=16).w == w
    assert core.LidarFrame(h=h, w=w, field_types=[], columns_per_packet=16).h == h
    with pytest.raises(ValueError, match='Cannot construct LidarFrame with zero width or height'):
        core.LidarFrame(0, 10, [], 16)
    with pytest.raises(ValueError, match='columns_per_packet must be greater than 0'):
        core.LidarFrame(10, 10, [], 0)


def test_frame_writeable() -> None:
    """Check that a native frame is a writeable view of data."""
    ls = core.LidarFrame(fake_sensor_info(1024, 32))

    assert not ls.field(core.ChanField.RANGE).flags.owndata
    assert not ls.status.flags.owndata

    assert ls.field(core.ChanField.SIGNAL).flags.aligned
    assert ls.measurement_id.flags.aligned

    assert ls.field(core.ChanField.NEAR_IR).flags.aligned
    assert ls.timestamp.flags.aligned

    ls.field(core.ChanField.RANGE)[0, 0] = 42
    assert ls.field(core.ChanField.RANGE)[0, 0] == 42

    ls.field(core.ChanField.RANGE)[:] = 7
    assert np.all(ls.field(core.ChanField.RANGE) == 7)

    ls.status[-1] = 0xffff
    assert ls.status[-1] == 0xffff

    ls.status[:] = 0x1
    assert np.all(ls.status == 0x1)

    assert np.all(ls.body_to_world == np.eye(4))

    ls.body_to_world[1][0, 2] = 8
    assert np.all(ls.body_to_world[1] == np.array([[1, 0, 8, 0], [0, 1, 0, 0],
                                          [0, 0, 1, 0], [0, 0, 0, 1]]))


def test_frame_field_ref() -> None:
    """Test that field references keep frames alive."""

    ls = core.LidarFrame(fake_sensor_info(512, 16))
    range = ls.field(core.ChanField.RANGE)
    range[:] = 42

    del ls
    assert np.all(range == 42)

    range[:] = 43
    assert np.all(range == 43)


@pytest.mark.parametrize(
    'field',
    ['status', "timestamp", "alert_flags", "body_to_world",
     "measurement_id", "packet_timestamp"])
def test_frame_header_ref(field: str) -> None:
    """Test that header references keep frames alive."""

    ls = core.LidarFrame(512, 16, [], 16)
    data = getattr(ls, field)
    data[:] = 0x11

    del ls

    assert np.all(data == 0x11)

    data[:] = 0x01
    assert np.all(data == 0x01)


def test_frame_not_complete() -> None:
    """Test that not all frames are considered complete."""
    ls = core.LidarFrame(fake_sensor_info(32, 1024))
    ls.sensor_info = None  # type: ignore

    status = ls.status
    # trying complete with no arguments should fail if no sensor info is provided
    with pytest.raises(RuntimeError):
        ls.complete()
    assert not ls.complete((0, 1023))

    status[0] = 0x02
    assert not ls.complete((0, 1023))
    assert not ls.complete((0, 0))

    status[1:] = 0xFFFFFFFF
    assert not ls.complete((0, 1023))

    status[:] = 0xFFFFFFFF
    status[-1] = 0x02
    assert not ls.complete((0, 1023))

    # windows are inclusive but python slicing is not
    status[:] = 0x00
    status[:10] = 0xFFFFFFFF
    assert not ls.complete((0, 10))

    status[:] = 0x00
    status[11:21] = 0xFFFFFFFF
    assert not ls.complete((10, 20))

    # window [i, i]
    status[:] = 0x00
    status[0] = 0xFFFFFFFF
    assert not ls.complete((0, 1023))
    assert not ls.complete((0, 1))
    assert ls.complete((0, 0))

    status[:] = 0x00
    status[128] = 0xFFFFFFFF
    assert not ls.complete((0, 1023))
    assert not ls.complete((127, 128))
    assert ls.complete((128, 128))

    # test window start > end and ensure it is properly inclusive
    status[:] = 1
    assert ls.complete((128, 126))
    status[127] = 0
    assert ls.complete((128, 126))
    status[126] = 0
    assert not ls.complete((128, 126))
    status[126] = 1
    status[128] = 0
    assert not ls.complete((128, 126))


# TODO: Add 4096 to this test
@pytest.mark.parametrize("w, win_start, win_end", [
    (512, 0, 511),
    (512, 1, 0),
    (512, 256, 0),
    (512, 256, 1),
    (1024, 0, 1023),
    (1024, 0, 512),
    (1024, 0, 0),
    (1024, 1023, 1023),
    (1024, 1023, 0),
    (1024, 1023, 1),
    (2048, 0, 2047),
    (2048, 1024, 512),
    (2048, 1024, 0),
    (2048, 1024, 1),
    (2048, 511, 511),
])
def test_frame_complete(w, win_start, win_end) -> None:
    """Set the status headers to the specified window and check complete()."""
    ls = core.LidarFrame(32, w, [], 16)

    status = ls.status

    if win_start <= win_end:
        status[win_start:win_end + 1] = 0xFFFFFFFF
    else:
        status[0:win_end + 1] = 0xFFFFFFFF
        status[win_start:] = 0xFFFFFFFF

    assert ls.complete((win_start, win_end))


def test_frame_fields_ref() -> None:
    """Make sure ref to fields keeps frame alive."""
    fields = core.LidarFrame(fake_sensor_info(32, 1024)).fields

    # should fail (or trip asan) if the field iterator doesn't keep frame alive
    assert set(fields) == {
        core.ChanField.RANGE,
        core.ChanField.REFLECTIVITY,
        core.ChanField.SIGNAL,
        core.ChanField.NEAR_IR,
        core.ChanField.FLAGS,
    }


def test_frame_default_fields() -> None:
    """Default frame has the expected fields for the LEGACY profile."""
    ls = core.LidarFrame(fake_sensor_info(32, 1024))

    assert set(ls.fields) == {
        core.ChanField.RANGE,
        core.ChanField.REFLECTIVITY,
        core.ChanField.SIGNAL,
        core.ChanField.NEAR_IR,
        core.ChanField.FLAGS,
    }

    for f in ls.fields:
        if f == core.ChanField.FLAGS or f == core.ChanField.REFLECTIVITY:
            assert ls.field(f).dtype == np.uint8
        elif f == core.ChanField.RANGE:
            assert ls.field(f).dtype == np.uint32
        else:
            assert ls.field(f).dtype == np.uint16


def test_frame_dual_profile() -> None:
    """Dual returns frame has the expected fields."""
    ls = core.LidarFrame(fake_sensor_info(
        32, 1024,
        core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL))

    assert set(ls.fields) == {
        core.ChanField.RANGE,
        core.ChanField.RANGE2,
        core.ChanField.REFLECTIVITY,
        core.ChanField.REFLECTIVITY2,
        core.ChanField.SIGNAL,
        core.ChanField.SIGNAL2,
        core.ChanField.FLAGS,
        core.ChanField.FLAGS2,
        core.ChanField.NEAR_IR,
        core.ChanField.WINDOW,
    }


def test_frame_low_data_rate() -> None:
    """Low Data Rate frame has the expected fields."""
    ls = core.LidarFrame(fake_sensor_info(32, 1024,
                        core.UDPProfileLidar.RNG15_RFL8_NIR8))

    assert set(ls.fields) == {
        core.ChanField.RANGE,
        core.ChanField.REFLECTIVITY,
        core.ChanField.NEAR_IR,
        core.ChanField.FLAGS,
    }


def test_frame_single_return() -> None:
    """Single Return frame has the expected fields."""
    ls = core.LidarFrame(fake_sensor_info(
        32, 1024, core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16))

    assert set(ls.fields) == {
        core.ChanField.RANGE,
        core.ChanField.REFLECTIVITY,
        core.ChanField.SIGNAL,
        core.ChanField.NEAR_IR,
        core.ChanField.FLAGS,
        core.ChanField.WINDOW,
    }


def test_frame_empty() -> None:
    """Sanity check frame with no fields."""
    ls = core.LidarFrame(32, 1024, [], 16)
    assert ls.fields == []


def test_frame_custom() -> None:
    """Sanity check frame with a custom set of fields."""
    ls = core.LidarFrame(
        32, 1024, [
            FieldType(core.ChanField.SIGNAL, np.uint16),
            FieldType(core.ChanField.FLAGS, np.uint8),
            FieldType("custom0", np.uint32)
        ], 16)

    assert set(ls.fields) == {
        core.ChanField.SIGNAL, core.ChanField.FLAGS,
        "custom0"
    }
    assert ls.field(core.ChanField.SIGNAL).dtype == np.uint16
    assert ls.field("custom0").dtype == np.uint32

    with pytest.raises(IndexError):
        ls.field(core.ChanField.RANGE)


def test_frame_eq_fields() -> None:
    """Test equality between frames with different fields."""
    ls0 = core.LidarFrame(fake_sensor_info(32, 1024))
    ls1 = core.LidarFrame(fake_sensor_info(32, 1024))
    ls2 = core.LidarFrame(fake_sensor_info(
        32, 1024,
        core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL))
    ls3 = core.LidarFrame(32, 1024, [FieldType(core.ChanField.SIGNAL, np.uint32)], 16)
    ls4 = core.LidarFrame(32, 1024, [FieldType(core.ChanField.SIGNAL, np.uint16)], 16)
    ls5 = core.LidarFrame(32, 1024, [], 16)

    assert ls0 == ls1
    assert not (ls0 != ls1)  # should be implemented using __eq__
    assert ls1 != ls2
    assert ls3 != ls4
    assert ls5 != ls0
    assert ls5 != ls2
    assert ls5 != ls4


def test_frame_zero_init() -> None:
    """Test that frame fields and headers are zero initialized."""
    ls = core.LidarFrame(fake_sensor_info(
        64, 1024,
        core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL))

    assert ls.frame_id == -1
    assert ls.frame_status == 0

    assert np.count_nonzero(ls.timestamp) == 0
    assert np.count_nonzero(ls.measurement_id) == 0
    assert np.count_nonzero(ls.status) == 0

    for f in ls.fields:
        assert np.count_nonzero(ls.field(f)) == 0


def test_frame_copy_eq() -> None:
    """Test equality with a copy."""

    ls0 = core.LidarFrame(fake_sensor_info(32, 512))
    ls0.status[:] = 0x1
    ls0.field(core.ChanField.REFLECTIVITY)[:] = 100
    ls0.body_to_world[:, 0, 3] = 8

    ls1 = deepcopy(ls0)

    assert ls0 is not ls1
    assert ls0 == ls1

    ls0.frame_id = 9
    assert ls0 != ls1

    ls1.frame_id = 9
    assert ls0 == ls1

    ls0.frame_status = 1
    assert ls0 != ls1

    ls1.frame_status = 1
    assert ls0 == ls1

    ls0.measurement_id[0] = 1
    assert ls0 != ls1

    ls1.measurement_id[0] = 1
    assert ls0 == ls1

    ls0.field(core.ChanField.RANGE)[0, 0] = 42
    assert ls0 != ls1

    ls1.field(core.ChanField.RANGE)[0, 0] = 42
    assert ls0 == ls1

    ls0.body_to_world[1] = np.eye(4)
    assert ls0 != ls1

    ls0.body_to_world[1, 0, 3] = 8
    assert ls0 == ls1


def test_frame_eq_with_custom_fields() -> None:
    """Test equality with custom fields."""

    ls0 = core.LidarFrame(32, 512, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ], 16)

    ls1 = deepcopy(ls0)

    ls0.field("custom0")[:] = 100

    ls2 = deepcopy(ls0)

    assert np.count_nonzero(
        ls2.field("custom0") == 100) == ls0.h * ls0.w
    assert np.count_nonzero(ls2.field("custom4") == 100) == 0

    assert ls1 is not ls0
    assert ls1 != ls0
    assert ls2 == ls0


def test_frame_copy_extension() -> None:
    """ Verify we can clone a frame and null pad missing desired fields """
    ls0 = core.LidarFrame(32, 512, [
        FieldType("custom4", np.uint8)
    ], 16)

    ls0.field("custom4")[:] = 123

    ls1 = core.LidarFrame(ls0, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ])

    assert len(list(ls1.fields)) == 2, ls1.fields
    assert np.count_nonzero(ls1.field("custom0")[0, 0]) == 0
    assert np.count_nonzero(
        ls1.field("custom4") == 123) == ls1.h * ls1.w


def test_frame_copy_retraction() -> None:
    """ Verify we can clone a frame and remove undesired fields """
    ls0 = core.LidarFrame(32, 512, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ], 16)

    ls0.field("custom0")[:] = 100
    ls0.field("custom4")[:] = 123

    ls1 = core.LidarFrame(ls0, [
        FieldType("custom0", np.uint32),
    ])

    assert ls0.h == ls1.h
    assert ls0.w == ls1.w

    assert len(list(ls1.fields)) == 1
    assert np.count_nonzero(
        ls1.field("custom0") == 100) == ls1.h * ls1.w
    with pytest.raises(IndexError):
        ls1.field("custom4")[0, 0] == 100


def test_frame_copy_cast() -> None:
    """ Verify we can clone a frame and cast between field types """
    ls0 = core.LidarFrame(32, 512, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ], 16)

    ls0.field("custom0")[:] = 2 ** 16 - 1
    ls0.field("custom4")[:] = 255

    ls1 = core.LidarFrame(ls0, [
        FieldType("custom0", np.uint8),
        FieldType("custom4", np.uint16)
    ])

    assert ls0.h == ls1.h
    assert ls0.w == ls1.w

    assert len(list(ls1.fields)) == 2
    assert ls1.field("custom0").dtype == np.uint8
    assert ls1.field("custom4").dtype == np.uint16
    assert np.count_nonzero(
        ls1.field("custom0") == 255) == ls1.h * ls1.w
    assert np.count_nonzero(
        ls1.field("custom4") == 255) == ls1.h * ls1.w


def test_frame_copy() -> None:
    ls0 = core.LidarFrame(32, 512, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ], 16)

    ls0.field("custom0")[:] = 100
    ls0.field("custom4")[:] = 123

    ls1 = core.LidarFrame(ls0)

    assert ls0.h == ls1.h
    assert ls0.w == ls1.w

    assert len(list(ls1.fields)) == 2
    assert np.count_nonzero(
        ls1.field("custom0") == 100) == ls1.h * ls1.w
    assert np.count_nonzero(
        ls1.field("custom4") == 123) == ls1.h * ls1.w


def test_packet_writer_bindings(meta: core.SensorInfo) -> None:
    pf = PacketFormat.from_info(meta)
    packet = core.LidarPacket(pf)

    pf.set_frame_id(packet, 700)
    assert pf.frame_id(packet.buf) == 700

    with pytest.raises(ValueError):
        pf.set_col_timestamp(packet, pf.columns_per_packet, 100)
    with pytest.raises(ValueError):
        pf.set_col_measurement_id(packet, pf.columns_per_packet, 100)
    with pytest.raises(ValueError):
        pf.set_col_status(packet, pf.columns_per_packet, 0x1)

    try:
        for i in range(pf.columns_per_packet):
            pf.set_col_timestamp(packet, i, 100)
            pf.set_col_status(packet, i, 0x1)
            pf.set_col_measurement_id(packet, i, 100)
    except ValueError:
        assert False, "setting cols up to columns_per_packet should not raise"

    for dt in [np.uint8, np.uint16, np.uint32, np.uint64]:
        p = core.LidarPacket(pf)
        for chan in pf.fields:
            if chan in [core.ChanField.RAW32_WORD1,
                        core.ChanField.RAW32_WORD2,
                        core.ChanField.RAW32_WORD3,
                        core.ChanField.RAW32_WORD4]:
                continue
            # mypy is going nuts with mask notation
            value_mask = pf.field_value_mask(chan) & np.iinfo(dt).max  # type: ignore
            shape = (pf.pixels_per_column, pf.columns_per_packet)
            assert value_mask > 0
            _max = max(value_mask, value_mask + 1)
            field = np.random.randint(_max, size=shape, dtype=dt)  # type: ignore
            field = field & value_mask  # type: ignore
            s = hex(value_mask)
            assert np.any(field > 0), f"{chan}, {dt}, {s}"
            pf.set_field(p, chan, field)  # type: ignore
            assert np.all(pf.packet_field(chan, p.buf) == field), f"{chan}, {dt}, {s}"

    columns_per_frame = meta.format.columns_per_frame
    ls = core.LidarFrame(meta)
    # all fields are invalid, expect zero packets
    packets = frame_to_packets(ls, pf, 0, 0)
    assert len(packets) == 0

    expected_packets = columns_per_frame / pf.columns_per_packet
    ls.status[:] = 0x1
    packets = frame_to_packets(ls, pf, 0, 0)
    assert len(packets) == expected_packets


def test_to_string_doesnt_cause_fp_exception():
    """It shouldn't crash with a floating point exception when std::to_string(LidarFrame&) is called."""
    str(core.LidarFrame(fake_sensor_info(1024, 128, core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL)))


def test_frame_float_double() -> None:
    """Test that we can add float fields and that setting floats in them works."""
    ls = core.LidarFrame(fake_sensor_info(
        64, 1024,
        core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL))

    ls.add_field("f32", np.float32, (), core.FieldClass.PIXEL_FIELD)
    ls.add_field("f64", np.float64, (), core.FieldClass.PIXEL_FIELD)

    ls.field("f32")[:] = 3.3
    ls.field("f64")[:] = 6.6

    assert ls.field("f32")[1, 1] == np.float32(3.3)
    assert ls.field("f64")[1, 1] == 6.6


def test_frame_int() -> None:
    """Test that we can add int fields and that setting ints in them works."""
    ls = core.LidarFrame(fake_sensor_info(
        64, 1024,
        core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL))

    fields = [
        ("i8", np.int8, (), core.FieldClass.PIXEL_FIELD),
        ("i16", np.int16, (), core.FieldClass.PIXEL_FIELD),
        ("i32", np.int32, (), core.FieldClass.PIXEL_FIELD),
        ("i64", np.int64, (), core.FieldClass.PIXEL_FIELD),
    ]

    # add fields with various integer dtype as defined above
    for field in fields:
        ls.add_field(*field)

    field_types_by_name = {name: field_type for name, field_type in zip(ls.fields, ls.field_types)}

    # for each of the fields we added
    for field in fields:
        name, dtype, extra_dims, field_class = field

        # the numpy array should have the correct shape and type
        field_value = ls.field(name)
        assert field_value.dtype == dtype
        assert field_value.shape == (ls.h, ls.w)

        # the FieldType that corresponds to this field should have the correct attributes too
        field_type = field_types_by_name[name]
        assert field_type.element_type == dtype
        assert field_type.field_class == field_class
        assert field_type.extra_dims == extra_dims

        # setting values should work
        field_value[:] = -1
        field_value[1, 2] = -2
        assert field_value[1, 1] == -1
        field_value[1, 2] == -2


def test_frame_empty_field() -> None:
    """Test that we can add zero size fields through different means and that zero size PFs are disallowed."""
    ls = core.LidarFrame(fake_sensor_info(
        64, 1024,
        core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL))

    ls.add_field("floats", np.ones((64, 1024, 0), np.float64), core.FieldClass.FRAME_FIELD)
    ls.add_field("float", np.array([], np.float64), core.FieldClass.FRAME_FIELD)
    ls.add_field("i8", np.int8, (0,), core.FieldClass.FRAME_FIELD)

    with pytest.raises(ValueError):
        ls.add_field("error", np.ones((64, 1024, 0), np.float64))

    with pytest.raises(ValueError):
        ls.add_field("i82", np.int8, (0,))


def test_lidar_frame_3d_field() -> None:
    """It should allow adding a 3d field."""
    h, w, d = 64, 1024, 3
    field_types = [
        core.FieldType(core.ChanField.RANGE, np.uint32, (d,), core.FieldClass.PIXEL_FIELD)
    ]
    ls = core.LidarFrame(h, w, field_types, 16)
    assert ls.fields == [core.ChanField.RANGE]
    range_field = ls.field(core.ChanField.RANGE)
    assert range_field.dtype == np.uint32
    assert range_field.shape == (h, w, d)


def test_lidar_frame_add_field_3d_field() -> None:
    """It should allow adding a 3d field."""
    h, w, d = 64, 1024, 3
    ls = core.LidarFrame(h, w, [], 16)
    ls.add_field(core.ChanField.RANGE, np.uint32, (d,), core.FieldClass.PIXEL_FIELD)
    assert ls.fields == [core.ChanField.RANGE]
    range_field = ls.field(core.ChanField.RANGE)
    assert range_field.dtype == np.uint32
    assert range_field.shape == (h, w, 3)

    # different constructor accepting dtype directly
    ls.add_field(core.ChanField.RANGE2, np.dtype(np.uint32), (d,), core.FieldClass.PIXEL_FIELD)
    assert ls.fields == [core.ChanField.RANGE, core.ChanField.RANGE2]
    range2_field = ls.field(core.ChanField.RANGE2)
    assert range2_field.dtype == np.uint32
    assert range2_field.shape == (h, w, 3)


def test_lidar_frame_add_field_default_pixel() -> None:
    """FieldType flags should be PIXEL by default."""
    ft = core.FieldType(core.ChanField.RANGE, np.uint32, ())
    assert ft.field_class == core.FieldClass.PIXEL_FIELD
    # it's mutable
    ft.field_class = core.FieldClass.COLUMN_FIELD
    assert ft.field_class == core.FieldClass.COLUMN_FIELD

    # the constructor sets it
    ft = core.FieldType(core.ChanField.RANGE, np.uint32, (), core.FieldClass.COLUMN_FIELD)
    assert ft.field_class == core.FieldClass.COLUMN_FIELD


def test_lidar_frame_fieldtype_name() -> None:
    """FieldType name should be accessible."""
    ft = core.FieldType(core.ChanField.RANGE, np.uint32, ())
    assert ft.name == core.ChanField.RANGE

    # it's mutable
    newname = 'foobar'
    ft.name = newname
    assert ft.name == newname


def test_lidar_frame_min_max_timestamp() -> None:
    """Min and max timestamp should work as expected"""
    ls = core.LidarFrame(fake_sensor_info(
        64, 1024,
        core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL))
    assert len(ls.packet_timestamp) > 6  # just to make sure the logic of this test doesnt break

    cpp = ls.w // len(ls.packet_timestamp)
    # set all packets as valid except packet 1
    ls.status[:] = 1
    ls.status[cpp:cpp + cpp] = 0

    ls.packet_timestamp[:] = 5
    ls.packet_timestamp[0] = 10
    ls.packet_timestamp[1] = 1000
    ls.packet_timestamp[5] = 100

    # make sure it picks 100 as max despite it not being the "last valid"
    assert ls.get_max_valid_packet_timestamp() == 100
    # make sure it picks 5 as min despite it not being the "first valid"
    assert ls.get_min_valid_packet_timestamp() == 5

    # validate that it checks zm packets
    zpt = np.zeros((1,), np.uint64)
    zpt[0] = 101
    ls.add_field("ZONE_PACKET_TIMESTAMP", zpt, core.FieldClass.FRAME_FIELD)
    assert ls.get_max_valid_packet_timestamp() == 101
    assert ls.get_min_valid_packet_timestamp() == 5
    ls.field("ZONE_PACKET_TIMESTAMP")[0] = 4
    assert ls.get_max_valid_packet_timestamp() == 100
    assert ls.get_min_valid_packet_timestamp() == 4

    # validate that it checks imu packets
    ipt = np.zeros((8,), np.uint64)
    ipt[2] = 102
    ipt[4] = 3
    ls.add_field("IMU_PACKET_TIMESTAMP", ipt, core.FieldClass.FRAME_FIELD)
    mpp = 8  # measurements per packet
    istat = np.zeros((ipt.shape[0] * mpp,), np.uint16)
    istat[2 * mpp] = 1
    istat[4 * mpp] = 3
    ls.add_field("IMU_STATUS", istat, core.FieldClass.FRAME_FIELD)
    assert ls.get_max_valid_packet_timestamp() == 102
    assert ls.get_min_valid_packet_timestamp() == 3


def test_lidar_frame_fieldtype_extra_dims() -> None:
    """FieldType extra_dims should be accessible."""
    extra_dims = (1, 2, 3)
    ft = core.FieldType(core.ChanField.RANGE, np.uint32, extra_dims)
    print(ft.extra_dims)
    assert ft.extra_dims == extra_dims

    # it's mutable
    extra_dims = (4, 5, 6)
    ft.extra_dims = extra_dims
    assert ft.extra_dims == extra_dims


def test_lidar_frame_fieldtype_dtype() -> None:
    """FieldType element_type should be accessible"""
    ft = core.FieldType(core.ChanField.RANGE, np.uint32, ())
    assert ft.element_type is np.dtype(np.uint32)

    # it's mutable
    ft.element_type = np.dtype(np.uint8)
    assert ft.element_type is np.dtype(np.uint8)


def test_fieldtype_char_dtype() -> None:
    """FieldType element_type should be accessible"""
    ft = core.FieldType(core.ChanField.RANGE, np.dtype("S25"), (10,), core.FieldClass.FRAME_FIELD)
    assert ft.element_type == np.dtype("S1")
    assert ft.extra_dims == (10, 25)

    # it's mutable
    ft.element_type = np.dtype(np.uint8)
    assert ft.element_type == np.dtype(np.uint8)
    assert ft.extra_dims == (10,)  # drops the fixed string dimension

    ft.element_type = np.dtype("S30")
    assert ft.element_type == np.dtype("S1")
    assert ft.extra_dims == (10, 30)  # adds the fixed string dimension

    ft.element_type = np.dtype("S25")
    assert ft.element_type == np.dtype("S1")
    assert ft.extra_dims == (10, 25)  # replaces the fixed string dimension


def test_lidar_frame_add_field_with_value() -> None:
    """LidarFrame.add_field should accept an array to be used as the field's
    initial value"""
    h, w = 64, 1024
    ls = core.LidarFrame(h, w, [], 16)
    assert core.ChanField.RANGE not in ls.fields
    with pytest.raises(IndexError):
        ls.field(core.ChanField.RANGE)
    ls.add_field(core.ChanField.RANGE, np.ones((h, w), np.int16))
    assert ls.field(core.ChanField.RANGE).all()


def test_lidar_frame_packet_header_width():
    """The packet headers should be wide enough to fit values from the expected number of packets."""
    frame = LidarFrame(1, 1, [], 16)
    assert frame.packet_count == 1
    frame = LidarFrame(1, 1024, [], 16)
    assert frame.packet_count == 64
    frame = LidarFrame(1, 1023, [], 16)
    assert frame.packet_count == 64


def test_lidar_frame_zones_access():
    frame = LidarFrame(1, 1, [], 16)

    from ouster.sdk._bindings.client import ZoneState

    assert frame.zones is not None
    assert frame.zones.shape == (0,)
    assert frame.zones.dtype == ZoneState.dtype()

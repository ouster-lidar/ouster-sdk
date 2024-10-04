"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Tests for lidar data parsing.

Checks that the output of parsing hasn't changed unexpectedly.
"""
from copy import deepcopy

import numpy as np
import pytest

from ouster.sdk import client
from ouster.sdk._bindings.client import scan_to_packets
from ouster.sdk.client import PacketValidationFailure, PacketFormat, PacketWriter, FieldType, LidarScan


def test_make_packets(meta: client.SensorInfo) -> None:
    pf = PacketFormat.from_info(meta)

    p1 = client.ImuPacket()
    p2 = client.ImuPacket(pf.imu_packet_size)

    p3 = client.LidarPacket()
    p4 = client.LidarPacket(pf.lidar_packet_size)

    assert p1.validate(meta, pf) == PacketValidationFailure.PACKET_SIZE
    assert p2.validate(meta, pf) == PacketValidationFailure.NONE

    assert p3.validate(meta, pf) == PacketValidationFailure.PACKET_SIZE
    assert p4.validate(meta, pf) == PacketValidationFailure.NONE


def test_imu_packet(meta: client.SensorInfo) -> None:
    pf = PacketFormat.from_info(meta)

    p = client.ImuPacket(pf.imu_packet_size)

    assert pf.imu_sys_ts(p.buf) == 0
    assert pf.imu_accel_ts(p.buf) == 0
    assert pf.imu_gyro_ts(p.buf) == 0
    assert pf.imu_av_x(p.buf) == 0.0
    assert pf.imu_av_y(p.buf) == 0.0
    assert pf.imu_av_z(p.buf) == 0.0
    assert pf.imu_la_x(p.buf) == 0.0
    assert pf.imu_la_y(p.buf) == 0.0
    assert pf.imu_la_z(p.buf) == 0.0


def test_lidar_packet(meta: client.SensorInfo) -> None:
    """Test reading and writing values from empty packets."""
    pf = PacketFormat(meta)
    p = client.LidarPacket(pf.lidar_packet_size)
    w = pf.columns_per_packet
    h = pf.pixels_per_column

    scan_has_signal = (meta.format.udp_profile_lidar !=
                       client.UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8)

    assert np.array_equal(pf.packet_field(client.ChanField.RANGE, p.buf), np.zeros((h, w)))
    assert np.array_equal(pf.packet_field(client.ChanField.REFLECTIVITY, p.buf),
                          np.zeros((h, w)))
    assert np.array_equal(pf.packet_field(client.ChanField.NEAR_IR, p.buf), np.zeros((h, w)))

    if scan_has_signal:
        assert np.array_equal(pf.packet_field(client.ChanField.SIGNAL, p.buf), np.zeros(
            (h, w)))

    assert len(
        client.ColHeader.__members__) == 5, "Don't forget to update tests!"
    assert np.array_equal(pf.packet_header(client.ColHeader.TIMESTAMP, p.buf), np.zeros(w))
    assert np.array_equal(pf.packet_header(client.ColHeader.MEASUREMENT_ID, p.buf), np.zeros(w))
    assert np.array_equal(pf.packet_header(client.ColHeader.STATUS, p.buf), np.zeros(w))

    assert pf.frame_id(p.buf) == 0


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_read_legacy_packet(packet: client.LidarPacket, packets: client.PacketSource) -> None:
    """Read some arbitrary values from a packet and check header invariants."""
    pf = client.PacketFormat(packets.metadata)
    assert pf.packet_field(client.ChanField.RANGE, packet.buf)[-1, 0] == 12099
    assert pf.packet_field(client.ChanField.REFLECTIVITY, packet.buf)[-1, 0] == 249
    assert pf.packet_field(client.ChanField.SIGNAL, packet.buf)[-1, 0] == 6
    assert pf.packet_field(client.ChanField.NEAR_IR, packet.buf)[-1, 0] == 13

    assert np.all(np.diff(pf.packet_header(client.ColHeader.TIMESTAMP, packet.buf)) > 0)
    assert np.all(np.diff(pf.packet_header(client.ColHeader.MEASUREMENT_ID, packet.buf)) == 1)
    assert pf.packet_type(packet.buf) == 0
    assert pf.frame_id(packet.buf) == 5424
    assert pf.init_id(packet.buf) == 0
    assert pf.prod_sn(packet.buf) == 0
    assert pf.shot_limiting(packet.buf) == 0
    assert pf.thermal_shutdown(packet.buf) == 0
    # in 1024xN mode, the angle between measurements is exactly 88 encoder ticks
    assert np.all(pf.packet_header(client.ColHeader.STATUS, packet.buf) == 0xffffffff)


@pytest.mark.parametrize('test_key', ['single-2.3'])
def test_read_single_return_packet(packet: client.LidarPacket, packets: client.PacketSource) -> None:
    """Read some arbitrary values from packet and check header invariants."""
    pf = client.PacketFormat(packets.metadata)
    assert pf.packet_field(client.ChanField.RANGE, packet.buf)[-1, 0] == 11610
    assert pf.packet_field(client.ChanField.REFLECTIVITY, packet.buf)[-1, 0] == 11
    assert pf.packet_field(client.ChanField.SIGNAL, packet.buf)[-1, 0] == 34
    assert pf.packet_field(client.ChanField.NEAR_IR, packet.buf)[-1, 0] == 393

    assert np.all(np.diff(pf.packet_header(client.ColHeader.TIMESTAMP, packet.buf)) > 0)
    assert np.all(np.diff(pf.packet_header(client.ColHeader.MEASUREMENT_ID, packet.buf)) == 1)
    assert pf.packet_type(packet.buf) == 1
    assert pf.frame_id(packet.buf) == 1259
    assert pf.init_id(packet.buf) == 5431293
    assert pf.prod_sn(packet.buf) == 992210000957
    assert pf.shot_limiting(packet.buf) == 0
    assert pf.thermal_shutdown(packet.buf) == 0

    # Changes from LEGACY
    assert np.all(pf.packet_header(client.ColHeader.STATUS, packet.buf) == 0x01)


def test_lidarscan_init() -> None:
    """If kwargs are used, they should set the scan shape correctly."""
    w, h = 1024, 128
    assert client.LidarScan(h, w).w == w
    assert client.LidarScan(h, w).h == h
    assert client.LidarScan(w=w, h=h).w == w
    assert client.LidarScan(w=w, h=h).h == h
    assert client.LidarScan(h=h, w=w).w == w
    assert client.LidarScan(h=h, w=w).h == h


def test_scan_writeable() -> None:
    """Check that a native scan is a writeable view of data."""
    ls = client.LidarScan(1024, 32)

    assert not ls.field(client.ChanField.RANGE).flags.owndata
    assert not ls.status.flags.owndata

    assert ls.field(client.ChanField.SIGNAL).flags.aligned
    assert ls.measurement_id.flags.aligned

    assert ls.field(client.ChanField.NEAR_IR).flags.aligned
    assert ls.timestamp.flags.aligned

    ls.field(client.ChanField.RANGE)[0, 0] = 42
    assert ls.field(client.ChanField.RANGE)[0, 0] == 42

    ls.field(client.ChanField.RANGE)[:] = 7
    assert np.all(ls.field(client.ChanField.RANGE) == 7)

    ls.status[-1] = 0xffff
    assert ls.status[-1] == 0xffff

    ls.status[:] = 0x1
    assert np.all(ls.status == 0x1)

    assert np.all(ls.pose == np.eye(4))

    ls.pose[1][0, 2] = 8
    assert np.all(ls.pose[1] == np.array([[1, 0, 8, 0], [0, 1, 0, 0],
                                          [0, 0, 1, 0], [0, 0, 0, 1]]))


def test_scan_field_ref() -> None:
    """Test that field references keep scans alive."""

    ls = client.LidarScan(512, 16)
    range = ls.field(client.ChanField.RANGE)
    range[:] = 42

    del ls
    assert np.all(range == 42)

    range[:] = 43
    assert np.all(range == 43)


def test_scan_header_ref() -> None:
    """Test that header references keep scans alive."""

    ls = client.LidarScan(512, 16)
    status = ls.status
    status[:] = 0x11

    del ls
    assert np.all(status == 0x11)

    status[:] = 0x01
    assert np.all(status == 0x01)


def test_scan_not_complete() -> None:
    """Test that not all scans are considered complete."""
    ls = client.LidarScan(32, 1024)

    status = ls.status
    assert not ls.complete()

    status[0] = 0x02
    assert not ls.complete()
    assert not ls.complete((0, 0))

    status[1:] = 0xFFFFFFFF
    assert not ls.complete()

    status[:] = 0xFFFFFFFF
    status[-1] = 0x02
    assert not ls.complete()

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
    assert not ls.complete()
    assert not ls.complete((0, 1))
    assert ls.complete((0, 0))

    status[:] = 0x00
    status[128] = 0xFFFFFFFF
    assert not ls.complete()
    assert not ls.complete((127, 128))
    assert ls.complete((128, 128))


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
def test_scan_complete(w, win_start, win_end) -> None:
    """Set the status headers to the specified window and check complete()."""
    ls = client.LidarScan(32, w)

    status = ls.status

    if win_start <= win_end:
        status[win_start:win_end + 1] = 0xFFFFFFFF
    else:
        status[0:win_end + 1] = 0xFFFFFFFF
        status[win_start:] = 0xFFFFFFFF

    assert ls.complete((win_start, win_end))


def test_scan_fields_ref() -> None:
    """Make sure ref to fields keeps scan alive."""
    fields = client.LidarScan(32, 1024).fields

    # should fail (or trip asan) if the field iterator doesn't keep scan alive
    assert set(fields) == {
        client.ChanField.RANGE,
        client.ChanField.REFLECTIVITY,
        client.ChanField.SIGNAL,
        client.ChanField.NEAR_IR,
        client.ChanField.FLAGS,
    }


def test_scan_default_fields() -> None:
    """Default scan has the expected fields for the LEGACY profile."""
    ls = client.LidarScan(32, 1024)

    assert set(ls.fields) == {
        client.ChanField.RANGE,
        client.ChanField.REFLECTIVITY,
        client.ChanField.SIGNAL,
        client.ChanField.NEAR_IR,
        client.ChanField.FLAGS,
    }

    for f in ls.fields:
        if f == client.ChanField.FLAGS or f == client.ChanField.REFLECTIVITY:
            assert ls.field(f).dtype == np.uint8
        elif f == client.ChanField.RANGE:
            assert ls.field(f).dtype == np.uint32
        else:
            assert ls.field(f).dtype == np.uint16


def test_scan_dual_profile() -> None:
    """Dual returns scan has the expected fields."""
    ls = client.LidarScan(
        32, 1024,
        client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)

    assert set(ls.fields) == {
        client.ChanField.RANGE,
        client.ChanField.RANGE2,
        client.ChanField.REFLECTIVITY,
        client.ChanField.REFLECTIVITY2,
        client.ChanField.SIGNAL,
        client.ChanField.SIGNAL2,
        client.ChanField.FLAGS,
        client.ChanField.FLAGS2,
        client.ChanField.NEAR_IR,
    }


def test_scan_low_data_rate() -> None:
    """Low Data Rate scan has the expected fields."""
    ls = client.LidarScan(32, 1024,
                          client.UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8)

    assert set(ls.fields) == {
        client.ChanField.RANGE,
        client.ChanField.REFLECTIVITY,
        client.ChanField.NEAR_IR,
        client.ChanField.FLAGS,
    }


def test_scan_single_return() -> None:
    """Single Return scan has the expected fields."""
    ls = client.LidarScan(
        32, 1024, client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16)

    assert set(ls.fields) == {
        client.ChanField.RANGE,
        client.ChanField.REFLECTIVITY,
        client.ChanField.SIGNAL,
        client.ChanField.NEAR_IR,
        client.ChanField.FLAGS,
    }


def test_scan_empty() -> None:
    """Sanity check scan with no fields."""
    ls = client.LidarScan(32, 1024, [])
    assert ls.fields == []


def test_scan_custom() -> None:
    """Sanity check scan with a custom set of fields."""
    ls = client.LidarScan(
        32, 1024, [
            FieldType(client.ChanField.SIGNAL, np.uint16),
            FieldType(client.ChanField.FLAGS, np.uint8),
            FieldType("custom0", np.uint32)
        ])

    assert set(ls.fields) == {
        client.ChanField.SIGNAL, client.ChanField.FLAGS,
        "custom0"
    }
    assert ls.field(client.ChanField.SIGNAL).dtype == np.uint16
    assert ls.field("custom0").dtype == np.uint32

    with pytest.raises(IndexError):
        ls.field(client.ChanField.RANGE)


def test_scan_eq_fields() -> None:
    """Test equality between scans with different fields."""
    ls0 = client.LidarScan(32, 1024)
    ls1 = client.LidarScan(32, 1024,
                           client.UDPProfileLidar.PROFILE_LIDAR_LEGACY)
    ls2 = client.LidarScan(
        32, 1024,
        client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)
    ls3 = client.LidarScan(32, 1024, [FieldType(client.ChanField.SIGNAL, np.uint32)])
    ls4 = client.LidarScan(32, 1024, [FieldType(client.ChanField.SIGNAL, np.uint16)])
    ls5 = client.LidarScan(32, 1024, [])

    assert ls0 == ls1
    assert not (ls0 != ls1)  # should be implemented using __eq__
    assert ls1 != ls2
    assert ls3 != ls4
    assert ls5 != ls0
    assert ls5 != ls2
    assert ls5 != ls4


def test_scan_zero_init() -> None:
    """Test that scan fields and headers are zero initialized."""
    ls = client.LidarScan(
        64, 1024,
        client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)

    assert ls.frame_id == -1
    assert ls.frame_status == 0

    assert np.count_nonzero(ls.timestamp) == 0
    assert np.count_nonzero(ls.measurement_id) == 0
    assert np.count_nonzero(ls.status) == 0

    for f in ls.fields:
        assert np.count_nonzero(ls.field(f)) == 0


def test_scan_copy_eq() -> None:
    """Test equality with a copy."""

    ls0 = client.LidarScan(32, 512)
    ls0.status[:] = 0x1
    ls0.field(client.ChanField.REFLECTIVITY)[:] = 100
    ls0.pose[:, 0, 3] = 8

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

    ls0.field(client.ChanField.RANGE)[0, 0] = 42
    assert ls0 != ls1

    ls1.field(client.ChanField.RANGE)[0, 0] = 42
    assert ls0 == ls1

    ls0.pose[1] = np.eye(4)
    assert ls0 != ls1

    ls0.pose[1, 0, 3] = 8
    assert ls0 == ls1


def test_scan_eq_with_custom_fields() -> None:
    """Test equality with custom fields."""

    ls0 = client.LidarScan(32, 512, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ])

    ls1 = deepcopy(ls0)

    ls0.field("custom0")[:] = 100

    ls2 = deepcopy(ls0)

    assert np.count_nonzero(
        ls2.field("custom0") == 100) == ls0.h * ls0.w
    assert np.count_nonzero(ls2.field("custom4") == 100) == 0

    assert ls1 is not ls0
    assert ls1 != ls0
    assert ls2 == ls0


def test_scan_copy_extension() -> None:
    """ Verify we can clone a scan and null pad missing desired fields """
    ls0 = client.LidarScan(32, 512, [
        FieldType("custom4", np.uint8)
    ])

    ls0.field("custom4")[:] = 123

    ls1 = client.LidarScan(ls0, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ])

    assert len(list(ls1.fields)) == 2, ls1.fields
    assert np.count_nonzero(ls1.field("custom0")[0, 0]) == 0
    assert np.count_nonzero(
        ls1.field("custom4") == 123) == ls1.h * ls1.w


def test_scan_copy_retraction() -> None:
    """ Verify we can clone a scan and remove undesired fields """
    ls0 = client.LidarScan(32, 512, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ])

    ls0.field("custom0")[:] = 100
    ls0.field("custom4")[:] = 123

    ls1 = client.LidarScan(ls0, [
        FieldType("custom0", np.uint32),
    ])

    assert ls0.h == ls1.h
    assert ls0.w == ls1.w

    assert len(list(ls1.fields)) == 1
    assert np.count_nonzero(
        ls1.field("custom0") == 100) == ls1.h * ls1.w
    with pytest.raises(IndexError):
        ls1.field("custom4")[0, 0] == 100


def test_scan_copy_cast() -> None:
    """ Verify we can clone a scan and cast between field types """
    ls0 = client.LidarScan(32, 512, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ])

    ls0.field("custom0")[:] = 2 ** 16 - 1
    ls0.field("custom4")[:] = 255

    ls1 = client.LidarScan(ls0, [
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


def test_scan_copy() -> None:
    ls0 = client.LidarScan(32, 512, [
        FieldType("custom0", np.uint32),
        FieldType("custom4", np.uint8)
    ])

    ls0.field("custom0")[:] = 100
    ls0.field("custom4")[:] = 123

    ls1 = client.LidarScan(ls0)

    assert ls0.h == ls1.h
    assert ls0.w == ls1.w

    assert len(list(ls1.fields)) == 2
    assert np.count_nonzero(
        ls1.field("custom0") == 100) == ls1.h * ls1.w
    assert np.count_nonzero(
        ls1.field("custom4") == 123) == ls1.h * ls1.w


def test_packet_writer_bindings(meta: client.SensorInfo) -> None:
    pf = PacketFormat.from_info(meta)
    packet = client.LidarPacket(pf.lidar_packet_size)

    pw = PacketWriter.from_info(meta)
    pw.set_frame_id(packet, 700)
    assert pf.frame_id(packet.buf) == 700

    with pytest.raises(ValueError):
        pw.set_col_timestamp(packet, pw.columns_per_packet, 100)
    with pytest.raises(ValueError):
        pw.set_col_measurement_id(packet, pw.columns_per_packet, 100)
    with pytest.raises(ValueError):
        pw.set_col_status(packet, pw.columns_per_packet, 0x1)

    try:
        for i in range(pw.columns_per_packet):
            pw.set_col_timestamp(packet, i, 100)
            pw.set_col_status(packet, i, 0x1)
            pw.set_col_measurement_id(packet, i, 100)
    except ValueError:
        assert False, "setting cols up to columns_per_packet should not raise"

    for dt in [np.uint8, np.uint16, np.uint32, np.uint64]:
        p = client.LidarPacket(pf.lidar_packet_size)
        for chan in pf.fields:
            if chan in [client.ChanField.RAW32_WORD1,
                        client.ChanField.RAW32_WORD2,
                        client.ChanField.RAW32_WORD3,
                        client.ChanField.RAW32_WORD4]:
                continue
            # mypy is going nuts with mask notation
            value_mask = pw.field_value_mask(chan) & np.iinfo(dt).max  # type: ignore
            shape = (pw.pixels_per_column, pw.columns_per_packet)
            assert value_mask > 0
            _max = max(value_mask, value_mask + 1)
            field = np.random.randint(_max, size=shape, dtype=dt)  # type: ignore
            field = field & value_mask  # type: ignore
            s = hex(value_mask)
            assert np.any(field > 0), f"{chan}, {dt}, {s}"
            pw.set_field(p, chan, field)
            assert np.all(pw.packet_field(chan, p.buf) == field), f"{chan}, {dt}, {s}"

    columns_per_frame = meta.format.columns_per_frame
    ls = client.LidarScan(pf.pixels_per_column, columns_per_frame,
                          pf.udp_profile_lidar, pf.columns_per_packet)
    # all fields are invalid, expect zero packets
    packets = scan_to_packets(ls, pw, 0, 0)
    assert len(packets) == 0

    expected_packets = columns_per_frame / pf.columns_per_packet
    ls.status[:] = 0x1
    packets = scan_to_packets(ls, pw, 0, 0)
    assert len(packets) == expected_packets


def test_to_string_doesnt_cause_fp_exception():
    """It shouldn't crash with a floating point exception when std::to_string(LidarScan&) is called."""
    str(client.LidarScan(1024, 128, client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL))


def test_scan_float_double() -> None:
    """Test that we can add float fields and that setting floats in them works."""
    ls = client.LidarScan(
        64, 1024,
        client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)

    ls.add_field("f32", np.float32, (), client.FieldClass.PIXEL_FIELD)
    ls.add_field("f64", np.float64, (), client.FieldClass.PIXEL_FIELD)

    ls.field("f32")[:] = 3.3
    ls.field("f64")[:] = 6.6

    assert ls.field("f32")[1, 1] == np.float32(3.3)
    assert ls.field("f64")[1, 1] == 6.6


def test_scan_int() -> None:
    """Test that we can add int fields and that setting ints in them works."""
    ls = client.LidarScan(
        64, 1024,
        client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)

    ls.add_field("i8", np.int8, (), client.FieldClass.PIXEL_FIELD)
    ls.add_field("i16", np.int16, (), client.FieldClass.PIXEL_FIELD)
    ls.add_field("i32", np.int32, (), client.FieldClass.PIXEL_FIELD)
    ls.add_field("i64", np.int64, (), client.FieldClass.PIXEL_FIELD)

    ls.field("i8")[:] = 0x7FF
    ls.field("i8")[1, 2] = -2
    ls.field("i16")[:] = 0x7FFFF
    ls.field("i16")[1, 2] = -2
    ls.field("i32")[:] = 0x7FFFFFFF
    ls.field("i32")[1, 2] = -2
    ls.field("i64")[:] = 0x7FFFFFFFFFFFFFFF
    ls.field("i64")[1, 2] = -2

    assert ls.field("i8")[1, 1] == -1
    assert ls.field("i8")[1, 2] == -2
    assert ls.field("i16")[1, 1] == -1
    assert ls.field("i16")[1, 2] == -2
    assert ls.field("i32")[1, 1] == 0x7FFFFFFF
    assert ls.field("i32")[1, 2] == -2
    assert ls.field("i64")[1, 1] == 0x7FFFFFFFFFFFFFFF
    assert ls.field("i64")[1, 2] == -2


def test_scan_empty_field() -> None:
    """Test that we can add zero size fields through different means and that zero size PFs are disallowed."""
    ls = client.LidarScan(
        64, 1024,
        client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)

    ls.add_field("floats", np.ones((64, 1024, 0), np.float64), client.FieldClass.SCAN_FIELD)
    ls.add_field("float", np.array([], np.float64), client.FieldClass.SCAN_FIELD)
    ls.add_field("i8", np.int8, (0,), client.FieldClass.SCAN_FIELD)

    with pytest.raises(ValueError):
        ls.add_field("error", np.ones((64, 1024, 0), np.float64))

    with pytest.raises(ValueError):
        ls.add_field("i82", np.int8, (0,))


def test_lidarscan_3d_field() -> None:
    """It should allow adding a 3d field."""
    h, w, d = 64, 1024, 3
    field_types = [
        client.FieldType(client.ChanField.RANGE, np.uint32, (d,), client.FieldClass.PIXEL_FIELD)
    ]
    ls = client.LidarScan(h, w, field_types)
    assert ls.fields == [client.ChanField.RANGE]
    range_field = ls.field(client.ChanField.RANGE)
    assert range_field.dtype == np.uint32
    assert range_field.shape == (h, w, d)


def test_lidarscan_add_field_3d_field() -> None:
    """It should allow adding a 3d field."""
    h, w, d = 64, 1024, 3
    ls = client.LidarScan(h, w, [])
    ls.add_field(client.ChanField.RANGE, np.uint32, (d,), client.FieldClass.PIXEL_FIELD)
    assert ls.fields == [client.ChanField.RANGE]
    range_field = ls.field(client.ChanField.RANGE)
    assert range_field.dtype == np.uint32
    assert range_field.shape == (h, w, 3)


def test_lidarscan_add_field_default_pixel() -> None:
    """FieldType flags should be PIXEL by default."""
    ft = client.FieldType(client.ChanField.RANGE, np.uint32, ())
    assert ft.field_class == client.FieldClass.PIXEL_FIELD
    # it's mutable
    ft.field_class = client.FieldClass.COLUMN_FIELD
    assert ft.field_class == client.FieldClass.COLUMN_FIELD

    # the constructor sets it
    ft = client.FieldType(client.ChanField.RANGE, np.uint32, (), client.FieldClass.COLUMN_FIELD)
    assert ft.field_class == client.FieldClass.COLUMN_FIELD


def test_lidarscan_fieldtype_name() -> None:
    """FieldType name should be accessible."""
    ft = client.FieldType(client.ChanField.RANGE, np.uint32, ())
    assert ft.name == client.ChanField.RANGE

    # it's mutable
    newname = 'foobar'
    ft.name = newname
    assert ft.name == newname


def test_lidarscan_fieldtype_extra_dims() -> None:
    """FieldType extra_dims should be accessible."""
    extra_dims = (1, 2, 3)
    ft = client.FieldType(client.ChanField.RANGE, np.uint32, extra_dims)
    print(ft.extra_dims)
    assert ft.extra_dims == extra_dims

    # it's mutable
    extra_dims = (4, 5, 6)
    ft.extra_dims = extra_dims
    assert ft.extra_dims == extra_dims


def test_lidarscan_fieldtype_dtype() -> None:
    """FieldType element_type should be accessible"""
    ft = client.FieldType(client.ChanField.RANGE, np.uint32, ())
    assert ft.element_type is np.dtype(np.uint32)

    # it's mutable
    ft.element_type = np.dtype(np.uint8)
    assert ft.element_type is np.dtype(np.uint8)


def test_lidarscan_add_field_with_value() -> None:
    """LidarScan.add_field should accept an array to be used as the field's
    initial value"""
    h, w = 64, 1024
    ls = client.LidarScan(h, w, [])
    assert client.ChanField.RANGE not in ls.fields
    with pytest.raises(IndexError):
        ls.field(client.ChanField.RANGE)
    ls.add_field(client.ChanField.RANGE, np.ones((h, w), np.int16))
    assert ls.field(client.ChanField.RANGE).all()


def test_lidar_scan_packet_header_width():
    """The packet headers should be wide enough to fit values from the expected number of packets."""
    scan = LidarScan(1, 1)
    assert scan.packet_count == 1
    scan = LidarScan(1, 1024)
    assert scan.packet_count == 64
    scan = LidarScan(1, 1023)
    assert scan.packet_count == 64

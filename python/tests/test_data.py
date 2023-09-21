"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Tests for lidar data parsing.

Checks that the output of parsing hasn't changed unexpectedly.
"""
import os
import json
from copy import deepcopy

import numpy as np
import pytest

from ouster import client
from ouster.client import _client
from ouster.pcap import _pcap

from tests.conftest import PCAPS_DATA_DIR


def test_make_packets(meta: client.SensorInfo) -> None:
    pf = _client.PacketFormat.from_info(meta)

    client.ImuPacket(bytes(pf.imu_packet_size), meta)
    client.ImuPacket(bytearray(pf.imu_packet_size), meta)

    with pytest.raises(ValueError):
        client.ImuPacket(bytes(), meta)

    with pytest.raises(ValueError):
        client.ImuPacket(bytes(pf.imu_packet_size - 1), meta)

    client.LidarPacket(bytes(pf.lidar_packet_size), meta)
    client.LidarPacket(bytearray(pf.lidar_packet_size), meta)

    with pytest.raises(ValueError):
        client.LidarPacket(bytes(), meta)

    with pytest.raises(ValueError):
        client.LidarPacket(bytes(pf.lidar_packet_size - 1), meta)


def test_imu_packet(meta: client.SensorInfo) -> None:
    pf = _client.PacketFormat.from_info(meta)

    p = client.ImuPacket(bytes(pf.imu_packet_size), meta)

    assert p.sys_ts == 0
    assert p.accel_ts == 0
    assert p.gyro_ts == 0
    assert np.array_equal(p.accel, np.array([0.0, 0.0, 0.0]))
    assert np.array_equal(p.angular_vel, np.array([0.0, 0.0, 0.0]))

    with pytest.raises(AttributeError):
        p.accel_ts = 0  # type: ignore


def test_lidar_packet(meta: client.SensorInfo) -> None:
    """Test reading and writing values from empty packets."""
    pf = _client.PacketFormat.from_info(meta)
    p = client.LidarPacket(bytes(pf.lidar_packet_size), meta)
    w = pf.columns_per_packet
    h = pf.pixels_per_column

    scan_has_signal = (meta.format.udp_profile_lidar !=
                       client.UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8)

    assert len(
        client.ChanField.__members__) == 29, "Don't forget to update tests!"
    assert np.array_equal(p.field(client.ChanField.RANGE), np.zeros((h, w)))
    assert np.array_equal(p.field(client.ChanField.REFLECTIVITY),
                          np.zeros((h, w)))
    assert np.array_equal(p.field(client.ChanField.NEAR_IR), np.zeros((h, w)))

    if scan_has_signal:
        assert np.array_equal(p.field(client.ChanField.SIGNAL), np.zeros(
            (h, w)))

    assert len(
        client.ColHeader.__members__) == 5, "Don't forget to update tests!"
    assert np.array_equal(p.header(client.ColHeader.TIMESTAMP), np.zeros(w))
    assert np.array_equal(p.timestamp, np.zeros(w))
    assert np.array_equal(p.header(client.ColHeader.FRAME_ID), np.zeros(w))
    assert np.array_equal(p.header(client.ColHeader.MEASUREMENT_ID),
                          np.zeros(w))
    assert np.array_equal(p.measurement_id, np.zeros(w))
    assert np.array_equal(p.header(client.ColHeader.ENCODER_COUNT),
                          np.zeros(w))
    assert np.array_equal(p.header(client.ColHeader.STATUS), np.zeros(w))
    assert np.array_equal(p.status, np.zeros(w))

    assert p.frame_id == 0

    # should not be able to modify packet data
    with pytest.raises(ValueError):
        p.field(client.ChanField.REFLECTIVITY)[0] = 1

    with pytest.raises(ValueError):
        p.header(client.ColHeader.MEASUREMENT_ID)[0] = 1

    with pytest.raises(ValueError):
        p.status[:] = 1

    with pytest.raises(AttributeError):
        p.frame_id = 1  # type: ignore


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_read_legacy_packet(packet: client.LidarPacket) -> None:
    """Read some arbitrary values from a packet and check header invariants."""
    assert packet.field(client.ChanField.RANGE)[-1, 0] == 12099
    assert packet.field(client.ChanField.REFLECTIVITY)[-1, 0] == 1017
    assert packet.field(client.ChanField.SIGNAL)[-1, 0] == 6
    assert packet.field(client.ChanField.NEAR_IR)[-1, 0] == 13

    assert np.all(np.diff(packet.header(client.ColHeader.FRAME_ID)) == 0)
    assert np.all(np.diff(packet.header(client.ColHeader.MEASUREMENT_ID)) == 1)
    assert np.all(np.diff(packet.timestamp) > 0)
    assert np.all(np.diff(packet.measurement_id) == 1)
    assert packet.packet_type == 0
    assert packet.frame_id == 5424
    assert packet.init_id == 0
    assert packet.prod_sn == 0
    assert packet.shot_limiting == 0
    assert packet.thermal_shutdown == 0
    # in 1024xN mode, the angle between measurements is exactly 88 encoder ticks
    assert np.all(np.diff(packet.header(client.ColHeader.ENCODER_COUNT)) == 88)
    assert np.all(packet.status == 0xffffffff)


@pytest.mark.parametrize('test_key', ['single-2.3'])
def test_read_single_return_packet(packet: client.LidarPacket) -> None:
    """Read some arbitrary values from packet and check header invariants."""
    assert packet.field(client.ChanField.RANGE)[-1, 0] == 11610
    assert packet.field(client.ChanField.REFLECTIVITY)[-1, 0] == 11
    assert packet.field(client.ChanField.SIGNAL)[-1, 0] == 34
    assert packet.field(client.ChanField.NEAR_IR)[-1, 0] == 393

    assert np.all(np.diff(packet.header(client.ColHeader.FRAME_ID)) == 0)
    assert np.all(np.diff(packet.header(client.ColHeader.MEASUREMENT_ID)) == 1)
    assert np.all(np.diff(packet.timestamp) > 0)
    assert np.all(np.diff(packet.measurement_id) == 1)
    assert packet.packet_type == 1
    assert packet.frame_id == 1259
    assert packet.init_id == 5431293
    assert packet.prod_sn == 992210000957
    assert packet.shot_limiting == 0
    assert packet.thermal_shutdown == 0

    # Changes from LEGACY
    assert np.all(np.diff(packet.header(client.ColHeader.ENCODER_COUNT)) == 0)
    assert np.all(packet.status == 0x01)


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


def test_scan_from_native() -> None:
    ls = client.LidarScan(1024, 32)
    ls2 = client.LidarScan.from_native(ls)

    assert ls is ls2


def test_scan_to_native() -> None:
    ls = client.LidarScan(1024, 32)
    ls2 = ls.to_native()

    assert ls is ls2


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
    }


def test_scan_default_fields() -> None:
    """Default scan has the expected fields for the LEGACY profile."""
    ls = client.LidarScan(32, 1024)

    assert set(ls.fields) == {
        client.ChanField.RANGE,
        client.ChanField.REFLECTIVITY,
        client.ChanField.SIGNAL,
        client.ChanField.NEAR_IR,
    }

    for f in ls.fields:
        assert ls.field(f).dtype == np.uint32


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
    }


def test_scan_empty() -> None:
    """Sanity check scan with no fields."""
    ls = client.LidarScan(32, 1024, {})

    assert set(ls.fields) == set()

    for f in client.ChanField.values:
        with pytest.raises(ValueError):
            ls.field(f)


def test_scan_custom() -> None:
    """Sanity check scan with a custom set of fields."""
    ls = client.LidarScan(
        32, 1024, {
            client.ChanField.SIGNAL: np.uint16,
            client.ChanField.FLAGS: np.uint8,
            client.ChanField.CUSTOM0: np.uint32
        })

    assert set(ls.fields) == {
        client.ChanField.SIGNAL, client.ChanField.FLAGS,
        client.ChanField.CUSTOM0
    }
    assert ls.field(client.ChanField.SIGNAL).dtype == np.uint16
    assert ls.field(client.ChanField.CUSTOM0).dtype == np.uint32

    with pytest.raises(ValueError):
        ls.field(client.ChanField.RANGE)


def test_scan_eq_fields() -> None:
    """Test equality between scans with different fields."""
    ls0 = client.LidarScan(32, 1024)
    ls1 = client.LidarScan(32, 1024,
                           client.UDPProfileLidar.PROFILE_LIDAR_LEGACY)
    ls2 = client.LidarScan(
        32, 1024,
        client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)
    ls3 = client.LidarScan(32, 1024, {client.ChanField.SIGNAL: np.uint32})
    ls4 = client.LidarScan(32, 1024, {client.ChanField.SIGNAL: np.uint16})
    ls5 = client.LidarScan(32, 1024, {})

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

    ls0 = client.LidarScan(32, 512, {
        client.ChanField.CUSTOM0: np.uint32,
        client.ChanField.CUSTOM4: np.uint8
    })

    ls1 = deepcopy(ls0)

    ls0.field(client.ChanField.CUSTOM0)[:] = 100

    ls2 = deepcopy(ls0)

    assert np.count_nonzero(
        ls2.field(client.ChanField.CUSTOM0) == 100) == ls0.h * ls0.w
    assert np.count_nonzero(ls2.field(client.ChanField.CUSTOM4) == 100) == 0

    assert ls1 is not ls0
    assert ls1 != ls0
    assert ls2 == ls0


def test_error_eq() -> None:
    assert client.PacketSizeError("abc") == client.PacketSizeError("abc")


def test_lidar_packet_validator() -> None:
    """LidarPacketValidator should be capable of the same init_id/sn check as LidarPacket."""
    meta_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')
    pcap_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    metadata = client.SensorInfo(open(meta_file_path).read())

    metadata2 = deepcopy(metadata)
    metadata2.init_id = 1234
    metadata2.sn = "5678"

    validator = client.LidarPacketValidator(metadata2)
    pcap_handle = _pcap.replay_initialize(pcap_file_path)
    buf = bytearray(2**16)
    info = _pcap.packet_info()
    _pcap.next_packet_info(pcap_handle, info)
    n_bytes = _pcap.read_packet(pcap_handle, buf)
    errors = validator.check_packet(buf, n_bytes)
    assert n_bytes == 8448
    try:
        # LidarPacket constructor should throw the same error if _raise_on_id_check is True
        client.LidarPacket(buf, metadata2, None, _raise_on_id_check=True)
    except client.data.PacketIdError as e:
        assert type(errors) is list
        assert len(errors) == 1
        assert str(e) == str(errors[0])
        assert str(e) == "Metadata init_id/sn does not match: expected by metadata \
- 1234/5678, but got from packet buffer - 5431292/122150000150"
        pass
    else:
        assert False, "Expected an exception to be raised by client.LidarPacket."
    finally:
        _pcap.replay_uninitialize(pcap_handle)


def test_lidar_packet_validator_2() -> None:
    """LidarPacketValidator check_packet should return None if there are no issues."""
    meta_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')
    pcap_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    metadata = client.SensorInfo(open(meta_file_path).read())

    metadata2 = deepcopy(metadata)

    validator = client.LidarPacketValidator(metadata2)
    pcap_handle = _pcap.replay_initialize(pcap_file_path)
    buf = bytearray(2**16)
    info = _pcap.packet_info()
    _pcap.next_packet_info(pcap_handle, info)
    n_bytes = _pcap.read_packet(pcap_handle, buf)
    assert n_bytes == 8448
    assert validator.check_packet(buf, n_bytes) == []
    _pcap.replay_uninitialize(pcap_handle)


def test_lidar_packet_validator_3() -> None:
    """LidarPacketValidator check_packet should identify a packet
    that's the wrong size according to the lidar UDP profile."""
    meta_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')
    pcap_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    metadata = client.SensorInfo(open(meta_file_path).read())

    metadata2 = deepcopy(metadata)
    metadata2.format.udp_profile_lidar = client.UDPProfileLidar.PROFILE_LIDAR_FIVE_WORD_PIXEL

    validator = client.LidarPacketValidator(metadata2)
    pcap_handle = _pcap.replay_initialize(pcap_file_path)
    buf = bytearray(2**16)
    info = _pcap.packet_info()
    _pcap.next_packet_info(pcap_handle, info)
    n_bytes = _pcap.read_packet(pcap_handle, buf)
    assert n_bytes == 8448
    errors = validator.check_packet(buf, n_bytes)
    assert len(errors) == 1
    assert type(errors[0]) is client.PacketSizeError
    assert str(errors[0]) == 'Expected a packet of size 41216 but got a buffer of size 8448'
    _pcap.replay_uninitialize(pcap_handle)


def test_lidar_packet_validator_4() -> None:
    """LidarPacketValidator check_packet should identify a packet
    that's the wrong size according to the data format."""
    meta_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')
    pcap_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    meta_json = json.load(open(meta_file_path))
    meta_json['data_format']['columns_per_frame'] = 4096
    meta_json['data_format']['pixels_per_column'] = 16
    meta_json['data_format']['pixel_shift_by_row'] = [0] * 16
    meta_json['beam_altitude_angles'] = [1] * 16
    meta_json['beam_azimuth_angles'] = [1] * 16
    meta_json['prod_sn'] = '1234'
    metadata2 = client.SensorInfo(json.dumps(meta_json))

    validator = client.LidarPacketValidator(metadata2)
    pcap_handle = _pcap.replay_initialize(pcap_file_path)
    buf = bytearray(2**16)
    info = _pcap.packet_info()
    _pcap.next_packet_info(pcap_handle, info)
    n_bytes = _pcap.read_packet(pcap_handle, buf)
    assert n_bytes == 8448
    errors = validator.check_packet(buf, n_bytes)
    assert len(errors) == 2
    assert type(errors[0]) is client.PacketIdError
    assert str(
        errors[0]) == 'Metadata init_id/sn does not match: expected by metadata - \
5431292/1234, but got from packet buffer - 5431292/122150000150'
    assert type(errors[1]) is client.PacketSizeError
    assert str(errors[1]) == 'Expected a packet of size 1280 but got a buffer of size 8448'
    _pcap.replay_uninitialize(pcap_handle)


def test_packet_writer_bindings(meta: client.SensorInfo) -> None:
    pf = _client.PacketFormat.from_info(meta)
    packet = client.LidarPacket(packet_format=pf)

    pw = _client.PacketWriter.from_info(meta)
    pw.set_frame_id(packet, 700)
    assert pf.frame_id(packet._data) == 700

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
        p = client.LidarPacket(packet_format=pf)
        for chan in p.fields:
            if chan in [_client.ChanField.RAW32_WORD1,
                        _client.ChanField.RAW32_WORD2,
                        _client.ChanField.RAW32_WORD3,
                        _client.ChanField.RAW32_WORD4]:
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
            assert np.all(pw.packet_field(chan, p._data) == field), f"{chan}, {dt}, {s}"

    columns_per_frame = meta.format.columns_per_frame
    ls = client.LidarScan(pf.pixels_per_column, columns_per_frame,
                          pf.udp_profile_lidar, pf.columns_per_packet)
    # all fields are invalid, expect zero packets
    packets = _client.scan_to_packets(ls, pw)
    assert len(packets) == 0

    expected_packets = columns_per_frame / pf.columns_per_packet
    ls.status[:] = 0x1
    packets = _client.scan_to_packets(ls, pw)
    assert len(packets) == expected_packets

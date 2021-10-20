"""Tests for lidar data parsing.

Checks that the output of parsing hasn't changed unexpectedly.
"""
from os import path

import numpy as np
import pytest

from ouster import client
from ouster.client import _client

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


@pytest.fixture(scope="module")
def stream_digest():
    digest_path = path.join(DATA_DIR, "os-992011000121_digest.json")
    with open(digest_path, 'r') as f:
        return digest.StreamDigest.from_json(f.read())


@pytest.fixture(scope="module")
def meta():
    meta_path = path.join(DATA_DIR, "os-992011000121_meta.json")
    with open(meta_path, 'r') as f:
        return client.SensorInfo(f.read())


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

    assert len(
        client.ChanField.__members__) == 7, "Don't forget to update tests!"
    assert np.array_equal(p.field(client.ChanField.RANGE), np.zeros((h, w)))
    assert np.array_equal(p.field(client.ChanField.REFLECTIVITY),
                          np.zeros((h, w)))
    assert np.array_equal(p.field(client.ChanField.SIGNAL), np.zeros((h, w)))
    assert np.array_equal(p.field(client.ChanField.NEAR_IR), np.zeros((h, w)))

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
        p.field(client.ChanField.SIGNAL)[0] = 1

    with pytest.raises(ValueError):
        p.header(client.ColHeader.MEASUREMENT_ID)[0] = 1

    with pytest.raises(ValueError):
        p.status[:] = 1

    with pytest.raises(AttributeError):
        p.frame_id = 1  # type: ignore


@pytest.fixture(scope="module")
def packet(stream_digest, meta):
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")
    with open(bin_path, 'rb') as b:
        return next(iter(digest.LidarBufStream(b, meta)))


def test_read_real_packet(packet: client.LidarPacket) -> None:
    """Read some arbitrary values from a packet and check header invariants."""
    assert packet.field(client.ChanField.RANGE)[0, 0] == 1723
    assert packet.field(client.ChanField.REFLECTIVITY)[0, 0] == 196
    assert packet.field(client.ChanField.SIGNAL)[0, 0] == 66
    assert packet.field(client.ChanField.NEAR_IR)[0, 0] == 1768

    assert np.all(np.diff(packet.header(client.ColHeader.FRAME_ID)) == 0)
    assert np.all(np.diff(packet.header(client.ColHeader.MEASUREMENT_ID)) == 1)
    assert np.all(np.diff(packet.timestamp) > 0)
    assert np.all(np.diff(packet.measurement_id) == 1)
    assert packet.packet_type == 0
    assert packet.frame_id == 34266
    assert packet.init_id == 0
    assert packet.prod_sn == 0
    # in 512xN mode, the angle between measurements is exactly 176 encoder ticks
    assert np.all(
        np.diff(packet.header(client.ColHeader.ENCODER_COUNT)) == 176)
    assert np.all(packet.status == 0xffffffff)


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
    assert ls.header(client.ColHeader.STATUS)[-1] == 0xffff

    ls.header(client.ColHeader.STATUS)[-2] = 0xffff
    assert ls.status[-2] == 0xffff
    assert ls.header(client.ColHeader.STATUS)[-2] == 0xffff

    ls.status[:] = 0x1
    assert np.all(ls.status == 0x1)
    assert np.all(ls.header(client.ColHeader.STATUS) == 0x1)


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
    assert not ls._complete()

    status[0] = 0x02
    assert not ls._complete()
    assert not ls._complete((0, 0))

    status[1:] = 0xFFFFFFFF
    assert not ls._complete()

    status[:] = 0xFFFFFFFF
    status[-1] = 0x02
    assert not ls._complete()

    # windows are inclusive but python slicing is not
    status[:] = 0x00
    status[:10] = 0xFFFFFFFF
    assert not ls._complete((0, 10))

    status[:] = 0x00
    status[11:21] = 0xFFFFFFFF
    assert not ls._complete((10, 20))

    # window [i, i]
    status[:] = 0x00
    status[0] = 0xFFFFFFFF
    assert not ls._complete()
    assert not ls._complete((0, 1))
    assert ls._complete((0, 0))

    status[:] = 0x00
    status[128] = 0xFFFFFFFF
    assert not ls._complete()
    assert not ls._complete((127, 128))
    assert ls._complete((128, 128))


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
    """Set the status headers to the specified window and check _complete()."""
    ls = client.LidarScan(32, w)

    status = ls.status

    if win_start <= win_end:
        status[win_start:win_end + 1] = 0xFFFFFFFF
    else:
        status[0:win_end + 1] = 0xFFFFFFFF
        status[win_start:] = 0xFFFFFFFF

    assert ls._complete((win_start, win_end))

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
def info(stream_digest) -> client.SensorInfo:
    return stream_digest.meta


def test_make_packets(info: client.SensorInfo) -> None:
    pf = _client.PacketFormat.from_info(info)

    client.ImuPacket(bytes(pf.imu_packet_size), info)
    client.ImuPacket(bytearray(pf.imu_packet_size), info)

    with pytest.raises(ValueError):
        client.ImuPacket(bytes(), info)

    with pytest.raises(ValueError):
        client.ImuPacket(bytes(pf.imu_packet_size - 1), info)

    client.LidarPacket(bytes(pf.lidar_packet_size), info)
    client.LidarPacket(bytearray(pf.lidar_packet_size), info)

    with pytest.raises(ValueError):
        client.LidarPacket(bytes(), info)

    with pytest.raises(ValueError):
        client.LidarPacket(bytes(pf.lidar_packet_size - 1), info)


def test_imu_packet(info: client.SensorInfo) -> None:
    pf = _client.PacketFormat.from_info(info)

    p = client.ImuPacket(bytes(pf.imu_packet_size), info)

    assert p.sys_ts == 0
    assert p.accel_ts == 0
    assert p.gyro_ts == 0
    assert np.array_equal(p.accel, np.array([0.0, 0.0, 0.0]))
    assert np.array_equal(p.angular_vel, np.array([0.0, 0.0, 0.0]))

    with pytest.raises(AttributeError):
        p.accel_ts = 0  # type: ignore


def test_lidar_packet(info: client.SensorInfo) -> None:
    pf = _client.PacketFormat.from_info(info)
    """Test reading and writing values from empty packets."""
    p = client.LidarPacket(bytes(pf.lidar_packet_size), info)
    w = pf.columns_per_packet
    h = pf.pixels_per_column

    assert len(
        client.ChanField.__members__) == 4, "Don't forget to update tests!"
    assert np.array_equal(p.field(client.ChanField.RANGE), np.zeros((h, w)))
    assert np.array_equal(p.field(client.ChanField.REFLECTIVITY),
                          np.zeros((h, w)))
    assert np.array_equal(p.field(client.ChanField.SIGNAL), np.zeros((h, w)))
    assert np.array_equal(p.field(client.ChanField.NEAR_IR), np.zeros((h, w)))
    # TODO: what to do with other fields

    assert len(
        client.ColHeader.__members__) == 5, "Don't forget to update tests!"
    assert np.array_equal(p.header(client.ColHeader.TIMESTAMP), np.zeros(w))
    assert np.array_equal(p.header(client.ColHeader.FRAME_ID), np.zeros(w))
    assert np.array_equal(p.header(client.ColHeader.MEASUREMENT_ID),
                          np.zeros(w))
    assert np.array_equal(p.header(client.ColHeader.ENCODER_COUNT),
                          np.zeros(w))
    assert np.array_equal(p.header(client.ColHeader.STATUS), np.zeros(w))

    # should not be able to modify a packet built from a read-only buffer
    with pytest.raises(ValueError):
        p.field(client.ChanField.SIGNAL)[0] = 1

    with pytest.raises(ValueError):
        p.header(client.ColHeader.MEASUREMENT_ID)[0] = 1

    # a writeable lidar packet
    q = client.LidarPacket(bytearray(pf.lidar_packet_size), info)

    q.field(client.ChanField.SIGNAL)[:] = np.ones((h, w))
    assert np.array_equal(q.field(client.ChanField.SIGNAL), np.ones((h, w)))

    # TODO: masking prevents writing RANGE. Need separate get/set for fields
    # q.view(client.ChanField.RANGE)[:] = np.ones((h, w))
    # assert np.array_equal(q.view(client.ChanField.RANGE), np.ones((h, w)))

    with pytest.raises(ValueError):
        q.field(client.ChanField.SIGNAL)[:] = np.ones((w, h))
    with pytest.raises(ValueError):
        q.field(client.ChanField.SIGNAL)[:] = np.ones((h - 1, w))
    with pytest.raises(ValueError):
        q.field(client.ChanField.SIGNAL)[:] = np.ones((h, w + 1))

    q.header(client.ColHeader.MEASUREMENT_ID)[:] = np.ones(w)
    assert np.array_equal(q.header(client.ColHeader.MEASUREMENT_ID),
                          np.ones(w))


@pytest.fixture(scope="module")
def packet(stream_digest):
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")
    with open(bin_path, 'rb') as b:
        return next(iter(digest.LidarBufStream(b, stream_digest.meta)))


def test_read_real_packet(packet: client.LidarPacket) -> None:
    """Read some arbitrary values from a packet and check header invariants."""
    assert packet.field(client.ChanField.RANGE)[0, 0] == 1723
    assert packet.field(client.ChanField.REFLECTIVITY)[0, 0] == 196
    assert packet.field(client.ChanField.SIGNAL)[0, 0] == 66
    assert packet.field(client.ChanField.NEAR_IR)[0, 0] == 1768

    assert np.all(np.diff(packet.header(client.ColHeader.FRAME_ID)) == 0)
    assert np.all(np.diff(packet.header(client.ColHeader.MEASUREMENT_ID)) == 1)
    # in 512xN mode, the angle between measurements is exactly 176 encoder ticks
    assert np.all(
        np.diff(packet.header(client.ColHeader.ENCODER_COUNT)) == 176)
    assert np.all(packet.header(client.ColHeader.STATUS) == 0xffffffff)


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

    status[0] = 0x01
    assert not ls._complete()
    assert not ls._complete((0, 0))

    status[1:] = 0xFFFFFFFF
    assert not ls._complete()

    status[:] = 0xFFFFFFFF
    status[-1] = 0x01
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

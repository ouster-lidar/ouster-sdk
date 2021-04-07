"""Tests for lidar data parsing.

Checks that the output of parsing hasn't changed unexpectedly.
"""
from os import path

import numpy as np
import pytest

from ouster import client

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


@pytest.fixture(scope="module")
def stream_digest():
    digest_path = path.join(DATA_DIR, "os-992011000121_digest.json")
    with open(digest_path, 'r') as f:
        return digest.StreamDigest.from_json(f.read())


@pytest.fixture(scope="module")
def pf(stream_digest):
    return client.PacketFormat(stream_digest.meta)


def test_make_packets(pf: client.PacketFormat) -> None:
    client.ImuPacket(bytes(pf.imu_packet_size), pf)
    client.ImuPacket(bytearray(pf.imu_packet_size), pf)

    with pytest.raises(ValueError):
        client.ImuPacket(bytes(), pf)

    with pytest.raises(ValueError):
        client.ImuPacket(bytes(pf.imu_packet_size - 1), pf)

    client.LidarPacket(bytes(pf.lidar_packet_size), pf)
    client.LidarPacket(bytearray(pf.lidar_packet_size), pf)

    with pytest.raises(ValueError):
        client.LidarPacket(bytes(), pf)

    with pytest.raises(ValueError):
        client.LidarPacket(bytes(pf.lidar_packet_size - 1), pf)


def test_imu_packet(pf: client.PacketFormat) -> None:
    p = client.ImuPacket(bytes(pf.imu_packet_size), pf)

    assert p.sys_ts == 0
    assert p.accel_ts == 0
    assert p.gyro_ts == 0
    assert np.array_equal(p.accel, np.array([0.0, 0.0, 0.0]))
    assert np.array_equal(p.angular_vel, np.array([0.0, 0.0, 0.0]))

    with pytest.raises(AttributeError):
        p.accel_ts = 0  # type: ignore


def test_lidar_packet(pf: client.PacketFormat) -> None:
    """Test reading and writing values from empty packets."""
    p = client.LidarPacket(bytes(pf.lidar_packet_size), pf)
    w = pf.columns_per_packet
    h = pf.pixels_per_column

    assert len(
        client.ChanField.__members__) == 4, "Don't forget to update tests!"
    assert np.array_equal(p.view(client.ChanField.RANGE), np.zeros((h, w)))
    assert np.array_equal(p.view(client.ChanField.REFLECTIVITY),
                          np.zeros((h, w)))
    assert np.array_equal(p.view(client.ChanField.INTENSITY), np.zeros((h, w)))
    assert np.array_equal(p.view(client.ChanField.AMBIENT), np.zeros((h, w)))

    assert len(
        client.ColHeader.__members__) == 5, "Don't forget to update tests!"
    assert np.array_equal(p.view(client.ColHeader.TIMESTAMP), np.zeros(w))
    assert np.array_equal(p.view(client.ColHeader.FRAME_ID), np.zeros(w))
    assert np.array_equal(p.view(client.ColHeader.MEASUREMENT_ID), np.zeros(w))
    assert np.array_equal(p.view(client.ColHeader.ENCODER_COUNT), np.zeros(w))
    assert np.array_equal(p.view(client.ColHeader.VALID), np.zeros(w))

    # should not be able to modify a packet built from a read-only buffer
    with pytest.raises(ValueError):
        p.view(client.ChanField.INTENSITY)[0] = 1

    with pytest.raises(ValueError):
        p.view(client.ColHeader.MEASUREMENT_ID)[0] = 1

    # a writeable lidar packet
    q = client.LidarPacket(bytearray(pf.lidar_packet_size), pf)

    q.view(client.ChanField.INTENSITY)[:] = np.ones((h, w))
    assert np.array_equal(q.view(client.ChanField.INTENSITY), np.ones((h, w)))

    # TODO: masking prevents writing RANGE. Need separate get/set for fields
    # q.view(client.ChanField.RANGE)[:] = np.ones((h, w))
    # assert np.array_equal(q.view(client.ChanField.RANGE), np.ones((h, w)))

    with pytest.raises(ValueError):
        q.view(client.ChanField.INTENSITY)[:] = np.ones((w, h))
    with pytest.raises(ValueError):
        q.view(client.ChanField.INTENSITY)[:] = np.ones((h - 1, w))
    with pytest.raises(ValueError):
        q.view(client.ChanField.INTENSITY)[:] = np.ones((h, w + 1))

    q.view(client.ColHeader.MEASUREMENT_ID)[:] = np.ones(w)
    assert np.array_equal(q.view(client.ColHeader.MEASUREMENT_ID), np.ones(w))


@pytest.fixture(scope="module")
def packet(stream_digest):
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")
    with open(bin_path, 'rb') as b:
        return next(iter(digest.LidarBufStream(b, stream_digest.meta)))


def test_read_real_packet(packet: client.LidarPacket) -> None:
    """Read some arbitrary values from a packet and check header invariants."""
    assert packet.view(client.ChanField.RANGE)[0, 0] == 1723
    assert packet.view(client.ChanField.REFLECTIVITY)[0, 0] == 196
    assert packet.view(client.ChanField.INTENSITY)[0, 0] == 66
    assert packet.view(client.ChanField.AMBIENT)[0, 0] == 1768

    assert np.all(np.diff(packet.view(client.ColHeader.FRAME_ID)) == 0)
    assert np.all(np.diff(packet.view(client.ColHeader.MEASUREMENT_ID)) == 1)
    # in 512xN mode, the angle between measurements is exactly 176 encoder ticks
    assert np.all(np.diff(packet.view(client.ColHeader.ENCODER_COUNT)) == 176)
    assert np.all(packet.view(client.ColHeader.VALID) == 0xffffffff)

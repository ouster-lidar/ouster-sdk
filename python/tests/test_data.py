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


@pytest.fixture
def packets(stream_digest: digest.StreamDigest):
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")
    with open(bin_path, 'rb') as b:
        yield digest.LidarBufStream(b, stream_digest.meta)


def test_scans_meta(packets: client.PacketSource) -> None:
    scans = iter(client.Scans(packets))
    scan = next(scans)

    assert scan.frame_id != -1
    assert scan.h == packets.metadata.format.pixels_per_column
    assert scan.w == packets.metadata.format.columns_per_frame
    assert len(scan.headers) == scan.w

    assert not scan.complete, "test data should have missing packet!"

    # check that the scan is missing exactly one packet's worth of columns
    valid_columns = [h.status for h in scan.headers].count(0xffffffff)
    assert valid_columns == (packets.metadata.format.columns_per_frame -
                             packets.metadata.format.columns_per_packet)

    missing_ts = [h.timestamp for h in scan.headers].count(0)
    assert missing_ts == packets.metadata.format.columns_per_packet

    zero_enc = [h.encoder for h in scan.headers].count(0)
    assert zero_enc == packets.metadata.format.columns_per_packet + 1

    with pytest.raises(StopIteration):
        next(scans)


def test_batch_first_packet(packet: client.LidarPacket,
                            packets: client.PacketSource) -> None:
    """Check that data in the first packet survives batching to a scan."""
    scans = iter(client.Scans(packets))
    scan = next(scans)

    h = packet._pf.pixels_per_column
    w = packet._pf.columns_per_packet

    assert np.array_equal(packet.view(client.ChanField.RANGE),
                          scan.field(client.ChanField.RANGE)[:h, :w])

    assert np.array_equal(packet.view(client.ChanField.REFLECTIVITY),
                          scan.field(client.ChanField.REFLECTIVITY)[:h, :w])

    assert np.array_equal(packet.view(client.ChanField.INTENSITY),
                          scan.field(client.ChanField.INTENSITY)[:h, :w])

    assert np.array_equal(packet.view(client.ChanField.AMBIENT),
                          scan.field(client.ChanField.AMBIENT)[:h, :w])

    assert np.all(packet.view(client.ColHeader.FRAME_ID) == scan.frame_id)

    assert np.array_equal(packet.view(client.ColHeader.TIMESTAMP),
                          [h.timestamp for h in scan.headers][:w])

    assert np.array_equal(packet.view(client.ColHeader.ENCODER_COUNT),
                          [h.encoder for h in scan.headers][:w])

    assert np.array_equal(packet.view(client.ColHeader.VALID),
                          [h.status for h in scan.headers][:w])


def test_scans_complete(packets: client.PacketSource) -> None:
    """Test built-in filtering for complete scans.

    The test dataset only contains a single incomplete frame. Check that
    specifying ``complete=True`` discards it.
    """
    scans = iter(client.Scans(packets, complete=True))

    with pytest.raises(StopIteration):
        next(scans)


def test_scans_timeout(packets: client.PacketSource) -> None:
    """A zero timeout should deterministically throw."""
    scans = iter(client.Scans(packets, timeout=0.0))

    with pytest.raises(client.ClientTimeout):
        next(scans)


def test_parse_and_batch_packets(stream_digest,
                                 packets: client.PacketSource) -> None:
    """Test that parsing packets produces expected results.

    Checks hashes of all packet and scans fields and headers against
    known-good values.
    """
    other = digest.StreamDigest.from_packets(packets)
    stream_digest.check(other)

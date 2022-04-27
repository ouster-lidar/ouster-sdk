"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from contextlib import closing
import socket

import numpy as np
import pytest

from ouster import client
from ouster.client import ColHeader, ChanField

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa


@pytest.fixture
def default_meta():
    return client.SensorInfo.from_default(client.LidarMode.MODE_1024x10)


def test_sensor_init(default_meta: client.SensorInfo) -> None:
    """Initializing a data stream with metadata makes no network calls."""
    with closing(client.Sensor("", 0, 0, metadata=default_meta)) as source:
        assert source._cli.lidar_port != 0
        assert source._cli.imu_port != 0


def test_sensor_timeout(default_meta: client.SensorInfo) -> None:
    """Setting a zero timeout reliably raises an exception."""
    with closing(client.Sensor("", 0, 0, metadata=default_meta,
                               timeout=0.0)) as source:
        with pytest.raises(client.ClientTimeout):
            next(iter(source))


def test_sensor_closed(default_meta: client.SensorInfo) -> None:
    """Check reading from a closed source raises an exception."""
    with closing(client.Sensor("", 0, 0, metadata=default_meta)) as source:
        source.close()
        with pytest.raises(ValueError):
            next(iter(source))


def test_sensor_port_in_use(default_meta: client.SensorInfo) -> None:
    """Instantiating clients listening to the same port does not fail."""
    with closing(client.Sensor("", 0, 0, metadata=default_meta)) as s1:
        with closing(
                client.Sensor("",
                              s1._cli.lidar_port,
                              s1._cli.imu_port,
                              metadata=default_meta)) as s2:
            assert s2._cli.lidar_port != 0
            assert s2._cli.imu_port != 0
            assert s2._cli.lidar_port == s1._cli.lidar_port
            assert s2._cli.imu_port == s1._cli.imu_port


def test_sensor_packet(default_meta: client.SensorInfo) -> None:
    """Check that the client will read single properly-sized LEGACY packet."""
    with closing(
            client.Sensor("",
                          0,
                          0,
                          metadata=default_meta,
                          timeout=5.0,
                          _flush_before_read=False)) as source:
        data = np.random.randint(255,
                                 size=source._pf.lidar_packet_size,
                                 dtype=np.uint8)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(data.tobytes(), ("localhost", source._cli.lidar_port))
        packet = next(iter(source))
        assert (packet._data == data).all()


def test_sensor_packet_bad_size(default_meta: client.SensorInfo) -> None:
    """Check that the client will ignore improperly-sized packets."""
    with closing(
            client.Sensor("",
                          0,
                          0,
                          metadata=default_meta,
                          timeout=1.0,
                          _flush_before_read=False)) as source:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(b"hello", ("localhost", source._cli.lidar_port))
        with pytest.raises(client.ClientTimeout):
            next(iter(source))


def test_scans_simple(packets: client.PacketSource) -> None:
    """Check that the test data contains exactly one scan."""
    scans = iter(client.Scans(packets))
    assert next(scans) is not None

    with pytest.raises(StopIteration):
        next(scans)


def test_scans_closed(default_meta: client.SensorInfo) -> None:
    """Check reading from closed scans raises an exception."""
    with closing(client.Sensor("", 0, 0, metadata=default_meta)) as source:
        scans = client.Scans(source)
        scans.close()
        with pytest.raises(ValueError):
            next(iter(scans))


def test_scans_meta(packets: client.PacketSource) -> None:
    """Sanity check metadata and column headers of a batched scan."""
    scans = iter(client.Scans(packets))
    scan = next(scans)

    assert scan.frame_id != -1
    assert scan.h == packets.metadata.format.pixels_per_column
    assert scan.w == packets.metadata.format.columns_per_frame
    assert len(scan.timestamp) == scan.w
    assert len(scan.measurement_id) == scan.w
    assert len(scan.status) == scan.w
    assert len(scan.header(ColHeader.ENCODER_COUNT)) == scan.w

    assert scan._complete()

    # all timestamps valid
    assert np.count_nonzero(scan.timestamp) == scan.w

    if (packets.metadata.format.udp_profile_lidar ==
            client.UDPProfileLidar.PROFILE_LIDAR_LEGACY):
        # check that all columns are valid
        assert (scan.status == 0xffffffff).all()
        # only first encoder count is zero
        assert np.count_nonzero(scan.header(
            ColHeader.ENCODER_COUNT)) == scan.w - 1
    else:
        # only lowest bit indicates valid
        assert (scan.status & 0x1).all()
        # encoder counts zeroed
        assert (scan.header(ColHeader.ENCODER_COUNT) == 0).all()


def test_scans_first_packet(packet: client.LidarPacket,
                            packets: client.PacketSource) -> None:
    """Check that data in the first packet survives batching to a scan."""
    scans = iter(client.Scans(packets))
    scan = next(scans)

    h = packet._pf.pixels_per_column
    w = packet._pf.columns_per_packet

    assert np.array_equal(packet.field(ChanField.RANGE),
                          scan.field(ChanField.RANGE)[:h, :w])

    assert np.array_equal(packet.field(ChanField.REFLECTIVITY),
                          scan.field(ChanField.REFLECTIVITY)[:h, :w])

    assert np.array_equal(packet.field(ChanField.SIGNAL),
                          scan.field(ChanField.SIGNAL)[:h, :w])

    assert np.array_equal(packet.field(ChanField.NEAR_IR),
                          scan.field(ChanField.NEAR_IR)[:h, :w])

    assert packet.frame_id == scan.frame_id

    assert np.array_equal(packet.timestamp, scan.timestamp[:w])

    assert np.array_equal(packet.header(ColHeader.ENCODER_COUNT),
                          scan.header(ColHeader.ENCODER_COUNT)[:w])

    assert np.array_equal(packet.status, scan.status[:w])


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_scans_complete(packets: client.PacketSource) -> None:
    """Test built-in filtering for complete scans."""

    # make new packet source missing a packet
    ps = list(packets)
    del ps[5]
    dropped = client.Packets(ps, packets.metadata)

    scans = iter(client.Scans(dropped, complete=True))

    with pytest.raises(StopIteration):
        next(scans)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_scans_timeout(packets: client.PacketSource) -> None:
    """A zero timeout should deterministically throw.

    TODO: should it, though?
    """
    scans = iter(client.Scans(packets, timeout=0.0))

    with pytest.raises(client.ClientTimeout):
        next(scans)


def test_scans_digest(stream_digest, packets: client.PacketSource) -> None:
    """Test that parsing packets produces expected results.

    Checks hashes of all packet and scans fields and headers against
    known-good values.
    """
    other = digest.StreamDigest.from_packets(packets)
    stream_digest.check(other)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_dual(packets: client.PacketSource) -> None:
    """Test scans from dual returns data."""
    scans = client.Scans(packets, complete=True)

    assert (scans.metadata.format.udp_profile_lidar ==
            client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)

    ls = list(scans)

    assert len(ls) == 1
    assert set(ls[0].fields) == {
        ChanField.RANGE,
        ChanField.RANGE2,
        ChanField.REFLECTIVITY,
        ChanField.REFLECTIVITY2,
        ChanField.SIGNAL,
        ChanField.SIGNAL2,
        ChanField.NEAR_IR,
    }


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_empty_fields(packets: client.PacketSource) -> None:
    """Test batching scans with no fields."""
    scan = next(iter(client.Scans(packets, fields={})))
    assert set(scan.fields) == set()


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_one_field(packets: client.PacketSource) -> None:
    """Test batching scans with a single field."""
    fields = {ChanField.FLAGS: np.uint8}
    scan = next(iter(client.Scans(packets, fields=fields)))

    assert set(scan.fields) == {ChanField.FLAGS}
    assert scan.field(ChanField.FLAGS).dtype == np.uint8

    with pytest.raises(ValueError):
        scan.field(ChanField.RANGE)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_bad_type(packets: client.PacketSource) -> None:
    """Test batching scans with a type too small for source data."""
    fields = {ChanField.RANGE: np.uint16}

    with pytest.raises(ValueError):
        next(iter(client.Scans(packets, fields=fields)))


@pytest.mark.parametrize('test_key', ['legacy-2.0', 'legacy-2.1'])
def test_scans_bad_field(packets: client.PacketSource) -> None:
    """Test batching scans with a field not present on source packets."""
    fields = {ChanField.RANGE2: np.uint32}

    with pytest.raises(IndexError):
        next(iter(client.Scans(packets, fields=fields)))


@pytest.mark.parametrize('test_key', ['legacy-2.0', 'legacy-2.1'])
def test_scans_raw(packets: client.PacketSource) -> None:
    """Smoke test reading raw channel field data."""
    fields = {
        ChanField.RAW32_WORD1: np.uint32,
        ChanField.RAW32_WORD2: np.uint32,
        ChanField.RAW32_WORD3: np.uint32
    }

    scans = client.Scans(packets, fields=fields)

    ls = list(scans)
    assert len(ls) == 1
    assert set(ls[0].fields) == {
        ChanField.RAW32_WORD1, ChanField.RAW32_WORD2, ChanField.RAW32_WORD3
    }

    # just check that raw fields are populated?
    for f in ls[0].fields:
        assert np.count_nonzero(ls[0].field(f)) != 0

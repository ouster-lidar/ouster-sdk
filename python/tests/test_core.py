"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from contextlib import closing
import socket
import random

import numpy as np
import pytest

from ouster.sdk import client
from ouster.sdk.client import ChanField, LidarPacket, ImuPacket, PacketFormat, ColHeader, Version, FieldType
from ouster.sdk.client.core import ClientTimeout

pytest.register_assert_rewrite('ouster.sdk.client._digest')
import ouster.sdk.client._digest as digest  # noqa


@pytest.fixture
def default_meta():
    meta = client.SensorInfo.from_default(client.LidarMode.MODE_1024x10)
    meta.config.udp_port_imu = random.randint(10000, 65000)
    meta.config.udp_port_lidar = random.randint(10000, 65000)
    return meta


def test_sensor_init(default_meta: client.SensorInfo) -> None:
    """Initializing a data stream with metadata makes no network calls."""
    with closing(client.Sensor("localhost", None, None, metadata=default_meta)) as source:
        assert source.lidar_port != 0
        assert source.imu_port != 0


def test_sensor_timeout(default_meta: client.SensorInfo) -> None:
    """Setting a zero timeout reliably raises an exception."""
    with closing(client.Sensor("localhost", None, None, metadata=default_meta,
                               timeout=0.1)) as source:
        with pytest.raises(client.ClientTimeout):
            next(iter(source))


def test_sensor_closed(default_meta: client.SensorInfo) -> None:
    """Check reading from a closed source raises an exception."""
    with closing(client.Sensor("localhost", None, None, metadata=default_meta)) as source:
        source.close()
        with pytest.raises(ValueError):
            next(iter(source))


def test_sensor_port_in_use(default_meta: client.SensorInfo) -> None:
    """Instantiating clients listening to the same port does not fail."""
    with closing(client.Sensor("localhost", None, None, metadata=default_meta)) as s1:
        with closing(
                client.Sensor("localhost",
                              s1.lidar_port,
                              s1.imu_port,
                              metadata=default_meta)) as s2:
            assert s2.lidar_port != 0
            assert s2.imu_port != 0
            assert s2.lidar_port == s1.lidar_port
            assert s2.imu_port == s1.imu_port


def test_sensor_packet2(default_meta: client.SensorInfo) -> None:
    """Check that the client will read single properly-sized IMU/LIDAR packet."""
    with closing(
            client.Sensor("127.0.0.1",
                          None,
                          None,
                          metadata=default_meta,
                          timeout=5.0,
                          _flush_before_read=False)) as source:
        pf = PacketFormat(source.metadata)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data = np.random.randint(255,
                                 size=pf.lidar_packet_size,
                                 dtype=np.uint8)
        sock.sendto(data.tobytes(), ("127.0.0.1", source.lidar_port))
        packet = next(iter(source))
        assert (packet.buf == data).all()
        assert isinstance(packet, LidarPacket)

        data = np.random.randint(255,
                                 size=pf.imu_packet_size,
                                 dtype=np.uint8)
        sock.sendto(data.tobytes(), ("127.0.0.1", source.imu_port))
        packet = next(iter(source))
        assert (packet.buf == data).all()
        assert isinstance(packet, ImuPacket)


def test_sensor_flush(default_meta: client.SensorInfo) -> None:
    with closing(
            client.Sensor("127.0.0.1",
                          None,
                          None,
                          metadata=default_meta,
                          timeout=1.0,
                          _flush_before_read=False)) as source:
        pf = client.PacketFormat(source.metadata)
        total_packets_sent = 5
        for i in range(total_packets_sent):
            data = np.zeros((pf.lidar_packet_size), dtype=np.uint8)
            data[:] = i
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(data.tobytes(), ("127.0.0.1", source.lidar_port))
        flushed_packets = 2
        source.flush(flushed_packets)
        for i in range(flushed_packets, total_packets_sent):
            packet = next(iter(source))
            assert (packet.buf == i).all()


def test_sensor_packet_bad_size(default_meta: client.SensorInfo) -> None:
    """Check that the client will ignore improperly-sized packets."""
    with closing(
            client.Sensor("127.0.0.1",
                          None,
                          None,
                          metadata=default_meta,
                          timeout=1.0,
                          _flush_before_read=False)) as source:
        pf = PacketFormat.from_info(source.metadata)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # send packet too small
        sock.sendto(b"hello", ("127.0.0.1", source.lidar_port))
        with pytest.raises(client.ClientTimeout):
            next(iter(source))

        # send packet too big
        data = np.random.randint(255,
                                 size=pf.lidar_packet_size + 10,
                                 dtype=np.uint8)
        sock.sendto(data.tobytes(), ("127.0.0.1", source.lidar_port))
        with pytest.raises(client.ClientTimeout):
            next(iter(source))


# TODO: reenable once we figure out CI determinism
@pytest.mark.skip
def test_sensor_overflow(default_meta: client.SensorInfo) -> None:
    with closing(
            client.Sensor("",
                          None,
                          None,
                          buf_size=10,
                          metadata=default_meta,
                          timeout=1.0,
                          _overflow_err=True,
                          _flush_before_read=False)) as source:
        pf = client.PacketFormat(source.metadata)
        total_packets_sent = 20
        for i in range(total_packets_sent):
            data = np.random.randint(255,
                                     size=pf.lidar_packet_size,
                                     dtype=np.uint8)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(data.tobytes(), ("127.0.0.1", source.lidar_port))
        with pytest.raises(client.ClientOverflow):
            for i in range(total_packets_sent):
                next(iter(source))


def test_scans_simple(packets: client.PacketSource) -> None:
    """Check that the test data contains exactly one scan."""
    scans = iter(client.Scans(packets))
    assert next(scans) is not None

    with pytest.raises(StopIteration):
        next(scans)


def test_scans_closed(default_meta: client.SensorInfo) -> None:
    """Check reading from closed scans raises an exception."""
    with closing(client.Sensor("localhost", None, None, metadata=default_meta)) as source:
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

    assert scan.complete()

    # all timestamps valid
    assert np.count_nonzero(scan.timestamp) == scan.w

    if (packets.metadata.format.udp_profile_lidar ==
            client.UDPProfileLidar.PROFILE_LIDAR_LEGACY):
        # check that all columns are valid
        assert (scan.status == 0xffffffff).all()
    else:
        # only lowest bit indicates valid
        assert (scan.status & 0x1).all()


def test_scans_first_packet(packet: client.LidarPacket,
                            packets: client.PacketSource) -> None:
    """Check that data in the first packet survives batching to a scan."""
    scans = iter(client.Scans(packets))
    scan = next(scans)

    pf = PacketFormat(packets.metadata)
    h = pf.pixels_per_column
    w = pf.columns_per_packet

    is_low_data_rate_profile = (packets.metadata.format.udp_profile_lidar ==
                                client.UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8)

    if not is_low_data_rate_profile:  # low data rate profile RANGE is scaled up
        assert np.array_equal(pf.packet_field(ChanField.RANGE, packet.buf),
                              scan.field(ChanField.RANGE)[:h, :w])

    assert np.array_equal(pf.packet_field(ChanField.REFLECTIVITY, packet.buf),
                          scan.field(ChanField.REFLECTIVITY)[:h, :w])

    if is_low_data_rate_profile:  # low data rate profile has no SIGNAL
        assert ChanField.SIGNAL not in set(scan.fields)
    else:
        assert np.array_equal(pf.packet_field(ChanField.SIGNAL, packet.buf),
                              scan.field(ChanField.SIGNAL)[:h, :w])

    if not is_low_data_rate_profile:  # low data rate profile NEAR_IR is scaled up
        assert np.array_equal(pf.packet_field(ChanField.NEAR_IR, packet.buf),
                              scan.field(ChanField.NEAR_IR)[:h, :w])

    assert pf.frame_id(packet.buf) == scan.frame_id

    assert np.array_equal(pf.packet_header(ColHeader.TIMESTAMP, packet.buf), scan.timestamp[:w])

    assert np.array_equal(pf.packet_header(ColHeader.STATUS, packet.buf), scan.status[:w])


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
    """A zero timeout should deterministically throw a ClientTimeout.
    """
    scans = client.Scans(packets, timeout=0.0)
    scans_itr = iter(scans)

    with pytest.raises(ClientTimeout):
        next(scans_itr)


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
        ChanField.FLAGS,
        ChanField.FLAGS2,
        ChanField.NEAR_IR,
    }


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_empty_fields(packets: client.PacketSource) -> None:
    """Test batching scans with no fields."""
    scan = next(iter(client.Scans(packets, fields=[])))
    assert set(scan.fields) == set()


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_one_field(packets: client.PacketSource) -> None:
    """Test batching scans with a single field."""
    fields = [FieldType(ChanField.FLAGS, np.uint8)]
    scan = next(iter(client.Scans(packets, fields=fields)))

    assert set(scan.fields) == {ChanField.FLAGS}
    assert scan.field(ChanField.FLAGS).dtype == np.uint8

    with pytest.raises(IndexError):
        scan.field(ChanField.RANGE)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_bad_type(packets: client.PacketSource) -> None:
    """Test batching scans with a type too small for source data."""
    fields = [FieldType(ChanField.RANGE, np.uint16)]

    with pytest.raises(ValueError):
        next(iter(client.Scans(packets, fields=fields)))


@pytest.mark.parametrize('test_key', ['legacy-2.0', 'legacy-2.1'])
def test_scans_raw(packets: client.PacketSource) -> None:
    """Smoke test reading raw channel field data."""
    fields = [
        FieldType(ChanField.RAW32_WORD1, np.uint32),
        FieldType(ChanField.RAW32_WORD2, np.uint32),
        FieldType(ChanField.RAW32_WORD3, np.uint32)
    ]

    scans = client.Scans(packets, fields=fields)

    ls = list(scans)
    assert len(ls) == 1
    assert set(ls[0].fields) == {
        ChanField.RAW32_WORD1, ChanField.RAW32_WORD2, ChanField.RAW32_WORD3
    }

    # just check that raw fields are populated?
    for f in ls[0].fields:
        assert np.count_nonzero(ls[0].field(f)) != 0


def test_version_compare() -> None:
    v1 = Version.from_string("1.2.3")
    v2 = Version.from_string("1.3.3")
    v3 = Version.from_string("v1.2.3")

    assert v1 == v3
    assert v1 != v2


def test_version_parse() -> None:
    v = Version.from_string("v1.2.3")
    assert v.stage == ""
    assert v.machine == ""
    assert v.major == 1
    assert v.minor == 2
    assert v.patch == 3

    v = Version.from_string("1.2.3")
    assert v.major == 1
    assert v.minor == 2
    assert v.patch == 3

    v = Version.from_string("ousteros-prod-bootes-v1.2.3-rc1+123456")
    assert v.major == 1
    assert v.minor == 2
    assert v.patch == 3
    assert v.stage == "prod"
    assert v.machine == "bootes"
    assert v.prerelease == "rc1"
    assert v.build == "123456"

    v = Version.from_string("ousteros-prod-bootes-v1.2.3+123456")
    assert v.major == 1
    assert v.minor == 2
    assert v.patch == 3
    assert v.stage == "prod"
    assert v.machine == "bootes"
    assert v.prerelease == ""
    assert v.build == "123456"

    v = Version.from_string("ousteros-image-prod-aries-v2.0.0-rc.2+20201023140416.staging")
    assert v.major == 2
    assert v.minor == 0
    assert v.patch == 0
    assert v.stage == "prod"
    assert v.machine == "aries"
    assert v.prerelease == "rc.2"
    assert v.build == "20201023140416.staging"

    # Invalid version strings should parse as all zeros
    v = Version.from_string("2.3")
    assert v.major == 0
    assert v.minor == 0
    assert v.patch == 0

    v = Version.from_string("a.2.3")
    assert v.major == 0
    assert v.minor == 0
    assert v.patch == 0

    v = Version.from_string("3")
    assert v.major == 0
    assert v.minor == 0
    assert v.patch == 0

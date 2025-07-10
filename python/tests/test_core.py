"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from contextlib import closing
import socket
import random
from os import path

import numpy as np
import pytest

from ouster.sdk import core, sensor
from ouster.sdk.core import ChanField, LidarPacket, ImuPacket, PacketFormat, ColHeader, Version, FieldType
# need to include both directly so that pytest.raises works correctly
from ouster.sdk.sensor import ClientError, ClientTimeout # noqa

pytest.register_assert_rewrite('ouster.sdk.core._digest')
import ouster.sdk.core._digest as digest  # noqa


@pytest.fixture
def default_meta():
    meta = core.SensorInfo.from_default(core.LidarMode.MODE_1024x10)
    meta.config.udp_port_imu = random.randint(10000, 65000)
    meta.config.udp_port_lidar = random.randint(10000, 65000)
    return meta


def test_sensor_init(default_meta: core.SensorInfo) -> None:
    """Initializing a data stream with metadata makes no network calls."""
    with closing(sensor.SensorPacketSource("localhost", sensor_info=[default_meta])) as source:
        assert source.sensor_info == [default_meta]


def test_sensor_timeout(default_meta: core.SensorInfo) -> None:
    """Setting a zero timeout reliably raises an exception."""
    with closing(sensor.SensorPacketSource("localhost", sensor_info=[default_meta],
                               timeout=0.1)) as source:
        with pytest.raises(sensor.ClientTimeout):
            next(iter(source))


def test_sensor_closed(default_meta: core.SensorInfo) -> None:
    """Check reading from a closed source raises an exception."""
    with closing(sensor.SensorPacketSource("localhost", sensor_info=[default_meta])) as source:
        source.close()
        with pytest.raises(RuntimeError):
            next(iter(source))


def test_sensor_port_in_use(default_meta: core.SensorInfo) -> None:
    """Instantiating clients listening to the same port does not fail."""
    with closing(sensor.SensorPacketSource("localhost", sensor_info=[default_meta])) as _:
        with closing(
                sensor.SensorPacketSource("localhost",
                              sensor_info=[default_meta])) as _:
            pass


def test_sensor_packet2(default_meta: core.SensorInfo) -> None:
    """Check that the client will read single properly-sized IMU/LIDAR packet."""
    with closing(
            sensor.SensorPacketSource("127.0.0.1",
                          sensor_info=[default_meta],
                          timeout=5.0)) as source:
        pf = PacketFormat(source.sensor_info[0])
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data = np.random.randint(255,
                                 size=pf.lidar_packet_size,
                                 dtype=np.uint8)
        sock.sendto(data.tobytes(), ("127.0.0.1", source.sensor_info[0].config.udp_port_lidar))
        id, packet = next(iter(source))
        assert (packet.buf == data).all()
        assert isinstance(packet, LidarPacket)

        data = np.random.randint(255,
                                 size=pf.imu_packet_size,
                                 dtype=np.uint8)
        sock.sendto(data.tobytes(), ("127.0.0.1", source.sensor_info[0].config.udp_port_imu))
        id, packet = next(iter(source))
        assert (packet.buf == data).all()
        assert isinstance(packet, ImuPacket)


def test_sensor_packet_bad_size(default_meta: core.SensorInfo) -> None:
    """Check that the client will ignore improperly-sized packets."""
    with closing(
            sensor.SensorPacketSource("127.0.0.1",
                          sensor_info=[default_meta],
                          timeout=1.0)) as source:
        pf = PacketFormat.from_info(source.sensor_info[0])
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # send packet too small
        sock.sendto(b"hello", ("127.0.0.1", source.sensor_info[0].config.udp_port_lidar))
        with pytest.raises(ClientTimeout):
            next(iter(source))

        # send packet too big
        data = np.random.randint(255,
                                 size=pf.lidar_packet_size + 10,
                                 dtype=np.uint8)
        sock.sendto(data.tobytes(), ("127.0.0.1", source.sensor_info[0].config.udp_port_lidar))
        with pytest.raises(ClientTimeout):
            packet = next(iter(source))
            print(packet, len(packet[1].buf), pf.lidar_packet_size)


def test_scans_simple(packets: core.PacketSource) -> None:
    """Check that the test data contains exactly one scan."""
    scans = iter(core.Scans(packets))
    assert next(scans) is not None

    with pytest.raises(StopIteration):
        next(scans)


def test_scans_closed(default_meta: core.SensorInfo) -> None:
    """Check reading from closed scans raises an exception."""
    with closing(sensor.SensorPacketSource("localhost", sensor_info=[default_meta])) as source:
        scans = core.Scans(source)
        scans.close()
        with pytest.raises(ValueError):
            next(iter(scans))


def test_scans_meta(packets: core.PacketSource) -> None:
    """Sanity check metadata and column headers of a batched scan."""
    scans = iter(core.Scans(packets))
    scan = next(scans)[0]
    assert scan is not None

    assert scan.frame_id != -1
    assert scan.h == packets.sensor_info[0].format.pixels_per_column
    assert scan.w == packets.sensor_info[0].format.columns_per_frame
    assert len(scan.timestamp) == scan.w
    assert len(scan.measurement_id) == scan.w
    assert len(scan.status) == scan.w

    assert scan.complete()

    assert packets.sensor_info[0] == scan.sensor_info

    # all timestamps valid
    assert np.count_nonzero(scan.timestamp) == scan.w

    if (packets.sensor_info[0].format.udp_profile_lidar ==
            core.UDPProfileLidar.PROFILE_LIDAR_LEGACY):
        # check that all columns are valid
        assert (scan.status == 0xffffffff).all()
    else:
        # only lowest bit indicates valid
        assert (scan.status & 0x1).all()


def test_scans_first_packet(packet: core.LidarPacket,
                            packets: core.PacketSource) -> None:
    """Check that data in the first packet survives batching to a scan."""
    scans = iter(core.Scans(packets))
    scan = next(scans)[0]
    assert scan is not None

    pf = PacketFormat(packets.sensor_info[0])
    h = pf.pixels_per_column
    w = pf.columns_per_packet

    is_low_data_rate_profile = (packets.sensor_info[0].format.udp_profile_lidar ==
                                core.UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8)

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


def test_scans_digest(stream_digest, packets: core.PacketSource) -> None:
    """Test that parsing packets produces expected results.

    Checks hashes of all packet and scans fields and headers against
    known-good values.
    """
    other = digest.StreamDigest.from_packets(packets)
    stream_digest.check(other)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_dual(packets: core.PacketSource) -> None:
    """Test scans from dual returns data."""
    scans = core.Scans(packets)

    assert (packets.sensor_info[0].format.udp_profile_lidar ==
            core.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)

    ls = list(scans)

    assert len(ls) == 1
    assert ls[0][0] is not None
    assert set(ls[0][0].fields) == {
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
def test_scans_empty_fields(packets: core.PacketSource) -> None:
    """Test batching scans with no fields."""
    scan = next(iter(core.Scans(packets, fields=[[]])))[0]
    assert scan is not None
    assert set(scan.fields) == set()


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_one_field(packets: core.PacketSource) -> None:
    """Test batching scans with a single field."""
    fields = [FieldType(ChanField.FLAGS, np.uint8)]
    scan = next(iter(core.Scans(packets, fields=[fields])))[0]
    assert scan is not None
    assert set(scan.fields) == {ChanField.FLAGS}
    assert scan.field(ChanField.FLAGS).dtype == np.uint8

    with pytest.raises(IndexError):
        scan.field(ChanField.RANGE)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_bad_type(packets: core.PacketSource) -> None:
    """Test batching scans with a type too small for source data."""
    fields = [FieldType(ChanField.RANGE, np.uint16)]

    with pytest.raises(ValueError):
        next(iter(core.Scans(packets, fields=[fields])))


@pytest.mark.parametrize('test_key', ['legacy-2.0', 'legacy-2.1'])
def test_scans_raw(packets: core.PacketSource) -> None:
    """Smoke test reading raw channel field data."""
    fields = [
        FieldType(ChanField.RAW32_WORD1, np.uint32),
        FieldType(ChanField.RAW32_WORD2, np.uint32),
        FieldType(ChanField.RAW32_WORD3, np.uint32)
    ]

    scans = core.Scans(packets, fields=[fields])

    ls = list(scans)
    assert len(ls) == 1
    assert ls[0][0] is not None
    assert set(ls[0][0].fields) == {
        ChanField.RAW32_WORD1, ChanField.RAW32_WORD2, ChanField.RAW32_WORD3
    }

    # just check that raw fields are populated?
    for f in ls[0][0].fields:
        assert np.count_nonzero(ls[0][0].field(f)) != 0


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


CURRENT_DIR = path.dirname(path.abspath(__file__))
CLOUDS_DATA_DIR = path.join(CURRENT_DIR, "..", "..", "tests", "clouds")


@pytest.mark.parametrize('model', ['cloud-ascii.pcd', 'cloud-binary.pcd', 'cloud-ascii.ply', 'cloud-binary.ply'])
def test_pointcloud_load(model: str):
    from ouster.sdk.core import read_pointcloud

    expected = np.array([[1, 2, 3]])

    points = read_pointcloud(CLOUDS_DATA_DIR + "/" + model)

    assert np.array_equal(points, expected)


def array_equal_order_independent(a1, a2):
    # make sure arrays are the same length
    if len(a1) != len(a2):
        return False

    # finally make sure each element exists exactly once in the second array
    count = [0] * len(a1)
    for idx, i1 in enumerate(a1):
        for i2 in a2:
            if (i1 == i2).all():
                count[idx] += 1
                break

    if np.array_equal(count, np.ones((a1.shape[0],))):
        return True
    return False


def test_downsample():
    from ouster.sdk.core import voxel_downsample

    # make sure that wrong shape throws
    with pytest.raises(ValueError):
        pts, attrs = voxel_downsample(0.1, np.array([[0.0, 0.0]]), [])

    with pytest.raises(ValueError):
        pts, attrs = voxel_downsample([0.1, 0.1], np.array([[0.0, 0.0]]), [])

    # make sure we cant do zero or negative voxel sizes
    with pytest.raises(ValueError):
        pts, attrs = voxel_downsample(0.0, np.array([[0.0, 0.0]]), [])

    # make a simple 4 point test
    in_pts = np.array([[0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 2.0, 0.0], [0.0, 2.0, 0.0]])
    in_attrs = np.array([[10.0, 100.0], [12.0, 102.0], [20.0, 200.0], [22.0, 202.0]])

    tests = []
    tests.append((0.1, [[0.0, 2.0, 0.0], [0.0, 1.0, 0.0]]))
    tests.append((4.0, [[0.0, 1.5, 0.0]]))
    tests.append(([4.0, 0.1, 0.1], [[0.0, 2.0, 0.0], [0.0, 1.0, 0.0]]))
    tests.append(([4.0, 4.0, 0.1], [[0.0, 1.5, 0.0]]))

    for voxel, expected_pts in tests:
        if len(expected_pts) == 2:
            expected_attrs = [[21.0, 201.0], [11.0, 101.0]]
        else:
            expected_attrs = [[16.0, 151.0]]

        # Test each with each ordering
        for order in ['F', 'C']:
            out_pts, out_attrs = voxel_downsample(
                voxel,
                np.asarray(in_pts, order=order),
                np.asarray(in_attrs, order=order)
            )
            assert array_equal_order_independent(out_pts, expected_pts)
            assert array_equal_order_independent(out_attrs, expected_attrs)

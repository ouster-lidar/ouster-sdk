"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from contextlib import closing, contextmanager
from tests.multi import Frames
import socket
import random
import sys
import copy
from os import path
from typing import Set

import numpy as np
import psutil
import pytest

from ouster.sdk import core, sensor, open_source
from ouster.sdk.core import (ChanField, LidarPacket, ImuPacket, PacketFormat, ColHeader, Version, FieldType,
                             LidarFrame)
# need to include both directly so that pytest.raises works correctly
from ouster.sdk.sensor import ClientError, ClientTimeout # noqa

pytest.register_assert_rewrite('ouster.sdk.core._digest')
import ouster.sdk.core._digest as digest  # noqa


def _reserve_socket(allow_reuse: bool) -> socket.socket:
    """Create a UDP socket bound to localhost, optionally allowing reuse."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Reuse options must be set after bind(): on Linux, an ephemeral bind() with
    # SO_REUSEADDR already set may be assigned a port another reusable socket holds.
    sock.bind(("127.0.0.1", 0))
    if allow_reuse:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                pass
    return sock


@contextmanager
def _meta_with_reserved_ports(base_meta: core.LidarInfo,
                              allow_reuse: bool = True):
    """Yield a copy of meta with ports reserved for the duration."""
    sockets = []
    try:
        meta = copy.copy(base_meta)
        lidar_sock = _reserve_socket(allow_reuse)
        imu_sock = _reserve_socket(allow_reuse)
        sockets.extend([lidar_sock, imu_sock])
        meta.config.udp_port_lidar = lidar_sock.getsockname()[1]
        meta.config.udp_port_imu = imu_sock.getsockname()[1]
        yield meta
    finally:
        for sock in sockets:
            sock.close()


@pytest.fixture
def default_meta():
    meta = core.LidarInfo.from_default(core.LidarMode._1024x10)
    meta.config.udp_port_imu = random.randint(10000, 65000)
    meta.config.udp_port_lidar = random.randint(10000, 65000)
    return meta


def test_sensor_init(default_meta: core.LidarInfo) -> None:
    """Initializing a data stream with metadata makes no network calls."""
    with _meta_with_reserved_ports(default_meta) as meta:
        with closing(sensor.SensorPacketSource("localhost", sensor_info=[meta],
                                   reuse_ports=True)) as source:
            assert source.sensor_info == [meta]


def test_sensor_timeout(default_meta: core.LidarInfo) -> None:
    """Setting a zero timeout reliably raises an exception."""
    with _meta_with_reserved_ports(default_meta) as meta:
        with closing(sensor.SensorPacketSource("localhost", sensor_info=[meta],
                                   timeout=0.1, reuse_ports=True)) as source:
            with pytest.raises(sensor.ClientTimeout):
                next(iter(source))


def test_sensor_closed(default_meta: core.LidarInfo) -> None:
    """Check reading from a closed source raises an exception."""
    with _meta_with_reserved_ports(default_meta) as meta:
        with closing(sensor.SensorPacketSource("localhost", sensor_info=[meta],
                                   reuse_ports=True)) as source:
            source.close()
            with pytest.raises(RuntimeError):
                next(iter(source))


def test_sensor_port_in_use_reuse(default_meta: core.LidarInfo) -> None:
    """Instantiating clients listening to the same port does not fail."""
    with _meta_with_reserved_ports(default_meta) as meta:
        with closing(sensor.SensorPacketSource("localhost", sensor_info=[meta], reuse_ports=True)) as _:
            with closing(
                    sensor.SensorPacketSource("localhost",
                                  sensor_info=[meta], reuse_ports=True)) as _:
                pass


@pytest.mark.skipif(sys.platform.startswith("win"), reason="Broken on Windows")
def test_sensor_port_in_use_no_reuse(default_meta: core.LidarInfo) -> None:
    """Instantiating clients listening to the same port does fail."""
    with _meta_with_reserved_ports(default_meta, allow_reuse=False) as meta:
        with pytest.raises(RuntimeError):
            with closing(
                    sensor.SensorPacketSource("localhost",
                                              sensor_info=[meta])) as _:
                pass


@pytest.mark.parametrize("wrap", [
    lambda src: src.single(0),
    lambda src: core.collate(src),
], ids=["single", "collate"])
def test_wrapped_sensor_source_close_releases_sockets(default_meta: core.LidarInfo, wrap) -> None:
    """Closing a wrapper around a sensor source closes the wrapped source (OSDK-1184)."""
    with _meta_with_reserved_ports(default_meta) as meta:
        raw = sensor.SensorFrameSetSource("127.0.0.1", sensor_info=[meta], reuse_ports=True)
        try:
            wrapped = wrap(raw)
            assert raw.sockets()
            wrapped.close()
            assert raw.sockets() == []
        finally:
            raw.close()


def _udp_ports_bound_by_this_process() -> Set[int]:
    proc = psutil.Process()
    conns = getattr(proc, "net_connections", proc.connections)(kind="udp")
    return {conn.laddr.port for conn in conns if conn.laddr}


@pytest.mark.skipif(sys.platform.startswith("win"), reason="Broken on Windows")
def test_open_source_sensor_idx_close_releases_port(default_meta: core.LidarInfo) -> None:
    """Closing a sensor source opened with a sensor_idx releases its ports (OSDK-1184)."""
    # The reservation is held until open_source has bound the ports, then released
    # so the source is the only socket on them.
    with _meta_with_reserved_ports(default_meta) as meta:
        source = open_source("127.0.0.1", sensor_idx=0, sensor_info=[meta], reuse_ports=True)
    ports = {meta.config.udp_port_lidar, meta.config.udp_port_imu}
    try:
        _ = iter(source)
        assert ports <= _udp_ports_bound_by_this_process()
        source.close()
        # Only this process's sockets are checked; other test processes may legitimately
        # bind the released ports.
        assert not ports & _udp_ports_bound_by_this_process()
    finally:
        source.close()


def test_sensor_packet2(default_meta: core.LidarInfo) -> None:
    """Check that the client will read single properly-sized IMU/LIDAR packet."""
    # Reserve an ephemeral port by binding to '0'.
    tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tmp_socket.bind(('127.0.0.1', 0))
    tmp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Use the reserved port for our dummy sensor
    reserved_port = tmp_socket.getsockname()[1]

    meta = copy.copy(default_meta)
    meta.config.udp_port_lidar = reserved_port
    meta.config.udp_port_imu = reserved_port
    with closing(
            sensor.SensorPacketSource("127.0.0.1",
                          sensor_info=[meta],
                          timeout=5.0, reuse_ports=True)) as source:
        tmp_socket.close()
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


def test_sensor_packet_bad_size(default_meta: core.LidarInfo) -> None:
    """Check that the client will ignore improperly-sized packets."""
    # Reserve an ephemeral port by binding to '0'.
    tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tmp_socket.bind(('127.0.0.1', 0))
    tmp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Use the reserved port for our dummy sensor
    reserved_port = tmp_socket.getsockname()[1]

    meta = copy.copy(default_meta)
    meta.config.udp_port_lidar = reserved_port
    with closing(
            sensor.SensorPacketSource("127.0.0.1",
                          sensor_info=[meta],
                          timeout=1.0, reuse_ports=True)) as source:
        tmp_socket.close()
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


def test_frames_simple(packets: core.PacketSource) -> None:
    """Check that the test data contains exactly one frame."""
    frames = iter(Frames(packets))
    assert next(frames) is not None

    with pytest.raises(StopIteration):
        next(frames)


def test_frames_closed(default_meta: core.LidarInfo) -> None:
    """Check reading from closed frames raises an exception."""
    with _meta_with_reserved_ports(default_meta) as meta:
        with closing(sensor.SensorPacketSource("localhost", sensor_info=[meta],
                                   reuse_ports=True)) as source:
            frames = Frames(source)
            frames.close()
            with pytest.raises(ValueError):
                next(iter(frames))


def test_frames_meta(packets: core.PacketSource) -> None:
    """Sanity check metadata and column headers of a batched frame."""
    frames = iter(Frames(packets))
    frame = next(frames)[0]
    assert frame is not None
    # Frames is a lidar-only source, so every frame is a LidarFrame
    assert isinstance(frame, LidarFrame)

    assert frame.frame_id != -1
    assert frame.h == packets.sensor_info[0].format.pixels_per_column
    assert frame.w == packets.sensor_info[0].format.columns_per_frame
    assert len(frame.timestamp) == frame.w
    assert len(frame.measurement_id) == frame.w
    assert len(frame.status) == frame.w

    assert frame.complete()

    assert packets.sensor_info[0] == frame.sensor_info

    # all timestamps valid
    assert np.count_nonzero(frame.timestamp) == frame.w

    if (packets.sensor_info[0].format.udp_profile_lidar ==
            core.UDPProfileLidar.LEGACY):
        # check that all columns are valid
        assert (frame.status == 0xffffffff).all()
    else:
        # only lowest bit indicates valid
        assert (frame.status & 0x1).all()


def test_frames_first_packet(packet: core.LidarPacket,
                            packets: core.PacketSource) -> None:
    """Check that data in the first packet survives batching to a frame."""
    frames = iter(Frames(packets))
    frame = next(frames)[0]
    assert frame is not None
    # Frames is a lidar-only source, so every frame is a LidarFrame
    assert isinstance(frame, LidarFrame)

    pf = PacketFormat(packets.sensor_info[0])
    h = pf.pixels_per_column
    w = pf.columns_per_packet

    is_low_data_rate_profile = (packets.sensor_info[0].format.udp_profile_lidar ==
                                core.UDPProfileLidar.RNG15_RFL8_NIR8)

    if not is_low_data_rate_profile:  # low data rate profile RANGE is scaled up
        assert np.array_equal(pf.packet_field(ChanField.RANGE, packet.buf),
                              frame.field(ChanField.RANGE)[:h, :w])

    assert np.array_equal(pf.packet_field(ChanField.REFLECTIVITY, packet.buf),
                          frame.field(ChanField.REFLECTIVITY)[:h, :w])

    if is_low_data_rate_profile:  # low data rate profile has no SIGNAL
        assert ChanField.SIGNAL not in set(frame.fields)
    else:
        assert np.array_equal(pf.packet_field(ChanField.SIGNAL, packet.buf),
                              frame.field(ChanField.SIGNAL)[:h, :w])

    if not is_low_data_rate_profile:  # low data rate profile NEAR_IR is scaled up
        assert np.array_equal(pf.packet_field(ChanField.NEAR_IR, packet.buf),
                              frame.field(ChanField.NEAR_IR)[:h, :w])

    assert pf.frame_id(packet.buf) == frame.frame_id

    assert np.array_equal(pf.packet_header(ColHeader.TIMESTAMP, packet.buf), frame.timestamp[:w])

    assert np.array_equal(pf.packet_header(ColHeader.STATUS, packet.buf), frame.status[:w])


def test_frames_digest(stream_digest, packets: core.PacketSource) -> None:
    """Test that parsing packets produces expected results.

    Checks hashes of all packet and frames fields and headers against
    known-good values.
    """
    other = digest.StreamDigest.from_packets(packets)
    stream_digest.check(other)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_frames_dual(packets: core.PacketSource) -> None:
    """Test frames from dual returns data."""
    frames = Frames(packets)

    assert (packets.sensor_info[0].format.udp_profile_lidar ==
            core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL)

    ls = list(frames)

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
        ChanField.NEAR_IR
    }


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_frames_dual_3_2(packets: core.PacketSource) -> None:
    """Test frames from dual returns data from 3.2 FW."""
    packets.sensor_info[0].image_rev = "3.2.0"
    frames = Frames(packets)

    assert (packets.sensor_info[0].format.udp_profile_lidar ==
            core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL)

    ls = list(frames)

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
        ChanField.WINDOW
    }


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_frames_empty_fields(packets: core.PacketSource) -> None:
    """Test batching frames with no fields."""
    frame = next(iter(Frames(packets, fields=[[]])))[0]
    assert frame is not None
    assert set(frame.fields) == set()


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_frames_one_field(packets: core.PacketSource) -> None:
    """Test batching frames with a single field."""
    fields = [FieldType(ChanField.FLAGS, np.uint8)]
    frame = next(iter(Frames(packets, fields=[fields])))[0]
    assert frame is not None
    assert set(frame.fields) == {ChanField.FLAGS}
    assert frame.field(ChanField.FLAGS).dtype == np.uint8

    with pytest.raises(IndexError):
        frame.field(ChanField.RANGE)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_frames_bad_type(packets: core.PacketSource) -> None:
    """Test batching frames with a type too small for source data."""
    fields = [FieldType(ChanField.RANGE, np.uint16)]

    with pytest.raises(ValueError):
        next(iter(Frames(packets, fields=[fields])))


@pytest.mark.parametrize('test_key', ['legacy-2.0', 'legacy-2.1'])
def test_frames_raw(packets: core.PacketSource) -> None:
    """Smoke test reading raw channel field data."""
    fields = [
        FieldType(ChanField.RAW32_WORD1, np.uint32),
        FieldType(ChanField.RAW32_WORD2, np.uint32),
        FieldType(ChanField.RAW32_WORD3, np.uint32)
    ]

    frames = Frames(packets, fields=[fields])

    ls = list(frames)
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


def test_voxel_downsample():
    from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy

    in_pts = np.array([[0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 2.0, 0.0], [0.0, 2.0, 0.0]])
    # Two float64 scalar attributes, one array each.
    in_a = np.array([10.0, 12.0, 20.0, 22.0])
    in_b = np.array([100.0, 102.0, 200.0, 202.0])

    tests = []
    tests.append((0.1, [[0.0, 2.0, 0.0], [0.0, 1.0, 0.0]]))
    tests.append((4.0, [[0.0, 1.5, 0.0]]))

    for voxel_size, expected_pts in tests:
        if len(expected_pts) == 2:
            expected_attrs = [[21.0, 201.0], [11.0, 101.0]]
        else:
            expected_attrs = [[16.0, 151.0]]

        out_pts, (out_a, out_b) = voxel_downsample(
            in_pts, voxel_size, 1, 1, VoxelDownsampleStrategy.AVERAGE_POINT, [in_a, in_b])
        assert out_a.dtype == np.float64 and out_b.dtype == np.float64
        assert array_equal_order_independent(out_pts, expected_pts)
        assert array_equal_order_independent(np.column_stack([out_a, out_b]), expected_attrs)


def test_voxel_downsample_first_vs_random():
    from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy

    # All four points land in the same voxel (voxel_size=10). Each carries a
    # unique attribute so we can tell which one the strategy kept.
    xyz = np.array([
        [1.0, 1.0, 1.0],
        [1.5, 1.5, 1.5],
        [2.0, 2.0, 2.0],
        [2.5, 2.5, 2.5],
    ], dtype=np.float64)
    ids = np.array([100, 200, 300, 400], dtype=np.uint32)

    # FIRST_N_POINT with max_points_per_voxel=1 must always keep the first
    # point inserted into the voxel and ignore later arrivals.
    first_xyz, (first_ids,) = voxel_downsample(
        xyz, 10.0, 1, 1, VoxelDownsampleStrategy.FIRST_N_POINT, [ids])
    assert first_xyz.shape == (1, 3)
    np.testing.assert_allclose(first_xyz[0], [1.0, 1.0, 1.0], atol=1e-4)
    assert first_ids.tolist() == [100]

    # Repeating should be deterministic.
    first_xyz2, (first_ids2,) = voxel_downsample(
        xyz, 10.0, 1, 1, VoxelDownsampleStrategy.FIRST_N_POINT, [ids])
    np.testing.assert_array_equal(first_xyz, first_xyz2)
    np.testing.assert_array_equal(first_ids, first_ids2)

    # RANDOM uses Algorithm R reservoir sampling: any of the input points can
    # be the survivor. Feed many same-voxel points (50) so the reservoir has
    # enough turnover to statistically *not* always keep the first one.
    base_xyz = np.full((50, 3), 1.0)
    base_ids = np.arange(50, dtype=np.uint32)
    survivors = set()
    rng = random.Random(0)
    for _ in range(20):
        order = list(range(base_xyz.shape[0]))
        rng.shuffle(order)
        _, (kept,) = voxel_downsample(
            base_xyz[order], 10.0, 1, 1, VoxelDownsampleStrategy.RANDOM, [base_ids[order]])
        survivors.add(int(kept[0]))
    assert survivors.issubset(set(range(50)))
    assert len(survivors) > 1, "RANDOM k=1 should not always pick the same input point"


def test_voxel_downsample_last_n_point_evicts_the_oldest_once_full():
    from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy, VHMInternalStorageMode

    # voxel_size=10, max_points_per_voxel=2 -> near-duplicate rejection radius
    # is voxel_size / sqrt(max_points_per_voxel) ~= 7.07, so points placed near
    # four corners of one 10-unit voxel (pairwise distance >= 9) are never
    # rejected as near-duplicates of one another. ABS storage keeps positions
    # exact rather than quantized, so they can be compared directly.
    xyz = np.array([
        [0.5, 0.5, 0.5],
        [9.5, 9.5, 9.5],
        [0.5, 9.5, 0.5],
        [9.5, 0.5, 9.5],
    ])

    # FIRST_N_POINT: once full (after the first 2), later arrivals are dropped.
    first_xyz, _ = voxel_downsample(
        xyz, 10.0, max_points_per_voxel=2, strategy=VoxelDownsampleStrategy.FIRST_N_POINT,
        internal_storage_mode=VHMInternalStorageMode.ABS)
    np.testing.assert_allclose(np.sort(first_xyz, axis=0), np.sort(xyz[:2], axis=0))

    # LAST_N_POINT: the same voxel instead keeps the last 2 admitted.
    last_xyz, _ = voxel_downsample(
        xyz, 10.0, max_points_per_voxel=2, strategy=VoxelDownsampleStrategy.LAST_N_POINT,
        internal_storage_mode=VHMInternalStorageMode.ABS)
    np.testing.assert_allclose(np.sort(last_xyz, axis=0), np.sort(xyz[-2:], axis=0))

    # The near-duplicate filter still applies to LAST_N_POINT: a point close
    # to one already kept is dropped rather than evicting anything.
    near_duplicate_xyz = np.vstack([xyz[:2], [[0.6, 0.5, 0.5]]])
    dup_xyz, _ = voxel_downsample(
        near_duplicate_xyz, 10.0, max_points_per_voxel=2,
        strategy=VoxelDownsampleStrategy.LAST_N_POINT,
        internal_storage_mode=VHMInternalStorageMode.ABS)
    np.testing.assert_allclose(np.sort(dup_xyz, axis=0), np.sort(xyz[:2], axis=0))


def test_voxel_downsample_smallest_and_largest_strategies():
    from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy

    # Four points share one voxel with RANGE 100, 50, 200, 10 (insertion
    # order); a fifth sits alone in a second voxel with RANGE 999.
    xyz = np.array([
        [0.1, 0.1, 0.1],
        [0.15, 0.15, 0.15],
        [0.2, 0.2, 0.2],
        [0.25, 0.25, 0.25],
        [5.1, 5.1, 5.1],
    ])
    ranges = np.array([100, 50, 200, 10, 999], dtype=np.uint32)

    # RANGE is attribute 0. With max_points_per_voxel=2 the crowded voxel
    # keeps only its two smallest (SMALLEST) or largest (LARGEST) ranges.
    out_xyz, (out_ranges,) = voxel_downsample(
        xyz, 1.0, max_points_per_voxel=2, strategy=VoxelDownsampleStrategy.SMALLEST,
        attributes=[ranges], attribute_index=0)
    assert out_xyz.shape == (3, 3)
    assert sorted(out_ranges.tolist()) == [10, 50, 999]

    out_xyz2, (out_ranges2,) = voxel_downsample(
        xyz, 1.0, max_points_per_voxel=2, strategy=VoxelDownsampleStrategy.LARGEST,
        attributes=[ranges], attribute_index=0)
    assert out_xyz2.shape == (3, 3)
    assert sorted(out_ranges2.tolist()) == [100, 200, 999]

    # attribute_index defaults to 0 (the first attribute), so omitting it
    # gives the same result as passing it explicitly above.
    default_xyz, (default_ranges,) = voxel_downsample(
        xyz, 1.0, max_points_per_voxel=2, strategy=VoxelDownsampleStrategy.SMALLEST,
        attributes=[ranges])
    np.testing.assert_array_equal(np.sort(default_xyz, axis=0), np.sort(out_xyz, axis=0))
    assert sorted(default_ranges.tolist()) == sorted(out_ranges.tolist())

    # Not tied to uint32/RANGE: any scalar dtype works, e.g. float32.
    signal = np.array([3.5, 1.5, 2.5, 9.5, -1.0], dtype=np.float32)
    _, (out_signal,) = voxel_downsample(
        xyz, 1.0, max_points_per_voxel=1, strategy=VoxelDownsampleStrategy.SMALLEST,
        attributes=[signal], attribute_index=0)
    assert out_signal.dtype == np.float32
    assert 1.5 in out_signal.tolist()

    # attribute_index naming a non-scalar attribute (RGB, 3 elements): raises.
    rgb = np.tile(np.array([0.1, 0.2, 0.3], dtype=np.float32), (5, 1))
    with pytest.raises(ValueError, match="scalar"):
        voxel_downsample(xyz, 1.0, strategy=VoxelDownsampleStrategy.SMALLEST,
                         attributes=[rgb, ranges], attribute_index=0)

    # The default (0) is validated too, not silently accepted: RGB first.
    with pytest.raises(ValueError, match="scalar"):
        voxel_downsample(xyz, 1.0, strategy=VoxelDownsampleStrategy.SMALLEST,
                         attributes=[rgb, ranges])

    # An out-of-range attribute_index raises ValueError (not IndexError, which
    # AttributeLayout.byte_offset()'s own bounds check would otherwise surface).
    with pytest.raises(ValueError, match="attribute_index"):
        voxel_downsample(xyz, 1.0, strategy=VoxelDownsampleStrategy.SMALLEST,
                         attributes=[ranges], attribute_index=5)

    # Positions only (no attributes given) has nothing to rank by.
    with pytest.raises(ValueError, match="voxel_downsample_xd"):
        voxel_downsample(xyz, 1.0, strategy=VoxelDownsampleStrategy.SMALLEST)
    with pytest.raises(ValueError, match="voxel_downsample_xd"):
        voxel_downsample(xyz, 1.0, strategy=VoxelDownsampleStrategy.LARGEST)


def test_voxel_downsample_keeps_attribute_dtypes():
    from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy

    # Points 0 and 1 share a voxel; point 2 sits alone. The uint64 value is
    # beyond a double's 53-bit mantissa, so it only survives if nothing widens it.
    xyz = np.array([[0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [5.1, 5.1, 5.1]])
    ranges = np.array([2**64 - 1, 2**64 - 3, 7], dtype=np.uint64)
    reflectivity = np.array([[250], [4], [9]], dtype=np.uint8)  # (N, 1) is scalar
    rgb = np.array([[0.1, 0.2, 0.3], [0.3, 0.4, 0.5], [1.0, 1.0, 1.0]], dtype=np.float32)

    out_xyz, (out_ranges, out_refl, out_rgb) = voxel_downsample(
        xyz, 1.0, strategy=VoxelDownsampleStrategy.FIRST_N_POINT,
        attributes=[ranges, reflectivity, rgb])
    assert out_xyz.shape == (2, 3)
    assert out_ranges.dtype == np.uint64 and out_ranges.shape == (2,)
    assert out_refl.dtype == np.uint8 and out_refl.shape == (2,)
    assert out_rgb.dtype == np.float32 and out_rgb.shape == (2, 3)
    near = int(np.argmin(out_xyz[:, 0]))
    assert out_ranges[near] == 2**64 - 1
    assert out_refl[near] == 250
    np.testing.assert_array_equal(out_rgb[near], rgb[0])

    # Half precision (np.float16) is a supported attribute type too.
    signal = np.array([1.5, -0.25, 3.0], dtype=np.float16)
    out_xyz_f16, (out_signal,) = voxel_downsample(
        xyz, 1.0, strategy=VoxelDownsampleStrategy.FIRST_N_POINT, attributes=[signal])
    assert out_signal.dtype == np.float16
    near_f16 = int(np.argmin(out_xyz_f16[:, 0]))
    assert out_signal[near_f16] == signal[0]

    # AVERAGE_POINT rounds integer means; float means stay exact.
    _, (avg_ranges, avg_refl, avg_rgb) = voxel_downsample(
        xyz, 1.0, strategy=VoxelDownsampleStrategy.AVERAGE_POINT,
        attributes=[np.array([10, 11, 7], dtype=np.uint32), reflectivity, rgb])
    assert avg_ranges.dtype == np.uint32
    assert sorted(avg_ranges.tolist()) == [7, 11]   # 10.5 rounds to 11
    assert sorted(avg_refl.tolist()) == [9, 127]


def test_voxel_downsample_attributes_are_optional_and_validated():
    from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy

    rng = np.random.default_rng(0)
    xyz = rng.uniform(-5, 5, size=(500, 3))
    ranges = rng.integers(0, 2**32, size=500, dtype=np.uint32)
    for strategy in (VoxelDownsampleStrategy.FIRST_N_POINT, VoxelDownsampleStrategy.RANDOM,
                     VoxelDownsampleStrategy.AVERAGE_POINT):
        # Attributes ride along without changing which points are kept.
        typed_xyz, (typed_ranges,) = voxel_downsample(xyz, 1.0, 2, 1, strategy, [ranges])
        # Positions only: attributes left out, None, or an empty list.
        plain_xyz, no_attributes = voxel_downsample(xyz, 1.0, 2, 1, strategy)
        assert no_attributes == []
        for no_attrs in (None, []):
            same_xyz, _ = voxel_downsample(xyz, 1.0, 2, 1, strategy, no_attrs)
            np.testing.assert_array_equal(same_xyz, plain_xyz)
        np.testing.assert_array_equal(np.sort(typed_xyz, axis=0), np.sort(plain_xyz, axis=0))
        assert typed_ranges.dtype == np.uint32 and typed_ranges.shape == (typed_xyz.shape[0],)
        if strategy != VoxelDownsampleStrategy.AVERAGE_POINT:
            # Kept points carry their own, unmodified range.
            assert set(typed_ranges.tolist()) <= set(ranges.tolist())

    with pytest.raises(ValueError, match="unsupported dtype"):
        voxel_downsample(xyz, 1.0, attributes=[np.zeros(500, dtype=np.complex64)])
    with pytest.raises(ValueError, match="one row per point"):
        voxel_downsample(xyz, 1.0, attributes=[np.zeros(499, dtype=np.uint32)])
    with pytest.raises(ValueError, match="Nx3"):
        voxel_downsample(np.zeros((5, 4)), 1.0)


def test_voxel_downsample_is_the_only_python_downsampler():
    import ouster.sdk.core as core
    from ouster.sdk._bindings import client

    assert hasattr(core, "voxel_downsample")
    for removed in ("voxel_downsample_xd", "voxel_downsample_3d"):
        assert not hasattr(core, removed)
        assert not hasattr(client, removed)


@pytest.mark.parametrize("n_attributes", [2, 1, 0])
def test_voxel_downsample_migration_from_nxd_frame(n_attributes):
    # The recipe documented in docs/migration/migration-1.0.0-2.0.0.rst for callers
    # of the old `voxel_downsample_xd(frame, voxel_size, ...)` form.
    from ouster.sdk.core import voxel_downsample, VoxelDownsampleStrategy

    frame = np.array([[0.1, 0.1, 0.1, 10.0, 100.0], [0.2, 0.2, 0.2, 12.0, 102.0],
                      [5.1, 5.1, 5.1, 20.0, 200.0], [5.2, 5.2, 5.2, 22.0, 202.0]])
    frame = frame[:, :3 + n_attributes]
    voxel_size = 1.0

    # [doc-stag-voxel-downsample-xd-migration-py]
    # Before (1.0): out = voxel_downsample_xd(frame, voxel_size, strategy=...)
    # After (2.0): positions and attributes travel separately; one (N, K)
    # float64 array carries every attribute column after x, y, z.
    attributes = [frame[:, 3:]] if frame.shape[1] > 3 else None
    xyz, attributes = voxel_downsample(frame[:, :3], voxel_size,
                                       strategy=VoxelDownsampleStrategy.AVERAGE_POINT,
                                       attributes=attributes)
    out = np.column_stack([xyz, *attributes])  # the old Mx(3+K) layout
    # [doc-etag-voxel-downsample-xd-migration-py]

    assert out.shape == (2, 3 + n_attributes)
    out = out[np.argsort(out[:, 0])]
    np.testing.assert_allclose(out[:, :3], [[0.15] * 3, [5.15] * 3], atol=1e-4)
    expected_attributes = np.array([[11.0, 101.0], [21.0, 201.0]])[:, :n_attributes]
    np.testing.assert_allclose(out[:, 3:], expected_attributes)


@pytest.mark.parametrize("strategy", ["FIRST_N_POINT", "AVERAGE_POINT", "RANDOM"])
def test_voxel_hash_map_proxy_with_and_without_attributes(strategy):
    from ouster.sdk.core import VoxelHashMap, VoxelDownsampleStrategy

    strategy = getattr(VoxelDownsampleStrategy, strategy)
    rng = np.random.default_rng(3)
    xyz = rng.uniform(-4, 4, size=(400, 3))
    ranges = rng.integers(0, 2**32, size=400, dtype=np.uint32)

    # No attribute layout: positions only.
    plain = VoxelHashMap.create(1.0, 2, 1, strategy)
    assert plain.strategy == strategy and plain.point_cols == 3
    assert len(plain.attribute_layout) == 0
    plain.add_points(xyz)

    # With one: attributes ride along in their own dtype.
    attributed = VoxelHashMap.create(1.0, 2, 1, strategy, [(np.uint32, 1)])
    assert attributed.point_cols == 4
    attributed.add_points(xyz, [ranges])
    got_xyz, (got_ranges,) = attributed.pointcloud()
    assert got_ranges.dtype == np.uint32 and got_ranges.shape == (got_xyz.shape[0],)
    if strategy != VoxelDownsampleStrategy.AVERAGE_POINT:
        assert set(got_ranges.tolist()) <= set(ranges.tolist())

    # Carrying attributes doesn't change which positions are kept.
    np.testing.assert_allclose(np.sort(got_xyz, axis=0), np.sort(plain.pointcloud().xyz, axis=0))


def test_voxel_hash_map_proxy_api():
    from ouster.sdk.core import AttributedPointCloud, VoxelHashMap, VoxelDownsampleStrategy

    # Defaults follow voxel_downsample().
    proxy = VoxelHashMap.create(0.5)
    assert proxy.strategy == VoxelDownsampleStrategy.RANDOM
    assert proxy.max_points_per_voxel == 1 and proxy.min_points_per_voxel == 1
    assert proxy.voxel_size == 0.5 and proxy.empty

    proxy.add_points(np.array([[0.1, 0.1, 0.1], [200.1, 0.1, 0.1]]))
    point, dist_sq = proxy.get_closest_neighbor(np.array([0.1, 0.1, 0.1]))
    assert point.shape == (3,) and dist_sq < 1e-3
    far_xyz, far_attributes = proxy.extract_voxels_far_from_location(np.zeros(3))
    assert far_xyz.shape == (1, 3) and far_attributes == []
    cloud = proxy.pointcloud()
    assert isinstance(cloud, AttributedPointCloud)
    assert cloud.xyz.shape == (1, 3) and cloud.attributes == []
    # It also unpacks, and indexes, like an (xyz, attributes) pair.
    xyz, attributes = cloud
    assert xyz is not None and attributes == []
    # len() is the number of points, as it was for the 1.0 array.
    assert len(cloud) == cloud.xyz.shape[0] == 1
    np.testing.assert_array_equal(cloud[0], cloud.xyz)
    with pytest.raises(IndexError):
        cloud[2]
    with pytest.raises(ValueError, match="unexpected point dimension"):
        proxy.add_points(np.zeros((2, 4)))
    proxy.clear()
    assert proxy.empty

    with pytest.raises(ValueError):
        VoxelHashMap.create(0.0)
    with pytest.raises(ValueError, match="unsupported attribute dtype"):
        VoxelHashMap.create(1.0, attribute_layout=[(np.complex64, 1)])

    # Half precision (np.float16) is a supported attribute type.
    half_map = VoxelHashMap.create(1.0, attribute_layout=[(np.float16, 1)])
    assert half_map.point_cols == 4
    half_map.add_points(np.array([[0.1, 0.1, 0.1]]),
                        [np.array([1.5], dtype=np.float16)])
    half_xyz, (half_values,) = half_map.pointcloud()
    assert half_xyz.shape == (1, 3)
    assert half_values.dtype == np.float16
    assert half_values[0] == np.float16(1.5)


def test_voxel_hash_map_proxy_update():
    from ouster.sdk.core import VoxelHashMap, VoxelDownsampleStrategy

    proxy = VoxelHashMap.create(1.0, 10, 1, VoxelDownsampleStrategy.RANDOM)
    # update() inserts points, then trims voxels far from position (default
    # max_distance=100.0), in one call.
    proxy.update(np.array([[0.1, 0.1, 0.1], [250.1, 0.1, 0.1]]), np.array([0.1, 0.1, 0.1]))
    assert proxy.pointcloud().xyz.shape == (1, 3)

    # Every point is checked against point_cols() before any is inserted.
    proxy.clear()
    with pytest.raises(ValueError):
        proxy.update(np.zeros((2, 2)), np.zeros(3))
    assert proxy.empty


def test_voxel_downsample_storage_mode_and_empty_input_validation():
    from ouster.sdk.core import (voxel_downsample, VoxelDownsampleStrategy,
                                 VHMInternalStorageMode)

    # One point per voxel, so every point is kept and any quantization shows.
    xyz = np.array([[0.01234567, 0.04567891, 0.07891234],
                    [0.51234567, 0.54567891, 0.57891234],
                    [1.01234567, 1.04567891, 1.07891234]])
    ranges = np.array([1, 2, 3], dtype=np.uint32)
    first = VoxelDownsampleStrategy.FIRST_N_POINT
    abs_mode = VHMInternalStorageMode.ABS

    def by_x(points):
        return points[np.argsort(points[:, 0])]

    # ABS returns the input positions exactly; the AUTO default quantizes.
    plain_xyz, _ = voxel_downsample(xyz, 0.1, strategy=first, internal_storage_mode=abs_mode)
    np.testing.assert_array_equal(by_x(plain_xyz), xyz)
    out_xyz, (out_ranges,) = voxel_downsample(xyz, 0.1, strategy=first, attributes=[ranges],
                                              internal_storage_mode=abs_mode)
    np.testing.assert_array_equal(by_x(out_xyz), xyz)
    quantized = by_x(voxel_downsample(xyz, 0.1, strategy=first)[0])
    assert not np.array_equal(quantized, xyz)
    np.testing.assert_allclose(quantized, xyz, atol=0.1 / 64)

    # Bad arguments are rejected even when there is nothing to downsample.
    empty = np.zeros((0, 3))
    with pytest.raises(ValueError):
        voxel_downsample(empty, 0.0)
    with pytest.raises(ValueError):
        voxel_downsample(empty, 0.0, attributes=[np.zeros(0, dtype=np.uint32)])
    assert voxel_downsample(empty, 1.0)[0].shape == (0, 3)

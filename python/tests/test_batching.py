"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from copy import deepcopy
from typing import Iterator

from more_itertools import take
import numpy as np
import pytest

from ouster.sdk import core
from ouster.sdk.core import PacketFormat, \
    FieldType, FrameBatcher, LidarFrame, ImuPacket, LidarPacket, SensorInfo
from ouster.sdk.pcap import PcapPacketSource
from ouster.sdk.core import ShotLimitingStatus, ThermalShutdownStatus


CUSTOM0 = 'custom0'
CUSTOM8 = 'custom8'


def _patch_frame_id(packet: core.LidarPacket, fid: int) -> None:
    """Rewrite the frame id of a non-legacy format lidar packet."""
    packet.buf[2:4] = memoryview(fid.to_bytes(2, byteorder='little'))


def _valid_packet_idxs(frame: LidarFrame) -> np.ndarray:
    """Checks for valid packets that was used in in the frame construction"""
    valid_cols = frame.status & 0x1
    valid_packet_ts = frame.packet_timestamp != 0
    sp = np.split(valid_cols, frame.packet_timestamp.shape[0])
    # here we consider the packet is valid when either one is true:
    #   - any columns in the packet has a valid status
    #   - packet_timestamp is not zero, which may occur even when
    #     all columns/px data in invalid state within the packet.
    #     It means that we received the packet without per px data
    #     but with all other headers in place
    valid_packets = np.logical_or(np.any(sp, axis=1), valid_packet_ts)
    return np.nonzero(valid_packets)[0]


@pytest.fixture
def lidar_stream(packets: core.PacketSource) -> core.PacketSource:
    """Infinite stream of lidar packets with spoofed frame ids and packets ts."""
    def gen_packets():
        frame_id = 0
        next_ts = -1
        packets_per_frame = (packets.sensor_info[0].format.columns_per_frame //
                             packets.sensor_info[0].format.columns_per_packet)
        dt = 1 / (packets.sensor_info[0].format.fps * packets_per_frame)
        while True:
            plist = deepcopy(list(packets))
            for idx, p in plist:
                if isinstance(p, core.LidarPacket):
                    if next_ts < 0:
                        next_ts = p.host_timestamp
                    _patch_frame_id(p, frame_id)
                    p.host_timestamp = int(next_ts)
                    yield p
                    next_ts += dt * 1e9
            frame_id += 1

    return core.Packets(gen_packets(), packets.sensor_info[0])


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_batch_missing_zeroed(lidar_stream: core.PacketSource) -> None:
    """Check that missing data is zeroed out when batching."""

    info = lidar_stream.sensor_info[0]
    print("info was", info)

    packets_per_frame = (info.format.columns_per_frame //
                         info.format.columns_per_packet)

    batcher = FrameBatcher(info)

    ls = core.LidarFrame(info)
    assert ls.packet_timestamp.shape == (packets_per_frame,)

    def non_zero_frame(frame: core.LidarFrame):
        frame.timestamp[:] = 1
        frame.measurement_id[:] = 1
        frame.status[:] = 1
        frame.packet_timestamp[:] = 1
        for f in frame.fields:
            frame.field(f)[:] = 1

    # initialize fields and headers with nonzero values
    non_zero_frame(ls)

    # packet indices to drop
    drop_inds = [10, 20, 63]

    # column indexes should be in [0, info.format.columns_per_packet - 1] range
    # columns_per_packet in these tests is 16, as defined byt the seed pcap + json
    drop_columns = {
        2: [0, 1, 2],
        3: [12, 13, 14, 15],
        5: [15],
        13: range(15),
        26: range(16)
    }
    drop_columns_num = sum([len(set(v)) for v in drop_columns.values()])

    # number of frames we want to get from packets
    num_frames = 3

    # drop some packets, and +1 to ensure the "cut" of the frame by batcher
    packets = list(take(packets_per_frame * num_frames + 1, lidar_stream))

    packet_format = PacketFormat(info)

    # parse the packets into a frame
    def frames():
        # reusing the same lidar frame object over and over
        for ind, pb in enumerate(packets):
            p = pb[1]
            packet_ind = ind % packets_per_frame
            if packet_ind not in drop_inds:
                if packet_ind in drop_columns:
                    # invalidating columns
                    # column is invalid if first bit of status != 1
                    for col in drop_columns[packet_ind]:
                        packet_format.set_col_status(p, col, 0)

                if batcher.batch(p, ls):
                    # when batch returns True it means
                    # that the outstanding and missed packets
                    # in the frame should be zeroed properly
                    yield ls

                    # intentionally make all dirty before we batch
                    # packets into the frame again
                    non_zero_frame(ls)

    # check that data associated with the dropped packets is zeroed
    n = info.format.columns_per_packet

    for frame in frames():
        assert (np.count_nonzero(frame.status) == info.format.columns_per_frame -
                n * len(drop_inds) - drop_columns_num)

        for i in drop_inds:
            assert (frame.timestamp[i * n:(i + 1) * n] == 0).all()
            assert (frame.measurement_id[i * n:(i + 1) * n] == 0).all()
            assert (frame.status[i * n:(i + 1) * n] == 0).all()
            assert (frame.packet_timestamp[i:i + 1] == 0).all()

            for f in frame.fields:
                assert (frame.field(f)[:, i * n:n] == 0).all()

        valid_packets = _valid_packet_idxs(frame)
        assert (frame.packet_timestamp[valid_packets] != 0).all()

        # valid packets timestamps should be non-decreasing
        for i, j in zip(valid_packets[:-1], valid_packets[1:]):
            assert frame.packet_timestamp[i] <= frame.packet_timestamp[j]


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_batch_custom_fields(lidar_stream: core.PacketSource) -> None:
    """Test batching of a LidarFrame with custom fields set."""
    print("start")
    info = lidar_stream.sensor_info[0]

    packets_per_frame = (info.format.columns_per_frame //
                         info.format.columns_per_packet)

    batcher = FrameBatcher(info)
    packet_format = PacketFormat(info)

    # create LidarFrame with only 2 fields
    fields = [
        FieldType(core.ChanField.RANGE, np.uint32),
        FieldType(core.ChanField.SIGNAL, np.uint16),
        FieldType(CUSTOM0, np.uint8),
        FieldType(CUSTOM8, np.uint16)
    ]

    ls = core.LidarFrame(info, fields)

    # we expect zero initialized fields
    for f in ls.fields:
        assert np.count_nonzero(ls.field(f)) == 0

    # set non zero data into users' custom field
    ls.field(CUSTOM8)[:] = 8

    # do batching into ls with a fields subset
    for idx, p in take(packets_per_frame, lidar_stream):
        if isinstance(p, core.LidarPacket):
            batcher.batch(p, ls)
            assert ShotLimitingStatus(packet_format.shot_limiting(p.buf)) == ls.shot_limiting()
            assert ThermalShutdownStatus(packet_format.thermal_shutdown(p.buf)) == ls.thermal_shutdown()

    # it should contain the same num fields as we've added
    assert len(list(ls.fields)) == len(fields)

    # and the content shouldn't be zero after batching
    for f in ls.fields:
        if f in [core.ChanField.RANGE, core.ChanField.SIGNAL]:
            assert np.count_nonzero(ls.field(f)) > 0

    # custom field data should be preserved after batching
    assert np.all(ls.field(CUSTOM0) == 0)
    assert np.all(ls.field(CUSTOM8) == 8)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_incompatible_profile(lidar_stream: core.PacketSource) -> None:
    """Test batching of a LidarFrame with custom fields set."""

    info = lidar_stream.sensor_info[0]
    assert info.format.udp_profile_lidar == core.UDPProfileLidar.LEGACY

    packets_per_frame = (info.format.columns_per_frame //
                         info.format.columns_per_packet)

    batcher = FrameBatcher(info)

    fields = [
        FieldType(core.ChanField.RANGE, np.uint8)
    ]
    ls = core.LidarFrame(info, fields)

    # Test for decoding frames to a bad dest buffer type
    with pytest.raises(ValueError):
        for idx, p in take(packets_per_frame, lidar_stream):
            if isinstance(p, core.LidarPacket):
                batcher.batch(p, ls)


@pytest.fixture
def lidar_stream_with_lagging_frame_ids(packets: core.PacketSource) -> core.PacketSource:
    """A stream of lidar packets with spoofed out of order frame ids in proximity
    to the sensor frame_id wrap-around values."""
    def gen_packets():
        s = np.iinfo(np.ushort).max
        ids = [s, s, 0, s, s, s, 0, 0, s, s, 0, 1, 1]
        idx = 0
        frame_id = ids[idx]
        while True:
            plist = deepcopy(list(packets))
            for sensor_idx, p in plist:
                if isinstance(p, core.LidarPacket):
                    _patch_frame_id(p, frame_id)
                    yield p
            idx += 1
            frame_id = ids[idx % len(ids)]

    return core.Packets(gen_packets(), packets.sensor_info[0])


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_frames_multi_wraparound(lidar_stream_with_lagging_frame_ids: core.PacketSource) -> None:
    """Test FrameBatcher with some packets coming out of order (only lagging case)
    by no more than a single id."""
    def frames_method(source: core.PacketSource) -> Iterator[core.LidarFrame]:
        metadata = source.sensor_info[0]
        ls = LidarFrame(metadata)
        batcher = FrameBatcher(metadata)
        for i, s in source:
            if isinstance(s, LidarPacket):
                if batcher.batch(s, ls):
                    yield ls
                    ls = LidarFrame(metadata)
    frames = take(3, frames_method(lidar_stream_with_lagging_frame_ids))
    assert list(map(lambda s: s.frame_id, frames)) == [65535, 0, 1]


@pytest.mark.parametrize('file', ['OS-0-32-U1_v2.2.0_1024x10.pcap', 'windowed_frame1.pcap', 'windowed_frame2.pcap'])
def test_early_release(file: str, test_data_dir):
    """ Verify that frame batcher releases a complete frame without waiting for a packet
    from the next one"""
    path = test_data_dir / "pcaps" / file
    source = PcapPacketSource(str(path))

    metadata = source.sensor_info[0]
    ls = LidarFrame(metadata)
    batcher = FrameBatcher(metadata)
    packet_format = PacketFormat(metadata)
    completed = False
    for i, s in source:
        if isinstance(s, LidarPacket):
            frame_id = packet_format.frame_id(s.buf)
            if batcher.batch(s, ls):
                completed = True
                assert ls.frame_id == frame_id
                assert ls.complete(metadata.format.column_window)
    assert completed


def test_batching_alerts():
    """It should include alert_flags from the packet in the LidarFrame."""
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 1024
    info.format.columns_per_packet = 16
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.udp_profile_imu = core.UDPProfileIMU.LEGACY
    info.format.header_type = core.HeaderType.STANDARD
    packet_format = PacketFormat(info)
    batcher = FrameBatcher(info)
    frame = LidarFrame(info)
    frame.frame_id = 0

    num_packets = info.format.columns_per_frame // info.format.columns_per_packet
    for packet_id in range(num_packets):

        # create a packet and set the measurement id of the first column, which
        # FrameBatcher uses to determine which packet in the frame it is
        packet = LidarPacket(packet_format)
        packet_format.set_col_measurement_id(packet, 0, packet_id * info.format.columns_per_packet)

        # set the alert flag value to the packet id (just so we have something to test.)
        packet_format.set_alert_flags(packet, packet_id)
        assert packet_format.alert_flags(packet.buf) == packet_id
        batcher.batch(packet, frame)

    # confirm that the alert flag values are increasing from 0..63 (the same as the packet ids)
    assert np.array_equal(frame.alert_flags, np.array(range(num_packets)))


def test_batching_bad_column_id():
    """It should include alert_flags from the packet in the LidarFrame."""
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.column_window = [0, 1023]
    info.format.columns_per_frame = 1024
    info.format.columns_per_packet = 16
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.udp_profile_imu = core.UDPProfileIMU.LEGACY
    info.format.header_type = core.HeaderType.STANDARD
    packet_format = PacketFormat(info)
    batcher = FrameBatcher(info)
    frame = LidarFrame(info)
    frame.frame_id = 0

    # create a packet that has valid columns, but ones too close
    # to block parsing boundaries to make sure we dont crash
    # it should fail over to column parsing instead
    packet = LidarPacket(packet_format)
    packet.host_timestamp = 1
    for i in range(info.format.columns_per_packet):
        packet_format.set_col_status(packet, i, 1)
        packet_format.set_col_measurement_id(packet, i, 1020)

    batcher.batch(packet, frame)

    # it should still parse the column, without crashing
    assert frame.status[1020] == 1
    assert frame.status[1021] == 0


def test_batching_countdowns():
    """It should include shutdown_countdown and shot_limiting_countdown in the LidarFrame."""
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 1024
    info.format.columns_per_packet = 16
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.udp_profile_imu = core.UDPProfileIMU.LEGACY
    info.format.header_type = core.HeaderType.STANDARD
    packet_format = PacketFormat(info)
    batcher = FrameBatcher(info)
    frame = LidarFrame(info)
    frame.frame_id = -1

    num_packets = info.format.columns_per_frame // info.format.columns_per_packet
    for packet_id in range(num_packets):
        packet = LidarPacket(packet_format)
        # According to FW, the values from each packet will be the same within a given frame
        packet_format.set_shutdown_countdown(packet, 30)
        packet_format.set_shot_limiting_countdown(packet, 29)
        batcher.batch(packet, frame)

    assert frame.shutdown_countdown == packet_format.countdown_thermal_shutdown(packet.buf)
    assert frame.shot_limiting_countdown == packet_format.countdown_shot_limiting(packet.buf)


def test_batching_dups():
    """It handles a good packet stream with packets in the expected order."""
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 4
    info.format.columns_per_packet = 2
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.imu_packets_per_frame = 1
    info.format.udp_profile_imu = core.UDPProfileIMU.ACCEL32_GYRO32_NMEA
    info.format.header_type = core.HeaderType.STANDARD
    info.format.column_window = [0, info.format.columns_per_frame - 1]

    assert info.format.lidar_packets_per_frame() == info.format.columns_per_frame // info.format.columns_per_packet

    packet_format = PacketFormat(info)
    batcher = FrameBatcher(info)
    frame = LidarFrame(info)
    frame.frame_id = -1

    def add_packet(packet_type, frame_id, measurement_id):
        packet = packet_type(packet_format)
        packet_format.set_frame_id(packet, frame_id)
        packet.host_timestamp = 100
        if isinstance(packet, LidarPacket):
            packet_format.set_col_measurement_id(packet, 0, measurement_id)
        return batcher.batch(packet, frame)

    assert not add_packet(LidarPacket, 100, 1)
    assert frame.frame_id == 100

    assert not add_packet(LidarPacket, 100, 2)
    assert frame.frame_id == 100

    assert add_packet(ImuPacket, 100, 0)
    assert frame.frame_id == 100

    assert not add_packet(LidarPacket, 101, 1)
    assert frame.frame_id == 101

    assert not add_packet(LidarPacket, 101, 2)
    assert frame.frame_id == 101

    assert add_packet(ImuPacket, 101, 0)
    assert frame.frame_id == 101


def test_batching_dups_2():
    """It handles frame id roll over"""
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 4
    info.format.columns_per_packet = 2
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.imu_packets_per_frame = 1
    info.format.udp_profile_imu = core.UDPProfileIMU.ACCEL32_GYRO32_NMEA
    info.format.header_type = core.HeaderType.STANDARD
    info.format.column_window = [0, info.format.columns_per_frame - 1]

    assert info.format.lidar_packets_per_frame() == info.format.columns_per_frame // info.format.columns_per_packet

    packet_format = PacketFormat(info)
    batcher = FrameBatcher(info)
    frame = LidarFrame(info)
    frame.frame_id = -1

    def add_packet(packet_type, frame_id, measurement_id):
        packet = packet_type(packet_format)
        packet_format.set_frame_id(packet, frame_id)
        packet.host_timestamp = 100
        if isinstance(packet, LidarPacket):
            packet_format.set_col_measurement_id(packet, 0, measurement_id)
        return batcher.batch(packet, frame)

    assert not add_packet(LidarPacket, packet_format.max_frame_id, 1)
    assert frame.frame_id == packet_format.max_frame_id

    assert not add_packet(LidarPacket, packet_format.max_frame_id, 2)
    assert frame.frame_id == packet_format.max_frame_id

    assert add_packet(ImuPacket, packet_format.max_frame_id, 0)
    assert frame.frame_id == packet_format.max_frame_id

    assert not add_packet(LidarPacket, 0, 1)
    assert frame.frame_id == 0

    assert not add_packet(LidarPacket, 0, 2)
    assert frame.frame_id == 0

    assert add_packet(ImuPacket, 0, 0)
    assert frame.frame_id == 0


def test_batching_dups_3():
    """It handles out of order packets, but may drop old packets."""
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 4
    info.format.columns_per_packet = 2
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.imu_packets_per_frame = 1
    info.format.udp_profile_imu = core.UDPProfileIMU.ACCEL32_GYRO32_NMEA
    info.format.header_type = core.HeaderType.STANDARD
    info.format.column_window = [0, info.format.columns_per_frame - 1]

    assert info.format.lidar_packets_per_frame() == info.format.columns_per_frame // info.format.columns_per_packet

    packet_format = PacketFormat(info)
    for cache_size in range(1, 10, 2):
        batcher = FrameBatcher(info)
        batcher.set_max_cache_size(cache_size)
        frame = LidarFrame(info)
        frame.frame_id = -1

        def add_packet(packet_type, frame_id, measurement_id):
            packet = packet_type(packet_format)
            packet_format.set_frame_id(packet, frame_id)
            packet.host_timestamp = 100
            if isinstance(packet, LidarPacket):
                packet_format.set_col_measurement_id(packet, 0, measurement_id)
            return batcher.batch(packet, frame)

        assert not add_packet(LidarPacket, 100, 1)
        assert frame.frame_id == 100

        assert not add_packet(LidarPacket, 100, 2)
        assert frame.frame_id == 100

        # Fill the cache with packets from the next frame
        # to simulate a late packet for the current frame.
        # But the current frame won't be finalized until the cache is full.
        for _ in range(batcher.get_max_cache_size() - 1):
            # this new packet is cached in the batcher and the current frame isn't yet finalized
            assert not add_packet(LidarPacket, 101, 1)
        # this new packet is cached in the batcher and the current frame is finalized
        assert add_packet(LidarPacket, 101, 1)
        assert frame.frame_id == 100  # <--- the frame frame id is still 100

        # the old imu packet is discarded since the frame was already finalized
        assert not add_packet(ImuPacket, 100, 0)
        assert frame.frame_id == 101               # <--- the frame frame id is now 101

        assert not add_packet(LidarPacket, 101, 2)
        assert frame.frame_id == 101

        assert add_packet(ImuPacket, 101, 0)
        assert frame.frame_id == 101


def test_batching_dups_4():
    """Packets from prior frames are ignored, and won't cause a frame with a duplicate id."""
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 8
    info.format.columns_per_packet = 2
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.imu_packets_per_frame = 1
    info.format.udp_profile_imu = core.UDPProfileIMU.ACCEL32_GYRO32_NMEA
    info.format.header_type = core.HeaderType.STANDARD
    info.format.column_window = [0, info.format.columns_per_frame - 1]

    assert info.format.lidar_packets_per_frame() == info.format.columns_per_frame // info.format.columns_per_packet

    packet_format = PacketFormat(info)
    batcher = FrameBatcher(info)
    frame = LidarFrame(info)
    frame.frame_id = -1

    def add_packet(packet_type, frame_id, measurement_id):
        packet = packet_type(packet_format)
        packet_format.set_frame_id(packet, frame_id)
        packet.host_timestamp = 100
        if isinstance(packet, LidarPacket):
            packet_format.set_col_measurement_id(packet, 0, measurement_id)
            packet_format.set_col_measurement_id(packet, 1, measurement_id)
        return batcher.batch(packet, frame)

    assert not add_packet(LidarPacket, 100, 0)
    assert frame.frame_id == 100

    assert not add_packet(ImuPacket, 98, 0)
    assert frame.frame_id == 100

    assert not add_packet(LidarPacket, 100, 2)
    assert frame.frame_id == 100

    assert not add_packet(ImuPacket, 98, 0)
    assert frame.frame_id == 100

    assert not add_packet(LidarPacket, 100, 4)
    assert frame.frame_id == 100

    assert not add_packet(ImuPacket, 98, 0)
    assert frame.frame_id == 100

    assert not add_packet(LidarPacket, 100, 6)
    assert frame.frame_id == 100

    assert add_packet(ImuPacket, 100, 0)
    assert frame.frame_id == 100


def test_batching_dups_5():
    """Packets from prior frames are ignored, and won't cause a frame with a duplicate id."""
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 6
    info.format.columns_per_packet = 2
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.imu_packets_per_frame = 0
    info.format.udp_profile_imu = core.UDPProfileIMU.OFF
    info.format.header_type = core.HeaderType.STANDARD
    info.format.column_window = [0, info.format.columns_per_frame - 1]

    assert info.format.lidar_packets_per_frame() == info.format.columns_per_frame // info.format.columns_per_packet

    packet_format = PacketFormat(info)
    batcher = FrameBatcher(info)
    frame = LidarFrame(info)
    frame.frame_id = -1

    def add_packet(packet_type, frame_id, measurement_id):
        packet = packet_type(packet_format)
        packet_format.set_frame_id(packet, frame_id)
        packet.host_timestamp = 100
        if isinstance(packet, LidarPacket):
            packet_format.set_col_measurement_id(packet, 0, measurement_id)
            packet_format.set_col_measurement_id(packet, 1, measurement_id)
        return batcher.batch(packet, frame)

    assert not add_packet(LidarPacket, 100, 0)
    assert frame.frame_id == 100

    assert not add_packet(LidarPacket, 100, 2)
    assert frame.frame_id == 100

    assert not add_packet(LidarPacket, 101, 0)
    assert frame.frame_id == 100

    assert add_packet(LidarPacket, 100, 4)
    assert frame.frame_id == 100


def test_batcher_late_finalization():
    """
    It will still produce frames if they're never complete but the cache is too full.
    """
    profile = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 4
    info.format.columns_per_packet = 2
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    info.format.imu_packets_per_frame = 0
    info.format.udp_profile_imu = core.UDPProfileIMU.OFF
    info.format.header_type = core.HeaderType.STANDARD
    info.format.column_window = [0, info.format.columns_per_frame - 1]

    assert info.format.lidar_packets_per_frame() == info.format.columns_per_frame // info.format.columns_per_packet

    packet_format = PacketFormat(info)
    for cache_size in range(1, 10, 2):
        batcher = FrameBatcher(info)
        batcher.set_max_cache_size(cache_size)
        frame = LidarFrame(info)
        frame.frame_id = -1

        def add_packet(packet_type, frame_id, measurement_id):
            packet = packet_type(packet_format)
            packet_format.set_frame_id(packet, frame_id)
            packet.host_timestamp = 100
            if isinstance(packet, LidarPacket):
                packet_format.set_col_measurement_id(packet, 0, measurement_id)
                packet_format.set_col_measurement_id(packet, 1, measurement_id)
            return batcher.batch(packet, frame)

        for fid in range(100, 100 + batcher.get_max_cache_size()):
            assert not add_packet(LidarPacket, fid, 0)
            assert frame.frame_id == 100

        for fid in range(100 + batcher.get_max_cache_size(), 200):
            assert add_packet(LidarPacket, fid, 0)
            assert frame.frame_id == fid - batcher.get_max_cache_size()

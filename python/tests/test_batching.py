# type: ignore
"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from copy import deepcopy
from typing import Iterator

from more_itertools import take
import numpy as np
import pytest

from ouster.sdk import client
from ouster.sdk.client import PacketFormat, PacketWriter, \
    FieldType, ScanBatcher, LidarScan, LidarPacket, SensorInfo
from ouster.sdk.pcap import PcapMultiPacketReader


client.ChanField.CUSTOM0 = 'custom0'
client.ChanField.CUSTOM8 = 'custom8'


def _patch_frame_id(packet: client.LidarPacket, fid: int) -> None:
    """Rewrite the frame id of a non-legacy format lidar packet."""
    packet.buf[2:4] = memoryview(fid.to_bytes(2, byteorder='little'))


def _simple_scans(source: client.PacketSource) -> Iterator[client.LidarScan]:
    """Batch packets to scans without zeroing between frames."""
    batch = ScanBatcher(source.metadata)
    info = source.metadata

    ls = client.LidarScan(info.format.pixels_per_column,
                          info.format.columns_per_frame,
                          info.format.udp_profile_lidar)

    for p in source:
        if batch(p.buf, ls):
            yield deepcopy(ls)


@pytest.fixture
def lidar_stream(packets: client.PacketSource) -> client.PacketSource:
    """Infinite stream of lidar packets with spoofed frame ids and packets ts."""
    def gen_packets():
        frame_id = 0
        next_ts = -1
        packets_per_frame = (packets.metadata.format.columns_per_frame //
                             packets.metadata.format.columns_per_packet)
        dt = 1 / (packets.metadata.format.fps * packets_per_frame)
        while True:
            plist = deepcopy(list(packets))
            for p in plist:
                if isinstance(p, client.LidarPacket):
                    if next_ts < 0:
                        next_ts = p.host_timestamp
                    _patch_frame_id(p, frame_id)
                    p.host_timestamp = int(next_ts)
                    yield p
                    next_ts += dt * 1e9
            frame_id += 1

    return client.Packets(gen_packets(), packets.metadata)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_multi(lidar_stream: client.PacketSource) -> None:
    """Test batching multiple scans."""
    scans = take(10, client.Scans(lidar_stream))

    assert list(map(lambda s: s.frame_id, scans)) == list(range(10))

    # host timestmps are filled and not zeros
    assert all(map(lambda s: np.all(s.packet_timestamp), scans))


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_batch_missing_zeroed(lidar_stream: client.PacketSource) -> None:
    """Check that missing data is zeroed out when batching."""

    info = lidar_stream.metadata

    packets_per_frame = (info.format.columns_per_frame //
                         info.format.columns_per_packet)

    batch = ScanBatcher(lidar_stream.metadata)

    ls = client.LidarScan(info.format.pixels_per_column,
                          info.format.columns_per_frame,
                          info.format.udp_profile_lidar)
    assert ls.packet_timestamp.shape == (packets_per_frame,)

    def non_zero_scan(scan: client.LidarScan):
        scan.timestamp[:] = 1
        scan.measurement_id[:] = 1
        scan.status[:] = 1
        scan.packet_timestamp[:] = 1
        for f in scan.fields:
            scan.field(f)[:] = 1

    # initialize fields and headers with nonzero values
    non_zero_scan(ls)

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

    # number of scans we want to get from packets
    num_scans = 3

    # drop some packets, and +1 to ensure the "cut" of the scan by batcher
    packets = list(take(packets_per_frame * num_scans + 1, lidar_stream))

    writer = PacketWriter.from_info(info)

    # parse the packets into a scan
    def scans():
        # reusing the same lidar scan object over and over
        nonlocal ls
        for ind, p in enumerate(packets):
            packet_ind = ind % packets_per_frame
            if packet_ind not in drop_inds:
                if packet_ind in drop_columns:
                    # invalidating columns
                    # column is invalid if first bit of status != 1
                    for col in drop_columns[packet_ind]:
                        writer.set_col_status(p, col, 0)

                if batch(p.buf, client.packet_ts(p), ls):
                    # when batch returns True it means
                    # that the outstanding and missed packets
                    # in the scan should be zeroed properly
                    yield ls

                    # intentionally make all dirty before we batch
                    # packets into the scan again
                    non_zero_scan(ls)

    # check that data associated with the dropped packets is zeroed
    n = info.format.columns_per_packet

    for scan in scans():
        assert (np.count_nonzero(scan.status) == info.format.columns_per_frame -
                n * len(drop_inds) - drop_columns_num)

        for i in drop_inds:
            assert (scan.timestamp[i * n:(i + 1) * n] == 0).all()
            assert (scan.measurement_id[i * n:(i + 1) * n] == 0).all()
            assert (scan.status[i * n:(i + 1) * n] == 0).all()
            assert (scan.packet_timestamp[i:i + 1] == 0).all()

            for f in scan.fields:
                assert (scan.field(f)[:, i * n:n] == 0).all()

        valid_packets = client.valid_packet_idxs(scan)
        assert (scan.packet_timestamp[valid_packets] != 0).all()

        # valid packets timestamps should be non-decreasing
        for i, j in zip(valid_packets[:-1], valid_packets[1:]):
            assert scan.packet_timestamp[i] <= scan.packet_timestamp[j]


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_batch_custom_fields(lidar_stream: client.PacketSource) -> None:
    """Test batching of a LidarScan with custom fields set."""

    info = lidar_stream.metadata

    packets_per_frame = (info.format.columns_per_frame //
                         info.format.columns_per_packet)

    batch = ScanBatcher(lidar_stream.metadata)
    pf = PacketFormat(info)

    # create LidarScan with only 2 fields
    fields = [
        FieldType(client.ChanField.RANGE, np.uint32),
        FieldType(client.ChanField.SIGNAL, np.uint16),
        FieldType(client.ChanField.CUSTOM0, np.uint8),
        FieldType(client.ChanField.CUSTOM8, np.uint16)
    ]

    ls = client.LidarScan(info.format.pixels_per_column,
                          info.format.columns_per_frame, fields)

    # we expect zero initialized fields
    for f in ls.fields:
        assert np.count_nonzero(ls.field(f)) == 0

    # set non zero data into users' custom field
    ls.field(client.ChanField.CUSTOM8)[:] = 8

    # do batching into ls with a fields subset
    for p in take(packets_per_frame, lidar_stream):
        batch(p.buf, ls)
        if isinstance(p, client.LidarPacket):
            assert pf.shot_limiting(p.buf) == ls.shot_limiting()
            assert pf.thermal_shutdown(p.buf) == ls.thermal_shutdown()

    # it should contain the same num fields as we've added
    assert len(list(ls.fields)) == len(fields)

    # and the content shouldn't be zero after batching
    for f in ls.fields:
        if f in [client.ChanField.RANGE, client.ChanField.SIGNAL]:
            assert np.count_nonzero(ls.field(f)) > 0

    # custom field data should be preserved after batching
    assert np.all(ls.field(client.ChanField.CUSTOM0) == 0)
    assert np.all(ls.field(client.ChanField.CUSTOM8) == 8)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_incompatible_profile(lidar_stream: client.PacketSource) -> None:
    """Test batching of a LidarScan with custom fields set."""

    info = lidar_stream.metadata
    assert info.format.udp_profile_lidar == client.UDPProfileLidar.PROFILE_LIDAR_LEGACY

    packets_per_frame = (info.format.columns_per_frame //
                         info.format.columns_per_packet)

    batch = ScanBatcher(lidar_stream.metadata)

    fields = [
        FieldType(client.ChanField.RANGE, np.uint8)
    ]
    ls = client.LidarScan(info.format.pixels_per_column,
                          info.format.columns_per_frame,
                          fields)

    # Test for decoding scans to a bad dest buffer type
    with pytest.raises(ValueError):
        for p in take(packets_per_frame, lidar_stream):
            batch(p.buf, ls)


@pytest.fixture
def lidar_stream_with_lagging_frame_ids(packets: client.PacketSource) -> client.PacketSource:
    """A stream of lidar packets with spoofed out of order frame ids in proximity
    to the sensor frame_id wrap-around values."""
    def gen_packets():
        s = np.iinfo(np.ushort).max
        ids = [s, s, 0, s, s, s, 0, 0, s, s, 0, 1, 1]
        idx = 0
        frame_id = ids[idx]
        while True:
            plist = deepcopy(list(packets))
            for p in plist:
                if isinstance(p, client.LidarPacket):
                    _patch_frame_id(p, frame_id)
                    yield p
            idx += 1
            frame_id = ids[idx % len(ids)]

    return client.Packets(gen_packets(), packets.metadata)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_multi_wraparound(lidar_stream_with_lagging_frame_ids: client.PacketSource) -> None:
    """Test ScanBatcher with some packets coming out of order (only lagging case)
    by no more than a single id."""
    scans = take(3, client.Scans(lidar_stream_with_lagging_frame_ids))
    assert list(map(lambda s: s.frame_id, scans)) == [65535, 0, 1]


@pytest.mark.parametrize('file', ['OS-0-32-U1_v2.2.0_1024x10.pcap', 'windowed_frame1.pcap', 'windowed_frame2.pcap'])
def test_early_release(file: str, test_data_dir):
    """ Verify that scan batcher releases a complete scan without waiting for a packet
    from the next one"""
    path = test_data_dir / "pcaps" / file
    source = PcapMultiPacketReader(str(path))

    metadata = source.metadata[0]
    ls = LidarScan(metadata)
    b = ScanBatcher(metadata)
    pf = PacketFormat(metadata)
    completed = False
    for i, s in source:
        if isinstance(s, LidarPacket):
            frame_id = pf.frame_id(s.buf)
            if b(s, ls):
                completed = True
                assert ls.frame_id == frame_id
                assert ls.complete(metadata.format.column_window)
    assert completed


def test_batching_alerts():
    """It should include alert_flags from the packet in the LidarScan."""
    profile = client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 1024
    info.format.columns_per_packet = 16
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    writer = PacketWriter.from_profile(profile, info.format.pixels_per_column, info.format.columns_per_packet)
    batcher = ScanBatcher(info)
    scan = LidarScan(info)
    scan.frame_id = 0

    num_packets = info.format.columns_per_frame // info.format.columns_per_packet
    for packet_id in range(num_packets):

        # create a packet and set the measurement id of the first column, which
        # ScanBatcher uses to determine which packet in the scan it is
        packet = LidarPacket()
        writer.set_col_measurement_id(packet, 0, packet_id * info.format.columns_per_packet)

        # set the alert flag value to the packet id (just so we have something to test.)
        writer.set_alert_flags(packet, packet_id)
        assert writer.alert_flags(packet.buf) == packet_id
        batcher(packet, scan)

    # confirm that the alert flag values are increasing from 0..63 (the same as the packet ids)
    assert np.array_equal(scan.alert_flags, np.array(range(num_packets)))


def test_batching_countdowns():
    """It should include shutdown_countdown and shot_limiting_countdown in the LidarScan."""
    profile = client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL
    info = SensorInfo()
    info.format.columns_per_frame = 1024
    info.format.columns_per_packet = 16
    info.format.pixels_per_column = 128
    info.format.udp_profile_lidar = profile
    writer = PacketWriter.from_profile(profile, info.format.pixels_per_column, info.format.columns_per_packet)
    batcher = ScanBatcher(info)
    scan = LidarScan(info)
    scan.frame_id = -1

    num_packets = info.format.columns_per_frame // info.format.columns_per_packet
    for packet_id in range(num_packets):
        packet = LidarPacket()
        # According to FW, the values from each packet will be the same within a given frame
        writer.set_shutdown_countdown(packet, 30)
        writer.set_shot_limiting_countdown(packet, 29)
        batcher(packet, scan)

    assert scan.shutdown_countdown == writer.countdown_thermal_shutdown(packet.buf)
    assert scan.shot_limiting_countdown == writer.countdown_shot_limiting(packet.buf)

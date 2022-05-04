"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from copy import deepcopy
from typing import Dict, Iterator

from more_itertools import take
import numpy as np
import pytest

from ouster import client
from ouster.client._client import ScanBatcher


def _patch_frame_id(packet: client.LidarPacket, fid: int) -> None:
    """Rewrite the frame id of a non-legacy format lidar packet."""
    packet._data[2:4] = memoryview(fid.to_bytes(2, byteorder='little'))


def _simple_scans(source: client.PacketSource) -> Iterator[client.LidarScan]:
    """Batch packets to scans without zeroing between frames."""
    batch = ScanBatcher(source.metadata)
    info = source.metadata

    ls = client.LidarScan(info.format.pixels_per_column,
                          info.format.columns_per_frame,
                          info.format.udp_profile_lidar)

    for p in source:
        if batch(p._data, ls):
            yield deepcopy(ls)


@pytest.fixture
def lidar_stream(packets: client.PacketSource) -> client.PacketSource:
    """Infinite stream of lidar packets with spoofed frame ids."""
    def gen_packets():
        frame_id = 0
        while True:
            plist = deepcopy(list(packets))
            for p in plist:
                if isinstance(p, client.LidarPacket):
                    _patch_frame_id(p, frame_id)
                    yield p
            frame_id += 1

    return client.Packets(gen_packets(), packets.metadata)


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_scans_multi(lidar_stream: client.PacketSource) -> None:
    """Test batching multiple scans."""
    scans = take(10, client.Scans(lidar_stream))

    assert list(map(lambda s: s.frame_id, scans)) == list(range(10))


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

    # initialize fields and headers with nonzero values
    ls.timestamp[:] = 1
    ls.measurement_id[:] = 1
    ls.status[:] = 1
    for f in ls.fields:
        ls.field(f)[:] = 1

    # packet indices to drop
    drop_inds = [10, 20]

    # drop some packets
    packets = list(take(packets_per_frame, lidar_stream))
    for i in drop_inds:
        del packets[i]

    # parse the packets into a scan
    for p in packets:
        batch(p._data, ls)

    # check that data associated with the dropped packets is zeroed
    n = info.format.columns_per_packet

    assert np.count_nonzero(
        ls.status) == info.format.columns_per_frame - n * len(drop_inds)

    for i in drop_inds:
        assert (ls.timestamp[i * n:n] == 0).all()
        assert (ls.measurement_id[i * n:n] == 0).all()
        assert (ls.status[i * n:n] == 0).all()

        for f in ls.fields:
            assert (ls.field(f)[:, i * n:n] == 0).all()


@pytest.mark.parametrize('test_key', ['dual-2.2'])
def test_batch_custom_fields(lidar_stream: client.PacketSource) -> None:
    """Test batching of a LidarScan with custom fields set."""

    info = lidar_stream.metadata

    packets_per_frame = (info.format.columns_per_frame //
                         info.format.columns_per_packet)

    batch = ScanBatcher(lidar_stream.metadata)

    # create LidarScan with only 2 fields
    fields: Dict[client.ChanField, client.FieldDType] = {
        client.ChanField.RANGE: np.uint32,
        client.ChanField.SIGNAL: np.uint16
    }

    ls = client.LidarScan(info.format.pixels_per_column,
                          info.format.columns_per_frame, fields)

    # we expect zero initialized fields
    for f in ls.fields:
        assert np.count_nonzero(ls.field(f)) == 0

    # do batching into ls with a fields subset
    for p in take(packets_per_frame, lidar_stream):
        batch(p._data, ls)

    # it should contain the same num fields as we've added
    assert len(list(ls.fields)) == len(fields)

    # and the content shouldn't be zero after batching
    for f in ls.fields:
        assert np.count_nonzero(ls.field(f)) > 0


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_incompatible_profile(lidar_stream: client.PacketSource) -> None:
    """Test batching of a LidarScan with custom fields set."""

    info = lidar_stream.metadata
    assert info.format.udp_profile_lidar == client.UDPProfileLidar.PROFILE_LIDAR_LEGACY

    packets_per_frame = (info.format.columns_per_frame //
                         info.format.columns_per_packet)

    batch = ScanBatcher(lidar_stream.metadata)

    ls = client.LidarScan(info.format.pixels_per_column,
                          info.format.columns_per_frame,
                          client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL)

    # Try to decode a legacy packet into a dual returns scan
    # TODO change exception thrown on the cpp side
    with pytest.raises(IndexError):
        for p in take(packets_per_frame, lidar_stream):
            batch(p._data, ls)

    batch = ScanBatcher(lidar_stream.metadata)

    fields = {
        client.ChanField.RANGE: np.uint8,
    }
    ls = client.LidarScan(info.format.pixels_per_column,
                          info.format.columns_per_frame,
                          fields)

    # Test for decoding scans to a bad dest buffer type
    with pytest.raises(ValueError):
        for p in take(packets_per_frame, lidar_stream):
            batch(p._data, ls)

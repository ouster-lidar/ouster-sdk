"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from collections import defaultdict
from copy import copy
from os import path
from random import getrandbits, shuffle, random
from typing import (Dict, Iterable, Iterator, List)
from itertools import chain

import pytest
import time

from ouster import pcap
from ouster.pcap import _pcap
from ouster import client
from ouster.client import _client

SLL_PROTO = 42
ETH_PROTO = 1
UDP_PROTO = 17


def fake_packets(metadata: client.SensorInfo,
                 n_lidar: int = 0,
                 n_imu: int = 0,
                 timestamped: bool = False) -> Iterator[client.Packet]:

    pf = _client.PacketFormat.from_info(metadata)
    current_ts = time.time()

    choices = [True] * n_lidar + [False] * n_imu
    shuffle(choices)

    for is_lidar in choices:
        current_ts += random()
        packet_ts = current_ts if timestamped else None

        if is_lidar:
            buf = bytearray(
                getrandbits(8) for _ in range(pf.lidar_packet_size))
            yield client.LidarPacket(buf, metadata, packet_ts)
        else:
            buf = bytearray(getrandbits(8) for _ in range(pf.imu_packet_size))
            yield client.ImuPacket(buf, metadata, packet_ts)


@pytest.fixture
def n_packets() -> int:
    return 10


@pytest.fixture
def lidar_imu_frac() -> float:
    return 0.5


@pytest.fixture
def use_sll() -> int:
    return False


@pytest.fixture
def fake_meta(meta_2_0: client.SensorInfo) -> client.SensorInfo:
    """Use fw 2.0 metadata for randomly generated packets."""
    return meta_2_0


@pytest.fixture
def fake_pcap_path(lidar_imu_frac: float, fake_meta: client.SensorInfo,
                   n_packets: int, use_sll: bool, tmpdir: str) -> str:
    file_path = path.join(tmpdir, "pcap_test.pcap")
    n_lidar = int(lidar_imu_frac * n_packets)
    n_imu = n_packets - n_lidar
    packets = fake_packets(fake_meta, n_lidar, n_imu)
    pcap.record(packets, file_path, use_sll_encapsulation=use_sll)

    return file_path


@pytest.fixture
def fake_pcap(fake_meta, fake_pcap_path) -> Iterable[pcap.Pcap]:
    pc = pcap.Pcap(fake_pcap_path, fake_meta)
    yield pc
    pc.close()


@pytest.mark.parametrize('n_packets', [0])
def test_pcap_read_empty(fake_meta, fake_pcap_path) -> None:
    """Check reading an empty pcap doesn't fail."""
    fake_pcap = pcap.Pcap(fake_pcap_path, fake_meta)
    assert fake_pcap.ports == (0, 0)
    assert list(fake_pcap) == []


@pytest.mark.parametrize('n_packets', [10])
def test_pcap_read_wrong_ports(fake_meta, fake_pcap_path) -> None:
    """Check specifying wrong ports."""
    fake_pcap = pcap.Pcap(fake_pcap_path,
                          fake_meta,
                          lidar_port=7505,
                          imu_port=7506)
    assert fake_pcap.ports == (7505, 7506)
    assert fake_pcap._guesses == []
    assert list(fake_pcap) == []


@pytest.mark.parametrize('n_packets', [10])
def test_pcap_guess_one_port_invalid(fake_meta, fake_pcap_path) -> None:
    """Check that a port belonging to another pair isn't inferred.

    Capture contains data on (7502, 7503), but user expects lidar data on 7505.
    """
    fake_pcap = pcap.Pcap(fake_pcap_path,
                          fake_meta,
                          lidar_port=7505,
                          imu_port=0)
    assert fake_pcap._guesses == []
    assert fake_pcap.ports == (7505, 0)
    assert list(fake_pcap) == []


@pytest.mark.parametrize('n_packets', [10])
@pytest.mark.parametrize('lidar_imu_frac', [1.0])
def test_pcap_guess_one_port_valid(fake_meta, fake_pcap_path) -> None:
    """Check that an unpaired port can be infered.

    Capture contains just lidar data on 7502, and user expects imu on 7503.
    """
    fake_pcap = pcap.Pcap(fake_pcap_path,
                          fake_meta,
                          lidar_port=0,
                          imu_port=7503)
    assert fake_pcap._guesses == [(7502, 0)]  # no imu packets, no guess
    assert fake_pcap.ports == (7502, 7503)
    assert len(list(fake_pcap)) == 10


@pytest.mark.parametrize('n_packets', [10])
def test_pcap_read_10(fake_pcap) -> None:
    """Check that reading a test pcap produces the right number of packets."""
    assert fake_pcap.ports == (7502, 7503)
    assert len(list(fake_pcap)) == 10


@pytest.mark.parametrize('n_packets', [10])
def test_pcap_infer_one_port(fake_meta, fake_pcap_path) -> None:
    """Check that a matching port is inferred when one is specified. """
    fake_pcap = pcap.Pcap(fake_pcap_path,
                          fake_meta,
                          lidar_port=0,
                          imu_port=7503)
    assert fake_pcap._guesses == [(7502, 7503)]
    assert fake_pcap.ports == (7502, 7503)
    assert len(list(fake_pcap)) == 10


@pytest.mark.parametrize('n_packets', [10])
def test_pcap_info_10(fake_meta, fake_pcap_path) -> None:
    """Test reading packet headers with private helper."""
    ports: Dict[int, int] = defaultdict(int)
    sizes: Dict[int, int] = defaultdict(int)
    encap: Dict[int, int] = defaultdict(int)
    net: Dict[int, int] = defaultdict(int)
    af: Dict[int, int] = defaultdict(int)

    for item in pcap._packet_info_stream(fake_pcap_path):
        ports[item.dst_port] += 1
        sizes[item.payload_size] += 1
        encap[item.encapsulation_protocol] += 1
        net[item.network_protocol] += 1
        af[item.ip_version] += 1

    assert ports[7502] + ports[7503] == 10
    assert sizes[6464] + sizes[48] == 10
    assert encap == {ETH_PROTO: 10}
    assert net == {UDP_PROTO: 10}
    assert af == {4: 10}


@pytest.mark.parametrize('n_packets', [10])
@pytest.mark.parametrize('use_sll', [True, False])
def test_pcap_info_encap_proto(fake_pcap_path, use_sll) -> None:
    """Test reading/writing pcaps with different encapsulation."""
    encap: Dict[int, int] = defaultdict(int)

    for item in pcap._packet_info_stream(fake_pcap_path):
        encap[item.encapsulation_protocol] += 1

    proto = SLL_PROTO if use_sll else ETH_PROTO
    assert encap == {proto: 10}


def test_pcap_reset(fake_pcap) -> None:
    """Test that resetting a pcap after reading works."""
    packets1 = list(fake_pcap)
    fake_pcap.reset()
    packets2 = list(fake_pcap)

    bufs1 = [bytes(p._data) for p in packets1]
    bufs2 = [bytes(p._data) for p in packets2]
    assert bufs1 == bufs2


def test_pcap_read_closed(fake_pcap: pcap.Pcap) -> None:
    """Check that reading from a closed pcap raises an error."""
    fake_pcap.close()
    with pytest.raises(ValueError):
        next(iter(fake_pcap))


@pytest.mark.parametrize("n_lidar, n_imu", [
    pytest.param(1, 0, id="one lidar ether"),
    pytest.param(20, 0, id="multi lidar ether"),
    pytest.param(0, 1, id="one imu ether"),
    pytest.param(0, 20, id="multi imu ether"),
    pytest.param(1, 1, id="one each ether"),
    pytest.param(20, 20, id="multi each ether"),
])
def test_read_write_lidar_imu(n_lidar, n_imu, fake_meta, tmpdir) -> None:
    """Test that random packets read back from pcap are identical."""
    in_packets = list(fake_packets(fake_meta, n_lidar, n_imu))
    file_path = path.join(tmpdir, "pcap_test.pcap")

    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, fake_meta))
    out_bufs = [bytes(p._data) for p in out_packets]
    in_bufs = [bytes(p._data) for p in in_packets]

    assert in_bufs == out_bufs


def test_timestamp_read_write(fake_meta, tmpdir) -> None:
    """Check that timestamps are preserved by a round trip."""
    in_packets = list(
        fake_packets(fake_meta, n_lidar=10, n_imu=10, timestamped=True))
    file_path = path.join(tmpdir, "pcap_test.pcap")

    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, fake_meta))
    out_timestamps = [p.capture_timestamp for p in out_packets]
    in_timestamps = [p.capture_timestamp for p in in_packets]

    assert len(in_timestamps) == len(out_timestamps)
    assert in_timestamps == pytest.approx(out_timestamps)


def test_no_timestamp_read_write(fake_meta, tmpdir) -> None:
    """Check that capture timestamps are set when recording."""
    in_packets = list(
        fake_packets(fake_meta, n_lidar=10, n_imu=10, timestamped=False))
    file_path = path.join(tmpdir, "pcap_test.pcap")

    current_timestamp = time.time()
    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, fake_meta))
    out_timestamps = [p.capture_timestamp for p in out_packets]
    in_timestamps = [p.capture_timestamp for p in in_packets]

    assert len(in_timestamps) == len(out_timestamps)
    assert all(ts is None for ts in in_timestamps)
    assert all(ts == pytest.approx(current_timestamp, abs=1.0)
               for ts in out_timestamps)


@pytest.mark.parametrize(
    "n_lidar_timestamp, n_lidar_no_timestamp, n_imu_timestamp, n_imu_no_timestamp",
    [
        pytest.param(10, 0, 10, 0, id="yes timestamps"),
        pytest.param(0, 10, 0, 10, id="no timestamps"),
        pytest.param(10, 10, 0, 0, id="mixed: lidar"),
        pytest.param(0, 0, 10, 10, id="mixed: imu"),
        pytest.param(10, 0, 0, 10, id="mixed: lidar ts, imu no ts"),
        pytest.param(0, 10, 10, 10, id="mixed: lidar no ts, imu ts"),
        pytest.param(10, 10, 10, 10, id="mixed: all"),
    ])
def test_mixed_timestamp_write(n_lidar_timestamp, n_lidar_no_timestamp,
                               n_imu_timestamp, n_imu_no_timestamp, fake_meta,
                               tmpdir) -> None:
    """Test recording mixed (un)timestamped packets fails."""

    lidar_ts_packets = fake_packets(fake_meta,
                                    n_lidar=n_lidar_timestamp,
                                    timestamped=True)
    lidar_no_ts_packets = fake_packets(fake_meta,
                                       n_lidar=n_lidar_no_timestamp,
                                       timestamped=False)
    imu_ts_packets = fake_packets(fake_meta,
                                  n_imu=n_imu_timestamp,
                                  timestamped=True)
    imu_no_ts_packets = fake_packets(fake_meta,
                                     n_imu=n_imu_no_timestamp,
                                     timestamped=False)
    in_packets: List[client.Packet] = list(
        chain(lidar_ts_packets, lidar_no_ts_packets, imu_ts_packets,
              imu_no_ts_packets))
    shuffle(in_packets)

    file_path = path.join(tmpdir, "pcap_test.pcap")

    yes_timestamps = n_lidar_timestamp + n_imu_timestamp
    no_timestamps = n_lidar_no_timestamp + n_imu_no_timestamp

    if yes_timestamps > 0 and no_timestamps > 0:
        with pytest.raises(ValueError):
            pcap.record(in_packets, file_path)
    else:
        pcap.record(in_packets, file_path)


def test_write_nonsensical_packet_type(tmpdir) -> None:
    """Check that writing nonsense raises an error and cleans up."""
    file_path = path.join(tmpdir, "pcap_test.pcap")

    in_packets = [42]
    with pytest.raises(ValueError):
        pcap.record(in_packets, file_path)  # type: ignore

    assert not path.exists(file_path), "Didn't clean up empty file"


def test_lidar_guess_ambiguous(fake_meta, tmpdir) -> None:
    """Test reading when there's more than one possible lidar port."""
    packets = fake_packets(fake_meta, n_lidar=2)
    file_path = path.join(tmpdir, "pcap_test.pcap")

    buf_size = 2**16
    handle = _pcap.record_initialize(file_path, "127.0.0.1", "127.0.0.1",
                                     buf_size)
    try:
        _pcap.record_packet(handle, 7502, 7502, (next(packets))._data, 1)
        _pcap.record_packet(handle, 7503, 7503, (next(packets))._data, 2)
    finally:
        _pcap.record_uninitialize(handle)

    source = pcap.Pcap(file_path, fake_meta)
    assert len(source._guesses) > 1
    assert source.ports == (7503, 0)  # arbitrary but deterministic
    assert len(list(source)) == 1


def test_imu_guess_ambiguous(fake_meta, tmpdir) -> None:
    """Test reading when there's more than one possible imu port."""
    packets = fake_packets(fake_meta, n_imu=2)
    file_path = path.join(tmpdir, "pcap_test.pcap")

    buf_size = 2**16
    handle = _pcap.record_initialize(file_path, "127.0.0.1", "127.0.0.1",
                                     buf_size)
    try:
        _pcap.record_packet(handle, 7502, 7502, (next(packets))._data, 1)
        _pcap.record_packet(handle, 7503, 7503, (next(packets))._data, 2)
    finally:
        _pcap.record_uninitialize(handle)

    source = pcap.Pcap(file_path, fake_meta)
    assert len(source._guesses) > 1
    assert source.ports == (0, 7503)  # arbitrary but deterministic
    assert len(list(source)) == 1


def test_pcap_read_real(real_pcap: pcap.Pcap) -> None:
    """Sanity check reading pcaps with real data."""

    assert len(real_pcap._guesses) == 1

    # lidar data should be on 7502
    assert real_pcap.ports[0] == 7502

    packets = list(real_pcap)
    lidar_packets = [p for p in packets if isinstance(p, client.LidarPacket)]

    # test data should contain one frame of 1024-mode data
    assert len(lidar_packets) == 64

    # and should have timestamps
    assert all(p.capture_timestamp is not None for p in packets)


def test_pcap_guess_real(meta: client.SensorInfo, real_pcap_path: str) -> None:
    """Check that lidar port for real data can inferred."""

    meta_no_ports = copy(meta)
    meta_no_ports.udp_port_lidar = 0
    meta_no_ports.udp_port_imu = 0

    real_pcap = pcap.Pcap(real_pcap_path, meta_no_ports)
    assert real_pcap.ports[0] == 7502

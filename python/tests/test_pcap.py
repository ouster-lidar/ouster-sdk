#  type: ignore
"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from collections import defaultdict
from copy import copy
from os import path
from random import getrandbits, shuffle, random
from typing import (Dict, Iterable, Iterator, List, Callable)
from itertools import chain
from more_itertools import consume

import pytest
import time

from ouster import pcap
from ouster.pcap import _pcap
from ouster import client
from ouster.client import _client
from tests.conftest import PCAPS_DATA_DIR, TESTS
from tests.test_batching import _patch_frame_id

SLL_PROTO = 113
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


def set_init_id(data: bytearray, init_id: int) -> None:
    """Rewrite the init id of a non-legacy format lidar packet."""
    data[4:7] = memoryview(init_id.to_bytes(3, byteorder='little'))


def set_prod_sn(data: bytearray, prod_sn: int) -> None:
    """Rewrite the sn of a non-legacy format lidar packet."""
    data[7:11] = memoryview(prod_sn.to_bytes(5, byteorder='little'))


def fake_packet_stream_with_frame_id(metadata: client.SensorInfo,
                n_frames: int,
                n_lidar_packets_per_frame: int,
                n_imu_packets_per_frame: int,
                frame_id_fn: Callable[[int], int]) -> Iterator[client.Packet]:
    """Generate non-legacy lidar packets with frame_id set.
    Include some IMU packets to make sure indices are
    computed correctly with IMU data present."""
    pf = _client.PacketFormat.from_info(metadata)

    for frame in range(n_frames):
        choices = [True] * n_lidar_packets_per_frame + [False] * n_imu_packets_per_frame
        shuffle(choices)
        for is_lidar in choices:
            packet_ts = None
            if is_lidar:
                buf = bytearray(
                    getrandbits(8) for _ in range(pf.lidar_packet_size))
                set_init_id(buf, metadata.init_id)
                set_prod_sn(buf, int(metadata.sn))
                packet = client.LidarPacket(buf, metadata, packet_ts)
                assert not packet.id_error
                _patch_frame_id(packet, frame_id_fn(frame))
                yield packet
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
    af: Dict[int, int] = defaultdict(int)
    info = pcap._packet_info_stream(fake_pcap_path, 0)

    for key in info.udp_streams:
        ports[key.dst_port] += 1
        for size in info.udp_streams[key].payload_size_counts:
            sizes[size] += info.udp_streams[key].payload_size_counts[size]
        for net_ver in info.udp_streams[key].ip_version_counts:
            af[net_ver] += info.udp_streams[key].ip_version_counts[net_ver]

    assert ports[7502] + ports[7503] == 2
    assert sizes[6464] + sizes[48] == 10
    assert info.encapsulation_protocol == ETH_PROTO
    assert af == {4: 10}


@pytest.mark.parametrize('n_packets', [10])
@pytest.mark.parametrize('use_sll', [True, False])
def test_pcap_info_encap_proto(fake_pcap_path, use_sll) -> None:
    """Test reading/writing pcaps with different encapsulation."""

    info = pcap._packet_info_stream(fake_pcap_path, 0)

    proto = SLL_PROTO if use_sll else ETH_PROTO
    assert info.encapsulation_protocol == proto


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
    handle = _pcap.record_initialize(file_path, buf_size)
    try:
        _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", 7502, 7502,
                            (next(packets))._data, 1)
        _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", 7503, 7503,
                            (next(packets))._data, 2)
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
    handle = _pcap.record_initialize(file_path, buf_size)
    try:
        _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", 7502, 7502,
                            (next(packets))._data, 1)
        _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", 7503, 7503,
                            (next(packets))._data, 2)
    finally:
        _pcap.record_uninitialize(handle)

    source = pcap.Pcap(file_path, fake_meta)
    assert len(source._guesses) > 1
    assert source.ports == (0, 7503)  # arbitrary but deterministic
    assert len(list(source)) == 1


def test_lidar_imu_guess_ambiguous(fake_meta, tmpdir) -> None:
    """Test reading when there's more than one possible lidar port."""
    lidar_packets = fake_packets(fake_meta, n_lidar=2)
    imu_packets = fake_packets(fake_meta, n_imu=2)
    file_path = path.join(tmpdir, "pcap_test.pcap")

    buf_size = 2**16
    handle = _pcap.record_initialize(file_path, buf_size)
    try:
        _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", 7502, 7502,
                            (next(lidar_packets))._data, 1)
        _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", 7503, 7503,
                            (next(lidar_packets))._data, 2)
        _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", 7504, 7504,
                            (next(imu_packets))._data, 3)
        _pcap.record_packet(handle, "127.0.0.1", "127.0.0.1", 7505, 7505,
                            (next(imu_packets))._data, 4)
    finally:
        _pcap.record_uninitialize(handle)

    source = pcap.Pcap(file_path, fake_meta)
    assert len(source._guesses) > 1
    assert source.ports == (7503, 7505)  # arbitrary but deterministic
    assert len(list(source)) == 2


def test_pcap_read_real(real_pcap: pcap.Pcap) -> None:
    """Sanity check reading pcaps with real data."""

    assert len(real_pcap._guesses) == 1

    # lidar data should be on 7502
    assert real_pcap.ports[0] == 7502

    packets = list(real_pcap)
    lidar_packets = [p for p in packets if isinstance(p, client.LidarPacket)]

    # TODO - update test to expect based on mode
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


def test_record_packet_info(fake_meta, tmpdir) -> None:
    """Test recording packets using the packet_info interface."""
    packets = fake_packets(fake_meta, 10, 10)
    file_path = path.join(tmpdir, "pcap_test.pcap")

    buf_size = 2**16
    record = _pcap.record_initialize(file_path, buf_size)
    i = 0
    for next_packet in packets:
        info = _pcap.packet_info()

        info.dst_ip = "127.0.0." + str(i)
        info.src_ip = "127.0.1." + str(i)

        info.dst_port = 10000 + i
        info.src_port = 20000 + i

        info.timestamp = i

        _pcap.record_packet(record, info, next_packet._data)

        i += 1

    _pcap.record_uninitialize(record)

    playback = _pcap.replay_initialize(file_path)
    info = _pcap.packet_info()
    i = 0
    while _pcap.next_packet_info(playback, info):
        assert (info.dst_ip == "127.0.0." + str(i))
        assert (info.src_ip == "127.0.1." + str(i))

        assert (info.dst_port == 10000 + i)
        assert (info.src_port == 20000 + i)

        assert (info.timestamp == i)

        i += 1

    _pcap.replay_uninitialize(playback)


def test_indexed_pcap_reader(tmpdir):
    """It should correctly locate the start of frames in a PCAP file"""
    meta_path = path.join(PCAPS_DATA_DIR, f"{TESTS['dual-2.2']}.json")

    sensor_info = client.SensorInfo(open(meta_path).read())
    num_frames = 10
    in_packets = list(fake_packet_stream_with_frame_id(sensor_info, num_frames, 3, 3, lambda frame_num: frame_num))
    assert len(in_packets) > 0
    file_path = path.join(tmpdir, "pcap_index_test.pcap")
    pcap.record(in_packets, file_path)
    reader = _pcap.IndexedPcapReader(file_path, [meta_path])
    while reader.next_packet():
        reader.update_index_for_current_packet()
    reader.reset()

    # the index should contain the number of frames from the file
    assert reader.get_index().frame_count(0) == num_frames
    assert len(reader.get_index().frame_indices[0]) == num_frames

    # make sure that the file offset for a frame in the index corresponds to the offset of the first packet of the frame
    frame_num = 0
    previous_frame_id = None
    while reader.next_packet():
        info = reader.current_info()
        if info.dst_port == sensor_info.udp_port_lidar:
            packet_frame_id = reader.current_frame_id()
            if previous_frame_id is None or packet_frame_id > previous_frame_id:
                assert reader.get_index().frame_indices[0][frame_num] == info.file_offset
                previous_frame_id = packet_frame_id
                frame_num += 1


def test_indexed_pcap_reader_seek(tmpdir):
    """After seeking to the start of a frame, next_packet should return the first packet of that frame"""
    meta_path = path.join(PCAPS_DATA_DIR, f"{TESTS['dual-2.2']}.json")

    sensor_info = client.SensorInfo(open(meta_path).read())
    packet_format = _client.PacketFormat.from_info(sensor_info)
    num_frames = 10
    packets_per_frame = 3
    in_packets = list(fake_packet_stream_with_frame_id(sensor_info, num_frames,
                      packets_per_frame, 3, lambda frame_num: frame_num))
    assert len(in_packets) > 0
    file_path = path.join(tmpdir, "pcap_index_test.pcap")
    pcap.record(in_packets, file_path)
    reader = _pcap.IndexedPcapReader(file_path, [meta_path])
    while reader.next_packet():
        reader.update_index_for_current_packet()
    reader.reset()

    for frame in range(num_frames):
        reader.get_index().seek_to_frame(reader, 0, frame)
        assert packet_format.lidar_packet_size == reader.next_packet()
        assert reader.current_info().file_offset == reader.get_index().frame_indices[0][frame]
        assert reader.current_frame_id() == frame
        assert reader.current_frame_id() == packet_format.frame_id(reader.current_data().tobytes())


def test_out_of_order_frames(tmpdir):
    """Frames that are out of order are skipped"""
    meta_path = path.join(PCAPS_DATA_DIR, f"{TESTS['dual-2.2']}.json")

    sensor_info = client.SensorInfo(open(meta_path).read())
    packet_format = _client.PacketFormat.from_info(sensor_info)
    num_frames = 10
    packets_per_frame = 3

    # odd number frames are out of order in this dataset
    # frame ids are 2, 1, 4, 3, 6, ...
    in_packets = list(fake_packet_stream_with_frame_id(sensor_info, num_frames,
                      packets_per_frame, 3, lambda f: f + 1 + (1 - f % 2) - f % 2))

    assert len(in_packets) > 0
    file_path = path.join(tmpdir, "pcap_index_test.pcap")
    pcap.record(in_packets, file_path)
    reader = _pcap.IndexedPcapReader(file_path, [meta_path])
    while reader.next_packet():
        reader.update_index_for_current_packet()
    reader.reset()

    # since odd number frames are out of order, there should be half as many in the index as in the input
    assert len(reader.get_index().frame_indices[0]) == num_frames // 2

    for frame in range(len(reader.get_index().frame_indices[0])):
        reader.get_index().seek_to_frame(reader, 0, frame)
        assert packet_format.lidar_packet_size == reader.next_packet()
        assert reader.current_info().file_offset == reader.get_index().frame_indices[0][frame]
        assert reader.current_frame_id() == (frame + 1) * 2
        assert reader.current_frame_id() == packet_format.frame_id(reader.current_data().tobytes())


def test_current_data(fake_meta, tmpdir):
    """It should provide access to current packet data as a memory view"""
    meta_path = path.join(PCAPS_DATA_DIR, f"{TESTS['dual-2.2']}.json")

    sensor_info = client.SensorInfo(open(meta_path).read())
    packet_format = _client.PacketFormat.from_info(sensor_info)
    num_frames = 10
    packets_per_frame = 1

    # odd number frames are out of order in this dataset
    # frame ids are 2, 1, 4, 3, 6, ...
    in_packets = list(fake_packet_stream_with_frame_id(sensor_info, num_frames,
                      packets_per_frame, 3, lambda f: f + 1 + (1 - f % 2) - f % 2))

    assert len(in_packets) > 0
    file_path = path.join(tmpdir, "pcap_index_test.pcap")
    pcap.record(in_packets, file_path)
    reader = _pcap.IndexedPcapReader(file_path, [meta_path])
    while reader.next_packet():
        reader.update_index_for_current_packet()
    reader.reset()

    frame_ids = []
    while reader.next_packet():
        info = reader.current_info()
        if info.dst_port == sensor_info.udp_port_lidar:
            packet_data = reader.current_data()
            frame_ids.append(packet_format.frame_id(packet_data.tobytes()))
    assert frame_ids == [2, 1, 4, 3, 6, 5, 8, 7, 10, 9]


def test_validation():
    """The Pcap class should accumlate errors detected by LidarPacketValidator"""
    meta_file_path = path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')
    pcap_file_path = path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    metadata = client.SensorInfo(open(meta_file_path).read())
    metadata.init_id = 123
    metadata.format.udp_profile_lidar = client.UDPProfileLidar.PROFILE_LIDAR_FIVE_WORD_PIXEL
    reader = pcap.Pcap(pcap_file_path, metadata)
    consume(reader)
    assert reader._errors == {
        client.PacketIdError('Metadata init_id/sn does not match: expected by metadata - \
123/122150000150, but got from packet buffer - 5431292/122150000150'): 64,
        client.PacketSizeError('Expected a packet of size 41216 but got a buffer of size 8448'): 64
    }


def test_legacy_reduced_json_data():
    """PCAP data with the legacy reduced metadata json should work."""
    meta_file_path = path.join(PCAPS_DATA_DIR, 'OS-1-64_sensor_config_reduced.json')
    pcap_file_path = path.join(PCAPS_DATA_DIR, 'OS-1-64_1024x10_fw20.pcap')
    metadata = client.SensorInfo(open(meta_file_path).read())
    packet_source = pcap.Pcap(pcap_file_path, metadata)
    scans = client.Scans(packet_source)
    assert 1 == sum(1 for _ in scans)

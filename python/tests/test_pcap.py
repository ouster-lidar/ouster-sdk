from collections import defaultdict
import os
from random import getrandbits, shuffle, random
from typing import Dict, Iterable, Iterator, List
from itertools import chain, islice

from more_itertools import roundrobin
import pytest
import time

from ouster import pcap
from ouster.pcap import _pcap
from ouster import client
from ouster.client import _client

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

NO_RANDOM_TIME = 0
RANDOM_FLOAT = 1
SLL_PROTO = 42
ETH_PROTO = 1
UDP_PROTO = 17
# Seed the start timestamp
current_timestamp = time.time()


def random_lidar_packets(metadata,
                         random_time=NO_RANDOM_TIME
                         ) -> Iterator[client.LidarPacket]:
    global current_timestamp

    pf = _client.PacketFormat.from_info(metadata)

    timestamp = None
    current_timestamp += random() * 4.0
    if random_time == RANDOM_FLOAT:
        timestamp = current_timestamp

    while True:
        buf = bytearray(getrandbits(8) for _ in range(pf.lidar_packet_size))
        yield client.LidarPacket(buf, metadata, timestamp)


def random_imu_packets(metadata,
                       random_time=NO_RANDOM_TIME
                       ) -> Iterator[client.ImuPacket]:
    global current_timestamp

    pf = _client.PacketFormat.from_info(metadata)

    timestamp = None
    current_timestamp += random() * 4.0
    if random_time == RANDOM_FLOAT:
        timestamp = current_timestamp

    while True:
        buf = bytearray(getrandbits(8) for _ in range(pf.imu_packet_size))
        yield client.ImuPacket(buf, metadata, timestamp)


@pytest.fixture
def n_packets() -> int:
    return 10


@pytest.fixture
def use_sll() -> int:
    return False


@pytest.fixture
def pcap_path(meta, n_packets, use_sll, tmpdir) -> Iterable[str]:
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    packets: Iterable[client.Packet] = islice(
        roundrobin(random_lidar_packets(meta), random_imu_packets(meta)),
        n_packets)

    pcap.record(packets, file_path, use_sll_encapsulation=use_sll)

    yield file_path


@pytest.fixture
def pcap_obj(meta, pcap_path) -> Iterable[pcap.Pcap]:
    pc = pcap.Pcap(pcap_path, meta)
    yield pc
    pc.close()


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.parametrize('n_packets', [0])
def test_pcap_read_empty(meta, pcap_path) -> None:
    """Check reading with fully specified ports doesn't fail."""
    pcap_obj = pcap.Pcap(pcap_path, meta)
    assert pcap_obj.ports == (0, 0)
    assert list(pcap_obj) == []


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.parametrize('n_packets', [10])
def test_pcap_read_10(pcap_obj) -> None:
    """Check that reading a test pcap produces the right number of packets."""
    assert pcap_obj.ports == (7502, 7503)
    assert len(list(pcap_obj)) == 10


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.parametrize('n_packets', [10])
def test_pcap_info_10(pcap_path, meta) -> None:
    """Test reading packet headers with private helper."""
    ports: Dict[int, int] = defaultdict(int)
    sizes: Dict[int, int] = defaultdict(int)
    encap: Dict[int, int] = defaultdict(int)
    net: Dict[int, int] = defaultdict(int)
    af: Dict[int, int] = defaultdict(int)

    for item in pcap._packet_info_stream(pcap_path):
        ports[item.dst_port] += 1
        sizes[item.payload_size] += 1
        encap[item.encapsulation_protocol] += 1
        net[item.network_protocol] += 1
        af[item.ip_version] += 1

    # roundrobin -> 5 of each
    assert ports == {7502: 5, 7503: 5}
    assert sizes == {6464: 5, 48: 5}
    assert encap == {ETH_PROTO: 10}
    assert net == {UDP_PROTO: 10}
    assert af == {4: 10}


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.parametrize('n_packets', [10])
@pytest.mark.parametrize('use_sll', [True, False])
def test_pcap_info_encap_proto(pcap_path, use_sll) -> None:
    """Test reading/writing pcaps with different encapsulation."""
    encap: Dict[int, int] = defaultdict(int)

    for item in pcap._packet_info_stream(pcap_path):
        encap[item.encapsulation_protocol] += 1

    proto = SLL_PROTO if use_sll else ETH_PROTO
    assert encap == {proto: 10}


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_pcap_reset(pcap_obj) -> None:
    """Test that resetting a pcap after reading works."""
    packets1 = list(pcap_obj)
    pcap_obj.reset()
    packets2 = list(pcap_obj)

    bufs1 = [bytes(p._data) for p in packets1]
    bufs2 = [bytes(p._data) for p in packets2]
    assert bufs1 == bufs2


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_pcap_read_closed(pcap_obj) -> None:
    """Check that reading from a closed pcap raises an error."""
    pcap_obj.close()
    with pytest.raises(ValueError):
        next(iter(pcap_obj))


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.parametrize("n_lidar, n_imu", [
    pytest.param(1, 0, id="one lidar ether"),
    pytest.param(20, 0, id="multi lidar ether"),
    pytest.param(0, 1, id="one imu ether"),
    pytest.param(0, 20, id="multi imu ether"),
    pytest.param(1, 1, id="one each ether"),
    pytest.param(20, 20, id="multi each ether"),
])
def test_read_write_lidar_imu(n_lidar, n_imu, meta, tmpdir) -> None:
    """Test that random packets read back from pcap are identical."""
    lidar_packets = islice(random_lidar_packets(meta), n_lidar)
    imu_packets = islice(random_imu_packets(meta), n_imu)
    in_packets: List[client.Packet] = list(chain(lidar_packets, imu_packets))

    shuffle(in_packets)

    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, meta))
    out_bufs = [bytes(p._data) for p in out_packets]
    in_bufs = [bytes(p._data) for p in in_packets]

    assert in_bufs == out_bufs


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
@pytest.mark.parametrize("mode", [
    pytest.param(RANDOM_FLOAT, id="random float timestamp"),
])
def test_timestamp_float_read_write(mode, meta, tmpdir):
    lidar_packets = islice(random_lidar_packets(meta, random_time=mode), 10)
    imu_packets = islice(random_imu_packets(meta, random_time=mode), 10)
    in_packets = list(chain(lidar_packets, imu_packets))

    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, meta))
    out_timestamps = []
    in_timestamps = []

    out_timestamps = [p.capture_timestamp for p in out_packets]
    in_timestamps = [p.capture_timestamp for p in in_packets]

    assert len(in_timestamps) == len(out_timestamps)
    for i, o in zip(in_timestamps, out_timestamps):
        # Make sure to deal with float rounding issues in the compare
        assert i == pytest.approx(o, abs=1e-6)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_no_timestamp_read_write(meta, tmpdir):
    mode = NO_RANDOM_TIME
    current_timestamp = time.time()
    lidar_packets = islice(random_lidar_packets(meta, random_time=mode), 10)
    imu_packets = islice(random_imu_packets(meta, random_time=mode), 10)
    in_packets = list(chain(lidar_packets, imu_packets))

    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, meta))
    out_timestamps = []
    in_timestamps = []

    out_timestamps = [p.capture_timestamp for p in out_packets]
    in_timestamps = [p.capture_timestamp for p in in_packets]

    assert len(in_timestamps) == len(out_timestamps)
    for i, o in zip(in_timestamps, out_timestamps):
        assert i is None
        assert current_timestamp == pytest.approx(o, abs=5e-1)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
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
                               n_imu_timestamp, n_imu_no_timestamp, meta,
                               tmpdir) -> None:

    lidar_timestamp_packets = islice(
        random_lidar_packets(meta, random_time=RANDOM_FLOAT),
        n_lidar_timestamp)
    lidar_no_timestamp_packets = islice(
        random_lidar_packets(meta, random_time=NO_RANDOM_TIME),
        n_lidar_no_timestamp)
    imu_timestamp_packets = islice(
        random_imu_packets(meta, random_time=RANDOM_FLOAT), n_imu_timestamp)
    imu_no_timestamp_packets = islice(
        random_imu_packets(meta, random_time=NO_RANDOM_TIME),
        n_imu_no_timestamp)
    in_packets: List[client.Packet] = list(
        chain(lidar_timestamp_packets, lidar_no_timestamp_packets,
              imu_timestamp_packets, imu_no_timestamp_packets))

    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    yes_timestamps = n_lidar_timestamp + n_imu_timestamp
    no_timestamps = n_lidar_no_timestamp + n_imu_no_timestamp

    if yes_timestamps > 0 and no_timestamps > 0:
        with pytest.raises(ValueError):
            pcap.record(in_packets, file_path)
    else:
        pcap.record(in_packets, file_path)


def test_write_nonsensical_packet_type(tmpdir) -> None:
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    in_packets = [42]
    with pytest.raises(ValueError):
        pcap.record(in_packets, file_path)  # type: ignore

    assert not os.path.exists(file_path), "Didn't clean up empty file"


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_lidar_guess_error(meta, tmpdir) -> None:
    packets = islice(random_lidar_packets(meta), 2)
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    buf_size = 2**16
    handle = _pcap.record_initialize(file_path, "127.0.0.1", "127.0.0.1",
                                     buf_size)
    try:
        _pcap.record_packet(handle, 7502, 7502, (next(packets))._data, 1)
        _pcap.record_packet(handle, 7503, 7503, (next(packets))._data, 2)
    finally:
        _pcap.record_uninitialize(handle)

    source = pcap.Pcap(file_path, meta)
    assert len(source._guesses) > 1
    assert len(list(source)) == 1


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_imu_guess_error(meta, tmpdir) -> None:
    packets = islice(random_imu_packets(meta), 2)
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    buf_size = 2**16
    handle = _pcap.record_initialize(file_path, "127.0.0.1", "127.0.0.1",
                                     buf_size)
    try:
        _pcap.record_packet(handle, 7502, 7502, (next(packets))._data, 1)
        _pcap.record_packet(handle, 7503, 7503, (next(packets))._data, 2)
    finally:
        _pcap.record_uninitialize(handle)

    source = pcap.Pcap(file_path, meta)
    assert len(source._guesses) > 1
    assert len(list(source)) == 1

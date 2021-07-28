import os
from random import getrandbits, shuffle, random
from typing import Iterator
from itertools import chain, islice

from more_itertools import roundrobin
import pytest
import time

from ouster import pcap
from ouster.pcap import _pcap
from ouster import client
from ouster.client import _client, _digest

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

NO_RANDOM_TIME = 0
RANDOM_FLOAT = 1

# Seed the start timestamp
current_timestamp = time.time()


def random_lidar_packets(metadata, random_time=NO_RANDOM_TIME) -> Iterator[client.LidarPacket]:
    global current_timestamp

    pf = _client.PacketFormat.from_info(metadata)

    timestamp = None
    current_timestamp += random() * 4.0
    if random_time == RANDOM_FLOAT:
        timestamp = current_timestamp

    while True:
        buf = bytearray(getrandbits(8) for _ in range(pf.lidar_packet_size))
        yield client.LidarPacket(buf, metadata, timestamp)


def random_imu_packets(metadata, random_time=NO_RANDOM_TIME) -> Iterator[client.ImuPacket]:
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
def metadata() -> client.SensorInfo:
    digest_path = os.path.join(DATA_DIR, "os-992011000121_digest.json")
    with open(digest_path, 'r') as f:
        return _digest.StreamDigest.from_json(f.read()).meta


@pytest.fixture
def pcap_path(metadata, n_packets, tmpdir):
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    packets = islice(
        roundrobin(random_lidar_packets(metadata),
                   random_imu_packets(metadata)), n_packets)

    pcap.record(packets, file_path)

    yield file_path


@pytest.fixture
def pcap_obj(metadata, pcap_path):
    pc = pcap.Pcap(pcap_path, metadata)
    yield pc
    pc.close()


@pytest.mark.parametrize('n_packets', [0])
def test_pcap_info_empty(pcap_path) -> None:
    """Test that querying an empty pcap returns an empty PcapInfo."""
    res = pcap.info(pcap_path)
    assert res == pcap.PcapInfo()


@pytest.mark.parametrize('n_packets', [0])
def test_pcap_read_empty(pcap_obj) -> None:
    """Check that reading an empty pcap yields an empty list."""
    assert list(pcap_obj) == []


@pytest.mark.parametrize('n_packets', [10])
def test_pcap_read_10(pcap_obj) -> None:
    """Check that reading a test pcap produces the right number of packets."""
    assert len(list(pcap_obj)) == 10


@pytest.mark.parametrize('n_packets', [10])
def test_pcap_info_10(pcap_path) -> None:
    """Check that reading a test pcap produces the right number of packets."""
    res = pcap.info(pcap_path)
    assert res.packets_processed == 10
    assert res.packets_reassembled == 10
    assert res.non_udp_packets == 0

    # default ports
    assert res.guessed_lidar_port == 7502
    assert res.guessed_imu_port == 7503

    # roundrobin -> 5 of each
    assert res.ports == {7502: (6464, 5), 7503: (48, 5)}


def test_pcap_reset(pcap_obj) -> None:
    """Test that resetting a pcap after reading works."""
    packets1 = list(pcap_obj)
    pcap_obj.reset()
    packets2 = list(pcap_obj)

    bufs1 = [bytes(p._data) for p in packets1]
    bufs2 = [bytes(p._data) for p in packets2]
    assert bufs1 == bufs2


def test_pcap_read_closed(pcap_obj) -> None:
    """Check that reading from a closed pcap raises an error."""
    pcap_obj.close()
    with pytest.raises(ValueError):
        next(iter(pcap_obj))


@pytest.mark.parametrize("n_lidar, n_imu", [
    pytest.param(1, 0, id="one lidar ether"),
    pytest.param(20, 0, id="multi lidar ether"),
    pytest.param(0, 1, id="one imu ether"),
    pytest.param(0, 20, id="multi imu ether"),
    pytest.param(1, 1, id="one each ether"),
    pytest.param(20, 20, id="multi each ether"),
])
def test_read_write_lidar_imu(n_lidar, n_imu, metadata, tmpdir):
    """Test that random packets read back from pcap are identical."""
    lidar_packets = islice(random_lidar_packets(metadata), n_lidar)
    imu_packets = islice(random_imu_packets(metadata), n_imu)
    in_packets = list(chain(lidar_packets, imu_packets))

    shuffle(in_packets)

    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, metadata))
    out_bufs = [bytes(p._data) for p in out_packets]
    in_bufs = [bytes(p._data) for p in in_packets]

    assert in_bufs == out_bufs


@pytest.mark.parametrize("mode", [
    pytest.param(RANDOM_FLOAT, id="random float timestamp"),
])
def test_timestamp_float_read_write(mode, metadata, tmpdir):
    lidar_packets = islice(random_lidar_packets(metadata, random_time=mode), 10)
    imu_packets = islice(random_imu_packets(metadata, random_time=mode), 10)
    in_packets = list(chain(lidar_packets, imu_packets))

    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, metadata))
    out_timestamps = []
    in_timestamps = []

    out_timestamps = [p.capture_timestamp for p in out_packets]
    in_timestamps = [p.capture_timestamp for p in in_packets]

    assert len(in_timestamps) == len(out_timestamps)
    for i, o in zip(in_timestamps, out_timestamps):
        # Make sure to deal with float rounding issues in the compare
        assert i == pytest.approx(o, abs=1e-6)


def test_no_timestamp_read_write(metadata, tmpdir):
    mode = NO_RANDOM_TIME
    current_timestamp = time.time()
    lidar_packets = islice(random_lidar_packets(metadata, random_time=mode), 10)
    imu_packets = islice(random_imu_packets(metadata, random_time=mode), 10)
    in_packets = list(chain(lidar_packets, imu_packets))

    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    pcap.record(in_packets, file_path)
    out_packets = list(pcap.Pcap(file_path, metadata))
    out_timestamps = []
    in_timestamps = []

    out_timestamps = [p.capture_timestamp for p in out_packets]
    in_timestamps = [p.capture_timestamp for p in in_packets]

    assert len(in_timestamps) == len(out_timestamps)
    for i, o in zip(in_timestamps, out_timestamps):
        assert i is None
        assert current_timestamp == pytest.approx(o, abs=5e-1)


@pytest.mark.parametrize("n_lidar_timestamp, n_lidar_no_timestamp, n_imu_timestamp, n_imu_no_timestamp", [
    pytest.param(10, 0, 10, 0, id="yes timestamps"),
    pytest.param(0, 10, 0, 10, id="no timestamps"),
    pytest.param(10, 10, 0, 0, id="mixed: lidar"),
    pytest.param(0, 0, 10, 10, id="mixed: imu"),
    pytest.param(10, 0, 0, 10, id="mixed: lidar ts, imu no ts"),
    pytest.param(0, 10, 10, 10, id="mixed: lidar no ts, imu ts"),
    pytest.param(10, 10, 10, 10, id="mixed: lidar ts, lidar no ts, imu ts, imu no ts"),
])
def test_mixed_timestamp_write(n_lidar_timestamp, n_lidar_no_timestamp, n_imu_timestamp,
                               n_imu_no_timestamp, metadata, tmpdir):

    lidar_timestamp_packets = islice(random_lidar_packets(metadata, random_time=RANDOM_FLOAT),
                                     n_lidar_timestamp)
    lidar_no_timestamp_packets = islice(random_lidar_packets(metadata, random_time=NO_RANDOM_TIME),
                                        n_lidar_no_timestamp)
    imu_timestamp_packets = islice(random_imu_packets(metadata, random_time=RANDOM_FLOAT),
                                   n_imu_timestamp)
    imu_no_timestamp_packets = islice(random_imu_packets(metadata, random_time=NO_RANDOM_TIME),
                                      n_imu_no_timestamp)
    in_packets = list(chain(lidar_timestamp_packets, lidar_no_timestamp_packets, imu_timestamp_packets,
                            imu_no_timestamp_packets))

    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    yes_timestamps = n_lidar_timestamp + n_imu_timestamp
    no_timestamps = n_lidar_no_timestamp + n_imu_no_timestamp

    if yes_timestamps > 0 and no_timestamps > 0:
        with pytest.raises(ValueError):
            pcap.record(in_packets, file_path)
    else:
        pcap.record(in_packets, file_path)


def test_write_nonsensical_packet_type(metadata, tmpdir):
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    in_packets = [42]
    with pytest.raises(ValueError):
        pcap.record(in_packets, file_path)

    assert not os.path.exists(file_path), "Didn't clean up empty file"


def test_lidar_guess_error(metadata, tmpdir):
    packets = islice(random_lidar_packets(metadata), 2)
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    buf_size = 2**16
    handle = _pcap.record_initialize(file_path, "127.0.0.1", "127.0.0.1", buf_size)
    try:
        _pcap.record_packet(handle, 7502, 7502, (next(packets))._data, 1)
        _pcap.record_packet(handle, 7503, 7503, (next(packets))._data, 2)
    finally:
        _pcap.record_uninitialize(handle)

    with pytest.raises(ValueError):
        pcap.Pcap(file_path, metadata)


def test_imu_guess_error(metadata, tmpdir):
    packets = islice(random_imu_packets(metadata), 2)
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    buf_size = 2**16
    handle = _pcap.record_initialize(file_path, "127.0.0.1", "127.0.0.1", buf_size)
    try:
        _pcap.record_packet(handle, 7502, 7502, (next(packets))._data, 1)
        _pcap.record_packet(handle, 7503, 7503, (next(packets))._data, 2)
    finally:
        _pcap.record_uninitialize(handle)

    with pytest.raises(ValueError):
        pcap.Pcap(file_path, metadata)

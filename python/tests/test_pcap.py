import os
from random import getrandbits, shuffle
from typing import Iterator
from itertools import chain, islice

from more_itertools import roundrobin
import pytest

from ouster import pcap
from ouster import client
from ouster.client import _client, _digest

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def random_lidar_packets(metadata) -> Iterator[client.LidarPacket]:
    pf = _client.PacketFormat.from_info(metadata)

    while True:
        buf = bytearray(getrandbits(8) for _ in range(pf.lidar_packet_size))
        yield client.LidarPacket(buf, metadata)


def random_imu_packets(metadata) -> Iterator[client.ImuPacket]:
    pf = _client.PacketFormat.from_info(metadata)

    while True:
        buf = bytearray(getrandbits(8) for _ in range(pf.imu_packet_size))
        yield client.ImuPacket(buf, metadata)


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
    assert res.ipv6_packets == 0
    assert res.ipv4_packets == 10
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
    pytest.param(1, 0, id="one lidar"),
    pytest.param(20, 0, id="multi lidar"),
    pytest.param(0, 1, id="one imu"),
    pytest.param(0, 20, id="multi imu"),
    pytest.param(1, 1, id="one each"),
    pytest.param(20, 20, id="multi each"),
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

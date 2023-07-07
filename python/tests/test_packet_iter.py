import os
from itertools import count
from typing import Iterator, List
import weakref

import pytest

from ouster.client import SensorInfo
from ouster.pcap import Pcap
from ouster.sdkx import packet_iter
from tests.conftest import PCAPS_DATA_DIR


@pytest.fixture
def alist() -> List[int]:
    return list(range(10))


def test_ichunked_false(alist: List[int]) -> None:
    """Test outputting a single chunk."""
    it = packet_iter.ichunked_before(alist, lambda _: False)
    c0 = next(it)
    assert list(c0) == alist
    assert list(it) == []


def test_ichunked_true(alist: List[int]) -> None:
    """Test outputting a chunk per item."""
    it = packet_iter.ichunked_before(alist, lambda _: True)
    assert [list(c) for c in it] == [[i] for i in alist]


def test_ichunked_two() -> None:
    """Test splitting at a particular item."""
    s = "abcdef"
    it = packet_iter.ichunked_before(s, lambda c: c == "d")
    assert [list(c) for c in it] == [["a", "b", "c"], ["d", "e", "f"]]


def test_ichunked_first() -> None:
    """Check case where pred evaluates to true for first item."""
    l = [0, 1, 2, 3]
    it = packet_iter.ichunked_before(l, lambda c: c % 2 == 0)
    assert [list(c) for c in it] == [[0, 1], [2, 3]]


def test_ichunked_lazy() -> None:
    """Check that chunks are not evaluated eagerly."""

    i = -1

    def mycount() -> Iterator[int]:
        nonlocal i
        for i in count():
            yield i

    it = packet_iter.ichunked_before(mycount(), lambda i: i % 3 == 0)

    # doesn't read full chunk before yielding
    assert next(next(it)) == i == 0

    # reading next chunk advances to elt where pred is true
    next(it)
    assert i == 3


def test_ichunked_map() -> None:
    """Check that it's easy to read chunks into lists."""
    it = packet_iter.ichunked_before(count(), lambda i: i % 3 == 0)

    # pretty annoying: https://github.com/python/mypy/issues/9253
    chunks = map(list, it)  # type: ignore

    c0 = next(chunks)
    assert list(next(chunks)) == [3, 4, 5]
    assert list(c0) == [0, 1, 2]


def test_ichunked_noref() -> None:
    """Check that chunks are not retained in memory."""
    class Object(object):
        pass

    # iterator that doesn't keep references to yielded objects
    l = [Object() for _ in range(10)]
    consume = iter(l.pop, l[0])

    # split at third object
    o3 = l[-3]
    it = packet_iter.ichunked_before(consume, lambda o: o is o3)

    # to check if object from first chunk is in memory
    o2 = weakref.ref(l[-2])

    # read object from first chunk
    next(next(it))
    assert o2() is not None

    # go to next chunk, rest of first chunk should be deallocated
    next(it)
    assert o2() is None


def test_ichunked_exhaust_chunk(alist: List[int]) -> None:
    """Check that exhausting a chunk exhausts the chunk iterator."""
    it = packet_iter.ichunked_before(alist, lambda _: False)
    list(next(it))
    with pytest.raises(StopIteration):
        next(it)


def test_recording_packet_source(tmp_path) -> None:
    """It writes packets contained in the source to the output directory."""
    meta_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')
    pcap_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    sensor_info = SensorInfo(open(meta_file_path).read())
    source = Pcap(pcap_file_path, sensor_info)
    recording_iter = packet_iter.RecordingPacketSource(source, tmp_path, n_frames=1)
    emitted_packets = 0
    for packet in recording_iter:
        emitted_packets += 1
    assert emitted_packets == 74
    source = Pcap(pcap_file_path, sensor_info)
    assert len(os.listdir(tmp_path)) == 1
    recording_path = os.path.join(tmp_path, os.listdir(tmp_path)[0])
    recorded_pcap = Pcap(recording_path, sensor_info)
    recorded_packets = 0
    for packet in recorded_pcap:
        recorded_packets += 1
    assert recorded_packets == emitted_packets


def test_recording_packet_source_bad_packet_format(tmp_path) -> None:
    """It silently ignores packets that aren't lidar/imu because the Pcap PacketSource ignores these.
    (Note: RecordingPacketSource is written to raise ValueError
    if a packet in the source is neither a lidar packet nor imu packet.
    """
    meta_file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.json')
    pcap_file_path = os.path.join(PCAPS_DATA_DIR, 'VLI-16-one-packet.pcap')
    sensor_info = SensorInfo(open(meta_file_path).read())
    sensor_info.udp_port_lidar = 2368
    source = Pcap(pcap_file_path, sensor_info, lidar_port = 2368)
    recording_iter = packet_iter.RecordingPacketSource(source, tmp_path, n_frames=1)
    emitted_packets = 0
    for packet in recording_iter:
        emitted_packets += 1
    assert emitted_packets == 0

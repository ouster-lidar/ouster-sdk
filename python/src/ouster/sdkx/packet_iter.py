import os
import time
from datetime import datetime
from typing import Callable, Iterable, Iterator, TypeVar, Optional

from more_itertools import consume

from ouster.client import Packet, LidarPacket, ImuPacket, PacketSource, SensorInfo
from ouster.pcap.pcap import MTU_SIZE
import ouster.pcap._pcap as _pcap


T = TypeVar('T')


def ichunked_before(it: Iterable[T],
                    pred: Callable[[T], bool]) -> Iterator[Iterator[T]]:
    """Return the given stream chunked by the predicate.

    Each sub-iterator will be fully consumed when the next chunk is
    requested. No caching of unused items is performed, so client code should
    evaluate sub-iterators (e.g. into lists) to avoid dropping items.

    This should behave same as more_itertools.split_before, except that chunks
    aren't eagerly evaluated into lists. This makes it safe to use on streams
    where it's possible that ``pred`` never evaluates to true.
    """
    i = iter(it)

    # flag used by chunks to signal that the underlying iterator is exhausted
    done = False

    # first item of the next chunk. See: nonlocal below
    try:
        t = next(i)
    except StopIteration:
        return

    def chunk() -> Iterator[T]:
        nonlocal done, t

        yield t
        for t in i:
            if pred(t):
                break
            else:
                yield t
        # only if the iterator is exhausted
        else:
            done = True

    while not done:
        c = chunk()
        yield c
        consume(c)


def ichunked_framed(
    packets: Iterable[Packet],
    pred: Callable[[Packet],
                   bool] = lambda _: True) -> Iterator[Iterator[Packet]]:
    """Delimit a packets when the frame id changes and pred is true."""

    last_f_id = -1

    def frame_boundary(p: Packet) -> bool:
        nonlocal last_f_id
        if isinstance(p, LidarPacket):
            f_id = p.frame_id
            changed = last_f_id != -1 and f_id != last_f_id
            last_f_id = f_id
            return changed and pred(p)
        return False

    return ichunked_before(packets, frame_boundary)


def n_frames(packets: Iterable[Packet], n: int) -> Iterator[Packet]:
    for i, frame in enumerate(ichunked_framed(packets)):
        if i < n:
            yield from frame
        else:
            break


class RecordingPacketSource:
    # TODO: deduplicate this & pcap.record
    def __init__(self,
                 source: PacketSource,
                output_directory: str,
                *,
                prefix: str = "",
                n_seconds: float = 0.0,
                n_frames: Optional[int],
                chunk_size: int = 0,
                src_ip: str = "127.0.0.1",
                dst_ip: str = "127.0.0.1",
                lidar_port: int = 7502,
                imu_port: int = 7503,
                use_sll_encapsulation: bool = False):
        self.source = source
        self.output_directory = output_directory
        self.prefix = prefix
        self.n_seconds = n_seconds
        self.n_frames = n_frames
        self.chunk_size = chunk_size
        self.src_ip = src_ip
        self.dst_ip = dst_ip
        self.lidar_port = lidar_port
        self.imu_port = imu_port
        self.use_sll_encapsulation = use_sll_encapsulation

    @property
    def metadata(self) -> SensorInfo:
        """Return metadata from the underlying PacketSource."""
        return self.source.metadata

    def close(self) -> None:
        """Close the underlying PacketSource."""
        self.source.close()

    def __iter__(self) -> Iterator[Packet]:
        has_timestamp = None
        error = False
        n = 0
        file_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        metadata = self.source.metadata
        base_name = f"{self.prefix}{metadata.prod_line}_{metadata.fw_rev}_{metadata.mode}_{file_timestamp}"

        last_f_id = -1

        def frame_boundary(p: Packet) -> bool:
            nonlocal last_f_id
            if isinstance(p, LidarPacket):
                f_id = p.frame_id
                changed = last_f_id != -1 and f_id != last_f_id
                last_f_id = f_id
                return changed
            return False

        try:
            start_time = time.time()
            chunk = 0
            num_frames = 0
            pcap_path = os.path.join(self.output_directory, base_name) + f"-{chunk:03}.pcap"
            handle = _pcap.record_initialize(pcap_path, MTU_SIZE,
                                             self.use_sll_encapsulation)
            for packet in self.source:
                if isinstance(packet, LidarPacket):
                    src_port = self.lidar_port
                    dst_port = self.lidar_port
                elif isinstance(packet, ImuPacket):
                    src_port = self.imu_port
                    dst_port = self.imu_port
                else:
                    raise ValueError("Unexpected packet type")

                if has_timestamp is None:
                    has_timestamp = (packet.capture_timestamp is not None)
                elif has_timestamp != (packet.capture_timestamp is not None):
                    raise ValueError("Mixing timestamped/untimestamped packets")

                ts = packet.capture_timestamp or time.time()
                _pcap.record_packet(handle, self.src_ip, self.dst_ip, src_port, dst_port, packet._data, ts)
                if frame_boundary(packet):
                    num_frames += 1
                    if self.chunk_size and os.path.getsize(pcap_path) > self.chunk_size * 2**20:
                        # file size exceeds chunk size; create a new chunk
                        chunk += 1
                        pcap_path = os.path.join(self.output_directory, base_name) + f"-{chunk:03}.pcap"
                        _pcap.record_uninitialize(handle)
                        handle = _pcap.record_initialize(pcap_path, MTU_SIZE,
                                                         self.use_sll_encapsulation)
                    if (self.n_frames and num_frames > self.n_frames) or \
                        (self.n_seconds and time.time() - start_time > self.n_seconds):
                        break
                n += 1
                yield packet
        except Exception:
            error = True
            raise
        finally:
            _pcap.record_uninitialize(handle)
            if error and os.path.exists(pcap_path) and n == 0:
                os.remove(pcap_path)

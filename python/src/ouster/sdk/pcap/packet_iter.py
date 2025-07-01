import os
import time
from typing import (Callable, Iterable, Iterator, TypeVar,
                    Optional)

from more_itertools import consume

from ouster.sdk.core import (PacketSource, LidarPacket, ImuPacket, FrameBorder)
from ouster.sdk.pcap.pcap import MTU_SIZE
import ouster.sdk._bindings.pcap as _pcap


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


class RecordingPacketSource:
    # TODO: deduplicate this & pcap.record
    def __init__(self,
                source: PacketSource,
                prefix_path: str,
                *,
                sensor_idx: int = -1,
                n_seconds: float = 0.0,
                n_frames: Optional[int],
                chunk_size: int = 0,
                src_ip: str = "127.0.0.1",
                dst_ip: str = "127.0.0.1",
                lidar_port: int = -1,
                imu_port: int = -1,
                use_sll_encapsulation: bool = False,
                overwrite: bool = True):
        self.source = source
        self.sensor_idx = sensor_idx
        self.prefix_path = prefix_path
        self.n_seconds = n_seconds
        self.n_frames = n_frames
        self.chunk_size = chunk_size
        self.src_ip = src_ip
        self.dst_ip = dst_ip
        self.lidar_port = lidar_port
        self.imu_port = imu_port
        self.use_sll_encapsulation = use_sll_encapsulation
        self.overwrite = overwrite
        self._metadata = self.source.sensor_info

    @property  # type: ignore
    def __class__(self):
        # report the class of the wrapped packet source
        return self.source.__class__

    def __iter__(self):
        has_timestamp = None
        error = False
        n = 0

        frame_bound = []
        for m in self._metadata:
            frame_bound.append(FrameBorder(m))

        ports = []
        for m in self._metadata:
            lidar = m.config.udp_port_lidar if self.lidar_port < 0 else self.lidar_port
            imu = m.config.udp_port_imu if self.imu_port < 0 else self.imu_port
            ports.append((lidar, imu))

        chunk = 0
        pcap_path = self.prefix_path + f"-{chunk:03}.pcap"
        print(f"Saving PCAP file at {pcap_path}")
        if os.path.isfile(pcap_path) and not self.overwrite:
            raise FileExistsError(f"File '{pcap_path}' already exists")

        try:
            start_time = time.time()
            num_frames = 0
            handle = _pcap.record_initialize(pcap_path, MTU_SIZE,
                                             self.use_sll_encapsulation)
            for next_packet in self.source:
                idx, packet = next_packet if (type(next_packet) is tuple) else (0, next_packet)
                if (self.sensor_idx < 0) or (idx == self.sensor_idx):
                    if isinstance(packet, LidarPacket):
                        src_port = dst_port = ports[idx][0]
                    elif isinstance(packet, ImuPacket):
                        src_port = dst_port = ports[idx][1]
                    else:
                        raise ValueError("Unexpected packet type")

                    if has_timestamp is None:
                        has_timestamp = (packet.host_timestamp != 0)
                    elif has_timestamp != (packet.host_timestamp != 0):
                        raise ValueError("Mixing timestamped/untimestamped packets")

                    ts = (packet.host_timestamp / 1e9) or time.time()
                    _pcap.record_packet(handle, self.src_ip, self.dst_ip, src_port, dst_port, packet.buf, ts)

                    if frame_bound[idx](packet):
                        num_frames += 1
                        if self.chunk_size and os.path.getsize(pcap_path) > self.chunk_size * 2**20:
                            # file size exceeds chunk size; create a new chunk
                            chunk += 1
                            pcap_path = self.prefix_path + f"-{chunk:03}.pcap"
                            print(f"Saving PCAP file at {pcap_path}")
                            _pcap.record_uninitialize(handle)
                            if os.path.isfile(pcap_path) and not self.overwrite:
                                raise FileExistsError(f"File '{pcap_path}' already exists")
                            handle = _pcap.record_initialize(pcap_path, MTU_SIZE,
                                                            self.use_sll_encapsulation)
                        if (self.n_frames and num_frames > self.n_frames) or \
                            (self.n_seconds and time.time() - start_time > self.n_seconds):
                            break
                    n += 1
                yield next_packet
        except Exception:
            error = True
            raise
        finally:
            _pcap.record_uninitialize(handle)
            if error and os.path.exists(pcap_path) and n == 0:
                os.remove(pcap_path)

    def __getattr__(self, attr):
        # forward all other calls to self.source
        return self.source.__getattribute__(attr)

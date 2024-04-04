import os
import time
from typing import (Callable, Iterable, Iterator, TypeVar,
                    Optional, Any)

from more_itertools import consume

from ouster.sdk.client import (Packet, PacketMultiSource, LidarPacket, ImuPacket, FrameBorder)
from ouster.sdk.pcap.pcap import MTU_SIZE
import ouster.sdk.pcap._pcap as _pcap


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

    return ichunked_before(packets, FrameBorder(pred))


def n_frames(packets: Iterable[Packet], n: int) -> Iterator[Packet]:
    for i, frame in enumerate(ichunked_framed(packets)):
        if i < n:
            yield from frame
        else:
            break


# TODO: these currently account for SensorScanSource being based on Scans internally and will
#       require rework once that has a proper ScansMulti implementation -- Tim T.
class RecordingPacketSource:
    # TODO: deduplicate this & pcap.record
    def __init__(self,
                source: PacketMultiSource,
                prefix_path: str,
                *,
                sensor_idx: int = 0,
                n_seconds: float = 0.0,
                n_frames: Optional[int],
                chunk_size: int = 0,
                src_ip: str = "127.0.0.1",
                dst_ip: str = "127.0.0.1",
                lidar_port: int = 7502,
                imu_port: int = 7503,
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

    @property  # type: ignore
    def __class__(self):
        # report the class of the wrapped packet source
        return self.source.__class__

    def __iter__(self):
        has_timestamp = None
        error = False
        n = 0

        metadata = self.source.metadata
        if type(metadata) is list:
            metadata = metadata[self.sensor_idx]

        frame_bound = FrameBorder()

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
                idx, packet = next_packet if (type(next_packet) is tuple) else (None, next_packet)
                if (idx is None) or (idx == self.sensor_idx):
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

                    if frame_bound(packet):
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


# TODO: these currently account for SensorScanSource being based on Scans internally and will
#       require rework once that has a proper ScansMulti implementation -- Tim T.
class BagRecordingPacketSource:
    def __init__(self, packet_source: PacketMultiSource,
                 filename: str, sensor_idx: int = 0,
                 lidar_topic: str = "/os_node/lidar_packets",
                 imu_topic: str = "/os_node/imu_packets"):
        self.packet_source = packet_source
        self.filename = filename
        self.sensor_idx = sensor_idx
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic

    @property  # type: ignore
    def __class__(self):
        # report the class of the wrapped packet source
        return self.packet_source.__class__

    def __iter__(self):
        from ouster.cli.core.util import import_rosbag_modules
        import_rosbag_modules(raise_on_fail=True)

        from ouster.sdk.bag import PacketMsg  # type: ignore
        import rosbag  # type: ignore
        import rospy  # type: ignore
        try:
            with rosbag.Bag(self.filename, 'w') as outbag:
                for next_packet in self.packet_source:
                    idx, packet = next_packet if (type(next_packet) is tuple) else (None, next_packet)
                    if (idx is None) or (idx == self.sensor_idx):
                        ts = rospy.Time.from_sec(packet.capture_timestamp)
                        msg = PacketMsg(buf=packet._data.tobytes())
                        if isinstance(packet, LidarPacket):
                            outbag.write(self.lidar_topic, msg, ts)
                        elif isinstance(packet, ImuPacket):
                            outbag.write(self.imu_topic, msg, ts)
                    yield next_packet
        except (KeyboardInterrupt, StopIteration):
            pass

    def __getattr__(self, attr: str) -> Any:
        # forward all other calls to self.source
        return self.packet_source.__getattribute__(attr)

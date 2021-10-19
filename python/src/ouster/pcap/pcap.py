import os
import time
import socket
from threading import Lock
from typing import (Dict, Iterable, Iterator, Optional, Tuple)

from ouster.client import (LidarPacket, ImuPacket, Packet, PacketSource,
                           SensorInfo)
from ouster.pcap import _pcap


def _guess_imu_port(stream_data: Dict[int, Dict[int, int]]) -> Optional[int]:
    result = None
    imu_size = 48
    if (imu_size in stream_data):
        correct_packet_size = stream_data[imu_size]
        if (len(correct_packet_size) > 1):
            raise ValueError("Error: Multiple possible imu packets found")
        else:
            result = list(correct_packet_size)[0]

    return result


def _guess_lidar_port(stream_data: Dict[int, Dict[int, int]]) -> Optional[int]:
    result = None
    lidar_sizes = {3392, 6464, 12608, 24896}

    hit_count = 0
    multiple_error_msg = "Error: Multiple possible lidar packets found"
    for s in lidar_sizes:
        if s in stream_data:
            correct_packet_size = stream_data[s]
            hit_count += 1
            if (len(correct_packet_size) > 1):
                raise ValueError(multiple_error_msg)
            else:
                result = list(correct_packet_size)[0]

    if (hit_count > 1):
        raise ValueError(multiple_error_msg)

    return result


def _guess_ports(pcap_path: str) -> Tuple[Optional[int], Optional[int]]:
    p = _pcap.replay_initialize(pcap_path)
    packet_info = _pcap.packet_info()
    loop = True
    count = 10000
    sizes: Dict[int, Dict[int, int]] = {}

    while loop and count > 0:
        if not _pcap.next_packet_info(p, packet_info):
            loop = False
        else:
            if packet_info.payload_size not in sizes:
                sizes[packet_info.payload_size] = {}
            if packet_info.dst_port not in sizes[packet_info.payload_size]:
                sizes[packet_info.payload_size][packet_info.dst_port] = 0
            sizes[packet_info.payload_size][packet_info.dst_port] += 1
        count -= 1
    _pcap.replay_uninitialize(p)

    return _guess_lidar_port(sizes), _guess_imu_port(sizes)


def _pcap_info(path: str) -> Iterator[_pcap.packet_info]:
    handle = _pcap.replay_initialize(path)
    try:
        while True:
            if handle is None:
                raise ValueError("I/O operation on closed packet source")
            packet_info = _pcap.packet_info()
            if not _pcap.next_packet_info(handle, packet_info):
                break
            yield packet_info
    finally:
        if handle:
            _pcap.replay_uninitialize(handle)


class Pcap(PacketSource):
    """Read a sensor packet stream out of a pcap file as an iterator."""

    _lidar_port: Optional[int]
    _imu_port: Optional[int]
    _metadata: SensorInfo
    _rate: float
    _handle: Optional[_pcap.playback_handle]
    _lock: Lock

    def __init__(self,
                 pcap_path: str,
                 info: SensorInfo,
                 *,
                 rate: float = 0.0,
                 lidar_port: int = 0,
                 imu_port: int = 0) -> None:
        """Read a single-sensor data stream from a packet capture.

        This assumes a pcap contains the packet stream of a single sensor and
        attempts to guess which destination ports were associated with lidar vs.
        imu data based on packet sizes, unless explicitly specified.

        When a rate is specified, output packets in (a multiple of) real time using
        the pcap packet capture timestamps.

        Args:
            info: Sensor metadata
            pcap_path: File path of recorded pcap
            rate: Output packets in real time, if non-zero
            lidar_port: Specify the destination port of lidar packets
            imu_port: Specify the destination port of imu packets

        Returns:
            An iterator of lidar and IMU packets.
        """

        lidar_port_guess, imu_port_guess = _guess_ports(pcap_path)

        # use guessed values unless ports are specified (0 is falsey)
        self._lidar_port = lidar_port or info.udp_port_lidar or lidar_port_guess
        self._imu_port = imu_port or info.udp_port_imu or imu_port_guess

        self._metadata = info
        self._rate = rate
        self._handle = _pcap.replay_initialize(pcap_path)
        self._lock = Lock()

    def __iter__(self) -> Iterator[Packet]:
        with self._lock:
            if self._handle is None:
                raise ValueError("I/O operation on closed packet source")

        buf = bytearray(2**16)
        packet_info = _pcap.packet_info()

        real_start_ts = time.monotonic()
        pcap_start_ts = None
        metadata = self._metadata
        while True:
            with self._lock:
                if (self._handle is None or
                        not _pcap.next_packet_info(self._handle, packet_info)):
                    break
                timestamp = packet_info.timestamp

                n = _pcap.read_packet(self._handle, buf)

            if self._rate:
                if not pcap_start_ts:
                    pcap_start_ts = packet_info.timestamp
                real_delta = time.monotonic() - real_start_ts
                pcap_delta = (packet_info.timestamp -
                              pcap_start_ts) / self._rate
                delta = max(0, pcap_delta - real_delta)
                time.sleep(delta)

            if packet_info.dst_port == self._lidar_port and n != 0:
                yield LidarPacket(buf[0:n], metadata, timestamp)

            elif packet_info.dst_port == self._imu_port and n != 0:
                yield ImuPacket(buf[0:n], metadata, timestamp)

    @property
    def metadata(self) -> SensorInfo:
        return self._metadata

    def reset(self) -> None:
        """Restart playback from beginning. Thread-safe."""
        with self._lock:
            if self._handle is not None:
                _pcap.replay_reset(self._handle)

    @property
    def closed(self) -> bool:
        """Check if source is closed. Thread-safe."""
        with self._lock:
            return self._handle is None

    def close(self) -> None:
        """Release resources. Thread-safe."""
        with self._lock:
            if self._handle is not None:
                _pcap.replay_uninitialize(self._handle)
                self._handle = None


def _replay(pcap_path: str, info: SensorInfo, dst_ip: str, dst_lidar_port: int,
            dst_imu_port: int) -> Iterator[bool]:
    """Replay UDP packets out over the network.

    Todo:
        Not really sure about this interface

    Args:
        pcap_path: Path to the pcap file to replay
        info: The sensor metadata
        dst_ip: IP to send packets to
        dst_lidar_port: Destination port for lidar packets
        dst_imu_port: Destination port for imu packets

    Returns:
        An iterator that reports whether packets were sent successfully as it's
        consumed.
    """
    try:
        socket_out = socket.socket(family=socket.AF_INET,
                                   type=socket.SOCK_DGRAM)

        pcap_handle = Pcap(pcap_path, info)
        for item in pcap_handle:
            port = 0
            if isinstance(item, LidarPacket):
                port = dst_lidar_port
            if isinstance(item, ImuPacket):
                port = dst_imu_port

            socket_out.sendto(item._data.tobytes(), (dst_ip, port))
            yield True
        yield False
    finally:
        if pcap_handle is not None:
            pcap_handle.close()


def record(packets: Iterable[Packet],
           pcap_path: str,
           *,
           src_ip: str = "127.0.0.1",
           dst_ip: str = "127.0.0.1",
           lidar_port: int = 7502,
           imu_port: int = 7503,
           use_sll_encapsulation: bool = False) -> int:
    """Record a sequence of sensor packets to a pcap file.

    Args:
        packets: A (finite!) sequence of packets
        pcap_path: Path of the output pcap file
        src_ip: Source IP to use for all packets
        dst_ip: Destination IP to use for all packets
        lidar_port: Src/dst port to use for lidar packets
        imu_port: Src/dst port to use for imu packets
        use_sll_encapsulation: Use sll encapsulaiton for pcaps(ouster studio can not read)

    Returns:
        Number of packets captured
    """
    has_timestamp = None
    error = False
    buf_size = 2**16
    n = 0
    handle = _pcap.record_initialize(pcap_path, src_ip, dst_ip, buf_size,
                                     use_sll_encapsulation)
    try:
        for packet in packets:
            if isinstance(packet, LidarPacket):
                src_port = lidar_port
                dst_port = lidar_port
            elif isinstance(packet, ImuPacket):
                src_port = imu_port
                dst_port = imu_port
            else:
                raise ValueError("Unexpected packet type")

            if has_timestamp is None:
                has_timestamp = (packet.capture_timestamp is not None)
            elif has_timestamp != (packet.capture_timestamp is not None):
                raise ValueError("Mixing timestamped/untimestamped packets")

            ts = packet.capture_timestamp or time.time()
            _pcap.record_packet(handle, src_port, dst_port, packet._data, ts)
            n += 1
    except Exception:
        error = True
        raise
    finally:
        _pcap.record_uninitialize(handle)
        if error and os.path.exists(pcap_path) and n == 0:
            os.remove(pcap_path)

    return n

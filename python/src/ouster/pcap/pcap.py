from dataclasses import dataclass, field
import os
import time
from threading import Lock
from typing import Dict, Iterable, Iterator, Optional, Tuple

from ouster.client import (LidarPacket, ImuPacket, Packet, PacketSource,
                           SensorInfo)
from ouster.pcap import _pcap


# TODO: Look into at possible custom OusterExceptions to raise
@dataclass
class PcapInfo:
    """Information queried about a packet capture from an Ouster sensor.

    Type of the output of :func:`.pcap.info`.

    Attributes:
        packets_processed: Total number of all packets processed
        packets_reassembled: Total number of IPv4 UDP packets reassembled
        non_udp_packets: Number of non UDP packets processed
        guessed_lidar_port: Best guess for port with Ouster lidar packets
        guessed_imu_port: Best guess for port with Ouster imu packets
        ports: A mapping from port numbers to the size (in bytes) and number of
            packets receved on that port.
    """
    packets_processed: int = 0
    packets_reassembled: int = 0
    non_udp_packets: int = 0
    guessed_lidar_port: Optional[int] = None
    guessed_imu_port: Optional[int] = None
    ports: Dict[int, Tuple[int, int]] = field(default_factory=dict)


def _guess_imu_port(stream_data: _pcap.stream_info) -> Optional[int]:
    result = None
    imu_size = 48
    if (imu_size in stream_data.packet_size_to_port):
        correct_packet_size = stream_data.packet_size_to_port[imu_size]
        if (len(correct_packet_size) > 1):
            raise ValueError("Error: Multiple possible imu packets found")
        else:
            result = list(correct_packet_size)[0]

    return result


def _guess_lidar_port(stream_data: _pcap.stream_info) -> Optional[int]:
    result = None
    lidar_sizes = {3392, 6464, 12608, 24896}

    hit_count = 0
    multiple_error_msg = "Error: Multiple possible lidar packets found"
    for s in lidar_sizes:
        if s in stream_data.packet_size_to_port:
            correct_packet_size = stream_data.packet_size_to_port[s]
            hit_count += 1
            if (len(correct_packet_size) > 1):
                raise ValueError(multiple_error_msg)
            else:
                result = list(correct_packet_size)[0]

    if (hit_count > 1):
        raise ValueError(multiple_error_msg)

    return result


def _guess_ports(
        stream_data: _pcap.stream_info) -> Tuple[Optional[int], Optional[int]]:
    """Guess the ports for lidar and imu streams from a stream_info struct.

    Args:
        stream_data: The stream_info struct for a pcap file

    Returns:
        A tuple<int lidar_port, int imu_port> for the guessed lidar and imu ports
    """
    return _guess_lidar_port(stream_data), _guess_imu_port(stream_data)


class Pcap(PacketSource):
    """Read a sensor packet stream out of a pcap file as an iterator."""

    _metadata: SensorInfo
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

        pcap_info = _pcap.replay_get_pcap_info(pcap_path, 10000)
        lidar_port_guess, imu_port_guess = _guess_ports(pcap_info)

        # use guessed values unless ports are specified (0 is falsey)
        self._lidar_port = lidar_port or lidar_port_guess
        self._imu_port = imu_port or imu_port_guess

        self._metadata = info
        self._rate = rate
        self._handle = _pcap.replay_initialize(pcap_path, "", "", {})
        self._lock = Lock()

    def __iter__(self) -> Iterator[Packet]:
        buf = bytearray(2**16)
        packet_info = _pcap.packet_info()

        real_start_ts = time.monotonic()
        pcap_start_ts = None

        while True:
            with self._lock:
                if self._handle is None:
                    raise ValueError("I/O operation on closed packet source")
                if not _pcap.next_packet_info(self._handle, packet_info):
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
                yield LidarPacket(buf[0:n], self._metadata, timestamp)

            elif packet_info.dst_port == self._imu_port and n != 0:
                yield ImuPacket(buf[0:n], self._metadata, timestamp)

    @property
    def metadata(self) -> SensorInfo:
        return self._metadata

    def reset(self) -> None:
        """Restart playback from beginning. Thread-safe."""
        with self._lock:
            if self._handle is not None:
                _pcap.replay_reset(self._handle)

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


def info(pcap_path: str, n_packets: int = 1024) -> PcapInfo:
    """Return some stats info from sampling data in a pcap file.

    Args:
        pcap_path: File path of recorded pcap
        n_packets: Number of ip packets / fragments to sample

    Returns:
        A dictionary mapping destination ports to sizes and packet counts.
    """
    info = _pcap.replay_get_pcap_info(pcap_path, n_packets)

    lidar_port, imu_port = _guess_ports(info)

    pcap_info = PcapInfo()
    pcap_info.packets_processed = info.packets_processed
    pcap_info.packets_reassembled = info.packets_reassembled
    pcap_info.non_udp_packets = info.non_udp_packets
    pcap_info.guessed_lidar_port = lidar_port
    pcap_info.guessed_imu_port = imu_port
    pcap_info.ports = {
        port: list(size_count.items())[0]
        for port, size_count in info.port_to_packet_sizes.items()
    }

    return pcap_info


def _replay(pcap_path: str, dst_ip: str, dst_lidar_port: int,
            dst_imu_port: int) -> Iterator[bool]:
    """Replay UDP packets out over the network.

    Todo:
        Not really sure about this interface

    Args:
        pcap_path: Path to the pcap file to replay
        dst_ip: IP to send packets to
        dst_lidar_port: Destination port for lidar packets
        dst_imu_port: Destination port for imu packets

    Returns:
        An iterator that reports whether packets were sent successfully as it's
        consumed.
    """

    pcap_info = _pcap.replay_get_pcap_info(pcap_path, 10000)
    lidar_port_guess, imu_port_guess = _guess_ports(pcap_info)

    pcap_handle = _pcap.replay_initialize(
        pcap_path, dst_ip, dst_ip, {
            (lidar_port_guess or 0): dst_lidar_port,
            (imu_port_guess or 0): dst_imu_port
        })

    try:
        packet_info = _pcap.packet_info()
        while _pcap.next_packet_info(pcap_handle, packet_info):
            yield _pcap.replay_packet(pcap_handle)
    finally:
        _pcap.replay_uninitialize(pcap_handle)

    yield False


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

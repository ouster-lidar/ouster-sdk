import time
from threading import Lock
from typing import Any, Dict, Iterable, Iterator, Optional

from ouster.client import  _pcap, LidarPacket, ImuPacket, Packet, PacketFormat, PacketSource, SensorInfo


class Pcap(PacketSource):
    """Read a sensor packet stream out of pcap as an iterator."""

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
        lidar_port_guess, imu_port_guess = _pcap.guess_ports(pcap_info)

        # use guessed values unless ports are specified (0 is falsey)
        self._lidar_port = lidar_port or lidar_port_guess
        self._imu_port = imu_port or imu_port_guess

        self._metadata = info
        self._rate = rate
        self._handle = _pcap.replay_initialize(pcap_path, "", "", {})
        self._lock = Lock()

    def __iter__(self) -> Iterator[Packet]:
        pf = PacketFormat(self._metadata)

        buf = bytearray(2**16)
        packet_info = _pcap.packet_info()

        real_start_ts = time.monotonic()
        pcap_start_ts = None

        while True:
            with self._lock:
                if self._handle is None:
                    break
                if not _pcap.next_packet_info(self._handle, packet_info):
                    break

                n = _pcap.read_packet(self._handle, buf)

            if self._rate:
                if not pcap_start_ts:
                    pcap_start_ts = packet_info.timestamp
                real_delta = time.monotonic() - real_start_ts
                pcap_delta = (packet_info.timestamp -
                              pcap_start_ts) / self._rate
                delta = max(0, pcap_delta.total_seconds() - real_delta)
                time.sleep(delta)

            if packet_info.dst_port == self._lidar_port and n != 0:
                yield LidarPacket(buf[0:n], pf)

            elif packet_info.dst_port == self._imu_port and n != 0:
                yield ImuPacket(buf[0:n], pf)

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


def info(pcap_path: str, n_packets: int = 512) -> Dict[str, Any]:
    """Return some info from sampling data in a pcap file.

    Args:
        pcap_path: File path of recorded pcap
        n_packets: Number of ip packets / fragments to sample

    Returns:
        A dictionary mapping destination ports to sizes and packet counts.
    """
    info = _pcap.replay_get_pcap_info(pcap_path, n_packets)

    lidar_port, imu_port = _pcap.guess_ports(info)

    result: Dict[str, Any] = {}
    result['ports'] = {
        port: list(size_count.items())[0]
        for port, size_count in info.port_to_packet_sizes.items()
    }
    result['ipv6_packets'] = info.ipv6_packets,
    result['ipv4_packets'] = info.ipv4_packets,
    result['packets_processed'] = info.packets_processed,
    result['packets_reassembled'] = info.packets_reassembled,
    result['non_udp_packets'] = info.non_udp_packets
    result['guessed_lidar_port'] = lidar_port
    result['guessed_imu_port'] = imu_port
    return result


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
    pcap_port_guess = _pcap.guess_ports(pcap_info)

    pcap_handle = _pcap.replay_initialize(pcap_path, dst_ip, dst_ip, {
        pcap_port_guess[0]: dst_lidar_port,
        pcap_port_guess[1]: dst_imu_port
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
           imu_port: int = 7503) -> int:
    """Record a sequence of sensor packets to a pcap.

    Args:
        packets: A (finite!) sequence of packets
        pcap_path: Path of the output pcap file
        src_ip: Source IP to use for all packets
        dst_ip Destination IP to use for all packets
        lidar_port: Src/dst port to use for lidar packets
        imu_port: Src/dst port to use for imu packets

    Returns:
        Number of packets captured
    """

    buf_size = 2**16
    n = 0
    handle = _pcap.record_initialize(pcap_path, src_ip, dst_ip, buf_size)
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
            _pcap.record_packet(handle, src_port, dst_port, packet._data)
            n += 1
    finally:
        _pcap.record_uninitialize(handle)

    return n

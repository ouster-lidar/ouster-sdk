"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""
from collections import defaultdict
from copy import copy
from dataclasses import dataclass, field
from itertools import islice
import os
import socket
import time
from threading import Lock
from typing import (Dict, Iterable, Iterator, List, Optional, Set, Tuple)

from ouster.client import (LidarPacket, ImuPacket, Packet, PacketSource,
                           SensorInfo, _client)
from . import _pcap


@dataclass(frozen=True, order=True)
class _UDPStreamKey:
    """Identifies a single logical 'stream' of UDP packets."""
    src_ip: str
    dst_ip: str
    src_port: int
    dst_port: int


@dataclass
class _UDPStreamInfo:
    """Info about packets in a stream."""
    count: int = 0
    payload_size: Set[int] = field(default_factory=set)
    fragments_in_packet: Set[int] = field(default_factory=set)
    ip_version: Set[int] = field(default_factory=set)


class _stream_info:
    """Gather some useful info about UDP data in a pcap."""

    def __init__(self, infos: Iterable[_pcap.packet_info]) -> None:
        self.total_packets = 0
        self.encapsulation_protocol = set()
        self.timestamp_min = float('inf')
        self.timestamp_max = float('-inf')
        self.udp_streams: Dict[_UDPStreamKey,
                               _UDPStreamInfo] = defaultdict(_UDPStreamInfo)

        for i in infos:
            self.total_packets += 1
            self.encapsulation_protocol.add(i.encapsulation_protocol)
            self.timestamp_min = min(self.timestamp_min, i.timestamp)
            self.timestamp_max = max(self.timestamp_max, i.timestamp)

            val = self.udp_streams[_UDPStreamKey(i.src_ip, i.dst_ip,
                                                 i.src_port, i.dst_port)]
            val.count += 1
            val.payload_size.add(i.payload_size)
            val.fragments_in_packet.add(i.fragments_in_packet)
            val.ip_version.add(i.ip_version)


def _guess_ports(udp_streams: Dict[_UDPStreamKey, _UDPStreamInfo],
                 info: SensorInfo) -> List[Tuple[int, int]]:
    """Find possible UDP sources matching the metadata.

    The current approach is roughly: 1) treat each unique source / destination
    port and IP as a single logical 'stream' of data, 2) filter out streams that
    don't match the expected packet sizes specified by the metadata, 3) pair up
    any potential lidar/imu streams that appear to be coming from the same
    sensor (have matching source IPs) 4) and finally, filter out the pairs that
    contradict any ports specified in the metadata.

    Returns:
        List of dst port pairs that probably contain lidar/imu data. Duplicate
        entries are possible and indicate packets from distinct sources.
    """

    # allow lone streams when there's no matching data from the same ip
    lidar_keys: Set[Optional[_UDPStreamKey]] = {None}
    imu_keys: Set[Optional[_UDPStreamKey]] = {None}

    # find all lidar and imu 'streams' that match expected packet sizes
    pf = _client.PacketFormat.from_info(info)
    ss = udp_streams.items()
    lidar_keys |= {k for k, v in ss if pf.lidar_packet_size in v.payload_size}
    imu_keys |= {k for k, v in ss if pf.imu_packet_size in v.payload_size}

    # find all src ips for candidate streams
    lidar_src_ips = {k.src_ip for k in lidar_keys if k}
    imu_src_ips = {k.src_ip for k in imu_keys if k}

    # yapf: disable
    # "full outer join" on src_ip to produce lidar/imu streams from one source
    keys = [(klidar, kimu) for klidar in lidar_keys for kimu in imu_keys
            if (klidar and kimu and klidar.src_ip == kimu.src_ip)
            or (not klidar and kimu and kimu.src_ip not in lidar_src_ips)
            or (klidar and not kimu and klidar.src_ip not in imu_src_ips)]

    # map down to just dst port pairs, with 0 meaning none found
    ports = [(klidar.dst_port if klidar else 0, kimu.dst_port if kimu else 0)
             for (klidar, kimu) in keys]

    # filter out candidates that don't match specified ports
    lidar_spec, imu_spec = info.udp_port_lidar, info.udp_port_imu
    guesses = [(plidar, pimu) for plidar, pimu in ports
               if (plidar == lidar_spec or lidar_spec == 0 or plidar == 0)
               and (pimu == imu_spec or imu_spec == 0 or pimu == 0)]
    # yapf: enable

    # sort sensor ports to prefer both found > just lidar > just imu
    guesses.sort(reverse=True, key=lambda p: (p[0] != 0, p[1] != 0, p))

    return guesses


def _packet_info_stream(path: str) -> Iterator[_pcap.packet_info]:
    """Read just packet headers without payloads."""
    handle = _pcap.replay_initialize(path)

    try:
        while True:
            pi = _pcap.packet_info()
            if not _pcap.next_packet_info(handle, pi):
                break
            yield pi
    finally:
        _pcap.replay_uninitialize(handle)


class Pcap(PacketSource):
    """Read a sensor packet stream out of a pcap file as an iterator."""

    _metadata: SensorInfo
    _rate: float
    _handle: Optional[_pcap.playback_handle]
    _lock: Lock

    def __init__(self,
                 pcap_path: str,
                 info: SensorInfo,
                 *,
                 rate: float = 0.0,
                 lidar_port: Optional[int] = None,
                 imu_port: Optional[int] = None):
        """Read a single sensor data stream from a packet capture.

        Packet captures can contain arbitrary network traffic or even multiple
        valid sensor data streans. To avoid passing invalid data to the user,
        this class assumes that lidar and/or imu packets are associated with
        distinct destination ports, which may be recorded in the sensor metadata
        or specified explicitly.

        When not specified, ports are guessed by sampling some packets and
        looking for the expected packet size based on the sensor metadata. If
        packets that might be valid sensor data appear on multiple ports, one is
        chosen arbitrarily. See ``_guess_ports`` for details. on the heuristics.

        Packets with the selected destination port that clearly don't match the
        metadata (e.g. wrong size or init_id) will be silently ignored.

        When a rate is specified, output packets in (a multiple of) real time
        using the pcap packet capture timestamps.

        Args:
            info: Sensor metadata
            pcap_path: File path of recorded pcap
            rate: Output packets in real time, if non-zero
            lidar_port: Specify the destination port of lidar packets
            imu_port: Specify the destination port of imu packets
        """

        # prefer explicitly specified ports (can probably remove the args?)
        lidar_port = info.udp_port_lidar if lidar_port is None else lidar_port
        imu_port = info.udp_port_imu if imu_port is None else imu_port

        # override ports in metadata, if explicitly specified
        self._metadata = copy(info)
        self._metadata.udp_port_lidar = lidar_port
        self._metadata.udp_port_imu = imu_port

        # sample pcap and attempt to find UDP ports consistent with metadata
        n_packets = 1000
        stats = _stream_info(islice(_packet_info_stream(pcap_path), n_packets))
        self._guesses = _guess_ports(stats.udp_streams, self._metadata)

        # fill in unspecified (0) ports with inferred values
        if len(self._guesses) > 0:
            lidar_guess, imu_guess = self._guesses[0]
            # guess != port only if port == 0 or guess == 0
            self._metadata.udp_port_lidar = lidar_guess or lidar_port
            self._metadata.udp_port_imu = imu_guess or imu_port

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
        while True:
            with self._lock:
                if not (self._handle
                        and _pcap.next_packet_info(self._handle, packet_info)):
                    break
                n = _pcap.read_packet(self._handle, buf)

            # if rate is set, read in 'real time' simulating UDP stream
            # TODO: factor out into separate packet iterator utility
            timestamp = packet_info.timestamp
            if self._rate:
                if not pcap_start_ts:
                    pcap_start_ts = timestamp
                real_delta = time.monotonic() - real_start_ts
                pcap_delta = (timestamp - pcap_start_ts) / self._rate
                delta = max(0, pcap_delta - real_delta)
                time.sleep(delta)

            try:
                if (packet_info.dst_port == self._metadata.udp_port_lidar):
                    yield LidarPacket(buf[0:n], self._metadata, timestamp)
                elif (packet_info.dst_port == self._metadata.udp_port_imu):
                    yield ImuPacket(buf[0:n], self._metadata, timestamp)
            except ValueError:
                # TODO: bad packet size or init_id here, use specific exceptions
                pass

    @property
    def metadata(self) -> SensorInfo:
        return self._metadata

    @property
    def ports(self) -> Tuple[int, int]:
        """Specified or inferred ports associated with lidar and imu data.

        Values <= 0 indicate that no lidar or imu data will be read.
        """
        return (self._metadata.udp_port_lidar, self._metadata.udp_port_imu)

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
        use_sll_encapsulation: Use sll encapsulaiton

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

"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""
from copy import copy
import os
import socket
import time
from threading import Lock
from collections import defaultdict
from typing import (Iterable, Iterator, Optional, Tuple, Dict)  # noqa: F401

from ouster.client import (LidarPacketValidator, LidarPacket, ImuPacket, Packet, PacketSource,  # noqa: F401
                           SensorInfo, _client, PacketValidationFailure, PacketIdError)         # noqa: F401
from . import _pcap

MTU_SIZE = 1500


def _guess_ports(stream_info, sensor_info):
    pf = _client.PacketFormat.from_info(sensor_info)
    lidar_spec, imu_spec = sensor_info.udp_port_lidar, sensor_info.udp_port_imu
    guesses = [(i.lidar, i.imu) for i in _pcap.guess_ports(
        stream_info, pf.lidar_packet_size, pf.imu_packet_size,
        lidar_spec, imu_spec)]
    guesses.sort(reverse=True, key=lambda p: (p[0] != 0, p[1] != 0, p))
    return guesses


def _packet_info_stream(path: str, n_packets, progress_callback=None, callback_frequency=1):
    if progress_callback is not None:
        result = _pcap.get_stream_info(path, progress_callback, callback_frequency, n_packets)
    else:
        result = _pcap.get_stream_info(path, n_packets)
    return result


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
                 imu_port: Optional[int] = None,
                 loop: bool = False,
                 _soft_id_check: bool = False):
        """Read a single sensor data stream from a packet capture.

        Packet captures can contain arbitrary network traffic or even multiple
        valid sensor data streams. To avoid passing invalid data to the user,
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
            loop: Specify whether to reload the PCAP file when the end is reached
            _soft_id_check: if True, don't skip lidar packets buffers on init_id/sn mismatch
        """

        # prefer explicitly specified ports (can probably remove the args?)
        lidar_port = info.udp_port_lidar if lidar_port is None else lidar_port
        imu_port = info.udp_port_imu if imu_port is None else imu_port

        # override ports in metadata, if explicitly specified
        self._metadata = copy(info)
        self._metadata.udp_port_lidar = lidar_port
        self._metadata.udp_port_imu = imu_port

        self.loop = loop
        self._soft_id_check = _soft_id_check
        self._id_error_count = 0    # TWS 20230615 TODO generialize error counting and reporting
        self._errors = defaultdict(int)  # type: Dict[PacketValidationFailure,int]

        # sample pcap and attempt to find UDP ports consistent with metadata
        n_packets = 1000
        stats = _packet_info_stream(pcap_path, n_packets)
        self._guesses = _guess_ports(stats, self._metadata)

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
        validator = LidarPacketValidator(self.metadata)
        while True:
            with self._lock:
                if not self._handle:
                    break
                if not _pcap.next_packet_info(self._handle, packet_info):
                    if self.loop:
                        _pcap.replay_reset(self._handle)
                        _pcap.next_packet_info(self._handle, packet_info)
                    else:
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
                    for error in validator.check_packet(buf, n):
                        self._errors[error] += 1  # accumulate counts of errors
                    lp = LidarPacket(
                        buf[0:n],
                        self._metadata,
                        timestamp,
                        _raise_on_id_check=not self._soft_id_check)
                    if lp.id_error:
                        self._id_error_count += 1
                    yield lp
                elif (packet_info.dst_port == self._metadata.udp_port_imu):
                    yield ImuPacket(buf[0:n], self._metadata, timestamp)
            except PacketIdError:
                self._id_error_count += 1
            except ValueError:
                # bad packet size: this can happen when
                # packets are buffered by the OS, not necessarily an error
                # same pass as in core.py
                # TODO: introduce status for PacketSource to indicate frequency
                # of bad packet size or init_id/sn errors
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

    @property
    def id_error_count(self) -> int:
        return self._id_error_count

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
        use_sll_encapsulation: Use sll encapsulation

    Returns:
        Number of packets captured
    """
    has_timestamp = None
    error = False
    n = 0
    handle = _pcap.record_initialize(pcap_path, MTU_SIZE,
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
            _pcap.record_packet(handle, src_ip, dst_ip, src_port, dst_port, packet._data, ts)
            n += 1
    except Exception:
        error = True
        raise
    finally:
        _pcap.record_uninitialize(handle)
        if error and os.path.exists(pcap_path) and n == 0:
            os.remove(pcap_path)

    return n

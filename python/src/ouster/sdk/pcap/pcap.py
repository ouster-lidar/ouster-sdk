"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""
import os
import socket
import time
import warnings
from typing import (Iterable, Iterator, Optional, Tuple, Dict)  # noqa: F401

from ouster.sdk.client import (LidarPacket, ImuPacket, Packet, PacketSource,  # noqa: F401
                           SensorInfo, PacketValidationFailure)      # noqa: F401
import ouster.sdk._bindings.pcap as _pcap
import ouster.sdk._bindings.client as _client

MTU_SIZE = 1500


def _guess_ports(stream_info, sensor_info):
    pf = _client.PacketFormat.from_info(sensor_info)
    lidar_spec, imu_spec = sensor_info.config.udp_port_lidar, sensor_info.config.udp_port_imu
    guesses = [(i.lidar, i.imu) for i in _pcap.guess_ports(
        stream_info, pf.lidar_packet_size, pf.imu_packet_size,
        lidar_spec or 0, imu_spec or 0)]
    guesses.sort(reverse=True, key=lambda p: (p[0] != 0, p[1] != 0, p))
    return guesses


def _packet_info_stream(path: str, n_packets, progress_callback=None, callback_frequency=1):
    if progress_callback is not None:
        result = _pcap.get_stream_info(path, progress_callback, callback_frequency, n_packets)
    else:
        result = _pcap.get_stream_info(path, n_packets)
    return result


class Pcap(PacketSource):
    """Deprecated: Read a sensor packet stream out of a pcap file as an iterator."""

    def __init__(self,
                 pcap_path: str,
                 info: SensorInfo,
                 *,
                 rate: float = 0.0,
                 loop: bool = False,
                 soft_id_check: bool = False):
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
            soft_id_check: if True, don't skip lidar packets buffers on init_id/sn mismatch
        """
        warnings.warn("pcap.Pcap(...) is deprecated: "
                  "Use pcap.PcapMultiPacketReader(...).single_source(0) instead. "
                  "This API is planned to be removed in Q4 2024.",
                  DeprecationWarning, stacklevel=2)
        from ouster.sdk.pcap import PcapMultiPacketReader
        self._source = PcapMultiPacketReader(pcap_path, [], rate=rate, metadatas=[info],
                                             soft_id_check=soft_id_check)

    def __iter__(self) -> Iterator[Packet]:
        for idx, packet in self._source:
            if idx == 0:
                yield packet

    @property
    def is_live(self) -> bool:
        return False

    @property
    def metadata(self) -> SensorInfo:
        return self._source._metadata[0]

    @property
    def ports(self) -> Tuple[int, int]:
        """Specified or inferred ports associated with lidar and imu data.

        Values of 0 indicate that no data of that type will be read.
        """
        return (self._source._metadata[0].config.udp_port_lidar or 0,
                self._source._metadata[0].config.udp_port_imu or 0)

    def reset(self) -> None:
        """Restart playback from beginning. Thread-safe."""
        self._source.restart()

    @property
    def closed(self) -> bool:
        """Check if source is closed. Thread-safe."""
        return self._source.closed

    @property
    def id_error_count(self) -> int:
        return self._source.id_error_count

    @property
    def size_error_count(self) -> int:
        return self._source.size_error_count

    def close(self) -> None:
        """Release resources. Thread-safe."""
        self._source.close()


def _replay(pcap_path: str, info: SensorInfo, dst_ip: str, dst_lidar_port: int,
            dst_imu_port: int, address: Optional[Tuple[str, int]] = None) -> Iterator[bool]:
    """Replay UDP packets out over the network.

    Todo:
        Not really sure about this interface

    Args:
        pcap_path: Path to the pcap file to replay
        info: The sensor metadata
        dst_ip: IP to send packets to
        dst_lidar_port: Destination port for lidar packets
        dst_imu_port: Destination port for imu packets
        address: Address and port to bind to send packets from. Optional.

    Returns:
        An iterator that reports whether packets were sent successfully as it's
        consumed.
    """
    try:
        socket_out = socket.socket(family=socket.AF_INET,
                                   type=socket.SOCK_DGRAM)
        pcap_handle = Pcap(pcap_path, info)
        if address is not None:
            socket_out.bind(address)
        for item in pcap_handle:
            port = 0
            if isinstance(item, LidarPacket):
                port = dst_lidar_port
            if isinstance(item, ImuPacket):
                port = dst_imu_port

            socket_out.sendto(item.buf.tobytes(), (dst_ip, port))
            yield True
        yield False
    finally:
        if pcap_handle is not None:
            pcap_handle.close()
        if socket_out:
            socket_out.close()


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
                has_timestamp = (packet.host_timestamp != 0)
            elif has_timestamp != (packet.host_timestamp != 0):
                raise ValueError("Mixing timestamped/untimestamped packets")

            ts = packet.host_timestamp / 1e9
            if ts == 0.0:
                ts = time.time()
            _pcap.record_packet(handle, src_ip, dst_ip, src_port, dst_port, packet.buf, ts)
            n += 1
    except Exception:
        error = True
        raise
    finally:
        _pcap.record_uninitialize(handle)
        if error and os.path.exists(pcap_path) and n == 0:
            os.remove(pcap_path)

    return n

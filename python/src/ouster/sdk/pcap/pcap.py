"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""
import os
import socket
import time
from typing import (Iterable, Iterator, Optional, Tuple, Dict)  # noqa: F401

from ouster.sdk.core import (LidarPacket, ImuPacket, Packet, PacketSource,  # noqa: F401
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
        pcap_handle = _pcap.PcapPacketSource(pcap_path, sensor_info=[info])
        if address is not None:
            socket_out.bind(address)
        for idx, item in pcap_handle:
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

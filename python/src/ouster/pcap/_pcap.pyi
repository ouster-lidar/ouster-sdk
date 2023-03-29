"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Type annotations for pcap python bindings.
"""

from typing import (overload, List, Callable)

from ..client.data import BufferT


class playback_handle:
    pass


class record_handle:
    pass


class guessed_ports:
    def __init__(self) -> None:
        ...

    lidar: int
    imu:   int


class stream_info:
    def __init__(self) -> None:
        ...

    total_packets: int
    encapsulation_protocol: int
    

def guess_ports(file: str, 
                meta_data_file: str, 
                packets_to_process: int) -> List[guessed_ports]:
    ...


@overload
def get_stream_info(file: str, packets_to_process: int) -> stream_info:
    pass


@overload
def get_stream_info(file: str, progress_callback: Callable[[int, int], int],
                    callback_frequency: int,
                    packets_to_process: int) -> stream_info:
    pass


class packet_info:

    def __init__(self) -> None:
        ...

    dst_ip: str
    src_ip: str
    dst_port: int
    src_port: int

    @property
    def payload_size(self) -> int:
        ...

    @property
    def timestamp(self) -> float:
        ...

    @timestamp.setter
    def timestamp(self, seconds: float) -> None:
        ...

    @property
    def fragments_in_packet(self) -> int:
        ...

    @property
    def ip_version(self) -> int:
        ...

    @property
    def encapsulation_protocol(self) -> int:
        ...

    @property
    def network_protocol(self) -> int:
        ...


def replay_initialize(file_name: str) -> playback_handle:
    ...


def replay_uninitialize(handle: playback_handle) -> None:
    ...


def next_packet_info(handle: playback_handle, pi: packet_info) -> bool:
    ...


def read_packet(handle: playback_handle, buf: BufferT) -> int:
    ...


def replay_reset(handle: playback_handle) -> None:
    ...


def record_initialize(file_name: str,
                      frag_size: int,
                      use_sll_encapsulation: bool = ...) -> record_handle:
    ...


def record_uninitialize(handle: record_handle) -> None:
    ...


@overload
def record_packet(handle: record_handle, src_ip: str, dst_ip: str,
                  src_port: int, dst_port: int, buf: BufferT,
                  timestamp: float) -> None:
    ...


@overload
def record_packet(handle: record_handle, info: packet_info,
                  buf: BufferT) -> None:
    ...

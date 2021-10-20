"""Type annotations for pcap python bindings."""
from ouster.client import BufferT
from typing import Dict


class playback_handle:
    def __init__(self) -> None:
        ...


class record_handle:
    def __init__(self) -> None:
        ...


class packet_info:
    def __init__(self) -> None:
        ...

    @property
    def dst_ip(self) -> str:
        ...

    @property
    def dst_port(self) -> int:
        ...

    @property
    def payload_size(self) -> int:
        ...

    @property
    def src_ip(self) -> str:
        ...

    @property
    def src_port(self) -> int:
        ...

    @property
    def timestamp(self) -> float:
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
    def network_protocol(self) -> float:
        ...


def next_packet_info(handle: playback_handle, pi: packet_info) -> bool:
    ...


def read_packet(handle: playback_handle, buf: BufferT) -> int:
    ...


def record_initialize(file_name: str,
                      src_ip: str,
                      dst_ip: str,
                      frag_size: int,
                      use_sll_encapsulation: bool = ...) -> record_handle:
    ...


def record_uninitialize(handle: record_handle) -> None:
    ...


def record_packet(handle: record_handle, src_port: int, dst_port: int,
                  buf: BufferT, timestamp: float) -> None:
    ...


def replay_initialize(file_name: str) -> playback_handle:
    ...


def replay_packet(handle: playback_handle) -> bool:
    ...


def replay_pcap(handle: playback_handle, rate: float) -> int:
    ...


def replay_uninitialize(handle: playback_handle) -> None:
    ...


def replay_reset(handle: playback_handle) -> None:
    ...

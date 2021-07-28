"""Type annotations for pcap python bindings."""
from ouster.client import BufferT
from typing import Dict


class playback_handle:
    def __init__(self) -> None:
        ...


class record_handle:
    def __init__(self) -> None:
        ...


class stream_info:
    def __init__(self) -> None:
        ...

    @property
    def ipv4_packets(self) -> int:
        ...

    @property
    def ipv6_packets(self) -> int:
        ...

    @property
    def non_udp_packets(self) -> int:
        ...

    @property
    def packet_size_to_port(self) -> Dict[int, Dict[int, int]]:
        ...

    @property
    def packets_processed(self) -> int:
        ...

    @property
    def packets_reassembled(self) -> int:
        ...

    @property
    def port_to_packet_count(self) -> Dict[int, int]:
        ...

    @property
    def port_to_packet_sizes(self) -> Dict[int, Dict[int, int]]:
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


def replay_get_pcap_info(file_name: str,
                         packets_to_process: int) -> stream_info:
    ...


def replay_initialize(file_name: str, src_ip: str, dst_ip: str,
                      port_map: Dict[int, int]) -> playback_handle:
    ...


def replay_packet(handle: playback_handle) -> bool:
    ...


def replay_pcap(handle: playback_handle, rate: float) -> int:
    ...


def replay_uninitialize(handle: playback_handle) -> None:
    ...


def replay_reset(handle: playback_handle) -> None:
    ...

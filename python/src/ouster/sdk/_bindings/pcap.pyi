"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Type annotations for pcap python bindings.
"""
import numpy
from typing import Dict, overload, List, Callable, Optional
from ouster.sdk.core.data import BufferT
from ouster.sdk.core import ScanSource, PacketSource, SensorInfo

class PcapDuplicatePortException(Exception):
    ...

class PcapPacketSource(PacketSource):

    def __init__(self, source: str, *, sensor_info: Optional[List[SensorInfo]]=None, meta: Optional[List[str]]=None, extrinsics: List[numpy.ndarray]=[], extrinsics_file: str='', index: bool=False, soft_id_check: bool=False) -> None:
        """__init__(self: ouster.sdk._bindings.pcap.PcapPacketSource, file: str, **kwargs) -> None
"""
        ...

    @property
    def id_error_count(self) -> int:
        ...

    @property
    def size_error_count(self) -> int:
        ...

class PcapScanSource(ScanSource):

    def __init__(self, source: str, *, sensor_info: Optional[List[SensorInfo]]=None, meta: Optional[List[str]]=None, extrinsics: List[numpy.ndarray]=[], extrinsics_file: str='', index: bool=False, soft_id_check: bool=False, field_names: List[str]=[], raw_fields: bool=False, raw_headers: bool=False) -> None:
        """__init__(self: ouster.sdk._bindings.pcap.PcapScanSource, file: str, **kwargs) -> None
"""
        ...

    @property
    def id_error_count(self) -> int:
        ...

    @property
    def size_error_count(self) -> int:
        ...

class PlaybackHandle:
    pass

class RecordHandle:
    pass

class GuessedPorts:

    def __init__(self) -> None:
        """__init__(self: ouster.sdk._bindings.pcap.GuessedPorts) -> None
"""
        ...

    @property
    def lidar(self) -> int:
        ...

    @property
    def imu(self) -> int:
        ...

class StreamInfo:

    def __init__(self) -> None:
        """__init__(self: ouster.sdk._bindings.pcap.StreamInfo) -> None
"""
        ...

    @property
    def total_packets(self) -> int:
        ...

    @property
    def encapsulation_protocol(self) -> int:
        ...

    @property
    def timestamp_min(self) -> float:
        ...

    @property
    def timestamp_max(self) -> float:
        ...

    @property
    def udp_streams(self) -> object:
        ...

def guess_ports(file: str, meta_data_file: str, packets_to_process: int) -> List[GuessedPorts]:
    """guess_ports(arg0: ouster.sdk._bindings.pcap.StreamInfo, arg1: int, arg2: int, arg3: int, arg4: int) -> ouster.sdk._bindings.VectorGuessedPorts
"""
    ...

@overload
def get_stream_info(file: str, packets_to_process: int) -> StreamInfo:
    """get_stream_info(*args, **kwargs)
Overloaded function.

1. get_stream_info(arg0: str, arg1: int) -> ouster.sdk._bindings.pcap.StreamInfo

2. get_stream_info(arg0: str, arg1: Callable[[int, int, int], None], arg2: int, arg3: int) -> ouster.sdk._bindings.pcap.StreamInfo
"""
    pass

@overload
def get_stream_info(file: str, progress_callback: Callable[[int, int], int], callback_frequency: int, packets_to_process: int) -> StreamInfo:
    """get_stream_info(*args, **kwargs)
Overloaded function.

1. get_stream_info(arg0: str, arg1: int) -> ouster.sdk._bindings.pcap.StreamInfo

2. get_stream_info(arg0: str, arg1: Callable[[int, int, int], None], arg2: int, arg3: int) -> ouster.sdk._bindings.pcap.StreamInfo
"""
    pass

class StreamData:

    @property
    def count(self) -> int:
        ...

    @property
    def payload_size_counts(self) -> Dict[int, int]:
        ...

    @property
    def fragment_counts(self) -> Dict[int, int]:
        ...

    @property
    def ip_version_counts(self) -> Dict[int, int]:
        ...

class StreamKey:

    @property
    def dst_ip(self) -> str:
        ...

    @property
    def src_ip(self) -> str:
        ...

    @property
    def src_port(self) -> int:
        ...

    @property
    def dst_port(self) -> int:
        ...

class PacketInfo:

    def __init__(self) -> None:
        """__init__(self: ouster.sdk._bindings.pcap.PacketInfo) -> None
"""
        ...
    dst_ip: str
    src_ip: str
    dst_port: int
    src_port: int

    @property
    def file_offset(self) -> int:
        ...

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

class PcapIndex:

    def __init__(self, int) -> None:
        """__init__(self: ouster.sdk._bindings.pcap.PcapIndex, arg0: int) -> None
"""
        ...

    @property
    def frame_id_indices(self) -> List[Dict[int, int]]:
        ...

    def frame_count(self, int) -> int:
        """
            Get the total number of frames in the PCAP file.

            Returns:
                int: The total number of frames.
            """
        ...

    @property
    def frame_indices(self) -> List[numpy.ndarray]:
        """
            Get the indices of frames in the PCAP file.
            """
        ...

    @property
    def frame_timestamp_indices(self) -> List[Dict[int, int]]:
        ...

    def seek_to_frame(self, reader: PcapReader, sensor_idx: int, frame_number: int):
        """
            Seek to a specific frame in the PCAP file.

            Args:
                frame_index (int): The index of the frame to seek to.
            """
        ...

class PcapReader:
    ...

class IndexedPcapReader(PcapReader):

    def __init__(self, filename: str, metadata_filename: List[str]) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.pcap.IndexedPcapReader, arg0: str, arg1: list[str]) -> None

2. __init__(self: ouster.sdk._bindings.pcap.IndexedPcapReader, arg0: str, arg1: list[ouster.sdk._bindings.client.SensorInfo]) -> None
"""
        ...

    def build_index(self) -> None:
        """build_index(self: ouster.sdk._bindings.pcap.IndexedPcapReader) -> None
"""
        ...

    def next_packet(self) -> int:
        """next_packet(self: ouster.sdk._bindings.pcap.IndexedPcapReader) -> int
"""
        ...

    def current_info(self) -> PacketInfo:
        """current_info(self: ouster.sdk._bindings.pcap.IndexedPcapReader) -> ouster.sdk._bindings.pcap.PacketInfo
"""
        ...

    def current_data(self) -> BufferT:
        """current_data(self: ouster.sdk._bindings.pcap.IndexedPcapReader) -> numpy.ndarray
"""
        ...

    def get_index(self) -> PcapIndex:
        """get_index(self: ouster.sdk._bindings.pcap.IndexedPcapReader) -> ouster.sdk._bindings.pcap.PcapIndex
"""
        ...

    def seek(self, int) -> None:
        """seek(self: ouster.sdk._bindings.pcap.IndexedPcapReader, arg0: int) -> None
"""
        ...

    def reset(self) -> None:
        """reset(self: ouster.sdk._bindings.pcap.IndexedPcapReader) -> None
"""
        ...

    def current_frame_id(self) -> Optional[int]:
        """current_frame_id(self: ouster.sdk._bindings.pcap.IndexedPcapReader) -> object
"""
        ...

    def update_index_for_current_packet(self) -> int:
        """update_index_for_current_packet(self: ouster.sdk._bindings.pcap.IndexedPcapReader) -> int
"""
        ...

def replay_initialize(file_name: str) -> PlaybackHandle:
    """replay_initialize(arg0: str) -> ouster.sdk._bindings.pcap.PlaybackHandle
"""
    ...

def replay_uninitialize(handle: PlaybackHandle) -> None:
    """replay_uninitialize(arg0: ouster.sdk._bindings.pcap.PlaybackHandle) -> None
"""
    ...

def next_packet_info(handle: PlaybackHandle, pi: PacketInfo) -> bool:
    """next_packet_info(arg0: ouster.sdk._bindings.pcap.PlaybackHandle, arg1: ouster.sdk._bindings.pcap.PacketInfo) -> bool
"""
    ...

def read_packet(handle: PlaybackHandle, buf: BufferT) -> int:
    """read_packet(arg0: ouster.sdk._bindings.pcap.PlaybackHandle, arg1: Buffer) -> int
"""
    ...

def replay_reset(handle: PlaybackHandle) -> None:
    """replay_reset(arg0: ouster.sdk._bindings.pcap.PlaybackHandle) -> None
"""
    ...

def record_initialize(file_name: str, frag_size: int, use_sll_encapsulation: bool=...) -> RecordHandle:
    """
                ``def record_initialize(file_name: str, frag_size: int,
                      use_sll_encapsulation: bool = ...) -> RecordHandle:``

                  Initialize record handle for single sensor pcap files

            """
    ...

def record_uninitialize(handle: RecordHandle) -> None:
    """record_uninitialize(arg0: ouster.sdk._bindings.pcap.RecordHandle) -> None
"""
    ...

@overload
def record_packet(handle: RecordHandle, src_ip: str, dst_ip: str, src_port: int, dst_port: int, buf: BufferT, timestamp: float) -> None:
    """record_packet(*args, **kwargs)
Overloaded function.

1. record_packet(arg0: ouster.sdk._bindings.pcap.RecordHandle, arg1: str, arg2: str, arg3: int, arg4: int, arg5: Buffer, arg6: float) -> None

2. record_packet(arg0: ouster.sdk._bindings.pcap.RecordHandle, arg1: ouster.sdk._bindings.pcap.PacketInfo, arg2: Buffer) -> None
"""
    ...

@overload
def record_packet(handle: RecordHandle, info: PacketInfo, buf: BufferT) -> None:
    """record_packet(*args, **kwargs)
Overloaded function.

1. record_packet(arg0: ouster.sdk._bindings.pcap.RecordHandle, arg1: str, arg2: str, arg3: int, arg4: int, arg5: Buffer, arg6: float) -> None

2. record_packet(arg0: ouster.sdk._bindings.pcap.RecordHandle, arg1: ouster.sdk._bindings.pcap.PacketInfo, arg2: Buffer) -> None
"""
    ...
packet_info = PacketInfo
stream_key = StreamKey
guessed_ports = GuessedPorts
stream_data = StreamData
stream_info = StreamInfo
record_handle = RecordHandle
playback_handle = PlaybackHandle
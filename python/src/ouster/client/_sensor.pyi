"""Type annotations for the sensor client python bindings.

This is a mypy stub file defining just tye type signatures of the module
generated by pybind11. It was generated using the ``stubgen`` utility and then
modified.

Note:
    This file should be updated whenever the bindings are modified.

"""
# flake8: noqa (linter complains about scoping, but afaict mypy doesn't care)

from numpy import ndarray
from typing import Any, Callable, ClassVar, List, Optional, overload, Union

from . import BufferT


class Client:
    @overload
    def __init__(self,
                 hostname: str = ...,
                 lidar_port: int = ...,
                 imu_port: int = ...,
                 capacity: int = ...) -> None:
        ...

    @overload
    def __init__(self,
                 hostname: str,
                 udp_dest_host: str,
                 mode: LidarMode = ...,
                 ts_mode: TimestampMode = ...,
                 lidar_port: int = ...,
                 imu_port: int = ...,
                 timeout_sec: int = ...,
                 capacity: int = ...) -> None:
        ...

    def get_metadata(self, timeout_sec: int = ...) -> str:
        ...

    def shutdown(self) -> None:
        ...

    def consume(self, buf: bytearray, timeout_sec: float) -> ClientState:
        ...

    def produce(self, pf: PacketFormat) -> ClientState:
        ...

    def flush(self, n_packets: int = ...) -> None:
        ...

    @property
    def capacity(self) -> int:
        ...

    @property
    def size(self) -> int:
        ...


class ClientState:
    ERROR: ClassVar[ClientState]
    EXIT: ClassVar[ClientState]
    IMU_DATA: ClassVar[ClientState]
    LIDAR_DATA: ClassVar[ClientState]
    TIMEOUT: ClassVar[ClientState]
    OVERFLOW: ClassVar[ClientState]

    def __init__(self, x: int) -> None:
        ...

    def __and__(self, s: ClientState) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __getstate__(self) -> tuple:
        ...

    def __hash__(self) -> int:
        ...

    def __int__(self) -> int:
        ...

    def __invert__(self) -> int:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __or__(self, s: ClientState) -> int:
        ...

    def __setstate__(self, st: tuple) -> None:
        ...

    def __xor__(self, s: ClientState) -> int:
        ...

    @property
    def __members__(self) -> dict:
        ...


class SensorInfo:
    hostname: str
    sn: str
    fw_rev: str
    mode: LidarMode
    prod_line: str
    format: DataFormat
    beam_azimuth_angles: List[float]
    beam_altitude_angles: List[float]
    imu_to_sensor_transform: ndarray
    lidar_to_sensor_transform: ndarray
    extrinsic: ndarray

    @classmethod
    def from_default(cls, mode: LidarMode) -> SensorInfo:
        ...

    @overload
    def __init__(self) -> None:
        ...

    @overload
    def __init__(self, metadata: str) -> None:
        ...


class DataFormat:
    columns_per_frame: int
    columns_per_packet: int
    pixel_shift_by_row: List[int]
    pixels_per_column: int


class PacketFormat:
    def __init__(self, info: SensorInfo) -> None:
        ...

    @property
    def lidar_packet_size(self) -> int:
        ...

    @property
    def imu_packet_size(self) -> int:
        ...

    @property
    def columns_per_packet(self) -> int:
        ...

    @property
    def pixels_per_column(self) -> int:
        ...

    @property
    def encoder_ticks_per_rev(self) -> int:
        ...

    def col_timestamp(self, col: int, buf: bytes) -> int:
        ...

    def col_encoder(self, col: int, buf: bytes) -> int:
        ...

    def col_measurement_id(self, col: int, buf: bytes) -> int:
        ...

    def col_frame_id(self, col: int, buf: bytes) -> int:
        ...

    def col_status(self, col: int, buf: bytes) -> int:
        ...

    def imu_sys_ts(self, buf: BufferT) -> int:
        ...

    def imu_accel_ts(self, buf: BufferT) -> int:
        ...

    def imu_gyro_ts(self, buf: BufferT) -> int:
        ...

    def imu_av_x(self, buf: BufferT) -> float:
        ...

    def imu_av_y(self, buf: BufferT) -> float:
        ...

    def imu_av_z(self, buf: BufferT) -> float:
        ...

    def imu_la_x(self, buf: BufferT) -> float:
        ...

    def imu_la_y(self, buf: BufferT) -> float:
        ...

    def imu_la_z(self, buf: BufferT) -> float:
        ...


class LidarMode:
    MODE_1024x10: ClassVar[LidarMode]
    MODE_1024x20: ClassVar[LidarMode]
    MODE_2048x10: ClassVar[LidarMode]
    MODE_512x10: ClassVar[LidarMode]
    MODE_512x20: ClassVar[LidarMode]

    def __init__(self, code: int) -> None:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __getstate__(self) -> tuple:
        ...

    def __hash__(self) -> int:
        ...

    def __int__(self) -> int:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __setstate__(self, st: tuple) -> None:
        ...

    @property
    def cols(self) -> int:
        ...

    @property
    def frequency(self) -> int:
        ...

    @property
    def __members__(self) -> dict:
        ...

    @classmethod
    def from_string(cls, s: str) -> LidarMode:
        ...


class TimestampMode:
    TIME_FROM_INTERNAL_OSC: ClassVar[TimestampMode]
    TIME_FROM_PTP_1588: ClassVar[TimestampMode]
    TIME_FROM_SYNC_PULSE_IN: ClassVar[TimestampMode]

    def __init__(self, code: int) -> None:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __getstate__(self) -> tuple:
        ...

    def __hash__(self) -> int:
        ...

    def __int__(self) -> int:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __setstate__(self, st: tuple) -> None:
        ...

    @property
    def __members__(self) -> dict:
        ...

    @classmethod
    def from_string(cls, s: str) -> TimestampMode:
        ...


class Version:
    major: int
    minor: int
    patch: int

    def __init__(self) -> None:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __le__(self, v: Version) -> bool:
        ...

    def __lt__(self, v: Version) -> bool:
        ...

    @classmethod
    def of_string(cls, s: str) -> Version:
        ...


class BlockHeader:
    encoder: int
    status: int

    def __init__(self, timestamp: int, encoder: int, status: int):
        ...

    @property
    def timestamp(self) -> int:
        ...


class LidarScan:
    frame_id: int
    headers: List[BlockHeader]

    def __init__(self, w: int, h: int) -> None:
        ...

    @property
    def w(self) -> int:
        ...

    @property
    def h(self) -> int:
        ...

    @property
    def data(self) -> ndarray:
        ...


def destagger(field: ndarray, info: SensorInfo) -> ndarray:
    ...


class ScanBatcher:
    def __init__(self, w: int, pf: PacketFormat) -> None:
        ...

    def __call__(self, buf: BufferT, ls: LidarScan) -> bool:
        ...


class XYZLut:
    def __init__(self, info: SensorInfo) -> None:
        ...

    def __call__(self, scan: LidarScan) -> ndarray:
        ...

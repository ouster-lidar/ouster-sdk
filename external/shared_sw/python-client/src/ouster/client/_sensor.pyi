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

BufferT = Union[bytes, bytearray, memoryview]

ERROR: ClientState
EXIT: ClientState
IMU_DATA: ClientState
LIDAR_DATA: ClientState
TIMEOUT: ClientState

MODE_1024x10: LidarMode
MODE_1024x20: LidarMode
MODE_2048x10: LidarMode
MODE_512x10: LidarMode
MODE_512x20: LidarMode

TIME_FROM_INTERNAL_OSC: TimestampMode
TIME_FROM_PTP_1588: TimestampMode
TIME_FROM_SYNC_PULSE_IN: TimestampMode

invalid_version: Version
min_version: Version


class Client:
    ...


@overload
def init_client(hostname: str = ...,
                lidar_port: int = ...,
                imu_port: int = ...) -> Optional[Client]:
    ...


@overload
def init_client(hostname: str,
                udp_dest_host: str,
                mode: LidarMode = ...,
                ts_mode: TimestampMode = ...,
                lidar_port: int = ...,
                imu_port: int = ...,
                timeout_sec: int = ...) -> Optional[Client]:
    ...


def get_metadata(cli: Client, timeout_sec: int = ...) -> str:
    ...


def poll_client(cli: Client, timeout_sec: int = ...) -> ClientState:
    ...


def read_imu_packet(cli: Client, buf: bytearray, pf: PacketFormat) -> bool:
    ...


def read_lidar_packet(cli: Client, buf: bytearray, pf: PacketFormat) -> bool:
    ...


class ClientState:
    ERROR: ClassVar[ClientState]
    EXIT: ClassVar[ClientState]
    IMU_DATA: ClassVar[ClientState]
    LIDAR_DATA: ClassVar[ClientState]
    TIMEOUT: ClassVar[ClientState]

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
    def __init__(self) -> None:
        ...

    @property
    def beam_altitude_angles(self) -> Any:
        ...

    @beam_altitude_angles.setter
    def beam_altitude_angles(self, val: Any) -> None:
        ...

    @property
    def beam_azimuth_angles(self) -> Any:
        ...

    @beam_azimuth_angles.setter
    def beam_azimuth_angles(self, val: Any) -> None:
        ...

    @property
    def format(self) -> DataFormat:
        ...

    @format.setter
    def format(self, val: DataFormat) -> None:
        ...

    @property
    def fw_rev(self) -> str:
        ...

    @fw_rev.setter
    def fw_rev(self, val: str) -> None:
        ...

    @property
    def hostname(self) -> str:
        ...

    @hostname.setter
    def hostname(self, val: str) -> None:
        ...

    @property
    def imu_to_sensor_transform(self) -> Any:
        ...

    @imu_to_sensor_transform.setter
    def imu_to_sensor_transform(self, val: Any) -> None:
        ...

    @property
    def lidar_to_sensor_transform(self) -> Any:
        ...

    @lidar_to_sensor_transform.setter
    def lidar_to_sensor_transform(self, val: Any) -> None:
        ...

    @property
    def mode(self) -> LidarMode:
        ...

    @mode.setter
    def mode(self, val: LidarMode) -> None:
        ...

    @property
    def prod_line(self) -> str:
        ...

    @prod_line.setter
    def prod_line(self, val: str) -> None:
        ...

    @property
    def sn(self) -> str:
        ...

    @sn.setter
    def sn(self, val: str) -> None:
        ...


def default_sensor_info(mode) -> SensorInfo:
    ...


def parse_metadata(s: str) -> SensorInfo:
    ...


class DataFormat:
    def __init__(self, *args, **kwargs) -> None:
        ...

    @property
    def columns_per_frame(self) -> int:
        ...

    @columns_per_frame.setter
    def columns_per_frame(self, val: int) -> None:
        ...

    @property
    def columns_per_packet(self) -> int:
        ...

    @columns_per_packet.setter
    def columns_per_packet(self, val: int) -> None:
        ...

    @property
    def pixel_shift_by_row(self) -> Any:
        ...

    @pixel_shift_by_row.setter
    def pixel_shift_by_row(self, val: Any) -> None:
        ...

    @property
    def pixels_per_column(self) -> int:
        ...

    @pixels_per_column.setter
    def pixels_per_column(self, val: int) -> None:
        ...


class PacketFormat:
    def __init__(self, *args, **kwargs) -> None:
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

    @property
    def columns_per_packet(self) -> int:
        ...

    @property
    def encoder_ticks_per_rev(self) -> int:
        ...

    @property
    def imu_packet_size(self) -> int:
        ...

    @property
    def lidar_packet_size(self) -> int:
        ...

    @property
    def pixels_per_column(self) -> int:
        ...

    def col_measurement_id(self, col: int, buf: bytes) -> int:
        ...

    def col_frame_id(self, col: int, buf: bytes) -> int:
        ...


def get_format(info: SensorInfo) -> PacketFormat:
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
    def __members__(self) -> dict:
        ...


def lidar_mode_of_string(s: str) -> LidarMode:
    ...


def n_cols_of_lidar_mode(mode: LidarMode) -> int:
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


def timestamp_mode_of_string(s: str) -> TimestampMode:
    ...


class Version:
    def __init__(self) -> None:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __le__(self, v: Version) -> bool:
        ...

    def __lt__(self, v: Version) -> bool:
        ...

    @property
    def major(self) -> int:
        ...

    @major.setter
    def major(self, val: int) -> None:
        ...

    @property
    def minor(self) -> int:
        ...

    @minor.setter
    def minor(self, val: int) -> None:
        ...

    @property
    def patch(self) -> int:
        ...

    @patch.setter
    def patch(self, val: int) -> None:
        ...


def version_of_string(s: str) -> Version:
    ...


class LidarScan:
    def __init__(self, w: int, h: int) -> None:
        ...

    @property
    def w(self) -> int:
        pass

    @property
    def h(self) -> int:
        pass

    @property
    def data(self) -> ndarray:
        pass

    @property
    def ts(self) -> List[int]:
        pass


def batch_to_scan(
        w: int, pf: PacketFormat,
        cb: Callable[[int], None]) -> Callable[[BufferT, LidarScan], None]:
    pass

from dataclasses import dataclass
from enum import Enum
from typing import Callable, ClassVar, List, Type, Union

import numpy.lib.stride_tricks
import numpy as np

from . import PacketFormat, SensorInfo, _sensor

BufferT = Union[bytes, bytearray, memoryview, np.ndarray]
"""Types that support the buffer protocol."""

Packet = Union['ImuPacket', 'LidarPacket']
"""Packets emitted by a sensor."""


class ImuPacket:
    """Read IMU Packet data from a bufer."""
    _pf: PacketFormat
    _data: np.ndarray

    def __init__(self, data: BufferT, pf: PacketFormat) -> None:
        self._pf = pf
        self._data = np.frombuffer(data,
                                   dtype=np.uint8,
                                   count=pf.imu_packet_size)

    @property
    def sys_ts(self) -> int:
        """System timestamp in nanoseconds."""
        return self._pf.imu_sys_ts(self._data)

    @property
    def accel_ts(self) -> int:
        """Accelerometer read time in nanoseconds."""
        return self._pf.imu_accel_ts(self._data)

    @property
    def gyro_ts(self) -> int:
        """Gyro read time in nanoseconds."""
        return self._pf.imu_gyro_ts(self._data)

    @property
    def accel(self) -> np.ndarray:
        """Acceleration as a 3-D vector in G."""
        return np.array([
            self._pf.imu_la_x(self._data),
            self._pf.imu_la_y(self._data),
            self._pf.imu_la_z(self._data)
        ])

    @property
    def angular_vel(self) -> np.ndarray:
        """Angular velocity as a 3-D vector in deg/second."""
        return np.array([
            self._pf.imu_av_x(self._data),
            self._pf.imu_av_y(self._data),
            self._pf.imu_av_z(self._data)
        ])


class ChanField(Enum):
    """Channel fields available in lidar data."""
    RANGE = (0, 0, np.uint32)
    REFLECTIVITY = (3, 4, np.uint16)
    INTENSITY = (1, 6, np.uint16)
    AMBIENT = (2, 8, np.uint16)

    def __init__(self, ind: int, offset: int, dtype: type):
        self.ind = ind
        self.offset = offset
        self.dtype = dtype


class ColHeader(Enum):
    """Column headers available in lidar data."""
    TIMESTAMP = (0, np.uint64)
    FRAME_ID = (10, np.uint16)
    MEASUREMENT_ID = (8, np.uint16)
    ENCODER_COUNT = (12, np.uint32)
    # negative offsets are considered relative to the end of the col buffer
    VALID = (-4, np.uint32)

    def __init__(self, offset: int, dtype: type):
        self.offset = offset
        self.dtype = dtype


class LidarPacket:
    """Read lidar packet data using numpy views."""

    _PIXEL_BYTES: ClassVar[int] = 12
    _COL_PREAMBLE_BYTES: ClassVar[int] = 16
    _COL_FOOTER_BYTES: ClassVar[int] = 4

    _pf: PacketFormat
    _data: np.ndarray
    _column_bytes: int

    def __init__(self, data: BufferT, pf: PacketFormat) -> None:
        """
        This will always alias the supplied buffer-like object. Pass in a copy
        to avoid unintentional aliasing.

        Args:
            data: buffer containing the packet payload
            pf: format determining how to interpret the buffer

        Raises:
            ValueError if the buffer is smaller than the size specified by the
            packet format.
        """
        self._pf = pf
        self._data = np.frombuffer(data,
                                   dtype=np.uint8,
                                   count=pf.lidar_packet_size)
        self._column_bytes = LidarPacket._COL_PREAMBLE_BYTES + \
            (LidarPacket._PIXEL_BYTES * self._pf.pixels_per_column) + \
            LidarPacket._COL_FOOTER_BYTES

    def view(self, field: Union[ChanField, ColHeader]) -> np.ndarray:
        """Create a zero-copy view of the specified data.

        Args:
            field: Specifies either a channel or column header

        Returns:
            view of the specified data as a numpy array
        """

        if isinstance(field, ChanField):
            return np.lib.stride_tricks.as_strided(
                self._data[LidarPacket._COL_PREAMBLE_BYTES +
                           field.offset:].view(dtype=field.dtype),
                shape=(self._pf.pixels_per_column,
                       self._pf.columns_per_packet),
                strides=(LidarPacket._PIXEL_BYTES, self._column_bytes))

        elif isinstance(field, ColHeader):
            start = 0 if field.offset >= 0 else self._column_bytes
            return np.lib.stride_tricks.as_strided(
                self._data[field.offset + start:].view(dtype=field.dtype),
                shape=(self._pf.columns_per_packet, ),
                strides=(self._column_bytes, ))

        else:
            raise TypeError("Expected either a Channel or ColHeader")


@dataclass
class BlockHeader:
    timestamp: int = 0
    encoder: int = 0
    status: int = 0


class LidarScan:
    """Represents a single "scan" or "frame" of lidar data.

    Internally, shares the same memory representation as the C++ LidarScan type
    and should allow passing data without unnecessary copying.
    """
    N_FIELDS: ClassVar[int] = 4

    w: int
    h: int
    frame_id: int
    headers: List[BlockHeader]
    _data: np.ndarray

    def __init__(self, w: int, h: int):
        """
        Args:
            w: horizontal resolution of the scan
            h: vertical resolution of the scan
        """
        self.w = w
        self.h = h
        self.frame_id = -1  # init with invalid frame_id
        self.headers = [BlockHeader()] * w
        self._data = np.ndarray((w * h, 4), dtype=np.uint32, order='F')

    @property
    def complete(self) -> bool:
        """Whether all columns of the scan are valid."""
        return all(h.status == 0xFFFFFFFF for h in self.headers)

    def field(self, field: ChanField) -> np.ndarray:
        """Return a view of the specified channel field."""
        return self._data[:, field.ind].reshape(self.h, self.w)

    def destaggered(self, info: SensorInfo, field: ChanField) -> np.ndarray:
        """Return a destaggered copy of the specified channel."""
        return _sensor.destagger(
            self._data[:, field.ind].reshape(self.h, self.w), info)

    def to_native(self) -> _sensor.LidarScan:
        ls = _sensor.LidarScan(self.w, self.h)
        ls.frame_id = self.frame_id
        ls.headers = [
            _sensor.BlockHeader(h.timestamp, h.encoder, h.status)
            for h in self.headers
        ]
        ls.data[:] = self._data
        return ls

    @classmethod
    def from_native(cls: Type['LidarScan'],
                    scan: _sensor.LidarScan) -> 'LidarScan':
        ls = cls.__new__(cls)
        ls.w = scan.w
        ls.h = scan.h
        ls.frame_id = scan.frame_id
        ls.headers = [
            BlockHeader(h.timestamp, h.encoder, h.status) for h in scan.headers
        ]
        ls._data = scan.data.reshape(ls.w * ls.h, 4)
        return ls


def XYZLut(info: SensorInfo) -> Callable[[LidarScan], np.ndarray]:
    """Return a function that can project scans into cartesian coordinates.

    Internally, this will pre-compute a lookup table using the supplied
    intrinsic parameters. XYZ points are returned as an N x 3 array of doubles,
    where N is the number of points in a scan (e.g. 131072 for a 128-beam sensor
    in 1024x10 mode).

    Args:
        info: sensor metadata
    """
    lut = _sensor.XYZLut(info)

    def res(ls: LidarScan) -> np.ndarray:
        return lut(ls.to_native())

    return res
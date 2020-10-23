from enum import Enum
from typing import ClassVar, Union

import numpy as np

from . import _sensor

BufferT = Union[bytes, bytearray, memoryview]


class ImuPacket:
    def __init__(self, data: BufferT, pf: _sensor.PacketFormat) -> None:
        self._pf = pf
        self._data = data

    @property
    def sys_ts(self) -> int:
        return self._pf.imu_sys_ts(self._data)

    @property
    def accel_ts(self) -> int:
        return self._pf.imu_accel_ts(self._data)

    @property
    def gyro_ts(self) -> int:
        return self._pf.imu_gyro_ts(self._data)

    @property
    def accel(self) -> np.ndarray:
        return np.array([
            self._pf.imu_la_x(self._data),
            self._pf.imu_la_y(self._data),
            self._pf.imu_la_z(self._data)
        ])

    @property
    def angular_vel(self) -> np.ndarray:
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
    """Column header datatypes and offsets.

    Negative offsets are considered relative to the end of the col buffer.
    """
    TIMESTAMP = (0, np.uint64)
    FRAME_ID = (10, np.uint16)
    MEASUREMENT_ID = (8, np.uint16)
    ENCODER_COUNT = (12, np.uint32)
    VALID = (-4, np.uint32)

    def __init__(self, offset: int, dtype: type):
        self.offset = offset
        self.dtype = dtype


class LidarPacket:
    """Read packet data using numpy views.

    Todo:
        This requires some information about packet data layout not provided by
        packet_format. Bytes per pixel will be variable in future fw.
    """

    PIXEL_BYTES: int = 12
    COL_PREAMBLE_BYTES: int = 16
    COL_FOOTER_BYTES: int = 4

    def __init__(self, data: BufferT, pf: _sensor.PacketFormat) -> None:
        """Create a new packet view of a buffer.

        This will always alias the supplied buffer-like object. Pass in a copy
        to avoid unintentional aliasing.
        """

        self._pf = pf
        self._data = np.frombuffer(data, dtype=np.uint8)
        self._column_bytes = LidarPacket.COL_PREAMBLE_BYTES + \
            (LidarPacket.PIXEL_BYTES * self._pf.pixels_per_column) + \
            LidarPacket.COL_FOOTER_BYTES

    def _view(self, offset: int, data_type: type) -> np.ndarray:
        """Internal method for creating a view from a numpy array.

        Args:
            offset (int): Byte offset of the view from the beginning of data
            data_type (type): The numpy data type to use for elements

        Returns:
            np.ndarray: view of the buffer starting at the given offset
        """
        min = offset
        max = -((self._data.size - offset) % np.dtype(data_type).itemsize)
        return self._data[min:max].view(dtype=data_type)

    def view(self, field: Union[ChanField, ColHeader]) -> np.ndarray:
        """Create a zero-copy view of the specified data.

        Args:
            field: Specifies either a channel or column header

        Returns:
            view of the specified data as a numpy array
        """

        if isinstance(field, ChanField):
            return np.lib.stride_tricks.as_strided(
                self._view(LidarPacket.COL_PREAMBLE_BYTES + field.offset,
                           field.dtype),
                shape=(self._pf.pixels_per_column,
                       self._pf.columns_per_packet),
                strides=(LidarPacket.PIXEL_BYTES, self._column_bytes))

        elif isinstance(field, ColHeader):
            start = 0 if field.offset >= 0 else self._column_bytes
            return np.lib.stride_tricks.as_strided(
                self._view(field.offset + start, field.dtype),
                shape=(self._pf.columns_per_packet, ),
                strides=(self._column_bytes, ))

        else:
            raise TypeError("Expected either a Channel or ColHeader")


class LidarScan:
    """Represents a single "scan" or "frame" of lidar data.

    Internally, shares the same memory representation as the C++ LidarScan type
    and should allow passing data without unnecessary copying.
    """

    N_FIELDS: ClassVar[int] = 4

    def __init__(self, w: int, h: int):
        """Initialize an empty lidarscan.

        LidarScan is currently represented with a w*h X 4 column-major array
        of uint32.
        """
        self.w = w
        self.h = h
        self._data = np.ndarray((w * h, 4), dtype=np.uint32, order='F')

    def field(self, field: ChanField):
        """Return a view of the specified channel field."""
        return self._data[:, field.ind].reshape(self.h, self.w)

    def destaggered(self, info: _sensor.SensorInfo, field: ChanField):
        """Return a destaggered copy of the specified channel."""
        return _sensor.destagger(self._data[:, field.ind],
                                 info).reshape(self.h, self.w)

    @classmethod
    def from_buffer(cls, w: int, h: int, buf: np.ndarray):
        """Alternative constructor from an existing buffer."""
        ls = cls.__new__(cls)
        ls.w = w
        ls.h = h
        ls._data = buf.astype(np.uint32).reshape((w * h, LidarScan.N_FIELDS),
                                                 order='F')
        return ls

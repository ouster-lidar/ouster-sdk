from enum import Enum
from typing import Callable, ClassVar, Dict, List, Optional, Type, Union, Tuple

import numpy.lib.stride_tricks
import numpy as np

from . import SensorInfo, _client

BufferT = Union[bytes, bytearray, memoryview, np.ndarray]
"""Types that support the buffer protocol."""

Packet = Union['ImuPacket', 'LidarPacket']
"""Packets emitted by a sensor."""


class ImuPacket:
    """Read IMU Packet data from a bufer."""
    _pf: _client.PacketFormat
    _data: np.ndarray
    capture_timestamp: Optional[float]

    def __init__(self, data: BufferT, info: SensorInfo, timestamp: Optional[float] = None) -> None:
        """
        Args:
            data: Buffer containing the packet payload
            pf: Format determining how to interpret the buffer

        Raises:
            ValueError: If the buffer is smaller than the size specified by the
                packet format.
        """

        self._pf = _client.PacketFormat.from_info(info)
        self._data = np.frombuffer(data,
                                   dtype=np.uint8,
                                   count=self._pf.imu_packet_size)

        self.capture_timestamp = timestamp

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
    RANGE = (0, 0, np.uint32, 0x000FFFFF)
    REFLECTIVITY = (3, 4, np.uint16, None)
    SIGNAL = (1, 6, np.uint16, None)
    NEAR_IR = (2, 8, np.uint16, None)

    def __init__(self, ind: int, offset: int, dtype: type,
                 mask: Optional[int]):
        self.ind = ind
        self.offset = offset
        self.dtype = dtype
        self.mask = mask


class ColHeader(Enum):
    """Column headers available in lidar data."""
    TIMESTAMP = (0, np.uint64)
    FRAME_ID = (10, np.uint16)
    MEASUREMENT_ID = (8, np.uint16)
    ENCODER_COUNT = (12, np.uint32)
    # negative offsets are considered relative to the end of the col buffer
    STATUS = (-4, np.uint32)

    def __init__(self, offset: int, dtype: type):
        self.offset = offset
        self.dtype = dtype


class LidarPacket:
    """Read lidar packet data using numpy views."""

    _PIXEL_BYTES: ClassVar[int] = 12
    _COL_PREAMBLE_BYTES: ClassVar[int] = 16
    _COL_FOOTER_BYTES: ClassVar[int] = 4

    _pf: _client.PacketFormat
    _data: np.ndarray
    _column_bytes: int
    capture_timestamp: Optional[float]

    def __init__(self, data: BufferT, info: SensorInfo, timestamp: Optional[float] = None) -> None:
        """
        This will always alias the supplied buffer-like object. Pass in a copy
        to avoid unintentional aliasing.

        Args:
            data: Buffer containing the packet payload
            pf: Format determining how to interpret the buffer

        Raises:
            ValueError: If the buffer is smaller than the size specified by the
                packet format.
        """
        self._pf = _client.PacketFormat.from_info(info)
        self._data = np.frombuffer(data,
                                   dtype=np.uint8,
                                   count=self._pf.lidar_packet_size)
        self._column_bytes = LidarPacket._COL_PREAMBLE_BYTES + \
            (LidarPacket._PIXEL_BYTES * self._pf.pixels_per_column) + \
            LidarPacket._COL_FOOTER_BYTES
        self.capture_timestamp = timestamp

    def field(self, field: ChanField) -> np.ndarray:
        """Create a view of the specified channel field.

        Args:
            field: The channel field to view

        Returns:
            A view of the specified field as a numpy array
        """
        v = np.lib.stride_tricks.as_strided(
            self._data[LidarPacket._COL_PREAMBLE_BYTES +
                       field.offset:].view(dtype=field.dtype),
            shape=(self._pf.pixels_per_column, self._pf.columns_per_packet),
            strides=(LidarPacket._PIXEL_BYTES, self._column_bytes))
        return v if field.mask is None else v & field.mask

    def header(self, header: ColHeader) -> np.ndarray:
        """Create a view of the specified column header.

        Args:
            header: The column header to view

        Returns:
            A view of the specified header as a numpy array
        """

        start = 0 if header.offset >= 0 else self._column_bytes
        return np.lib.stride_tricks.as_strided(
            self._data[header.offset + start:].view(dtype=header.dtype),
            shape=(self._pf.columns_per_packet, ),
            strides=(self._column_bytes, ))


class LidarScan:
    """Represents a single "scan" or "frame" of lidar data.

    Internally, shares the same memory representation as the C++ LidarScan type
    and should allow passing data without unnecessary copying.
    """
    N_FIELDS: ClassVar[int] = _client.LidarScan.N_FIELDS

    w: int
    h: int
    frame_id: int
    _data: np.ndarray
    _headers: Dict[ColHeader, np.ndarray]

    def __init__(self, h: int, w: int):
        """
        Args:
            h: Vertical resolution of the scan
            w: Horizontal resolution of the scan
        """
        self.w = w
        self.h = h
        self.frame_id = -1  # init with invalid frame_id
        self._data = np.ndarray((LidarScan.N_FIELDS, w * h), dtype=np.uint32)
        self._headers = {
            h: np.zeros(w, h.dtype)
            for h in (ColHeader.TIMESTAMP, ColHeader.ENCODER_COUNT,
                      ColHeader.STATUS)
        }

    def _complete(self,
                  column_window: Optional[Tuple[int, int]] = None) -> bool:
        """Whether all columns of the scan are valid within given window.

        Args:
            column_window: metadata.format.column_window if it's not default
                to full scan
        """
        if column_window is None:
            column_window = (0, self.w - 1)

        win_start, win_end = column_window
        status = self.header(ColHeader.STATUS)

        if win_start <= win_end:
            return (status[win_start:win_end + 1] == 0xFFFFFFFF).all()
        else:
            return ((status[:win_end + 1] == 0xFFFFFFFF).all()
                    and (status[win_start:] == 0xFFFFFFFF).all())

    def field(self, field: ChanField) -> np.ndarray:
        """Return a view of the specified channel field."""
        return self._data[field.ind, :].reshape(self.h, self.w)

    def header(self, header: ColHeader) -> np.ndarray:
        """Return the specified column header as a numpy array.

        Note that only TIMESTAMP, ENCODER_COUNT, and STATUS are currently
        supported.
        """
        return self._headers[header]

    def to_native(self) -> _client.LidarScan:
        ls = _client.LidarScan(self.w, self.h)
        ls.frame_id = self.frame_id
        ls.headers = [
            _client.BlockHeader(
                self.header(ColHeader.TIMESTAMP)[i],
                self.header(ColHeader.ENCODER_COUNT)[i],
                self.header(ColHeader.STATUS)[i]) for i in range(self.w)
        ]
        ls.data[:] = self._data
        return ls

    @classmethod
    def from_native(cls: Type['LidarScan'],
                    scan: _client.LidarScan) -> 'LidarScan':
        ls = cls.__new__(cls)
        ls.w = scan.w
        ls.h = scan.h
        ls.frame_id = scan.frame_id
        ls._data = scan.data
        ls._headers = {
            ColHeader.TIMESTAMP:
            np.array([h.timestamp for h in scan.headers],
                     dtype=ColHeader.TIMESTAMP.dtype),
            ColHeader.ENCODER_COUNT:
            np.array([h.encoder for h in scan.headers],
                     dtype=ColHeader.ENCODER_COUNT.dtype),
            ColHeader.STATUS:
            np.array([h.status for h in scan.headers],
                     dtype=ColHeader.STATUS.dtype),
        }

        return ls


def _destagger(field: np.ndarray, shifts: List[int],
               inverse: bool) -> np.ndarray:
    return {
        np.dtype(np.int8): _client.destagger_int8,
        np.dtype(np.int16): _client.destagger_int16,
        np.dtype(np.int32): _client.destagger_int32,
        np.dtype(np.int64): _client.destagger_int64,
        np.dtype(np.uint8): _client.destagger_uint8,
        np.dtype(np.uint16): _client.destagger_uint16,
        np.dtype(np.uint32): _client.destagger_uint32,
        np.dtype(np.uint64): _client.destagger_uint64,
        np.dtype(np.single): _client.destagger_float,
        np.dtype(np.double): _client.destagger_double,
    }[field.dtype](field, shifts, inverse)


def destagger(info: SensorInfo,
              fields: np.ndarray,
              inverse=False) -> np.ndarray:
    """Return a destaggered copy of the provided fields.

    In the default staggered representation, each column corresponds to a
    single timestamp. A destaggered representation compensates for the
    azimuth offset of each beam, returning columns that correspond to a
    single azimuth angle.

    Args:
        info: Sensor metadata associated with the provided data
        fields: A numpy array of shape H X W or H X W X N
        inverse: perform inverse "staggering" operation

    Returns:
        A destaggered numpy array of the same shape
    """
    h = info.format.pixels_per_column
    w = info.format.columns_per_frame
    shifts = info.format.pixel_shift_by_row

    # remember original shape
    shape = fields.shape
    fields = fields.reshape((h, w, -1))

    # apply destagger to each channel
    # note: astype() needed due to some strange behavior of the pybind11
    # bindings. The wrong overload is chosen otherwise (due to the indexing?)
    return np.dstack([
        _destagger(fields[:, :, i], shifts, inverse)
        for i in range(fields.shape[2])
    ]).reshape(shape)


def XYZLut(info: SensorInfo) -> Callable[[LidarScan], np.ndarray]:
    """Return a function that can project scans into cartesian coordinates.

    Internally, this will pre-compute a lookup table using the supplied
    intrinsic parameters. XYZ points are returned as a H x W x 3 array of
    doubles, where H is the number of beams and W is the horizontal resolution
    of the scan.

    The coordinates are reported in meters in the *sensor frame* as
    defined in the sensor documentation.

    Args:
        info: sensor metadata

    Returns:
        A function that computes a numpy array of a scan's point coordinates
    """
    lut = _client.XYZLut(info)

    def res(ls: LidarScan) -> np.ndarray:
        return lut(ls.to_native()).reshape(info.format.pixels_per_column,
                                           info.format.columns_per_frame, 3)

    return res

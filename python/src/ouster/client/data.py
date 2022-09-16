"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from copy import deepcopy
from enum import Enum
from typing import Callable, Iterator, Type, List, Optional, Union
import warnings

import numpy as np

from . import _client
from ._client import (ChanField, LidarScan, SensorInfo)

BufferT = Union[bytes, bytearray, memoryview, np.ndarray]
"""Types that support the buffer protocol."""

FieldDType = Type[np.unsignedinteger]
"""Numpy dtype of fields."""

Packet = Union['ImuPacket', 'LidarPacket']
"""Packets emitted by a sensor."""


class ImuPacket:
    """Read IMU Packet data from a bufer."""
    _pf: _client.PacketFormat
    _data: np.ndarray
    capture_timestamp: Optional[float]

    def __init__(self,
                 data: BufferT,
                 info: SensorInfo,
                 timestamp: Optional[float] = None) -> None:
        """
        This will always alias the supplied buffer-like object. Pass in a copy
        to avoid unintentional aliasing.

        Args:
            data: Buffer containing the packet payload
            info: Metadata associated with the sensor packet stream
            timestamp: A capture timestamp, in seconds

        Raises:
            ValueError: If the buffer is smaller than the size specified by the
                packet format
        """

        self._pf = _client.PacketFormat.from_info(info)
        self._data = np.frombuffer(data,
                                   dtype=np.uint8,
                                   count=self._pf.imu_packet_size)

        self.capture_timestamp = timestamp

    def __deepcopy__(self, memo) -> 'ImuPacket':
        cls = type(self)
        cpy = cls.__new__(cls)
        # don't copy packet format, which is intended to be shared
        cpy._pf = self._pf
        cpy._data = deepcopy(self._data, memo)
        cpy.capture_timestamp = self.capture_timestamp
        return cpy

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


class ColHeader(Enum):
    """Column headers available in lidar data.

    This definition is deprecated.
    """
    TIMESTAMP = 0
    ENCODER_COUNT = 1
    MEASUREMENT_ID = 2
    STATUS = 3
    FRAME_ID = 4

    def __int__(self) -> int:
        return self.value


class LidarPacket:
    """Read lidar packet data as numpy arrays.

    The dimensions of returned arrays depend on the sensor product line and
    configuration. Measurement headers will be arrays of size matching the
    configured ``columns_per_packet``, while measurement fields will be 2d
    arrays of size ``pixels_per_column`` by ``columns_per_packet``.
    """
    _pf: _client.PacketFormat
    _data: np.ndarray
    capture_timestamp: Optional[float]

    def __init__(self,
                 data: BufferT,
                 info: SensorInfo,
                 timestamp: Optional[float] = None) -> None:
        """
        This will always alias the supplied buffer-like object. Pass in a copy
        to avoid unintentional aliasing.

        Args:
            data: Buffer containing the packet payload
            info: Metadata associated with the sensor packet stream
            timestamp: A capture timestamp, in seconds

        Raises:
            ValueError: If the buffer is smaller than the size specified by the
                packet format, or if the init_id doesn't match the metadata
        """
        self._pf = _client.PacketFormat.from_info(info)
        self._data = np.frombuffer(data,
                                   dtype=np.uint8,
                                   count=self._pf.lidar_packet_size)
        self.capture_timestamp = timestamp

        # check that metadata came from the same sensor initialization as data
        if self.init_id and self.init_id != info.init_id:
            raise ValueError("Metadata init id does not match")

    def __deepcopy__(self, memo) -> 'LidarPacket':
        cls = type(self)
        cpy = cls.__new__(cls)
        # don't copy packet format, which is intended to be shared
        cpy._pf = self._pf
        cpy._data = deepcopy(self._data, memo)
        cpy.capture_timestamp = self.capture_timestamp
        return cpy

    @property
    def packet_type(self) -> int:
        """Get the type header of the packet."""
        return self._pf.packet_type(self._data)

    @property
    def frame_id(self) -> int:
        """Get the frame id of the packet."""
        return self._pf.frame_id(self._data)

    @property
    def init_id(self) -> int:
        """Get the initialization id of the packet."""
        return self._pf.init_id(self._data)

    @property
    def prod_sn(self) -> int:
        """Get the serial no header of the packet."""
        return self._pf.prod_sn(self._data)

    @property
    def fields(self) -> Iterator[ChanField]:
        """Get available fields of LidarScan as Iterator."""
        return self._pf.fields

    def field(self, field: ChanField) -> np.ndarray:
        """Create a view of the specified channel field.

        Args:
            field: The channel field to view

        Returns:
            A numpy array containing a copy of the specified field values
        """
        res = self._pf.packet_field(field, self._data)
        res.flags.writeable = False
        return res

    def header(self, header: ColHeader) -> np.ndarray:
        """Create a view of the specified column header.

        This method is deprecated. Use the ``timestamp``, ``measurement_id`` or
        ``status`` properties instead.

        Args:
            header: The column header to parse

        Returns:
            A numpy array containing a copy of the specified header values
        """
        warnings.warn("LidarPacket.header is deprecated", DeprecationWarning)

        res = self._pf.packet_header(header, self._data)
        res.flags.writeable = False
        return res

    @property
    def timestamp(self) -> np.ndarray:
        """Parse the measurement block timestamps out of a packet buffer.

        Returns:
            An array of the timestamps of all measurement blocks in the packet.
        """
        res = self._pf.packet_header(ColHeader.TIMESTAMP, self._data)
        res.flags.writeable = False
        return res

    @property
    def measurement_id(self) -> np.ndarray:
        """Parse the measurement ids out of a packet buffer.

        Returns:
            An array of the ids of all measurement blocks in the packet.
        """
        res = self._pf.packet_header(ColHeader.MEASUREMENT_ID, self._data)
        res.flags.writeable = False
        return res

    @property
    def status(self) -> np.ndarray:
        """Parse the measurement statuses of a packet buffer.

        Returns:
            An array of the statuses of all measurement blocks in the packet.
        """
        res = self._pf.packet_header(ColHeader.STATUS, self._data)
        res.flags.writeable = False
        return res


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


def XYZLut(
        info: SensorInfo
) -> Callable[[Union[LidarScan, np.ndarray]], np.ndarray]:
    """Return a function that can project scans into Cartesian coordinates.

    If called with a numpy array representing a range image, the range image
    must be in "staggered" form, where each column corresponds to a single
    measurement block. LidarScan fields are always staggered.

    Internally, this will pre-compute a lookup table using the supplied
    intrinsic parameters. XYZ points are returned as a H x W x 3 array of
    doubles, where H is the number of beams and W is the horizontal resolution
    of the scan.

    The coordinates are reported in meters in the *sensor frame* as
    defined in the sensor documentation.

    Args:
        info: sensor metadata

    Returns:
        A function that computes a point cloud given a range image
    """
    lut = _client.XYZLut(info)

    def res(ls: Union[LidarScan, np.ndarray]) -> np.ndarray:
        if isinstance(ls, LidarScan):
            xyz = lut(ls)
        else:
            # will create a temporary to cast if dtype != uint32
            xyz = lut(ls.astype(np.uint32, copy=False))

        return xyz.reshape(info.format.pixels_per_column,
                           info.format.columns_per_frame, 3)

    return res

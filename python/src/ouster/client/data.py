"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from enum import Enum
from typing import Callable, Iterator, Type, List, Optional, Union, Dict
import logging
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

FieldTypes = Dict[ChanField, FieldDType]
"""LidarScan chan fields with types"""

logger = logging.getLogger("ouster.client.data")


class ImuPacket(_client._ImuPacket):
    """Read IMU Packet data from a buffer."""
    _pf: _client.PacketFormat

    def __init__(self,
                 data: Optional[BufferT] = None,
                 info: Optional[SensorInfo] = None,
                 timestamp: Optional[float] = None,
                 *,
                 packet_format: Optional[_client.PacketFormat] = None) -> None:
        """
        Args:
            data: Buffer containing the packet payload
            info: Metadata associated with the sensor packet stream
            timestamp: A capture timestamp, in seconds

        Raises:
            ValueError: If the buffer is smaller than the size specified by the
                packet format
        """
        if packet_format:
            self._pf = packet_format
        elif info:
            # TODO: we should deprecate this, constructing a full PacketFormat
            # for every single packet seems like an antipattern -- Tim T.
            self._pf = _client.PacketFormat.from_info(info)
        else:
            raise ValueError("either packet_format or info should be specified")

        n = self._pf.imu_packet_size
        super().__init__(size=n)
        if data is not None:
            self._data[:] = np.frombuffer(data, dtype=np.uint8, count=n)
        self.capture_timestamp = timestamp

    def __deepcopy__(self, memo) -> 'ImuPacket':
        cls = type(self)
        cpy = cls(self._data, packet_format=self._pf)
        cpy._host_timestamp = self._host_timestamp
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


class PacketValidationFailure(Exception):
    def __eq__(self, other):
        return type(self) is type(other) and self.args == other.args

    def __hash__(self):
        return hash((type(self), self.args))


class PacketIdError(PacketValidationFailure):
    """Exception raised when init_id/sn from metadata and packet doesn't match."""
    pass


class PacketSizeError(PacketValidationFailure):
    """Exception raised when the packet size wrong for the given metadata."""
    pass


class LidarPacketValidator:
    """A utility class for validating lidar packets for a given sensor info."""
    def __init__(self, metadata: SensorInfo, checks=['id_and_sn_valid', 'packet_size_valid']):
        self._metadata = metadata
        self._metadata_init_id = metadata.init_id
        self._metadata_sn = int(metadata.sn) if metadata.sn else 0
        self._pf = _client.PacketFormat.from_info(metadata)
        self._checks = [getattr(self, check) for check in checks]

    def check_packet(self, data: BufferT, n_bytes: int) -> List[PacketValidationFailure]:
        errors = []
        for check in self._checks:
            error = check(data, n_bytes)
            if error:
                errors.append(error)
        return errors

    def id_and_sn_valid(self, data: BufferT, n_bytes: int) -> Optional[PacketValidationFailure]:
        """Check the metadata init_id/sn and packet init_id/sn mismatch."""
        init_id = self._pf.init_id(data)
        sn = self._pf.prod_sn(data)
        if bool(init_id and (init_id != self._metadata_init_id or sn != self._metadata_sn)):
            error_msg = f"Metadata init_id/sn does not match: " \
                    f"expected by metadata - {self._metadata_init_id}/{self._metadata_sn}, " \
                    f"but got from packet buffer - {init_id}/{sn}"
            return PacketIdError(error_msg)
        return None

    def packet_size_valid(self, data: BufferT, n_bytes: int) -> Optional[PacketValidationFailure]:
        if self._pf.lidar_packet_size != n_bytes:
            return PacketSizeError(
                f"Expected a packet of size {self._pf.lidar_packet_size} but got a buffer of size {n_bytes}")
        return None


class LidarPacket(_client._LidarPacket):
    """Read lidar packet data as numpy arrays.

    The dimensions of returned arrays depend on the sensor product line and
    configuration. Measurement headers will be arrays of size matching the
    configured ``columns_per_packet``, while measurement fields will be 2d
    arrays of size ``pixels_per_column`` by ``columns_per_packet``.
    """
    _pf: _client.PacketFormat
    _metadata_init_id: int
    _metadata_sn: int

    def __init__(self,
                 data: Optional[BufferT] = None,
                 info: Optional[SensorInfo] = None,
                 timestamp: Optional[float] = None,
                 *,
                 packet_format: Optional[_client.PacketFormat] = None,
                 _raise_on_id_check: bool = True) -> None:
        """
        Args:
            data: Buffer containing the packet payload
            info: Metadata associated with the sensor packet stream
            timestamp: A capture timestamp, in seconds
            _raise_on_id_check: raise PacketIdError if metadata
                init_id/sn doesn't match packet init_id/sn.

        Raises:
            ValueError: If the buffer is smaller than the size specified by the
                packet format, or if the init_id doesn't match the metadata
        """
        if packet_format:
            self._pf = packet_format
        elif info:
            # TODO: we should deprecate this, constructing a full PacketFormat
            # for every single packet seems like an antipattern -- Tim T.
            self._pf = _client.PacketFormat.from_info(info)
        else:
            raise ValueError("either packet_format or info should be specified")

        n = self._pf.lidar_packet_size
        super().__init__(size=n)
        if data is not None:
            self._data[:] = np.frombuffer(data, dtype=np.uint8, count=n)
        self.capture_timestamp = timestamp

        if info:
            self._metadata_init_id = info.init_id
            self._metadata_sn = int(info.sn) if info.sn else 0

        # check that metadata came from the same sensor initialization as data
        if info and self.id_error:
            error_msg = f"Metadata init_id/sn does not match: " \
                f"expected by metadata - {info.init_id}/{info.sn}, " \
                f"but got from packet buffer - {self.init_id}/{self.prod_sn}"
            if _raise_on_id_check:
                raise PacketIdError(error_msg)
            else:
                # Continue with warning. When init_ids/sn doesn't match
                # the resulting LidarPacket has high chances to be
                # incompatible with data format set in metadata json file
                logger.warn(f"LidarPacket validation: {error_msg}")

    def __deepcopy__(self, memo) -> 'LidarPacket':
        cls = type(self)
        cpy = cls(self._data, packet_format=self._pf)
        cpy._host_timestamp = self._host_timestamp
        return cpy

    @property
    def id_error(self) -> bool:
        """Check the metadata init_id/sn and packet init_id/sn mismatch."""
        return bool(self.init_id and (self.init_id != self._metadata_init_id or
                                      self.prod_sn != self._metadata_sn))

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
    def countdown_thermal_shutdown(self) -> int:
        """Get the thermal shutdown countdown of the packet."""
        return self._pf.countdown_thermal_shutdown(self._data)

    @property
    def countdown_shot_limiting(self) -> int:
        """Get the shot limiting countdown of the packet."""
        return self._pf.countdown_shot_limiting(self._data)

    @property
    def thermal_shutdown(self) -> int:
        """Get the thermal shutdown status of the packet."""
        return self._pf.thermal_shutdown(self._data)

    @property
    def shot_limiting(self) -> int:
        """Get the shot limiting status of the packet."""
        return self._pf.shot_limiting(self._data)

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
        info: SensorInfo,
        use_extrinsics: bool = False
) -> Callable[[Union[LidarScan, np.ndarray]], np.ndarray]:
    """Return a function that can project scans into Cartesian coordinates.

    If called with a numpy array representing a range image, the range image
    must be in "staggered" form, where each column corresponds to a single
    measurement block. LidarScan fields are always staggered.

    Internally, this will pre-compute a lookup table using the supplied
    intrinsic parameters. XYZ points are returned as a H x W x 3 array of
    doubles, where H is the number of beams and W is the horizontal resolution
    of the scan.

    The coordinates are reported in meters in the *sensor frame* (when
    ``use_extrinsics`` is False, default) as defined in the sensor documentation.

    However, the result is returned in the "extrinsics frame" if
    ``use_extrinsics`` is True, which makes additional transform from
    "sensor frame" to "extrinsics frame" using the homogeneous 4x4 transform
    matrix from ``info.extrinsic`` property.

    Args:
        info: sensor metadata
        use_extrinsics: if True, applies the ``info.extrinsic`` transform to the
                        resulting "sensor frame" coordinates and returns the
                        result in "extrinsics frame".

    Returns:
        A function that computes a point cloud given a range image
    """
    lut = _client.XYZLut(info, use_extrinsics)

    def res(ls: Union[LidarScan, np.ndarray]) -> np.ndarray:
        if isinstance(ls, LidarScan):
            xyz = lut(ls)
        else:
            # will create a temporary to cast if dtype != uint32
            xyz = lut(ls.astype(np.uint32, copy=False))

        return xyz.reshape(info.format.pixels_per_column,
                           info.format.columns_per_frame, 3)

    return res


def packet_ts(packet: Packet) -> int:
    """Return the packet timestamp in nanoseconds"""
    return int(packet.capture_timestamp *
               10**9) if packet.capture_timestamp else 0

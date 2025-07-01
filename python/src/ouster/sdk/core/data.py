"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from enum import Enum
from typing import Callable, List, Union
import logging

import numpy as np

from ouster.sdk._bindings.client import (LidarScan, SensorInfo, Packet)

from ouster.sdk._bindings.client import (destagger_int8, destagger_int16, destagger_int32,
                      destagger_int64, destagger_uint8, destagger_uint16,
                      destagger_uint32, destagger_uint64, destagger_float,
                      destagger_double, destagger_bool)

from ouster.sdk._bindings.client import XYZLut as client_XYZLut
from ouster.sdk._bindings.client import XYZLutFloat as client_XYZLutFloat

BufferT = Union[bytes, bytearray, memoryview, np.ndarray]
"""Types that support the buffer protocol."""

logger = logging.getLogger("ouster.sdk.core.data")


class ChanField:
    RANGE = "RANGE"
    RANGE2 = "RANGE2"
    SIGNAL = "SIGNAL"
    SIGNAL2 = "SIGNAL2"
    REFLECTIVITY = "REFLECTIVITY"
    REFLECTIVITY2 = "REFLECTIVITY2"
    NEAR_IR = "NEAR_IR"
    FLAGS = "FLAGS"
    FLAGS2 = "FLAGS2"
    RAW_HEADERS = "RAW_HEADERS"
    RAW32_WORD1 = "RAW32_WORD1"
    RAW32_WORD2 = "RAW32_WORD2"
    RAW32_WORD3 = "RAW32_WORD3"
    RAW32_WORD4 = "RAW32_WORD4"
    RAW32_WORD5 = "RAW32_WORD5"
    RAW32_WORD6 = "RAW32_WORD6"
    RAW32_WORD7 = "RAW32_WORD7"
    RAW32_WORD8 = "RAW32_WORD8"
    RAW32_WORD9 = "RAW32_WORD9"


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


def _destagger(field: np.ndarray, shifts: List[int],
               inverse: bool) -> np.ndarray:
    return {
        np.dtype(bool): destagger_bool,
        np.dtype(np.int8): destagger_int8,
        np.dtype(np.int16): destagger_int16,
        np.dtype(np.int32): destagger_int32,
        np.dtype(np.int64): destagger_int64,
        np.dtype(np.uint8): destagger_uint8,
        np.dtype(np.uint16): destagger_uint16,
        np.dtype(np.uint32): destagger_uint32,
        np.dtype(np.uint64): destagger_uint64,
        np.dtype(np.single): destagger_float,
        np.dtype(np.double): destagger_double,
    }[field.dtype](field, shifts, inverse)


def stagger(info: SensorInfo,
            fields: np.ndarray) -> np.ndarray:
    """Return a staggered copy of the provided fields.

    In the default staggered representation, each column corresponds to a
    single timestamp. A destaggered representation compensates for the
    azimuth offset of each beam, returning columns that correspond to a
    single azimuth angle.

    Args:
        info: Sensor metadata associated with the provided data
        fields: A numpy array of shape H X W or H X W X N

    Returns:
        A staggered numpy array of the same shape
    """
    return destagger(info, fields, inverse=True)


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
    lut = client_XYZLut(info, use_extrinsics)

    def res(ls: Union[LidarScan, np.ndarray]) -> np.ndarray:
        if isinstance(ls, LidarScan):
            xyz = lut(ls)
        else:
            # will create a temporary to cast if dtype != uint32
            xyz = lut(ls.astype(np.uint32, copy=False))

        return xyz.reshape(info.format.pixels_per_column,
                           info.format.columns_per_frame, 3)

    return res


def XYZLutFloat(
        info: SensorInfo,
        use_extrinsics: bool = False
) -> Callable[[Union[LidarScan, np.ndarray]], np.ndarray]:
    lut = client_XYZLutFloat(info, use_extrinsics)

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
    return packet.host_timestamp

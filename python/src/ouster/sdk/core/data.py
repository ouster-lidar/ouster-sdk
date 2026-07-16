"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from enum import Enum
from typing import Union
import logging

import numpy as np

from ouster.sdk._bindings.client import (SensorInfo, Packet)

from ouster.sdk._bindings.client import destagger

BufferT = Union[bytes, bytearray, memoryview, np.ndarray]
"""Types that support the buffer protocol."""

logger = logging.getLogger("ouster.sdk.core.data")


class ChanField:
    RANGE = "RANGE"
    RANGE2 = "RANGE2"
    SIGNAL = "SIGNAL"
    SIGNAL2 = "SIGNAL2"
    R = "R"
    G = "G"
    B = "B"
    RGB = "RGB"
    REFLECTIVITY = "REFLECTIVITY"
    REFLECTIVITY2 = "REFLECTIVITY2"
    NEAR_IR = "NEAR_IR"
    FLAGS = "FLAGS"
    FLAGS2 = "FLAGS2"
    WINDOW = "WINDOW"
    ZONE = "ZONE"
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
    NORMALS = "NORMALS"
    NORMALS2 = "NORMALS2"
    GROUND = "GROUND"
    GROUND2 = "GROUND2"
    IMU_ACC = "IMU_ACC"
    IMU_GYRO = "IMU_GYRO"
    IMU_TIMESTAMP = "IMU_TIMESTAMP"
    IMU_MEASUREMENT_ID = "IMU_MEASUREMENT_ID"
    IMU_STATUS = "IMU_STATUS"
    IMU_PACKET_TIMESTAMP = "IMU_PACKET_TIMESTAMP"
    IMU_ALERT_FLAGS = "IMU_ALERT_FLAGS"
    POSITION_STRING = "POSITION_STRING"
    POSITION_LAT_LONG = "POSITION_LAT_LONG"
    POSITION_TIMESTAMP = "POSITION_TIMESTAMP"
    LIVE_ZONESET_HASH = "LIVE_ZONESET_HASH"
    ZONE_TIMESTAMP = "ZONE_TIMESTAMP"
    ZONE_PACKET_TIMESTAMP = "ZONE_PACKET_TIMESTAMP"
    ZONE_STATES = "ZONE_STATES"
    ZONE_ALERT_FLAGS = "ZONE_ALERT_FLAGS"


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


def packet_ts(packet: Packet) -> int:
    """Return the packet timestamp in nanoseconds"""
    return packet.host_timestamp

from enum import Enum
from typing import Union

import numpy as np

from . import _sensor

BufferT = Union[bytes, bytearray, memoryview]


class Channel(Enum):
    """Channels available in lidar packets."""
    RANGE = (0, np.uint32)
    REFLECTIVITY = (4, np.uint16)
    INTENSITY = (6, np.uint16)
    AMBIENT = (8, np.uint16)

    def __init__(self, offset: int, dtype: type):
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


class Packet:
    """Read packet data using numpy views.

    Todo:
        This requires some information about packet data layout not provided by
        packet_format. Bytes per pixel will be variable in future fw.
    """

    PIXEL_BYTES: int = 12
    COL_PREAMBLE_BYTES: int = 16
    COL_FOOTER_BYTES: int = 4

    def __init__(self, data: BufferT, pf: _sensor.PacketFormat) -> None:
        self._pf = pf
        self._data = np.array(data, copy=False)
        self._column_bytes = Packet.COL_PREAMBLE_BYTES + \
            (Packet.PIXEL_BYTES * self._pf.pixels_per_column) + \
            Packet.COL_FOOTER_BYTES

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

    def view(self, field: Union[Channel, ColHeader]) -> np.ndarray:
        """Create a zero-copy view of the specified data.

        Args:
            field: Specifies either a channel or column header

        Returns:
            view of the specified data as a numpy array
        """

        if isinstance(field, Channel):
            return np.lib.stride_tricks.as_strided(
                self._view(Packet.COL_PREAMBLE_BYTES + field.offset,
                           field.dtype),
                shape=(self._pf.pixels_per_column,
                       self._pf.columns_per_packet),
                strides=(Packet.PIXEL_BYTES, self._column_bytes))

        elif isinstance(field, ColHeader):
            start = 0 if field.offset >= 0 else self._column_bytes
            return np.lib.stride_tricks.as_strided(
                self._view(field.offset + start, field.dtype),
                shape=(self._pf.columns_per_packet, ),
                strides=(self._column_bytes, ))

        else:
            raise TypeError("Expected either a Channel or ColHeader")

"""
\package OsLidarData
This module has code for viewing lidar data
"""
from . import _sensor as sensor
import struct
import math
import numpy as np


class OsLidarData:
    """
    \class OsLidarData
    \brief This class has code creating no-copy views of the lidar data
    """
    def __init__(self, data: bytearray, pf: sensor.PacketFormat) -> None:
        self._pf = pf
        self._pybuffer = data
        self._data = np.array(data, copy=False)
        # TODO: this requires some information about packet data layout not
        # provided by packet_format. Bytes per pixel will be variable in future fw
        self._pixel_bytes = 12
        self._col_preamble_size = 16
        self._column_bytes = self._col_preamble_size + self._pixel_bytes * self._pf.pixels_per_column + 4

    def _view(self, offset: int, data_type: type) -> np.ndarray:
        """
        \brief Internal method for creating a view from a numpy array
        \param offset Number of bytes the view should be offseted from the base data
        \param data_type The numpy data type that the view will be casted into
        """
        min = offset
        max = -((self._data.size - offset) % np.dtype(data_type).itemsize)
        return self._data[min:max].view(dtype=data_type)

    def make_pixel_range_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for px range
        \return The zero copy view of the os lidar data for px range

        This method creates a view (no memory copied) that is a 2 dimensional array
        of the pixel range data. This view is indexed by pixels, columns
        """
        data_type = np.uint32
        temp_view = self._view(self._col_preamble_size, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.pixels_per_column, self._pf.columns_per_packet),
            strides=(self._pixel_bytes, self._column_bytes))

    def make_pixel_reflectivity_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for px reflectivity
        \return The zero copy view of the os lidar data for px reflectivity

        This method creates a view (no memory copied) that is a 2 dimensional array
        of the pixel reflectivity data. This view is indexed by pixels, columns
        """
        data_type = np.uint16
        temp_view = self._view(self._col_preamble_size + 4, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.pixels_per_column, self._pf.columns_per_packet),
            strides=(self._pixel_bytes, self._column_bytes))

    def make_pixel_signal_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for px signal
        \return The zero copy view of the os lidar data for px signal

        This method creates a view (no memory copied) that is a 2 dimensional array
        of the pixel signal data. This view is indexed by pixels, columns
        """
        data_type = np.uint16
        temp_view = self._view(self._col_preamble_size + 6, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.pixels_per_column, self._pf.columns_per_packet),
            strides=(self._pixel_bytes, self._column_bytes))

    def make_pixel_noise_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for px noise
        \return The zero copy view of the os lidar data for px noise

        This method creates a view (no memory copied) that is a 2 dimensional array
        of the pixel noise data. This view is indexed by pixels, columns
        """
        data_type = np.uint16
        temp_view = self._view(self._col_preamble_size + 8, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.pixels_per_column, self._pf.columns_per_packet),
            strides=(self._pixel_bytes, self._column_bytes))

    def make_col_timestamp_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for column timestamps
        \return The zero copy view of the os lidar data for column timestamps

        This method creates a view (no memory copied) that is a 1 dimensional array
        of the column timestamps data. This view is indexed by columns
        """
        data_type = np.int64
        temp_view = self._view(0, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.columns_per_packet, ),
            strides=(self._column_bytes, ))

    def make_col_encoder_count_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for column encoder counts
        \return The zero copy view of the os lidar data for column encoder counts

        This method creates a view (no memory copied) that is a 1 dimensional array
        of the column encoder counts data. This view is indexed by columns
        """
        data_type = np.uint32
        temp_view = self._view(12, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.columns_per_packet, ),
            strides=(self._column_bytes, ))

    def make_col_measurement_id_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for column measurement ids
        \return The zero copy view of the os lidar data for column measurement ids

        This method creates a view (no memory copied) that is a 1 dimensional array
        of the column measurement ids data. This view is indexed by columns
        """
        data_type = np.uint16
        temp_view = self._view(8, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.columns_per_packet, ),
            strides=(self._column_bytes, ))

    def make_col_frame_id_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for column frame ids
        \return The zero copy view of the os lidar data for column frame ids

        This method creates a view (no memory copied) that is a 1 dimensional array
        of the column frame ids data. This view is indexed by columns
        """
        data_type = np.uint16
        temp_view = self._view(10, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.columns_per_packet, ),
            strides=(self._column_bytes, ))

    def make_col_valid_view(self) -> np.ndarray:
        """
        \brief Create a zero copy view of the os lidar data for column validity
        \return The zero copy view of the os lidar data for column validity

        This method creates a view (no memory copied) that is a 1 dimensional array
        of the column validity data. This view is indexed by columns
        """
        data_type = np.uint32
        temp_view = self._view(
            self._col_preamble_size +
            self._pf.pixels_per_column * self._pixel_bytes, data_type)
        return np.lib.stride_tricks.as_strided(
            temp_view,
            shape=(self._pf.columns_per_packet, ),
            strides=(self._column_bytes, ))

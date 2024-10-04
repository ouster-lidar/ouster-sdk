"""Ouster sensor Python client.

Copyright (c) 2021, Ouster, Inc.
All rights reserved.

This module contains more idiomatic wrappers around the lower-level module
generated using pybind11.
"""
from contextlib import closing
from typing import (cast, Iterable, Iterator, List, Optional, Tuple,
                    Union, Callable)
import time
import logging
import numpy as np
import warnings
from more_itertools import take
from typing_extensions import Protocol

from ouster.sdk._bindings.client import (SensorInfo, PacketFormat, LidarScan, ScanBatcher, get_field_types,
                      LidarPacket, Packet, FieldType)

from .data import (FieldTypes)

from .scan_source import ScanSource

logger = logging.getLogger("ouster.sdk.client.core")


class ClientError(Exception):
    """Base class for client errors."""
    pass


class ClientTimeout(ClientError):
    """Raised when data does not arrive within the expected time."""
    pass


class ClientOverflow(ClientError):
    """Raised when data loss is possible due to internal buffers filling up."""
    pass


class PacketSource(Protocol):
    """Represents a single-sensor data stream."""

    def __iter__(self) -> Iterator[Packet]:
        """A PacketSource supports ``Iterable[Packet]``.

        Currently defined explicitly due to:
        https://github.com/python/typing/issues/561
        """
        ...

    @property
    def metadata(self) -> SensorInfo:
        """Metadata associated with the packet stream."""
        ...

    def close(self) -> None:
        """Release the underlying resource, if any."""
        ...

    @property
    def is_live(self):
        ...


class Packets(PacketSource):
    """Create a :class:`PacketSource` from an existing iterator."""

    _it: Iterable[Packet]
    _metadata: SensorInfo

    def __init__(self, it: Iterable[Packet], metadata: SensorInfo):
        """
        Args:
            it: A stream of packets
            metadata: Metadata for the packet stream
        """
        self._it = it
        self._metadata = metadata

    @property
    def metadata(self) -> SensorInfo:
        return self._metadata

    def __iter__(self) -> Iterator[Packet]:
        """Return the underlying iterator."""
        return iter(self._it)

    def close(self) -> None:
        pass

    @property
    def is_live(self) -> bool:
        return False


class Scans(ScanSource):
    """Deprecated: An iterable stream of scans batched from a PacketSource.

    Batching will emit a scan every time the frame_id increments (i.e. on
    receiving first packet in the next scan). Reordered packets will be handled,
    except across frame boundaries: packets from the previous scan will be
    dropped.

    Optionally filters out incomplete frames and enforces a timeout. A batching
    timeout can be useful to detect when we're only receiving incomplete frames
    or only imu packets. Can also be configured to manage internal buffers for
    soft real-time applications.
    """

    def __init__(self,
                 source: PacketSource,
                 *,
                 complete: bool = False,
                 timeout: Optional[float] = 2.0,
                 fields: Optional[List[FieldType]] = None,
                 _max_latency: int = 0) -> None:
        """
        Args:
            source: any source of packets
            complete: if True, only return full scans
            timeout: seconds to wait for a scan before error or None
            fields: specify which channel fields to populate on LidarScans
            _max_latency: (experimental) approximate max number of frames to buffer
        """
        warnings.warn("client.Scans(...) is deprecated: "
                  "Use client.ScansMulti(...).single_source(0) or the appropriate scan source directly instead. "
                  "This API is planned to be removed in Q4 2024.",
                  DeprecationWarning, stacklevel=2)
        self._source = source
        self._complete = complete
        self._timeout = timeout
        self._max_latency = _max_latency
        # used to initialize LidarScan
        self._field_types: FieldTypes = (
            fields if fields is not None else
            get_field_types(self._source.metadata.format.udp_profile_lidar))

        self._fields = []
        for f in self._field_types:
            self._fields.append(f.name)

    def __iter__(self) -> Iterator[LidarScan]:
        """Get an iterator."""

        w = self._source.metadata.format.columns_per_frame
        h = self._source.metadata.format.pixels_per_column
        columns_per_packet = self._source.metadata.format.columns_per_packet
        packets_per_frame = w // columns_per_packet
        column_window = self._source.metadata.format.column_window

        # If source is a sensor, make a type-specialized reference available
        from ouster.sdk.client import Sensor
        sensor = cast(Sensor, self._source) if isinstance(
            self._source, Sensor) else None

        ls_write = None
        pf = PacketFormat.from_info(self._source.metadata)
        batch = ScanBatcher(w, pf)

        # Time from which to measure timeout
        start_ts = time.monotonic()

        it = iter(self._source)
        self._packets_consumed = 0
        self._scans_produced = 0
        while True:
            try:
                packet = next(it)
                self._packets_consumed += 1
            except StopIteration:
                if ls_write is not None:
                    if not self._complete or ls_write.complete(column_window):
                        yield ls_write
                return

            if self._timeout is not None and (time.monotonic() >=
                                              start_ts + self._timeout):
                raise ClientTimeout(
                    f"No valid frames received within {self._timeout}s")

            if isinstance(packet, LidarPacket):
                ls_write = ls_write or LidarScan(
                    h, w, self._field_types, columns_per_packet)

                if batch(packet, ls_write):
                    # Got a new frame, return it and start another
                    if not self._complete or ls_write.complete(column_window):
                        yield ls_write
                        self._scans_produced += 1
                        start_ts = time.monotonic()
                    ls_write = None

                    # Drop data along frame boundaries to maintain _max_latency and
                    # clear out already-batched first packet of next frame
                    if self._max_latency and sensor is not None:
                        buf_frames = sensor.buf_use // packets_per_frame
                        drop_frames = buf_frames - self._max_latency + 1

                        if drop_frames > 0:
                            sensor.flush(drop_frames)
                            batch = ScanBatcher(w, pf)

    def close(self) -> None:
        """Close the underlying PacketSource."""
        self._source.close()

    @property
    def metadata(self) -> SensorInfo:
        """Return metadata from the underlying PacketSource."""
        return self._source.metadata

    @property
    def is_live(self) -> bool:
        return self._source.is_live

    @property
    def is_seekable(self) -> bool:
        return False

    @property
    def is_indexed(self) -> bool:
        return False

    @property
    def field_types(self) -> List[FieldType]:
        return self._field_types

    @property
    def fields(self) -> List[str]:
        return self._fields

    @property
    def scans_num(self) -> Optional[int]:
        return None

    def __len__(self) -> int:
        raise TypeError("len is not supported on live or non-indexed sources")

    def _seek(self, _) -> None:
        raise RuntimeError(
            "can not invoke __getitem__ on non-indexed source")

    def __getitem__(self, _: Union[int, slice]
                    ) -> Union[Optional[LidarScan], ScanSource]:
        raise RuntimeError(
            "can not invoke __getitem__ on non-indexed source")

    def __del__(self) -> None:
        pass

    def _slice_iter(self, _: slice) -> Iterator[Optional[LidarScan]]:
        raise NotImplementedError

    def slice(self, _: slice) -> ScanSource:
        raise NotImplementedError

    @classmethod
    def sample(
        cls,
        hostname: str = "localhost",
        n: int = 1,
        lidar_port: int = 7502,
        *,
        metadata: Optional[SensorInfo] = None
    ) -> Tuple[SensorInfo, Iterator[List[LidarScan]]]:
        """Conveniently sample n consecutive scans from a sensor.

        Does not leave UDP ports open. Suitable for interactive use.

        Args:
            hostname: hostname of the sensor
            n: number of consecutive frames in each sample
            lidar_port: UDP port to listen on for lidar data
            metadata: explicitly provide metadata for the stream

        Returns:
            A tuple of metadata queried from the sensor and an iterator that
            samples n consecutive scans
        """
        from .sensor import Sensor
        with closing(Sensor(hostname, lidar_port, 7503,
                            metadata=metadata)) as sensor:
            metadata = sensor.metadata

        def next_batch() -> List[LidarScan]:
            with closing(
                    Sensor(hostname,
                           lidar_port,
                           7503,
                           metadata=metadata,
                           buf_size=n * 0.2,
                           _flush_before_read=False)) as source:
                source.flush(full=True)
                scans = cls(source, timeout=2.0, complete=True, _max_latency=0)
                return take(n, scans)

        return metadata, iter(next_batch, [])

    @classmethod
    def stream(
            cls,
            hostname: str = "localhost",
            lidar_port: int = 7502,
            *,
            buf_size: float = 1.0,
            timeout: Optional[float] = 2.0,
            complete: bool = True,
            metadata: Optional[SensorInfo] = None,
            fields: Optional[List[FieldType]] = None) -> 'Scans':
        """Stream scans from a sensor.

        Will drop frames preemptively to avoid filling up internal buffers and
        to avoid returning frames older than the scanning period of the sensor.

        Args:
            hostname: hostname of the sensor
            lidar_port: UDP port to listen on for lidar data
            timeout: seconds to wait for scans before signaling error
            complete: if True, only return full scans
            metadata: explicitly provide metadata for the stream
            fields: specify which channel fields to populate on LidarScans
        """
        from .sensor import Sensor
        source = Sensor(hostname,
                        lidar_port,
                        7503,
                        metadata=metadata,
                        buf_size=buf_size,
                        timeout=timeout,
                        _flush_before_read=True)

        return cls(source,
                   timeout=timeout,
                   complete=complete,
                   fields=fields,
                   _max_latency=2)


class FrameBorder:
    """Create callable helper that indicates the cross frames packets."""

    def __init__(self, meta: SensorInfo, pred: Callable[[Packet], bool] = lambda _: True):
        self._last_f_id = -1
        self._last_packet_ts = None
        self._last_packet_res = False
        self._pred = pred
        self._pf = PacketFormat(meta)

    def __call__(self, packet: Packet) -> bool:
        if isinstance(packet, LidarPacket):
            # don't examine packets again
            if (self._last_packet_ts and (packet.host_timestamp != 0) and
                    self._last_packet_ts == packet.host_timestamp):
                return self._last_packet_res
            f_id = self._pf.frame_id(packet.buf)
            changed = (self._last_f_id != -1 and f_id != self._last_f_id)
            self._last_packet_res = changed and self._pred(packet)
            self._last_f_id = f_id
            return self._last_packet_res
        return False


def first_valid_column(scan: LidarScan) -> int:
    """Return first valid column of a LidarScan"""
    return int(np.bitwise_and(scan.status, 1).argmax())


def last_valid_column(scan: LidarScan) -> int:
    """Return last valid column of a LidarScan"""
    return int(scan.w - 1 - np.bitwise_and(scan.status, 1)[::-1].argmax())


def first_valid_column_ts(scan: LidarScan) -> int:
    """Return first valid column timestamp of a LidarScan"""
    return scan.timestamp[first_valid_column(scan)]


def first_valid_packet_ts(scan: LidarScan) -> int:
    """Return first valid packet timestamp of a LidarScan"""
    columns_per_packet = scan.w // scan.packet_timestamp.shape[0]
    return scan.packet_timestamp[first_valid_column(scan) // columns_per_packet]


def last_valid_packet_ts(scan: LidarScan) -> int:
    """Return first valid packet timestamp of a LidarScan"""
    columns_per_packet = scan.w // scan.packet_timestamp.shape[0]
    return scan.packet_timestamp[last_valid_column(scan) // columns_per_packet]


def last_valid_column_ts(scan: LidarScan) -> int:
    """Return last valid column timestamp of a LidarScan"""
    return scan.timestamp[last_valid_column(scan)]


def first_valid_column_pose(scan: LidarScan) -> np.ndarray:
    """Return first valid column pose of a LidarScan"""
    return scan.pose[first_valid_column(scan)]


def last_valid_column_pose(scan: LidarScan) -> np.ndarray:
    """Return last valid column pose of a LidarScan"""
    return scan.pose[last_valid_column(scan)]


def valid_packet_idxs(scan: LidarScan) -> np.ndarray:
    """Checks for valid packets that was used in in the scan construction"""
    valid_cols = scan.status & 0x1
    valid_packet_ts = scan.packet_timestamp != 0
    sp = np.split(valid_cols, scan.packet_timestamp.shape[0])
    # here we consider the packet is valid when either one is true:
    #   - any columns in the packet has a valid status
    #   - packet_timestamp is not zero, which may occur even when
    #     all columns/px data in invalid state within the packet.
    #     It means that we received the packet without per px data
    #     but with all other headers in place
    valid_packets = np.logical_or(np.any(sp, axis=1), valid_packet_ts)
    return np.nonzero(valid_packets)[0]


def poses_present(scan: LidarScan) -> bool:
    """Check whether any of scan.pose in not identity"""
    return not np.allclose(np.eye(4), scan.pose)

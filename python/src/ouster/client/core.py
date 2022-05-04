"""Ouster sensor Python client.

Copyright (c) 2021, Ouster, Inc.
All rights reserved.
 
This module contains more idiomatic wrappers around the lower-level module
generated using pybind11.
"""
from contextlib import closing
from typing import cast, Dict, Iterable, Iterator, List, Optional, Tuple, Union
from threading import Thread
import time

from more_itertools import take
from typing_extensions import Protocol

from . import _client
from ._client import (SensorInfo, LidarScan, UDPProfileLidar)
from .data import (ChanField, FieldDType, ImuPacket, LidarPacket, Packet)


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


class ScanSource(Protocol):
    """Represents a single-sensor data stream."""

    def __iter__(self) -> Iterator[LidarScan]:
        """A ScanSource supports ``Iterable[LidarScan]``.

        Currently defined explicitly due to:
        https://github.com/python/typing/issues/561
        """
        ...

    @property
    def metadata(self) -> SensorInfo:
        """Metadata associated with the scan stream."""
        ...

    def close(self) -> None:
        """Release the underlying resource, if any."""
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


class Sensor(PacketSource):
    """A packet source listening on local UDP ports.

    Uses a separate thread that fills internal buffers without holding the GIL.

    Note:
        Make sure ``close()`` will be called on all instances before Python
        attempts to exit, or the interpreter will hang waiting to join the
        thread (like any other non-daemonized Python thread).
    """

    _cli: _client.Client
    _timeout: Optional[float]
    _metadata: SensorInfo
    _pf: _client.PacketFormat
    _producer: Thread
    _cache: Optional[Tuple[_client.ClientState, bytearray]]

    def __init__(self,
                 hostname: str = "localhost",
                 lidar_port: int = 7502,
                 imu_port: int = 7503,
                 *,
                 metadata: Optional[SensorInfo] = None,
                 buf_size: int = 128,
                 timeout: Optional[float] = 1.0,
                 _overflow_err: bool = False,
                 _flush_before_read: bool = True,
                 _legacy_format: bool = True) -> None:
        """
        Neither the ports nor udp destination configuration on the sensor will
        be updated. The metadata will be fetched over the network from the
        sensor unless explicitly provided using the ``metadata`` parameter.

        Args:
            hostname: hostname or IP address of the sensor
            lidar_port: UDP port to listen on for lidar data
            imu_port: UDP port to listen on for imu data
            metadata: explicitly provide metadata for the stream
            buf_size: number of packets to buffer before dropping data
            timeout: seconds to wait for packets before signaling error or None
            _overflow_err: if True, raise ClientOverflow
            _flush_before_read: if True, try to clear buffers before reading
            _legacy_format: if True, use legacy metadata format

        Raises:
            ClientError: If initializing the client fails.
        """
        self._cli = _client.Client(hostname, lidar_port, imu_port, buf_size)
        self._timeout = timeout
        self._overflow_err = _overflow_err
        self._flush_before_read = _flush_before_read
        self._cache = None
        self._fetched_meta = ""
        self._legacy_format = _legacy_format

        # Fetch from sensor if not explicitly provided
        if metadata:
            self._metadata = metadata
        else:
            self._fetch_metadata()
            self._metadata = SensorInfo(self._fetched_meta)
        self._pf = _client.PacketFormat.from_info(self._metadata)

        # Use args to avoid capturing self causing circular reference
        self._producer = Thread(target=lambda cli, pf: cli.produce(pf),
                                args=(self._cli, self._pf))
        self._producer.start()

    def _fetch_metadata(self) -> None:
        if not self._fetched_meta:
            self._fetched_meta = self._cli.get_metadata(
                legacy=self._legacy_format)
            if not self._fetched_meta:
                raise ClientError("Failed to collect metadata")

    def write_metadata(self, path: str) -> None:
        """Save metadata to disk.

        Args:
            path: path to write
        """
        self._fetch_metadata()
        with open(path, 'w') as f:
            f.write(self._fetched_meta)

    @property
    def metadata(self) -> SensorInfo:
        return self._metadata

    def _next_packet(self) -> Optional[Packet]:
        st, buf = self._peek()
        self._cache = None

        if self._overflow_err and st & _client.ClientState.OVERFLOW:
            raise ClientOverflow()
        if st & _client.ClientState.LIDAR_DATA:
            return LidarPacket(buf, self._metadata)
        elif st & _client.ClientState.IMU_DATA:
            return ImuPacket(buf, self._metadata)
        elif st == _client.ClientState.TIMEOUT:
            raise ClientTimeout(f"No packets received within {self._timeout}s")
        elif st & _client.ClientState.ERROR:
            raise ClientError("Client returned ERROR state")
        elif st & _client.ClientState.EXIT:
            return None

        raise AssertionError("Should be unreachable")

    def _peek(self) -> Tuple[_client.ClientState, bytearray]:
        if self._cache is None:
            # Lidar packets are bigger than IMU: wastes some space but is simple
            buf = bytearray(self._pf.lidar_packet_size)
            st = self._cli.consume(
                buf, -1 if self._timeout is None else self._timeout)
            self._cache = (st, buf)
        return self._cache

    def __iter__(self) -> Iterator[Packet]:
        """Access the UDP data stream as an iterator.

        Reading may block waiting for network data for up to the specified
        timeout. Failing to consume this iterator faster than the data rate of
        the sensor may cause packets to be dropped.

        Raises:
            ClientTimeout: if no packets are received within the configured
                timeout
            ClientError: if the client enters an unspecified error state
            ValueError: if the packet source has already been closed
        """

        if not self._producer.is_alive():
            raise ValueError("I/O operation on closed packet source")

        # Attempt to flush any old data before producing packets
        if self._flush_before_read:
            self.flush(full=True)

        while True:
            try:
                p = self._next_packet()
                if p is not None:
                    yield p
                else:
                    break
            except ValueError:
                # TODO: bad packet size or init_id here: this can happen when
                # packets are buffered by the OS, not necessarily an error
                pass

    def flush(self, n_frames: int = 3, *, full=False) -> int:
        """Drop some data to clear internal buffers.

        Args:
            n_frames: number of frames to drop
            full: clear internal buffers first, so data is read from the OS
                  receive buffers (or the network) directly

        Returns:
            The number of packets dropped

        Raises:
            ClientTimeout: if a lidar packet is not received within the
                configured timeout
            ClientError: if the client enters an unspecified error state
        """
        if full:
            self._cli.flush()

        last_frame = -1
        n_dropped = 0
        last_ts = time.monotonic()
        while True:
            # check next packet to see if it's the start of a new frame
            st, buf = self._peek()
            if st & _client.ClientState.LIDAR_DATA:
                frame = self._pf.frame_id(buf)
                if frame != last_frame:
                    last_frame = frame
                    n_frames -= 1
                    if n_frames < 0:
                        break
                last_ts = time.monotonic()
            elif st & _client.ClientState.ERROR:
                raise ClientError("Client returned ERROR state")
            elif st & _client.ClientState.EXIT:
                break

            # check for timeout
            if self._timeout is not None and (time.monotonic() >=
                                              last_ts + self._timeout):
                raise ClientTimeout(
                    f"No packets received within {self._timeout}s")
            # drop cached packet
            self._cache = None
            n_dropped += 1

        return n_dropped

    def buf_use(self) -> int:
        return self._cli.size

    def close(self) -> None:
        """Shut down producer thread and close network connection.

        Attributes may be unset if constructor throws an exception.
        """
        if hasattr(self, '_cli'):
            self._cli.shutdown()
        if hasattr(self, '_producer'):
            self._producer.join()

    def __del__(self) -> None:
        self.close()


class Scans:
    """An iterable stream of scans batched from a PacketSource.

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
                 timeout: Optional[float] = None,
                 fields: Optional[Dict[ChanField, FieldDType]] = None,
                 _max_latency: int = 0) -> None:
        """
        Args:
            source: any source of packets
            complete: if True, only return full scans
            timeout: seconds to wait for a scan before error or None
            fields: specify which channel fields to populate on LidarScans
            _max_latency: (experimental) approximate max number of frames to buffer
        """
        self._source = source
        self._complete = complete
        self._timeout = timeout
        self._max_latency = _max_latency
        # used to initialize LidarScan
        self._fields: Union[Dict[ChanField, FieldDType], UDPProfileLidar] = (
            fields if fields is not None else
            self._source.metadata.format.udp_profile_lidar)

    def __iter__(self) -> Iterator[LidarScan]:
        """Get an iterator."""

        w = self._source.metadata.format.columns_per_frame
        h = self._source.metadata.format.pixels_per_column
        packets_per_frame = w // self._source.metadata.format.columns_per_packet
        column_window = self._source.metadata.format.column_window

        # If source is a sensor, make a type-specialized reference available
        sensor = cast(Sensor, self._source) if isinstance(
            self._source, Sensor) else None

        ls_write = None
        pf = _client.PacketFormat.from_info(self._source.metadata)
        batch = _client.ScanBatcher(w, pf)

        # Time from which to measure timeout
        start_ts = time.monotonic()

        it = iter(self._source)
        while True:
            try:
                packet = next(it)
            except StopIteration:
                if ls_write is not None:
                    if not self._complete or ls_write._complete(column_window):
                        yield ls_write
                return

            if self._timeout is not None and (time.monotonic() >=
                                              start_ts + self._timeout):
                raise ClientTimeout(f"No lidar scans within {self._timeout}s")

            if isinstance(packet, LidarPacket):
                ls_write = ls_write or LidarScan(h, w, self._fields)

                if batch(packet._data, ls_write):
                    # Got a new frame, return it and start another
                    if not self._complete or ls_write._complete(column_window):
                        yield ls_write
                        start_ts = time.monotonic()
                    ls_write = None

                    # Drop data along frame boundaries to maintain _max_latency and
                    # clear out already-batched first packet of next frame
                    if self._max_latency and sensor is not None:
                        buf_frames = sensor.buf_use() // packets_per_frame
                        drop_frames = buf_frames - self._max_latency + 1

                        if drop_frames > 0:
                            sensor.flush(drop_frames)
                            batch = _client.ScanBatcher(w, pf)

    def close(self) -> None:
        """Close the underlying PacketSource."""
        self._source.close()

    @property
    def metadata(self) -> SensorInfo:
        """Return metadata from the underlying PacketSource."""
        return self._source.metadata

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
        with closing(Sensor(hostname, metadata=metadata)) as sensor:
            metadata = sensor.metadata

        def next_batch() -> List[LidarScan]:
            with closing(
                    Sensor(hostname,
                           lidar_port,
                           metadata=metadata,
                           buf_size=n * 128,
                           _flush_before_read=False)) as source:
                source.flush(full=True)
                scans = cls(source, timeout=1.0, complete=True, _max_latency=0)
                return take(n, scans)

        return metadata, iter(next_batch, [])

    @classmethod
    def stream(
            cls,
            hostname: str = "localhost",
            lidar_port: int = 7502,
            *,
            buf_size: int = 640,
            timeout: Optional[float] = 1.0,
            complete: bool = True,
            metadata: Optional[SensorInfo] = None,
            fields: Optional[Dict[ChanField, FieldDType]] = None) -> 'Scans':
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
        source = Sensor(hostname,
                        lidar_port,
                        metadata=metadata,
                        buf_size=buf_size,
                        timeout=timeout,
                        _flush_before_read=True)

        return cls(source,
                   timeout=timeout,
                   complete=complete,
                   fields=fields,
                   _max_latency=2)

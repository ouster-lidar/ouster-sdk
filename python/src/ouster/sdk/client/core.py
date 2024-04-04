"""Ouster sensor Python client.

Copyright (c) 2021, Ouster, Inc.
All rights reserved.

This module contains more idiomatic wrappers around the lower-level module
generated using pybind11.
"""
from contextlib import closing
from typing import (cast, Dict, Iterable, Iterator, List, Optional, Tuple,
                    Union, Callable)
from threading import Thread
import time
from math import ceil
import numpy as np

from more_itertools import take
from typing_extensions import Protocol

from ._client import (SensorInfo, SensorConnection, Client, ClientState,
                      PacketFormat, LidarScan, ScanBatcher, get_field_types)

from .data import (ChanField, FieldDType, ImuPacket,
                   LidarPacket, Packet, PacketIdError,
                   FieldTypes)

from .scan_source import ScanSource


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


class Sensor(PacketSource):
    """A packet source listening on local UDP ports.

    Uses a separate thread that fills internal buffers without holding the GIL.

    Note:
        Make sure ``close()`` will be called on all instances before Python
        attempts to exit, or the interpreter will hang waiting to join the
        thread (like any other non-daemonized Python thread).
    """

    _connection: SensorConnection
    _cli: Client
    _timeout: Optional[float]
    _metadata: SensorInfo
    _pf: PacketFormat
    _producer: Thread
    _cache: Optional[ClientState]
    _lidarbuf: LidarPacket
    _imubuf: ImuPacket
    _lidar_port: int
    _hostname: str

    def __init__(self,
                 hostname: str,
                 lidar_port: int,
                 imu_port: int,
                 *,
                 metadata: Optional[SensorInfo] = None,
                 buf_size: int = 128,
                 timeout: Optional[float] = 2.0,
                 _overflow_err: bool = False,
                 _flush_before_read: bool = True,
                 _flush_frames: int = 5,
                 _legacy_format: bool = False,
                 soft_id_check: bool = False,
                 _skip_metadata_beam_validation: bool = False) -> None:
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
            _flush_frames: the number of frames to skip/flush on start of a new iter
            _legacy_format: if True, use legacy metadata format
            soft_id_check: if True, don't skip lidar packets buffers on,
            id mismatch (init_id/sn pair),
            _skip_metadata_beam_validation: if True, skip metadata beam angle check

        Raises:
            ClientError: If initializing the client fails.
        """
        self._connection = SensorConnection(
            hostname, lidar_port, imu_port)
        self._timeout = timeout
        self._overflow_err = _overflow_err
        self._flush_before_read = _flush_before_read
        self._cache = None
        self._fetched_meta = ""
        self._flush_frames = _flush_frames
        self._legacy_format = _legacy_format

        self._soft_id_check = soft_id_check
        self._id_error_count = 0
        self._skip_metadata_beam_validation = _skip_metadata_beam_validation

        self._hostname = hostname
        self._lidar_port = lidar_port

        # Fetch from sensor if not explicitly provided
        if metadata:
            self._metadata = metadata
        else:
            self._fetch_metadata()
            self._metadata = SensorInfo(
                self._fetched_meta, self._skip_metadata_beam_validation)
        self._pf = PacketFormat.from_info(self._metadata)
        self._cli = Client(self._connection, buf_size,
                           self._pf.lidar_packet_size, buf_size,
                           self._pf.imu_packet_size)
        self._lidarbuf = LidarPacket(None,
                                     self._metadata,
                                     _raise_on_id_check=not self._soft_id_check)
        self._imubuf = ImuPacket(packet_format=self._pf)

        # Use args to avoid capturing self causing circular reference
        self._producer = Thread(target=self._cli.produce)
        self._producer.start()

    @property
    def is_live(self) -> bool:
        return True

    @property
    def lidar_port(self) -> int:
        return self._connection.lidar_port

    @property
    def imu_port(self) -> int:
        return self._connection.imu_port

    def _fetch_metadata(self, timeout: Optional[float] = None) -> None:
        timeout_sec = 45
        if timeout:
            timeout_sec = ceil(timeout)
        if not self._fetched_meta:
            self._fetched_meta = self._connection.get_metadata(
                legacy=self._legacy_format, timeout_sec=timeout_sec)
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
        st = self._peek()
        self._cache = None

        # TODO: revise this part and upper loop to eliminate ValueError
        if st & ClientState.OVERFLOW:
            if self._overflow_err:
                raise ClientOverflow("client packets overflow")
            else:
                raise ValueError()
        if st & ClientState.LIDAR_DATA:
            packet = LidarPacket(self._lidarbuf._data, self._metadata,
                                 self._lidarbuf.capture_timestamp,
                                 _raise_on_id_check = not self._soft_id_check)
            if packet.id_error:
                self._id_error_count += 1
            return packet
        elif st & ClientState.IMU_DATA:
            return self._imubuf
        elif st == ClientState.TIMEOUT:
            raise ClientTimeout(f"No packets received within {self._timeout}s from sensor "
                                f"{self._hostname} using udp destination {self._metadata.config.udp_dest} "
                                f"on port {self._lidar_port}. Check your firewall settings and/or ensure "
                                f"that the lidar port {self._lidar_port} is not being held open.")
        elif st & ClientState.ERROR:
            raise ClientError("Client returned ERROR state")
        elif st & ClientState.EXIT:
            return None

        raise AssertionError("Should be unreachable")

    def _peek(self) -> ClientState:
        if self._cache is None:
            st = self._cli.consume(self._lidarbuf,
                                   self._imubuf,
                                   -1 if self._timeout is None else self._timeout)
            self._cache = st
        return self._cache

    def __iter__(self) -> Iterator[Packet]:
        """Access the UDP data stream as an iterator.

        Reading may block waiting for network data for up to the specified
        timeout. Failing to consume this iterator faster than the data rate of
        the sensor may cause packets to be dropped. Returned packet is meant to
        be consumed prior to incrementing the iterator, and storing the returned
        packet in a container may result in the contents being invalidated. If
        such behaviour is necessary, deepcopy the packets upon retrieval.

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
            self.flush(n_frames=self._flush_frames, full=True)

        while True:
            try:
                p = self._next_packet()
                if p is not None:
                    yield p
                else:
                    break
            except PacketIdError:
                self._id_error_count += 1
            except ValueError:
                # bad packet size here: this can happen when
                # packets are buffered by the OS, not necessarily an error
                # same pass as in data.py
                # TODO: introduce status for PacketSource to indicate frequency
                # of bad packet size or init_id errors
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
            st = self._peek()
            if st & ClientState.LIDAR_DATA:
                frame = self._pf.frame_id(self._lidarbuf._data)
                if frame != last_frame:
                    last_frame = frame
                    n_frames -= 1
                    if n_frames < 0:
                        break
                last_ts = time.monotonic()
            elif st & ClientState.ERROR:
                raise ClientError("Client returned ERROR state")
            elif st & ClientState.EXIT:
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

    @property
    def buf_use(self) -> int:
        return self._cli.size

    @property
    def id_error_count(self) -> int:
        return self._id_error_count

    def close(self) -> None:
        """Shut down producer thread and close network connection.

        Attributes may be unset if constructor throws an exception.
        """
        if hasattr(self, '_cli'):
            self._cli.shutdown()
        if hasattr(self, '_producer'):
            self._producer.join()
        if hasattr(self, '_connection'):
            self._connection.shutdown()

    def __del__(self) -> None:
        self.close()


class Scans(ScanSource):
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
                 timeout: Optional[float] = 2.0,
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
        self._fields: FieldTypes = (
            fields if fields is not None else
            get_field_types(self._source.metadata.format.udp_profile_lidar))

    def __iter__(self) -> Iterator[LidarScan]:
        """Get an iterator."""

        w = self._source.metadata.format.columns_per_frame
        h = self._source.metadata.format.pixels_per_column
        columns_per_packet = self._source.metadata.format.columns_per_packet
        packets_per_frame = w // columns_per_packet
        column_window = self._source.metadata.format.column_window

        # If source is a sensor, make a type-specialized reference available
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
                raise ClientTimeout(f"No valid frames received within {self._timeout}s")

            if isinstance(packet, LidarPacket):
                ls_write = ls_write or LidarScan(
                    h, w, self._fields, columns_per_packet)

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
    def fields(self) -> FieldTypes:
        return self._fields

    @property
    def scans_num(self) -> Optional[int]:
        return None

    def __len__(self) -> int:
        raise TypeError("len is not supported on live or non-indexed sources")

    def _seek(self, int) -> None:
        raise RuntimeError(
            "can not invoke __getitem__ on non-indexed source")

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[Optional[LidarScan], List[Optional[LidarScan]]]:
        raise RuntimeError(
            "can not invoke __getitem__ on non-indexed source")

    def __del__(self) -> None:
        pass

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
        with closing(Sensor(hostname, lidar_port, 7503,
                            metadata=metadata)) as sensor:
            metadata = sensor.metadata

        def next_batch() -> List[LidarScan]:
            with closing(
                    Sensor(hostname,
                           lidar_port,
                           7503,
                           metadata=metadata,
                           buf_size=n * 128,
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
            buf_size: int = 640,
            timeout: Optional[float] = 2.0,
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

    def __init__(self, pred: Callable[[Packet], bool] = lambda _: True):
        self._last_f_id = -1
        self._last_packet_ts = None
        self._last_packet_res = False
        self._pred = pred

    def __call__(self, packet: Packet) -> bool:
        if isinstance(packet, LidarPacket):
            # don't examine packets again
            if (self._last_packet_ts and packet.capture_timestamp and
                    self._last_packet_ts == packet.capture_timestamp):
                return self._last_packet_res
            f_id = packet.frame_id
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

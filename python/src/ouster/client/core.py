"""Ouster sensor python client.

This module is WIP and will contain more idiomatic wrappers around the
lower-level pyblind11-generated module (ouster.client._sensor).
"""
from contextlib import closing
from more_itertools import take
from typing import cast, Iterable, Iterator, List, Optional
from typing_extensions import Protocol
from threading import Thread
import time

from . import _sensor
from . import (ColHeader, PacketFormat, SensorInfo, ImuPacket, LidarPacket,
               LidarScan, Packet)


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
        """A PacketSource supports Iterable[Packet].

        Currently defined explicitly due to:
        https://github.com/python/typing/issues/561
        """
        ...

    @property
    def metadata(self) -> SensorInfo:
        """Metadata associated with the data."""
        ...

    def close(self) -> None:
        """Release the underlying resource, if any."""
        ...


class Packets(PacketSource):
    """Packets from an iterator."""

    _it: Iterator[Packet]
    _metadata: SensorInfo

    def __init__(self, it: Iterable[Packet], metadata: SensorInfo):
        """
        Args:
            it: a stream of packets
            metadata: metadata for the packet stream
        """
        self._it = iter(it)
        self._metadata = metadata

    @property
    def metadata(self) -> SensorInfo:
        return self._metadata

    def __iter__(self) -> Iterator[Packet]:
        return self._it

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

    _cli: Optional[_sensor.Client] = None
    _metadata: SensorInfo
    _pf: PacketFormat
    _producer: Thread

    def __init__(self,
                 hostname: str = "localhost",
                 lidar_port: int = 7502,
                 imu_port: int = 7503,
                 *,
                 metadata: Optional[SensorInfo] = None,
                 buf_size: int = 128,
                 timeout: float = 1.0,
                 _overflow_err: bool = True,
                 _flush_before_read: bool = True) -> None:
        """
        Neither the ports nor udp destination on the sensor will be updated. The
        metadata will be fetched over the network from the sensor unless
        explicitly set using the ``metadata`` parameter.

        Args:
            hostname: hostname of the sensor
            lidar_port: UDP port to listen on for lidar data
            imu_port: UDP port to listen on for imu data
            metadata: explicitly provide metadata for the stream
            buf_size: number of packets to buffer before dropping data
            timeout: seconds to wait for packets before signaling error
            _overflow_err: if True, raise ClientOverflow
            _flush_before_read: if True, try to clear buffers before reading
        """
        self._cli = _sensor.Client(hostname, lidar_port, imu_port, buf_size)
        self._timeout = timeout
        self._overflow_err = _overflow_err
        self._flush_before_read = _flush_before_read

        # Fetch from sensor if not explicitly provided
        if metadata:
            self._metadata = metadata
        else:
            raw = self._cli.get_metadata()
            if not raw:
                self._cli = None
                raise ClientError("Failed to collect metadata")
            self._metadata = SensorInfo(raw)
        self._pf = PacketFormat(self._metadata)

        # Use args to avoid capturing self causing circular reference
        self._producer = Thread(target=lambda cli, pf: cli.produce(pf),
                                args=(self._cli, self._pf))
        self._producer.start()

    @property
    def metadata(self) -> SensorInfo:
        return self._metadata

    def __iter__(self) -> Iterator[Packet]:
        if self._cli is None:
            raise ClientError("Client is already shut down")

        # attempt to flush any old data before producing packets
        if self._flush_before_read:
            self.flush(full=True)

        while True:
            # Lidar packets are bigger than IMU: wastes some space but is simple
            buf = bytearray(self._pf.lidar_packet_size)
            st = self._cli.consume(buf, self._timeout)
            if self._overflow_err and st & _sensor.ClientState.OVERFLOW:
                raise ClientOverflow()
            if st & _sensor.ClientState.LIDAR_DATA:
                yield LidarPacket(buf, self._pf)
            elif st & _sensor.ClientState.IMU_DATA:
                yield ImuPacket(buf, self._pf)
            elif st == _sensor.ClientState.TIMEOUT:
                raise ClientTimeout()
            elif st & _sensor.ClientState.ERROR:
                raise ClientError("Client returned ERROR state")
            elif st & _sensor.ClientState.EXIT:
                return

    def buf_use(self) -> int:
        return 0 if self._cli is None else self._cli.size

    def flush(self, n_frames: int = 1, *, full=False) -> int:
        """Drop some data to clear internal buffers.

        Will raise ClientTimeout if a lidar packet is not received withing the
        configured timeout.

        Args:
            n_frames: number of frames to drop
            full: clear internal buffers first, so data is read from the OS
                  receive buffers (or the network) directly

        Notes:
            - need some limit to avoid hanging when we're only receiving IMU
            - some code duplication with __iter__ here
        """

        if self._cli is None:
            return 0

        # Flush internal buffer completely before looking for frames
        if full:
            self._cli.flush()

        # Look for the last packet in a frame
        last_packet_ind = (self._metadata.format.columns_per_frame -
                           self._metadata.format.columns_per_packet)

        n_dropped = 0
        last_ts = time.monotonic()
        buf = bytearray(self._pf.lidar_packet_size)
        while n_frames > 0:
            st = self._cli.consume(buf, self._timeout)
            if st & _sensor.ClientState.LIDAR_DATA:
                if LidarPacket(buf, self._pf).view(
                        ColHeader.MEASUREMENT_ID)[0] == last_packet_ind:
                    n_frames -= 1
                last_ts = time.monotonic()
            elif self._timeout and time.monotonic() > last_ts + self._timeout:
                raise ClientTimeout()
            elif st == _sensor.ClientState.TIMEOUT:
                raise ClientTimeout()
            elif st & _sensor.ClientState.ERROR:
                raise ClientError("Client returned ERROR state")
            elif st & _sensor.ClientState.EXIT:
                break
            n_dropped += 1

        return n_dropped

    def close(self) -> None:
        """Shut down producer thread and close network connection."""
        if self._cli is not None:
            self._cli.shutdown()
        # may be unset if constructor throw and close() is called from del
        if hasattr(self, '_producer'):
            self._producer.join()

    def __del__(self) -> None:
        self.close()


class Scans:
    """An iterable stream of scans batched from a PacketSource.

    Batching will emitting a scan every time the frame_id increments (i.e. on
    receiving first packet in the next scan). Reordered packets will be handled,
    except across frame boundaries: packets from the previous scan will be
    dropped.

    Optionally filters out incomplete frames and enforces a timeout. A batching
    timeout can be useful to detect when we're only receiving incomplete frames
    or only imu packets. Also can be configured to manage internal buffers for
    soft real-time applications.

    Managing the buffer size may eventually be done by the underlying
    PacketSource. Due to the way the underlying C++ code is structured, it
    currently needs to interact closely with scan batching in order to drop data
    intelligently between frame boundaries.
    """
    def __init__(self,
                 source: PacketSource,
                 *,
                 complete: bool = False,
                 timeout: Optional[float] = None,
                 _max_latency: int = 0) -> None:
        """
        If the packet source is a ``Sensor`` and _max_latency is n > 0, try to
        manage internal buffers so that we only ever return at most the nth
        oldest frame. This will drop data earlier than the buffered PacketSource
        would otherwise.

        Args:
            source: any source of packets
            complete: if True, only return full scans
            timeout: if non-zero, maximum time to wait for a scan
            _max_latency: (experimental) approximate max number of frames to buffer
        """
        self._source = source
        self._complete = complete
        self._timeout = timeout
        self._max_latency = _max_latency

    def __iter__(self) -> Iterator[LidarScan]:
        w = self._source.metadata.format.columns_per_frame
        h = self._source.metadata.format.pixels_per_column
        packets_per_frame = w // self._source.metadata.format.columns_per_packet

        # if source is a sensor, make a type-specialized reference available
        sensor = cast(Sensor, self._source) if isinstance(
            self._source, Sensor) else None

        ls_write = _sensor.LidarScan(w, h)
        pf = PacketFormat(self._source.metadata)
        batch = _sensor.ScanBatcher(w, pf)

        # time from which to measure timeout
        last_ts = time.monotonic()

        it = iter(self._source)
        while True:
            try:
                packet = next(it)
            except StopIteration:
                return

            if self._timeout is not None and (time.monotonic() >=
                                              last_ts + self._timeout):
                raise ClientTimeout("Failed to produce a scan before timeout")

            if isinstance(packet, LidarPacket):
                if batch(packet._data, ls_write):
                    # Got a new frame, return it and start another
                    ls = LidarScan.from_native(ls_write)
                    if not self._complete or ls.complete:
                        last_ts = time.monotonic()
                        yield ls
                    ls_write = _sensor.LidarScan(w, h)

                    # Drop data along frame boundaries to maintain _max_latency and
                    # clear out already-batched first packet of next frame
                    if self._max_latency and sensor is not None:
                        buf_frames = sensor.buf_use() // packets_per_frame
                        drop_frames = buf_frames - self._max_latency + 1

                        if drop_frames > 0:
                            sensor.flush(drop_frames)
                            batch = _sensor.ScanBatcher(w, pf)

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
    ) -> Iterator[List[LidarScan]]:
        """Sample n consecutive scans from a sensor.

        Does not leave UDP ports open. Suitable for interactive use.

        Args:
            hostname: hostname of the sensor
            n: number of consecutive frames in each sample
            lidar_port: UDP port to listen on for lidar data
            metadata: explicitly provide metadata for the stream
        """
        with closing(Sensor(hostname, metadata=metadata)) as sensor:
            metadata = sensor.metadata

        while True:
            with closing(
                    Sensor(hostname,
                           lidar_port,
                           metadata=metadata,
                           buf_size=n * 128,
                           _overflow_err=False,
                           _flush_before_read=False)) as source:
                source.flush(full=True)
                scans = cls(source, timeout=1.0, complete=True, _max_latency=0)
                ls = take(n, scans)
            yield ls

    @classmethod
    def stream(cls,
               hostname: str = "localhost",
               lidar_port: int = 7502,
               *,
               buf_size: int = 640,
               timeout: float = 1.0,
               complete: bool = True,
               metadata: Optional[SensorInfo] = None) -> 'Scans':
        """Stream scans from a sensor.

        Will drop frames preemptively to avoid filling up internal buffers and
        to avoid returning frames older than the scanning period of the sensor.

        Args:
            hostname: hostname of the sensor
            lidar_port: UDP port to listen on for lidar data
            timeout: seconds to wait for scans before signaling error
            complete: if True, only return full scans
            metadata: explicitly provide metadata for the stream
        """
        source = Sensor(hostname,
                        lidar_port,
                        metadata=metadata,
                        buf_size=buf_size,
                        timeout=timeout,
                        _overflow_err=True,
                        _flush_before_read=True)

        return cls(source, timeout=timeout, complete=complete, _max_latency=1)

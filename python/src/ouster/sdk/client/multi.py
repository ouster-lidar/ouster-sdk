from typing_extensions import Protocol
from typing import Any, Tuple, List, Union, Optional, Iterator, Iterable, Callable

import copy

from .core import PacketSource, first_valid_packet_ts, ClientError, ClientTimeout, ClientOverflow
from ouster.sdk._bindings.client import (SensorInfo, LidarScan, PacketFormat, ScanBatcher, ClientEventType,
                      SensorConfig, SensorClient, ClientEvent, FieldType,
                      get_field_types, Packet, ImuPacket, LidarPacket, PacketValidationFailure)
from .data import packet_ts, FieldTypes
from .scan_source import ScanSource
from .multi_scan_source import MultiScanSource
from ouster.sdk._bindings.client import Sensor as _Sensor
import logging
import time

logger = logging.getLogger("ouster.sdk.client.multi")


def collate_scans(
    source: Iterable[Tuple[int, Any]],
    sensors_count: int,
    get_ts: Callable[[Any], int],
    *,
    dt: int = 210000000
) -> Iterator[List[Optional[Any]]]:
    """Collate by sensor idx with a cut every `dt` (ns) time length.

    Assuming that multi sensor packets stream are PTP synced, so the sensor
    time of LidarScans don't have huge deltas in time, though some latency
    of packets receiving (up to dt) should be ok.

    Args:
        source: data stream with scans
        sensors_count: number of sensors generating the stream of scans
        dt: max time difference between scans in the collated scan (i.e.
            max time period at which every new collated scan is released/cut),
            default is 0.21 s
    Returns:
        List of LidarScans elements
    """
    min_ts = -1
    max_ts = -1
    collated = [None] * sensors_count
    for idx, m in source:
        ts = get_ts(m)
        if min_ts < 0 or max_ts < 0 or (
                ts >= min_ts + dt or ts < max_ts - dt):
            if any(collated):
                # process collated (reached dt boundary, if used)
                yield collated  # type: ignore
                collated = [None] * sensors_count

            min_ts = max_ts = ts

        if collated[idx]:
            # process collated (reached the existing scan)
            yield collated
            collated = [None] * sensors_count
            min_ts = max_ts = ts

        collated[idx] = m   # type: ignore

        # if we have one of everything, yield
        if not any(elem is None for elem in collated):
            # process collated (reached the existing scan)
            yield collated
            collated = [None] * sensors_count
            min_ts = max_ts = ts

        if ts < min_ts:
            min_ts = ts

        if ts > max_ts:
            max_ts = ts

    # process the last one
    if any(collated):
        # process collated (the very last one, if any)
        yield collated  # type: ignore


class PacketMultiSource(Protocol):
    """Represents a multi-sensor data stream."""

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        """A PacketSource supports ``Iterable[Tuple[int, Packet]]``.

        Currently defined explicitly due to:
        https://github.com/python/typing/issues/561
        """
        ...

    @property
    def metadata(self) -> List[SensorInfo]:
        """Metadata associated with the packet streams."""
        ...

    @property
    def is_live(self) -> bool:
        ...

    @property
    def is_seekable(self) -> bool:
        ...

    @property
    def is_indexed(self) -> bool:
        ...

    def restart(self) -> None:
        """Restart playback, only relevant to non-live sources"""
        ...

    def close(self) -> None:
        """Release the underlying resources, if any."""
        ...


class SensorPacketSource(PacketMultiSource):
    """A packet source listening on local UDP ports.

    Optionally uses a background thread to buffer packets without holding the GIL.

    Note:
        Make sure ``close()`` will be called on all instances before Python
        attempts to exit, or the interpreter will hang waiting to join the
        thread (like any other non-daemonized Python thread).
    """

    _cli: SensorClient
    _timeout: Optional[float]
    _metadata: List[SensorInfo]
    _pf: List[PacketFormat]
    _cache: Optional[ClientEvent]
    _lidarbuf: LidarPacket
    _imubuf: ImuPacket

    def __init__(self, sensors: List[Tuple[str, SensorConfig]],
                 *,
                 metadata: Optional[List[SensorInfo]] = None,
                 buf_size: float = 0.2,
                 timeout: Optional[float] = 2.0,
                 _overflow_err: bool = False,
                 _flush_before_read: bool = True,
                 _flush_frames: int = 5,
                 soft_id_check: bool = False,
                 _skip_metadata_beam_validation: bool = False) -> None:
        """
        Neither the ports nor udp destination configuration on the sensor will
        be updated. The metadata will be fetched over the network from the
        sensor unless explicitly provided using the ``metadata`` parameter.

        Args:
            sensors: list of pairs of hostnames and desired sensor configurations to connect to
            metadata: explicitly provide metadata for the stream
            buf_size: time in seconds to buffer packets before dropping data
            timeout: seconds to wait for packets before signaling error or None
            _overflow_err: if True, raise ClientOverflow
            _flush_before_read: if True, try to clear buffers before reading
            _flush_frames: the number of frames to skip/flush on start of a new iter
            soft_id_check: if True, don't skip lidar packets buffers on,
            id mismatch (init_id/sn pair),
            _skip_metadata_beam_validation: if True, skip metadata beam angle check

        Raises:
            ValueError: If invalid arguments are provided
            RuntimeError: If initializing the client fails.
        """
        self._sensors = []
        for hostname, config in sensors:
            self._sensors.append(_Sensor(hostname, config))

        self._timeout = timeout
        self._overflow_err = _overflow_err
        self._flush_before_read = _flush_before_read
        self._cache = None
        self._flush_frames = _flush_frames

        self._soft_id_check = soft_id_check
        self._id_error_count = 0
        self._skip_metadata_beam_validation = _skip_metadata_beam_validation

        self._cli = SensorClient(self._sensors, metadata if metadata is not None else [], 45, buf_size)
        self._pf = []
        for meta in self._cli.get_sensor_info():
            self._pf.append(PacketFormat(meta))

        self._lidarbuf = LidarPacket()
        self._imubuf = ImuPacket()

        self._metadata = self._cli.get_sensor_info()
        self._last_times = [time.monotonic()] * len(self._metadata)
        self._last_dropped = 0

    @property
    def is_live(self) -> bool:
        return True

    @property
    def is_indexed(self) -> bool:
        return False

    @property
    def is_seekable(self) -> bool:
        return False

    def restart(self) -> None:
        return

    def write_metadata(self, index: int, path: str) -> None:
        """Save metadata to disk.

        Args:
            path: path to write
        """
        with open(path, 'w') as f:
            f.write(self._metadata[index].to_json_string())

    @property
    def metadata(self) -> List[SensorInfo]:
        return self._metadata

    def _next_packet(self) -> Optional[Tuple[int, Packet]]:
        st = self._peek()
        self._cache = None

        # TODO: revise this part and upper loop to eliminate ValueError
        if self._overflow_err:
            new = self._cli.dropped_packets()
            if self._last_dropped < new:
                self._last_dropped = new
                raise ClientOverflow("client packets overflow")
        if st.type == ClientEventType.LidarPacket:
            msg = self._lidarbuf
            self._lidarbuf = LidarPacket(self._pf[st.source].lidar_packet_size)
            res = msg.validate(self._metadata[st.source], self._pf[st.source])
            if res == PacketValidationFailure.PACKET_SIZE:
                raise ValueError(f"Packet was unexpected size {len(msg.buf)}")
            if res == PacketValidationFailure.ID:
                self._id_error_count += 1
                init_id = self._pf[st.source].init_id(msg.buf)
                prod_sn = self._pf[st.source].prod_sn(msg.buf)
                error_msg = f"Metadata init_id/sn does not match: " \
                    f"expected by metadata - {self._metadata[st.source].init_id}/{self._metadata[st.source].sn}, " \
                    f"but got from packet buffer - {init_id}/{prod_sn}"
                if not self._soft_id_check:
                    raise ValueError(error_msg)
                else:
                    # Continue with warning. When init_ids/sn doesn't match
                    # the resulting LidarPacket has high chances to be
                    # incompatible with data format set in metadata json file
                    logger.warn(f"LidarPacket validation: {error_msg}")
            self._last_times[st.source] = time.monotonic()
            return (st.source, msg)
        elif st.type == ClientEventType.ImuPacket:
            msg2 = self._imubuf
            self._imubuf = ImuPacket(self._pf[st.source].imu_packet_size)
            self._last_times[st.source] = time.monotonic()
            return (st.source, msg2)
        elif st.type == ClientEventType.Error:
            raise ClientError("Client returned ERROR state")
        elif st.type == ClientEventType.Exit:
            raise ValueError("I/O operation on closed packet source")

        raise AssertionError("Should be unreachable")

    def _peek(self) -> ClientEvent:
        if self._cache is None:
            while True:
                st = self._cli.get_packet(self._lidarbuf, self._imubuf)
                self.check_timeout()
                if st.type != ClientEventType.PollTimeout:
                    break
            self._cache = st
        return self._cache

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
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
        while True:
            # check next packet to see if it's the start of a new frame
            st = self._peek()
            if st.type == ClientEventType.LidarPacket:
                frame = self._pf[st.source].frame_id(self._lidarbuf.buf)
                if frame != last_frame:
                    last_frame = frame
                    n_frames -= 1
                    if n_frames < 0:
                        break
                self._last_times[st.source] = time.monotonic()
            elif st.type == ClientEventType.Error:
                raise ClientError("Client returned ERROR state")
            elif st.type == ClientEventType.Exit:
                raise ValueError("I/O operation on closed packet source")

            # drop cached packet
            self._cache = None
            n_dropped += 1

        return n_dropped

    def check_timeout(self):
        expire_time = time.monotonic() - self._timeout
        for i, t in enumerate(self._last_times):
            if t < expire_time:
                metadata = self._metadata[i]
                sensor = self._sensors[i]
                raise ClientTimeout(f"No packets received within {self._timeout}s from sensor "
                                    f"{sensor.hostname()} using udp destination {metadata.config.udp_dest} "
                                    f"on port {metadata.config.udp_port_lidar}. Check your "
                                    f"firewall settings and/or ensure "
                                    f"that the lidar port {metadata.config.udp_port_lidar} is not being held open.")

    @property
    def buf_use(self) -> int:
        return self._cli.buffer_size()

    @property
    def id_error_count(self) -> int:
        return self._id_error_count

    def close(self) -> None:
        """Shut down producer thread and close network connection.

        Attributes may be unset if constructor throws an exception.
        """
        if hasattr(self, '_cli'):
            self._cli.close()

    def __del__(self) -> None:
        self.close()

    def single_source(self, stream_idx: int) -> PacketSource:
        from ouster.sdk.client.packet_source_adapter import PacketSourceAdapter
        return PacketSourceAdapter(self, stream_idx)


# TODO: schedule for removal
class PacketMultiWrapper(PacketMultiSource):
    """Wrap PacketSource to the PacketMultiSource interface"""

    def __init__(self,
                 source: Union[PacketSource, PacketMultiSource]) -> None:
        self._source = source

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        for p in self._source:
            yield (0, p) if isinstance(p, (LidarPacket,
                                           ImuPacket)) else p  # type: ignore

    @property
    def metadata(self) -> List[SensorInfo]:
        """Metadata associated with the packet streams."""
        meta = self._source.metadata
        return [meta] if isinstance(meta, SensorInfo) else meta

    def close(self) -> None:
        """Release the underlying resource, if any."""
        self._source.close()

    @property
    def buf_use(self) -> int:
        if hasattr(self._source, "buf_use"):
            return self._source.buf_use
        else:
            return -1


class ScansMulti(MultiScanSource):
    """Multi LidarScan source."""

    def __init__(
        self,
        source: PacketMultiSource,
        *,
        dt: int = 210000000,
        complete: bool = False,
        cycle: bool = False,
        fields: Optional[List[FieldTypes]] = None,
        **_
    ) -> None:
        """
        Args:
            source: packet multi source
            dt: max time difference between scans in the collated scan (i.e.
                time period at which every new collated scan is released/cut),
                default is 0.21s
            complete: set to True to only release complete scans
            cycle: repeat infinitely after iteration is finished is True.
                    in case source refers to a live sensor then this parameter
                    has no effect.
            fields: specify which channel fields to populate on LidarScans
        """
        self._source = source
        self._dt = dt
        self._complete = complete
        self._cycle = cycle
        # NOTE[self]: this fields override property may need to double checked
        # for example, what would happen if the length of override doesn't
        # match with the actual underlying metadata size. Is this a supported
        # behavior? For now throwing an error if they don't match in size.
        file_fields = [get_field_types(
            sinfo) for sinfo in self._source.metadata]
        if fields:
            if len(fields) != len(file_fields):
                raise ValueError("Size of Field override doesn't match")
            self._field_types = fields
        else:
            self._field_types = file_fields
        self._fields = []
        for l in self._field_types:
            fl = []
            for f in l:
                fl.append(f.name)
            self._fields.append(fl)

    @property
    def sensors_count(self) -> int:
        return len(self._source.metadata)

    @property
    def metadata(self) -> List[SensorInfo]:
        return self._source.metadata

    @property
    def is_live(self) -> bool:
        return self._source.is_live

    @property
    def is_seekable(self) -> bool:
        return self._source.is_seekable

    @property
    def is_indexed(self) -> bool:
        return self._source.is_indexed

    @property
    def fields(self) -> List[List[str]]:
        return self._fields

    @property
    def field_types(self) -> List[List[FieldType]]:
        return self._field_types

    @property
    def scans_num(self) -> List[Optional[int]]:
        if self.is_live or not self.is_indexed:
            return [None] * self.sensors_count
        raise NotImplementedError

    def __len__(self) -> int:
        if self.is_live or not self.is_indexed:
            raise TypeError(
                "len is not supported on unindexed or live sources")
        raise NotImplementedError

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:
        return collate_scans(self._scans_iter(True, self._cycle, True), self.sensors_count,
                             first_valid_packet_ts,
                             dt=self._dt)

    def _scans_iter(self, restart=True, cycle=False, deep_copy=False
                    ) -> Iterator[Tuple[int, LidarScan]]:
        """
        Parameters:
            restart: restart source from beginning if applicable
            cycle: when reaching end auto restart
            deep_copy: perform deepcopy when yielding scans
        """
        w = [0] * self.sensors_count
        h = [0] * self.sensors_count
        col_window = [(0, 0)] * self.sensors_count
        columns_per_packet = [0] * self.sensors_count
        pf = []

        # construct things that dont need to reset with a loop
        for i, sinfo in enumerate(self.metadata):
            w[i] = sinfo.format.columns_per_frame
            h[i] = sinfo.format.pixels_per_column
            col_window[i] = sinfo.format.column_window
            columns_per_packet[i] = sinfo.format.columns_per_packet
            pf.append(PacketFormat.from_info(sinfo))

        # autopep8: off
        scan_shallow_yield = lambda x: x
        scan_deep_yield = lambda x: copy.deepcopy(x)
        scan_yield_op = scan_deep_yield if deep_copy else scan_shallow_yield
        # autopep8: on

        if restart and hasattr(self._source, "restart"):
            self._source.restart()  # start from the beginning
        while True:
            ls_write = []
            batch = []
            yielded: List[Optional[int]] = [None] * self.sensors_count

            # construct things that we need to reset with a loop
            for i, sinfo in enumerate(self.metadata):
                batch.append(ScanBatcher(sinfo))
                ls_write.append(LidarScan(
                    h[i], w[i], self._field_types[i], columns_per_packet[i]))

            had_message = False
            for idx, packet in self._source:
                if isinstance(packet, LidarPacket):
                    if batch[idx](packet.buf, packet_ts(packet), ls_write[idx]):
                        if not self._complete or ls_write[idx].complete(col_window[idx]):
                            had_message = True
                            yield idx, scan_yield_op(ls_write[idx])
                            yielded[idx] = ls_write[idx].frame_id

            # return the last not fully cut scans in the sensor timestamp order if
            # they satisfy the completeness criteria
            skip_ls = lambda idx, ls: ls is None or ls.frame_id in [yielded[idx], -1]
            last_scans = sorted(
                [(idx, ls) for idx, ls in enumerate(ls_write) if not skip_ls(idx, ls)],
                key=lambda si: first_valid_packet_ts(si[1]))
            while last_scans:
                idx, ls = last_scans.pop(0)
                if not self._complete or ls.complete(col_window[idx]):
                    had_message = True
                    yield idx, scan_yield_op(ls)

            # exit if we had no scans so we dont infinite loop when cycling
            if cycle and had_message:
                self._source.restart()
            else:
                break

    def _seek(self, offset: int) -> None:
        if not self.is_seekable:
            raise RuntimeError("can not invoke _seek on non-seekable source")
        self._source.seek(offset)  # type: ignore

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], MultiScanSource]:

        if not self.is_indexed:
            raise RuntimeError(
                "can not invoke __getitem__ on non-indexed source")
        raise NotImplementedError

    def close(self) -> None:
        if self._source:
            self._source.close()
            self._source = None  # type: ignore

    def __del__(self) -> None:
        self.close()

    def single_source(self, stream_idx: int) -> ScanSource:
        from .scan_source_adapter import ScanSourceAdapter
        return ScanSourceAdapter(self, stream_idx)

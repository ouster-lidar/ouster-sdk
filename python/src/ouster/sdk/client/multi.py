#  type: ignore
from typing_extensions import Protocol
from typing import Any, Tuple, List, Union, Optional, Iterator, Callable

import copy

from ._client import (SensorInfo, LidarScan, PacketFormat, ScanBatcher,
                      get_field_types)
from .data import Packet, ImuPacket, LidarPacket, packet_ts, FieldTypes
from .core import PacketSource, first_valid_packet_ts
from .scan_source import ScanSource
from .multi_scan_source import MultiScanSource


def collate_scans(
    source: Iterator[Tuple[int, Any]],
    sensors_count: int,
    get_ts: Callable[[Any], int],
    *,
    dt: int = 10**8
) -> Iterator[List[Optional[Any]]]:
    """Collate by sensor idx with a cut every `dt` (ns) time length.

    Assuming that multi sensor packets stream are PTP synced, so the sensor
    time of LidarScans don't have huge deltas in time, though some latency
    of packets receiving (up to dt) should be ok.

    Args:
        source: data stream with scans
        sensors_count: number of sensors generating the stream of scans
        dt: max time difference between scans in the collated scan (i.e.
            time period at which every new collated scan is released/cut),
            default is 0.1 s
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


# TODO: schedule for removal
class PacketMultiWrapper(PacketMultiSource):
    """Wrap PacketSource to the PacketMultiSource interface"""

    def __init__(self,
                 source: Union[PacketSource, PacketMultiSource]) -> None:
        self._source = source

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        for p in self._source:
            yield (0, p) if isinstance(p, (LidarPacket,
                                           ImuPacket)) else p

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
        dt: int = 10**8,
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
                default is 0.1s
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
                raise ValueError("Size of Field override doens't match")
            self._fields = fields
        else:
            self._fields = file_fields

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
    def fields(self) -> List[FieldTypes]:
        return self._fields

    @property
    def scans_num(self) -> List[Optional[int]]:
        if self.is_live or not self.is_indexed:
            return [None] * self.sensors_count
        raise NotImplementedError

    def __len__(self) -> int:
        if self.is_live or not self.is_indexed:
            raise TypeError("len is not supported on unindexed or live sources")
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
        w = [int] * self.sensors_count
        h = [int] * self.sensors_count
        col_window = [int] * self.sensors_count
        columns_per_packet = [int] * self.sensors_count
        pf = [None] * self.sensors_count
        ls_write = [None] * self.sensors_count
        batch = [None] * self.sensors_count

        for i, sinfo in enumerate(self.metadata):
            w[i] = sinfo.format.columns_per_frame
            h[i] = sinfo.format.pixels_per_column
            col_window[i] = sinfo.format.column_window
            columns_per_packet[i] = sinfo.format.columns_per_packet
            pf[i] = PacketFormat.from_info(sinfo)
            batch[i] = ScanBatcher(w[i], pf[i])

        # autopep8: off
        scan_shallow_yield = lambda x: x
        scan_deep_yield = lambda x: copy.deepcopy(x)
        scan_yield_op = scan_deep_yield if deep_copy else scan_shallow_yield
        # autopep8: on

        if restart:
            self._source.restart()  # start from the beginning
        while True:
            had_message = False
            for idx, packet in self._source:
                if isinstance(packet, LidarPacket):
                    ls_write[idx] = ls_write[idx] or LidarScan(
                        h[idx], w[idx], self._fields[idx], columns_per_packet[idx])
                    if batch[idx](packet._data, packet_ts(packet), ls_write[idx]):
                        if not self._complete or ls_write[idx].complete(col_window[idx]):
                            had_message = True
                            yield idx, scan_yield_op(ls_write[idx])

            # return the last not fully cut scans in the sensor timestamp order if
            # they satisfy the completeness criteria
            last_scans = sorted(
                [(idx, ls) for idx, ls in enumerate(ls_write) if ls is not None],
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
        self._source.seek(offset)

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], List[List[Optional[LidarScan]]]]:

        if not self.is_indexed:
            raise RuntimeError(
                "can not invoke __getitem__ on non-indexed source")
        raise NotImplementedError

    def close(self) -> None:
        if self._source:
            self._source.close()
            self._source = None

    def __del__(self) -> None:
        self.close()

    def single_source(self, stream_idx: int) -> ScanSource:
        from .scan_source_adapter import ScanSourceAdapter
        return ScanSourceAdapter(self, stream_idx)

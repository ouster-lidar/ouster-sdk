from typing import Any, Tuple, List, Optional, Iterator, Iterable, Callable

import copy

from .core import PacketSource
from ouster.sdk._bindings.client import (SensorInfo, LidarScan, PacketFormat, ScanBatcher,
                      FieldType, ScanSource,
                      get_field_types, LidarPacket)
import numpy as np
import logging
import warnings

logger = logging.getLogger("ouster.sdk.core.multi")


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
    warnings.warn("collate_scans is deprecated: use ouster.sdk.core.collate instead. "
                  "collate_scans will be removed in the upcoming release.",
                  DeprecationWarning, stacklevel=2)

    min_ts = np.int64(-1)
    max_ts = np.int64(-1)
    collated = [None] * sensors_count
    for idx, m in source:
        ts = np.int64(get_ts(m))
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


class Scans(ScanSource):
    """Multi LidarScan source."""

    def __init__(
        self,
        source: PacketSource,
        *,
        complete: bool = False,
        cycle: bool = False,
        fields: Optional[List[List[FieldType]]] = None,
        **_
    ) -> None:
        """
        Args:
            source: packet multi source
            complete: set to True to only release complete scans
            cycle: repeat infinitely after iteration is finished is True.
                    in case source refers to a live sensor then this parameter
                    has no effect.
            fields: specify which channel fields to populate on LidarScans
        """
        ScanSource.__init__(self)
        self._source = source
        self._complete = complete
        self._cycle = cycle
        # NOTE[self]: this fields override property may need to double checked
        # for example, what would happen if the length of override doesn't
        # match with the actual underlying metadata size. Is this a supported
        # behavior? For now throwing an error if they don't match in size.
        file_fields = [get_field_types(
            sinfo) for sinfo in self._source.sensor_info]
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
    def sensor_info(self) -> List[SensorInfo]:
        return self._source.sensor_info

    @property
    def is_live(self) -> bool:
        return self._source.is_live

    @property
    def is_indexed(self) -> bool:
        return False

    @property
    def fields(self) -> List[List[str]]:
        return self._fields

    @property
    def field_types(self) -> List[List[FieldType]]:
        return self._field_types

    @property
    def scans_num(self) -> List[int]:
        raise NotImplementedError

    def __len__(self) -> int:
        if self.is_live or not self.is_indexed:
            raise TypeError(
                "len is not supported on unindexed or live sources")
        raise NotImplementedError

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:
        return self._scans_iter(self._cycle, True)

    def _scans_iter(self, cycle=False, deep_copy=False
                    ) -> Iterator[List[Optional[LidarScan]]]:
        """
        Parameters:
            cycle: when reaching end auto restart
            deep_copy: perform deepcopy when yielding scans
        """
        if self._source is None:
            raise ValueError("Cannot iterate a closed scan source")
        sensors_count = len(self.sensor_info)
        w = [0] * sensors_count
        h = [0] * sensors_count
        col_window = [(0, 0)] * sensors_count
        columns_per_packet = [0] * sensors_count
        pf = []

        # construct things that dont need to reset with a loop
        for i, sinfo in enumerate(self.sensor_info):
            w[i] = sinfo.format.columns_per_frame
            h[i] = sinfo.format.pixels_per_column
            col_window[i] = sinfo.format.column_window
            columns_per_packet[i] = sinfo.format.columns_per_packet
            pf.append(PacketFormat(sinfo))

        # autopep8: off
        scan_shallow_yield = lambda x: x
        scan_deep_yield = lambda x: copy.deepcopy(x)
        scan_yield_op = scan_deep_yield if deep_copy else scan_shallow_yield
        # autopep8: on

        while True:
            ls_write = []
            batch = []
            yielded: List[Optional[int]] = [None] * sensors_count

            # construct things that we need to reset with a loop
            for i, sinfo in enumerate(self.sensor_info):
                batch.append(ScanBatcher(sinfo))
                ls_write.append(LidarScan(
                    h[i], w[i], self._field_types[i], columns_per_packet[i]))

            had_message = False
            for idx, packet in self._source:
                if isinstance(packet, LidarPacket):
                    if batch[idx](packet, ls_write[idx]):
                        if not self._complete or ls_write[idx].complete(col_window[idx]):
                            had_message = True
                            yield [scan_yield_op(ls_write[idx])]
                            yielded[idx] = ls_write[idx].frame_id

            # return the last not fully cut scans in the sensor timestamp order if
            # they satisfy the completeness criteria
            skip_ls = lambda idx, ls: ls is None or ls.frame_id in [yielded[idx], -1]
            last_scans = sorted(
                [(idx, ls) for idx, ls in enumerate(ls_write) if not skip_ls(idx, ls)],
                key=lambda si: si[1].get_first_valid_packet_timestamp())
            while last_scans:
                idx, ls = last_scans.pop(0)
                if not self._complete or ls.complete(col_window[idx]):
                    had_message = True
                    yield [scan_yield_op(ls)]

            # exit if we had no scans so we dont infinite loop when cycling
            if cycle and had_message:
                continue
            else:
                break

    def close(self) -> None:
        if self._source:
            self._source.close()
            self._source = None  # type: ignore

    def __del__(self) -> None:
        self.close()

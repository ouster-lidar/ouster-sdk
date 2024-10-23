from typing import Iterator, List, Optional, Union

from ouster.sdk.client import LidarScan, first_valid_packet_ts
from ouster.sdk.client import ScansMulti     # type: ignore
from ouster.sdk.client import MultiScanSource
from ouster.sdk.client.multi import collate_scans   # type: ignore
from ouster.sdk.util import (resolve_field_types, ForwardSlicer, progressbar)  # type: ignore
from .bag_packet_source import BagPacketSource


class BagScanSource(ScansMulti):
    """Implements MultiScanSource protocol for pcap files with multiple sensors."""

    def __init__(
        self,
        file_path: str,
        *,
        dt: int = 210000000,
        complete: bool = False,
        index: bool = False,
        cycle: bool = False,
        raw_headers: bool = False,
        raw_fields: bool = False,
        soft_id_check: bool = False,
        meta: Optional[List[str]] = None,
        field_names: Optional[List[str]] = None,
        **_
    ) -> None:
        """
        Args:
            file_path: OSF filename as scans source
            dt: max time difference between scans in the collated scan (i.e.
                max time period at which every new collated scan is released/cut),
                default is 0.21s.
            complete: set to True to only release complete scans
            index: if this flag is set to true an index will be built for the pcap
                file enabling len, index and slice operations on the scan source, if
                the flag is set to False indexing is skipped (default is False).
            cycle: repeat infinitely after iteration is finished (default is False)
            raw_headers: if True, include raw headers in decoded LidarScans
            raw_fields: if True, include raw fields in decoded LidarScans
            soft_id_check: if True, don't skip packets on init_id/serial_num mismatch
            meta: optional list of metadata files to load, if not provided metadata
                is loaded from the bag instead
            field_names: list of fields to decode into a LidarScan, if not provided
                decodes all default fields
        """

        # initialize the attribute so close works correctly if we fail out
        self._source = None  # type: ignore
        try:
            self._source = BagPacketSource(file_path, soft_id_check=soft_id_check, meta=meta)
        except Exception:
            self._source = None  # type: ignore
            raise

        # generate the field types per sensor with flags/raw_fields if specified
        raw_fields |= (field_names is not None and len(field_names) != 0)
        field_types = resolve_field_types(self._source.metadata,
                                          raw_headers=raw_headers,
                                          raw_fields=raw_fields)
        # Cut out any undesired fields
        if field_names is not None:
            for i in range(0, len(field_types)):
                new_fts = []
                for name in field_names:
                    found = False
                    for ft in field_types[i]:
                        if ft.name == name:
                            new_fts.append(ft)
                            found = True
                            break
                    if not found:
                        raise RuntimeError(f"Requested field '{name}' does not exist in packet"
                                           f" format {self._source.metadata[i].config.udp_profile_lidar}")
                field_types[i] = new_fts

        super().__init__(self._source, dt=dt, complete=complete,
                         cycle=cycle, fields=field_types)

        if index:
            self._frame_offset = []
            pi = self._source._index    # type: ignore
            scans_itr = self._collated_scans_itr(
                self._scans_iter(True, False, True))
            # scans count in first source
            scans_count = len(pi.frame_id_indices[0])   # type: ignore
            for scan_idx, scans in enumerate(scans_itr):
                offsets = [pi.frame_id_indices[idx].get(    # type: ignore
                    scan.frame_id) for idx, scan in enumerate(scans) if scan]
                self._frame_offset.append(min([v for v in offsets if v]))
                progressbar(scan_idx, scans_count, "", "indexed")
            print("\nfinished building index")

    def _collated_scans_itr(self, scans_itr):
        return collate_scans(scans_itr, self.sensors_count,
                             first_valid_packet_ts, dt=self._dt)

    @property
    def scans_num(self) -> List[Optional[int]]:
        if not self.is_indexed:
            return [None] * self.sensors_count
        pi = self._source._index    # type: ignore
        return [pi.frame_count(i) for i in range(self.sensors_count)]   # type: ignore

    @property
    def id_error_count(self) -> int:
        return self._source.id_error_count  # type: ignore

    @property
    def size_error_count(self) -> int:
        return self._source.size_error_count  # type: ignore

    def __len__(self) -> int:
        if not self.is_indexed:
            raise TypeError("len is not supported on non-indexed source")
        return len(self._frame_offset)

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], MultiScanSource]:

        if not self.is_indexed:
            raise TypeError(
                "can not invoke __getitem__ on non-indexed source")

        if isinstance(key, int):
            L = len(self)
            if key < 0:
                key += L
            if key < 0 or key >= L:
                raise IndexError("index is out of range")
            offset = self._frame_offset[key]
            self._source.seek(offset)   # type: ignore
            scans_itr = self._scans_iter(False, False, True)
            return next(self._collated_scans_itr(scans_itr))

        if isinstance(key, slice):
            return self.slice(key)

        raise TypeError(
            f"indices must be integer or slice, not {type(key).__name__}")

    def _slice_iter(self, key: slice) -> Iterator[List[Optional[LidarScan]]]:
        # NOTE: In this method if key.step was negative, this won't be
        # result in the output being reversed, it is the responsibility of
        # the caller to accumulate the results into a vector then return them.
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        count = k.stop - k.start
        if count <= 0:
            return iter(())
        offset = self._frame_offset[k.start]
        self._source.seek(offset)   # type: ignore
        scans_itr = self._collated_scans_itr(
            self._scans_iter(False, False, True))
        return ForwardSlicer.slice_iter(scans_itr, k)

    def slice(self, key: slice) -> MultiScanSource:
        """Constructs a MultiScanSource matching the specificed slice"""
        from ouster.sdk.client.multi_sliced_scan_source import MultiSlicedScanSource
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        if k.step < 0:
            raise TypeError("slice() can't work with negative step")
        return MultiSlicedScanSource(self, k)

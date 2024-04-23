from typing import List, Optional, Tuple, Union

from ouster.sdk.client import LidarScan, first_valid_packet_ts
from ouster.sdk.client import ScansMulti     # type: ignore
from ouster.sdk.client.multi import collate_scans   # type: ignore
from ouster.sdk.util import (resolve_field_types, resolve_metadata_multi,
                             ForwardSlicer, progressbar)    # type: ignore
from .pcap_multi_packet_reader import PcapMultiPacketReader


class PcapScanSource(ScansMulti):
    """Implements MultiScanSource protocol for pcap files with multiple sensors."""

    def __init__(
        self,
        file_path: str,
        *,
        dt: int = 10**8,
        complete: bool = False,
        index: bool = False,
        cycle: bool = False,
        flags: bool = True,
        raw_headers: bool = False,
        raw_fields: bool = False,
        soft_id_check: bool = False,
        meta: Tuple[str, ...] = (),
        **_
    ) -> None:
        """
        Args:
            file_path: OSF filename as scans source
            dt: max time difference between scans in the collated scan (i.e.
                time period at which every new collated scan is released/cut),
                default is 0.1s.
            complete: set to True to only release complete scans
            index: if this flag is set to true an index will be built for the pcap
                file enabling len, index and slice operations on the scan source, if
                the flag is set to False indexing is skipped (default is False).
            cycle: repeat infinitely after iteration is finished (default is False)
            flags: when this option is set, the FLAGS field will be added to the list
                of fields of every scan, in case of dual returns FLAGS2 will also be
                appended (default is True).
        """

        self._source: Optional[PcapMultiPacketReader]
        self._source = None  # initialize the attribute so close works correctly if we fail out

        try:
            metadata_paths = list(meta)
            if not meta:
                metadata_paths = resolve_metadata_multi(file_path)

            if not metadata_paths:
                raise RuntimeError(
                    "Metadata jsons not found. Make sure that metadata json files "
                    "have common prefix with a PCAP file")

            # TODO: need a better way to save these
            self._metadata_paths = metadata_paths
            print(f"loading metadata from {metadata_paths}")

            self._source = PcapMultiPacketReader(file_path,
                                                 metadata_paths=metadata_paths,
                                                 index=index,
                                                 soft_id_check=soft_id_check)
        except Exception:
            self._source = None
            raise

        # generate the field types per sensor with flags/raw_fields if specified
        field_types = resolve_field_types(self._source.metadata,
                                          flags=flags,
                                          raw_headers=raw_headers,
                                          raw_fields=raw_fields)

        super().__init__(self._source, dt=dt, complete=complete,
                         cycle=cycle, fields=field_types)

        # TODO[IMPORTANT]: there is a bug with collate scans in which it always
        # skips the first frame
        def collate_scans_itr(scans_itr):
            return collate_scans(scans_itr, self.sensors_count,
                                 first_valid_packet_ts, dt=self._dt)

        if index:
            self._frame_offset = []
            pi = self._source._index    # type: ignore
            scans_itr = collate_scans_itr(self._scans_iter(True, False, False))
            # scans count in first source
            scans_count = len(pi.frame_id_indices[0])   # type: ignore
            for scan_idx, scans in enumerate(scans_itr):
                offsets = [pi.frame_id_indices[idx].get(    # type: ignore
                    scan.frame_id) for idx, scan in enumerate(scans) if scan]
                self._frame_offset.append(min([v for v in offsets if v]))
                progressbar(scan_idx, scans_count, "", "indexed")
            print("\nfinished building index")

    @property
    def scans_num(self) -> List[Optional[int]]:
        if not self.is_indexed:
            return [None] * self.sensors_count
        pi = self._source._index    # type: ignore
        return [pi.frame_count(i) for i in range(self.sensors_count)]   # type: ignore

    def __len__(self) -> int:
        if not self.is_indexed:
            raise TypeError("len is not supported on non-indexed source")
        return len(self._frame_offset)

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], List[List[Optional[LidarScan]]]]:

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
            return next(collate_scans(scans_itr, self.sensors_count,
                                      first_valid_packet_ts, dt=self._dt))

        if isinstance(key, slice):
            L = len(self)
            k = ForwardSlicer.normalize(key, L)
            count = k.stop - k.start
            if count <= 0:
                return []
            offset = self._frame_offset[k.start]
            self._source.seek(offset)   # type: ignore
            scans_itr = collate_scans(self._scans_iter(False, False, True),
                                      self.sensors_count,
                                      first_valid_packet_ts,
                                      dt=self._dt)
            result = [scan for idx, scan in ForwardSlicer.slice(
                enumerate(scans_itr), k) if idx < count]
            return result if k.step > 0 else list(reversed(result))

        raise TypeError(
            f"indices must be integer or slice, not {type(key).__name__}")

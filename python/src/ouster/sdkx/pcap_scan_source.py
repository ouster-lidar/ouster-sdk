from typing import List, Optional, Union

import copy
from ouster.sdk.util import resolve_metadata_multi
from .parsing import resolve_field_types    # type: ignore
from .multi import PcapMultiPacketReader, collate_scans, ScansMulti     # type: ignore
from .forward_slicer import ForwardSlicer
from ouster.client import LidarScan, first_valid_packet_ts


class PcapScanSource(ScansMulti):

    def __init__(
        self,
        file_path: str,
        *,
        complete: bool = False,
        index: bool = False,
        cycle: bool = False,
        flags: bool = False,
        raw_headers: bool = False,
        raw_fields: bool = False,
        _soft_id_check: bool = False,
        **_
    ) -> None:

        metadata_paths = resolve_metadata_multi(file_path)
        if not metadata_paths:
            raise RuntimeError(
                "Metadata jsons not found. Make sure that metadata json files "
                "have common prefix with a PCAP file")

        # TODO: need a better way to save these
        self._metadata_paths = metadata_paths

        source = PcapMultiPacketReader(file_path,
                                       metadata_paths=metadata_paths,
                                       index=index,
                                       _soft_id_check=_soft_id_check,
                                       _resolve_extrinsics=True)

        # print extrinsics if any were found
        for ext_source, m in zip(source.extrinsics_source,
                                 source._metadata):
            if ext_source:
                print(f"Found extrinsics for {m.sn} "
                      f"(from {ext_source}):\n{m.extrinsic}")

        # generate the field types per sensor with flags/raw_fields if specified
        field_types = resolve_field_types(source.metadata,
                                          flags=flags,
                                          raw_headers=raw_headers,
                                          raw_fields=raw_fields)

        # set sensor names as idx in the source
        for idx, m in enumerate(source.metadata):
            source.metadata[idx].hostname = f"sensoridx: {idx}"

        super().__init__(source, fields=field_types, complete=complete, cycle=cycle)

        # TODO[IMPORTANT]: there is a bug with collate scans in which it always
        # skips the first frame
        def collate_scans_itr(scans_itr):
            return collate_scans(scans_itr, self.sensors_count,
                                 first_valid_packet_ts, dt=self._dt)

        if index:
            self._frame_offset = []
            pi = self._source._index
            scans_itr = collate_scans_itr(self._async_iter(True, False))
            for scans in scans_itr:
                offsets = [pi.frame_id_indices[idx].get(
                    scan.frame_id) for idx, scan in enumerate(scans) if scan]
                self._frame_offset.append(min([v for v in offsets if v]))

    @property
    def scans_num(self) -> List[int]:
        if not self.is_indexed:
            return [0] * self.sensors_count
        pi = self._source._index
        return [pi.frame_count(i) for i in range(self.sensors_count)]

    def __len__(self) -> int:
        return len(self._frame_offset) if self.is_indexed else 0

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], List[List[Optional[LidarScan]]]]:

        if not self.is_indexed:
            raise RuntimeError(
                "can not invoke __getitem__ on non-indexed source")

        if isinstance(key, int):
            L = len(self)
            if key < 0:
                key += L
            if key < 0 or key >= L:
                raise IndexError("index is out of range")
            offset = self._frame_offset[key]
            self._source.seek(offset)
            scans_itr = self._async_iter(False, False)
            return next(collate_scans(scans_itr, self.sensors_count,
                                      first_valid_packet_ts, dt=self._dt))

        if isinstance(key, slice):
            L = len(self)
            k = ForwardSlicer.normalize(key, L)
            count = k.stop - k.start
            if count <= 0:
                return []
            offset = self._frame_offset[k.start]
            self._source.seek(offset)
            scans_itr = collate_scans(self._async_iter(False, False),
                                      self.sensors_count,
                                      first_valid_packet_ts,
                                      dt=self._dt)
            result = [msg for idx, msg in ForwardSlicer.slice(
                enumerate([copy.deepcopy(scans) for scans in scans_itr]), k) if idx < count]
            return result if k.step > 0 else list(reversed(result))

        raise TypeError(
            f"indices must be integer or slice, not {type(key).__name__}")

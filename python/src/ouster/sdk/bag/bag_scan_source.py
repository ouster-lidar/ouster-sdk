from typing import Iterator, List, Optional, Union
import numpy as np

from ouster.sdk.core import LidarScan
from ouster.sdk.core import SensorInfo, ScanBatcher, LidarPacket
from ouster.sdk.util import resolve_field_types  # type: ignore
from .bag_packet_source import BagPacketSource

from ouster.sdk._bindings.client import ScanSource


class BagScanSource(ScanSource):
    """Implements ScanSource protocol for pcap files with multiple sensors."""

    _source: BagPacketSource

    def __init__(
        self,
        file_path: Union[str, List[str]],
        *,
        extrinsics_file: Optional[str] = None,
        raw_headers: bool = False,
        raw_fields: bool = False,
        soft_id_check: bool = False,
        meta: Optional[List[str]] = None,
        field_names: Optional[List[str]] = None,
        extrinsics: List[np.ndarray] = [],
        **kwargs
    ) -> None:
        """
        Args:
            file_path: OSF filename as scans source
            raw_headers: if True, include raw headers in decoded LidarScans
            raw_fields: if True, include raw fields in decoded LidarScans
            soft_id_check: if True, don't skip packets on init_id/serial_num mismatch
            meta: optional list of metadata files to load, if not provided metadata
                is loaded from the bag instead
            field_names: list of fields to decode into a LidarScan, if not provided
                decodes all default fields
        """
        ScanSource.__init__(self)

        # initialize the attribute so close works correctly if we fail out
        self._source = None  # type: ignore
        try:
            self._source = BagPacketSource(file_path, soft_id_check=soft_id_check,
                                           meta=meta, extrinsics_file=extrinsics_file,
                                           extrinsics=extrinsics)
        except Exception:
            self._source = None  # type: ignore
            raise

        # generate the field types per sensor with flags/raw_fields if specified
        self._field_types = resolve_field_types(self._source.sensor_info,
                                          raw_headers=raw_headers,
                                          raw_fields=raw_fields,
                                          field_names=field_names)

    @property
    def is_live(self) -> bool:
        return False

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return self._source.sensor_info

    @property
    def id_error_count(self) -> int:
        return self._source.id_error_count  # type: ignore

    @property
    def size_error_count(self) -> int:
        return self._source.size_error_count  # type: ignore

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:
        batchers = []
        scans: List[Optional[LidarScan]] = []
        for m in self.sensor_info:
            batchers.append(ScanBatcher(m))
            scans.append(None)
        for idx, packet in self._source:
            if isinstance(packet, LidarPacket):
                scan = scans[idx]
                if not scan:
                    scan = LidarScan(self._source.sensor_info[idx], self._field_types[idx])
                    scans[idx] = scan
                if batchers[idx](packet, scan):
                    yield [scan]
                    scans[idx] = None
        # yield any remaining scans
        # todo maybe do this in time order
        for idx, scan in enumerate(scans):
            if scan is not None:
                yield [scan]
                scans[idx] = None

    def close(self):
        return

    def __len__(self) -> int:
        raise TypeError("len is not supported on non-indexed source")

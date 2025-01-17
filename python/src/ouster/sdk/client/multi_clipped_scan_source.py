from typing import Optional, Iterator, Union, List
from ouster.sdk.client.multi_scan_source import MultiScanSource
from ouster.sdk.client import SensorInfo, LidarScan
from ouster.sdk.client.data import FieldTypes
from .scan_ops import clip


class MultiClippedScanSource(MultiScanSource):
    """
    limits the values of the specified set of fields to within the range = [lower, upper], any value
    that exceeds this range is replaced by zero.
    """

    def __init__(self, scan_source: MultiScanSource, fields: List[str], lower: int, upper: int) -> None:
        # Make sure current requirements are met
        if upper < lower:
            raise ValueError("`upper` value can't be less than `lower`")

        self._scan_source = scan_source
        self._fields = fields
        self._lower = lower
        self._upper = upper

    @property
    def sensors_count(self) -> int:
        return self._scan_source.sensors_count

    @property
    def metadata(self) -> List[SensorInfo]:
        return self._scan_source.metadata

    @property
    def is_live(self) -> bool:
        return self._scan_source.is_live

    @property
    def is_seekable(self) -> bool:
        return self._scan_source.is_seekable

    @property
    def is_indexed(self) -> bool:
        return self._scan_source.is_indexed

    @property
    def field_types(self) -> List[FieldTypes]:
        return self._scan_source.field_types

    @property
    def fields(self) -> List[List[str]]:
        return self._scan_source.fields

    @property
    def scans_num(self) -> List[Optional[int]]:
        return self._scan_source.scans_num

    def __len__(self) -> int:
        return len(self._scan_source)

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:

        def clip_with_copy(scan, fields, lower, upper):
            cpy = LidarScan(scan)
            clip(cpy, fields, lower, upper)
            return cpy

        for scans in self._scan_source.__iter__():
            yield [clip_with_copy(scan, self._fields, self._lower, self._upper)
                   if scan else None for scan in scans]

    def _seek(self, key: int) -> None:
        self._scan_source._seek(key)

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], MultiScanSource]:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()

    def _slice_iter(self, key: slice) -> Iterator[List[Optional[LidarScan]]]:
        from itertools import islice
        from ouster.sdk.util.forward_slicer import ForwardSlicer
        # here we are slicing the current slice
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        return islice(iter(self), k.start, k.stop, k.step)

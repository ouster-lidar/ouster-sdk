from typing import Optional, Iterator, Union, List, overload
from ouster.sdk._bindings.client import ScanSource
from ouster.sdk.core import SensorInfo, LidarScan
from .scan_ops import clip


class ClippedScanSource(ScanSource):
    """
    limits the values of the specified set of fields to within the range = [lower, upper], any value
    that exceeds this range is replaced by zero.
    """

    def __init__(self, scan_source: ScanSource, fields: List[str], lower: int, upper: int) -> None:
        ScanSource.__init__(self)
        # Make sure current requirements are met
        if upper < lower:
            raise ValueError("`upper` value can't be less than `lower`")

        self._scan_source = scan_source
        self._fields = fields
        self._lower = lower
        self._upper = upper

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return self._scan_source.sensor_info

    @property
    def is_live(self) -> bool:
        return self._scan_source.is_live

    @property
    def is_indexed(self) -> bool:
        return self._scan_source.is_indexed

    @property
    def scans_num(self) -> List[int]:
        return self._scan_source.scans_num

    def __len__(self) -> int:
        return len(self._scan_source)

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:

        def clip_with_copy(scan, fields, lower, upper):
            cpy = LidarScan(scan)
            clip(cpy, fields, lower, upper)
            return cpy

        for scans in self._scan_source.__iter__():
            print("clip", self._upper, self._lower)
            yield [clip_with_copy(scan, self._fields, self._lower, self._upper)
                   if scan else None for scan in scans]

    @overload
    def __getitem__(self, key: int) -> List[LidarScan]:
        ...

    @overload
    def __getitem__(self, key: slice) -> ScanSource:
        ...

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[LidarScan], ScanSource]:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()


def _clip_fn(src: ScanSource, fields: List[str], lower: int, upper: int):
    """
    limits the values of the specified set of fields to within the range = [lower, upper], any value
    that exceeds this range is replaced by zero.
    """
    return ClippedScanSource(src, fields, lower, upper)


ScanSource.clip = _clip_fn  # type: ignore

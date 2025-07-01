from typing import List, Optional, Iterator, Union, overload
import numpy as np
from ouster.sdk._bindings.client import ScanSource
from ouster.sdk.core import SensorInfo, LidarScan
from .scan_ops import mask
from ouster.sdk.core import destagger


class MaskedScanSource(ScanSource):

    def __init__(self, scan_source: ScanSource, fields: List[str],
                 masks: List[Optional[np.ndarray]]) -> None:
        ScanSource.__init__(self)
        # Make sure current requirements are met
        if len(scan_source.sensor_info) != len(masks):
            raise ValueError("the number of masks should match the count of sensors")

        self._scan_source = scan_source
        self._fields = fields
        self._masks = [destagger(si, mask, inverse=True) if mask is not None else None
                       for si, mask in zip(scan_source.sensor_info, masks)]

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
        for scans in self._scan_source.__iter__():
            result = []
            for idx, scan in enumerate(scans):
                if scan:
                    cpy = LidarScan(scan)
                    if self._masks[idx] is not None:
                        mask(cpy, self._fields, self._masks[idx])   # type: ignore
                    result.append(cpy)
                else:
                    result.append(None)     # type: ignore
            yield result    # type: ignore

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


def _mask_fn(src: ScanSource, fields: List[str], masks: List[Optional[np.ndarray]]):
    return MaskedScanSource(src, fields, masks)


ScanSource.mask = _mask_fn  # type: ignore

from typing import List, Optional, Iterator, Union
import numpy as np
from ouster.sdk.client.multi_scan_source import MultiScanSource
from ouster.sdk.client import SensorInfo, LidarScan
from ouster.sdk.client.data import FieldTypes
from .scan_ops import mask
from ouster.sdk.client import destagger


class MultiMaskedScanSource(MultiScanSource):

    def __init__(self, scan_source: MultiScanSource, fields: List[str],
                 masks: List[Optional[np.ndarray]]) -> None:

        # Make sure current requirements are met
        if scan_source.sensors_count != len(masks):
            raise ValueError("the number of masks should match the count of sensors")

        self._scan_source = scan_source
        self._fields = fields
        self._masks = [destagger(si, mask, inverse=True) if mask is not None else None
                       for si, mask in zip(scan_source.metadata, masks)]

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

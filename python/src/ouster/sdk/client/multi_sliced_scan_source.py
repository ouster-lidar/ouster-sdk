from typing import Optional, Iterator, Union, List
from itertools import islice
from ouster.sdk.client.scan_source import ScanSource
from ouster.sdk.client.multi_scan_source import MultiScanSource
from ouster.sdk._bindings.client import SensorInfo, LidarScan
from ouster.sdk.client.data import FieldTypes
from ouster.sdk.util.forward_slicer import ForwardSlicer


class MultiSlicedScanSource(MultiScanSource):

    def __init__(self, scan_source: MultiScanSource, key: slice) -> None:

        # Make sure current requirements are met
        if key.start is None or key.stop is None or key.step is None:
            raise TypeError("can't work with non-normalized key(slice)")
        if key.start < 0 or key.stop < 0 or key.step < 0:
            raise TypeError("can't work with negative key(slice)")
        if key.stop > len(scan_source):
            raise TypeError("range of key(slice) can't exceed scan_source")

        self._scan_source = scan_source
        self._key = key

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
        # TODO: this is not a true implementation of scans_num property in
        # the sliced scan source. Without iterating over each source it
        # wouldn't be completely possible to get real values. so for the time
        # being we are using the main length property
        return [len(self)] * self.sensors_count

    def __len__(self) -> int:
        k = self._key
        return (k.stop - k.start + k.step - 1) // k.step

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:
        return self._scan_source._slice_iter(self._key)

    def _seek(self, key: int) -> None:
        self._scan_source._seek(key)

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], MultiScanSource]:
        if isinstance(key, int):
            L = len(self)
            if key < 0:
                key += L
            if key < 0 or key >= L:
                raise IndexError("index is out of range")
            # TODO: This is functionally correct but can be in-efficient
            # when the slice range is rather large, the operator can be
            # improved by relying on the original scan_source idx but that
            # requires a major revision of current approach.
            # https://ouster.atlassian.net/browse/FLEETSW-6251
            return next(islice(iter(self), key, key + 1, 1))
        if isinstance(key, slice):
            return self.slice(key)
        raise TypeError(
            f"indices must be integer or slice, not {type(key).__name__}")

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()

    def single_source(self, stream_idx: int) -> ScanSource:
        from ouster.sdk.client.scan_source_adapter import ScanSourceAdapter
        return ScanSourceAdapter(self, stream_idx)

    def _slice_iter(self, key: slice) -> Iterator[List[Optional[LidarScan]]]:
        # here we are slicing the current slice
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        return islice(iter(self), k.start, k.stop, k.step)

    def slice(self, key: slice) -> MultiScanSource:
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        if k.step < 0:
            raise TypeError("slice() can't work with negative step")
        return MultiSlicedScanSource(self, k)

from typing import Iterator, List, Union, Optional, cast

from ouster.sdk._bindings.client import SensorInfo, LidarScan
from ouster.sdk.util.forward_slicer import ForwardSlicer
from .scan_source import ScanSource
from .multi_scan_source import MultiScanSource
from .data import FieldTypes

# TODO: since we have more than one adapter we out to rename this class
# to something specific


class ScanSourceAdapter(ScanSource):
    """Represents only data stream from one stream."""

    def __init__(self, scan_source: MultiScanSource, stream_idx: int = 0) -> None:
        if stream_idx < 0 or stream_idx >= scan_source.sensors_count:
            raise ValueError(
                f"stream_idx needs to be within the range [0, {scan_source.sensors_count})")
        self._scan_source = scan_source
        self._stream_idx = stream_idx

    @property
    def metadata(self) -> SensorInfo:
        """A list of Metadata objects associated with the scan streams."""
        return self._scan_source.metadata[self._stream_idx]

    @property
    def is_live(self) -> bool:
        """True if data obtained from the RUNNING sensor or as a stream from the socket

        Returns:
            True if data obtained from the RUNNING sensor or as a stream from the socket
            False if data is read from a stored media. Restarting an ``iter()`` means that
                    the data can be read again.
        """
        return self._scan_source.is_live

    @property
    def is_seekable(self) -> bool:
        """True for non-live sources, This property can be True regardless of scan source being indexed or not.
        """
        return self._scan_source.is_seekable

    @property
    def is_indexed(self) -> bool:
        """True for IndexedPcap and OSF scan sources, this property tells users whether the underlying source
        allows for random access of scans, see __getitem__.
        """
        return self._scan_source.is_indexed

    @property
    def field_types(self) -> FieldTypes:
        """Field types are present in the LidarScan objects on read from iterator"""
        return self._scan_source.field_types[self._stream_idx]

    @property
    def fields(self) -> List[str]:
        return self._scan_source.fields[self._stream_idx]

    @property
    def scans_num(self) -> Optional[int]:
        """Number of scans available, in case of a live sensor or non-indexable scan source this method
         returns None"""
        return self._scan_source.scans_num[self._stream_idx]

    def __len__(self) -> int:
        if self.scans_num is None:
            raise TypeError(
                "len is not supported on live or non-indexed sources")
        return self.scans_num

    # NOTE: we need to consider a case without collation of scans
    def __iter__(self) -> Iterator[LidarScan]:

        def _stream_iter(source: MultiScanSource) -> Iterator[LidarScan]:
            for ls in source:
                s = ls[self._stream_idx]
                if s is not None:
                    yield s

        return _stream_iter(self._scan_source)

    def _seek(self, key: int) -> None:
        """seek/jump to a specific item within the list of LidarScan objects that this particular scan
        source has access to"""
        raise NotImplementedError

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[Optional[LidarScan], ScanSource]:
        """Indexed access and slices support"""
        if isinstance(key, int):
            scans_list = self._scan_source[key]
            scans_list = cast(List[Optional[LidarScan]], scans_list)
            return scans_list[self._stream_idx]
        elif isinstance(key, slice):
            return self.slice(key)
        raise TypeError(
            f"indices must be integer or slice, not {type(key).__name__}")

    def close(self) -> None:
        """Release the underlying resource, if any."""
        pass

    def __del__(self) -> None:
        """Automatic release of any underlying resource."""
        self.close()

    def _slice_iter(self, key: slice) -> Iterator[Optional[LidarScan]]:
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        count = k.stop - k.start
        if count <= 0:
            return iter(())
        return ForwardSlicer.slice_iter(iter(self), k)

    def slice(self, key: slice) -> ScanSource:
        # NOTE: rather than creating a SlicedScanSource use the combination
        # of two decorators to achieve the same functionality
        from ouster.sdk.client.multi_sliced_scan_source import MultiSlicedScanSource
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        if k.step < 0:
            raise TypeError("slice() can't work with negative step")
        sliced = MultiSlicedScanSource(self._scan_source, k)        # type: ignore
        scan_source = ScanSourceAdapter(sliced, self._stream_idx)   # type: ignore
        return cast(ScanSource, scan_source)

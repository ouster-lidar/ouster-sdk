from typing import Iterator, List, Union, Optional
import typing
from .scan_source import ScanSource
from .multi_scan_source import MultiScanSource
from ._client import SensorInfo, LidarScan
from .data import FieldTypes


class ScanSourceAdapter(ScanSource):
    """Represents only data stream from one stream."""

    def __init__(self, scan_source: MultiScanSource, stream_idx: int = 0) -> None:
        if stream_idx < 0 or stream_idx >= scan_source.sensors_count:
            raise ValueError(f"stream_idx needs to be within the range [0, {scan_source.sensors_count})")
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
    def fields(self) -> FieldTypes:
        """Field types are present in the LidarScan objects on read from iterator"""
        return self._scan_source.fields[self._stream_idx]

    @property
    def scans_num(self) -> Optional[int]:
        """Number of scans available, in case of a live sensor or non-indexable scan source this method
         returns None"""
        return self._scan_source.scans_num[self._stream_idx]

    def __len__(self) -> int:
        if self.scans_num is None:
            raise TypeError("len is not supported on live or non-indexed sources")
        return self.scans_num

    # NOTE: we need to consider a case without collation of scans
    def __iter__(self) -> Iterator[Optional[LidarScan]]:

        def _stream_iter(source: MultiScanSource) -> Iterator[Optional[LidarScan]]:
            for ls in source:
                yield ls[self._stream_idx]

        return _stream_iter(self._scan_source)

    def _seek(self, key: int) -> None:
        """seek/jump to a specific item within the list of LidarScan objects that this particular scan
        source has access to"""
        raise NotImplementedError

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[Optional[LidarScan], List[Optional[LidarScan]]]:
        """Indexed access and slices support"""
        if isinstance(key, int):
            return self._scan_source[key][self._stream_idx]
        elif isinstance(key, slice):
            scans_list = self._scan_source[key]
            scans_list = typing.cast(List[List[Optional[LidarScan]]], scans_list)
            return [ls[self._stream_idx] for ls in scans_list] if scans_list else None
        raise TypeError(
            f"indices must be integer or slice, not {type(key).__name__}")

    # TODO: should this actually the parent scan source? any object why not
    def close(self) -> None:
        """Release the underlying resource, if any."""
        self._scan_source.close()

    def __del__(self) -> None:
        """Automatic release of any underlying resource."""
        self.close()

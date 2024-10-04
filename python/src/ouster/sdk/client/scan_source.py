from typing import Iterator, Union, Optional, List
from typing_extensions import Protocol
from ouster.sdk._bindings.client import SensorInfo, LidarScan, FieldType


class ScanSource(Protocol):
    """Represents only data stream from one stream."""

    @property
    def metadata(self) -> SensorInfo:
        """A 'SensorInfo' object associated with the scan streams."""
        ...

    @property
    def is_live(self) -> bool:
        """True if data obtained from the RUNNING sensor or as a stream from the socket

        Returns:
            True if data obtained from the RUNNING sensor or as a stream from the socket
            False if data is read from a stored media. Restarting an ``iter()`` means that
                    the data can be read again.
        """
        ...

    @property
    def is_seekable(self) -> bool:
        """True for non-live sources, This property can be True regardless of scan source being indexed or not.
        """
        ...

    @property
    def is_indexed(self) -> bool:
        """True for IndexedPcap and OSF scan sources, this property tells users whether the underlying source
        allows for random access of scans, see __getitem__.
        """
        ...

    @property
    def field_types(self) -> List[FieldType]:
        """Field types are present in the LidarScan objects on read from iterator"""
        ...

    @property
    def fields(self) -> List[str]:
        """Fields are present in the LidarScan objects on read from iterator"""
        ...

    @property
    def scans_num(self) -> Optional[int]:
        """Number of scans available, in case of a live sensor or non-indexable scan source
         this method returns None"""
        ...

    def __len__(self) -> int:
        """Number of scans available, in case of a live sensor or non-indexable scan source this method
         throws a TypeError"""
        ...

    def __iter__(self) -> Iterator[LidarScan]:
        ...

    def _seek(self, key: int) -> None:
        """seek/jump to a specific item within the list of LidarScan objects that this particular scan
        source has access to"""
        ...

    # NOTE: based on the underlying implemention the return type is
    # Optional[LidarScan] since MultiScanSource returns collate scans by default.
    # This can be solved by provide a method that gives access to uncollated scans
    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[Optional[LidarScan], 'ScanSource']:
        """Indexed access and slices support"""
        ...

    def close(self) -> None:
        """Manually release any underlying resource."""
        ...

    def __del__(self) -> None:
        """Automatic release of any underlying resource."""
        ...

    def _slice_iter(self, key: slice) -> Iterator[Optional[LidarScan]]:
        ...

    def slice(self, key: slice) -> 'ScanSource':
        """Constructs a ScanSource matching the specificed slice"""
        ...

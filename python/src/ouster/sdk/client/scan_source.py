from typing import Iterator, List, Union, Optional
from typing_extensions import Protocol
from ._client import SensorInfo, LidarScan
from .data import FieldTypes


class ScanSource(Protocol):
    """Represents only data stream from one stream."""

    @property
    def metadata(self) -> SensorInfo:
        """A list of Metadata objects associated with the scan streams."""
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
    def fields(self) -> FieldTypes:
        """Field types are present in the LidarScan objects on read from iterator"""
        ...

    @property
    def scans_num(self) -> Optional[int]:
        """Number of scans available, in case of a live sensor or non-indexable scan source this method
         returns None"""
        ...

    def __len__(self) -> int:
        """Number of scans available, in case of a live sensor or non-indexable scan source this method
         throws a TypeError"""
        ...

    # NOTE: based on the underlying implemention the return type is
    # Optional[LidarScan] since MultiScanSource returns collate scans by default.
    # This can be solved by  provide a method that gives access to uncollated scans
    def __iter__(self) -> Iterator[Optional[LidarScan]]:
        ...

    def _seek(self, key: int) -> None:
        """seek/jump to a specific item within the list of LidarScan objects that this particular scan
        source has access to"""
        ...

    # NOTE: based on the underlying implemention the return type is
    # Optional[LidarScan] since MultiScanSource returns collate scans by default.
    # This can be solved by provide a method that gives access to uncollated scans
    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[Optional[LidarScan], List[Optional[LidarScan]]]:
        """Indexed access and slices support"""
        ...

    def close(self) -> None:
        """Release the underlying resource, if any."""
        ...

    def __del__(self) -> None:
        """Automatic release of any underlying resource."""
        ...

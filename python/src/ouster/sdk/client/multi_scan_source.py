from typing import Iterator, List, Optional, Union
from typing_extensions import Protocol
from ._client import SensorInfo, LidarScan
from .data import FieldTypes
from .scan_source import ScanSource


class MultiScanSource(Protocol):
    """Represents only data stream from more than one source."""

    @property
    def sensors_count(self) -> int:
        """Number of individual scan streams that this scan source holds."""
        ...

    @property
    def metadata(self) -> List[SensorInfo]:
        """A list of Metadata objects associated with every scan streams."""
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

    # Pavlo/Oct19: Optional field, that is currently available only for OSFs and IndexedPcap, and
    # None everywhere else.
    # UN/Nov21: I don't understand why this only available in OSF and IndexedPcap source
    @property
    def fields(self) -> List[FieldTypes]:
        """Field types are present in the LidarScan objects on read from iterator"""
        ...

    @property
    def scans_num(self) -> List[Optional[int]]:
        """Number of scans available, in case of a live sensor or non-indexable scan source this method
         returns a None for that stream"""
        ...

    def __len__(self) -> int:
        """returns the number of scans containe with the scan_source, in case scan_source holds more than
        one stream then this would measure the number of collated scans across the streams
        in the case of a live sensor or non-indexable scan source this method throws a TypeError
        """
        ...

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:
        ...

    def _seek(self, key: int) -> None:
        """seek/jump to a specific item within the list of LidarScan objects that this particular scan
        source has access to"""
        ...

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], List[List[Optional[LidarScan]]]]:
        """Indexed access and slices support"""
        ...

    def close(self) -> None:
        """Manually release any underlying resource."""
        ...

    def __del__(self) -> None:
        """Automatic release of any underlying resource."""
        ...

    def single_source(self, stream_idx: int) -> ScanSource:
        ...

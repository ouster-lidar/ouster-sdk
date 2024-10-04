from typing import Iterator, List, Tuple, Optional
from ouster.sdk.client import PacketMultiSource, Packet, SensorInfo

class BagPacketSource(PacketMultiSource):
    def __init__(self,
                 bag_path: str,
                 *,
                 meta: Optional[List[str]] = None,
                 soft_id_check: bool = False):
        ...

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        ...

    @property
    def closed(self) -> bool:
        ...

    def close(self) -> None:
        ...

    @property
    def is_live(self) -> bool:
        ...

    @property
    def is_seekable(self) -> bool:
        ...

    @property
    def is_indexed(self) -> bool:
        ...
    
    @property
    def metadata(self) -> List[SensorInfo]:
        ...

    def restart(self) -> None:
        ...

    @property
    def id_error_count(self) -> int:
        ...

    @property
    def size_error_count(self) -> int:
        ...

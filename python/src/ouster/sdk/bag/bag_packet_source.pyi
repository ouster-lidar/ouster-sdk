from typing import List, Optional, Union
from ouster.sdk.core import PacketSource
import numpy as np

class BagPacketSource(PacketSource):
    def __init__(self,
                 bag_path: Union[str, List[str]],
                 *,
                 extrinsics_file: Optional[str] = None,
                 raw_headers: bool = False,
                 raw_fields: bool = False,
                 soft_id_check: bool = False,
                 meta: Optional[List[str]] = None,
                 field_names: Optional[List[str]] = None,
                 extrinsics: List[np.ndarray] = []):
        ...

    @property
    def closed(self) -> bool:
        ...

    @property
    def id_error_count(self) -> int:
        ...

    @property
    def size_error_count(self) -> int:
        ...

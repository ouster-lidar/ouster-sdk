from typing import Iterator, List, Optional, Union
import numpy as np
import copy

from ouster.sdk.core import LidarFrame, FrameSet
from ouster.sdk.core import SensorInfo, FrameBatcher
from ouster.sdk.util import resolve_field_types  # type: ignore
from .bag_packet_source import BagPacketSource

from ouster.sdk._bindings.client import FrameSetSource


class BagFrameSetSource(FrameSetSource):
    """Implements FrameSetSource protocol for pcap files with multiple sensors."""

    _source: BagPacketSource

    def __init__(
        self,
        file_path: Union[str, List[str]],
        *,
        extrinsics_file: Optional[str] = None,
        raw_headers: bool = False,
        raw_fields: bool = False,
        soft_id_check: bool = False,
        meta: Optional[List[str]] = None,
        field_names: Optional[List[str]] = None,
        extrinsics: List[np.ndarray] = [],
        **kwargs
    ) -> None:
        """
        Args:
            file_path: OSF filename as frames source
            raw_headers: if True, include raw headers in decoded LidarFrames
            raw_fields: if True, include raw fields in decoded LidarFrames
            soft_id_check: if True, don't skip packets on init_id/serial_num mismatch
            meta: optional list of metadata files to load, if not provided metadata
                is loaded from the bag instead
            field_names: list of fields to decode into a LidarFrame, if not provided
                decodes all default fields
        """
        FrameSetSource.__init__(self)

        # initialize the attribute so close works correctly if we fail out
        self._source = None  # type: ignore
        try:
            self._source = BagPacketSource(file_path, soft_id_check=soft_id_check,
                                           meta=meta, extrinsics_file=extrinsics_file,
                                           extrinsics=extrinsics)
        except Exception:
            self._source = None  # type: ignore
            raise

        # generate the field types per sensor with flags/raw_fields if specified
        self._field_types = resolve_field_types(self._source.sensor_info,
                                          raw_headers=raw_headers,
                                          raw_fields=raw_fields,
                                          field_names=field_names)

        # copy sensor info for decoding so it cant get messed with
        self._orig_sensor_info = [copy.copy(si) for si in self._source.sensor_info]

    @property
    def is_live(self) -> bool:
        return False

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return self._source.sensor_info

    @property
    def id_error_count(self) -> int:
        return self._source.id_error_count  # type: ignore

    @property
    def size_error_count(self) -> int:
        return self._source.size_error_count  # type: ignore

    def __length_hint__(self):
        return self._source.__length_hint__()

    def __iter__(self) -> Iterator[FrameSet]:
        batchers = []
        frames: List[Optional[LidarFrame]] = []
        for m in self._orig_sensor_info:
            batchers.append(FrameBatcher(m))
            frames.append(None)
        for idx, packet in self._source:
            frame = frames[idx]
            if not frame:
                frame = LidarFrame(self._orig_sensor_info[idx], self._field_types[idx])
                frames[idx] = frame
            if batchers[idx].batch(packet, frame):
                frame.sensor_info = self._source.sensor_info[idx]
                yield FrameSet([frame])
                frames[idx] = None
        # yield any remaining frames
        # todo maybe do this in time order
        for idx, frame in enumerate(frames):
            if frame is not None:
                # dont output frames that got no packets
                if batchers[idx].batched_packets() == 0:
                    frames[idx] = None
                    continue
                frame.sensor_info = self._source.sensor_info[idx]
                yield FrameSet([frame])
                frames[idx] = None

    def close(self):
        return

    def __len__(self) -> int:
        raise TypeError("len is not supported on non-indexed source")

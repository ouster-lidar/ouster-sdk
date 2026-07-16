from typing import List, Optional, Iterator
import numpy as np
from ouster.sdk._bindings.client import FrameSetSource
from ouster.sdk.core import SensorInfo, LidarFrame, FrameSet
from .frame_ops import mask
from ouster.sdk.core import destagger


class MaskedFrameSetSource(FrameSetSource):

    def __init__(self, frame_set_source: FrameSetSource, fields: List[str],
                 masks: List[Optional[np.ndarray]]) -> None:
        FrameSetSource.__init__(self)
        # Make sure current requirements are met
        if len(frame_set_source.sensor_info) != len(masks):
            raise ValueError("the number of masks should match the count of sensors")

        self._frame_set_source = frame_set_source
        self._fields = fields
        self._masks = [destagger(si, mask, inverse=True) if mask is not None else None
                       for si, mask in zip(frame_set_source.sensor_info, masks)]

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return self._frame_set_source.sensor_info

    @property
    def is_live(self) -> bool:
        return self._frame_set_source.is_live

    @property
    def is_indexed(self) -> bool:
        return self._frame_set_source.is_indexed

    @property
    def frames_num(self) -> List[int]:
        return self._frame_set_source.frames_num

    def __len__(self) -> int:
        return len(self._frame_set_source)

    def __length_hint__(self) -> int:
        return self._frame_set_source.__length_hint__()

    def __iter__(self) -> Iterator[FrameSet]:
        for frames in self._frame_set_source.__iter__():

            result: List[Optional[LidarFrame]] = []
            for idx, frame in enumerate(frames):
                if frame:
                    cpy = LidarFrame(frame)
                    if self._masks[idx] is not None:
                        mask(cpy, self._fields, self._masks[idx])   # type: ignore
                    result.append(cpy)
                else:
                    result.append(None)
            yield FrameSet(result)

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()


def _mask_fn(self: FrameSetSource, fields: List[str], masks: List[Optional[np.ndarray]]):
    return MaskedFrameSetSource(self, fields, masks)


FrameSetSource.mask = _mask_fn  # type: ignore

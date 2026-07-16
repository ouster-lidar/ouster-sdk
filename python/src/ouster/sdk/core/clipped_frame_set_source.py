from typing import Iterator, List
from ouster.sdk._bindings.client import FrameSetSource
from ouster.sdk.core import SensorInfo, LidarFrame, FrameSet
from .frame_ops import clip


class ClippedFrameSetSource(FrameSetSource):
    """
    limits the values of the specified set of fields to within the range = [lower, upper], any value
    that exceeds this range is replaced by zero.
    """

    def __init__(self, frame_set_source: FrameSetSource, fields: List[str], lower: int, upper: int) -> None:
        FrameSetSource.__init__(self)
        # Make sure current requirements are met
        if upper < lower:
            raise ValueError("`upper` value can't be less than `lower`")

        self._frame_set_source = frame_set_source
        self._fields = fields
        self._lower = lower
        self._upper = upper

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

        def clip_with_copy(frame, fields, lower, upper):
            cpy = LidarFrame(frame)
            clip(cpy, fields, lower, upper)
            return cpy

        for frames in self._frame_set_source.__iter__():
            yield FrameSet([clip_with_copy(frame, self._fields, self._lower, self._upper)
                   if frame else None for frame in frames])

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()


def _clip_fn(self: FrameSetSource, fields: List[str], lower: int, upper: int):
    """
    limits the values of the specified set of fields to within the range = [lower, upper], any value
    that exceeds this range is replaced by zero.
    """
    return ClippedFrameSetSource(self, fields, lower, upper)


FrameSetSource.clip = _clip_fn  # type: ignore

from typing import Optional, Iterator, List
from ouster.sdk._bindings.client import FrameSetSource
from ouster.sdk.core import SensorInfo, LidarFrame, FrameSet
from ouster.sdk.core.frame_ops import reduce_by_factor, reduce_by_factor_metadata


class ReducedFrameSetSource(FrameSetSource):
    """
    Takes a regular FrameSetSource and reduces the beams count to the specified values.
    """

    def __init__(self, frame_set_source: FrameSetSource, beams: List[int]) -> None:
        FrameSetSource.__init__(self)
        if len(frame_set_source.sensor_info) != len(beams):
            raise ValueError("beams should match the count of sensors")

        factor_list = []
        for b, m in zip(beams, frame_set_source.sensor_info):
            f = m.format.pixels_per_column / b
            if not (f.is_integer() and f > 0):
                raise ValueError(f"beams {b} must be divisor of {m.format.pixels_per_column}")
            factor_list.append(int(f))

        self._frame_set_source = frame_set_source
        self._factors = factor_list
        self._metadata = self._generate_metadata(factor_list)
        self._input_sensor_info = self._frame_set_source.sensor_info

    def _generate_metadata(self, factor_list):
        return [reduce_by_factor_metadata(m, f)
                for m, f in zip(self._frame_set_source.sensor_info, factor_list)]

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return self._metadata

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
        def reduce_frame(frame, factor, sensor_info):
            out = reduce_by_factor(frame, factor)
            out.sensor_info = sensor_info
            return out

        for frames in self._frame_set_source:
            out: List[Optional[LidarFrame]] = []
            for frame in frames:
                for idx, info in enumerate(self._input_sensor_info):
                    if frame is not None and info is frame.sensor_info:
                        break
                if frame is None:
                    out.append(None)
                else:
                    out.append(reduce_frame(frame, self._factors[idx], self._metadata[idx]))
            yield FrameSet(out)

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()


def _reduce_fn(self: FrameSetSource, beams: List[int]) -> FrameSetSource:
    """
    Takes a regular FrameSetSource and reduces the beams count to the specified values.
    """
    return ReducedFrameSetSource(self, beams)


# allow assigning this
FrameSetSource.reduce = _reduce_fn  # type: ignore

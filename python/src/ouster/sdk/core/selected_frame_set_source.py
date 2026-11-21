from typing import cast, Optional, Iterator, List
from ouster.sdk._bindings.client import FrameSetSource
from ouster.sdk.core import SensorInfo, LidarInfo, Frame, LidarFrame, FrameSet
from ouster.sdk.core.frame_ops import select_by_index, select_by_index_metadata


class SelectedFrameSetSource(FrameSetSource):
    """
    Takes a regular FrameSetSource and selects the specified beam indices.
    """

    def __init__(self, frame_set_source: FrameSetSource, indices: List[List[int]]) -> None:
        FrameSetSource.__init__(self)
        # Beam selection is a lidar concept; only lidar sensors participate.
        if len(frame_set_source.sensor_info) != len(indices):
            raise ValueError("beam indices should match the count of sensors")

        for beams, metadata in zip(indices, cast(List[LidarInfo], frame_set_source.sensor_info)):
            if len(beams) != len(set(beams)):
                raise ValueError("beam indices can't contain duplicates")
            invalid_indices = [
                i for i in beams
                if i < 0 or i >= metadata.format.pixels_per_column
            ]
            if invalid_indices:
                raise ValueError(
                    f"beam indices {invalid_indices} must be in the range "
                    f"[0, {metadata.format.pixels_per_column})")

        self._frame_set_source = frame_set_source
        self._indices = indices
        self._metadata = self._generate_metadata(indices)
        self._input_sensor_info = self._frame_set_source.sensor_info

    def _generate_metadata(self, indices):
        return [select_by_index_metadata(m, b)
                for m, b in zip(self._frame_set_source.sensor_info, indices)]

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return cast(List[SensorInfo], self._metadata)

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
        def select_frame(frame, indices, sensor_info):
            out = select_by_index(frame, indices)
            out.sensor_info = sensor_info
            return out

        for scans in self._frame_set_source:
            out: List[Optional[Frame]] = []
            for frame in scans:
                if not isinstance(frame, LidarFrame):
                    # pass through camera / other frame types (and None) unchanged
                    out.append(frame)
                    continue
                for idx, info in enumerate(self._input_sensor_info):
                    if info is frame.sensor_info:
                        break
                out.append(select_frame(frame, self._indices[idx],
                                       self._metadata[idx]))
            yield FrameSet(out)

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()


def _select_fn(self: FrameSetSource, indices: List[List[int]]) -> FrameSetSource:
    """
    Takes a regular FrameSetSource and selects the specified beam indices.
    """
    return SelectedFrameSetSource(self, indices)


# allow assigning this
FrameSetSource.select = _select_fn  # type: ignore

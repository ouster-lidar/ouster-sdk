from typing import Optional, Iterator, List
from ouster.sdk._bindings.client import FrameSetSource
from ouster.sdk.core import SensorInfo, Frame, FrameSet


class ComparisonFrameSetSource(FrameSetSource):
    """
    Plays back several sources in lockstep for comparison purposes.
    """

    def __init__(self, sources: List[FrameSetSource]) -> None:
        FrameSetSource.__init__(self)

        self._sources = sources
        self._info: List[SensorInfo] = []
        for src in sources:
            for sensor in src.sensor_info:
                self._info.append(sensor)

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return self._info

    @property
    def is_live(self) -> bool:
        return False

    @property
    def is_indexed(self) -> bool:
        return False

    @property
    def frames_num(self) -> List[int]:
        raise TypeError("Not supported")

    def __len__(self) -> int:
        raise TypeError("Not supported")

    def __length_hint__(self) -> int:
        return 0

    def __iter__(self) -> Iterator[FrameSet]:
        iters = []
        for src in self._sources:
            iters.append(iter(src))

        try:
            while True:
                out: List[Optional[Frame]] = []
                for it in iters:
                    scans = next(it)
                    for scan in scans:
                        out.append(scan)
                yield FrameSet(out)
        except StopIteration:
            return

    def close(self) -> None:
        for src in self._sources:
            src.close()

    def __del__(self) -> None:
        self.close()

from typing import Optional, Iterator, Union, List, overload
from ouster.sdk._bindings.client import ScanSource
from ouster.sdk.core import SensorInfo, LidarScan
from ouster.sdk.core.scan_ops import reduce_by_factor, reduce_by_factor_metadata


class ReducedScanSource(ScanSource):
    """
    Takes a regular ScanSource and reduces the beams count to the specified values.
    """

    def __init__(self, scan_source: ScanSource, beams: List[int]) -> None:
        ScanSource.__init__(self)
        if len(scan_source.sensor_info) != len(beams):
            raise ValueError("beams should match the count of sensors")

        factor_list = []
        for b, m in zip(beams, scan_source.sensor_info):
            f = m.format.pixels_per_column / b
            if not (f.is_integer() and f > 0):
                raise ValueError(f"beams {b} must be divisor of {m.format.pixels_per_column}")
            factor_list.append(int(f))

        self._scan_source = scan_source
        self._factors = factor_list
        self._metadata = self._generate_metadata(factor_list)
        self._input_sensor_info = self._scan_source.sensor_info

    def _generate_metadata(self, factor_list):
        return [reduce_by_factor_metadata(m, f)
                for m, f in zip(self._scan_source.sensor_info, factor_list)]

    @property
    def sensor_info(self) -> List[SensorInfo]:
        return self._metadata

    @property
    def is_live(self) -> bool:
        return self._scan_source.is_live

    @property
    def is_indexed(self) -> bool:
        return self._scan_source.is_indexed

    @property
    def scans_num(self) -> List[int]:
        return self._scan_source.scans_num

    def __len__(self) -> int:
        return len(self._scan_source)

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:
        def reduce_scan(scan, factor, sensor_info):
            out = reduce_by_factor(scan, factor)
            out.sensor_info = sensor_info
            return out

        for scans in self._scan_source:
            out: List[Optional[LidarScan]] = []
            for scan in scans:
                for idx, info in enumerate(self._input_sensor_info):
                    if scan is not None and info is scan.sensor_info:
                        break
                if scan is None:
                    out.append(None)
                else:
                    out.append(reduce_scan(scan, self._factors[idx], self._metadata[idx]))
            yield out

    @overload
    def __getitem__(self, key: int) -> List[LidarScan]:
        ...

    @overload
    def __getitem__(self, key: slice) -> ScanSource:
        ...

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[LidarScan], ScanSource]:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()


def _reduce_fn(source: ScanSource, beams: List[int]) -> ScanSource:
    """
    Takes a regular ScanSource and reduces the beams count to the specified values.
    """
    return ReducedScanSource(source, beams)


# allow assigning this
ScanSource.reduce = _reduce_fn  # type: ignore

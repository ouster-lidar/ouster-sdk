from typing import Optional, Iterator, Union, List
import copy
from ouster.sdk.client.multi_scan_source import MultiScanSource
from ouster.sdk.client import SensorInfo, LidarScan
from ouster.sdk.client.data import FieldTypes
from ouster.sdk.client.scan_ops import reduce_by_factor


class MultiReducedScanSource(MultiScanSource):
    """
    Takes a regular MultiScanSource and reduces the beams count of it by supplied
    factor. The reduction factor is applied to the sensor_info and every yielded
    scan.
    """

    def __init__(self, scan_source: MultiScanSource, beams: List[int]) -> None:
        """remarks: if supplied, beams takes precedence over factors"""

        if scan_source.sensors_count != len(beams):
            raise ValueError("beams should match the count of sensors")

        factors = []
        for b, m in zip(beams, scan_source.metadata):
            f = m.format.pixels_per_column / b
            if not (f.is_integer() and f > 0):
                raise ValueError(f"beams {b} must be divisor of {m.format.pixels_per_column}")
            factors.append(int(f))

        self._scan_source = scan_source
        self._factors = factors
        self._metadata = self._generate_metadata()

    def _generate_metadata(self):
        metas = copy.deepcopy(self._scan_source.metadata)
        v_res = [si.format.pixels_per_column // self._factors[idx]
                 for idx, si in enumerate(self._scan_source.metadata)]
        for idx, si, v_res in zip(range(len(metas)), metas, v_res):
            pi = si.get_product_info()
            form_factor = pi.form_factor if not pi.form_factor[-1].isdigit() else \
                F"{pi.form_factor[:-1]}-{pi.form_factor[-1]}"
            si.prod_line = F"{form_factor}-{v_res}"
            si.format.pixels_per_column = v_res
            si.format.pixel_shift_by_row = si.format.pixel_shift_by_row[::self._factors[idx]]
            si.beam_azimuth_angles = si.beam_azimuth_angles[::self._factors[idx]]
            si.beam_altitude_angles = si.beam_altitude_angles[::self._factors[idx]]
        return metas

    @property
    def sensors_count(self) -> int:
        return self._scan_source.sensors_count

    @property
    def metadata(self) -> List[SensorInfo]:
        return self._metadata

    @property
    def is_live(self) -> bool:
        return self._scan_source.is_live

    @property
    def is_seekable(self) -> bool:
        return self._scan_source.is_seekable

    @property
    def is_indexed(self) -> bool:
        return self._scan_source.is_indexed

    @property
    def field_types(self) -> List[FieldTypes]:
        return self._scan_source.field_types

    @property
    def fields(self) -> List[List[str]]:
        return self._scan_source.fields

    @property
    def scans_num(self) -> List[Optional[int]]:
        return self._scan_source.scans_num

    def __len__(self) -> int:
        return len(self._scan_source)

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:

        def reduce_scan(scan, factor, sensor_info):
            out = reduce_by_factor(scan, factor)
            out.sensor_info = sensor_info
            return out

        for scans in self._scan_source.__iter__():
            yield [reduce_scan(scan, self._factors[idx], self._metadata[idx])
                   if scan else None for idx, scan in enumerate(scans)]

    def _seek(self, key: int) -> None:
        self._scan_source._seek(key)

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], MultiScanSource]:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __del__(self) -> None:
        self.close()

    def _slice_iter(self, key: slice) -> Iterator[List[Optional[LidarScan]]]:
        from itertools import islice
        from ouster.sdk.util.forward_slicer import ForwardSlicer
        # here we are slicing the current slice
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        return islice(iter(self), k.start, k.stop, k.step)

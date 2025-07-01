from typing import List, Union, Callable, Optional
import copy
import numpy as np
from ouster.sdk.core import LidarScan, SensorInfo
from ouster.sdk.core.data import destagger


def clip(scan: LidarScan, fields: List[str], lower: int, upper: int, invalid: int = 0) -> None:
    """
    limits the values of the specified set of fields to within the range = [lower, upper], any value
    that exceeds this range is replaced by the supplied invalid value (default is zero)
    """
    if not fields:
        fields = list(scan.fields)
    for f in fields:
        if scan.has_field(f):
            m = scan.field(f)
            m[(m < lower) | (m > upper)] = invalid


def filter_field(scan: LidarScan, field: str, lower: int, upper: int, invalid: int = 0,
                 filtered_fields: Optional[List[str]] = None) -> None:
    """
    Filters out scan fields based on the specified axis and range.
    Values that falls in the range [lower, upper] are replaced by
    the supplied invalid value (default is zero).

    Parameters:
    - scan: LidarScan
    - field: str; the field to be used as the filter
    - lower: float; lower bound
    - upper: float; upper bound
    - invalid: int; the invalid value to use default is 0
    - filtered_fields: Optional[List[str]]; an optional list of fields to filter
    """
    m = scan.field(field)
    filtered_pts = (m >= lower) & (m <= upper)
    filtered_fields = filtered_fields or list(scan.fields)
    for target_f in filtered_fields:
        if scan.has_field(target_f):
            scan.field(target_f)[filtered_pts] = invalid


def filter_uv(scan: LidarScan, coord_2d: str, lower: Union[int, float], upper: Union[int, float],
              invalid: int = 0, filtered_fields: Optional[List[str]] = None) -> None:
    """
    Filters the scan based on the specified image axis.
    Values that falls in the range [lower, upper] are replaced by
    the supplied invalid value (default is zero).

    Parameters:
    - scan: LidarScan
    - field: str; the field to be used as the filter
    - lower: Union[int, float]; lower bound if float it is assumed a percentage
    - upper: Union[int, float]; upper bound if float it is assumed a percentage
    - invalid: int; the invalid value to use default is 0
    - filtered_fields: Optional[List[str]]; an optional list of fields to filter
    """
    if coord_2d not in ['u', 'v']:
        raise ValueError(f"coord_2d == {coord_2d} must be either 'u' or 'v'")

    coord_size = scan.h if coord_2d == 'u' else scan.w

    def _interpret_as_int(val: float) -> int:
        if val == float("-inf"):
            return 0
        if val == float("inf"):
            return coord_size
        if val > 0 and val < 1:
            return int(coord_size * val)
        return int(val)

    if isinstance(lower, float):
        lower = _interpret_as_int(lower)
    if isinstance(upper, float):
        upper = _interpret_as_int(upper)

    if lower < 0 or upper > coord_size:
        raise ValueError(f"lower == {lower} and upper == {upper} must be in the range [0, {coord_size}]")

    if lower > upper:
        raise ValueError(f"lower == {lower} must be less than upper == {upper}")

    filtered_fields = filtered_fields or list(scan.fields)
    u_slice, v_slice = (slice(lower, upper), slice(None)) if coord_2d == 'u' \
        else (slice(None), slice(lower, upper))
    for target_f in filtered_fields:
        if scan.has_field(target_f):
            if coord_2d == 'v':
                # destaggering mainly affects the v axis
                result = destagger(scan.sensor_info, scan.field(target_f))
                result[u_slice, v_slice] = invalid
                scan.field(target_f)[:] = destagger(scan.sensor_info, result, inverse=True)
            else:
                scan.field(target_f)[u_slice, v_slice] = invalid


def filter_xyz(scan: LidarScan, xyzlut: Callable[[Union[LidarScan, np.ndarray]], np.ndarray],
               axis_idx: int, lower: float = float("-inf"), upper: float = float("inf"),
               invalid: int = 0, filtered_fields: Optional[List[str]] = None) -> None:
    """
    Filters the scan based on the specified axis and range. Values below the lower bound or above the upper
    bound are replaced by the supplied invalid value (default is zero)

    Parameters:
    - scan: LidarScan
    - xyzlut: Callable[[Union[LidarScan, np.ndarray]], np.ndarray]
    - axis_idx: int; can be 0, 1, or 2
    - lower: float; lower bound
    - upper: float; upper bound
    - invalid: int; the invalid value to use default is 0
    - filtered_fields: Optional[List[str]]; an optional list of fields to filter
    """
    if axis_idx < 0 or axis_idx > 2:
        raise ValueError(f"axis_idx == {axis_idx} must be in the range [0, 2]")

    for range_f in ['RANGE', 'RANGE2']:
        if not scan.has_field(range_f):
            continue
        pts = xyzlut(scan.field(range_f))
        below_min = pts[:, :, axis_idx] >= lower
        above_max = pts[:, :, axis_idx] <= upper
        filtered_pts = below_min & above_max
        filtered_fields = filtered_fields or list(scan.fields)
        fields = set(filtered_fields) - (set(['RANGE', 'RANGE2']) - set([range_f]))
        for target_f in fields:
            if scan.has_field(target_f):
                scan.field(target_f)[filtered_pts] = invalid


def mask(scan: LidarScan, fields: List[str], mask: np.ndarray) -> None:
    """
    applies a mask/filter to all fields of the LidarScan
    mask should be of the same as size scan.PIXEL_FIELD
    """
    if mask.shape[0] != scan.h or mask.shape[1] != scan.w:
        raise ValueError(f"Used mask size {mask.shape} doesn't match scan size"
                         " ({scan.h}, {scan.w}")

    if not fields:
        fields = list(scan.fields)
    masked_indices = np.where(mask == 0)
    for f in fields:
        if scan.has_field(f):
            scan.field(f)[masked_indices] = 0


def reduce_by_factor_metadata(metadata: SensorInfo, factor: int) -> SensorInfo:
    out = copy.deepcopy(metadata)
    v_res = metadata.format.pixels_per_column // factor
    pi = metadata.get_product_info()
    form_factor = pi.form_factor if not pi.form_factor[-1].isdigit() else \
        F"{pi.form_factor[:-1]}-{pi.form_factor[-1]}"
    out.prod_line = F"{form_factor}-{v_res}"
    out.format.pixels_per_column = v_res
    out.format.pixel_shift_by_row = metadata.format.pixel_shift_by_row[::factor]
    out.beam_azimuth_angles = metadata.beam_azimuth_angles[::factor]
    out.beam_altitude_angles = metadata.beam_altitude_angles[::factor]
    return out


def reduce_by_factor(scan: LidarScan, factor: int,
                     update_metadata: bool = False) -> LidarScan:
    """
    Vertically downsample the LidarScan by the supplied factor
    factor must by a divisor of the LidarScan height
    """
    if factor <= 0:
        raise ValueError(f"factor == {factor} can't be negative")
    if not (scan.h / factor).is_integer():
        raise ValueError(f"factor == {factor} must be a divisor of {scan.h}")

    scan_h = scan.h // factor
    result = LidarScan(scan_h, scan.w, scan.field_types)
    # copy std properties
    result.frame_id = scan.frame_id
    result.frame_status = scan.frame_status
    for i in range(len(scan.timestamp)):
        result.timestamp[i] = scan.timestamp[i]
    for i in range(len(scan.packet_timestamp)):
        result.packet_timestamp[i] = scan.packet_timestamp[i]
    for i in range(len(scan.measurement_id)):
        result.measurement_id[i] = scan.measurement_id[i]
    for i in range(len(scan.status)):
        result.status[i] = scan.status[i]
    for i in range(len(scan.pose)):
        result.pose[i] = scan.pose[i]
    for f in scan.fields:
        result.field(f)[:] = scan.field(f)[::factor]
    if update_metadata:
        result.sensor_info = reduce_by_factor_metadata(scan.sensor_info, factor)
    return result

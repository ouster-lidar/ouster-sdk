from typing import List, Union, Callable, Optional
import copy
import numpy as np
from ouster.sdk.core import LidarScan, SensorInfo, FieldClass, dewarp
from ouster.sdk.core.data import destagger


def _resolve_pixel_fields(scan: LidarScan,
                          filtered_fields: Optional[List[str]]) -> List[str]:
    """Resolve which scan fields to operate on, restricted to PIXEL_FIELD types.

    Rules:
    - If filtered_fields is None: return all pixel fields present in the scan.
    - If filtered_fields is provided: validate that any present requested fields are
      pixel fields; non-pixel fields raise ValueError.
      (Missing fields are ignored to avoid failing mid-stream when fields vary.)
    """
    pixel_fields = {ft.name for ft in scan.field_types
                    if ft.field_class == FieldClass.PIXEL_FIELD}

    requested = filtered_fields if filtered_fields is not None else list(scan.fields)
    present = [f for f in requested if scan.has_field(f)]

    non_pixel = [f for f in present if f not in pixel_fields]
    if filtered_fields is not None and non_pixel:
        raise ValueError(
            f"Only PIXEL_FIELD scan fields are supported here; requested non-pixel fields: {non_pixel}"
        )

    return [f for f in present if f in pixel_fields]


def clip(scan: LidarScan, fields: List[str], lower: float, upper: float,
         invalid: int = 0) -> None:
    """
    Limits the values of the specified set of pixel fields to within the range
    [lower, upper]. Any value outside this range is replaced by the supplied
    invalid value (default is zero).
    """
    # PIXEL_FIELD targets only (default: all pixel fields; explicit: requested pixel fields).
    fields_to_clip = _resolve_pixel_fields(scan, fields if fields else None)
    for f in fields_to_clip:
        m = scan.field(f)
        m[(m < lower) | (m > upper)] = invalid


def filter_field(scan: LidarScan, field: str, lower: float, upper: float, invalid: int = 0,
                 filtered_fields: Optional[List[str]] = None) -> None:
    """
    Filters scan pixel fields based on the values of another pixel field.
    Pixels whose filter field values fall in the range [lower, upper] are
    replaced by the supplied invalid value (default is zero).

    Parameters:
    - scan: LidarScan
    - field: str; the pixel field to be used as the filter mask source
    - lower: float; lower bound
    - upper: float; upper bound
    - invalid: int; the invalid value to use default is 0
    - filtered_fields: Optional[List[str]]; an optional list of fields to filter
    """
    # PIXEL_FIELD targets only (default: all pixel fields; explicit: requested pixel fields).
    fields_to_filter = _resolve_pixel_fields(scan, filtered_fields)

    m = scan.field(field)
    if m.shape[0] != scan.h or m.shape[1] != scan.w:
        raise ValueError(
            f"filter_field requires a pixel field with shape (h, w) to build a mask; "
            f"got field '{field}' with shape {m.shape} while scan size is ({scan.h}, {scan.w})"
        )

    filtered_pts = (m >= lower) & (m <= upper)
    for target_f in fields_to_filter:
        scan.field(target_f)[filtered_pts] = invalid


def filter_uv(scan: LidarScan, coord_2d: str, lower: Union[int, float], upper: Union[int, float],
              invalid: int = 0, filtered_fields: Optional[List[str]] = None) -> None:
    """
    Filters the scan based on the specified image axis ('u' or 'v').
    Pixel values that fall within the specified index range [lower, upper)
    are replaced by the supplied invalid value (default is zero).

    Parameters:
    - scan: LidarScan
    - coord_2d: str; image axis to filter ('u' rows, 'v' columns)
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
        if 0 <= val <= 1:
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

    # PIXEL_FIELD targets only (default: all pixel fields; explicit: requested pixel fields).
    fields_to_filter = _resolve_pixel_fields(scan, filtered_fields)

    u_slice, v_slice = (slice(lower, upper), slice(None)) if coord_2d == 'u' \
        else (slice(None), slice(lower, upper))
    for target_f in fields_to_filter:
        if coord_2d == 'v':
            # destaggering mainly affects the v axis
            result = destagger(scan.sensor_info, scan.field(target_f))
            result[u_slice, v_slice] = invalid
            scan.field(target_f)[:] = destagger(scan.sensor_info, result, inverse=True)
        else:
            scan.field(target_f)[u_slice, v_slice] = invalid


def filter_xyz(scan: LidarScan, xyzlut: Callable[[Union[LidarScan, np.ndarray]], np.ndarray],
               axis_idx: int, lower: float = float("-inf"), upper: float = float("inf"),
               invalid: int = 0, filtered_fields: Optional[List[str]] = None,
               dewarp_points: bool = False) -> None:
    """
    Filters the scan based on spatial coordinates (X, Y, or Z). Points with coordinates inside
    the specified range [lower, upper] are replaced with the invalid value (default is zero).

    Only PIXEL_FIELD types (spatial/image-like data such as RANGE, SIGNAL, REFLECTIVITY) are
    filtered. Non-spatial fields (IMU, GNSS position, etc.) are preserved.

    Parameters:
    - scan: LidarScan
    - xyzlut: Callable[[Union[LidarScan, np.ndarray]], np.ndarray]
    - axis_idx: int; spatial axis to filter (0=X, 1=Y, 2=Z)
    - lower: float; lower bound
    - upper: float; upper bound
    - invalid: int; the invalid value to use (default is 0)
    - filtered_fields: Optional[List[str]]; specific fields to filter (if None, filters all PIXEL_FIELD types)
    - dewarp_points: bool; if True, dewarp XYZ points using scan.pose
    """
    if axis_idx < 0 or axis_idx > 2:
        raise ValueError(f"axis_idx == {axis_idx} must be in the range [0, 2]")

    # PIXEL_FIELD targets only (default: all pixel fields; explicit: requested pixel fields).
    fields_to_filter = _resolve_pixel_fields(scan, filtered_fields)

    def _xyz_points(field_name: str) -> np.ndarray:
        points = xyzlut(scan.field(field_name))
        if dewarp_points:
            return dewarp(points, scan.pose)
        return points

    # Compute spatial masks from range fields
    range_mask = None
    range2_mask = None

    if scan.has_field('RANGE'):
        pts = _xyz_points('RANGE')
        range_mask = (pts[:, :, axis_idx] >= lower) & (pts[:, :, axis_idx] <= upper)

    if scan.has_field('RANGE2'):
        pts = _xyz_points('RANGE2')
        range2_mask = (pts[:, :, axis_idx] >= lower) & (pts[:, :, axis_idx] <= upper)

    if range_mask is None and range2_mask is None:
        return

    # Second-return fields (RANGE2, SIGNAL2, etc.) use RANGE2 coordinates
    # All other fields use RANGE coordinates
    second_return_fields = {'RANGE2', 'SIGNAL2', 'REFLECTIVITY2', 'FLAGS2'}

    for field in fields_to_filter:
        if field in second_return_fields:
            mask = range2_mask if range2_mask is not None else range_mask
        else:
            mask = range_mask if range_mask is not None else range2_mask
        scan.field(field)[mask] = invalid


def mask(scan: LidarScan, fields: List[str], mask: np.ndarray) -> None:
    """
    Applies a boolean mask to scan pixel fields.

    mask should have shape (scan.h, scan.w). Pixels where mask == 0 are set to 0.
    """
    if mask.shape[0] != scan.h or mask.shape[1] != scan.w:
        raise ValueError(f"Used mask size {mask.shape} doesn't match scan size"
                         " ({scan.h}, {scan.w}")

    # PIXEL_FIELD targets only (default: all pixel fields; explicit: requested pixel fields).
    fields_to_mask = _resolve_pixel_fields(scan, fields if fields else None)
    masked_indices = np.where(mask == 0)
    for f in fields_to_mask:
        scan.field(f)[masked_indices] = 0


def _reduce_factor_to_slice(factor: int, height: int) -> slice:
    """
    Generate the slice to use for reducing. Handles special cases like single laser.
    """
    if factor == height:
        return slice(height // 2, height // 2 + 1, None)
    return slice(None, None, factor)


def reduce_by_factor_metadata(metadata: SensorInfo, factor: int) -> SensorInfo:
    out = copy.deepcopy(metadata)
    v_res = metadata.format.pixels_per_column // factor
    pi = metadata.get_product_info()
    form_factor = pi.form_factor if not pi.form_factor[-1].isdigit() else \
        F"{pi.form_factor[:-1]}-{pi.form_factor[-1]}"
    out.prod_line = F"{form_factor}-{v_res}"
    out.format.pixels_per_column = v_res
    factor_slice = _reduce_factor_to_slice(factor, metadata.h)
    out.format.pixel_shift_by_row = metadata.format.pixel_shift_by_row[factor_slice]
    out.beam_azimuth_angles = metadata.beam_azimuth_angles[factor_slice]
    out.beam_altitude_angles = metadata.beam_altitude_angles[factor_slice]
    # downsample zones
    if out.zone_set:
        for id in out.zone_set.zones:
            zone = out.zone_set.zones[id]
            if not zone.zrb:
                continue
            zrb = zone.zrb
            zrb.far_range_mm = zrb.far_range_mm[factor_slice]
            zrb.near_range_mm = zrb.near_range_mm[factor_slice]
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
    result = LidarScan(scan_h, scan.w, scan.field_types, scan.sensor_info.format.columns_per_packet)
    # copy std properties
    result.frame_id = scan.frame_id
    result.frame_status = scan.frame_status
    result.timestamp[:] = scan.timestamp
    result.packet_timestamp[:] = scan.packet_timestamp
    result.measurement_id[:] = scan.measurement_id
    result.status[:] = scan.status
    result.pose[:] = scan.pose
    factor_slice = _reduce_factor_to_slice(factor, scan.h)
    for f in scan.field_types:
        if f.field_class != FieldClass.PIXEL_FIELD:
            result.field(f.name)[:] = scan.field(f.name)
        else:
            result.field(f.name)[:] = scan.field(f.name)[factor_slice]
    if update_metadata:
        result.sensor_info = reduce_by_factor_metadata(scan.sensor_info, factor)
    return result

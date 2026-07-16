from typing import List, Union, Callable, Optional, cast
import numpy as np
from ouster.sdk.core import LidarFrame, SensorInfo, FieldClass, dewarp
from ouster.sdk._bindings.client import clip  # type: ignore[attr-defined]  # noqa: F401
from ouster.sdk._bindings.client import filter_field  # type: ignore[attr-defined]  # noqa: F401
from ouster.sdk._bindings.client import _frame_ops_filter_uv
from ouster.sdk._bindings.client import _frame_ops_mask
from ouster.sdk._bindings.client import select_by_index_metadata as _select_by_index_metadata
from ouster.sdk._bindings.client import select_by_index as _select_by_index
from ouster.sdk._bindings.client import reduce_by_factor_metadata as _reduce_by_factor_metadata
from ouster.sdk._bindings.client import reduce_by_factor as _reduce_by_factor


def _resolve_pixel_fields(frame: LidarFrame,
                          filtered_fields: Optional[List[str]]) -> List[str]:
    """Resolve which frame fields to operate on, restricted to PIXEL_FIELD types.

    Rules:
    - If filtered_fields is None: return all pixel fields present in the frame.
    - If filtered_fields is provided: validate that any present requested fields are
      pixel fields; non-pixel fields raise ValueError.
      (Missing fields are ignored to avoid failing mid-stream when fields vary.)
    """
    pixel_fields = {ft.name for ft in frame.field_types
                    if ft.field_class == FieldClass.PIXEL_FIELD}

    requested = filtered_fields if filtered_fields is not None else list(frame.fields)
    present = [f for f in requested if frame.has_field(f)]

    non_pixel = [f for f in present if f not in pixel_fields]
    if filtered_fields is not None and non_pixel:
        raise ValueError(
            f"Only PIXEL_FIELD frame fields are supported here; requested non-pixel fields: {non_pixel}"
        )

    return [f for f in present if f in pixel_fields]


def filter_uv(frame: LidarFrame, coord_2d: str, lower: Union[int, float], upper: Union[int, float],
              invalid: int = 0, filtered_fields: Optional[List[str]] = None) -> None:
    """
    Filters the frame based on the specified image axis ('u' or 'v').
    Pixel values that fall within the specified index range [lower, upper)
    are replaced by the supplied invalid value (default is zero).

    Parameters:
    - frame: LidarFrame
    - coord_2d: str; image axis to filter ('u' rows, 'v' columns)
    - lower: Union[int, float]; lower bound if float it is assumed a percentage
    - upper: Union[int, float]; upper bound if float it is assumed a percentage
    - invalid: int; the invalid value to use default is 0
    - filtered_fields: Optional[List[str]]; an optional list of fields to filter
    """
    if coord_2d not in ['u', 'v']:
        raise ValueError(f"coord_2d == {coord_2d} must be either 'u' or 'v'")

    coord_size = frame.h if coord_2d == 'u' else frame.w

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

    _frame_ops_filter_uv(frame, coord_2d, lower, upper, invalid,
                        filtered_fields or [], filtered_fields is not None)


def filter_xyz(frame: LidarFrame, xyzlut: Callable[[Union[LidarFrame, np.ndarray]], np.ndarray],
               axis_idx: int, lower: float = float("-inf"), upper: float = float("inf"),
               invalid: int = 0, filtered_fields: Optional[List[str]] = None,
               dewarp_points: bool = False) -> None:
    """
    Filters the frame based on spatial coordinates (X, Y, or Z). Points with coordinates inside
    the specified range [lower, upper] are replaced with the invalid value (default is zero).

    Only PIXEL_FIELD types (spatial/image-like data such as RANGE, SIGNAL, REFLECTIVITY) are
    filtered. Non-spatial fields (IMU, GNSS position, etc.) are preserved.

    Parameters:
    - frame: LidarFrame
    - xyzlut: Callable[[Union[LidarFrame, np.ndarray]], np.ndarray]
    - axis_idx: int; spatial axis to filter (0=X, 1=Y, 2=Z)
    - lower: float; lower bound
    - upper: float; upper bound
    - invalid: int; the invalid value to use (default is 0)
    - filtered_fields: Optional[List[str]]; specific fields to filter (if None, filters all PIXEL_FIELD types)
    - dewarp_points: bool; if True, dewarp XYZ points using frame.body_to_world
    """
    if axis_idx < 0 or axis_idx > 2:
        raise ValueError(f"axis_idx == {axis_idx} must be in the range [0, 2]")

    def _xyz_points(field_name: str) -> np.ndarray:
        points = xyzlut(frame.field(field_name))
        if dewarp_points:
            return cast(np.ndarray, dewarp(points, frame.body_to_world))
        return points

    # Compute spatial masks from range fields
    range_mask = None
    range2_mask = None

    if frame.has_field('RANGE'):
        pts = _xyz_points('RANGE')
        range_mask = (pts[:, :, axis_idx] >= lower) & (pts[:, :, axis_idx] <= upper)

    if frame.has_field('RANGE2'):
        pts = _xyz_points('RANGE2')
        range2_mask = (pts[:, :, axis_idx] >= lower) & (pts[:, :, axis_idx] <= upper)

    if range_mask is None and range2_mask is None:
        return

    fields_to_filter = _resolve_pixel_fields(frame, filtered_fields)
    second_return_fields = {'RANGE2', 'SIGNAL2', 'REFLECTIVITY2', 'FLAGS2'}

    for field in fields_to_filter:
        if field in second_return_fields:
            mask = range2_mask if range2_mask is not None else range_mask
        else:
            mask = range_mask if range_mask is not None else range2_mask
        frame.field(field)[mask] = invalid


def mask(frame: LidarFrame, fields: List[str], mask: np.ndarray) -> None:
    """
    Applies a boolean mask to frame pixel fields.

    mask should have shape (frame.h, frame.w). Pixels where mask == 0 are set to 0.
    """
    if mask.shape[0] != frame.h or mask.shape[1] != frame.w:
        raise ValueError(f"Used mask size {mask.shape} doesn't match frame size"
                         " ({frame.h}, {frame.w}")

    _frame_ops_mask(frame, fields if fields else [],
                   np.ascontiguousarray(mask, dtype=np.uint8))


def _validate_beam_indices(indices: List[int], height: int) -> None:
    if not indices:
        raise ValueError("beam indices can't be empty")
    if len(indices) != len(set(indices)):
        raise ValueError("beam indices can't contain duplicates")
    invalid_indices = [i for i in indices if i < 0 or i >= height]
    if invalid_indices:
        raise ValueError(
            f"beam indices {invalid_indices} must be in the range [0, {height})")


def select_by_index_metadata(metadata: SensorInfo, indices: List[int]) -> SensorInfo:
    """
    Generate a SensorInfo matching an arbitrary subset of beam indices.
    """
    _validate_beam_indices(indices, metadata.h)
    return _select_by_index_metadata(metadata, indices)


def select_by_index(frame: LidarFrame, indices: List[int],
                    update_metadata: bool = False) -> LidarFrame:
    """
    Select specific beam rows from the LidarFrame by the supplied indices.
    Indices must be within the range of the LidarFrame height.
    """
    _validate_beam_indices(indices, frame.h)
    return _select_by_index(frame, indices, update_metadata)


def _reduce_factor_to_slice(factor: int, height: int) -> slice:
    """
    Generate the slice to use for reducing. Handles special cases like single laser.
    """
    if factor == height:
        return slice(height // 2, height // 2 + 1, None)
    return slice(None, None, factor)


def _reduce_factor_to_indices(factor: int, height: int) -> List[int]:
    """
    Generate the beam indices to use for reducing.
    """
    if factor <= 0:
        raise ValueError(f"factor == {factor} can't be negative")
    if not (height / factor).is_integer():
        raise ValueError(f"factor == {factor} must be a divisor of {height}")

    factor_slice = _reduce_factor_to_slice(factor, height)
    return list(range(height))[factor_slice]


def reduce_by_factor_metadata(metadata: SensorInfo, factor: int) -> SensorInfo:
    _reduce_factor_to_indices(factor, metadata.h)
    return _reduce_by_factor_metadata(metadata, factor)


def reduce_by_factor(frame: LidarFrame, factor: int,
                     update_metadata: bool = False) -> LidarFrame:
    """
    Vertically downsample the LidarFrame by the supplied factor
    factor must by a divisor of the LidarFrame height
    """
    _reduce_factor_to_indices(factor, frame.h)
    return _reduce_by_factor(frame, factor, update_metadata)

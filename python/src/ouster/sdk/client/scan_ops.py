from typing import List
import copy
import numpy as np
from ouster.sdk.client import LidarScan, SensorInfo


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
    for f in fields:
        if scan.has_field(f):
            scan.field(f)[:] *= mask.astype(scan.field(f)[:].dtype)


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

"""Convert a planar depth image into the radial RANGE the SDK expects."""

import numpy as np


def depth_to_range_mm(depth_mm, model):
    # [doc-stag-depth-to-range]
    # depth_mm is planar depth (distance along the optical axis) in
    # millimeters, shaped (h, w). RANGE is the distance from the camera
    # origin, so each pixel is scaled by the length of its unit ray.
    height, width = depth_mm.shape
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    x = (u - model.cx) / model.fx
    y = (v - model.cy) / model.fy
    range_mm = depth_mm * np.sqrt(1.0 + x * x + y * y)
    return np.round(range_mm).astype(np.uint32)
    # [doc-etag-depth-to-range]

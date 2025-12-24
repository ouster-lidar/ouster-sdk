import numpy as np

from ouster.sdk.core import SensorInfo
from ouster.sdk.examples.lidar_scan import lidar_scan_example


def test_custom_field_populated_from_example(test_data_dir):
    """Custom field should exist with baseline value 1 and 255 override block."""
    info = SensorInfo(open(test_data_dir / "metadata" / "3_0_1_os-122246000293-128.json").read())
    ls = lidar_scan_example(info)
    field = ls.field("my-custom-field")

    assert field.dtype == np.uint8
    assert field.shape == (ls.h, ls.w)

    # Expect exactly two values: default 1 everywhere except a 10x10 block of 255
    values, counts = np.unique(field, return_counts=True)
    assert set(values.tolist()) == {1, 255}
    assert counts[values.tolist().index(255)] == 100
    assert counts.sum() == field.size

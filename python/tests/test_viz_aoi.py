import numpy as np
from ouster.sdk.core import SensorInfo, LidarMode
from ouster.sdk.viz.model import SensorModel, Selection2d


def test_selection_aoi_mask():
    # it should update the selection mask when either of the points defining the AOI are changed

    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)
    image = None  # not needed for this example
    assert meta.w == 1024
    assert meta.h == 64
    selection = Selection2d(
        (10, 10),
        (110, 110),  # NOTE! exceeds the size of the image, so it'll be clipped
        0,  # sensor index
        sensor,
        0,  # image index
        image
    )

    expected_result = np.zeros((meta.h, meta.w), np.float32)
    expected_result[10:64, 10:110] = 1
    assert np.array_equal(selection._aoi_mask, expected_result)

    #  make the selection smaller
    expected_result = np.zeros((meta.h, meta.w), np.float32)
    expected_result[20:40, 20:40] = 1
    selection.p1 = (20, 20)
    selection.p2 = (40, 40)
    assert np.array_equal(selection._aoi_mask, expected_result)

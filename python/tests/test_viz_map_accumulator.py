from typing import cast

import numpy as np

from .conftest import MockPointViz
from ouster.sdk.core import ChanField, LidarMode, LidarFrame, SensorInfo
from ouster.sdk._bindings.viz import PointViz
from ouster.sdk.viz.accumulators_config import LidarFrameVizAccumulatorsConfig
from ouster.sdk.viz.map_accumulator import MapAccumulator
from ouster.sdk.viz.model import LidarFrameVizModel
from ouster.sdk.viz.track import Track


def _make_map_accumulator(meta: SensorInfo, max_points: int = 10) -> MapAccumulator:
    viz = cast(PointViz, MockPointViz())
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    config = LidarFrameVizAccumulatorsConfig(
        map_enabled=True,
        map_select_ratio=1.0,
        map_max_points=max_points
    )
    return MapAccumulator(model, viz, Track(config), config)


def test_update_map_preserves_vector_key_shape_and_dtype():
    """It should cache vector-valued cloud keys for RGB and normals modes."""
    meta = SensorInfo.from_default(LidarMode._1024x10)
    accum = _make_map_accumulator(meta)
    frame = LidarFrame(meta)
    frame.status[:] = 1
    frame.timestamp[:] = np.arange(frame.w)
    frame.field(ChanField.RANGE)[:] = 0
    frame.field(ChanField.RANGE)[:2, :2] = 1000

    rgb = np.zeros((meta.h, meta.w, 3), dtype=np.uint8)
    rgb[:] = [10, 20, 30]
    frame.add_field("RGB", rgb)

    normals = np.zeros((meta.h, meta.w, 3), dtype=np.float32)
    normals[:] = [1.0, 0.0, -1.0]
    frame.add_field("NORMALS", normals)

    accum._model._amend_view_modes_all([frame])
    accum.update([frame], frame_num=0)

    assert accum._map_idx == 4
    assert accum._map_keys["RGB"].shape == (10, 3)
    assert accum._map_keys["RGB"].dtype == np.uint8
    np.testing.assert_array_equal(accum._map_keys["RGB"][:4], [[10, 20, 30]] * 4)
    assert accum._map_keys["NORMALS"].shape == (10, 3)
    assert accum._map_keys["NORMALS"].dtype == np.float32
    np.testing.assert_allclose(accum._map_keys["NORMALS"][:4], [[1.0, 0.5, 0.0]] * 4)

    accum._cloud_mode_name = "RGB"
    accum._draw_map()


def test_ensure_structs_map_preserves_key_trailing_shape_and_dtype():
    """It should keep scalar and vector key shapes when map storage grows."""
    meta = SensorInfo.from_default(LidarMode._1024x10)
    accum = _make_map_accumulator(meta)
    accum._map_xyz = np.zeros((2, 3), dtype=np.float32)
    accum._map_keys["REFLECTIVITY"] = np.array([0.1, 0.2], dtype=np.float32)
    accum._map_keys["RGB"] = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)
    accum._map_idx = 2

    accum._ensure_structs_map(1)

    grown_size = accum._map_xyz.shape[0]
    assert grown_size > 2
    assert accum._map_keys["REFLECTIVITY"].shape == (grown_size,)
    assert accum._map_keys["REFLECTIVITY"].dtype == np.float32
    np.testing.assert_allclose(accum._map_keys["REFLECTIVITY"][:2], [0.1, 0.2])
    assert accum._map_keys["RGB"].shape == (grown_size, 3)
    assert accum._map_keys["RGB"].dtype == np.uint8
    np.testing.assert_array_equal(accum._map_keys["RGB"][:2], [[1, 2, 3], [4, 5, 6]])

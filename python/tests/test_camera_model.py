"""
Copyright (c) 2026, Ouster, Inc.
All rights reserved.

Tests for the single optional CameraModel on CameraInfo.

The distinction these cover is the one the API is built around: a CameraInfo
with a model can be projected into a point cloud, and one without can only be
viewed.
"""

import numpy as np
import pytest

from ouster.sdk import core


def _camera_info(with_model=True):
    info = core.CameraInfo()
    info.format.columns_per_frame = 4
    info.format.pixels_per_column = 3
    if with_model:
        info.model = core.CameraModel(100.0, 100.0, 0.0, 0.0)
    return info


def test_camera_model_requires_positive_focal_lengths():
    model = core.CameraModel(100.0, 110.0, 2.0, 1.5)
    assert model.fx == 100.0
    assert model.fy == 110.0
    assert model.cx == 2.0
    assert model.cy == 1.5

    for bad in (0.0, -1.0, float('nan')):
        with pytest.raises(ValueError):
            core.CameraModel(bad, 100.0, 0.0, 0.0)
        with pytest.raises(ValueError):
            core.CameraModel(100.0, bad, 0.0, 0.0)

    # There is no no-argument form to sidestep the check.
    with pytest.raises(TypeError):
        core.CameraModel()  # type: ignore[call-arg]


def test_camera_model_equality_is_per_field():
    assert core.CameraModel(100.0, 110.0, 2.0, 1.5) == core.CameraModel(100.0, 110.0, 2.0, 1.5)
    assert core.CameraModel(100.0, 110.0, 2.0, 1.5) != core.CameraModel(100.0, 110.0, 2.0, 9.0)


def test_camera_info_model_defaults_to_none():
    assert core.CameraInfo().model is None


def test_camera_info_model_is_assignable_and_clearable():
    info = core.CameraInfo()
    info.model = core.CameraModel(100.0, 110.0, 2.0, 1.5)
    assert info.model is not None
    assert info.model.fx == 100.0

    info.model = None
    assert info.model is None


def test_camera_info_json_round_trip_with_model():
    info = _camera_info()
    info.sn = 12345
    info.format.fps = 30

    restored = core.CameraInfo(info.to_json_string())
    assert restored.format.columns_per_frame == 4
    assert restored.format.pixels_per_column == 3
    assert restored.fps == 30
    assert restored.model is not None
    assert restored.model.fx == 100.0
    assert restored.model.cy == 0.0
    assert restored == info


def test_camera_info_json_round_trip_without_model():
    info = _camera_info(with_model=False)

    # An absent model must survive as absent; a zero-valued model would claim a
    # calibration the camera does not have.
    json = info.to_json_string()
    assert '"model"' not in json

    restored = core.CameraInfo(json)
    assert restored.model is None
    assert restored.format.columns_per_frame == 4
    assert restored == info


def test_camera_info_model_participates_in_equality():
    with_model = _camera_info()
    without_model = _camera_info(with_model=False)
    assert with_model != without_model

    other = _camera_info(with_model=False)
    other.model = core.CameraModel(50.0, 100.0, 0.0, 0.0)
    assert with_model != other


def test_xyzlut_projects_with_a_model():
    info = _camera_info()
    lut = core.XYZLut(info)

    range_mm = np.zeros((3, 4), dtype=np.uint32)
    range_mm[0, 0] = 2000
    points = lut(range_mm)
    assert points.shape == (3, 4, 3)
    # cx = cy = 0 makes pixel (0,0) the unit ray along +X in the Ouster frame.
    np.testing.assert_allclose(points[0, 0], [2.0, 0.0, 0.0], atol=1e-5)


def test_xyzlut_is_empty_without_a_model():
    info = _camera_info(with_model=False)
    lut = core.XYZLut(info)
    assert (lut.h, lut.w) == (0, 0)
    with pytest.raises(ValueError):
        lut(np.zeros((3, 4), dtype=np.uint32))


@pytest.mark.parametrize("field", ["columns_per_frame", "pixels_per_column"])
def test_xyzlut_is_empty_with_zero_dimensions(field):
    info = _camera_info()
    setattr(info.format, field, 0)
    lut = core.XYZLut(info)
    assert (lut.h, lut.w) == (0, 0)


def test_xyzlut_rejects_model_mutated_to_zero_focal_length():
    info = _camera_info()
    model = info.model
    model.fx = 0.0
    info.model = model
    with pytest.raises(ValueError, match="focal lengths greater than zero"):
        core.XYZLut(info)


def test_camera_without_model_still_renders_images():
    """No model means viewable but not projectable, and viz must honor that."""
    from ouster.sdk.viz.model import CameraModel as VizCameraModel, Palettes
    from .conftest import MockPointViz

    viz = MockPointViz()
    camera = VizCameraModel(viz, _camera_info(with_model=False), palettes=Palettes([]))

    frame = core.CameraFrame()
    frame.w, frame.h = 4, 3
    frame.add_field("RANGE", np.full((3, 4), 2000, dtype=np.uint32))
    frame.add_field("LEFT_RGB", np.zeros((3, 4, 3), dtype=np.uint8))

    camera._amend_view_modes(frame)
    camera.update_images(["LEFT_RGB", "RANGE"], frame)
    # Image panels work; projection is simply unavailable.
    assert "LEFT_RGB" in camera._image_modes
    camera.update_clouds("RANGE", frame)
    assert camera._xyzlut is None
    assert camera.local_xyz(frame) is None
    # An unprojectable camera gets no overlay or frustum either.
    assert camera._object_overlay is None
    assert camera._frustum is None

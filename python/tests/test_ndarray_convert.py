"""Tests for flexible ndarray dtype/order acceptance in selected bindings."""

from typing import Literal, Union

import numpy as np
import numpy.typing as npt
import pytest

from ouster.sdk import viz
from ouster.sdk.core import (
    AutoExposure,
    BeamUniformityCorrector,
    XYZLut,
    XYZLutFloat,
    dewarp,
    interp_pose,
    interp_pose_float,
    transform,
)
from ouster.sdk._bindings.algorithm import normals


FLOAT_DTYPES = [np.float32, np.float64]
ORDERS = ['C', 'F']
FloatDtype = Union[np.float32, np.float64]
ArrayOrder = Literal['C', 'F']


def _as_order(
    arr: npt.ArrayLike,
    order: ArrayOrder,
    dtype: Union[FloatDtype, np.uint32],
) -> np.ndarray:
    out = np.asarray(arr, dtype=dtype)
    if order == 'F':
        out = np.asfortranarray(out)
    return out


def _transform_inputs(dtype: FloatDtype, order: ArrayOrder):
    points = _as_order([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], order, dtype)
    matrix = _as_order(
        [[0.866, -0.5, 0.0, 1.0],
         [0.5, 0.866, 0.0, 2.0],
         [0.0, 0.0, 1.0, -1.0],
         [0.0, 0.0, 0.0, 1.0]],
        order,
        dtype,
    )
    return points, matrix


def _every_other_row(arr: np.ndarray) -> np.ndarray:
    """Return a view with non-unit row stride (neither C- nor F-contiguous)."""
    if arr.shape[0] < 2:
        arr = np.vstack([arr, arr])
    strided = arr[::2, :]
    assert not strided.flags['C_CONTIGUOUS']
    assert not strided.flags['F_CONTIGUOUS']
    return strided


def _dewarp_inputs(dtype: FloatDtype, order: ArrayOrder):
    num_poses = 4
    pts_per_pose = 2
    poses = _as_order(
        [[1, 0, 0, 1, 0, 1, 0, -2, 0, 0, 1, 3, 0, 0, 0, 1]
         for _ in range(num_poses)],
        order,
        dtype,
    ).reshape(num_poses, 4, 4)
    points = _as_order(
        [[i - 3, i + 1, i + 2] for i in range(pts_per_pose * num_poses)],
        order,
        dtype,
    ).reshape(pts_per_pose, num_poses, 3)
    return points, poses


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_transform_accepts_dtype_and_order(order, dtype):
    points, matrix = _transform_inputs(dtype, order)
    expected = transform(
        _as_order(points, 'C', np.float64),
        _as_order(matrix, 'C', np.float64),
    )
    np.testing.assert_almost_equal(transform(points, matrix), expected, decimal=5)

    # test that the result dtype is the same as the points dtype, even if the matrix is a different dtype
    matrix_double = _as_order(matrix, 'C', np.float64)
    result_mixed = transform(points, matrix_double)
    assert result_mixed.dtype == points.dtype
    np.testing.assert_almost_equal(result_mixed, expected, decimal=5)

    with pytest.raises(TypeError, match="points and pose must be floating-point"):
        transform(points.astype(np.uint32), matrix)


@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_transform_accepts_strided_points(dtype):
    points, matrix = _transform_inputs(dtype, 'C')
    strided_points = _every_other_row(np.ascontiguousarray(np.vstack([points, points])))
    expected = transform(
        _as_order(strided_points, 'C', np.float64),
        _as_order(matrix, 'C', np.float64),
    )
    np.testing.assert_almost_equal(
        transform(strided_points, matrix), expected, decimal=5)


def test_transform_rejects_integer_inputs():
    points = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.int32)
    matrix = np.eye(4, dtype=np.float64)
    with pytest.raises(TypeError, match="floating-point"):
        transform(points, matrix)


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_dewarp_accepts_dtype_and_order(order, dtype):
    points, poses = _dewarp_inputs(dtype, order)
    expected = dewarp(
        _as_order(points, 'C', np.float64),
        _as_order(poses, 'C', np.float64),
    )
    result = dewarp(points, poses)
    if dtype == np.float32:
        assert result.dtype == np.float32
    else:
        assert result.dtype == np.float64
    np.testing.assert_allclose(result, expected, rtol=1e-5, atol=1e-6)

    # test that the result dtype is the same as the points dtype, even if the poses are a different dtype
    poses_double = _as_order(poses, 'C', np.float64)
    result_mixed = dewarp(points, poses_double)
    assert result_mixed.dtype == points.dtype
    np.testing.assert_allclose(result_mixed, expected, rtol=1e-5, atol=1e-6)

    with pytest.raises(TypeError, match="points and poses must be floating-point"):
        dewarp(points.astype(np.uint32), poses)


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_interp_pose_accepts_dtype_and_order(order, dtype):
    x_known = _as_order([0.0, 1.0, 2.0], order, dtype)
    x_interp = _as_order([0.5, 1.5], order, dtype)
    poses_known = _as_order(np.tile(np.eye(4), (3, 1, 1)), order, dtype)
    expected = interp_pose(
        _as_order(x_interp, 'C', np.float64),
        _as_order(x_known, 'C', np.float64),
        _as_order(poses_known, 'C', np.float64),
    )
    np.testing.assert_allclose(
        interp_pose(x_interp, x_known, poses_known), expected, rtol=1e-5, atol=1e-6)


@pytest.mark.parametrize('order', ORDERS)
def test_interp_pose_float_accepts_order(order):
    x_known = _as_order([0.0, 1.0, 2.0], order, np.float64)
    x_interp = _as_order([0.5, 1.5], order, np.float64)
    poses_known = _as_order(np.tile(np.eye(4), (3, 1, 1)), order, np.float32)
    expected = interp_pose_float(
        _as_order(x_interp, 'C', np.float64),
        _as_order(x_known, 'C', np.float64),
        _as_order(poses_known, 'C', np.float32),
    )
    result = interp_pose_float(x_interp, x_known, poses_known)
    assert result.dtype == np.float32
    np.testing.assert_allclose(result, expected, rtol=1e-5, atol=1e-6)


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_xyzlut_accepts_range_order(meta, order, dtype):
    lut = XYZLut(meta)
    lut_f = XYZLutFloat(meta)
    h, w = lut.h, lut.w
    range_img = np.arange(h * w, dtype=np.uint32).reshape(h, w)
    if order == 'F':
        range_img = np.asfortranarray(range_img)

    expected = lut(_as_order(range_img, 'C', np.uint32))
    np.testing.assert_allclose(lut(range_img), expected, rtol=0, atol=0)

    expected_f = lut_f(_as_order(range_img, 'C', np.uint32))
    np.testing.assert_allclose(lut_f(range_img), expected_f, rtol=0, atol=0)

    with pytest.raises(TypeError, match="unexpected array dtype"):
        lut(range_img.astype(np.float32))
    with pytest.raises(TypeError, match="unexpected array dtype"):
        lut_f(range_img.astype(np.float32))


@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_auto_exposure_update_in_place(dtype):
    image = np.outer(
        np.linspace(0.2, 1.0, 32, dtype=dtype),
        np.linspace(0.3, 1.0, 32, dtype=dtype),
    )
    original = image.copy()
    ae = AutoExposure()
    assert ae.update(image) is None
    assert not np.allclose(image, original)

    with pytest.raises(TypeError, match="incompatible function arguments"):
        ae.update(image.astype(np.uint16))
    with pytest.raises(TypeError, match="incompatible function arguments"):
        ae.update(image.astype(np.uint32))


@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_beam_uniformity_update_in_place(dtype):
    rng = np.random.default_rng(0)
    image = rng.random((32, 32), dtype=dtype)
    image += np.linspace(0.0, 0.4, 32, dtype=dtype)[:, np.newaxis]
    original = image.copy()
    buc = BeamUniformityCorrector()
    assert buc.update(image) is None
    assert not np.allclose(image, original)

    with pytest.raises(TypeError, match="incompatible function arguments"):
        buc.update(image.astype(np.uint16))


@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_auto_exposure_rgb_updates_in_place(dtype):
    mono = np.linspace(0.2, 1.0, 32 * 32, dtype=dtype).reshape(32, 32)
    image = np.stack([mono, mono * 0.9, mono * 0.8], axis=-1)
    original = image.copy()
    ae = AutoExposure()
    assert ae.update(image) is None
    assert not np.allclose(image, original)


def test_auto_exposure_float16_rgb_returns_float32():
    image = np.random.rand(8, 16, 3).astype(np.float16)
    ae = AutoExposure()
    result = ae.update(image)
    assert result is not None
    assert result.dtype == np.float32
    assert result.shape == (8, 16, 3)


def test_auto_exposure_rejects_non_c_contiguous_image():
    image = np.asfortranarray(np.random.rand(8, 16).astype(np.float32))
    ae = AutoExposure()
    with pytest.raises(TypeError):
        ae.update(image)


def test_auto_exposure_rgb_rejects_invalid_channel_count():
    ae = AutoExposure()
    with pytest.raises(ValueError, match="H x W x 3"):
        ae.update(np.random.rand(8, 16, 4).astype(np.float32))


def _normals_inputs(h: int, w: int, order: ArrayOrder, dtype: FloatDtype):
    xyz = _as_order(np.random.rand(h, w, 3), order, dtype)
    range_img = np.asfortranarray(np.full((h, w), 1000, dtype=np.uint32)) \
        if order == 'F' else np.full((h, w), 1000, dtype=np.uint32)
    origins = _as_order(np.zeros((w, 3)), order, dtype)
    return xyz, range_img, origins


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_normals_accepts_dtype_and_order(order, dtype):
    h, w = 8, 4
    xyz, range_img, origins = _normals_inputs(h, w, order, dtype)
    expected = normals(
        _as_order(xyz, 'C', np.float64),
        _as_order(range_img, 'C', np.uint32),
        _as_order(origins, 'C', np.float64),
    )
    result = normals(xyz, range_img, sensor_origins_xyz=origins)
    np.testing.assert_allclose(result, expected, rtol=1e-5, atol=1e-6)


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_normals_dual_return_accepts_dtype_and_order(order, dtype):
    h, w = 8, 4
    xyz, range_img, origins = _normals_inputs(h, w, order, dtype)
    xyz2 = _as_order(np.random.rand(h, w, 3), order, dtype)
    range2 = np.asfortranarray(np.full((h, w), 900, dtype=np.uint32)) \
        if order == 'F' else np.full((h, w), 900, dtype=np.uint32)
    expected = normals(
        _as_order(xyz, 'C', np.float64),
        _as_order(range_img, 'C', np.uint32),
        _as_order(xyz2, 'C', np.float64),
        _as_order(range2, 'C', np.uint32),
        _as_order(origins, 'C', np.float64),
    )
    result = normals(
        xyz, range_img, xyz2, range2, sensor_origins_xyz=origins)
    np.testing.assert_allclose(result[0], expected[0], rtol=1e-5, atol=1e-6)
    np.testing.assert_allclose(result[1], expected[1], rtol=1e-5, atol=1e-6)


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_cloud_setters_accept_dtype_and_order(order, dtype):
    n = 64
    cloud = viz.Cloud(n)

    xyz = _as_order(np.random.rand(n, 3), order, dtype)
    key = _as_order(np.random.rand(n), order, dtype)
    mask = _as_order(np.random.rand(n, 4), order, dtype)
    palette = _as_order(np.random.rand(256, 3), order, dtype)
    range_img = np.asfortranarray(np.full((8, 8), 1000, dtype=np.uint32)) \
        if order == 'F' else np.full((8, 8), 1000, dtype=np.uint32)

    cloud.set_xyz(xyz)
    cloud.set_key(key)
    cloud.set_mask(mask)
    cloud.set_palette(palette)
    cloud.set_range(range_img)


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_cloud_set_column_poses_accepts_dtype_and_order(meta, order, dtype):
    cloud = viz.Cloud(meta)
    w = cloud.cols
    column_poses = _as_order(np.tile(np.eye(4), (w, 1, 1)).reshape(-1), order, dtype)
    cloud.set_column_poses(column_poses)


@pytest.mark.parametrize('order', ORDERS)
def test_cloud_set_key_and_mask_uint8(order):
    n = 64
    cloud = viz.Cloud(n)
    key = np.asfortranarray(np.random.randint(0, 255, n, dtype=np.uint8)) \
        if order == 'F' else np.random.randint(0, 255, n, dtype=np.uint8)
    mask = np.asfortranarray(np.random.randint(0, 255, (n, 4), dtype=np.uint8)) \
        if order == 'F' else np.random.randint(0, 255, (n, 4), dtype=np.uint8)
    cloud.set_key(key)
    cloud.set_mask(mask)

    with pytest.raises(TypeError, match=r"key must be uint8 or floating-point"):
        cloud.set_key(key.astype(np.uint16))
    with pytest.raises(TypeError, match=r"key must be uint8 or floating-point"):
        cloud.set_key(key.astype(np.uint32))

    with pytest.raises(TypeError, match=r"mask must be uint8 or floating-point"):
        cloud.set_mask(mask.astype(np.uint16))
    with pytest.raises(TypeError, match=r"mask must be uint8 or floating-point"):
        cloud.set_mask(mask.astype(np.uint32))


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_image_setters_accept_dtype_and_order(order, dtype):
    image = viz.Image()
    mono = _as_order(np.random.rand(16, 32), order, dtype)
    mask = _as_order(np.random.rand(16, 32, 4), order, dtype)
    palette = _as_order(np.random.rand(256, 3), order, dtype)
    image.set_image(mono)
    image.set_mask(mask)
    image.set_palette(palette)

    with pytest.raises(TypeError, match=r"image must be uint8 or floating-point"):
        image.set_image(mono.astype(np.uint16))
    with pytest.raises(TypeError, match=r"image must be uint8 or floating-point"):
        image.set_image(mono.astype(np.uint32))


@pytest.mark.parametrize('order', ORDERS)
def test_image_set_image_uint8(order):
    image = viz.Image()
    mono = np.asfortranarray(np.random.randint(0, 255, (16, 32), dtype=np.uint8)) \
        if order == 'F' else np.random.randint(0, 255, (16, 32), dtype=np.uint8)
    image.set_image(mono)


@pytest.mark.parametrize('order', ORDERS)
@pytest.mark.parametrize('dtype', FLOAT_DTYPES)
def test_lines_set_points_accepts_dtype_and_order(order, dtype):
    lines = viz.Lines(np.eye(4, dtype=np.float64), (1.0, 1.0, 1.0, 1.0))
    points = _as_order(np.random.rand(10, 3), order, dtype)
    lines.set_points(points)

"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.

Unit normal computation tests
"""

import pytest
import numpy as np

from ouster.sdk import open_source
from ouster.sdk.core import (
    ChanField,
    XYZLut,
    normals,
    destagger,
)


@pytest.fixture
def input_car_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "single_scan_016.osf"


@pytest.fixture
def input_room_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "normals_test_data.osf"


@pytest.fixture
def input_normal_cases_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "normals_cases.osf"


# Tests both single-return and dual-return normals in sensor coordinate.
def test_normals_on_car_osf(input_car_osf_file):
    src = open_source(str(input_car_osf_file))
    scan_iter = iter(src)
    scans = next(scan_iter)
    scan = scans[0]
    sensor_info = src.sensor_info[0]

    # Compute normals
    scan.sensor_info = sensor_info
    xyzlut = XYZLut(sensor_info)
    h, w = scan.h, scan.w

    range_staggered = scan.field(ChanField.RANGE)
    xyz_staggered = xyzlut(range_staggered).reshape(h, w, 3)
    range_destaggered = destagger(sensor_info, range_staggered)
    xyz_destaggered = destagger(sensor_info, xyz_staggered)
    sensor_origins_xyz = np.zeros((w, 3))

    # Single-return normals check
    normals_destaggered = normals(
        xyz_destaggered, range_destaggered, sensor_origins_xyz=sensor_origins_xyz
    )
    assert normals_destaggered.shape == (h, w, 3)

    # Check that valid normals are unit length
    norms = np.linalg.norm(normals_destaggered, axis=2)
    valid = norms > 0
    if np.any(valid):
        np.testing.assert_allclose(norms[valid], 1.0, atol=1e-6)

    range2_staggered = scan.field(ChanField.RANGE2)
    xyz2_staggered = xyzlut(range2_staggered).reshape(h, w, 3)
    range2_destaggered = destagger(sensor_info, range2_staggered)
    xyz2_destaggered = destagger(sensor_info, xyz2_staggered)

    # Dual-return normals check
    normals_first, normals_second = normals(
        xyz_destaggered,
        range_destaggered,
        xyz2_destaggered,
        range2_destaggered,
        sensor_origins_xyz=sensor_origins_xyz,
    )

    assert normals_first.shape == (h, w, 3)
    assert normals_second.shape == (h, w, 3)

    norms_first = np.linalg.norm(normals_first, axis=2)
    valid_first = norms_first > 0
    if np.any(valid_first):
        np.testing.assert_allclose(norms_first[valid_first], 1.0, atol=1e-6)

    norms_second = np.linalg.norm(normals_second, axis=2)
    valid_second = norms_second > 0
    if np.any(valid_second):
        np.testing.assert_allclose(norms_second[valid_second], 1.0, atol=1e-6)

    # Regression checks for known pixel normals.
    expected_first_samples = [
        ((67, 798), np.array([0.063, 0.998, -0.012], dtype=np.float64)),
        ((68, 204), np.array([0.025, -0.999, 0.028], dtype=np.float64)),
        ((100, 512), np.array([-0.032, 0.017, 0.999], dtype=np.float64)),
    ]
    expected_second_samples = [
        ((58, 791), np.array([-0.009, 0.983, -0.182], dtype=np.float64)),
        ((46, 153), np.array([0.569, -0.823, -0.007], dtype=np.float64)),
    ]

    for (row, col), expected_vector in expected_first_samples:
        if row < h and col < w:
            np.testing.assert_allclose(
                normals_destaggered[row, col],
                expected_vector,
                atol=1e-3,
                rtol=0,
            )

    for (row, col), expected_vector in expected_second_samples:
        if row < h and col < w:
            np.testing.assert_allclose(
                normals_second[row, col],
                expected_vector,
                atol=1e-3,
                rtol=0,
            )


# Tests normals on known surfaces with boundaries in a room scan.
def test_normals_cube_boundaries(input_room_osf_file):
    src = open_source(str(input_room_osf_file))
    scans = next(iter(src))
    scan = scans[0]
    assert scan is not None
    info = src.sensor_info[0]
    h, w = info.h, info.w

    xyzlut = XYZLut(info)
    range_staggered = scan.field(ChanField.RANGE)
    xyz_staggered = xyzlut(range_staggered).reshape(h, w, 3)

    range_destaggered = destagger(info, range_staggered)
    xyz_destaggered = destagger(info, xyz_staggered)
    sensor_origins_xyz = np.zeros((w, 3))
    normals_destaggered = normals(
        xyz_destaggered, range_destaggered, sensor_origins_xyz=sensor_origins_xyz
    )
    normals_staggered = destagger(info, normals_destaggered, inverse=True)

    surfaces = {
        "wall_pos_x": ([1, 127], [0, 1023], np.array([1.0, 0.0, 0.0])),
        "wall_neg_x": ([1, 127], [357, 667], np.array([-1.0, 0.0, 0.0])),
        "wall_pos_y": ([1, 127], [613, 923], np.array([0.0, 1.0, 0.0])),
        "wall_neg_y": ([1, 127], [101, 411], np.array([0.0, -1.0, 0.0])),
        "ceiling": ([0, 13], [0, 1023], np.array([0.0, 0.0, -1.0])),
        "floor": ([116, 127], [48, 1008], np.array([0.0, 0.0, 1.0])),
    }

    for name, (row_range, col_range, expected_normal) in surfaces.items():
        row_min, row_max = row_range
        col_min, col_max = col_range
        rows = slice(row_min, row_max + 1)
        cols = slice(col_min, col_max + 1)

        region_normals = normals_staggered[rows, cols]
        if region_normals.size == 0:
            pytest.fail(f"{name}: empty region for provided bounds.")

        norms = np.linalg.norm(region_normals, axis=-1)
        valid = norms > 0
        assert np.any(valid), f"{name}: no valid normals"

        normalized = np.zeros_like(region_normals)
        normalized[valid] = region_normals[valid] / norms[valid, None]

        alignment_threshold = float(np.cos(np.deg2rad(0.5)))  # ~0.5° tolerance
        # cross_product_align holds the per-pixel cosine between the computed normal
        # and the expected plane normal.
        cross_product_align = np.tensordot(
            normalized, expected_normal, axes=([2], [0])
        )
        plane_mask = (cross_product_align > alignment_threshold) & valid
        assert np.any(
            plane_mask
        ), f"{name}: insufficient pixels aligned with expected normal"
        aligned_normals = normalized[plane_mask]
        # cos_aligned checks that all selected normals remain within the same tolerance.
        cos_aligned = np.tensordot(aligned_normals, expected_normal, axes=([1], [0]))
        assert (
            np.min(cos_aligned) > alignment_threshold
        ), f"{name}: normals deviate from expected direction"


def test_normals_target_distance_invalid():
    """It should raise ValueError when target_distance is negative."""
    xyz_destaggered = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]])
    range_destaggered = np.array([[0.0, 1.0], [1.0, 1.0]])
    sensor_origins_xyz = np.zeros((2, 3))
    # TODO should be ValueError
    with pytest.raises(RuntimeError, match=r"target_distance_m must be positive"):
        normals(
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            0.017453292519943295,
            -100
        )
    # TODO should be ValueError
    with pytest.raises(RuntimeError, match=r"target_distance_m must be positive"):
        normals(
            xyz_destaggered,
            range_destaggered,
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            0.017453292519943295,
            -100
        )


def test_normals_target_distance_sensor_origins_wrong_shape():
    """It should raise ValueError if sensor_origins_xyz is not of shape (N, 3)"""
    xyz_destaggered = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]])
    range_destaggered = np.array([[0.0, 1.0], [1.0, 1.0]])
    sensor_origins_xyz = np.zeros((0, 0))
    with pytest.raises(ValueError, match=r"Expected a 2D array with shape \(N, 3\)"):
        normals(
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            0.017453292519943295,
            100
        )
    with pytest.raises(ValueError, match=r"Expected a 2D array with shape \(N, 3\)"):
        normals(
            xyz_destaggered,
            range_destaggered,
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            0.017453292519943295,
            100
        )


def test_normals_target_distance_sensor_origins_doesnt_match_image_width():
    """It should raise RuntimeError if sensor_origins_xyz first dimension doesn't match image width"""
    xyz_destaggered = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]])
    range_destaggered = np.array([[0.0, 1.0], [1.0, 1.0]])
    sensor_origins_xyz = np.zeros((0, 3))
    with pytest.raises(RuntimeError, match=r"normals: sensor_origins size must match image width"):
        normals(
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            0.017453292519943295,
            100
        )
    with pytest.raises(RuntimeError, match=r"normals: sensor_origins size must match image width"):
        normals(
            xyz_destaggered,
            range_destaggered,
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            0.017453292519943295,
            100
        )


def test_normals_xyz_and_range_size_mismatch():
    """It should raise RuntimeError if xyz_destaggered and range_destaggered have different shapes"""
    xyz_destaggered = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]])
    range_destaggered = np.array([[0.0, 1.0]])
    sensor_origins_xyz = np.zeros((2, 3))
    with pytest.raises(RuntimeError, match=r"normals: xyz dimensions mismatch"):
        normals(
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            0.017453292519943295,
            100
        )
    with pytest.raises(RuntimeError, match=r"normals: xyz dimensions mismatch"):
        normals(
            xyz_destaggered,
            range_destaggered,
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            0.017453292519943295,
            100
        )


def test_normals_angle_of_incidence():
    """It should raise ValueError when angle_of_incidence_deg is not positive."""
    xyz_destaggered = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]])
    range_destaggered = np.array([[0.0, 1.0], [1.0, 1.0]])
    sensor_origins_xyz = np.zeros((2, 3))
    expected_err = r"normals: min_angle_of_incidence_rad must be positive"
    with pytest.raises(RuntimeError, match=expected_err):
        normals(
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            -0.1,
            100
        )
    with pytest.raises(RuntimeError, match=expected_err):
        normals(
            xyz_destaggered,
            range_destaggered,
            xyz_destaggered,
            range_destaggered,
            sensor_origins_xyz,
            1,
            -0.1,
            100
        )


def test_normals():
    xyz_destaggered = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]])
    range_destaggered = np.array([[0.0, 1.0], [1.0, 1.0]])
    sensor_origins_xyz = np.zeros((2, 3))
    result = normals(
        xyz_destaggered,
        range_destaggered,
        sensor_origins_xyz,
        1,
        0.1,
        100
    )
    assert np.allclose(result,
        # this is a regression check, not a geometric correctness check
        np.array([
            [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]],
            [[0.0, -1.0, 0.0], [-0.70710678, -0.70710678, 0.0]]
        ])
    )
    result, result2 = normals(
        xyz_destaggered,
        range_destaggered,
        xyz_destaggered,
        range_destaggered,
        sensor_origins_xyz,
        1,
        0.1,
        100
    )
    assert np.allclose(result,
        # this is a regression check, not a geometric correctness check
        np.array([
            [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]],
            [[0.0, -1.0, 0.0], [-0.70710678, -0.70710678, 0.0]]
        ])
    )


def test_normals_2():
    xyz_destaggered = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]])
    range_destaggered = np.array([[0.0, 1.0], [0.0, 0.0]])
    sensor_origins_xyz = np.zeros((2, 3))
    result = normals(
        xyz_destaggered,
        range_destaggered,
        sensor_origins_xyz,
        1,
        0.1,
        100
    )
    assert np.allclose(result,
        # this is a regression check, not a geometric correctness check
        np.array([
            [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]],
            [[0.0, 0.0, 0.0], [0, 0.0, 0.0]]
        ])
    )
    result, result2 = normals(
        xyz_destaggered,
        range_destaggered,
        xyz_destaggered,
        range_destaggered,
        sensor_origins_xyz,
        1,
        0.1,
        100
    )
    assert np.allclose(result,
        # this is a regression check, not a geometric correctness check
        np.array([
            [[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]],
            [[0.0, 0.0, 0.0], [0, 0.0, 0.0]]
        ])
    )

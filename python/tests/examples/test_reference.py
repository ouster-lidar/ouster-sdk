import numpy as np

from types import SimpleNamespace

from ouster.sdk import core
from ouster.sdk.examples import reference


def _fake_metadata(h: int, w: int, prod_line: str = "OS-1-64"):
    """Build minimal metadata with identity transforms and zero angles."""
    return SimpleNamespace(
        beam_to_lidar_transform=np.eye(4),
        lidar_to_sensor_transform=np.eye(4),
        beam_azimuth_angles=np.zeros(h),
        beam_altitude_angles=np.zeros(h),
        lidar_origin_to_beam_origin_mm=0,
        prod_line=prod_line,
    )


def _make_scan(w: int, h: int, ranges_mm) -> core.LidarScan:
    scan = core.LidarScan(w, h)
    scan.measurement_id[:] = np.arange(w)
    scan.field(core.ChanField.RANGE)[:] = ranges_mm
    return scan


def test_xyz_proj_beam_to_sensor_transform_identity_angles():
    w, h = 2, 2
    meta = _fake_metadata(h, w)
    ranges = np.array([[1000, 2000], [3000, 4000]], dtype=np.float64)
    scan = _make_scan(w, h, ranges)

    xyz = reference.xyz_proj_beam_to_sensor_transform(meta, scan)

    expected = np.array([
        [[1.0, 0.0, 0.0], [-2.0, 0.0, 0.0]],
        [[3.0, 0.0, 0.0], [-4.0, 0.0, 0.0]],
    ])
    assert xyz.shape == (h, w, 3)
    assert np.allclose(xyz, expected)


def test_xyz_proj_origin_to_origin_mm_identity_angles():
    w, h = 2, 2
    meta = _fake_metadata(h, w)
    ranges = np.array([[1000, 2000], [3000, 4000]], dtype=np.float64)
    scan = _make_scan(w, h, ranges)

    xyz = reference.xyz_proj_origin_to_origin_mm(meta, scan)

    expected = np.array([
        [[1.0, 0.0, 0.0], [-2.0, 0.0, 0.0]],
        [[3.0, 0.0, 0.0], [-4.0, 0.0, 0.0]],
    ])
    assert xyz.shape == (h, w, 3)
    assert np.allclose(xyz, expected)


def test_xyz_proj_origin_to_origin_mm_os_dome_returns_zeros():
    w, h = 2, 2
    meta = _fake_metadata(h, w, prod_line="OS-DOME")
    ranges = np.ones((h, w), dtype=np.float64)
    scan = _make_scan(w, h, ranges)

    xyz = reference.xyz_proj_origin_to_origin_mm(meta, scan)

    assert np.array_equal(xyz, np.zeros((w * h, 4)))


def test_destagger_rolls_rows():
    field = np.array([
        [0, 1, 2],
        [3, 4, 5],
    ])
    shifts = [1, -1]

    destaggered = reference.destagger(shifts, field)

    expected = np.array([
        [2, 0, 1],  # rolled right by 1
        [4, 5, 3],  # rolled left by 1
    ])
    assert np.array_equal(destaggered, expected)

"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from copy import copy

import numpy as np
import pytest

from ouster import client
import ouster.client._digest as digest
from ouster.sdk.examples import reference


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_xyz_lut_dims(stream_digest: digest.StreamDigest, meta: client.SensorInfo) -> None:
    """Check that (in)valid dimensions are handled when creating xyzlut."""
    w = meta.format.columns_per_frame
    h = meta.format.pixels_per_column

    client.XYZLut(meta)

    meta1 = copy(meta)
    client.XYZLut(meta1)

    # should fail when no. rows doesn't match beam angle arrays
    meta1.format.pixels_per_column = 0
    with pytest.raises(ValueError):
        client.XYZLut(meta1)

    meta1.format.pixels_per_column = h + 1
    with pytest.raises(ValueError):
        client.XYZLut(meta1)

    meta1.format.pixels_per_column = h - 1
    with pytest.raises(ValueError):
        client.XYZLut(meta1)

    meta2 = copy(meta)
    client.XYZLut(meta2)

    # this is nonsensical, but still good to check for memory errors
    meta2.format.columns_per_frame = w + 1
    client.XYZLut(meta2)

    meta2.format.columns_per_frame = w - 1
    client.XYZLut(meta2)

    meta2.format.columns_per_frame = 0
    with pytest.raises(ValueError):
        client.XYZLut(meta2)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_xyz_lut_angles(stream_digest: digest.StreamDigest, meta: client.SensorInfo) -> None:
    """Check that invalid beam angle dimensions are handled by xyzlut."""
    meta1 = copy(meta)
    client.XYZLut(meta1)

    meta1.beam_azimuth_angles = meta1.beam_azimuth_angles + [0.0]
    with pytest.raises(ValueError):
        client.XYZLut(meta1)

    meta1.beam_azimuth_angles = meta1.beam_azimuth_angles[:-2]
    with pytest.raises(ValueError):
        client.XYZLut(meta1)

    meta1.beam_azimuth_angles = []
    with pytest.raises(ValueError):
        client.XYZLut(meta1)

    meta2 = copy(meta)
    client.XYZLut(meta2)

    meta2.beam_azimuth_angles = meta1.beam_altitude_angles + [0.0]
    with pytest.raises(ValueError):
        client.XYZLut(meta2)

    meta2.beam_azimuth_angles = meta1.beam_altitude_angles[:-2]
    with pytest.raises(ValueError):
        client.XYZLut(meta2)

    meta2.beam_azimuth_angles = []
    with pytest.raises(ValueError):
        client.XYZLut(meta2)


@pytest.mark.parametrize('test_key', ['legacy-2.0'])
def test_xyz_lut_scan_dims(stream_digest: digest.StreamDigest, meta: client.SensorInfo) -> None:
    """Check that (in)valid lidar scan dimensions are handled by xyzlut."""
    w = meta.format.columns_per_frame
    h = meta.format.pixels_per_column

    xyzlut = client.XYZLut(meta)

    assert xyzlut(client.LidarScan(h, w)).shape == (h, w, 3)

    with pytest.raises(ValueError):
        xyzlut(client.LidarScan(h + 1, w))

    with pytest.raises(ValueError):
        xyzlut(client.LidarScan(h, w - 1))


def test_xyz_calcs(stream_digest: digest.StreamDigest,
                   scan: client.LidarScan, meta: client.SensorInfo) -> None:
    """Compare the optimized xyz projection to a reference implementation."""

    # compute 3d points using reference implementation
    xyz_beam_to_sensor_docs = reference.xyz_proj_beam_to_sensor_transform(meta, scan)

    # transform data to 3d points using optimized implementation
    xyzlut = client.XYZLut(meta)
    xyz_from_lut = xyzlut(scan)

    assert np.allclose(xyz_beam_to_sensor_docs, xyz_from_lut)

    if "OS-DOME" not in meta.prod_line:
        # for non-os-dome the old reference implementation should be correct as well
        xyz_origin_to_origin_mm_docs = reference.xyz_proj_origin_to_origin_mm(meta, scan)
        assert np.allclose(xyz_from_lut, xyz_origin_to_origin_mm_docs)


def test_xyz_range(stream_digest: digest.StreamDigest, scan: client.LidarScan,
                   meta: client.SensorInfo) -> None:
    """Test that projection works on an ndarrays as well."""
    xyzlut = client.XYZLut(meta)

    range = scan.field(client.ChanField.RANGE)
    xyz_from_scan = xyzlut(scan)
    xyz_from_range = xyzlut(range)
    assert np.array_equal(xyz_from_scan, xyz_from_range)


def test_xyz_range_dtype(stream_digest: digest.StreamDigest, scan: client.LidarScan,
                         meta: client.SensorInfo) -> None:
    """Test that projection works on an ndarrays of different dtypes."""
    xyzlut = client.XYZLut(meta)

    range = scan.field(client.ChanField.RANGE)
    range[:] = range.astype(np.uint8)
    xyz_from_scan = xyzlut(scan)
    xyz_from_range_8 = xyzlut(range.astype(np.uint8))
    xyz_from_range_16 = xyzlut(range.astype(np.uint16))
    xyz_from_range_32 = xyzlut(range.astype(np.uint32))
    xyz_from_range_64 = xyzlut(range.astype(np.uint64))

    assert np.array_equal(xyz_from_scan, xyz_from_range_8)
    assert np.array_equal(xyz_from_scan, xyz_from_range_16)
    assert np.array_equal(xyz_from_scan, xyz_from_range_32)
    assert np.array_equal(xyz_from_scan, xyz_from_range_64)

from copy import copy
from itertools import product
from math import cos, pi, sin
from os import path

import numpy as np
import pytest

from ouster import client
import ouster.client._digest as digest

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


def xyz_ref(metadata: client.SensorInfo, scan: client.LidarScan) -> np.ndarray:
    """Computes a point cloud from a scan as numpy array.

    This is a reference implementation that follows the calculations from
    `Section 3.1.2`_ of the Software User Manual exactly. Output is a point
    cloud in the *sensor frame* with points arranged in column-major order,
    with coordinates in meters.

    Args:
        metadata: Sensor metadata associated with the scan
        scan: A frame of lidar data

    Returns:
        A H x W x 3 array of point coordinates

    .. _Section 3.1.2: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf#a
    """

    # use homogeneous coordinates for convenient transformation
    xyz = np.zeros((scan.w * scan.h, 4))

    # iterate over each measurement channel/row and measurement block/column
    for u, v in product(range(scan.h), range(scan.w)):

        r = scan.field(client.ChanField.RANGE)[u, v]
        n = metadata.lidar_origin_to_beam_origin_mm

        encoder_count = scan.headers[v].encoder
        theta_encoder = 2.0 * pi * (1.0 - encoder_count / 90112.0)
        theta_azimuth = -2.0 * pi * (metadata.beam_azimuth_angles[u] / 360.0)
        phi = 2.0 * pi * (metadata.beam_altitude_angles[u] / 360.0)

        # zero ranges represent no return; avoid applying offsets to these
        if r == 0.0:
            continue

        # compute point coordinates in the lidar frame
        x = (r - n) * cos(theta_encoder +
                          theta_azimuth) * cos(phi) + n * cos(theta_encoder)
        y = (r - n) * sin(theta_encoder +
                          theta_azimuth) * cos(phi) + n * sin(theta_encoder)
        z = (r - n) * sin(phi)

        # insert into xyz; point order is row-major to match input scan
        xyz[u * scan.w + v] = [x, y, z, 1]

    # transform from lidar to sensor frame and scale to meters
    xyz_sensor = xyz @ metadata.lidar_to_sensor_transform.T
    return xyz_sensor[:, :3].reshape(scan.h, scan.w, 3) * 0.001


@pytest.fixture
def stream_digest() -> digest.StreamDigest:
    # load test scan and metadata
    digest_path = path.join(DATA_DIR, "os-992011000121_digest.json")

    with open(digest_path, 'r') as f:
        stream_digest = digest.StreamDigest.from_json(f.read())

    return stream_digest


@pytest.fixture
def scan(stream_digest: digest.StreamDigest) -> client.LidarScan:
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")

    with open(bin_path, 'rb') as b:
        source = digest.LidarBufStream(b, stream_digest.meta)
        scans = client.Scans(source)
        scan = next(iter(scans))

    return scan


def test_xyz_lut_dims(stream_digest: digest.StreamDigest) -> None:
    """Check that (in)valid dimensions are handled when creating xyzlut."""
    meta = stream_digest.meta
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


def test_xyz_lut_angles(stream_digest: digest.StreamDigest) -> None:
    """Check that invalid beam angle dimensions are handled by xyzlut."""
    meta = stream_digest.meta

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


def test_xyz_lut_scan_dims(stream_digest: digest.StreamDigest) -> None:
    """Check that (in)valid lidar scan dimensions are handled by xyzlut."""
    meta = stream_digest.meta
    w = meta.format.columns_per_frame
    h = meta.format.pixels_per_column

    xyzlut = client.XYZLut(meta)

    assert xyzlut(client.LidarScan(w, h)).shape == (h, w, 3)

    with pytest.raises(ValueError):
        xyzlut(client.LidarScan(w, h + 1))

    with pytest.raises(ValueError):
        xyzlut(client.LidarScan(w - 1, h))


def test_xyz_calcs(stream_digest: digest.StreamDigest,
                   scan: client.LidarScan) -> None:
    """Compare the optimized xyz projection to a reference implementation."""

    # compute 3d points using reference implementation
    xyz_from_docs = xyz_ref(stream_digest.meta, scan)

    # transform data to 3d points using optimized implementation
    xyzlut = client.XYZLut(stream_digest.meta)
    xyz_from_lut = xyzlut(scan)

    assert np.allclose(xyz_from_docs, xyz_from_lut)

from itertools import product
from math import cos, pi, sin
from os import path

import numpy as np

from ouster import client
import ouster.client._digest as digest

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


def xyz_ref(metadata: client.SensorInfo, scan: client.LidarScan) -> np.ndarray:
    """Computes a point cloud from a scan as numpy array.

    This is a reference implementation that follows the calculations from
    `Section 3.1.2`_ of the Software User Manual exactly. Output is a point
    cloud in the *sensor frame* with points arranged in row-major order.

    Args:
        metadata: Sensor metadata associated with the scan
        scan: A frame of lidar data

    Returns:
        A Nx3 numpy array representing N points as row vectors

    .. _Section 3.1.2: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf#a
    """

    # use homogeneous coordinates for convenient transformation
    xyz = np.zeros((scan.w * scan.h, 4))

    # iterate over each measurement block/column and channel/row
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

        # insert into xyz; point order is row-major
        xyz[u * scan.w + v] = [x, y, z, 1]

    # transform from lidar to sensor frame and scale to meters
    xyz_sensor = xyz @ metadata.lidar_to_sensor_transform.T
    return xyz_sensor[:, :3] * 0.001


def test_xyz_calcs() -> None:
    """Compare the optimized xyz projection to a reference implementation."""

    # load test scan and metadata
    digest_path = path.join(DATA_DIR, "os-992011000121_digest.json")
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")

    with open(digest_path, 'r') as f:
        stream = digest.StreamDigest.from_json(f.read())

    with open(bin_path, 'rb') as b:
        source = digest.LidarBufStream(b, stream.meta)
        scans = client.Scans(source)
        scan = next(iter(scans))

    # compute 3d points using reference implementation
    xyz_from_docs = xyz_ref(stream.meta, scan)

    # transform data to 3d points using optimized implementation
    xyzlut = client.XYZLut(stream.meta)
    xyz_from_lut = xyzlut(scan)

    assert np.allclose(xyz_from_docs, xyz_from_lut)

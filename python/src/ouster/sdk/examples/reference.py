"""Reference implementations of common operations."""

from itertools import product
from math import cos, pi, sin
from typing import List

import numpy as np

from ouster import client


def xyz_proj(metadata: client.SensorInfo,
             scan: client.LidarScan) -> np.ndarray:
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

        encoder_count = scan.header(client.ColHeader.ENCODER_COUNT)[v]
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


def destagger(pixel_shift_by_row: List[int], field: np.ndarray) -> np.ndarray:
    """Reference implementation for dsetaggering a field of data.

    In the default staggered representation, each column corresponds to a
    single timestamp. In the destaggered representation, each column
    corresponds to a single azimuth angle, compensating for the azimuth offset
    of each beam.

    Destaggering is used for visualizing lidar data as an image or for
    algorithms that exploit the structure of the lidar data, such as
    beam_uniformity in ouster_viz, or computer vision algorithms.

    Args:
        pixel_shift_by_row: List of pixel shifts by row from sensor metadata
        field: Staggered data as a H x W numpy array

    Returns:
        Destaggered data as a H x W numpy array
    """

    destaggered = np.zeros(field.shape)
    nrows = field.shape[0]

    # iterate over every row and apply pixel shift
    for u in range(nrows):
        destaggered[u, :] = np.roll(field[u, :], pixel_shift_by_row[u])

    return destaggered

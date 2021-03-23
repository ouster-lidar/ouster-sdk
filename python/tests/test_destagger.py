from os import path

import numpy as np
from typing import List

from ouster import client
import ouster.client._digest as digest

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


def destaggered_ref(pixel_shift_by_row: List[int],
                    field: np.ndarray) -> np.ndarray:
    """Reference implementation for dsetaggering a field of data.

    In the default staggered representation, each column corresponds to a
    single timestamp. In the destaggered representation, each column
    corresponds to a single azimuth angle, compensating for the azimuth offset
    of each beam.

    Destaggering is used for visualizing lidar data as an image or for
    algorithms that exploit the structure of the lidar data, such as
    beam_uniformity in ouster_viz, or computer vision algorithms.

    Args:
        pixel_shift_by_row: list of pixel shifts by row from sensor metadata
        field: staggered h x w view of a specific field
 """

    destaggered = np.zeros(field.shape)

    nrows = field.shape[0]

    # iterate over every row and apply pixel shift
    for u in range(nrows):
        destaggered[u, :] = np.roll(field[u, :], pixel_shift_by_row[u])

    return destaggered


def test_destagger() -> None:
    """Compare client destagger function to reference implementation."""

    # load test scan and metadata
    digest_path = path.join(DATA_DIR, "os-992011000121_digest.json")
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")

    with open(digest_path, 'r') as f:
        stream = digest.StreamDigest.from_json(f.read())

    with open(bin_path, 'rb') as b:
        source = digest.LidarBufStream(b, stream.meta)
        scans = client.Scans(source)
        scan = next(iter(scans))

    # get destaggered range field using reference implementation
    destagger_ref = destaggered_ref(stream.meta.format.pixel_shift_by_row,
                                    scan.field(client.ChanField.RANGE))

    # obtain destaggered range field using client implemenation
    destagger_client = scan.destaggered(stream.meta, client.ChanField.RANGE)

    assert np.array_equal(destagger_ref, destagger_client)

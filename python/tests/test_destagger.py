from os import path

import numpy as np
import pytest

from ouster import client
from ouster.sdk.examples import reference

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


@pytest.fixture(scope="module")
def meta() -> client.SensorInfo:
    # load test scan and metadata
    digest_path = path.join(DATA_DIR, "os-992011000121_digest.json")
    with open(digest_path, 'r') as f:
        return digest.StreamDigest.from_json(f.read()).meta


@pytest.fixture(scope="module")
def scan(meta) -> client.LidarScan:
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")
    with open(bin_path, 'rb') as b:
        source = digest.LidarBufStream(b, meta)
        scans = client.Scans(source)
        return next(iter(scans))


@pytest.mark.parametrize("dtype", [
    np.uint8, np.uint16, np.uint32, np.uint64, np.int8, np.int16, np.int32,
    np.int64, np.float32, np.float64
])
def test_destagger_type_good(meta, dtype) -> None:
    """Check that destaggering preserves dtype."""
    h = meta.format.pixels_per_column
    w = meta.format.columns_per_frame

    assert client.destagger(meta, np.zeros((h, w), dtype)).dtype == dtype
    assert client.destagger(meta, np.zeros((h, w, 2), dtype)).dtype == dtype


@pytest.mark.parametrize("shape", [(32, 512), (32, 512, 1), (64, 1024, 2),
                                   (128, 2048, 10)])
def test_destagger_shape_good(meta, shape) -> None:
    """Check that (de)staggering preserves shape."""
    assert client.destagger(meta, np.zeros(shape)).shape == shape
    assert client.destagger(meta, np.zeros(shape), inverse=True).shape == shape


def test_destagger_shape_bad(meta) -> None:
    """Check that arrays of the wrong shape are rejected."""
    h = meta.format.pixels_per_column
    w = meta.format.columns_per_frame

    with pytest.raises(ValueError):
        client.destagger(meta, np.zeros((0, w)))
    with pytest.raises(ValueError):
        client.destagger(meta, np.zeros((h, 0, 2)))

    with pytest.raises(ValueError):
        client.destagger(meta, np.zeros((h, w + 1)))
    with pytest.raises(ValueError):
        client.destagger(meta, np.zeros((h - 1, w)))
    with pytest.raises(ValueError):
        client.destagger(meta, np.zeros((h, w - 1, 1)))
    with pytest.raises(ValueError):
        client.destagger(meta, np.zeros((h + 1, w, 2)))


def test_destagger_inverse(meta) -> None:
    """Check that stagger/destagger are inverse operations."""
    h = meta.format.pixels_per_column
    w = meta.format.columns_per_frame
    a = np.arange(h * w).reshape((h, w))

    b = client.destagger(meta, a, inverse=True)
    c = client.destagger(meta, b)
    assert np.array_equal(a, c)

    d = client.destagger(meta, a)
    e = client.destagger(meta, d, inverse=True)
    assert np.array_equal(a, e)


def test_destagger_xyz(meta, scan) -> None:
    """Check that we can destagger the output of xyz projection."""
    h = meta.format.pixels_per_column
    w = meta.format.columns_per_frame
    xyz = client.XYZLut(meta)(scan)

    destaggered = client.destagger(meta, xyz)
    assert destaggered.shape == (h, w, 3)


def test_destagger_correct(meta, scan) -> None:
    """Compare client destagger function to reference implementation."""

    # get destaggered range field using reference implementation
    destagger_ref = reference.destagger(meta.format.pixel_shift_by_row,
                                        scan.field(client.ChanField.RANGE))

    # obtain destaggered range field using client implemenation
    destagger_client = client.destagger(meta,
                                        scan.field(client.ChanField.RANGE))

    assert np.array_equal(destagger_ref, destagger_client)


def test_destagger_correct_multi(meta, scan) -> None:
    """Compare client destagger function to reference on stacked fields."""

    near_ir = scan.field(client.ChanField.NEAR_IR)
    near_ir_stacked = np.repeat(near_ir[..., None], 5, axis=2)

    ref = reference.destagger(meta.format.pixel_shift_by_row, near_ir)
    ref_stacked = np.repeat(ref[..., None], 5, axis=2)

    destaggered_stacked = client.destagger(meta, near_ir_stacked)

    assert near_ir_stacked.dtype == np.uint32
    assert destaggered_stacked.dtype == np.uint32
    assert np.array_equal(ref_stacked, destaggered_stacked)

# Please note that the tests in `tests/integration` folder cannot be run on your system
# as they use external data (not included in the repository).

from contextlib import closing
import pytest
import numpy as np
import random
import os
import unittest

from typing import Tuple

from ouster.sdk import client, pcap

from ouster.sdk.client import _utils
import ouster.sdk.client._digest as digest

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "../pcaps")


def genparams():
    """Generate various autoexposures."""
    def mkparam(lo_percentile, hi_percentile, update_every):
        return pytest.param(
            (lo_percentile, hi_percentile, update_every),
            id=f"{lo_percentile}-{hi_percentile}-{update_every}")

    def generate():
        param_list = []
        param_list.append((0.1, 0.1, 1))  # near 'default', update every frame
        param_list.append((0.05, 0.1, 1))  # asymmetrical, update every frame
        param_list.append((0.1, 0.02, 1))  # asymmetrical, update every frame
        param_list.append((0.0, 0.0, 1))  # zero percentile, update every frame
        param_list.append((0.3, 0.3, 1))  # aggressive, update every frame

        return [
            mkparam(lo_percentile, hi_percentile, update_every)
            for lo_percentile, hi_percentile, update_every in param_list
        ]

    return generate()


@pytest.fixture(params=genparams())
def ae_with_params(
        request) -> Tuple[_utils.AutoExposure, Tuple[float, float, int]]:
    lo_percentile, hi_percentile, update_every = request.param
    return (_utils.AutoExposure(lo_percentile, hi_percentile, update_every),
            (lo_percentile, hi_percentile, update_every))


@pytest.fixture
def normal_params() -> Tuple[float, float, int, int]:
    return (100, 10, 64, 512)


def outlierize_normal_data(mean: float, stddev: float, rows: int,
                           cols: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    normal_with_outliers = np.random.normal(mean, stddev, (rows, cols))
    low_outliers = np.full_like(normal_with_outliers, False, dtype=np.bool_)
    high_outliers = np.full_like(normal_with_outliers, False, dtype=np.bool_)
    outliers = set()

    for i in range(0, 100):
        randrow = random.randint(0, rows - 1)
        randcol = random.randint(0, cols - 1)
        if ((randrow, randcol)) in outliers:
            continue
        outliers.add((randrow, randcol))
        if random.random() < 0.5:
            normal_with_outliers[randrow, randcol] = mean - stddev * 10
            low_outliers[randrow, randcol] = True
        else:
            normal_with_outliers[randrow, randcol] = mean + stddev * 10
            high_outliers[randrow, randcol] = True
    return (normal_with_outliers, low_outliers, high_outliers)


@pytest.fixture
def normal_data_with_outliers() -> np.ndarray:
    return outlierize_normal_data(mean=25, stddev=2, rows=64, cols=1024)[0]


@pytest.fixture
def straight_line() -> np.ndarray:
    return np.arange(0.0, 500, 0.3).reshape(1, -1)


@pytest.fixture
def stream_digest() -> digest.StreamDigest:
    # load test scan and metadata
    digest_path = os.path.join(DATA_DIR, "OS-1-32-G_v2.1.1_1024x10_digest.json")

    with open(digest_path, 'r') as f:
        stream_digest = digest.StreamDigest.from_json(f.read())

    return stream_digest

@pytest.fixture()
def meta():
    meta_path = os.path.join(DATA_DIR, "OS-1-32-G_v2.1.1_1024x10.json")
    with open(meta_path, 'r') as f:
        return client.SensorInfo(f.read())

@pytest.fixture
def scan(stream_digest: digest.StreamDigest, meta: client.SensorInfo) -> client.LidarScan:
    pcap_path = os.path.join(DATA_DIR, "OS-1-32-G_v2.1.1_1024x10.pcap")

    with closing(pcap.Pcap(pcap_path, meta)) as source:
        scans = client.Scans(source)
        scan = next(iter(scans))

    return scan


def test_map_max_min(ae_with_params, normal_data_with_outliers, straight_line,
                     scan):
    """Test max and min values of AE

    All values should follow: 0 <= x <= 1"""
    auto_exposure, params = ae_with_params

    def test_max_min(key, ae):
        ae(key)
        assert (np.all(key >= 0.0))
        assert (np.all(key <= 1.0))

    normal_data = np.random.normal(100, 5, (128, 512))
    test_max_min(normal_data, auto_exposure)

    test_max_min(normal_data_with_outliers, auto_exposure)
    test_max_min(straight_line, auto_exposure)
    test_max_min(
        scan.field(client.ChanField.NEAR_IR).astype(float), auto_exposure)


def test_map_zero(ae_with_params, normal_data_with_outliers, straight_line,
                  scan):
    """Test AE behavior for 0 values

    Should map 0 values to 0 no matter what"""

    auto_exposure, params = ae_with_params

    def test_zero(key, ae):
        zeros = (key <= 0.0)
        if (np.all(zeros == False)):  # can't map when they're all 0
            return
        ae(key)
        assert (np.all(key[zeros] == 0.0))

    test_zero(normal_data_with_outliers, auto_exposure)
    test_zero(straight_line, auto_exposure)
    test_zero(scan.field(client.ChanField.RANGE).astype(float), auto_exposure)


def test_map_constant(ae_with_params):
    """Test AE behavior on constant values

    Need all results after AE application to be equivalent"""
    auto_exposure, params = ae_with_params

    def test_constant(key, ae):
        ae(key)
        assert (np.all(key == key[0, 0]))

    ones = np.ones((50, 100))
    test_constant(np.array(ones), auto_exposure)


def test_straight_line(ae_with_params, straight_line):
    """Test AE behavior on straight line

    Output should be a straight line fulfilling
        - only 1 minimum at 0
        - only 1 max between 1 - hi_percentile and 1
        
    note: due to implementation max is not guaranteed to be 1.0 
    """
    auto_exposure, params = ae_with_params
    lo_percentile, hi_percentile, update_every = params

    auto_exposure(straight_line)
    assert (np.count_nonzero(np.isclose(straight_line, 0.0) == 1))
    max_after_ae = np.max(straight_line)
    assert (np.count_nonzero(np.isclose(straight_line, max_after_ae) == 1))
    assert (max_after_ae >= 1 - hi_percentile)
    assert (max_after_ae <= 1.0)


def test_map_normal_distribution(ae_with_params, normal_params):
    """Test AE on normal distribution with high mean and low std dev

    Assert that ae returns:
        - fewer 0s than half of lo_percentile
        - fewer 1s than 2x hi_percentile

    Because we add 0.1 at the end of AE algo, we max out more than
    expected at the high end.
    """

    auto_exposure, params = ae_with_params
    lo_percentile, hi_percentile, update_every = params
    mean, stddev, rows, cols = normal_params

    # skip where percentiles are 0
    if (lo_percentile == 0 and hi_percentile == 0):
        return

    key = np.random.normal(mean, stddev, (rows, cols))
    auto_exposure(key)
    assert (np.count_nonzero(
        np.isclose(key, 0.0) <= lo_percentile * rows * cols * .5))
    assert (np.count_nonzero(np.isclose(key, 1.0)) <=
            2 * hi_percentile * rows * cols)


def test_map_normal_with_outliers(ae_with_params, normal_params):
    """Test AE's ability to clamp outliers high and low"""
    auto_exposure, params = ae_with_params
    lo_percentile, hi_percentile, update_every = params
    mean, stddev, rows, cols = normal_params

    # skip where percentiles are 0
    if (lo_percentile == 0 and hi_percentile == 0):
        return

    key, low_outliers, high_outliers = outlierize_normal_data(
        mean, stddev, rows, cols)
    auto_exposure(key)

    assert (np.all(np.isclose(key[low_outliers], 0.0)))
    assert (np.all(np.isclose(key[high_outliers], 1.0)))


def test_real_data_skewed(ae_with_params, normal_params):
    """Test AE ignoring 0s when calculating low and high percentiles"""

    auto_exposure, params = ae_with_params
    mean, stddev, rows, cols = normal_params

    normal_data = np.random.normal(mean, stddev, (rows, cols))
    normal_data_copy = np.array(normal_data)

    rows = normal_data.shape[0]
    cols = normal_data.shape[1]

    blanked_out = np.full_like(normal_data, False, dtype=np.bool_)
    for i in range(0, round(rows * cols * .05)):
        randrow = random.randint(0, rows - 1)
        randcol = random.randint(0, cols - 1)
        normal_data_copy[randrow, randcol] = 0.0
        blanked_out[randrow, randcol] = True

    auto_exposure(normal_data)
    auto_exposure(normal_data_copy)

    # use large atol b/c should be similar, not exactly alike
    assert (np.all(
        np.isclose(normal_data[np.invert(blanked_out)],
                   normal_data_copy[np.invert(blanked_out)],
                   atol=0.1)))


def test_multiple_updates(ae_with_params, normal_params):
    """Test AE's shift over multiple updates"""
    auto_exposure, params = ae_with_params
    lo_percentile, hi_percentile, update_every = params
    mean, stddev, rows, cols = normal_params

    auto_exposure_copy = _utils.AutoExposure(lo_percentile, hi_percentile,
                                             update_every)

    key0 = outlierize_normal_data(mean, stddev, rows, cols)[0]
    key0_copy = np.array(key0)

    auto_exposure(key0)
    auto_exposure_copy(key0_copy)

    # now we differentiate the copy
    key1 = outlierize_normal_data(2000, 150, rows, cols)[0]
    auto_exposure_copy(key1)

    key2 = outlierize_normal_data(mean, stddev, rows, cols)[0]
    key2_copy = np.array(key2)

    auto_exposure(key2)
    auto_exposure(key2_copy)

    assert (not np.allclose(key2, key2_copy))


if __name__ == '__main__':
    unittest.main()

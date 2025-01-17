"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

This module verifies that a sliced ScanSource matches the original ScanSource
but limits the interaction with the source to the scope.
"""

from typing import cast
import os
import pytest
from ouster.sdk import open_source
from ouster.sdk.client import ChanField, LidarScan
from ouster.sdk.client import (MultiScanSource, MultiReducedScanSource,
                               MultiClippedScanSource, MultiMaskedScanSource)
import numpy as np

from tests.conftest import OSFS_DATA_DIR
from tests.conftest import PCAPS_DATA_DIR

paths = [os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf"),
         os.path.join(PCAPS_DATA_DIR, 'OS-1-128_v2.3.0_1024x10_lb_n3.pcap')]

L = 3   # BOTH FILES USED ARE STRICTLY 3


@pytest.fixture(params=paths)
def scan_source_path(request):
    return request.param


def test_reduce_raises_exception_on_factors_mismatch_sensor_count(scan_source_path) -> None:
    normal_src = open_source(scan_source_path, sensor_idx=-1, index=False, cycle=False)
    normal_src = cast(MultiScanSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = MultiReducedScanSource(normal_src, [64] * (normal_src.sensors_count + 1))
    assert str(ex.value) == "beams should match the count of sensors"


@pytest.mark.parametrize("beams", [
    (12.5),
    (20 / 2),
    (30),
])
def test_reduce_raises_exception_on_invalid_factor_values(scan_source_path, beams) -> None:
    normal_src = open_source(scan_source_path, sensor_idx=-1, index=False, cycle=False)
    normal_src = cast(MultiScanSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = MultiReducedScanSource(normal_src, [beams])
    assert (str(ex.value) == f"beams {beams} must be divisor of "
            f"{normal_src.metadata[0].format.pixels_per_column}")


@pytest.mark.parametrize("beams", [
    (8),
    (16),
    (32),
])
def test_reduce(scan_source_path, beams) -> None:
    normal_src = open_source(scan_source_path, sensor_idx=-1, index=False, cycle=False)
    normal_src = cast(MultiScanSource, normal_src)
    reduced_src = MultiReducedScanSource(normal_src, [beams])
    assert reduced_src.metadata[0].format.pixels_per_column == beams

    normal_scans = [s[0] for s in normal_src if s]
    reduced_scans = [s[0] for s in reduced_src if s]

    for n, r in zip(normal_scans, reduced_scans):
        nt = cast(LidarScan, n)
        rt = cast(LidarScan, r)
        assert nt.w == rt.w and rt.h == beams


def test_reduce_raises_exception_on_invalid_range(scan_source_path) -> None:
    normal_src = open_source(scan_source_path, sensor_idx=-1, index=False, cycle=False)
    normal_src = cast(MultiScanSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = MultiClippedScanSource(normal_src, [ChanField.RANGE], 5000, 1000)
    assert str(ex.value) == "`upper` value can't be less than `lower`"


@pytest.mark.parametrize("fields, lower, upper", [
    # note: selected ranges are known to work on most for most of these files
    # however, a file that has no range value which exceeds 20m would fail,
    # similarly, a file with reflectivity that is always less than 200 also fail
    # this test.
    ([ChanField.RANGE], 10000, 20000),
    ([ChanField.REFLECTIVITY], 100, 200),
])
def test_clip(scan_source_path, fields, lower, upper) -> None:
    normal_src = open_source(scan_source_path, sensor_idx=-1, index=False, cycle=False)
    normal_src = cast(MultiScanSource, normal_src)
    clipped_src = MultiClippedScanSource(normal_src, fields, lower, upper)
    assert normal_src.metadata == clipped_src.metadata

    for s in clipped_src:
        for f in fields:
            s0 = cast(LidarScan, s[0])
            if s0.has_field(f):
                arr = s0.field(f)
                assert np.max(arr) == upper
                nonzero = arr[arr != 0]
                assert np.min(nonzero) == lower


def test_mask_raises_exception_on_factors_mismatch_sensor_count(scan_source_path) -> None:
    normal_src = open_source(scan_source_path, sensor_idx=-1, index=False, cycle=False)
    normal_src = cast(MultiScanSource, normal_src)
    H = normal_src.metadata[0].format.pixels_per_column
    W = normal_src.metadata[0].format.columns_per_frame

    mask = np.vstack(
        (
            np.ones((H // 2, W), np.uint8),
            np.zeros((H // 2, W), np.uint8)
        ))

    with pytest.raises(ValueError) as ex:
        _ = MultiMaskedScanSource(normal_src, [], [mask] * (normal_src.sensors_count + 1))
    assert str(ex.value) == "the number of masks should match the count of sensors"


def test_mask(scan_source_path) -> None:
    normal_src = open_source(scan_source_path, sensor_idx=-1, index=False, cycle=False)
    normal_src = cast(MultiScanSource, normal_src)
    H = normal_src.metadata[0].format.pixels_per_column
    W = normal_src.metadata[0].format.columns_per_frame

    mask = np.vstack(
        (
            np.ones((H // 2, W), np.uint8),
            np.zeros((H // 2, W), np.uint8)
        ))

    masked_src = MultiMaskedScanSource(normal_src, [], [mask] * (normal_src.sensors_count))

    normal_scans = [s[0] for s in normal_src if s]
    masked_scans = [s[0] for s in masked_src if s]

    for n, m in zip(normal_scans, masked_scans):
        nt = cast(LidarScan, n)
        mt = cast(LidarScan, m)
        for f in nt.fields:
            assert np.max(nt.field(f)[0:H // 2, :]) == np.max(mt.field(f)[0:H // 2, :])
            assert np.max(nt.field(f)[H // 2:, :]) != 0 and np.max(mt.field(f)[H // 2:, :]) == 0

"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

This module verifies that a sliced ScanSource matches the original ScanSource
but limits the interaction with the source to the scope.
"""

from typing import cast, Union
import os
import pytest
import sys
import tempfile
import copy
from ouster.sdk import open_source, osf, core
from ouster.sdk.core import ChanField, LidarScan
from ouster.sdk.core import (ScanSource, ReducedScanSource,
                             ClippedScanSource, MaskedScanSource)
import ouster.sdk.core.scan_ops as scan_ops
import numpy as np

from tests.conftest import OSFS_DATA_DIR
from tests.conftest import PCAPS_DATA_DIR

paths = [os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf"),
         os.path.join(PCAPS_DATA_DIR, 'OS-1-128_v2.3.0_1024x10_lb_n3.pcap')]

L = 3   # BOTH FILES USED ARE STRICTLY 3


@pytest.fixture
def input_osf_file(test_data_dir):
    return test_data_dir / "osfs" / "OS-1-128_v2.3.0_1024x10_lb_n3.osf"


@pytest.fixture(params=paths)
def scan_source_path(request):
    return request.param


def test_reduce_raises_exception_on_factors_mismatch_sensor_count(scan_source_path) -> None:
    normal_src = open_source(scan_source_path)
    normal_src = cast(ScanSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = ReducedScanSource(normal_src, [64] * (len(normal_src.sensor_info) + 1))
    assert str(ex.value) == "beams should match the count of sensors"


@pytest.mark.parametrize("beams", [
    (12.5),
    (20 / 2),
    (30),
])
def test_reduce_raises_exception_on_invalid_factor_values(scan_source_path, beams) -> None:
    normal_src = open_source(scan_source_path)
    normal_src = cast(ScanSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = ReducedScanSource(normal_src, [beams])
    assert (str(ex.value) == f"beams {beams} must be divisor of "
            f"{normal_src.sensor_info[0].format.pixels_per_column}")


@pytest.mark.parametrize("beams", [
    (1),
    (8),
    (16),
    (32),
])
def test_reduce(scan_source_path, beams) -> None:
    normal_src = open_source(scan_source_path)
    normal_src = cast(ScanSource, normal_src)
    reduced_src = ReducedScanSource(normal_src, [beams])
    assert reduced_src.sensor_info[0].format.pixels_per_column == beams

    normal_scans = [s[0] for s in normal_src if s]
    reduced_scans = [s[0] for s in reduced_src if s]

    for n, r in zip(normal_scans, reduced_scans):
        nt = cast(LidarScan, n)
        rt = cast(LidarScan, r)
        assert rt.sensor_info == reduced_src.sensor_info[0]
        assert nt.sensor_info != rt.sensor_info
        assert nt.w == rt.w and rt.h == beams


def test_clip_raises_exception_on_invalid_range(scan_source_path) -> None:
    normal_src = open_source(scan_source_path)
    normal_src = cast(ScanSource, normal_src)
    with pytest.raises(ValueError) as ex:
        _ = ClippedScanSource(normal_src, [ChanField.RANGE], 5000, 1000)
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
    normal_src = open_source(scan_source_path)
    normal_src = cast(ScanSource, normal_src)
    clipped_src = ClippedScanSource(normal_src, fields, lower, upper)
    assert normal_src.sensor_info == clipped_src.sensor_info

    for s in clipped_src:
        for f in fields:
            s0 = cast(LidarScan, s[0])
            if s0.has_field(f):
                arr = s0.field(f)
                assert np.max(arr) == upper
                nonzero = arr[arr != 0]
                assert np.min(nonzero) == lower


def test_mask_raises_exception_on_factors_mismatch_sensor_count(scan_source_path) -> None:
    normal_src = open_source(scan_source_path)
    normal_src = cast(ScanSource, normal_src)
    H = normal_src.sensor_info[0].format.pixels_per_column
    W = normal_src.sensor_info[0].format.columns_per_frame

    mask = np.vstack(
        (
            np.ones((H // 2, W), np.uint8),
            np.zeros((H // 2, W), np.uint8)
        ))

    with pytest.raises(ValueError) as ex:
        _ = MaskedScanSource(normal_src, [], [mask] * (len(normal_src.sensor_info) + 1))
    assert str(ex.value) == "the number of masks should match the count of sensors"


def test_mask(scan_source_path) -> None:
    normal_src = open_source(scan_source_path)
    normal_src = cast(ScanSource, normal_src)
    H = normal_src.sensor_info[0].format.pixels_per_column
    W = normal_src.sensor_info[0].format.columns_per_frame

    mask = np.vstack(
        (
            np.ones((H // 2, W), np.uint8),
            np.zeros((H // 2, W), np.uint8)
        ))

    masked_src = MaskedScanSource(normal_src, [], [mask] * len(normal_src.sensor_info))

    normal_scans = [s[0] for s in normal_src if s]
    masked_scans = [s[0] for s in masked_src if s]

    for n, m in zip(normal_scans, masked_scans):
        nt = cast(LidarScan, n)
        mt = cast(LidarScan, m)
        for f in nt.fields:
            assert np.max(nt.field(f)[0:H // 2, :]) == np.max(mt.field(f)[0:H // 2, :])
            assert np.max(nt.field(f)[H // 2:, :]) != 0 and np.max(mt.field(f)[H // 2:, :]) == 0


@pytest.mark.skipif(sys.platform.startswith("win"), reason="Broken on Windows")
def test_collate(input_osf_file):
    """Make sure indexing and length of collated source works correctly"""

    data = osf.OsfScanSource(str(input_osf_file))
    scan = next(iter(data))[0]
    data.close()
    result = None
    f = None
    try:
        # Create an adversarial file with no overlap between the two sensors
        with tempfile.NamedTemporaryFile(delete=False) as f:
            w = osf.Writer(f.name, [scan.sensor_info, scan.sensor_info])
            for idx in range(10):
                scan.frame_id = idx
                scan.packet_timestamp[:] = 100000001 * idx
                if idx >= 5:
                    w.save(1, scan)
                else:
                    w.save(0, scan)
            w.close()

        # with a dt of 200000000 we should get N-1 collated scans
        result = core.collate(osf.OsfScanSource(f.name), 200000000)

        # validate the lengths are as expected
        assert result.scans_num == [5, 5]
        assert len(result) == 9

        # validate that looping through matches indexing through
        looped = []
        for scans in result:
            looped.append(copy.copy(scans))
        indexed = []
        for i in range(len(result)):
            indexed.append(copy.copy(result[i]))
        assert indexed == looped

        # with a dt of 100000000 we should get N collated scans
        result = core.collate(osf.OsfScanSource(f.name), 100000000)

        # validate the lengths are as expected
        assert result.scans_num == [5, 5]
        assert len(result) == 10

        # validate that looping through matches indexing through
        looped = []
        for scans in result:
            looped.append(copy.copy(scans))
        indexed = []
        for i in range(len(result)):
            indexed.append(copy.copy(result[i]))
        assert indexed == looped
    finally:
        if result:
            result.close()
        os.unlink(f.name)


def test_chain(scan_source_path) -> None:
    """Make sure chaining works with scan source ops"""
    src = open_source(scan_source_path)
    src = src.reduce([1])
    src = src.clip(["RANGE"], 0, 100)
    src = src.mask(["RANGE"], [None])
    src = src.single(0)

    got_scan = False
    for scan in src:
        got_scan = True
        break
    assert got_scan


def test_lidarscan_field_types_writable(scan_source_path) -> None:
    """Create LidarScan, add all supported field types, verify they are writable."""
    src = open_source(scan_source_path)
    info = src.sensor_info[0]

    ls = LidarScan(info.format.columns_per_frame,
                   info.format.pixels_per_column,
                   info.format.udp_profile_lidar)

    # Attempt to add fields of all possible numpy scalar types
    dtypes = [
        np.uint8, np.uint16, np.uint32, np.uint64,
        np.int8, np.int16, np.int32, np.int64,
        np.float32, np.float64
    ]

    for dtype in dtypes:
        try:
            ls.add_field(f"test_field_{dtype.__name__}", dtype)
        except (TypeError, ValueError):
            # Not all types might be supported by the bindings
            pass

    ls.add_field("X",
                 np.array([b'0', b'1', b'2', b'3', b'4', b'5', b'6', b'7'],
                          dtype='|S8').view(np.recarray),
                core.FieldClass.SCAN_FIELD)

    for field_name in ls.fields:
        arr = ls.field(field_name)
        # Check that it is a numpy array (or convertible/workable) and writable
        assert isinstance(arr, np.ndarray)
        assert arr.flags.writeable, f"Field {field_name} (dtype: {arr.dtype}) is not writable"


def test_filter_xyz_dewarp_points_changes_spatial_mask(scan_source_path) -> None:
    """Dewarping should change the XYZ mask when non-identity per-column poses are used."""
    src = cast(ScanSource, open_source(scan_source_path))
    try:
        first = next(iter(src))[0]
        assert first is not None
        scan = copy.deepcopy(cast(LidarScan, first))
    finally:
        src.close()

    assert scan.has_field(ChanField.RANGE), "RANGE field is required for XYZ filtering test"

    scan.field(ChanField.RANGE)[:] = 1000
    scan.pose[:] = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], scan.w, axis=0)

    poses = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], scan.w, axis=0)
    poses[:scan.w // 2, 0, 3] = 2.0

    def zero_xyz(scan_or_range: Union[LidarScan, np.ndarray]) -> np.ndarray:
        range_image = (
            scan_or_range.field(ChanField.RANGE)
            if isinstance(scan_or_range, LidarScan)
            else scan_or_range
        )
        h, w = range_image.shape
        return np.zeros((h, w, 3), dtype=np.float64)

    scan_body = copy.deepcopy(scan)
    scan_world = copy.deepcopy(scan)
    scan_world.pose[:] = poses

    scan_ops.filter_xyz(
        scan_body,
        zero_xyz,
        axis_idx=0,
        lower=1.5,
        upper=2.5,
        invalid=0,
        filtered_fields=[ChanField.RANGE],
        dewarp_points=False,
    )
    scan_ops.filter_xyz(
        scan_world,
        zero_xyz,
        axis_idx=0,
        lower=1.5,
        upper=2.5,
        invalid=0,
        filtered_fields=[ChanField.RANGE],
        dewarp_points=True,
    )

    body_zeroed = np.count_nonzero(scan_body.field(ChanField.RANGE) == 0)
    world_zeroed = np.count_nonzero(scan_world.field(ChanField.RANGE) == 0)

    assert body_zeroed == 0
    assert world_zeroed == scan.h * (scan.w // 2)

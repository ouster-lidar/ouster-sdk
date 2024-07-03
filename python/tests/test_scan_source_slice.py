"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

This module verifies that a sliced ScanSource matches the original ScanSource
but limits the interaction with the source to the scope.
"""

import os
import pytest
from ouster.sdk import open_source

from tests.conftest import OSFS_DATA_DIR
from tests.conftest import PCAPS_DATA_DIR

paths = [os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf"),
         os.path.join(PCAPS_DATA_DIR, 'OS-1-128_v2.3.0_1024x10_lb_n3.pcap')]

L = 3   # BOTH FILES USED ARE STRICTLY 3


@pytest.fixture(params=paths)
def scan_source_path(request):
    return request.param


@pytest.fixture
def sliceable_fixture(scan_source_path):
    scan_source = open_source(scan_source_path, index=True, cycle=False)
    ref_ids = [s.frame_id for s in scan_source]  # register ref ids
    return scan_source, ref_ids


@pytest.mark.parametrize("start, stop, step", [
    (0, L, None),
    (0, L - 1, None),
    (0, L - 2, None),
    (0, L - 3, None),
    (1, L, None),
    (2, L, None),
    (3, L, None),
    (0, L + 1, None),
    (0, L + L, None),
    (0, -1, None),
    (0, -2, None),
    (0, -L, None),
    (-1, L, None),
    (-2, L, None),
    (-L, L, None),
    # step=1 is covered by None and ForwardSlicer tests
    (0, L, 2),
    (0, L, 3),
    (0, L - 1, 2),
    (0, L - 1, 3),
    (1, L, 2),
    (1, L, 3),
    (1, L - 1, 2),
    (1, L - 1, 3),
])
def test_slicing_level_1(sliceable_fixture, start, stop, step) -> None:
    scan_source, ref_ids = sliceable_fixture
    s = slice(start, stop, step)
    ss_slice = scan_source[s]
    assert len(ss_slice) == len(ref_ids[s])
    sliced_ids = [s.frame_id for s in ss_slice]
    assert sliced_ids == ref_ids[s]


@pytest.mark.parametrize("start, stop, step", [
    (0, L - 1, None),
    (0, L - 2, None),
    (0, L - 3, None),
    (1, L, None),
    (2, L, None),
    (3, L, None),
])
def test_slicing_level_2_from_full(sliceable_fixture, start, stop, step) -> None:
    scan_source, ref_ids = sliceable_fixture
    s1 = slice(None)
    ss_l1 = scan_source[s1]
    assert len(ss_l1) == L
    s2 = slice(start, stop, step)
    ss_l2 = ss_l1[s2]
    assert len(ss_l2) == len(ref_ids[s1][s2])
    sliced_ids = [s.frame_id for s in ss_l2]
    assert sliced_ids == ref_ids[s1][s2]


@pytest.mark.parametrize("start, stop, step", [
    (0, L - 1, None),
    (0, L - 2, None),
    (0, L - 3, None),
    (1, L, None),
    (2, L, None),
    (3, L, None),
])
def test_slicing_level_2_from_subset(sliceable_fixture, start, stop, step) -> None:
    scan_source, ref_ids = sliceable_fixture
    s1 = slice(1, L)
    ss_l1 = scan_source[s1]
    assert len(ss_l1) == L - 1
    s2 = slice(start, stop, step)
    ss_l2 = ss_l1[s2]
    assert len(ss_l2) == len(ref_ids[s1][s2])
    sliced_ids = [s.frame_id for s in ss_l2]
    assert sliced_ids == ref_ids[s1][s2]


@pytest.mark.parametrize("start, stop, step", [
    (0, L, -1),
    (L, 0, -1),
    (L, 0, -2),
])
def test_slicing_negative_step_throws(sliceable_fixture, start, stop, step) -> None:
    scan_source, _ = sliceable_fixture
    s = slice(start, stop, step)
    with pytest.raises(TypeError) as ex:
        _ = scan_source.slice(s)
    assert str(ex.value) == "slice() can't work with negative step"


@pytest.mark.parametrize("start, stop, step", [
    (None, None, 0),
    (None, 8, 0),
    (3, None, 0),
    (3, 8, 0),
    (-3, -8, 0),
])
def test_slicing_zero_step_throws(sliceable_fixture, start, stop, step):
    scan_source, _ = sliceable_fixture
    s = slice(start, stop, step)
    with pytest.raises(ValueError) as ex:
        _ = scan_source.slice(s)
    assert str(ex.value) == "slice step cannot be zero"


def test_slicing_dual_slice(sliceable_fixture) -> None:
    scan_source, ref_ids = sliceable_fixture
    s1 = slice(0, 2)
    ss_slice1 = scan_source.slice(s1)
    assert len(ss_slice1) == len(ref_ids[s1])
    sliced_ids = [s.frame_id for s in ss_slice1]
    assert sliced_ids == ref_ids[s1]
    s2 = slice(1, 3)
    ss_slice2 = scan_source.slice(s2)
    assert len(ss_slice2) == len(ref_ids[s2])
    sliced_ids = [s.frame_id for s in ss_slice2]
    assert sliced_ids == ref_ids[s2]
    # revisit the first slice
    sliced_ids = [s.frame_id for s in ss_slice1]
    assert sliced_ids == ref_ids[s1]


@pytest.mark.parametrize("start, stop, step, idx", [
    (0, L, None, 0),
    (0, L, None, 1),
    (0, L, None, 2),
    (0, L, None, -1),
    (0, L, None, -2),
    (1, L, None, 0),
    (1, L, None, 1),
    (1, L, None, -1),
    (2, L, None, 0),
])
def test_index_from_level_1_slice(sliceable_fixture, start, stop, step, idx) -> None:
    scan_source, ref_ids = sliceable_fixture
    s = slice(start, stop, step)
    assert scan_source[s][idx].frame_id == ref_ids[s][idx]


@pytest.mark.parametrize("start, stop, step, idx", [
    (0, L, None, 0),
    (0, L, None, 1),
    (0, L, None, 2),
    (0, L, None, -1),
    (0, L, None, -2),
    (1, L, None, 0),
    (1, L, None, 1),
    (1, L, None, -1),
    (2, L, None, 0),
])
def test_index_from_level_2_slice(sliceable_fixture, start, stop, step, idx) -> None:
    scan_source, ref_ids = sliceable_fixture
    s1 = slice(None)
    ss_l1 = scan_source[s1]
    assert len(ss_l1) == L
    s2 = slice(start, stop, step)
    ss_l2 = ss_l1[s2]
    assert ss_l2[idx].frame_id == ref_ids[s1][s2][idx]


@pytest.mark.parametrize("start, stop, step, idx", [
    (0, L, None, L),
    (0, L, None, L + 1),
    (1, L, None, L - 1),
    (1, L, None, L),
    (2, L, None, L - 2),
    (L, L, None, 0),
])
def test_index_slicing_out_of_range_throws(sliceable_fixture, start, stop, step, idx):
    scan_source, _ = sliceable_fixture
    s = slice(start, stop, step)
    with pytest.raises(IndexError) as ex:
        _ = scan_source[s][idx]
    assert str(ex.value) == "index is out of range"

"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.
"""

import numpy as np

from ouster.sdk.core import LidarScanSet, LidarScan


def test_lidar_scan_set_bindings() -> None:
    scans_in = [LidarScan(10, 10), LidarScan(10, 10), None, LidarScan(10, 10)]
    collation = LidarScanSet(scans_in)
    scans_out = [x for x in collation]
    assert scans_in == scans_out
    assert (collation[0].w, collation[0].h) == (10, 10)  # type: ignore
    assert collation[2] is None
    assert len(collation) == 4

    collation.add_field("zz", np.uint32, (10, 10, 10))
    assert collation.has_field("zz")
    collation.del_field("zz")
    assert not collation.has_field("zz")

    collation.add_field("aa", np.uint32, (10, 10, 10))
    collation.add_field("cc", np.uint32, (10, 10, 10))
    collation.add_field("bb", np.uint32, (10, 10, 10))

    assert collation.fields == ["aa", "bb", "cc"]


def test_lidar_scan_set_with_missing_scans() -> None:
    scans_in = [LidarScan(10, 10), None, LidarScan(10, 10)]
    collation = LidarScanSet(scans_in)
    scans_out = [x for x in collation]
    assert scans_in == scans_out
    assert (collation[0].w, collation[0].h) == (10, 10)  # type: ignore
    assert collation[1] is None
    assert len(collation) == 3

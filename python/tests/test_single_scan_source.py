"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.

This module tests the compatibility between the ScanSource interface obtained
from a MultiScanSource reduced using single_source() and the original/legacy
ScanSource interfaces based of the concrete implementations Scans/osf.Scans
"""

import os

from tests.conftest import PCAPS_DATA_DIR, OSFS_DATA_DIR


def test_single_scan_source_pcap() -> None:
    file_path = os.path.join(PCAPS_DATA_DIR, 'OS-0-128-U1_v2.3.0_1024x10.pcap')

    # old interface
    from ouster.sdk import client, pcap
    from ouster.sdk.util import resolve_metadata

    meta_path = resolve_metadata(file_path)
    assert meta_path
    meta = client.SensorInfo(open(meta_path).read())
    pcap_source = pcap.Pcap(file_path, meta)
    scans = client.Scans(pcap_source)

    from ouster.sdk.client import MultiScanSource
    from ouster.sdk.pcap import PcapScanSource  # type: ignore

    # new interface
    scan_source: MultiScanSource
    scan_source = PcapScanSource(file_path, cycle=False)
    ss = scan_source.single_source(0)

    ref_ids = [s.frame_id for s in scans]
    upd_ids = [s.frame_id for s in ss]

    assert len(ref_ids) == len(upd_ids)
    assert ref_ids == upd_ids


def test_single_scan_source_osf() -> None:
    file_path = os.path.join(OSFS_DATA_DIR, "OS-1-128_v2.3.0_1024x10_lb_n3.osf")

    # old interface
    import ouster.sdk.osf as osf

    scans: osf.Scans
    scans = osf.Scans(file_path, cycle=False, sensor_id=0)

    # new interface
    from ouster.sdk.client import MultiScanSource
    from ouster.sdk.osf import OsfScanSource

    scan_source: MultiScanSource
    scan_source = OsfScanSource(file_path, cycle=False)
    ss = scan_source.single_source(0)

    ref_ids = [s.frame_id for s in scans]
    upd_ids = [s.frame_id for s in ss]  # type: ignore

    assert len(ref_ids) == len(upd_ids)
    assert ref_ids == upd_ids

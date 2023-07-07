# type: ignore
from tests.conftest import PCAPS_DATA_DIR
import pytest
from pathlib import Path
from ouster.sdkx.mapping.util import Source
from ouster.sdkx.mapping.slam import KissBackend
import numpy as np


has_kiss = False
try:
    import kiss_icp  # noqa
    has_kiss = True
except ImportError:
    print("kiss_icp is not installed. Please run pip install kiss-icp")


def test_source_with_string(test_data_dir):
    """Test we can load data with str. Not specifying metadata"""
    data = Path(PCAPS_DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap'
    source = Source(str(data))

    lidar_scan = next(iter(source))
    assert lidar_scan.complete()


def test_source_with_path(test_data_dir):
    """Test we can load data with Path. Not specifying metadata"""
    data = Path(PCAPS_DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap'
    source = Source(data)

    lidar_scan = next(iter(source))
    assert lidar_scan.complete()


@pytest.mark.skipif(not has_kiss, reason="kiss_icp not installed")
def test_kiss_slam_runs(test_data_dir):
    """We only work with kiss_slam as of now"""
    data = Path(PCAPS_DATA_DIR) / 'OS-1-128_v2.3.0_1024x10_lb_n3.pcap'
    scans = Source(data)
    slam = KissBackend(scans.metadata)

    # Kiss ICP intial first return scan all poses are eye(4)
    scan1 = next(iter(scans))
    result1 = slam.update(scan1)
    scan2 = next(iter(scans))
    # Kiss ICP second return scan poses are not eye(4)
    result2 = slam.update(scan2)
    assert (np.array_equal(result1.pose[0, :, :], np.eye(4)) and
            not np.array_equal(result2.pose[0, :, :], np.eye(4)))

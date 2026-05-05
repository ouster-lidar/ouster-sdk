# Please note that the tests in `tests/integration` folder cannot be run on your system
# as they use external data (not included in the repository).

# type: ignore
import logging
import os
import numpy as np

from ouster.sdk.open_source import open_source

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Unit_Tests')
data_path = os.getenv('TEST_DATA_DIR', None)
data_path = os.path.join(data_path, "mapping")


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Unit_Tests')


def test_kiss_slam_runs():
    from ouster.sdk.mapping import SlamConfig, SlamEngine
    data = os.path.join(data_path, "short.pcap")

    config = SlamConfig()
    config.min_range = 1
    config.max_range = 50
    config.voxel_size = 0.5
    config.backend = "kiss"

    scans = open_source(data)
    slam = SlamEngine(scans.sensor_info, config)
    scans_iter = iter(scans)

    # Kiss ICP intial first return scan all poses are eye(4)
    scan1 = next(scans_iter)
    result1 = slam.update(scan1)
    scan2 = next(scans_iter)
    # Kiss ICP second return scan poses are not eye(4)
    result2 = slam.update(scan2)
    assert (np.array_equal(result1[0].pose[0, :, :], np.eye(4)) and
            not np.array_equal(result2[0].pose[0, :, :], np.eye(4)))

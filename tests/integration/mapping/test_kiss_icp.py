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

    config = SlamConfig.create('lio')
    config.min_range = 1
    config.max_range = 50
    config.voxel_size = 0.5

    source = open_source(data)
    slam = SlamEngine.create(source.sensor_info, config)
    source_iter = iter(source)

    # Kiss ICP initial first return frame all poses are eye(4)
    frame1 = next(source_iter)
    result1 = slam.update(frame1)
    frame2 = next(source_iter)
    # Kiss ICP second return frame poses are not eye(4)
    result2 = slam.update(frame2)
    assert (np.array_equal(result1[0].body_to_world[0, :, :], np.eye(4)) and
            not np.array_equal(result2[0].body_to_world[0, :, :], np.eye(4)))

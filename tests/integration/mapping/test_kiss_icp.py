# Please note that the tests in `tests/integration` folder cannot be run on your system
# as they use external data (not included in the repository).

# type: ignore
import logging
import os
import numpy as np
import pytest
import platform
from unittest.mock import MagicMock
import ouster.sdk.util.pose_util as pu
import ouster.sdk.mapping.util as util

from ouster.sdk.open_source import open_source
import ouster.sdk.mapping

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Unit_Tests')


# Check if the platform is macOS and if it's running on ARM architecture
is_mac_arm = platform.system() == 'Darwin' and platform.processor() == 'arm'
try:
    from kiss_icp.kiss_icp import KissICP
    from ouster.sdk.mapping.slam import KissBackend
    kiss_icp_available = True
except ImportError:
    kiss_icp_available = False

def create_mock_ls(ts_off, pkt_ts_off):
    mock_ls = MagicMock()

    tss = np.array([1638345600, 1638352800, 1638360000, 1638367200, 1638374400])
    pkt_tss = np.array([1638365600, 1638372800, 1638380000, 1638387200, 1638394400])

    def mock_timestamp(ts_off):
        return tss + ts_off

    def mock_pkt_timestamp(pkt_ts_off):
        return pkt_tss + pkt_ts_off

    def mock_status():
        return [1, 1, 1, 1, 1]

    def mock_field(input_field):
        return np.array([0, 1, 2, 0, 3])

    mock_ls.timestamp = mock_timestamp(ts_off)
    mock_ls.packet_timestamp = mock_pkt_timestamp(pkt_ts_off)
    mock_ls.status = mock_status()
    mock_ls.w = 5
    mock_ls.h = 5
    mock_ls.packet_timestamp.shape = pkt_tss.shape
    mock_ls.field = mock_field
    mock_ls.pose = np.zeros((mock_ls.w, 4, 4))

    return mock_ls


class MockXYZLut:
    def __call__(self, input_array):
        # Create a list of zero points, one for each element in the input array
        zero_pts = np.zeros((len(input_array), 3))
        return zero_pts


@pytest.mark.skipif(not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm")
def test_getKissICPInputData_normalizedTS():

    dummy_ls_A = create_mock_ls(0, 0)
    dummy_ls_B = create_mock_ls(200000000, 150)
    ls_list = [dummy_ls_A, dummy_ls_B]

    # get the ts offset which remove the lidar timestamp offset and use pkt time for real ts diff
    tss_off = KissBackend._get_scans_ts_offset(ls_list)

    assert tss_off == [-1638345600, -1838345450], f"Expected [-1638345600, -1838345450], but got {tss_off}"

    dummy_xyzlut_list = [MockXYZLut(), MockXYZLut()]
    results = util.getKissICPInputData(ls_list, dummy_xyzlut_list ,tss_off)

    # expected normalized timestamp in each scan.
    expected_results = [np.array([0.0, 0.24870466, 0.49740933, 0.74611399, 0.99481865]),
                        np.array([0.00518135, 0.25388601, 0.50259067, 0.75129534, 1.0])]
    assert len(results[2]) == len(expected_results), \
            f"Expected {len(expected_results)} arrays, but got {len(results[2])}"

    # Assert each array individually
    for i, (result_array, expected_array) in enumerate(zip(results[2], expected_results)):
        assert np.allclose(result_array, expected_array), \
                f"Array {i} does not match. Expected {expected_array}, but got {result_array}"


@pytest.mark.skipif(not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm")
def test_writeScanColPose():
    start_ts_pose_v = np.array([0, 1, 0, 0, 1, 0])
    start_ts_pose_h = pu.exp_pose6(start_ts_pose_v)
    end_ts_pose_v = np.array([2, 3, 4, 1, 2, 3])
    end_ts_pose_h = pu.exp_pose6(end_ts_pose_v)

    dummy_ls_A = create_mock_ls(0, 0)
    dummy_ls_B = create_mock_ls(200000000, 150)
    ls_list = [dummy_ls_A, dummy_ls_B]

    # get the ts offset which remove the lidar timestamp offset and use pkt time for real ts diff
    tss_off = KissBackend._get_scans_ts_offset(ls_list)
    dummy_xyzlut_list = [MockXYZLut(), MockXYZLut()]
    results = util.getKissICPInputData(ls_list, dummy_xyzlut_list ,tss_off)

    util.writeScanColPose(start_ts_pose_h, end_ts_pose_h, ls_list, results[2])

    # Pre-Calculated expected poses for the first LidarScan (dummy_ls_A)
    expected_poses_A = [
        np.array([[ 0.90421104,  0.31953309,  0.28337432,  1.1853133 ],
                  [-0.36527989,  0.92240516,  0.12545645,  1.37348252],
                  [-0.22129845, -0.21695004,  0.95076794,  1.18300368],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        np.array([[ 0.86424722,  0.50122996, -0.04295667,  1.43850647],
                  [-0.47261132,  0.83821929,  0.27207896,  1.65357602],
                  [ 0.17238124, -0.21484168,  0.96131564,  1.96027934],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        np.array([[ 0.67775983,  0.6574975 , -0.32914836,  1.4484745 ],
                  [-0.50307586,  0.74113201,  0.44456499,  2.04820307],
                  [ 0.53624276, -0.13572169,  0.83308062,  2.72897091],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        np.array([[ 0.37610744,  0.76205874, -0.52707653,  1.25148159],
                  [-0.45155079,  0.64746891,  0.61391034,  2.57355171],
                  [ 0.8091014 ,  0.00710558,  0.5876261 ,  3.3861765 ],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        np.array([[ 0.01001405,  0.7973313 , -0.60345879,  0.91859322],
                  [-0.32670024,  0.5729798 ,  0.75163895,  3.22382867],
                  [ 0.94507496,  0.18962318,  0.26622617,  3.84774103],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]])
    ]

    # Pre-Calculated expected poses for the second LidarScan (dummy_ls_B)
    expected_poses_B = [
        np.array([[ 0.90496315,  0.32337439,  0.27653335,  1.19310965],
                  [-0.3681983 ,  0.92088451,  0.12806927,  1.37836019],
                  [-0.21324096, -0.21771708,  0.95243245,  1.19856636],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        np.array([[ 0.86178771,  0.50484689, -0.04951326,  1.44113739],
                  [-0.47407138,  0.83627345,  0.27550507,  1.66053684],
                  [ 0.1804945 , -0.21395407,  0.96002364,  1.97675347],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        np.array([[ 0.67250228,  0.66028186, -0.33431805,  1.44628801],
                  [-0.50283207,  0.73908817,  0.44822827,  2.05779625],
                  [ 0.54304751, -0.13332869,  0.82904937,  2.7441354 ],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        np.array([[ 0.36893592,  0.76354232, -0.52999001,  1.24563579],
                  [-0.44964414,  0.64567076,  0.6171948 ,  2.58588385],
                  [ 0.8134534 ,  0.01060157,  0.58153346,  3.39803048],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        np.array([[ 0.00213448,  0.79726464, -0.60362616,  0.91086152],
                  [-0.32345134,  0.57172971,  0.75399229,  3.23854577],
                  [ 0.9462424 ,  0.19363431,  0.25909665,  3.85484029],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]])
    ]

    for i, expected_pose in enumerate(expected_poses_A):
        actual_pose = dummy_ls_A.pose[i]
        assert np.allclose(actual_pose, expected_pose, rtol=1e-5),\
                f"Pose {i} does not match for dummy_ls_A"

    for i, expected_pose in enumerate(expected_poses_B):
        actual_pose = dummy_ls_B.pose[i]
        assert np.allclose(actual_pose, expected_pose, rtol=1e-5),\
                f"Pose {i} does not match for dummy_ls_B"

data_path = os.getenv('TEST_DATA_DIR', None)

data_path = os.path.join(data_path, "mapping")

@pytest.mark.skipif(not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm")
def test_kiss_slam_runs():
    from ouster.sdk.mapping import KissBackend
    data = os.path.join(data_path, "short.pcap")

    scans = open_source(data, sensor_idx=-1)
    slam = KissBackend(scans.metadata)
    scans_iter = iter(scans)

    # Kiss ICP intial first return scan all poses are eye(4)
    scan1 = next(scans_iter)
    result1 = slam.update(scan1)
    scan2 = next(scans_iter)
    # Kiss ICP second return scan poses are not eye(4)
    result2 = slam.update(scan2)
    assert (np.array_equal(result1[0].pose[0, :, :], np.eye(4)) and
            not np.array_equal(result2[0].pose[0, :, :], np.eye(4)))

import numpy as np
from unittest.mock import MagicMock
import ouster.sdkx.mapping.util as util
import ouster.sdk.pose_util as pu


def create_mock_ls():
    mock_ls = MagicMock()

    def mock_timestamp():
        return [6, 8, 10, 12, 14]

    def mock_status():
        return [1, 1, 1, 1, 1]

    mock_ls.timestamp = mock_timestamp()
    mock_ls.status = mock_status()
    mock_ls.w = 5
    mock_ls.h = 3

    return mock_ls


def test_getScanColPose():
    start_ts_pose_v = np.array([0, 1, 0, 0, 1, 0])
    start_ts_pose_h = pu.exp_pose6(start_ts_pose_v)
    end_ts_pose_v = np.array([2, 3, 4, 1, 2, 3])
    end_ts_pose_h = pu.exp_pose6(end_ts_pose_v)

    # test start ts and end ts diff is 1 s
    start_ts_pose = (0, start_ts_pose_h)
    end_ts_pose = (10, end_ts_pose_h)

    # got from ouster-trans trajectory

    expected = np.array([
        [[0.90677223, 0.39372832, 0.15080493, 1.31700025],
        [-0.41678434, 0.89109184, 0.17957215, 1.47422945],
        [-0.0636784, -0.22568418, 0.97211713, 1.48970589],
        [0.0, 0.0, 0.0, 1.0]],

        [[0.83708692, 0.53650174, -0.10701109, 1.45986223],
        [-0.48555864, 0.81872839, 0.3064582, 1.72474702],
        [0.25202837, -0.20457199, 0.94584988, 2.12305037],
        [0.0, 0.0, 0.0, 1.0]],

        [[0.67513737, 0.65889254, -0.33173809, 1.44739261],
        [-0.50295847, 0.74010989, 0.44639682, 2.05299246],
        [0.53965017, -0.13452869, 0.83107143, 2.73655935],
        [0.0, 0.0, 0.0, 1.0]],

        [[0.43862323, 0.74752449, -0.4988154, 1.30078437],
        [-0.4670822, 0.66382864, 0.58409396, 2.4662355],
        [0.76775249, -0.02320938, 0.64032604, 3.27695739],
        [0.0, 0.0, 0.0, 1.0]],

        [[0.1533934, 0.7927109, -0.58998296, 1.05589062],
        [-0.38185077, 0.5982215, 0.70450055, 2.96245636],
        [0.91140576, 0.11721971, 0.3944605, 3.69895944],
        [0.0, 0.0, 0.0, 1.0]]])

    # dummy ls ts len is 5. [6,8,10,12,14]. In the function getScanColPose
    dummy_ls = create_mock_ls()
    per_col_poses = util.getScanColPose(start_ts_pose, end_ts_pose, dummy_ls)

    assert np.allclose(per_col_poses, expected, rtol=1e-4)

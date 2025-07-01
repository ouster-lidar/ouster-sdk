# Please note that the tests in `tests/integration` folder cannot be run on your system
# as they use external data (not included in the repository).

# type: ignore
import logging
import os
import numpy as np
import pytest
import platform
from unittest.mock import MagicMock, PropertyMock, patch
import ouster.sdk.util.pose_util as pu
import ouster.sdk.mapping.util as util
import ouster.sdk.core as core

from ouster.sdk.open_source import open_source
import ouster.sdk.mapping

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Unit_Tests')
data_path = os.getenv('TEST_DATA_DIR', None)
data_path = os.path.join(data_path, "mapping")

# Check if the platform is macOS and if it's running on ARM architecture
is_mac_arm = platform.system() == 'Darwin' and platform.processor() == 'arm'
try:
    from ouster.sdk.mapping.kiss_slam import KissSlam
    kiss_icp_available = True
except ImportError:
    kiss_icp_available = False

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('Unit_Tests')


def create_mock_sensor_info():
    """Creates a mock SensorInfo object."""
    mock_info = MagicMock(spec=core.SensorInfo)
    mock_info.format.fps = 10
    return mock_info


def create_mock_lidarscan(w=5, h=5, frame_id=1):
    """Creates a mock LidarScan object."""
    mock_ls = MagicMock(spec=core.LidarScan)
    mock_ls.w = w
    mock_ls.h = h
    mock_ls.frame_id = frame_id
    # IMPORTANT: The .pose attribute must be a real array that can be written to
    mock_ls.pose = np.zeros((w, 4, 4), dtype=float)

    # This mock for the .field() method returns a 2D array (H x W)
    mock_ls.field.side_effect = lambda field: np.tile(
        np.array([0, 1, 2, 0, 3]), (h, 1)
    ) if field == core.ChanField.RANGE else np.zeros((h, w))

    mock_ls.get_first_valid_column.return_value = 0
    mock_ls.get_last_valid_column.return_value = w - 1
    mock_ls.get_first_valid_column_timestamp = MagicMock()
    mock_ls.get_last_valid_column_timestamp = MagicMock()
    mock_ls.get_first_valid_packet_timestamp = MagicMock()

    timestamp_data = np.zeros(w, dtype=np.int64)
    type(mock_ls).timestamp = PropertyMock(return_value=timestamp_data)

    return mock_ls


class MockXYZLut:
    """
    Correctly mocks the XYZLut behavior.
    It takes an (H, W) range array and returns an (H, W, 3) point cloud,
    without doing any filtering itself.
    """
    def __call__(self, range_array):
        h, w = range_array.shape
        # Return an unfiltered point cloud with the correct 3D shape
        return np.zeros((h, w, 3))


@pytest.mark.skipif(
    not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm"
)
def test_getKissICPInputData_normalizedTS_updated():
    """
    Tests the updated logic for calculating timestamp offsets and normalizing timestamps
    for KISS-ICP input.
    """
    mock_info_A = create_mock_sensor_info()
    mock_info_B = create_mock_sensor_info()
    slam_config = MagicMock()

    mock_packet_format = MagicMock()
    mock_packet_format.return_value.max_frame_id = 65535

    mock_xyz_lut = MagicMock(return_value=MockXYZLut())

    with patch.multiple(
        'ouster.sdk.mapping.kiss_slam.core',
        XYZLut=mock_xyz_lut,
        PacketFormat=mock_packet_format
    ):
        kiss_slam = KissSlam([mock_info_A, mock_info_B], slam_config)
        kiss_slam._xyz_lut = [MockXYZLut(), MockXYZLut()]

    scan_A = create_mock_lidarscan()
    scan_B = create_mock_lidarscan()

    scan_A.timestamp[:] = np.array([1000, 1010, 1020, 1030, 1040], dtype=np.int64)
    scan_A.get_first_valid_column_timestamp.return_value = np.int64(1000)
    scan_A.get_last_valid_column_timestamp.return_value = np.int64(1040)
    scan_A.get_first_valid_packet_timestamp.return_value = np.int64(1050)

    scan_B.timestamp[:] = np.array([1200, 1210, 1220, 1230, 1240], dtype=np.int64)
    scan_B.get_first_valid_column_timestamp.return_value = np.int64(1200)
    scan_B.get_last_valid_column_timestamp.return_value = np.int64(1240)
    scan_B.get_first_valid_packet_timestamp.return_value = np.int64(1250)

    ls_list = [scan_A, scan_B]

    ts_offsets = kiss_slam._calculate_fallback_ts_offset(ls_list)
    expected_offsets = [np.int64(-1000), np.int64(-1000)]
    assert ts_offsets == expected_offsets

    _, _, normalized_ts = util.get_total_frame_pts_and_ts(
        ls_list, kiss_slam._xyz_lut, ts_offsets
    )
    expected_results = [
        np.array([0.0, 0.04166667, 0.08333333, 0.125, 0.16666667]),
        np.array([0.83333333, 0.875, 0.91666667, 0.95833333, 1.0])
    ]

    assert len(normalized_ts) == len(expected_results)
    for i, (result_array, expected_array) in enumerate(zip(normalized_ts, expected_results)):
        assert np.allclose(result_array, expected_array, atol=1e-5)


@pytest.mark.skipif(
    not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm"
)
def test_check_monotonic_increase_ts():
    """
    Tests the _check_monotonic_increase_ts function with various scenarios.
    """
    mock_info = create_mock_sensor_info()
    slam_config = MagicMock()

    mock_packet_format = MagicMock()
    mock_packet_format.return_value.max_frame_id = 65535

    mock_xyz_lut = MagicMock(return_value=MockXYZLut())

    with patch.multiple(
        'ouster.sdk.mapping.kiss_slam.core',
        XYZLut=mock_xyz_lut,
        PacketFormat=mock_packet_format
    ):
        kiss_slam = KissSlam([mock_info], slam_config)

    scan1 = create_mock_lidarscan()
    scan1.timestamp[:] = np.array([100, 200, 300, 400, 500])
    kiss_slam._last_frame_ts_range = [(-1, 90)]
    kiss_slam._frame_valid_id_range = [(0, 4)]
    assert kiss_slam._check_monotonic_increase_ts([scan1]) == [True]

    scan2 = create_mock_lidarscan()
    scan2.timestamp[:] = np.array([100, 200, 150, 400, 500])
    assert kiss_slam._check_monotonic_increase_ts([scan2]) == [False]

    scan3 = create_mock_lidarscan()
    scan3.timestamp[:] = np.array([80, 200, 300, 400, 500])
    kiss_slam._last_frame_ts_range = [(-1, 90)]
    assert kiss_slam._check_monotonic_increase_ts([scan3]) == [False]

    scan4 = create_mock_lidarscan()
    scan4.timestamp[:] = np.array([100, 0, 300, 0, 500])
    kiss_slam._last_frame_ts_range = [(-1, 90)]
    assert kiss_slam._check_monotonic_increase_ts([scan4]) == [True]

    assert kiss_slam._check_monotonic_increase_ts([None]) == [True]

    scans_mixed = [scan1, scan2, scan3, scan4, None]
    with patch.multiple(
        'ouster.sdk.mapping.kiss_slam.core',
        XYZLut=mock_xyz_lut,
        PacketFormat=mock_packet_format
    ):
        kiss_slam_multi = KissSlam([mock_info] * len(scans_mixed), slam_config)

    kiss_slam_multi._last_frame_ts_range = [(-1, 90)] * len(scans_mixed)
    kiss_slam_multi._frame_valid_id_range = [(0, 4)] * len(scans_mixed)
    expected_results = [True, False, False, True, True]
    assert kiss_slam_multi._check_monotonic_increase_ts(scans_mixed) == expected_results


@pytest.mark.skipif(
    not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm"
)
def test_get_total_frame_pts_and_ts_with_zero_offset():
    """
    Tests the get_total_frame_pts_and_ts function with zero offsets.
    This replaces the logic from the old test_writeScanColPose.
    """
    # 1Set up mock scans with different timestamp
    # This replaces the old `create_mock_ls` calls
    scan_A = create_mock_lidarscan()
    scan_B = create_mock_lidarscan()

    # Configure timestamps to test normalization across two different scans
    scan_A.timestamp[:] = np.array([1000, 1010, 1020, 1030, 1040], dtype=np.int64)
    scan_A.get_first_valid_column_timestamp.return_value = np.int64(1000)
    scan_A.get_last_valid_column_timestamp.return_value = np.int64(1040)

    scan_B.timestamp[:] = np.array([1200, 1210, 1220, 1230, 1240], dtype=np.int64)
    scan_B.get_first_valid_column_timestamp.return_value = np.int64(1200)
    scan_B.get_last_valid_column_timestamp.return_value = np.int64(1240)

    ls_list = [scan_A, scan_B]
    mock_xyzlut_list = [MockXYZLut(), MockXYZLut()]

    ts_offsets = [0, 0]
    _, _, normalized_ts = util.get_total_frame_pts_and_ts(
        ls_list, mock_xyzlut_list, ts_offsets
    )

    # Manually calculate the expected normalized timestamps
    # total_start_ts = min(1000, 1200) = 1000
    # total_stop_ts = max(1040, 1240) = 1240
    # ts_dur = 240
    
    # For scan_A: (timestamps - 1000) / 240
    expected_A_ts = (np.array([1000, 1010, 1020, 1030, 1040]) - 1000) / 240.0
    # For scan_B: (timestamps - 1000) / 240
    expected_B_ts = (np.array([1200, 1210, 1220, 1230, 1240]) - 1000) / 240.0

    assert len(normalized_ts) == 2, "Should return normalized timestamps for two scans"

    assert np.allclose(normalized_ts[0], expected_A_ts, atol=1e-6), \
        "Normalized timestamps for scan A are incorrect"

    assert np.allclose(normalized_ts[1], expected_B_ts, atol=1e-6), \
        "Normalized timestamps for scan B are incorrect"


@pytest.mark.skipif(not kiss_icp_available or is_mac_arm, reason="No Platform Support or on mac arm")
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

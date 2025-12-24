#include <gtest/gtest.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>

#include <memory>
#include <vector>

#include "../../ouster_mapping/src/kiss_slam.h"
#include "../../ouster_mapping/src/slam_util.h"
#include "ouster/open_source.h"
#include "ouster/pcap_scan_source.h"
#include "ouster/slam_engine.h"

using namespace ouster::sdk::core;
using namespace ouster::sdk::mapping;
using namespace ouster::sdk::mapping::impl;

// Helper functions for test setup
std::shared_ptr<SensorInfo> create_mock_sensor_info(int w = 5, int h = 5) {
    auto info = std::make_shared<SensorInfo>();
    info->format.columns_per_frame = w;
    info->format.pixels_per_column = h;
    info->format.fps = 10;
    info->beam_altitude_angles = std::vector<double>(h, 1.0);
    info->beam_azimuth_angles = std::vector<double>(h, 1.0);
    info->format.udp_profile_lidar = UDPProfileLidar::PROFILE_LIDAR_LEGACY;
    info->format.udp_profile_imu = UDPProfileIMU::PROFILE_IMU_LEGACY;
    return info;
}

XYZLut create_mock_xyzlut(int w = 5, int h = 5) {
    XYZLut lut;
    lut.direction = Eigen::Array<double, Eigen::Dynamic, 3>::Zero(w * h, 3);
    lut.offset = Eigen::Array<double, Eigen::Dynamic, 3>::Zero(w * h, 3);
    return lut;
}

std::shared_ptr<LidarScan> create_mock_lidarscan(int w = 5, int h = 5,
                                                 int frame_id = 1) {
    auto scan = std::make_shared<LidarScan>(w, h);
    scan->frame_id = frame_id;

    Eigen::Matrix4d zero = Eigen::Matrix4d::Zero();
    for (size_t col = 0; col < scan->w; ++col) {
        scan->set_column_pose(col, zero);
    }

    // Add RANGE field to the scan
    // scan->add_field(ChanField::RANGE, ouster::fd_array<uint32_t>(h, w));
    auto range = scan->field<uint32_t>(ChanField::RANGE);
    for (int row = 0; row < h; ++row) {
        range.row(row) << 0, 1, 2, 0, 3;
    }

    scan->status() << 1, 1, 1, 1, 1;

    return scan;
}

// Tests
TEST(SlamTest, CheckMonotonicIncreaseTS) {
    auto mock_info = create_mock_sensor_info();

    SlamConfig slam_config;
    slam_config.voxel_size = 2;
    KissSlam kiss_slam({mock_info}, slam_config);

    ActiveTimeCorrection ts_correct({mock_info});

    auto scan1 = create_mock_lidarscan();
    scan1->timestamp() << 100, 200, 300, 400, 500;
    ts_correct.last_frame_ts_range() = {{-1, 90}};
    EXPECT_TRUE(ActiveTimeCorrection::is_monotonically_increasing(
        scan1->timestamp(), 90));

    auto scan2 = create_mock_lidarscan();
    scan2->timestamp() << 100, 200, 150, 400, 500;
    EXPECT_FALSE(ActiveTimeCorrection::is_monotonically_increasing(
        scan2->timestamp(), 90));
    auto scan3 = create_mock_lidarscan();
    scan3->timestamp() << 80, 200, 300, 400, 500;
    ts_correct.last_frame_ts_range() = {{-1, 90}};
    EXPECT_FALSE(ActiveTimeCorrection::is_monotonically_increasing(
        scan3->timestamp(), 90));
    auto scan4 = create_mock_lidarscan();
    scan4->timestamp() << 100, 0, 300, 0, 500;
    ts_correct.last_frame_ts_range() = {{-1, 90}};
    EXPECT_TRUE(ActiveTimeCorrection::is_monotonically_increasing(
        scan4->timestamp(), 90));
    std::vector<std::shared_ptr<LidarScan>> scans_mixed = {scan1, scan2, scan3,
                                                           scan4, nullptr};
    KissSlam kiss_slam_multi(
        {mock_info, mock_info, mock_info, mock_info, mock_info}, slam_config);
    ts_correct.last_frame_ts_range() =
        std::vector<std::pair<int64_t, int64_t>>(5, {-1, 90});
    std::vector<bool> expected_results = {true, false, false, true, true};
    std::vector<bool> generated_results;
    for (size_t index = 0; index < scans_mixed.size(); ++index) {
        const auto scan = scans_mixed[index];
        bool result = scan ? ActiveTimeCorrection::is_monotonically_increasing(
                                 scan->timestamp(), 90)
                           : true;
        generated_results.push_back(result);
    }
    EXPECT_EQ(generated_results, expected_results);
}

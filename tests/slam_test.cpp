#include <gtest/gtest.h>
#include <ouster/core/lidar_frame.h>
#include <ouster/core/types.h>

#include <memory>
#include <vector>

#include "../../ouster_mapping/src/lio_slam.h"
#include "../../ouster_mapping/src/slam_util.h"
#include "ouster/mapping/slam_engine.h"

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
    info->format.udp_profile_lidar = UDPProfileLidar::LEGACY;
    info->format.udp_profile_imu = UDPProfileIMU::LEGACY;
    return info;
}

XYZLut create_mock_xyzlut(int w = 5, int h = 5) {
    return XYZLut(Eigen::Array<double, Eigen::Dynamic, 3>::Zero(w * h, 3),
                  Eigen::Array<double, Eigen::Dynamic, 3>::Zero(w * h, 3), h, w);
}

std::shared_ptr<LidarFrame> create_mock_lidar_frame(int w = 5, int h = 5, int frame_id = 1) {
    auto frame = std::make_shared<LidarFrame>(h, w);
    frame->frame_id = frame_id;

    Matrix4dR zero = Matrix4dR::Zero();
    for (size_t col = 0; col < frame->w; ++col) {
        frame->set_column_pose(col, zero);
    }

    // Add RANGE field to the frame
    // frame->add_field(ChanField::RANGE, ouster::fd_array<uint32_t>(h, w));
    auto range = frame->field<uint32_t>(ChanField::RANGE);
    for (int row = 0; row < h; ++row) {
        range.row(row) << 0, 1, 2, 0, 3;
    }

    frame->status() << 1, 1, 1, 1, 1;

    return frame;
}

// Tests
TEST(SlamTest, CheckMonotonicIncreaseTS) {
    auto mock_info = create_mock_sensor_info();

    LIOSlamConfig slam_config;
    slam_config.voxel_size = 2;
    LIOSlam lio_slam({mock_info}, slam_config);

    ActiveTimeCorrection ts_correct({mock_info});

    auto frame1 = create_mock_lidar_frame();
    frame1->timestamp() << 100, 200, 300, 400, 500;
    ts_correct.last_frame_ts_range() = {{-1, 90}};
    EXPECT_TRUE(ActiveTimeCorrection::is_monotonically_increasing(frame1->timestamp(), 90));

    auto frame2 = create_mock_lidar_frame();
    frame2->timestamp() << 100, 200, 150, 400, 500;
    EXPECT_FALSE(ActiveTimeCorrection::is_monotonically_increasing(frame2->timestamp(), 90));
    auto frame3 = create_mock_lidar_frame();
    frame3->timestamp() << 80, 200, 300, 400, 500;
    ts_correct.last_frame_ts_range() = {{-1, 90}};
    EXPECT_FALSE(ActiveTimeCorrection::is_monotonically_increasing(frame3->timestamp(), 90));
    auto frame4 = create_mock_lidar_frame();
    frame4->timestamp() << 100, 0, 300, 0, 500;
    ts_correct.last_frame_ts_range() = {{-1, 90}};
    EXPECT_TRUE(ActiveTimeCorrection::is_monotonically_increasing(frame4->timestamp(), 90));
    std::vector<std::shared_ptr<LidarFrame>> frames_mixed = {frame1, frame2, frame3, frame4,
                                                             nullptr};
    LIOSlam lio_slam_multi({mock_info, mock_info, mock_info, mock_info, mock_info}, slam_config);
    ts_correct.last_frame_ts_range() = std::vector<std::pair<int64_t, int64_t>>(5, {-1, 90});
    std::vector<bool> expected_results = {true, false, false, true, true};
    std::vector<bool> generated_results;
    for (size_t index = 0; index < frames_mixed.size(); ++index) {
        const auto frame = frames_mixed[index];
        bool result =
            frame ? ActiveTimeCorrection::is_monotonically_increasing(frame->timestamp(), 90)
                  : true;
        generated_results.push_back(result);
    }
    EXPECT_EQ(generated_results, expected_results);
}

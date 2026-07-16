/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/object_util.h"

#include <gtest/gtest.h>

#include <memory>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/pose_util.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/types.h"

namespace ouster {
namespace sdk {
namespace core {
namespace {

constexpr double kTol = 1e-6;

std::shared_ptr<SensorInfo> make_sensor_info(int h, int w) {
    SensorInfo info;
    info.format.columns_per_packet = 16;
    info.format.pixels_per_column = h;
    info.format.columns_per_frame = w;
    info.format.udp_profile_lidar = UDPProfileLidar::LEGACY;
    info.format.udp_profile_imu = UDPProfileIMU::LEGACY;
    info.fw_rev = "3.2.1";
    info.image_rev = "3.2.1";
    return std::make_shared<SensorInfo>(std::move(info));
}

void set_column_valid(LidarFrame& frame, int col, bool valid) {
    frame.status()[col] = valid ? 1u : 0u;
}

void set_column_pose(LidarFrame& frame, int col, const Matrix4dR& matrix) {
    double* ptr = frame.body_to_world().get<double>();
    Eigen::Map<Matrix4dR>(ptr + col * 16, 4, 4) = matrix;
}

Matrix4dR translation_pose(double x, double y, double z) {
    Matrix4dR pose = Matrix4dR::Identity();
    pose(0, 3) = x;
    pose(1, 3) = y;
    pose(2, 3) = z;
    return pose;
}

void expect_pose_near(const Pose& actual, const Matrix4dR& expected, double tol = kTol) {
    EXPECT_TRUE(actual.to_matrix().isApprox(expected, tol));
}

LidarFrame make_frame_with_valid_columns(int w) {
    LidarFrame frame(make_sensor_info(1, w));
    for (int col = 0; col < w; ++col) {
        set_column_valid(frame, col, true);
    }
    return frame;
}

}  // namespace

TEST(PoseAtTimestamp, ThrowsWhenNoValidColumns) {
    LidarFrame frame(make_sensor_info(1, 4));
    frame.timestamp() << 100, 200, 300, 400;

    EXPECT_THROW(pose_at_timestamp(frame, 250), std::invalid_argument);
}

TEST(PoseAtTimestamp, ReturnsExactPoseWhenTimestampMatchesValidColumn) {
    LidarFrame frame = make_frame_with_valid_columns(3);
    frame.timestamp() << 100, 200, 300;

    const Matrix4dR expected = translation_pose(1.0, 2.0, 3.0);
    set_column_pose(frame, 1, expected);

    expect_pose_near(pose_at_timestamp(frame, 200), expected);
}

TEST(PoseAtTimestamp, InterpolatesBetweenTwoValidColumns) {
    LidarFrame frame = make_frame_with_valid_columns(3);
    frame.timestamp() << 100, 200, 300;
    set_column_valid(frame, 1, false);

    const Matrix4dR pose_before = translation_pose(0.0, 0.0, 0.0);
    const Matrix4dR pose_after = translation_pose(10.0, 0.0, 0.0);
    set_column_pose(frame, 0, pose_before);
    set_column_pose(frame, 2, pose_after);

    const std::vector<int64_t> x_interp{150};
    const Matrix4dR expected =
        interp_pose(x_interp, int64_t{100}, pose_before, int64_t{300}, pose_after)[0];
    expect_pose_near(pose_at_timestamp(frame, 150), expected);
}

TEST(PoseAtTimestamp, SkipsInvalidColumnsWhenBracketing) {
    LidarFrame frame = make_frame_with_valid_columns(3);
    frame.timestamp() << 100, 150, 200;
    set_column_valid(frame, 1, false);

    const Matrix4dR pose_before = translation_pose(0.0, 0.0, 0.0);
    const Matrix4dR pose_after = translation_pose(4.0, 0.0, 0.0);
    set_column_pose(frame, 0, pose_before);
    set_column_pose(frame, 2, pose_after);

    const std::vector<int64_t> x_interp{125};
    const Matrix4dR expected =
        interp_pose(x_interp, int64_t{100}, pose_before, int64_t{200}, pose_after)[0];
    expect_pose_near(pose_at_timestamp(frame, 125), expected);
}

TEST(PoseAtTimestamp, ThrowsWhenTimestampBeforeFirstValidColumn) {
    LidarFrame frame = make_frame_with_valid_columns(3);
    frame.timestamp() << 100, 200, 300;

    EXPECT_THROW(pose_at_timestamp(frame, 50), std::invalid_argument);
}

TEST(PoseAtTimestamp, ThrowsWhenTimestampAfterLastValidColumn) {
    LidarFrame frame = make_frame_with_valid_columns(3);
    frame.timestamp() << 100, 200, 300;

    EXPECT_THROW(pose_at_timestamp(frame, 350), std::invalid_argument);
}

TEST(PoseAtTimestamp, ThrowsWhenTimestampOutsidePartialValidRange) {
    LidarFrame frame(make_sensor_info(1, 4));
    frame.timestamp() << 100, 200, 300, 400;
    set_column_valid(frame, 2, true);
    set_column_valid(frame, 3, true);

    EXPECT_THROW(pose_at_timestamp(frame, 250), std::invalid_argument);
    expect_pose_near(pose_at_timestamp(frame, 350), translation_pose(0.0, 0.0, 0.0));
    EXPECT_THROW(pose_at_timestamp(frame, 450), std::invalid_argument);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

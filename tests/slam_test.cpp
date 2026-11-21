#include <gtest/gtest.h>
#include <ouster/core/camera_frame.h>
#include <ouster/core/camera_info.h>
#include <ouster/core/field.h>
#include <ouster/core/frame.h>
#include <ouster/core/frame_set.h>
#include <ouster/core/lidar_frame.h>
#include <ouster/core/pose_util.h>
#include <ouster/core/types.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "../../ouster_mapping/src/lio_slam.h"
#include "../../ouster_mapping/src/slam_util.h"
#include "ouster/mapping/slam_engine.h"

using namespace ouster::sdk::core;
using namespace ouster::sdk::mapping;
using namespace ouster::sdk::mapping::impl;

// Helper functions for test setup
std::shared_ptr<LidarInfo> create_mock_sensor_info(int w = 5, int h = 5) {
    auto info = std::make_shared<LidarInfo>();
    info->format.columns_per_frame = w;
    info->format.pixels_per_column = h;
    info->format.fps = 10;
    info->format.columns_per_packet = 16;
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

std::shared_ptr<LidarFrame> create_mock_lidar_frame(std::shared_ptr<LidarInfo> info,
                                                    int frame_id = 1) {
    auto frame = std::make_shared<LidarFrame>(info);
    frame->frame_id = frame_id;

    Matrix4dR zero = Matrix4dR::Zero();
    for (size_t col = 0; col < frame->w; ++col) {
        frame->set_column_pose(col, zero);
    }

    // Add RANGE field to the frame
    // frame->add_field(ChanField::RANGE, ouster::fd_array<uint32_t>(h, w));
    auto range = frame->field<uint32_t>(ChanField::RANGE);
    for (size_t row = 0; row < frame->h; ++row) {
        range.row(row) << 0, 1, 2, 0, 3;
    }

    frame->status() << 1, 1, 1, 1, 1;

    return frame;
}

std::shared_ptr<CameraInfo> create_mock_camera_info(int w = 4, int h = 3) {
    auto info = std::make_shared<CameraInfo>();
    info->model = CameraModel(100.0f, 100.0f, static_cast<float>(w - 1) / 2.0f,
                              static_cast<float>(h - 1) / 2.0f);
    info->format.columns_per_frame = static_cast<uint32_t>(w);
    info->format.pixels_per_column = static_cast<uint32_t>(h);
    // The shared XYZLut derives per-pixel direction/offset from the pinhole
    // intrinsics, so no beam angles are stored on the CameraInfo.
    return info;
}

// Radial range (mm) that a planar depth (m) maps to for pixel (row v, col u).
uint32_t pixel_range_mm(const CameraInfo& info, int u, int v, double depth_m) {
    const double fx = info.model->fx;
    const double fy = info.model->fy;
    const double cx = info.model->cx;
    const double cy = info.model->cy;
    const double x_cam = (static_cast<double>(u) - cx) / fx;
    const double y_cam = (static_cast<double>(v) - cy) / fy;
    const double m = std::sqrt(1.0 + (x_cam * x_cam) + (y_cam * y_cam));
    return static_cast<uint32_t>((depth_m * m * 1000.0) + 0.5);
}

std::shared_ptr<CameraFrame> create_range_camera_frame(std::shared_ptr<CameraInfo> info, int w,
                                                       int h, uint32_t range_mm, uint64_t ts) {
    auto camera = std::make_shared<CameraFrame>(info);
    camera->w = static_cast<size_t>(w);
    camera->h = static_cast<size_t>(h);
    camera->add_field("RANGE", fd_array<uint32_t>(h, w));
    camera->field<uint32_t>("RANGE").setConstant(range_mm);
    camera->timestamp()[0] = ts;
    return camera;
}

// Tests
TEST(SlamTest, CheckMonotonicIncreaseTS) {
    auto mock_info = create_mock_sensor_info();

    LIOSlamConfig slam_config;
    slam_config.voxel_size = 2;
    LIOSlam lio_slam({mock_info}, slam_config);

    ActiveTimeCorrection ts_correct({mock_info});

    auto frame1 = create_mock_lidar_frame(mock_info);
    frame1->timestamp() << 100, 200, 300, 400, 500;
    ts_correct.last_frame_ts_range() = {{-1, 90}};
    EXPECT_TRUE(ActiveTimeCorrection::is_monotonically_increasing(frame1->timestamp(), 90));

    auto frame2 = create_mock_lidar_frame(mock_info);
    frame2->timestamp() << 100, 200, 150, 400, 500;
    EXPECT_FALSE(ActiveTimeCorrection::is_monotonically_increasing(frame2->timestamp(), 90));
    auto frame3 = create_mock_lidar_frame(mock_info);
    frame3->timestamp() << 80, 200, 300, 400, 500;
    ts_correct.last_frame_ts_range() = {{-1, 90}};
    EXPECT_FALSE(ActiveTimeCorrection::is_monotonically_increasing(frame3->timestamp(), 90));
    auto frame4 = create_mock_lidar_frame(mock_info);
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

namespace {

constexpr uint64_t NS_PER_MS = 1000000;

std::shared_ptr<LidarFrame> create_timed_lidar_frame(std::shared_ptr<LidarInfo> info,
                                                     uint64_t start_ts, uint64_t receive_ts,
                                                     int frame_id) {
    auto lidar = create_mock_lidar_frame(std::move(info), frame_id);
    lidar->timestamp() << start_ts, start_ts + (20 * NS_PER_MS), start_ts + (40 * NS_PER_MS),
        start_ts + (60 * NS_PER_MS), start_ts + (80 * NS_PER_MS);
    lidar->receive_timestamp().setConstant(receive_ts);
    return lidar;
}

std::shared_ptr<CameraFrame> create_timed_camera_frame(std::shared_ptr<CameraInfo> info,
                                                       uint64_t timestamp, uint64_t receive_ts) {
    auto camera = create_range_camera_frame(std::move(info), 4, 3, 2000, timestamp);
    camera->receive_timestamp()[0] = receive_ts;
    return camera;
}

}  // namespace

TEST(SlamTest, ActiveTimeCorrectionShiftsCameraToHostClock) {
    auto lidar_info = create_mock_sensor_info();
    auto camera_info = create_mock_camera_info();
    camera_info->format.fps = 30;
    ActiveTimeCorrection ts_correct({lidar_info, camera_info});

    // The camera runs on its own clock, far from the lidar's.
    const uint64_t lidar_start = 1000 * NS_PER_MS;
    const uint64_t camera_ts = 9000 * NS_PER_MS;
    auto lidar = create_timed_lidar_frame(lidar_info, lidar_start, 5000 * NS_PER_MS, 1);
    auto camera = create_timed_camera_frame(camera_info, camera_ts, 5030 * NS_PER_MS);
    FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{lidar, camera});

    ts_correct.update(frame_set);
    EXPECT_EQ(lidar->timestamp()[0], 5000 * NS_PER_MS);
    EXPECT_EQ(lidar->timestamp()[4], 5080 * NS_PER_MS);
    EXPECT_EQ(camera->timestamp()[0], 5030 * NS_PER_MS);

    ts_correct.reset(frame_set);
    EXPECT_EQ(lidar->timestamp()[0], lidar_start);
    EXPECT_EQ(lidar->timestamp()[4], lidar_start + (80 * NS_PER_MS));
    EXPECT_EQ(camera->timestamp()[0], camera_ts);
}

TEST(SlamTest, ActiveTimeCorrectionSyncedCameraKeepsSensorClock) {
    auto lidar_info = create_mock_sensor_info();
    auto camera_info = create_mock_camera_info();
    // A camera faster than the lidar must not tighten the sync tolerance: its
    // exposure may land anywhere within the lidar sweep.
    camera_info->format.fps = 30;
    ActiveTimeCorrection ts_correct({lidar_info, camera_info});

    auto lidar = create_timed_lidar_frame(lidar_info, 1000 * NS_PER_MS, 5000 * NS_PER_MS, 1);
    auto camera = create_timed_camera_frame(camera_info, 1050 * NS_PER_MS, 5060 * NS_PER_MS);
    FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{lidar, camera});
    ts_correct.update(frame_set);
    EXPECT_EQ(lidar->timestamp()[0], 1000 * NS_PER_MS);
    EXPECT_EQ(camera->timestamp()[0], 1050 * NS_PER_MS);
    ts_correct.reset(frame_set);

    // A slower camera leaves its slot empty; that must not be read as a clock
    // disagreement.
    auto lidar2 = create_timed_lidar_frame(lidar_info, 1100 * NS_PER_MS, 5100 * NS_PER_MS, 2);
    FrameSet frame_set2(std::vector<std::shared_ptr<Frame>>{lidar2, nullptr});
    ts_correct.update(frame_set2);
    EXPECT_EQ(lidar2->timestamp()[0], 1100 * NS_PER_MS);
    ts_correct.reset(frame_set2);

    auto lidar3 = create_timed_lidar_frame(lidar_info, 1200 * NS_PER_MS, 5200 * NS_PER_MS, 3);
    auto camera3 = create_timed_camera_frame(camera_info, 1250 * NS_PER_MS, 5260 * NS_PER_MS);
    FrameSet frame_set3(std::vector<std::shared_ptr<Frame>>{lidar3, camera3});
    ts_correct.update(frame_set3);
    EXPECT_EQ(lidar3->timestamp()[0], 1200 * NS_PER_MS);
    EXPECT_EQ(camera3->timestamp()[0], 1250 * NS_PER_MS);
    ts_correct.reset(frame_set3);
}

TEST(SlamTest, ActiveTimeCorrectionDetectsBackwardsCameraTimestamp) {
    auto camera_info = create_mock_camera_info();
    camera_info->format.fps = 10;
    ActiveTimeCorrection ts_correct({camera_info});

    auto camera1 = create_timed_camera_frame(camera_info, 2000 * NS_PER_MS, 7000 * NS_PER_MS);
    FrameSet frame_set1(std::vector<std::shared_ptr<Frame>>{camera1});
    ts_correct.update(frame_set1);
    EXPECT_EQ(camera1->timestamp()[0], 2000 * NS_PER_MS);
    ts_correct.reset(frame_set1);

    // A single timestamp must still be checked against the previous frame.
    auto camera2 = create_timed_camera_frame(camera_info, 1500 * NS_PER_MS, 7100 * NS_PER_MS);
    EXPECT_FALSE(ActiveTimeCorrection::is_monotonically_increasing(
        camera2->timestamp(), static_cast<int64_t>(2000 * NS_PER_MS)));

    FrameSet frame_set2(std::vector<std::shared_ptr<Frame>>{camera2});
    ts_correct.update(frame_set2);
    EXPECT_EQ(camera2->timestamp()[0], 7100 * NS_PER_MS);
    ts_correct.reset(frame_set2);
    EXPECT_EQ(camera2->timestamp()[0], 1500 * NS_PER_MS);
}

TEST(SlamTest, UpdateCameraPosesInterpolatesFromLidar) {
    auto mock_info = create_mock_sensor_info();
    auto lidar = create_mock_lidar_frame(mock_info);
    lidar->timestamp() << 100, 200, 300, 400, 500;
    lidar->status().setConstant(1);

    Matrix4dR pose = Matrix4dR::Identity();
    pose(0, 3) = 1.5;
    for (size_t col = 0; col < lidar->w; ++col) {
        lidar->set_column_pose(static_cast<int>(col), pose);
    }

    auto camera = std::make_shared<CameraFrame>();
    camera->timestamp()[0] = 300;

    FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{lidar, camera});
    update_camera_poses(frame_set);

    EXPECT_TRUE(camera->get_column_pose(0).isApprox(pose));
}

TEST(SlamTest, UpdateCameraPosesLeavesUnbracketedCameraUnchanged) {
    auto mock_info = create_mock_sensor_info();
    auto lidar = create_mock_lidar_frame(mock_info);
    lidar->timestamp() << 100, 200, 300, 400, 500;
    lidar->status().setConstant(1);

    auto camera = std::make_shared<CameraFrame>();
    camera->timestamp()[0] = 50;

    FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{lidar, camera});
    update_camera_poses(frame_set);

    EXPECT_TRUE(camera->get_column_pose(0).isApprox(Matrix4dR::Identity()));
}

TEST(SlamTest, SlamEngineCreateAcceptsCameraInfos) {
    std::vector<std::shared_ptr<SensorInfo>> sensor_infos{create_mock_sensor_info(),
                                                          std::make_shared<CameraInfo>()};
    LIOSlamConfig config;
    config.voxel_size = 0.5;
    auto engine = SlamEngine::create(sensor_infos, config);
    ASSERT_NE(engine, nullptr);
}

TEST(SlamTest, SlamEngineCreateAcceptsCameraOnly) {
    std::vector<std::shared_ptr<SensorInfo>> sensor_infos{std::make_shared<CameraInfo>()};
    LIOSlamConfig config;
    config.voxel_size = 0.5;
    auto engine = SlamEngine::create(sensor_infos, config);
    ASSERT_NE(engine, nullptr);
}

TEST(SlamTest, SlamEngineCreateAcceptsMixedSensorInfoList) {
    // A single vector<SensorInfo> (as returned by FrameSetSource::sensor_info())
    // holding a mix of lidar and camera calibration, in any order.
    std::vector<std::shared_ptr<SensorInfo>> sensor_infos{create_mock_sensor_info(),
                                                          std::make_shared<CameraInfo>()};
    LIOSlamConfig config;
    config.voxel_size = 0.5;
    auto engine = SlamEngine::create(sensor_infos, config);
    ASSERT_NE(engine, nullptr);
}

TEST(SlamTest, SlamEngineCreateAcceptsCameraBeforeLidarOrder) {
    // Order-independent: camera entry precedes lidar in both the sensor-info
    // list and (by convention) the FrameSet.
    std::vector<std::shared_ptr<SensorInfo>> sensor_infos{std::make_shared<CameraInfo>(),
                                                          create_mock_sensor_info()};
    LIOSlamConfig config;
    config.voxel_size = 0.5;
    auto engine = SlamEngine::create(sensor_infos, config);
    ASSERT_NE(engine, nullptr);
}

TEST(SlamTest, CameraRangeDewarpProjectsPinholeCloud) {
    const int w = 4;
    const int h = 3;
    auto info = create_mock_camera_info(w, h);
    auto camera = create_range_camera_frame(info, w, h, 0, 100);
    const int pu = 2;
    const int pv = 1;
    const double z = 2.0;
    camera->field<uint32_t>("RANGE").setZero();
    camera->field<uint32_t>("RANGE")(pv, pu) = pixel_range_mm(*info, pu, pv, z);

    // A CameraFrame carries a single (default identity) pose. The shared dewarp
    // path applies it rigidly to the pinhole RANGE projection, exactly as it
    // applies per-column poses for a LidarFrame; only the pose count differs.
    const XYZLut lut(*info);
    const auto points = ouster::sdk::core::dewarp<double>(*camera, lut, 0.0, 150.0);

    ASSERT_EQ(points.size(), 1u);
    // Optical-frame coordinates from the pinhole model (+u right, +v down, depth
    // along the optical axis).
    const double opt_x = ((static_cast<double>(pu) - info->model->cx) * z) / info->model->fx;
    const double opt_y = ((static_cast<double>(pv) - info->model->cy) * z) / info->model->fy;
    // The shared XYZLut emits points in the Ouster sensor frame (X forward,
    // Y left, Z up): x = depth, y = -opt_x, z = -opt_y.
    EXPECT_NEAR(points[0].x(), z, 1e-3);
    EXPECT_NEAR(points[0].y(), -opt_x, 1e-3);
    EXPECT_NEAR(points[0].z(), -opt_y, 1e-3);
}

TEST(SlamTest, FrameSetDewarpAppliesSensorToBodyAndSkipsFramesWithoutInfo) {
    const int w = 4;
    const int h = 3;
    auto info = create_mock_camera_info(w, h);
    info->sensor_to_body(0, 3) = 1.0;
    info->sensor_to_body(1, 3) = 2.0;
    info->sensor_to_body(2, 3) = 3.0;
    auto camera = create_range_camera_frame(info, w, h, 0, 100);
    camera->field<uint32_t>("RANGE")(1, 2) = pixel_range_mm(*info, 2, 1, 2.0);

    auto no_info = std::make_shared<CameraFrame>();
    no_info->w = static_cast<size_t>(w);
    no_info->h = static_cast<size_t>(h);
    no_info->add_field("RANGE", fd_array<uint32_t>(h, w));
    no_info->field<uint32_t>("RANGE").setConstant(2000);

    const FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{camera, no_info});
    const auto points = ouster::sdk::core::dewarp<double>(frame_set, 0.0, 150.0);
    const auto expected =
        ouster::sdk::core::dewarp<double>(*camera, *info->xyzlut<double>(), 0.0, 150.0);

    ASSERT_EQ(points.size(), 1u);
    ASSERT_EQ(expected.size(), 1u);
    EXPECT_TRUE(points[0].isApprox(expected[0]));
    // The cached LUT bakes in sensor_to_body, so the point lands in the body frame.
    EXPECT_NEAR(points[0].x(), 2.0 + 1.0, 1e-3);
}

TEST(SlamTest, FrameSetDewarpSkipsCameraWithoutModel) {
    const int w = 4;
    const int h = 3;
    auto info = create_mock_camera_info(w, h);
    auto camera = create_range_camera_frame(info, w, h, 0, 100);
    camera->field<uint32_t>("RANGE")(1, 2) = pixel_range_mm(*info, 2, 1, 2.0);

    auto no_model_info = create_mock_camera_info(w, h);
    no_model_info->model = nonstd::nullopt;
    auto no_model = create_range_camera_frame(no_model_info, w, h, 2000, 100);

    const FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{no_model, camera});
    std::vector<Eigen::Vector3d> points;
    ASSERT_NO_THROW(points = ouster::sdk::core::dewarp<double>(frame_set, 0.0, 150.0));
    ASSERT_EQ(points.size(), 1u);
    EXPECT_NEAR(points[0].x(), 2.0, 1e-3);
}

TEST(SlamTest, FrameSetDewarpSkipsCameraWithZeroDimensions) {
    const int w = 4;
    const int h = 3;
    auto info = create_mock_camera_info(w, h);
    auto camera = create_range_camera_frame(info, w, h, 0, 100);
    camera->field<uint32_t>("RANGE")(1, 2) = pixel_range_mm(*info, 2, 1, 2.0);

    auto zero_dims_info = create_mock_camera_info(w, h);
    zero_dims_info->format.columns_per_frame = 0;
    zero_dims_info->format.pixels_per_column = 0;
    auto zero_dims = create_range_camera_frame(zero_dims_info, w, h, 2000, 100);

    const FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{zero_dims, camera});
    std::vector<Eigen::Vector3d> points;
    ASSERT_NO_THROW(points = ouster::sdk::core::dewarp<double>(frame_set, 0.0, 150.0));
    ASSERT_EQ(points.size(), 1u);
    EXPECT_NEAR(points[0].x(), 2.0, 1e-3);
}

TEST(SlamTest, FrameSetDewarpSkipsCameraWhoseLutCannotBeBuilt) {
    const int w = 4;
    const int h = 3;
    auto info = create_mock_camera_info(w, h);
    auto camera = create_range_camera_frame(info, w, h, 0, 100);
    camera->field<uint32_t>("RANGE")(1, 2) = pixel_range_mm(*info, 2, 1, 2.0);

    auto bad_model_info = create_mock_camera_info(w, h);
    bad_model_info->model->fx = 0.0f;
    auto bad_model = create_range_camera_frame(bad_model_info, w, h, 2000, 100);

    const FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{bad_model, camera});
    std::vector<Eigen::Vector3d> points;
    ASSERT_NO_THROW(points = ouster::sdk::core::dewarp<double>(frame_set, 0.0, 150.0));
    ASSERT_EQ(points.size(), 1u);
    EXPECT_NEAR(points[0].x(), 2.0, 1e-3);
}

TEST(SlamTest, FrameSetDewarpPropagatesLidarLutErrors) {
    auto info = create_mock_sensor_info();
    auto lidar = create_mock_lidar_frame(info);
    info->beam_altitude_angles.resize(1);

    const FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{lidar});
    EXPECT_THROW(ouster::sdk::core::dewarp<double>(frame_set, 0.0, 150.0), std::invalid_argument);
}

TEST(SlamTest, SlamUpdateSkipsCameraWhoseLutCannotBeBuilt) {
    const int w = 8;
    const int h = 8;
    auto info = create_mock_camera_info(w, h);
    auto bad_model_info = create_mock_camera_info(w, h);
    bad_model_info->model->fx = 0.0f;
    LIOSlamConfig config;
    config.voxel_size = 0.25;
    config.min_range = 0.1;
    config.max_range = 50.0;
    LIOSlam slam(std::vector<std::shared_ptr<SensorInfo>>{info, bad_model_info}, config);

    for (int i = 0; i < 3; ++i) {
        const auto ts = static_cast<uint64_t>(1000 * (i + 1));
        auto camera = create_range_camera_frame(info, w, h, 0, ts);
        auto range = camera->field<uint32_t>("RANGE");
        range.block(2, 2, 4, 4).setConstant(3000u);
        range.block(3, 3, 2, 2).setConstant(2000u);
        auto bad_model = create_range_camera_frame(bad_model_info, w, h, 2000, ts);
        FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{camera, bad_model});
        ASSERT_NO_THROW(slam.update(frame_set));
    }

    EXPECT_GT(slam.get_point_cloud().rows(), 0);
}

TEST(SlamTest, ComputeFrameTsRangeIncludesCameraTimestamp) {
    auto info = create_mock_camera_info();
    auto camera = create_range_camera_frame(info, 4, 3, 2000, 250);
    FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{camera});
    auto ts_range = compute_frame_ts_range(frame_set);
    EXPECT_EQ(ts_range.first, 250u);
    EXPECT_EQ(ts_range.second, 250u);
}

TEST(SlamTest, CameraOnlySlamUpdateBuildsMapFromRange) {
    auto info = create_mock_camera_info(8, 8);
    LIOSlamConfig config;
    config.voxel_size = 0.25;
    config.min_range = 0.1;
    config.max_range = 50.0;
    LIOSlam slam(std::vector<std::shared_ptr<SensorInfo>>{info}, config);

    for (int i = 0; i < 3; ++i) {
        auto camera =
            create_range_camera_frame(info, 8, 8, 0, static_cast<uint64_t>(1000 * (i + 1)));
        auto range = camera->field<uint32_t>("RANGE");
        range.block(2, 2, 4, 4).setConstant(3000u);
        range.block(3, 3, 2, 2).setConstant(2000u);
        FrameSet frame_set(std::vector<std::shared_ptr<Frame>>{camera});
        slam.update(frame_set);
    }

    const auto cloud = slam.get_point_cloud();
    EXPECT_GT(cloud.rows(), 0);
}

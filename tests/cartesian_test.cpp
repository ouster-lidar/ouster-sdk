/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/impl/cartesian.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "ouster/core/camera_info.h"
#include "ouster/core/lidar_config.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/xyzlut.h"
#include "util.h"

using ouster::sdk::core::CameraInfo;
using ouster::sdk::core::CameraModel;
using ouster::sdk::core::img_t;
using ouster::sdk::core::XYZLut;
using ouster::sdk::core::impl::cartesianT;

// A CameraInfo carrying per-pixel beam angles projected through the shared
// XYZLut with a RANGE image reproduces the old pinhole DEPTH projection:
// point = range_m * (ox, oy, oz) / |o|, where (ox, oy, oz) is the pixel ray in
// the Ouster sensor frame (X forward, Y left, Z up).
TEST(CameraXYZLutTest, RangeProjectionMatchesPinhole) {
    const size_t w = 4;
    const size_t h = 3;
    const double fx = 100.0;
    const double fy = 200.0;
    const double cx = 10.0;
    const double cy = 20.0;

    CameraInfo camera_info;
    camera_info.model = CameraModel(static_cast<float>(fx), static_cast<float>(fy),
                                    static_cast<float>(cx), static_cast<float>(cy));
    camera_info.format.columns_per_frame = static_cast<uint32_t>(w);
    camera_info.format.pixels_per_column = static_cast<uint32_t>(h);

    // Range scaling map |o| (mirrors the ZED source plugin) used to build the
    // radial RANGE image. The XYZLut derives direction/offset from the pinhole
    // intrinsics directly, so no beam angles are stored on the CameraInfo.
    std::vector<double> mag(w * h);
    for (size_t v = 0; v < h; ++v) {
        for (size_t u = 0; u < w; ++u) {
            const size_t i = (v * w) + u;
            const double x_cam = (static_cast<double>(u) - cx) / fx;
            const double y_cam = (static_cast<double>(v) - cy) / fy;
            const double ox = 1.0;
            const double oy = -x_cam;
            const double oz = -y_cam;
            mag[i] = std::sqrt((ox * ox) + (oy * oy) + (oz * oz));
        }
    }

    XYZLut lut(camera_info);

    const size_t pv = 1;
    const size_t pu = 2;
    const size_t pi = (pv * w) + pu;
    const double depth_m = 2.0;
    const uint32_t range_mm = static_cast<uint32_t>(depth_m * mag[pi] * 1000.0);

    img_t<uint32_t> range = img_t<uint32_t>::Zero(h, w);
    range(pv, pu) = range_mm;

    auto points = lut(range);

    const double r_m = static_cast<double>(range_mm) * 1e-3;
    const double x_cam = (static_cast<double>(pu) - cx) / fx;
    const double y_cam = (static_cast<double>(pv) - cy) / fy;
    const double ox = 1.0;
    const double oy = -x_cam;
    const double oz = -y_cam;
    const double m = mag[pi];

    EXPECT_NEAR(points(pi, 0), r_m * ox / m, 1e-9);
    EXPECT_NEAR(points(pi, 1), r_m * oy / m, 1e-9);
    EXPECT_NEAR(points(pi, 2), r_m * oz / m, 1e-9);
    EXPECT_DOUBLE_EQ(points(0, 0), 0.0);
}

using ouster::sdk::core::ArrayX3dR;
using ouster::sdk::core::ArrayX3fR;
using ouster::sdk::core::LidarInfo;
using ouster::sdk::core::LidarMode;
using ouster::sdk::core::PointCloudXYZd;
using ouster::sdk::core::PointCloudXYZf;

// Same as ouster::cartesian but uses single float precision
PointCloudXYZf cartesian_f(const Eigen::Ref<const img_t<uint32_t>>& range,
                           const ArrayX3fR& direction, const ArrayX3fR& offset) {
    if (range.cols() * range.rows() != direction.rows())
        throw std::invalid_argument("unexpected image dimensions");
    auto reshaped =
        Eigen::Map<const Eigen::ArrayX<uint32_t>>(range.data(), range.cols() * range.rows());
    auto nooffset = direction.colwise() * reshaped.cast<float>();
    return (nooffset == 0.0).select(nooffset, nooffset + offset).matrix();
}

// CameraModel's constructor rejects a non-positive focal length, but its
// members are public, so building the LUT re-checks them rather than dividing by
// whatever it is handed.
namespace {
bool lut_rejects(const CameraInfo& camera_info) {
    try {
        XYZLut lut(camera_info);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}
}  // namespace

TEST(CameraXYZLutTest, RejectsNonPositiveFocalLength) {
    CameraInfo camera_info;
    camera_info.model = CameraModel(100.0f, 100.0f, 2.0f, 1.0f);
    camera_info.format.columns_per_frame = 4;
    camera_info.format.pixels_per_column = 3;
    // The unmutated model builds a LUT fine.
    EXPECT_FALSE(lut_rejects(camera_info));

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::vector<std::pair<float, float>> bad_focals = {{0.0f, 100.0f},  {100.0f, 0.0f},
                                                             {-1.0f, 100.0f}, {100.0f, -1.0f},
                                                             {nan, 100.0f},   {100.0f, nan}};
    for (const auto& focal : bad_focals) {
        camera_info.model->fx = focal.first;
        camera_info.model->fy = focal.second;
        EXPECT_TRUE(lut_rejects(camera_info)) << "fx=" << focal.first << " fy=" << focal.second;
    }
}

class CartesianParameterizedTestFixture : public ::testing::TestWithParam<std::pair<int, int>> {
   protected:
    int frame_width;
    int frame_height;
};

INSTANTIATE_TEST_CASE_P(CartesianParametrisedTests, CartesianParameterizedTestFixture,
                        ::testing::Values(std::pair<int, int>{512, 128},
                                          std::pair<int, int>{1024, 128},
                                          std::pair<int, int>{2048, 128},
                                          std::pair<int, int>{4096, 128}));

TEST(CachedXYZLutTest, CachedLutIsSharedUntilCacheCleared) {
    const auto info = LidarInfo::from_default(LidarMode::_1024x10);
    const auto first = info->xyzlut<double>();
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(first, info->xyzlut<double>());

    info->clear_cache();
    const auto second = info->xyzlut<double>();
    EXPECT_NE(first, second);
}

TEST(CachedXYZLutTest, LutOutlivesClearCacheAndSensorInfo) {
    auto info = LidarInfo::from_default(LidarMode::_1024x10);
    img_t<uint32_t> range = img_t<uint32_t>::Constant(static_cast<Eigen::Index>(info->h()),
                                                      static_cast<Eigen::Index>(info->w()), 5000);
    const PointCloudXYZd expected = XYZLut(*info)(range);

    const auto lut = info->xyzlut<double>();
    info->clear_cache();
    info.reset();

    EXPECT_EQ(lut->w, 1024u);
    EXPECT_TRUE((*lut)(range).isApprox(expected));
}

TEST(CachedXYZLutTest, ConcurrentAccessAndClearCache) {
    const auto info = LidarInfo::from_default(LidarMode::_512x10);
    const img_t<uint32_t> range = img_t<uint32_t>::Constant(
        static_cast<Eigen::Index>(info->h()), static_cast<Eigen::Index>(info->w()), 5000);
    const PointCloudXYZd expected = XYZLut(*info)(range);

    const int num_threads = 4;
    std::atomic<bool> mismatch{false};
    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (int thread_idx = 0; thread_idx < num_threads; ++thread_idx) {
        threads.emplace_back([&info, &range, &expected, &mismatch, thread_idx]() {
            for (int i = 0; i < 50; ++i) {
                if (thread_idx == 0) {
                    info->clear_cache();
                    continue;
                }
                const auto lut = info->xyzlut<double>();
                if (!(*lut)(range).isApprox(expected)) {
                    mismatch = true;
                }
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }
    EXPECT_FALSE(mismatch);
}

TEST(CartesianParameterizedTestFixture, CartesianFunctionsMatch) {
    const auto WIDTH = 256;
    const auto HEIGHT = 32;
    const auto ROWS = WIDTH * HEIGHT;
    const auto COLS = 3;

    ArrayX3dR direction =
        0.5 * ArrayX3dR::Random(ROWS, COLS) + ArrayX3dR::Constant(ROWS, COLS, 1.0);
    ArrayX3dR offset =
        0.005 * (ArrayX3dR::Random(ROWS, COLS) + ArrayX3dR::Constant(ROWS, COLS, 1.0));
    XYZLut lut(direction, offset, HEIGHT, WIDTH);

    img_t<uint32_t> range = img_t<uint32_t>::Random(WIDTH, HEIGHT);

    auto points0 = lut(range);

    PointCloudXYZd points = PointCloudXYZd::Zero(ROWS, COLS);

    cartesianT<double>(points, range, direction, offset);
    EXPECT_TRUE(points.isApprox(points0));
}

TEST(CartesianParameterizedTestFixture, CartesianFunctionsMatchF) {
    const auto WIDTH = 256;
    const auto HEIGHT = 32;
    const auto ROWS = WIDTH * HEIGHT;
    const auto COLS = 3;

    ArrayX3dR direction =
        0.5 * ArrayX3dR::Random(ROWS, COLS) + ArrayX3dR::Constant(ROWS, COLS, 1.0);
    ArrayX3dR offset =
        0.005 * (ArrayX3dR::Random(ROWS, COLS) + ArrayX3dR::Constant(ROWS, COLS, 1.0));
    XYZLut lut(direction, offset, HEIGHT, WIDTH);

    ArrayX3fR directionF = direction.cast<float>();
    ArrayX3fR offsetF = offset.cast<float>();

    img_t<uint32_t> range = img_t<uint32_t>::Random(WIDTH, HEIGHT);

    auto points0 = lut(range);
    auto points0F = points0.cast<float>();

    PointCloudXYZf pointsF = PointCloudXYZf::Zero(ROWS, COLS);

    cartesianT<float>(pointsF, range, directionF, offsetF);
    EXPECT_TRUE(pointsF.isApprox(points0F));
}

TEST(CartesianTest, OddPixelCount) {
    const Eigen::Index width = 3;
    const Eigen::Index height = 3;
    const Eigen::Index rows = width * height;

    ArrayX3dR direction = ArrayX3dR::Random(rows, 3);
    ArrayX3dR offset = 0.01 * ArrayX3dR::Random(rows, 3);
    img_t<uint32_t> range(height, width);
    range << 0, 1000, 2000, 3000, 0, 4000, 5000, 6000, 7000;

    PointCloudXYZd expected = PointCloudXYZd::Zero(rows, 3);
    for (Eigen::Index i = 0; i < rows; ++i) {
        const auto r = range.data()[i];
        if (r != 0) {
            expected.row(i) = static_cast<double>(r) * direction.row(i) + offset.row(i);
        }
    }

    PointCloudXYZd points = PointCloudXYZd::Constant(rows, 3, 42.0);
    cartesianT<double>(points, range, direction, offset);
    EXPECT_TRUE(points.isApprox(expected));
    EXPECT_TRUE(points.row(rows - 1).isApprox(expected.row(rows - 1)));
}

TEST_P(CartesianParameterizedTestFixture, SpeedCheck) {
    std::map<std::string, std::string> styles = term_styles();

    const auto test_params = GetParam();
    const auto WIDTH = test_params.first;
    const auto HEIGHT = test_params.second;
    const auto ROWS = WIDTH * HEIGHT;
    const auto COLS = 3;
    std::cout << styles["yellow"] << styles["bold"] << "CHECKING PERFORMANCE FOR LIDAR MODE: ["
              << WIDTH << "x" << HEIGHT << "]" << styles["reset"] << std::endl;

    ArrayX3dR direction =
        0.5 * ArrayX3dR::Random(ROWS, COLS) + ArrayX3dR::Constant(ROWS, COLS, 1.0);
    ArrayX3dR offset =
        0.005 * (ArrayX3dR::Random(ROWS, COLS) + ArrayX3dR::Constant(ROWS, COLS, 1.0));
    XYZLut lut(direction, offset, HEIGHT, WIDTH);

    ArrayX3fR directionF = direction.cast<float>();
    ArrayX3fR offsetF = offset.cast<float>();

    // create an empty arrays of points
    PointCloudXYZd points = PointCloudXYZd(ROWS, COLS);
    PointCloudXYZf pointsF = PointCloudXYZf(ROWS, COLS);
    img_t<uint32_t> range = img_t<uint32_t>(WIDTH, HEIGHT);

    // By default run the shortest possible test unless OUSTER_PERF_COMPARISON
    // is set
    int N_FRAMES = enable_perf_comparison_tests() ? 100 : 1;

    constexpr int MOVING_AVG_WINDOW = 30;
    using MovingAverage64 = MovingAverage<int64_t, int64_t, MOVING_AVG_WINDOW>;
    static std::map<std::string, MovingAverage64> mv;

    Timer t;
    std::stringstream ss;
    int output_ctr = 0;

    using CartesianMethod = std::function<void(const img_t<uint32_t>& range)>;
    std::vector<std::pair<std::string, CartesianMethod>> all_cartesians;

    all_cartesians.emplace_back("c0", [&](const img_t<uint32_t>& range) { points = lut(range); });

    all_cartesians.emplace_back("c0f", [&](const img_t<uint32_t>& range) {
        pointsF = cartesian_f(range, directionF, offsetF);
    });

    all_cartesians.emplace_back("cT", [&](const img_t<uint32_t>& range) {
        cartesianT<double>(points, range, direction, offset);
    });

    all_cartesians.emplace_back("cfT", [&](const img_t<uint32_t>& range) {
        cartesianT<float>(pointsF, range, directionF, offsetF);
    });

    std::default_random_engine g;
    std::uniform_real_distribution<double> d(0.0, 1.0);
    std::vector<int> ids(all_cartesians.size());
    std::iota(std::begin(ids), std::end(ids), 0);

    for (auto i = 0; i < N_FRAMES; ++i) {
        auto p = std::ceil(10 * float(i) / float(N_FRAMES)) / 10;
        auto unary_expr = [&g, &d, p](auto) {
            return d(g) >= p ? 0U : static_cast<uint32_t>(d(g) * 10000);
        };

        auto valid_returns = (range != 0).count();
        auto percentage_valid = int(roundf(float(valid_returns) / float(range.size()) * 100));

        std::shuffle(std::begin(ids), std::end(ids), g);
        for (auto i : ids) {
            const auto& method = all_cartesians[i];
            range = range.unaryExpr(unary_expr);
            t.start();
            method.second(range);
            t.stop();
            mv[method.first](t.elapsed_microseconds());
        }

        if (++output_ctr % MOVING_AVG_WINDOW == 0) {
            ss.str("");
            ss << styles["bold"] << "returns: " << styles["reset"] << styles["magenta"]
               << std::setw(3) << percentage_valid << "%, " << styles["reset"];
            ss << styles["bold"] << "c0[time]: " << styles["reset"] << styles["cyan"]
               << std::setw(4) << mv["c0"] << "μs, " << styles["reset"];

            auto best_time = std::min_element(
                mv.begin(), mv.end(),
                [](const auto& a, const auto& b) -> bool { return a.second < b.second; });

            for (const auto& x : mv) {
                auto speedup = lround(100.0f * mv["c0"] / x.second);
                auto color_modifier = x.first == best_time->first
                                          ? styles["blue"]
                                          : (speedup >= 100 ? styles["green"] : styles["red"]);
                ss << styles["bold"] << x.first << ": " << color_modifier << std::setw(4) << speedup
                   << "%, " << styles["reset"];
            }
            std::cout << ss.str() << std::endl;
        }
    }
}

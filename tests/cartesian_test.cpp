/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <numeric>
#include <random>

#include "ouster/lidar_scan.h"
#include "util.h"

using namespace ouster;

// Same as ouster::cartesian but uses single float precision
PointsF cartesian_f(const Eigen::Ref<const img_t<uint32_t>>& range,
                    const PointsF& direction, const PointsF& offset) {
    if (range.cols() * range.rows() != direction.rows())
        throw std::invalid_argument("unexpected image dimensions");
    auto reshaped = Eigen::Map<const Eigen::Array<uint32_t, -1, 1>>(
        range.data(), range.cols() * range.rows());
    auto nooffset = direction.colwise() * reshaped.cast<float>();
    return (nooffset.array() == 0.0).select(nooffset, nooffset + offset);
}

class CartesianParametrisedTestFixture
    : public ::testing::TestWithParam<std::pair<int, int>> {
   protected:
    int scan_width;
    int scan_height;
};

INSTANTIATE_TEST_CASE_P(CartesianParametrisedTests,
                        CartesianParametrisedTestFixture,
                        ::testing::Values(std::pair<int, int>{512, 128},
                                          std::pair<int, int>{1024, 128},
                                          std::pair<int, int>{2048, 128},
                                          std::pair<int, int>{4096, 128}));

TEST(CartesianParametrisedTestFixture, CartesianFunctionsMatch) {
    const auto WIDTH = 512;
    const auto HEIGHT = 64;
    const auto ROWS = WIDTH * HEIGHT;
    const auto COLS = 3;

    PointsD direction =
        0.5 * PointsD::Random(ROWS, COLS) + PointsD::Constant(ROWS, COLS, 1.0);
    PointsD offset = 0.005 * (PointsD::Random(ROWS, COLS) +
                              PointsD::Constant(ROWS, COLS, 1.0));
    XYZLut lut{direction, offset};

    img_t<uint32_t> range = img_t<uint32_t>::Random(WIDTH, HEIGHT);

    auto points0 = cartesian(range, lut);

    PointsD points = PointsD::Zero(ROWS, COLS);

    cartesianT(points, range, direction, offset);
    EXPECT_TRUE(points.isApprox(points0));
}

TEST(CartesianParametrisedTestFixture, CartesianFunctionsMatchF) {
    const auto WIDTH = 512;
    const auto HEIGHT = 64;
    const auto ROWS = WIDTH * HEIGHT;
    const auto COLS = 3;

    PointsD direction =
        0.5 * PointsD::Random(ROWS, COLS) + PointsD::Constant(ROWS, COLS, 1.0);
    PointsD offset = 0.005 * (PointsD::Random(ROWS, COLS) +
                              PointsD::Constant(ROWS, COLS, 1.0));
    XYZLut lut{direction, offset};

    PointsF directionF = direction.cast<float>();
    PointsF offsetF = offset.cast<float>();

    img_t<uint32_t> range = img_t<uint32_t>::Random(WIDTH, HEIGHT);

    auto points0 = cartesian(range, lut);
    auto points0F = points0.cast<float>();

    PointsF pointsF = PointsF::Zero(ROWS, COLS);

    cartesianT(pointsF, range, directionF, offsetF);
    EXPECT_TRUE(pointsF.isApprox(points0F));
}

TEST_P(CartesianParametrisedTestFixture, SpeedCheck) {
    std::map<std::string, std::string> styles = term_styles();

    const auto test_params = GetParam();
    const auto WIDTH = test_params.first;
    const auto HEIGHT = test_params.second;
    const auto ROWS = WIDTH * HEIGHT;
    const auto COLS = 3;
    std::cout << styles["yellow"] << styles["bold"]
              << "CHECKING PERFORMANCE FOR LIDAR MODE: [" << WIDTH << "x"
              << HEIGHT << "]" << styles["reset"] << std::endl;

    PointsD direction =
        0.5 * PointsD::Random(ROWS, COLS) + PointsD::Constant(ROWS, COLS, 1.0);
    PointsD offset = 0.005 * (PointsD::Random(ROWS, COLS) +
                              PointsD::Constant(ROWS, COLS, 1.0));
    XYZLut lut{direction, offset};

    PointsF directionF = direction.cast<float>();
    PointsF offsetF = offset.cast<float>();

    // create an empty arrays of points
    PointsD points = PointsD(ROWS, COLS);
    PointsF pointsF = PointsF(ROWS, COLS);
    img_t<uint32_t> range = img_t<uint32_t>(WIDTH, HEIGHT);

    constexpr int N_SCANS = 1000;
    constexpr int MOVING_AVG_WINDOW = 100;
    using MovingAverage64 = MovingAverage<int64_t, int64_t, MOVING_AVG_WINDOW>;
    static std::map<std::string, MovingAverage64> mv;

    Timer t;
    std::stringstream ss;
    int output_ctr = 0;

    using CartesianMethod = std::function<void(const img_t<uint32_t>& range)>;
    std::vector<std::pair<std::string, CartesianMethod>> all_cartesians;

    all_cartesians.emplace_back("c0", [&](const img_t<uint32_t>& range) {
        points = cartesian(range, lut);
    });

    all_cartesians.emplace_back("c0f", [&](const img_t<uint32_t>& range) {
        pointsF = cartesian_f(range, directionF, offsetF);
    });

    all_cartesians.emplace_back("cT", [&](const img_t<uint32_t>& range) {
        cartesianT(points, range, direction, offset);
    });

    all_cartesians.emplace_back("cfT", [&](const img_t<uint32_t>& range) {
        cartesianT(pointsF, range, directionF, offsetF);
    });

    std::default_random_engine g;
    std::uniform_real_distribution<double> d(0.0, 1.0);
    std::vector<int> ids(all_cartesians.size());
    std::iota(std::begin(ids), std::end(ids), 0);

    for (auto i = 0; i < N_SCANS; ++i) {
        auto p = std::ceil(10 * float(i) / float(N_SCANS)) / 10;
        auto unary_expr = [&g, &d, p](auto) {
            return d(g) >= p ? 0U : static_cast<uint32_t>(d(g) * 10000);
        };

        auto valid_returns = (range != 0).count();
        auto percentage_valid =
            int(roundf(float(valid_returns) / float(range.size()) * 100));

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
            ss << styles["bold"] << "returns: " << styles["reset"]
               << styles["magenta"] << std::setw(3) << percentage_valid << "%, "
               << styles["reset"];
            ss << styles["bold"] << "c0[time]: " << styles["reset"]
               << styles["cyan"] << std::setw(4) << mv["c0"] << "μs, "
               << styles["reset"];

            auto best_time = std::min_element(
                mv.begin(), mv.end(), [](const auto& a, const auto& b) -> bool {
                    return a.second < b.second;
                });

            for (const auto& x : mv) {
                auto speedup = lround(100.0f * mv["c0"] / x.second);
                auto color_modifier =
                    x.first == best_time->first
                        ? styles["blue"]
                        : (speedup >= 100 ? styles["green"] : styles["red"]);
                ss << styles["bold"] << x.first << ": " << color_modifier
                   << std::setw(4) << speedup << "%, " << styles["reset"];
            }
            std::cout << ss.str() << std::endl;
        }
    }
}

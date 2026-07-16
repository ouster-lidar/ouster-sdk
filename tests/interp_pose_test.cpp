/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

// #include <../thirdparty/sophus/sophus/se3.hpp>
#include <Eigen/Core>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <numeric>
#include <random>
#include <vector>

#include "ouster/core/pose_util.h"
#include "ouster/core/typedefs.h"

using ouster::sdk::core::interp_pose;
using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::Matrix4R;
using ouster::sdk::core::MatrixX16dR;
using ouster::sdk::core::MatrixX16R;
using ouster::sdk::core::Vector16;

// clang-format off
// std::vector<Matrix4dR> pose_interp_se3(const std::vector<double>& x_interp,
//                                        const double t0, const Sophus::SE3d& a,
//                                        const double t1, const Sophus::SE3d& b) {
//     double duration = t1 - t0;
//     if (std::abs(duration) < std::numeric_limits<double>::epsilon()) {
//         throw std::invalid_argument(
//             "Cannot interpolate with zero duration between poses");
//     }
//     std::vector<Matrix4dR> result;
//     result.resize(x_interp.size());
//     // The relative transformation from a to b is (a^-1 * b)
//     Sophus::Vector6d twist = (a.inverse() * b).log();
//     Sophus::SE3d interpolated_pose;
//     for (size_t idx = 0; idx < x_interp.size(); ++idx) {
//         auto t = (x_interp[idx] - t0) / duration;
//         interpolated_pose = a * Sophus::SE3d::exp(t * twist);
//         result[idx] = interpolated_pose.matrix();
//     }
//     return result;
// }
// clang-format on

TEST(InterpPoseTest, PythonParityCheck) {
    double prev_ts = 101000.0;
    double curr_ts = 102000.0;
    double next_ts = 103000.0;

    Matrix4dR prev_pose = Matrix4dR::Identity();

    // clang-format off

    Matrix4dR curr_pose;
    curr_pose << 0.950564, -0.29552, -0.0953745, 5,
                 0.294044, 0.955336, -0.0295028, 3,
                 0.0998334, 0, 0.995004, 2,
                 0.0, 0.0, 0.0, 1.0;

    Matrix4dR next_pose;
    next_pose << 0.879923, -0.389418, -0.272192, 0,
                 0.372026, 0.921061, -0.115081, 0,
                 0.29552, 0, 0.955336, 0,
                 0.0, 0.0, 0.0, 1.0;

    std::vector<double> tss = {prev_ts, curr_ts, next_ts};
    std::vector<Matrix4dR> poses = {prev_pose, curr_pose, next_pose};

    std::vector<double> inquiring_ts = {
        100000.0, 100500.0, 101000.0, 101500.0,
        102000.0, 102500.0, 103000.0, 103500.0
    };

    std::vector<Matrix4dR> expected_poses(8);

    expected_poses[0] <<
        9.50563566e-01, 2.94044472e-01, 9.98336350e-02, -5.83461852e+00,
        -2.95520849e-01, 9.55336290e-01, -2.79214613e-08, -1.38840457e+00,
        -9.53747027e-02, -2.95028941e-02, 9.95004143e-01, -1.42462609e+00,
        0.0, 0.0, 0.0, 1.0;

    expected_poses[1] <<
        0.98756338, 0.149066, 0.04997893, -2.84746798,
        -0.14943741, 0.98876405, 0.00375782, -0.91059756,
        -0.0488572, -0.01117981, 0.9987432, -0.7874577,
        0.0, 0.0, 0.0, 1.0;

    expected_poses[2] <<
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    expected_poses[3] <<
        0.98756338, -0.14943741, -0.0488572, 2.63750479,
        0.149066, 0.98876405, -0.01117981, 1.31602317,
        0.04997893, 0.00375782, 0.9987432, 0.93220328,
        0.0, 0.0, 0.0, 1.0;

    expected_poses[4] <<
        9.50563566e-01, -2.95520849e-01, -9.53747027e-02, 5.00000000e+00,
        2.94044472e-01, 9.55336290e-01, -2.95028941e-02, 3.00000000e+00,
        9.98336350e-02, -2.79214613e-08, 9.95004143e-01, 2.00000000e+00,
        0.0, 0.0, 0.0, 1.0;

    expected_poses[5] <<
        0.9208184, -0.34289665, -0.18578365, 2.41554879,
        0.33559594, 0.93936924, -0.07042739, 1.5451861,
        0.1986689, 0.00250261, 0.98006331, 1.14334982,
        0.0, 0.0, 0.0, 1.0;

    expected_poses[6] <<
        8.79923149e-01, -3.89418384e-01, -2.72192394e-01, -2.23569899e-07,
        3.72025983e-01, 9.21060403e-01, -1.15081184e-01, -1.19328240e-06,
        2.95520591e-01, -1.60280748e-08, 9.55336219e-01, 3.73615875e-06,
        0.0, 0.0, 0.0, 1.0;

    expected_poses[7] <<
        0.82838856, -0.43450466, -0.35352245, -2.21632003,
        0.40287952, 0.90063797, -0.16290697, -1.61678417,
        0.38917989, -0.00747664, 0.9211313, -1.41550338,
        0.0, 0.0, 0.0, 1.0;

    // clang-format on

    std::vector<Matrix4dR> interp_poses = ouster::sdk::core::interp_pose(inquiring_ts, tss, poses);

    ASSERT_EQ(interp_poses.size(), expected_poses.size());
    for (size_t i = 0; i < expected_poses.size(); ++i) {
        EXPECT_TRUE(interp_poses[i].isApprox(expected_poses[i], 1e-4))
            << "Mismatch at pose index " << i << ":\n"
            << "Computed:\n"
            << interp_poses[i] << "\n"
            << "Expected:\n"
            << expected_poses[i] << "\n";
    }
}

TEST(InterpPoseTest, SpeedComparisonForTwoPoseVsGeneral) {
    // Setup data
    const int num_interp = 4096;
    const int num_iterations = 10;
    std::vector<double> x_interp(num_interp);
    std::iota(x_interp.begin(), x_interp.end(), 0.0);

    // Scale x_interp to be within [0, 1]
    for (auto& x : x_interp) x /= (double)num_interp;

    double t0 = 0.0;
    Matrix4dR x0 = Matrix4dR::Identity();
    double t1 = 1.0;
    Matrix4dR x1 = Matrix4dR::Identity();
    // Add some translation/rotation to x1
    x1.block<3, 1>(0, 3) = Eigen::Vector3d(1.0, 2.0, 3.0);
    // Add some rotation to x1
    Eigen::AngleAxisd rot(0.5, Eigen::Vector3d::UnitZ());
    x1.block<3, 3>(0, 0) = rot.toRotationMatrix();

    // Setup for general version
    std::vector<double> x_known = {t0, t1};
    std::vector<Matrix4dR> poses_known = {x0, x1};

    // Measure time for general version
    std::vector<Matrix4dR> res0;
    auto start0 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < num_iterations; ++i) {
        res0 = interp_pose(x_interp, x_known, poses_known);
    }
    auto end0 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff0 = end0 - start0;

    // Measure time for 2-pose version
    std::vector<Matrix4dR> res1;
    auto start1 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < num_iterations; ++i) {
        res1 = interp_pose(x_interp, t0, x0, t1, x1);
    }
    auto end1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff1 = end1 - start1;

    // // Measure time for se3 2-pose version
    // std::vector<Matrix4dR> res2;
    // Sophus::SE3d se3_a(x0);
    // Sophus::SE3d se3_b(x1);
    // auto start2 = std::chrono::high_resolution_clock::now();
    // for (size_t i = 0; i < num_iterations; ++i) {
    //     res2 = pose_interp_se3(x_interp, t0, se3_a, t1, se3_b);
    // }
    // auto end2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> diff2 = end2 - start2;

    std::cout << "General version time: " << diff0.count() << " s\n";
    std::cout << "2-pose version time: " << diff1.count() << " s\n";
    // std::cout << "Sophus 2-pose version time: " << diff2.count() << " s\n";

    // clang-format off
    std::cout << "2-pose version is " << (diff0.count()-diff1.count()) /diff1.count() * 100.0
              << " \% times faster than general version\n";

    // std::cout << "Sophus version is " << (diff0.count()-diff2.count()) / diff2.count() * 100.0
    //           << " \% times faster than general version\n";
    // clang-format on
}

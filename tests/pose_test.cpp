/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/pose.h"

#include <gtest/gtest.h>

#include <cmath>

#include "ouster/core/pose_conversion.h"
#include "ouster/core/typedefs.h"

using ouster::sdk::core::euler_pose_to_matrix;
using ouster::sdk::core::euler_to_rotation_matrix;
using ouster::sdk::core::Matrix3dR;
using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::Pose;
using ouster::sdk::core::quaternion_pose_to_matrix;

namespace {

constexpr double kTol = 1e-12;

void expect_matrix4_near(const Matrix4dR& actual, const Matrix4dR& expected, double tol = kTol) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            EXPECT_NEAR(actual(i, j), expected(i, j), tol)
                << "Mismatch at (" << i << "," << j << ")";
        }
    }
}

}  // namespace

TEST(Pose, DefaultConstructionTest) {
    Pose pose;
    EXPECT_TRUE(pose.position().isApprox(Eigen::Vector3d::Zero(), kTol));
    EXPECT_TRUE(pose.rotation().isApprox(Eigen::Quaterniond::Identity(), kTol));
    const Matrix4dR expected = Matrix4dR::Identity();
    expect_matrix4_near(pose.to_matrix(), expected);
}

TEST(Pose, PositionRotationConstructorTest) {
    const Eigen::Vector3d position(1.0, 2.0, -1.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()));
    Pose pose(position, rotation);
    EXPECT_TRUE(pose.position().isApprox(position, kTol));
    EXPECT_TRUE(pose.rotation().isApprox(rotation, kTol));
}

TEST(Pose, MatrixConstructorRoundTripTest) {
    // Use exact trig values
    const double c = std::cos(M_PI / 6.0);
    const double s = std::sin(M_PI / 6.0);
    Matrix4dR matrix;
    matrix << c, -s, 0.0, 1.0, s, c, 0.0, 2.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0;

    Pose pose(matrix);
    EXPECT_TRUE(pose.position().isApprox(Eigen::Vector3d(1.0, 2.0, -1.0), kTol));
    expect_matrix4_near(pose.to_matrix(), matrix);
}

TEST(Pose, ToMatrixQuaternionTest) {
    Eigen::Matrix<double, 7, 1> pose_vec;
    pose_vec << 0.8446, 0.1913, 0.4619, 0.1913, 1, 2, 3;

    Eigen::Quaterniond rotation(pose_vec(0), pose_vec(1), pose_vec(2), pose_vec(3));
    Pose pose(pose_vec.tail<3>(), rotation);

    expect_matrix4_near(pose.to_matrix(), quaternion_pose_to_matrix(pose_vec));
}

TEST(Pose, ToMatrixEulerTest) {
    Eigen::Matrix<double, 6, 1> pose_vec;
    pose_vec << 0.1, 0.2, 0.3, 1, 2, 3;

    Pose pose;
    pose.set_position(pose_vec.tail<3>());
    const Eigen::Vector3d euler = pose_vec.head<3>();
    pose.set_rotation(euler);

    expect_matrix4_near(pose.to_matrix(), euler_pose_to_matrix(pose_vec));
}

TEST(Pose, EulerAnglesRoundTripTest) {
    Eigen::Vector3d euler(0.1, 0.2, 0.3);

    Pose pose;
    pose.set_rotation(euler);

    EXPECT_TRUE(pose.euler_angles().isApprox(euler, kTol));
}

TEST(Pose, SetRotationMatrixTest) {
    Matrix3dR rot = euler_to_rotation_matrix(0.2, -0.1, 0.4);

    Pose pose;
    pose.set_rotation(rot);

    Matrix4dR expected = Matrix4dR::Identity();
    expected.block<3, 3>(0, 0) = rot;
    expect_matrix4_near(pose.to_matrix(), expected);
}

TEST(Pose, SetRotationEulerTest) {
    Eigen::Vector3d euler(0.15, -0.25, 0.35);
    Eigen::Matrix<double, 6, 1> pose_vec;
    pose_vec << euler(0), euler(1), euler(2), 0, 0, 0;

    Pose pose;
    pose.set_rotation(euler);

    Matrix4dR expected_rot = Matrix4dR::Identity();
    expected_rot.block<3, 3>(0, 0) = euler_to_rotation_matrix(euler(0), euler(1), euler(2));
    expect_matrix4_near(pose.to_matrix(), euler_pose_to_matrix(pose_vec));
}

TEST(Pose, InverseTest) {
    Pose pose;
    pose.set_position(Eigen::Vector3d(1.0, 2.0, 3.0));
    pose.set_rotation(Eigen::Vector3d(0.1, 0.2, 0.3));

    const Pose inv = pose.inverse();
    expect_matrix4_near(inv.to_matrix(), pose.to_matrix().inverse());

    const Pose identity;
    expect_matrix4_near((pose * inv).to_matrix(), identity.to_matrix());
    expect_matrix4_near((inv * pose).to_matrix(), identity.to_matrix());
}

TEST(Pose, MultiplicationTest) {
    Pose a;
    a.set_position(Eigen::Vector3d(1.0, 2.0, 3.0));
    a.set_rotation(Eigen::Vector3d(0.1, 0.2, 0.3));

    Pose b;
    b.set_position(Eigen::Vector3d(4.0, 5.0, 6.0));
    b.set_rotation(Eigen::Vector3d(-0.2, 0.1, 0.4));

    const Pose composed = a * b;
    const Matrix4dR expected = a.to_matrix() * b.to_matrix();
    expect_matrix4_near(composed.to_matrix(), expected);

    // identity multiplication
    const Pose identity;
    expect_matrix4_near((a * identity).to_matrix(), a.to_matrix());
    expect_matrix4_near((identity * a).to_matrix(), a.to_matrix());

    // associativity
    Pose c;
    c.set_position(Eigen::Vector3d(3.0, 4.0, -1.0));
    c.set_rotation(Eigen::Vector3d(-0.05, 0.4, 0.1));

    const Pose left = (a * b) * c;
    const Pose right = a * (b * c);
    expect_matrix4_near(left.to_matrix(), right.to_matrix());
}

TEST(Pose, EqualityTest) {
    Pose a;
    a.set_position(Eigen::Vector3d(1.0, 2.0, 3.0));
    a.set_rotation(Eigen::Vector3d(0.1, 0.2, 0.3));

    Pose b = a;
    EXPECT_TRUE(a == b);

    Pose c;
    c.set_position(Eigen::Vector3d(1.0, 2.0, 3.0));
    c.set_rotation(Eigen::Vector3d(0.1, 0.2, 0.31));
    EXPECT_FALSE(a == c);

    // quaternion sign does not affect equality
    Eigen::Quaterniond neg = a.rotation();
    neg.coeffs() *= -1;
    Pose d(a.position(), neg);
    EXPECT_TRUE(a == d);
}

TEST(Pose, TransformPointTest) {
    Pose a;
    a.set_position(Eigen::Vector3d(1.0, 2.0, 3.0));
    a.set_rotation(Eigen::Vector3d(0.1, 0.2, 0.3));

    const Eigen::Vector3d point(0.5, -1.0, 2.0);
    const Matrix4dR matrix = a.to_matrix();
    const Eigen::Vector3d expected = matrix.block<3, 3>(0, 0) * point + matrix.block<3, 1>(0, 3);

    EXPECT_TRUE((a * point).isApprox(expected, kTol));

    // identity
    const Pose identity;
    EXPECT_TRUE((identity * point).isApprox(point, kTol));

    // associativity
    Pose b;
    b.set_position(Eigen::Vector3d(4.0, 5.0, 6.0));
    b.set_rotation(Eigen::Vector3d(-0.2, 0.1, 0.4));

    EXPECT_TRUE(((a * b) * point).isApprox(a * (b * point), kTol));
}

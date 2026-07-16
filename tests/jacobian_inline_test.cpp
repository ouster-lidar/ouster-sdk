#include <gtest/gtest.h>

#include "ouster/mapping/icp_registration.h"

namespace {

using ouster::sdk::mapping::build_linear_system;
using ouster::sdk::mapping::Correspondences;
using ouster::sdk::mapping::LinearSystem;
using ouster::sdk::mapping::Matrix6d;
using ouster::sdk::mapping::Vector6d;

void ExpectLowerTriangleNear(const Matrix6d& expected, const Matrix6d& actual, double tol) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j <= i; ++j) {
            EXPECT_NEAR(expected(i, j), actual(i, j), tol)
                << "JTJ mismatch at (" << i << ", " << j << ")";
        }
    }
}

}  // namespace

// Golden values computed independently from the dense-Jacobian formulation:
//
//   J_r = [I₃ | -hat(s)]       (3×6 point-to-point Jacobian)
//   w   = k² / (k + |r|²)²     (Geman-McClure weight)
//   JᵀJ = Σ w · Jᵀ J           (6×6, only lower triangle stored)
//   JTr = Σ w · Jᵀ r
//
// Values were produced with NumPy using the dense J_r matrix multiply
// and verified against the closed-form block structure:
//
//   JᵀJ = [ w·I₃           |  -w·hat(s)           ]
//          [ w·hat(s)        |  w·(|s|²·I₃ − s·sᵀ) ]
//
//   JTr = w · [ r ;  s × r ]

TEST(JacobianInlineTest, SingleCorrespondence) {
    // s=(1,2,3), t=(0.5,1.5,2.5) → r=(0.5,0.5,0.5), |r|²=0.75
    // kernel_scale=0.5 → w = 0.25 / (0.5+0.75)² = 0.16
    Correspondences corrs;
    corrs.push_back(std::make_pair(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.5, 1.5, 2.5)));

    LinearSystem result = build_linear_system(corrs, 0.5);

    // w=0.16, all values are w times the block-structure entries above
    Matrix6d expected_JTJ = Matrix6d::Zero();
    expected_JTJ(0, 0) = 0.16;
    expected_JTJ(1, 1) = 0.16;
    expected_JTJ(2, 2) = 0.16;
    expected_JTJ(3, 1) = -0.48;
    expected_JTJ(3, 2) = 0.32;
    expected_JTJ(3, 3) = 2.08;
    expected_JTJ(4, 0) = 0.48;
    expected_JTJ(4, 2) = -0.16;
    expected_JTJ(4, 3) = -0.32;
    expected_JTJ(4, 4) = 1.60;
    expected_JTJ(5, 0) = -0.32;
    expected_JTJ(5, 1) = 0.16;
    expected_JTJ(5, 3) = -0.48;
    expected_JTJ(5, 4) = -0.96;
    expected_JTJ(5, 5) = 0.80;

    Vector6d expected_JTr;
    expected_JTr << 0.08, 0.08, 0.08, -0.08, 0.16, -0.08;

    ExpectLowerTriangleNear(expected_JTJ, result.first, 1e-14);
    EXPECT_TRUE(expected_JTr.isApprox(result.second, 1e-14));
}

TEST(JacobianInlineTest, ZeroResidual) {
    // s=t=(3,4,5) → r=0, |r|²=0
    // kernel_scale=2.0 → w = 4 / (2+0)² = 1.0
    // JTr must be zero since r=0.  JTJ still has structure from s.
    Correspondences corrs;
    corrs.push_back(std::make_pair(Eigen::Vector3d(3.0, 4.0, 5.0), Eigen::Vector3d(3.0, 4.0, 5.0)));

    LinearSystem result = build_linear_system(corrs, 2.0);

    EXPECT_TRUE(result.second.isZero(1e-14));

    // w=1.0 so values are directly from the block structure, e.g.
    // (3,3) = |s|²-sx² = 16+25 = 41, (4,3) = -sx*sy = -12, etc.
    Matrix6d expected_JTJ = Matrix6d::Zero();
    expected_JTJ(0, 0) = 1.0;
    expected_JTJ(1, 1) = 1.0;
    expected_JTJ(2, 2) = 1.0;
    expected_JTJ(3, 1) = -5.0;
    expected_JTJ(3, 2) = 4.0;
    expected_JTJ(3, 3) = 41.0;
    expected_JTJ(4, 0) = 5.0;
    expected_JTJ(4, 2) = -3.0;
    expected_JTJ(4, 3) = -12.0;
    expected_JTJ(4, 4) = 34.0;
    expected_JTJ(5, 0) = -4.0;
    expected_JTJ(5, 1) = 3.0;
    expected_JTJ(5, 3) = -15.0;
    expected_JTJ(5, 4) = -20.0;
    expected_JTJ(5, 5) = 25.0;

    ExpectLowerTriangleNear(expected_JTJ, result.first, 1e-14);
}

TEST(JacobianInlineTest, TwoCorrespondences) {
    // Tests that contributions from multiple correspondences sum correctly.
    // Corr 1: s=(1,0,0) t=(0,0,0) → |r|²=1, w₁ = 1/(1+1)² = 0.25
    // Corr 2: s=(0,1,0) t=(0,0.5,0.5) → |r|²=0.5, w₂ = 1/(1+0.5)² = 4/9
    // Expected values are the element-wise sum of both contributions.
    Correspondences corrs;
    corrs.push_back(std::make_pair(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)));
    corrs.push_back(std::make_pair(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.5, 0.5)));

    LinearSystem result = build_linear_system(corrs, 1.0);

    // a = w₁ + w₂ = 1/4 + 4/9 = 25/36,  b = w₂ = 4/9
    const double a = 25.0 / 36.0;
    const double b = 4.0 / 9.0;

    Matrix6d expected_JTJ = Matrix6d::Zero();
    expected_JTJ(0, 0) = a;
    expected_JTJ(1, 1) = a;
    expected_JTJ(2, 2) = a;
    expected_JTJ(3, 2) = b;
    expected_JTJ(3, 3) = b;
    expected_JTJ(4, 2) = -0.25;
    expected_JTJ(4, 4) = 0.25;
    expected_JTJ(5, 0) = -b;
    expected_JTJ(5, 1) = 0.25;
    expected_JTJ(5, 5) = a;

    Vector6d expected_JTr;
    expected_JTr << 0.25, 2.0 / 9.0, -2.0 / 9.0, -2.0 / 9.0, 0.0, 0.0;

    ExpectLowerTriangleNear(expected_JTJ, result.first, 1e-14);
    EXPECT_TRUE(expected_JTr.isApprox(result.second, 1e-14));
}

TEST(JacobianInlineTest, LDLTSolveResidual) {
    // 6 correspondences with small residuals (typical of ICP after rough
    // alignment), giving a well-conditioned 6×6 system (condition ~8.4).
    // Verifies that LDLT on the lower-triangle-only JTJ produces a dx
    // satisfying the normal equations JTJ·dx = -JTr.
    Correspondences corrs;
    corrs.push_back(std::make_pair(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.8, 1.9, 3.1)));
    corrs.push_back(
        std::make_pair(Eigen::Vector3d(-2.0, 1.0, 0.5), Eigen::Vector3d(-1.8, 0.8, 0.6)));
    corrs.push_back(
        std::make_pair(Eigen::Vector3d(0.5, -1.5, 2.0), Eigen::Vector3d(0.3, -1.3, 2.2)));
    corrs.push_back(
        std::make_pair(Eigen::Vector3d(3.0, 0.0, -1.0), Eigen::Vector3d(3.1, -0.1, -0.8)));
    corrs.push_back(
        std::make_pair(Eigen::Vector3d(-1.0, -2.0, -3.0), Eigen::Vector3d(-0.9, -2.1, -2.8)));
    corrs.push_back(std::make_pair(Eigen::Vector3d(0.0, 0.5, 1.5), Eigen::Vector3d(0.1, 0.4, 1.6)));

    LinearSystem sys = build_linear_system(corrs, 1.0);

    Vector6d dx = sys.first.ldlt().solve(-sys.second);

    // Symmetrize and check that JTJ·dx = -JTr holds.
    Matrix6d JTJ_sym = sys.first.selfadjointView<Eigen::Lower>();
    Vector6d residual = JTJ_sym * dx + sys.second;
    EXPECT_NEAR(residual.norm(), 0.0, 1e-12) << "Residual: " << residual.transpose();
}

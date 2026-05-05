#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
using Vector7d = Matrix<double, 7, 1>;
using MatrixX3d = Matrix<double, Dynamic, 3>;
using Matrix6d = Matrix<double, 6, 6>;
using ArrayX3d = Array<double, Dynamic, 3>;
}  // namespace Eigen

namespace ouster {
const double EPS = std::numeric_limits<double>::epsilon();
const double NUMERIC_EPS = std::sqrt(std::numeric_limits<double>::epsilon());

using RotQ = Eigen::Quaterniond;
using TransH = Eigen::Vector3d;
using TransV = Eigen::Vector3d;
using AdjointH = Eigen::Matrix6d;
using AdjointV = Eigen::Matrix6d;
}  // namespace ouster

#pragma once

#include <Eigen/Dense>

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, Dynamic, 3> MatrixX3d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Array<double, Dynamic, 3> ArrayX3d;
}  // namespace Eigen

namespace ouster {
const double EPS = std::numeric_limits<double>::epsilon();
const double NUMERIC_EPS = std::sqrt(std::numeric_limits<double>::epsilon());

typedef Eigen::Quaterniond RotQ;
typedef Eigen::Vector3d TransH;
typedef Eigen::Vector3d TransV;
typedef Eigen::Matrix6d AdjointH;
typedef Eigen::Matrix6d AdjointV;
}  // namespace ouster

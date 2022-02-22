#pragma once

#include <Eigen/Core>

namespace ouster {
namespace viz {
namespace impl {

struct CameraData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double aspect;
    Eigen::Matrix4d proj;
    Eigen::Matrix4d view;
    Eigen::Matrix4d target;
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster

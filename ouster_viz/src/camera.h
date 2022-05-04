/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Core>

#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

inline double window_aspect(const WindowCtx& ctx) {
    return ctx.viewport_width / static_cast<double>(ctx.viewport_height);
}

struct CameraData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4d proj;
    Eigen::Matrix4d view;
    Eigen::Matrix4d target;
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster

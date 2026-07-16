/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Core>

#include "ouster/core/typedefs.h"
#include "ouster/viz/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {
using core::Matrix4fR;

inline double window_aspect(const WindowCtx& ctx) {
    return ctx.viewport_width / static_cast<double>(ctx.viewport_height);
}

struct CameraData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    core::Matrix4dR proj;
    core::Matrix4dR view;
    core::Matrix4dR target;
};

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

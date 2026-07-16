/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include "ouster/core/eigen.h"  // NOLINT(unused-includes,misc-include-cleaner)

namespace ouster {
namespace sdk {
namespace core {

/// A 3D coordinate represented as an Eigen vector of three floats.
using Coord = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;

}  // namespace core
}  // namespace sdk
}  // namespace ouster

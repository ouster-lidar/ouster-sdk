/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Internal ground-segmentation helpers shared between ground_seg.cpp
 *        and align_clouds.cpp.  Not part of the public API.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
class LidarFrame;
template <typename T>
class XYZLutT;
using XYZLut = XYZLutT<double>;
}  // namespace core
}  // namespace sdk
}  // namespace ouster

namespace ouster {
namespace sdk {
namespace algorithm {
namespace impl {

// Internal helper shared with point-cloud alignment. Not part of the supported
// public API.
std::vector<std::vector<uint8_t>> get_ground_mask(const core::LidarFrame& frame, double grid_size,
                                                  const core::XYZLut& xyz_lut);

}  // namespace impl
}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster

/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include "ouster/coord.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief A unit vector respresenting the direction of a lidar beam.
 */
struct OUSTER_API_CLASS Ray {
    Coord offset;     ///< The origin of the beam
    Coord direction;  ///< The direction of the beam
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster

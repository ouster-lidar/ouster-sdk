#pragma once
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief Represents the nearest and farthest intersections with a mesh in
 * meters.
 */
using BoundsF = std::pair<float, float>;

/**
 * @brief Represents a single pixel in a ZM range image.
 */
struct OUSTER_API_CLASS BoundsNearFar {
    uint16_t
        near_range_unscaled;  ///< The unscaled near range value for this pixel.
    uint16_t
        far_range_unscaled;  ///< The unscaled far range value for this pixel.
};

/**
 * @brief Represents a single pixel in a ZM range image.
 */
union Bounds {
    uint32_t bits;     ///< All 32-bits, near and far range values.
    BoundsNearFar nf;  ///< Near and far range values represented as a struct.
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster

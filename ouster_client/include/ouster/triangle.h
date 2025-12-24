/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <array>

#include "ouster/coord.h"
#include "ouster/ray.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief Represents a triangle.
 */
struct OUSTER_API_CLASS Triangle {
    // TODO[tws] encapsulate and enforce invariants.
    std::array<Coord, 3>
        coords;  ///< Coords representing the corners of the Triangle.
    std::array<Coord, 3>
        edges;     ///< Coords representing the edges of the Triangle.
    Coord normal;  ///< A Coord representing the normal of the Triangle;

   public:
    /**
     * Constructs a Triangle from three Coords.
     * @param[in] coord_a Coord A.
     * @param[in] coord_b Coord B.
     * @param[in] coord_c Coord C.
     */
    OUSTER_API_FUNCTION
    Triangle(const Coord& coord_a, const Coord& coord_b, const Coord& coord_c);

    /**
     * Returns true if the provided Coord is inside this Triangle.
     * @param[in] coord a Coord to test.
     * @return true if the Coord is inside this Triangle.
     */
    OUSTER_API_FUNCTION
    bool inside(const Coord& coord) const;

    /**
     * Computes the intersection point of a Ray with this Triangle.
     * @param[in] beam the Ray.
     * point.
     * @return a positive float point distance or
     * std::numeric_limits<float>::lowest if there was no intersection.
     */
    OUSTER_API_FUNCTION
    float intersect(const Ray& beam) const;

    /**
     * Compares two Triangles for equality.
     * @param[in] rhs the Triangle to compare against.
     * @return true if the Triangles are equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator==(const Triangle& rhs) const {
        // TODO[tws] comparing edges and normal would be unnecesary if they're
        // derived
        return coords == rhs.coords && edges == rhs.edges &&
               normal == rhs.normal;
    }

    /**
     * Compares two Triangles for inequality.
     * @param[in] rhs the Triangle to compare against.
     * @return true if the Triangles are not equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const Triangle& rhs) const { return !(*this == rhs); }
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster

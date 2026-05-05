/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <istream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ouster/coord.h"
#include "ouster/ray.h"
#include "ouster/triangle.h"
#include "ouster/visibility.h"
#include "ouster/zone_lut.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * @brief A bounding sphere represented as a center coordinate and a radius.
 */
using BoundingSphere = std::pair<Coord, float>;

/**
 * @brief A collection of Triangles.
 */
class OUSTER_API_CLASS Mesh {
   public:
    /// Constructs an Mesh containing no Triangles.
    OUSTER_API_FUNCTION
    Mesh() = default;

    /**
     * Constructs a Mesh from a vector of Triangles.
     * @param[in] tris A vector of Triangles.
     */
    OUSTER_API_FUNCTION
    explicit Mesh(const std::vector<Triangle>& tris);

    /**
     * Constructs a Mesh from a vector of Triangles.
     * @param[in] tris A vector of Triangles.
     */
    OUSTER_API_FUNCTION
    explicit Mesh(std::vector<Triangle>&& tris);

    /**
     * Populates this Mesh with Triangles parsed from the contents of an STL
     * file.
     * @param[in] path The path to the STL file.
     * @return true if parsing the file succeeded.
     */
    OUSTER_API_FUNCTION
    bool load_from_stl(const std::string& path);

    /**
     * Populates this Mesh with Triangles parsed from the contents of an STL
     * file.
     * @param[in] bytes A vector of bytes, which should be the contents of an
     * STL file.
     * @return true if parsing the bytes succeeded.
     */
    OUSTER_API_FUNCTION
    bool load_from_stl_bytes(const std::vector<uint8_t>& bytes);

    /**
     * Populates this Mesh with Triangles parsed from the provided istream.
     * @param[in] input_stream An istream (perhaps an ifstream) which contains
     * STL ascii-formatted content.
     * @return true if parsing the file succeeded.
     */
    OUSTER_API_FUNCTION
    bool load_from_stl_stream(std::istream& input_stream);

    /**
     * Returns the Triangles.
     * @return A vector of Triangles.
     */
    OUSTER_API_FUNCTION
    const std::vector<Triangle>& triangles() const;

    /**
     * Returns the nearest and farthest distances the given ray intersects with
     * the Mesh.
     * @param[in] beam The Ray to test for intersections.
     * @param[out] z A BoundsF to populate with the near and far intersection
     * distances.
     * @return true if there was at least one intersection, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool closest_and_farthest_intersections(const Ray& beam, BoundsF& z) const;

    /**
     * Returns the distances to all intersections between the Mesh and the given
     * Ray.
     * @param[in] beam The Ray to test for intersections.
     * @return A multiset of floats representing the distances to all
     * intersections.
     */
    OUSTER_API_FUNCTION
    std::multiset<float> intersection_distances(const Ray& beam) const;

    /**
     * Returns true iff the beam intersects with the bounding sphere
     * @param[in] beam The Ray to test for intersection with the bounding
     * sphere
     * @return true if the beam intersects with the bounding sphere, false
     * otherwise
     */
    OUSTER_API_FUNCTION
    bool intersects_with_bounding_sphere(const Ray& beam) const;

    /**
     * Returns true if this Mesh equals another Mesh
     * @param[in] rhs The other Mesh to compare against
     * @return True if the two Meshes are equal, false otherwise
     */
    OUSTER_API_FUNCTION
    bool operator==(const Mesh& rhs) const {
        return triangles_ == rhs.triangles_;
    }

    /**
     * Returns true if this Mesh does not equal another Mesh
     * @param[in] rhs The other Mesh to compare against
     * @return True if the two Meshes are not equal, false otherwise
     */
    OUSTER_API_FUNCTION
    bool operator!=(const Mesh& rhs) const { return !(*this == rhs); }

    /**
     * Returns the bounding sphere of this Mesh.
     * @return The bounding sphere.
     */
    OUSTER_API_FUNCTION
    const BoundingSphere& bounding_sphere() const;

   private:
    bool load_from_stl_ascii(std::istream& stl_file);
    bool load_from_stl_binary(std::istream& input_stream);
    std::vector<Triangle> triangles_{};
    BoundingSphere bounding_sphere_{};
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster

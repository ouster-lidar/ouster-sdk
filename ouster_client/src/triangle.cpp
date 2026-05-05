#include "ouster/triangle.h"

#include <cstdlib>
#include <limits>

#include "ouster/coord.h"
#include "ouster/ray.h"

namespace ouster {
namespace sdk {
namespace core {

Triangle::Triangle(const Coord& coord_a, const Coord& coord_b,
                   const Coord& coord_c)
    : coords{coord_a, coord_b, coord_c},
      edges{coord_b - coord_a, coord_c - coord_b, coord_a - coord_c},
      normal{edges[0].cross(edges[1]).normalized()} {}

bool Triangle::inside(const Coord& coord) const {
    return (normal.dot(edges[0].cross(coord - coords[0])) > -1e-6 &&
            normal.dot(edges[1].cross(coord - coords[1])) > -1e-6 &&
            normal.dot(edges[2].cross(coord - coords[2])) > -1e-6);
}

// NOLINTBEGIN(readability-identifier-length)
float Triangle::intersect(const Ray& beam) const {
    constexpr float epsilon = std::numeric_limits<float>::epsilon();

    Coord edge1 = coords[1] - coords[0];
    Coord edge2 = coords[2] - coords[0];
    Coord ray_cross_e2 = beam.direction.cross(edge2);
    float det = edge1.dot(ray_cross_e2);

    if (det > -epsilon && det < epsilon) {
        return std::numeric_limits<float>::lowest();  // This ray is parallel to
                                                      // this triangle.
    }

    float inv_det = 1.0f / det;
    Coord s = beam.offset - coords[0];
    float u = inv_det * s.dot(ray_cross_e2);

    if ((u < 0 && std::abs(u) > epsilon) ||
        (u > 1 && std::abs(u - 1) > epsilon)) {
        return std::numeric_limits<float>::lowest();
    }

    Coord s_cross_e1 = s.cross(edge1);
    float v = inv_det * beam.direction.dot(s_cross_e1);

    if ((v < 0 && std::abs(v) > epsilon) ||
        (u + v > 1 && std::abs(u + v - 1) > epsilon)) {
        return std::numeric_limits<float>::lowest();
    }

    // At this stage we can compute t to find out where the intersection point
    // is on the line.
    float t = inv_det * edge2.dot(s_cross_e1);
    return t;
}
// NOLINTEND(readability-identifier-length)

}  // namespace core
}  // namespace sdk
}  // namespace ouster

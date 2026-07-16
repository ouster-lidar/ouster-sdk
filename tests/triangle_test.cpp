#include "ouster/core/triangle.h"

#include <gtest/gtest.h>

using namespace ouster::sdk::core;

TEST(Triangle, test_normal) {
    // A counterclockwise triangle in the XY plane.
    Triangle triangle1({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    // Normal points up: triangle normals obey the RH rule.
    EXPECT_EQ(triangle1.normal, Coord(0, 0, 1));

    // A triangle with all corners at the same point.
    Triangle triangle2({-1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0});

    // The normal will be 0, 0, 0.
    EXPECT_EQ(triangle2.normal, Coord(0, 0, 0));
}

TEST(Triangle, test_edges) {
    Triangle triangle1({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    EXPECT_EQ(triangle1.edges[0], Coord(2, 0, 0));    // B - A
    EXPECT_EQ(triangle1.edges[1], Coord(-1, 1, 0));   // C - B
    EXPECT_EQ(triangle1.edges[2], Coord(-1, -1, 0));  // A - C
}

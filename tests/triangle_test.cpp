#include "ouster/triangle.h"

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

TEST(Triangle, test_inside) {
    // A counterclockwise triangle in the XY plane.
    Triangle triangle1({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    EXPECT_TRUE(triangle1.inside({0, 0, 0}));
    EXPECT_FALSE(triangle1.inside({-2, 0, 0}));
    EXPECT_FALSE(triangle1.inside({2, 0, 0}));
    EXPECT_FALSE(triangle1.inside({0, -2, 0}));
    EXPECT_FALSE(triangle1.inside({0, 2, 0}));

    // NOTE, this method assumes the point is already in the same plane as the
    // triangle. Since the following points are not within the same plane, the
    // inside method would not necessarily return false.
    EXPECT_TRUE(triangle1.inside({0, 0, -2}));
    EXPECT_TRUE(triangle1.inside({0, 0, 2}));
    EXPECT_TRUE(triangle1.inside({0, 0, -200}));
    EXPECT_TRUE(triangle1.inside({0, 0, 200}));
}

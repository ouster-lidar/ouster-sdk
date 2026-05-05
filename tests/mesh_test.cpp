#include "ouster/mesh.h"

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

// TODO[tws] combine utils and test_utils
#include "ouster/coord.h"
#include "ouster/ray.h"
#include "test_utils.h"
#include "util.h"

using namespace ouster::sdk::core;

static Mesh test_mesh() {
    std::string data_dir = getenvs("DATA_DIR");
    Mesh mesh;
    auto res = mesh.load_from_stl(data_dir + "/0.stl");
    if (!res) {
        throw std::runtime_error("Couldn't parse STL.");
    }
    return mesh;
}

TEST(Mesh, load_from_stl_binary) {
    Mesh mesh = test_mesh();
    ASSERT_EQ(mesh.triangles().size(), 12);
}

TEST(Mesh, load_from_stl_ascii) {
    std::string data_dir = getenvs("DATA_DIR");
    Mesh mesh;
    ASSERT_TRUE(mesh.load_from_stl(data_dir + "/ascii.stl"));
    ASSERT_EQ(mesh.triangles().size(), 12);
    auto triangle = mesh.triangles()[0];
    Coord expected_normal{0, 0, 1};
    EXPECT_EQ(triangle.normal, expected_normal);
    Coord expected_coord_a{-20, -20, 40};
    Coord expected_coord_b{-20, 20, 40};
    Coord expected_coord_c{20, -20, 40};
    EXPECT_EQ(triangle.coords[0], expected_coord_a);
}

TEST(Mesh, load_from_ascii_invalid) {
    std::string data_dir = getenvs("DATA_DIR");
    Mesh mesh;
    EXPECT_FALSE(
        mesh.load_from_stl(data_dir + "/ascii_invalid_expected_vertex.stl"));
    EXPECT_FALSE(
        mesh.load_from_stl(data_dir + "/ascii_invalid_expected_endloop.stl"));
    EXPECT_FALSE(mesh.load_from_stl(data_dir +
                                    "/ascii_invalid_expected_outer_loop.stl"));
    EXPECT_FALSE(
        mesh.load_from_stl(data_dir + "/ascii_invalid_expected_endfacet.stl"));
    EXPECT_FALSE(mesh.load_from_stl(data_dir + "/ascii_empty.stl"));
    EXPECT_FALSE(
        mesh.load_from_stl(data_dir + "/ascii_invalid_expected_solid.stl"));
    EXPECT_FALSE(
        mesh.load_from_stl(data_dir + "/ascii_invalid_expected_endsolid.stl"));
    EXPECT_FALSE(
        mesh.load_from_stl(data_dir + "/ascii_invalid_unexpected_line.stl"));
}

TEST(Mesh, load_from_stl_bytes) {
    std::string data_dir = getenvs("DATA_DIR");
    std::string test_file = data_dir + "/0.stl";
    std::vector<uint8_t> bytes = get_file_as_bytes(test_file);
    Mesh mesh;
    ASSERT_TRUE(mesh.load_from_stl_bytes(bytes));
    Mesh expected;
    ASSERT_TRUE(expected.load_from_stl(test_file));
    EXPECT_EQ(mesh, expected);
}

TEST(Mesh, closest_and_farthest_intersections) {
    Ray beam;
    beam.offset = Coord{0.00397694f, 0.000619036f, 1.0436f};
    beam.direction = Coord{-0.0914688f, 0.975646f, -0.199368f};
    Mesh mesh = test_mesh();
    // Regression test: I determined the expected values experimentally.
    BoundsF bounds;
    auto res = mesh.closest_and_farthest_intersections(beam, bounds);
    ASSERT_TRUE(res);
    EXPECT_FLOAT_EQ(bounds.first, 2.02771592f);
    EXPECT_FLOAT_EQ(bounds.second, 2.65380812f);
}

TEST(Mesh, intersection_distances) {
    Ray beam;
    beam.offset = Coord{0.00397694f, 0.000619036f, 1.0436f};
    beam.direction = Coord{-0.0914688f, 0.975646f, -0.199368f};
    Mesh mesh = test_mesh();
    // Regression test: I determined the expected values experimentally.
    auto res = mesh.intersection_distances(beam);
    auto expected = std::multiset<float>({2.02771592f, 2.65380812f});
    EXPECT_FLOAT_EQ(*(res.begin()), *(expected.begin()));
    EXPECT_FLOAT_EQ(*(res.rbegin()), *(expected.rbegin()));
}

TEST(Mesh, bounding_sphere) {
    Triangle triangle1{{1.f, 1.f, 1.f}, {1.f, 1.f, 1.f}, {1.f, 1.f, 1.f}};
    Triangle triangle2{{2.f, 2.f, 2.f}, {2.f, 2.f, 2.f}, {2.f, 2.f, 2.f}};
    Triangle triangle3{{2.f, 2.f, 2.f}, {2.f, 2.f, 2.f}, {2.f, 2.f, 2.f}};
    Mesh mesh({triangle1, triangle2, triangle3});
    auto bounding_sphere = mesh.bounding_sphere();
    auto centroid = bounding_sphere.first;
    auto radius = bounding_sphere.second;
    EXPECT_FLOAT_EQ(radius, 1.1547004f);
    EXPECT_FLOAT_EQ(centroid[0], 1.666667f);
    EXPECT_FLOAT_EQ(centroid[1], 1.666667f);
    EXPECT_FLOAT_EQ(centroid[2], 1.666667f);

    Ray beam1({{0.f, 0.f, 0.f}, Coord(1.f, 1.f, 1.f).normalized()});
    EXPECT_TRUE(mesh.intersects_with_bounding_sphere(beam1));
    Ray beam2({{0.f, 0.f, 0.f}, Coord(-1.f, -1.f, -1.f).normalized()});
    EXPECT_FALSE(mesh.intersects_with_bounding_sphere(beam2));
}

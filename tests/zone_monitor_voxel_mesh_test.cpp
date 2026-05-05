#include "ouster/impl/zone_monitor_voxel_mesh.h"

#include <gtest/gtest.h>

#include "ouster/types.h"

namespace ouster {
namespace sdk {
namespace viz {

auto tiny_sensor_info() -> ouster::sdk::core::SensorInfo {
    ouster::sdk::core::SensorInfo sensor;
    sensor.format.pixels_per_column = 8;
    sensor.format.columns_per_frame = 8;
    sensor.format.pixel_shift_by_row =
        std::vector<int>{24, 8, -8, -24, 24, 8, -8, -24};

    sensor.beam_azimuth_angles = {8.4, 2.8,  -2.8,  -8.41,
                                  8.4, 2.82, -2.79, -8.42};

    sensor.beam_altitude_angles = {1.87,  1.17,  0.45,  -0.24,
                                   -0.93, -1.66, -2.35, -3.05};

    sensor.lidar_to_sensor_transform << -1., 0., 0., 0., 0., -1., 0., 0., 0.,
        0., 1., 38.195, 0., 0., 0., 1.;

    sensor.beam_to_lidar_transform << 1., 0., 0., 27.116, 0., 1., 0., 0., 0.,
        0., 1., 0., 0., 0., 0., 1.;
    return sensor;
}

TEST(ZoneMonitorTests, slow_altitude_angle) {
    auto sensor = tiny_sensor_info();
    EXPECT_FLOAT_EQ(lerp_vector(sensor.beam_altitude_angles, -100.0), 1.87);
    EXPECT_FLOAT_EQ(lerp_vector(sensor.beam_altitude_angles, 0.0), 1.87);
    EXPECT_FLOAT_EQ(lerp_vector(sensor.beam_altitude_angles, 0.5), 1.52);
    EXPECT_FLOAT_EQ(lerp_vector(sensor.beam_altitude_angles, 1.0), 1.17);
    EXPECT_FLOAT_EQ(lerp_vector(sensor.beam_altitude_angles, 2.0), 0.45);
    EXPECT_FLOAT_EQ(lerp_vector(sensor.beam_altitude_angles, 7.0), -3.05);
    EXPECT_FLOAT_EQ(lerp_vector(sensor.beam_altitude_angles, 7.5), -3.05);
    EXPECT_FLOAT_EQ(lerp_vector(sensor.beam_altitude_angles, 100.0), -3.05);
}

template <typename T>
void EXPECT_COORD_FLOAT_EQ(const T& a, const T& b) {
    EXPECT_FLOAT_EQ(a.x(), b.x());
    EXPECT_FLOAT_EQ(a.y(), b.y());
    EXPECT_FLOAT_EQ(a.z(), b.z());
}

TEST(ZoneMonitorTests, direction_rot) {
    auto sensor = tiny_sensor_info();
    auto i = 4.0f;
    auto j = 4.0f;
    auto v0_0_0 = direction_rot(sensor, j - 0.5f, i - 0.5f);
    auto v0_1_0 = direction_rot(sensor, j - 0.5f, i + 0.5f);
    auto v0_1_1 = direction_rot(sensor, j + 0.5f, i + 0.5f);
    auto v0_0_1 = direction_rot(sensor, j + 0.5f, i - 0.5f);

    EXPECT_COORD_FLOAT_EQ(
        v0_0_0,
        Eigen::Vector3f(9.23831377e-04, 3.82663486e-04, -1.02099987e-05));
    EXPECT_COORD_FLOAT_EQ(
        v0_1_0,
        Eigen::Vector3f(9.23831377e-04, -3.82663486e-04, -1.02099987e-05));
    EXPECT_COORD_FLOAT_EQ(
        v0_1_1,
        Eigen::Vector3f(9.23643560e-04, -3.82585689e-04, -2.26000895e-05));
    EXPECT_COORD_FLOAT_EQ(
        v0_0_1,
        Eigen::Vector3f(9.23643560e-04, 3.82585689e-04, -2.26000895e-05));
}

TEST(ZoneMonitorTests, offset) {
    auto sensor = tiny_sensor_info();
    auto i = 4.0f;
    auto j = 4.0f;
    auto v0_0_0 = offset(sensor, j - 0.5f, i - 0.5f);
    auto v0_1_0 = offset(sensor, j - 0.5f, i + 0.5f);
    auto v0_1_1 = offset(sensor, j + 0.5f, i + 0.5f);
    auto v0_0_1 = offset(sensor, j + 0.5f, i - 0.5f);

    EXPECT_COORD_FLOAT_EQ(
        v0_0_0,
        Eigen::Vector3f(-1.305072942e-06, -5.408008406e-07, 0.03847185522));
    EXPECT_COORD_FLOAT_EQ(
        v0_1_0,
        Eigen::Vector3f(-1.305072942e-06, 5.408008406e-07, 0.03847185522));
    EXPECT_COORD_FLOAT_EQ(
        v0_1_1,
        Eigen::Vector3f(-6.397843663e-06, 2.650801662e-06, 0.03880782425));
    EXPECT_COORD_FLOAT_EQ(
        v0_0_1,
        Eigen::Vector3f(-6.397843663e-06, -2.650801662e-06, 0.03880782425));
}

TEST(ZoneMonitorTests, voxel_style_mesh_components_from_range_images_with_lut) {
    auto sensor_info = tiny_sensor_info();
    ouster::sdk::core::img_t<uint32_t> near(
        sensor_info.format.pixels_per_column,
        sensor_info.format.columns_per_frame);
    ouster::sdk::core::img_t<uint32_t> far(
        sensor_info.format.pixels_per_column,
        sensor_info.format.columns_per_frame);
    near.setZero();
    far.setZero();
    for (int i = 4; i < 5; i++) {
        for (uint32_t j = 0; j < sensor_info.format.columns_per_frame; j++) {
            near(i, j) = 1000;
            far(i, j) = 1100;
        }
    }

    auto vertex_lookup = precompute_voxel_vertices(sensor_info);
    auto res = voxel_style_mesh_components_from_range_images_with_lut(
        near, far, sensor_info, vertex_lookup, false, true);

    auto& vertices = std::get<0>(res);
    auto& faces = std::get<1>(res);
    auto& edges = std::get<2>(res);
    ASSERT_EQ(vertices.size(), 144);
    ASSERT_EQ(faces.size(), 0);
    ASSERT_EQ(edges.size(), 164);

    EXPECT_COORD_FLOAT_EQ(vertices[0].position,
                          Eigen::Vector3f(-0.92383009, 0.38266295, 0.02826186));
    EXPECT_COORD_FLOAT_EQ(vertices[1].position,
                          Eigen::Vector3f(-0.38266295, 0.92383009, 0.02826186));
    EXPECT_COORD_FLOAT_EQ(vertices[2].position,
                          Eigen::Vector3f(-0.38258305, 0.92363715, 0.01620773));
    EXPECT_COORD_FLOAT_EQ(vertices[3].position,
                          Eigen::Vector3f(-0.92363715, 0.38258305, 0.01620773));

    EXPECT_COORD_FLOAT_EQ(
        vertices[0].normal,
        Eigen::Vector3f(0.70701363, -0.70701363, 0.016230849));
    EXPECT_FLOAT_EQ(vertices[0].normal.norm(), 1.0f);
    EXPECT_COORD_FLOAT_EQ(
        vertices[1].normal,
        Eigen::Vector3f(0.70701363, -0.70701363, 0.016230849));
    EXPECT_FLOAT_EQ(vertices[1].normal.norm(), 1.0f);
    EXPECT_COORD_FLOAT_EQ(
        vertices[2].normal,
        Eigen::Vector3f(0.70701363, -0.70701363, 0.016230849));
    EXPECT_FLOAT_EQ(vertices[2].normal.norm(), 1.0f);
    EXPECT_COORD_FLOAT_EQ(
        vertices[3].normal,
        Eigen::Vector3f(0.70701363, -0.70701363, 0.016230849));
    EXPECT_FLOAT_EQ(vertices[3].normal.norm(), 1.0f);

    std::vector<uint32_t> expected_edges{0, 1, 1, 2, 2, 3, 3, 0};
    EXPECT_EQ(std::vector<uint32_t>(edges.begin(), edges.begin() + 8),
              expected_edges);
}

}  // namespace viz
}  // namespace sdk
}  // namespace ouster

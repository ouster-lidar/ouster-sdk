#include <gtest/gtest.h>
#include <gtest/internal/gtest-port.h>

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <ios>
#include <iostream>
#include <jsoncons/json.hpp>
#include <jsoncons/json_type.hpp>
#include <jsoncons_ext/jsonpath/json_query.hpp>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/zone.h"
#include "test_utils.h"
#include "util.h"
#include "zone_header.h"

using namespace jsoncons;
using namespace ouster::sdk::core;
using ouster::sdk::core::mat4d;
using ouster::sdk::core::mat4d_from_array;

static BeamConfig test_beam_config(std::string data_dir) {
    std::string sensor_info_path = data_dir + "/785.json";
    std::ifstream sensor_info_file(sensor_info_path);
    auto sensor_info_json = json::parse(sensor_info_file);
    const json& beam_intrinsics = sensor_info_json["beam_intrinsics"];
    const json& lidar_intrinsics = sensor_info_json["lidar_intrinsics"];
    std::vector<double> beam_altitude_angles =
        beam_intrinsics["beam_altitude_angles"].as<std::vector<double>>();
    std::vector<double> beam_azimuth_angles =
        beam_intrinsics["beam_azimuth_angles"].as<std::vector<double>>();
    uint32_t columns_per_frame =
        sensor_info_json["lidar_data_format"]["columns_per_frame"]
            .as<uint32_t>();
    mat4d beam_to_lidar_transform = mat4d_from_array(
        beam_intrinsics["beam_to_lidar_transform"]
            .as<std::array<double, mat4d::RowsAtCompileTime *
                                       mat4d::ColsAtCompileTime>>());
    mat4d lidar_to_sensor_transform = mat4d_from_array(
        lidar_intrinsics["lidar_to_sensor_transform"]
            .as<std::array<double, mat4d::RowsAtCompileTime *
                                       mat4d::ColsAtCompileTime>>());
    uint64_t serial_number =
        sensor_info_json["sensor_info"]["prod_sn"].as<uint64_t>();

    mat4d sensor_to_body_transform = mat4d::Identity();

    // Add a translation to simulate a nontrivial zm metadata.json extrinsics
    sensor_to_body_transform(2, 3) = 1.0;

    BeamConfig beam_config(columns_per_frame, beam_altitude_angles,
                           beam_azimuth_angles, beam_to_lidar_transform,
                           lidar_to_sensor_transform, sensor_to_body_transform,
                           DEFAULT_M_PER_ZMBIN, serial_number);
    return beam_config;
}

TEST(Zone, render) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.point_count = 1000;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    auto stl = Stl(data_dir + "/0.stl");
    stl.coordinate_frame = Stl::CoordinateFrame::BODY;
    zone.stl = stl;
    EXPECT_TRUE(zone.render(beam_config));
    auto& zrb = zone.zrb.value();

    auto test_zrb = [&beam_config, &zone](Zrb& zrb) {
        ASSERT_EQ(zrb.near_range_mm.cols(), beam_config.n_cols);
        ASSERT_EQ(zrb.near_range_mm.rows(), beam_config.n_rows);
        ASSERT_EQ(zrb.far_range_mm.cols(), beam_config.n_cols);
        ASSERT_EQ(zrb.far_range_mm.rows(), beam_config.n_rows);
        ASSERT_TRUE(zrb.stl_hash);
        ASSERT_EQ(*zrb.stl_hash, zone.stl->hash());
        ASSERT_EQ(zrb.serial_number, beam_config.serial_number);

        // The beam didn't intersect
        EXPECT_EQ(zrb.near_range_mm(50, 100), 0);
        EXPECT_EQ(zrb.far_range_mm(50, 100), 0);

        // But these did
        constexpr int max_error = 4;  // mm
        EXPECT_LE(std::abs(static_cast<int>(zrb.near_range_mm(59, 180)) - 2253),
                  max_error);
        EXPECT_LE(std::abs(static_cast<int>(zrb.far_range_mm(59, 180)) - 3027),
                  max_error);
        EXPECT_LE(std::abs(static_cast<int>(zrb.near_range_mm(71, 274)) - 2285),
                  max_error);
        EXPECT_LE(std::abs(static_cast<int>(zrb.far_range_mm(71, 274)) - 2375),
                  max_error);

        // IMPORTANT: encoding a zrb from mm is lossy
    };

    test_zrb(zrb);

    // Write out the ZRB
    auto blob = zrb.blob();

    // The valid column mask is not set until save/blob is called
    EXPECT_FALSE(zrb.valid_col_mask.any());

    // Read it back in
    Zrb zrb2(blob);

    // When saving, we wrote the valid column mask - so it should be set now
    EXPECT_TRUE(zrb2.valid_col_mask.any());

    test_zrb(zrb2);
}

TEST(Zone, render_no_stl_or_zrb) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.point_count = 1000;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    EXPECT_THROW(
        {
            try {
                zone.render(beam_config);
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(e.what(), "Zone: must have either STL or ZRB");
                throw;
            }
        },
        std::logic_error);
}

TEST(Zone, point_count_not_set) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.stl = Stl(data_dir + "/0.stl");
    zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    EXPECT_THROW(
        {
            try {
                zone.render(beam_config);
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(e.what(),
                             "Zone: point_count must be in [1, 262143]");
                throw;
            }
        },
        std::logic_error);
}

TEST(Zone, frame_count_not_set) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.stl = Stl(data_dir + "/0.stl");
    zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    zone.point_count = 1000;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    EXPECT_THROW(
        {
            try {
                zone.render(beam_config);
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(e.what(),
                             "Zone: frame_count must be in [1, 65535]");
                throw;
            }
        },
        std::logic_error);
}

TEST(Zone, mode_not_set) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.stl = Stl(data_dir + "/0.stl");
    zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    zone.point_count = 1000;
    zone.frame_count = 10;
    EXPECT_THROW(
        {
            try {
                zone.render(beam_config);
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(e.what(),
                             "Zone: mode must be OCCUPANCY or VACANCY");
                throw;
            }
        },
        std::logic_error);
}

TEST(Zone, empty_stl_blob) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.stl = Stl(std::vector<uint8_t>());  // empty STL
    zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    zone.point_count = 1000;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    EXPECT_THROW(
        {
            try {
                zone.render(beam_config);
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(e.what(), "Zone: STL blob cannot be empty");
                throw;
            }
        },
        std::logic_error);
}

TEST(Zone, coordinate_frame_not_set) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.stl = Stl(data_dir + "/0.stl");
    // coordinate_frame not set
    zone.point_count = 1000;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    EXPECT_THROW(
        {
            try {
                zone.render(beam_config);
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(
                    e.what(),
                    "Zone: STL coordinate frame must be BODY or SENSOR");
                throw;
            }
        },
        std::logic_error);
}

TEST(Zone, no_stl_but_zrb) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.stl = Stl(data_dir + "/empty.stl");
    zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    zone.point_count = 1000;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    // TODO[tws] change to throw?
    EXPECT_FALSE(zone.render(beam_config));
}

TEST(Zone, equality) {
    std::string data_dir = getenvs("DATA_DIR");
    Zone zone1;
    zone1.point_count = 1000;
    zone1.frame_count = 10;
    zone1.mode = Zone::ZoneMode::OCCUPANCY;
    zone1.stl = Stl(data_dir + "/0.stl");
    Zone zone2 = zone1;
    ;
    EXPECT_EQ(zone1, zone2);
}

TEST(Zone, inequality) {
    std::string data_dir = getenvs("DATA_DIR");
    Zone zone1;
    zone1.point_count = 1000;
    zone1.frame_count = 10;
    zone1.mode = Zone::ZoneMode::OCCUPANCY;
    zone1.stl = Stl(data_dir + "/0.stl");
    Zone zone2 = zone1;
    zone2.point_count = 2000;
    EXPECT_NE(zone1, zone2);
    zone2 = zone1;
    zone2.frame_count = 20;
    EXPECT_NE(zone1, zone2);
    zone2 = zone1;
    zone2.mode = Zone::ZoneMode::VACANCY;
    EXPECT_NE(zone1, zone2);
    zone2 = zone1;
    zone2.stl = Stl(data_dir + "/1.stl");
    EXPECT_NE(zone1, zone2);
}

TEST(Zone, non_zero_far_pixel_count_must_be_less_than_point_count) {
    // It should throw if the ZRB far range image has fewer nonzero pixels than
    // point_count
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.point_count = 32000;  // very large point count
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    auto zrb = Zrb(data_dir + "/0.zrb");
    zone.zrb = zrb;
    EXPECT_EQ((zone.zrb->far_range_mm != 0).count(), 12097);

    EXPECT_THROW(
        {
            try {
                // Calls check_invariants
                zone.render(beam_config);
            } catch (const std::logic_error& e) {
                EXPECT_STREQ(e.what(),
                             "Zone: ZRB far range image has fewer nonzero "
                             "pixels than point_count");
                throw;
            }
        },
        std::logic_error);
}

TEST(Zone, area_of_rendered_zone) {
    // It should throw if the area of the rendered zone is smaller than
    // point_count
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.point_count = 32000;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    auto stl = Stl(data_dir + "/0.stl");
    stl.coordinate_frame = Stl::CoordinateFrame::BODY;
    zone.stl = stl;
    EXPECT_THROW(
        {
            try {
                zone.render(beam_config);
            } catch (const std::logic_error& e) {
                EXPECT_STREQ(e.what(),
                             "Zone: area of rendered zone (12097) is smaller "
                             "than point_count (32000) specified in zone.");
                throw;
            }
        },
        std::logic_error);
}

TEST(Zone, render_no_stl) {
    // It should return false if no STL is provided, but a ZRB is present
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.point_count = 10;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    auto zrb = Zrb(data_dir + "/0.zrb");
    zone.zrb = zrb;
    ASSERT_TRUE(zone.zrb.has_value());
    EXPECT_FALSE(zone.render(beam_config));
}

TEST(Zone, tiny_meshes_might_not_intersect) {
    std::string data_dir = getenvs("DATA_DIR");
    BeamConfig beam_config = test_beam_config(data_dir);
    Zone zone{};
    zone.point_count = 10;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    auto stl = Stl(data_dir + "/tiny.stl");
    stl.coordinate_frame = Stl::CoordinateFrame::BODY;
    zone.stl = stl;
    // No intersections occurred
    EXPECT_FALSE(zone.render(beam_config));
}
// No intersections occurred

TEST(Zone, string_to_zonemode) {
    // It should return the correct ZoneMode for valid strings, and false
    // otherwise
    Zone::ZoneMode mode;
    EXPECT_TRUE(Zone::string_to_zonemode("OCCUPANCY", mode));
    EXPECT_EQ(mode, Zone::ZoneMode::OCCUPANCY);
    EXPECT_TRUE(Zone::string_to_zonemode("VACANCY", mode));
    EXPECT_EQ(mode, Zone::ZoneMode::VACANCY);
    EXPECT_FALSE(Zone::string_to_zonemode("NONE", mode));
    EXPECT_FALSE(Zone::string_to_zonemode("ASDF", mode));
}

TEST(Zone, zonemode_to_string) {
    // It should return the correct string for valid ZoneModes, and "UNKNOWN"
    // otherwise
    EXPECT_EQ(ouster::sdk::core::to_string(Zone::ZoneMode::NONE), "NONE");
    EXPECT_EQ(ouster::sdk::core::to_string(Zone::ZoneMode::OCCUPANCY),
              "OCCUPANCY");
    EXPECT_EQ(ouster::sdk::core::to_string(Zone::ZoneMode::VACANCY), "VACANCY");
    // It should return "UNKNOWN" for invalid values
    EXPECT_EQ(ouster::sdk::core::to_string(static_cast<Zone::ZoneMode>(-1)),
              "UNKNOWN");
}

TEST(Zone, render_plane) {
    // Single mesh intersections should set the near range to zero
    // and far range to the distance to the plane
    std::string data_dir = getenvs("DATA_DIR");
    Stl stl(data_dir + "/plane.stl");
    stl.coordinate_frame = Stl::CoordinateFrame::BODY;
    auto sensor_to_body_transform = mat4d::Identity();
    auto sensor_info =
        ouster::sdk::core::metadata_from_json(data_dir + "/785.json");
    float m_per_zmbin = ouster::sdk::core::DEFAULT_M_PER_ZMBIN;
    BeamConfig beam_config(
        sensor_info.w(), sensor_info.beam_altitude_angles,
        sensor_info.beam_azimuth_angles, sensor_info.beam_to_lidar_transform,
        sensor_info.lidar_to_sensor_transform, sensor_to_body_transform,
        m_per_zmbin, sensor_info.sn);
    Zone zone{};
    zone.point_count = 10;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    zone.stl = stl;
    ASSERT_TRUE(zone.render(beam_config));
    auto zrb = zone.zrb.value();
    EXPECT_TRUE((zrb.near_range_mm == 0).all());
    int w = beam_config.n_cols;
    int h = beam_config.n_rows;
    // determined experimentally
    EXPECT_EQ(zrb.far_range_mm(h / 2, w / 2), 1011);
}

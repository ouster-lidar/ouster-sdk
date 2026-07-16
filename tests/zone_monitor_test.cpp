/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/core/zone_monitor.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <jsoncons/json.hpp>

#include "nonstd/optional.hpp"
#include "ouster/core/compat_ops.h"
#include "ouster/core/stl.h"
#include "ouster/core/types.h"
#include "ouster/core/zone.h"
#include "ouster/core/zrb.h"
#include "util.h"

using ouster::sdk::core::Coord;
using ouster::sdk::core::Stl;
using ouster::sdk::core::Zone;
using ouster::sdk::core::Zrb;

namespace ouster {
namespace sdk {
namespace core {

TEST(ZoneSet, it_allows_missing_sensor_to_body_transform_when_serializing) {
    ZoneSet zone_set;
    zone_set.power_on_live_ids = {0};
    Zone zone{};
    zone.point_count = 50;
    zone.frame_count = 2;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    std::string data_dir = getenvs("DATA_DIR");
    zone.stl = Stl(data_dir + "/0.stl");
    zone.stl->coordinate_frame = Stl::CoordinateFrame::SENSOR;
    zone_set.zones[0] = zone;

    EXPECT_NO_THROW(zone_set.to_zip_blob(ZoneSetOutputFilter::STL));
    auto metadata = jsoncons::json::parse(zone_set.to_json(ZoneSetOutputFilter::STL));
    EXPECT_FALSE(metadata.contains("sensor_to_body_transform"));
}

TEST(ZoneSet, it_parses_metadata_without_sensor_to_body_transform) {
    std::string data_dir = getenvs("DATA_DIR");
    ZoneSet zone_set;
    zone_set.power_on_live_ids = {0};
    Zone zone{};
    zone.point_count = 50;
    zone.frame_count = 2;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    zone.stl = Stl(data_dir + "/0.stl");
    zone.stl->coordinate_frame = Stl::CoordinateFrame::SENSOR;
    zone_set.zones[0] = zone;

    auto zip = zone_set.to_zip_blob(ZoneSetOutputFilter::STL);
    ZoneSet parsed(zip);
    EXPECT_FALSE(parsed.sensor_to_body_transform.has_value());
}

TEST(ZoneSet, render_fails_for_body_frame_without_sensor_to_body_transform) {
    std::string data_dir = getenvs("DATA_DIR");
    std::string sensor_info_path = data_dir + "/785.json";
    auto sensor = ouster::sdk::core::metadata_from_json(sensor_info_path);

    ZoneSet zone_set;
    Zone zone{};
    zone.point_count = 100;
    zone.frame_count = 2;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    zone.stl = Stl(data_dir + "/0.stl");
    zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    zone_set.zones[0] = zone;

    EXPECT_THROW(
        {
            try {
                zone_set.render(sensor);
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(e.what(), "ZoneSet::render: zone 0 was out of sensor FOV.");
                throw;
            }
        },
        std::runtime_error);
}

TEST(ZoneSet, render_succeeds_for_sensor_frame_without_sensor_to_body_transform) {
    std::string data_dir = getenvs("DATA_DIR");
    std::string sensor_info_path = data_dir + "/785.json";
    auto sensor = ouster::sdk::core::metadata_from_json(sensor_info_path);

    ZoneSet zone_set;
    Zone zone{};
    zone.point_count = 100;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    zone.stl = Stl(data_dir + "/0.stl");
    zone.stl->coordinate_frame = Stl::CoordinateFrame::SENSOR;
    zone_set.zones[0] = zone;

    EXPECT_NO_THROW(zone_set.render(sensor));
    EXPECT_TRUE(zone_set.zones[0].zrb.has_value());
}

TEST(ZoneSet, render) {
    // Get the sensor info
    std::string data_dir = getenvs("DATA_DIR");
    std::string sensor_info_path = data_dir + "/785.json";
    auto sensor = ouster::sdk::core::metadata_from_json(sensor_info_path);

    // Construct a ZoneSet with an STL zone
    ZoneSet zone_set;
    mat4d transform = mat4d::Identity();
    transform(2, 3) = 1.0;
    zone_set.sensor_to_body_transform = transform;
    Zone zone{};
    zone.point_count = 100;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    std::string stl_path = data_dir + "/0.stl";
    std::string zrb_path = data_dir + "/0.zrb";
    zone.stl = Stl(stl_path);
    zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    ASSERT_FALSE(zone.zrb);
    zone_set.zones[0] = zone;

    // Rendering should succeed and populate the zone_binary_blob
    zone_set.render(sensor);

    auto expected_bytes = ouster::sdk::core::get_file_as_bytes(zrb_path);
    EXPECT_EQ(zone_set.zones[0].zrb->blob().size(), expected_bytes.size());
    // TODO[tws] re-enable this after we merge deterministic rendering
    // EXPECT_EQ(zone_set.zones[0].zone_binary_blob,
    //          expected_bytes);
}

TEST(ZoneSet, render_out_of_fov) {
    // Get the sensor info
    std::string data_dir = getenvs("DATA_DIR");
    std::string sensor_info_path = data_dir + "/785.json";
    auto sensor = ouster::sdk::core::metadata_from_json(sensor_info_path);

    // Construct a ZoneSet with an STL zone
    ZoneSet zone_set;
    mat4d transform = mat4d::Identity();

    // Rotate 90 degrees around Y axis to point the zone away from the sensor
    Eigen::AngleAxisf rotation_y(M_PI / 2, Eigen::Vector3f::UnitY());
    transform.block<3, 3>(0, 0) = rotation_y.toRotationMatrix().cast<double>();
    zone_set.sensor_to_body_transform = transform;

    Zone zone{};
    zone.point_count = 1;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    std::string stl_path = data_dir + "/1.stl";
    std::string zrb_path = data_dir + "/1.zrb";
    zone.stl = Stl(stl_path);
    zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    ASSERT_FALSE(zone.zrb);
    zone_set.zones[0] = zone;

    // Rendering should throw because a zone is out of the sensor FOV
    EXPECT_THROW(
        {
            try {
                zone_set.render(sensor);
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(e.what(), "ZoneSet::render: zone 0 was out of sensor FOV.");
                throw;
            }
        },
        std::runtime_error);
}

ZoneSet test_zone_set() {
    // Get the sensor info
    std::string data_dir = getenvs("DATA_DIR");
    std::string sensor_info_path = data_dir + "/785.json";
    auto sensor = ouster::sdk::core::metadata_from_json(sensor_info_path);

    ZoneSet zone_set;
    zone_set.power_on_live_ids = std::vector<uint32_t>{0, 1, 2, 3, 4, 5};
    mat4d transform = mat4d::Identity();
    transform(2, 3) = 1.0;  // add a translation to make sure we
                            // handle non-identity sensor_to_body_transform
    zone_set.sensor_to_body_transform = transform;
    Zone zone0;
    zone0.point_count = 50;
    zone0.frame_count = 2;
    zone0.mode = Zone::ZoneMode::OCCUPANCY;
    zone0.stl = Stl(data_dir + "/0.stl");
    zone0.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    zone_set.zones[0] = zone0;
    Zone zone1;
    zone1.point_count = 50;
    zone1.frame_count = 2;
    zone1.mode = Zone::ZoneMode::VACANCY;
    zone1.stl = Stl(data_dir + "/1.stl");
    zone1.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
    zone_set.zones[1] = zone1;

    zone_set.render(sensor);
    return zone_set;
}

void zip_file_test(const ZoneSet& config) {
    auto expected_default_active_zones = std::vector<uint32_t>{0, 1, 2, 3, 4, 5};
    EXPECT_EQ(config.power_on_live_ids, expected_default_active_zones);

    mat4d mat = mat4d::Identity();
    mat(2, 3) = 1.0;  // This applies a translation (units in meters) to test
                      // that we render zones in the correct spot when
                      // sensor_to_body_transform aren't identity
    ASSERT_TRUE(config.sensor_to_body_transform.has_value());
    EXPECT_EQ(*config.sensor_to_body_transform, mat);

    EXPECT_EQ(config.zones.size(), 2);
    EXPECT_EQ(config.zones.at(0).point_count, 50);
    EXPECT_EQ(config.zones.at(0).frame_count, 2);
    EXPECT_EQ(config.zones.at(0).mode, Zone::ZoneMode::OCCUPANCY);
    EXPECT_TRUE(config.zones.at(0).stl.has_value());
    EXPECT_EQ(config.zones.at(1).point_count, 50);
    EXPECT_EQ(config.zones.at(1).frame_count, 2);
    EXPECT_EQ(config.zones.at(1).mode, Zone::ZoneMode::VACANCY);
    EXPECT_TRUE(config.zones.at(1).stl.has_value());
    EXPECT_TRUE(config.zones.at(0).zrb.has_value());
    EXPECT_TRUE(config.zones.at(1).zrb.has_value());
}

TEST(ZoneMonitorTests, from_zip_file_test) {
    ZoneSet config = test_zone_set();
    std::string data_dir = getenvs("DATA_DIR");
    std::string cfg_path = data_dir + "/test_6_zones_zmcfg.zip";
    config.save(cfg_path, ZoneSetOutputFilter::STL_AND_ZRB);
    ZoneSet config2 = ZoneSet(cfg_path);
    zip_file_test(config2);
    std::remove(cfg_path.c_str());
}

TEST(ZoneMonitorTests, from_zip_file_invalid) {
    EXPECT_THROW(
        {
            try {
                ZoneSet config = ZoneSet("nonexistent_file.zip");
            } catch (const std::invalid_argument& e) {
                ASSERT_STREQ(e.what(),
                             "Error reading metadata.json from zip archive: No "
                             "such file");
                throw;
            }
        },
        std::invalid_argument);
}

TEST(ZoneMonitorTests, blob_test) {
    // Get the sensor info
    std::string data_dir = getenvs("DATA_DIR");
    std::string sensor_info_path = data_dir + "/785.json";
    auto sensor = ouster::sdk::core::metadata_from_json(sensor_info_path);

    ZoneSet config;

    config.power_on_live_ids = std::vector<uint32_t>{0, 1, 2};
    mat4d transform;
    transform << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
    config.sensor_to_body_transform = transform;

    config.zones[0] = Zone();
    config.zones[0].point_count = 50;
    config.zones[0].frame_count = 2;
    config.zones[0].mode = Zone::ZoneMode::OCCUPANCY;
    config.zones[1] = Zone();
    config.zones[1].point_count = 51;
    config.zones[1].frame_count = 3;
    config.zones[1].mode = Zone::ZoneMode::OCCUPANCY;
    config.zones[2] = Zone();
    config.zones[2].point_count = 52;
    config.zones[2].frame_count = 4;
    config.zones[2].mode = Zone::ZoneMode::OCCUPANCY;

    // Technically, blob() will work without zone_binary_blob's all being
    // set, but from_zip_bytes will not, as the sensor will always produce them
    auto test_stl = Stl(std::vector<uint8_t>{0, 1, 2});
    test_stl.coordinate_frame = Stl::CoordinateFrame::BODY;
    config.zones[0].stl = test_stl;
    config.zones[1].stl = test_stl;
    config.zones[2].stl = test_stl;

    auto zip = config.to_zip_blob(ZoneSetOutputFilter::STL);
    ZoneSet cfg2 = ZoneSet(zip);

    for (unsigned i = 0; i < config.zones.size(); i++) {
        EXPECT_EQ(cfg2.zones[i].point_count, config.zones[i].point_count);
        EXPECT_EQ(cfg2.zones[i].frame_count, config.zones[i].frame_count);
        EXPECT_EQ(cfg2.zones[i].mode, config.zones[i].mode);
        EXPECT_EQ(cfg2.zones[i].stl->coordinate_frame, config.zones[i].stl->coordinate_frame);

        EXPECT_EQ(cfg2.zones[i].stl->blob().size(), config.zones[i].stl->blob().size());
        EXPECT_EQ(cfg2.zones[i].stl->hash().str(), config.zones[i].stl->hash().str());
        EXPECT_EQ(cfg2.zones[i].stl->blob(), config.zones[i].stl->blob());

        EXPECT_FALSE(cfg2.zones[i].zrb);
    }
}

TEST(ZoneSet, it_parses_the_coordinate_frame_field_if_the_zone_set_is_stl_type) {
    // Get the sensor info
    std::string data_dir = getenvs("DATA_DIR");
    std::string sensor_info_path = data_dir + "/785.json";
    auto sensor = ouster::sdk::core::metadata_from_json(sensor_info_path);

    ZoneSet config;
    config.sensor_to_body_transform = mat4d::Identity();
    auto zone = Zone();
    zone.point_count = 100;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    zone.stl = Stl(std::vector<uint8_t>{0, 1, 2});  // bogus STL data
    zone.stl->coordinate_frame = Stl::CoordinateFrame::SENSOR;
    config.zones[0] = zone;
    auto blob = config.to_zip_blob(ZoneSetOutputFilter::STL_AND_ZRB);
    ZoneSet cfg2 = ZoneSet(blob);
    ASSERT_TRUE(cfg2.zones[0].stl.has_value());
    EXPECT_EQ(cfg2.zones[0].stl->coordinate_frame, Stl::CoordinateFrame::SENSOR);
}

TEST(ZoneSet, point_cloud_sanity_check_when_saving_zrbs) {
    ZoneSet zone_set;
    zone_set.sensor_to_body_transform = mat4d::Identity();
    Zone zone{};
    zone.point_count = 1000;
    zone.frame_count = 10;
    zone.mode = Zone::ZoneMode::OCCUPANCY;
    std::string data_dir = getenvs("DATA_DIR");
    std::string stl_path = data_dir + "/0.stl";
    zone.zrb = Zrb();
    zone.zrb->near_range_mm = img_t<uint32_t>(10, 10);
    zone.zrb->far_range_mm = img_t<uint32_t>(10, 10);
    zone_set.zones[0] = zone;
    EXPECT_THROW(
        {
            try {
                zone_set.to_zip_blob(ZoneSetOutputFilter::STL_AND_ZRB);
            } catch (const std::logic_error& e) {
                EXPECT_STREQ(e.what(),
                             "ZoneSet: Zone 0 failed invariant check: Zone: ZRB far "
                             "range image has fewer nonzero pixels than point_count");
                throw;
            }
        },
        std::logic_error);
}

TEST(ZoneSet, equality) {
    ZoneSet config1 = test_zone_set();
    ZoneSet config2 = test_zone_set();
    EXPECT_EQ(config1, config2);
}

TEST(ZoneSet, inequality) {
    ZoneSet config1 = test_zone_set();
    ZoneSet config2 = test_zone_set();
    config2.power_on_live_ids.push_back(6);
    EXPECT_NE(config1, config2);
    config2 = config1;
    auto modified_sensor_to_body = *config1.sensor_to_body_transform;
    modified_sensor_to_body(0, 0) += 0.1;
    config2.sensor_to_body_transform = modified_sensor_to_body;
    EXPECT_NE(config1, config2);
    config2 = config1;
    config2.zones[0].point_count += 1;
    EXPECT_NE(config1, config2);
    config2 = config1;
    config2.zones[0].frame_count += 1;
    EXPECT_NE(config1, config2);
    config2 = config1;
    config2.zones[0].mode = (config2.zones[0].mode == Zone::ZoneMode::OCCUPANCY)
                                ? Zone::ZoneMode::VACANCY
                                : Zone::ZoneMode::OCCUPANCY;
    EXPECT_NE(config1, config2);
}

TEST(ZoneSet, all_zrbs_must_have_the_same_resolution) {
    ZoneSet zone_set;
    zone_set.sensor_to_body_transform = mat4d::Identity();
    Zone zone1{};
    zone1.point_count = 1;
    zone1.frame_count = 10;
    zone1.mode = Zone::ZoneMode::OCCUPANCY;

    Zrb zrb;
    zrb.serial_number = 1234;
    zrb.near_range_mm = img_t<uint32_t>(10, 10);
    zrb.far_range_mm = img_t<uint32_t>(10, 10);
    zrb.near_range_mm.fill(1);
    zrb.far_range_mm.fill(1);
    zrb.beam_to_lidar_transform = mat4d::Identity();
    zrb.lidar_to_sensor_transform = mat4d::Identity();
    zrb.sensor_to_body_transform = mat4d::Identity();

    zone1.zrb = zrb;
    zone_set.zones[0] = zone1;

    Zone zone2 = zone1;
    zone2.zrb->near_range_mm = img_t<uint32_t>(20, 20);
    zone2.zrb->far_range_mm = img_t<uint32_t>(20, 20);
    zone2.zrb->near_range_mm.fill(1);
    zone2.zrb->far_range_mm.fill(1);
    zone_set.zones[1] = zone2;

    ASSERT_NE(zone_set.zones[0].zrb->near_range_mm.rows(),
              zone_set.zones[1].zrb->near_range_mm.rows());
    ASSERT_NE(zone_set.zones[0].zrb->near_range_mm.cols(),
              zone_set.zones[1].zrb->near_range_mm.cols());

    EXPECT_THROW(
        {
            try {
                zone_set.to_zip_blob(ZoneSetOutputFilter::ZRB);
            } catch (const std::logic_error& e) {
                EXPECT_STREQ(e.what(), "ZoneSet: all ZRBs must have the same resolution.");
                throw;
            }
        },
        std::logic_error);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

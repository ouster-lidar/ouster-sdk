#include "ouster/zrb.h"

#include <gtest/gtest.h>

#include <fstream>

#include "util.h"

using ouster::sdk::core::Zrb;

TEST(Zrb, from_file_stream_bad) {
    // It should throw if the file does not exist
    std::ifstream file("non_existent_file.stl");
    EXPECT_THROW(
        {
            try {
                Zrb zrb(file);
            } catch (const std::runtime_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Zrb read error: could not open file", e.what());
                throw;
            }
        },
        std::runtime_error);
}

TEST(Zrb, from_file_bad) {
    // It should throw if the file does not exist
    EXPECT_THROW(
        {
            try {
                Zrb zrb("non_existent_file.zrb");
            } catch (const std::runtime_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Zrb read error: could not open file", e.what());
                throw;
            }
        },
        std::runtime_error);
}

TEST(Zrb, from_file) {
    std::string data_dir = getenvs("DATA_DIR");
    std::string zrb_path = data_dir + "/0.zrb";
    // It should not throw
    Zrb zrb(zrb_path);
    EXPECT_EQ(zrb.near_range_mm.cols(), 1024);
    EXPECT_EQ(zrb.near_range_mm.rows(), 128);
    EXPECT_EQ(zrb.serial_number, 122247000785ULL);
    EXPECT_EQ(zrb.stl_hash,
              ouster::sdk::core::Sha256("9cb392667efd9bb1dd2f02c138049243a6103b"
                                        "4a0ef86574681c0641a195c7fd"));
}

TEST(Zrb, equality) {
    // Its equality operator should return true for two identical ZRBs
    std::string data_dir = getenvs("DATA_DIR");
    std::string zrb_path = data_dir + "/0.zrb";
    Zrb zrb1(zrb_path);
    Zrb zrb2(zrb_path);
    EXPECT_EQ(zrb1, zrb2);
}

TEST(Zrb, inequality) {
    // Its inequality operator should return true if any field is different
    std::string data_dir = getenvs("DATA_DIR");
    std::string zrb_path = data_dir + "/0.zrb";
    Zrb zrb1(zrb_path);
    Zrb zrb2(zrb_path);
    // It should be not equal if any field is changed
    zrb2.serial_number += 1;
    EXPECT_NE(zrb1, zrb2);
    zrb2 = zrb1;
    zrb2.near_range_mm(0, 0) += 4;
    EXPECT_NE(zrb1, zrb2);
    zrb2 = zrb1;
    zrb2.far_range_mm(0, 0) += 4;
    EXPECT_NE(zrb1, zrb2);
    zrb2 = zrb1;
    zrb2.beam_to_lidar_transform(0, 0) += 0.1;
    EXPECT_NE(zrb1, zrb2);
    zrb2 = zrb1;
    zrb2.lidar_to_sensor_transform(0, 0) += 0.1;
    EXPECT_NE(zrb1, zrb2);
    zrb2 = zrb1;
    zrb2.sensor_to_body_transform(0, 0) += 0.1;
    EXPECT_NE(zrb1, zrb2);
}

TEST(Zrb, save_bad) {
    // It should throw if the output stream is not valid
    std::string data_dir = getenvs("DATA_DIR");
    std::string zrb_path = data_dir + "/0.zrb";
    Zrb zrb(zrb_path);
    EXPECT_THROW(
        {
            try {
                zrb.save("");
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Zrb::save: output stream not valid", e.what());
                throw;
            }
        },
        std::logic_error);
}

TEST(Zrb, blob_bad) {
    // It should throw if the serial number is not set or images have no data
    Zrb zrb;
    EXPECT_THROW(
        {
            try {
                zrb.blob();
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Zrb::save: serial number not set", e.what());
                throw;
            }
        },
        std::logic_error);

    zrb.serial_number = 123456789;
    EXPECT_THROW(
        {
            try {
                zrb.blob();
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Zrb::save: near image data missing", e.what());
                throw;
            }
        },
        std::logic_error);

    zrb.near_range_mm = ouster::sdk::core::img_t<uint32_t>(10, 10);
    EXPECT_THROW(
        {
            try {
                zrb.blob();
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Zrb::save: far image data missing", e.what());
                throw;
            }
        },
        std::logic_error);
    zrb.far_range_mm = ouster::sdk::core::img_t<uint32_t>(10, 10);
    EXPECT_THROW(
        {
            try {
                zrb.blob();
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Zrb::save: transforms not set for zrb", e.what());
                throw;
            }
        },
        std::logic_error);
}

TEST(Zrb, load_default_stl_hash) {
    // The stl hash should be set to nullopt on read if it is the default value
    std::string data_dir = getenvs("DATA_DIR");
    std::string zrb_path = data_dir + "/0.zrb";
    Zrb zrb(zrb_path);
    zrb.stl_hash = ouster::sdk::core::Sha256();  // Set to default value
    // Save to a temporary file
    std::string temp_zrb_path = data_dir + "/temp.zrb";
    zrb.save(temp_zrb_path);
    // Load it back
    Zrb zrb_loaded(temp_zrb_path);
    // The STL hash should be set to nullopt if it is the default value
    EXPECT_EQ(zrb_loaded.stl_hash, nonstd::nullopt);
}

TEST(Zrb, images_have_different_sizes) {
    // It should throw if near and far images have different sizes
    std::string data_dir = getenvs("DATA_DIR");
    Zrb zrb;
    zrb.serial_number = 123456789;
    zrb.near_range_mm = ouster::sdk::core::img_t<uint32_t>(10, 10);
    zrb.far_range_mm = ouster::sdk::core::img_t<uint32_t>(10, 11);
    EXPECT_THROW(
        {
            try {
                zrb.save(data_dir + "/temp.zrb");
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ(
                    "Zrb::save: near_range_mm and far_range_mm images have "
                    "different sizes",
                    e.what());
                throw;
            }
        },
        std::logic_error);
}

TEST(Zrb, overflow) {
    // It throws if the zrb encoder detects overflow in the images
    Zrb zrb;
    zrb.serial_number = 123456789;
    zrb.near_range_mm = ouster::sdk::core::img_t<uint32_t>(10, 10);
    zrb.far_range_mm = ouster::sdk::core::img_t<uint32_t>(10, 10);
    zrb.beam_to_lidar_transform = ouster::sdk::core::mat4d::Identity();
    zrb.lidar_to_sensor_transform = ouster::sdk::core::mat4d::Identity();
    zrb.sensor_to_body_transform = ouster::sdk::core::mat4d::Identity();
    // Fill images with max uint32_t values to test overflow
    zrb.near_range_mm.fill(UINT32_MAX - 1);
    zrb.far_range_mm.fill(UINT32_MAX);
    EXPECT_THROW(
        {
            try {
                zrb.blob();
            } catch (const std::logic_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ(
                    "Zrb::save: range value exceeds maximum encodable distance",
                    e.what());
                throw;
            }
        },
        std::logic_error);
}

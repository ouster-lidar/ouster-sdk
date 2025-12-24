#include "ouster/stl.h"

#include <gtest/gtest.h>

#include <fstream>

#include "util.h"

using ouster::sdk::core::Stl;

TEST(Stl, from_file_stream_bad) {
    // It should throw if the stream can't be read
    EXPECT_THROW(
        {
            try {
                std::ifstream file("non_existent_file.stl");
                Stl stl(file);
            } catch (const std::runtime_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Error reading STL from stream", e.what());
                throw;
            }
        },
        std::runtime_error);
}

TEST(Stl, from_file_bad) {
    // It should throw if the file does not exist
    EXPECT_THROW(
        {
            try {
                Stl stl("non_existent_file.stl");
            } catch (const std::runtime_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Error opening non_existent_file.stl", e.what());
                throw;
            }
        },
        std::runtime_error);
}

TEST(Stl, construct_from_file) {
    std::string data_dir = getenvs("DATA_DIR");
    std::string stl_path = data_dir + "/5.stl";
    Stl stl(stl_path);
    EXPECT_EQ(stl.blob().size(), 684);
    // Confirmed using sha256sum
    EXPECT_EQ(
        stl.hash().str(),
        "adf5909e932a04512dc7e03d5733848e40662a0bf381e32799c762ced9b55ef3");
    // It should not throw
    (void)stl.to_mesh();
}

TEST(Stl, to_mesh) {
    // It should throw if the stl data couldn't be parsed into a mesh
    Stl stl(std::vector<uint8_t>{0x00, 0x01, 0x02});  // Invalid STL data
    EXPECT_THROW(
        {
            try {
                stl.to_mesh();
            } catch (const std::runtime_error& e) {
                // Check that the exception message is as expected
                EXPECT_STREQ("Unable to parse STL", e.what());
                throw;
            }
        },
        std::runtime_error);
}

TEST(Stl, equality) {
    std::string data_dir = getenvs("DATA_DIR");
    std::string stl_path = data_dir + "/5.stl";
    Stl stl1(stl_path);
    Stl stl2(stl_path);
    EXPECT_EQ(stl1, stl2);
}

TEST(Stl, inequality) {
    std::string data_dir = getenvs("DATA_DIR");
    std::string stl_path1 = data_dir + "/5.stl";
    std::string stl_path2 = data_dir + "/0.stl";
    Stl stl1(stl_path1);
    Stl stl2(stl_path2);
    EXPECT_NE(stl1, stl2);
}

TEST(Stl, filename) {
    // It should extract the filename from the file path
    std::string data_dir = getenvs("DATA_DIR");
    std::string stl_path = data_dir + "/5.stl";
    Stl stl(stl_path);
    EXPECT_EQ(stl.filename, "5.stl");
}

TEST(Stl, coordinate_frame_from_string) {
    // It should return true for valid values and set the output parameter
    // accordingly It should return false otherwise and leave the output
    // parameter unchanged
    Stl::CoordinateFrame cf{Stl::CoordinateFrame::NONE};
    // NOTE - NONE is not considered a valid value for CoordinateFrame
    EXPECT_FALSE(Stl::string_to_coordinate_frame("NONE", cf));
    EXPECT_TRUE(Stl::string_to_coordinate_frame("BODY", cf));
    EXPECT_EQ(cf, Stl::CoordinateFrame::BODY);
    EXPECT_TRUE(Stl::string_to_coordinate_frame("SENSOR", cf));
    EXPECT_EQ(cf, Stl::CoordinateFrame::SENSOR);
    EXPECT_FALSE(Stl::string_to_coordinate_frame("ASDF", cf));
    EXPECT_EQ(cf, Stl::CoordinateFrame::SENSOR);
}

TEST(Stl, coordinate_frame_to_string) {
    EXPECT_EQ(ouster::sdk::core::to_string(Stl::CoordinateFrame::NONE), "NONE");
    EXPECT_EQ(ouster::sdk::core::to_string(Stl::CoordinateFrame::BODY), "BODY");
    EXPECT_EQ(ouster::sdk::core::to_string(Stl::CoordinateFrame::SENSOR),
              "SENSOR");
    // It should return "UNKNOWN" for invalid values
    EXPECT_EQ(
        ouster::sdk::core::to_string(static_cast<Stl::CoordinateFrame>(999)),
        "UNKNOWN");
}

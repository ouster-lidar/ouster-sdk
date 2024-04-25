#include <gtest/gtest.h>

#include <fstream>

#include "fb_utils.h"
#include "osf_test.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/meta_lidar_sensor.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"

namespace ouster {
namespace osf {
namespace {

class MetadataTest : public OsfTestWithData {};

TEST_F(MetadataTest, TestDupeMetadata) {
    MetadataStore meta_store_ = {};
    LidarScanStreamMeta data(1011121314, {});
    EXPECT_EQ(meta_store_.add(data), 1);

    std::stringstream output_stream;
    std::streambuf* old_output_stream = std::cout.rdbuf();
    std::cout.rdbuf(output_stream.rdbuf());
    EXPECT_EQ(meta_store_.add(data), 1);
    std::cout.rdbuf(old_output_stream);
    EXPECT_EQ(output_stream.str(),
              "WARNING: MetadataStore:"
              " ENTRY EXISTS! id = 1\n");
}

class MetadataTestApi : public ouster::osf::MetadataEntry {
   public:
    MetadataTestApi(std::string type, std::string static_type,
                    std::vector<uint8_t> buffer)
        : _type(type), _static_type(static_type), _buffer(buffer){};
    std::vector<uint8_t> buffer() const { return _buffer; };
    std::unique_ptr<MetadataEntry> clone() const { return nullptr; };
    std::string type() const { return _type; };
    std::string static_type() const { return _static_type; };

   private:
    std::string _type;
    std::string _static_type;
    std::vector<uint8_t> _buffer;
};

TEST_F(MetadataTest, MiscMetadataEntryTests) {
    MetadataTestApi test("Screams And Whispers - Dance With the Dead",
                         "Good song", {1, 2, 3, 4, 5});
    EXPECT_EQ(test.repr(), "MetadataEntry: 01 02 03 04 05");
    EXPECT_EQ(test.to_string(),
              "MetadataEntry: [id = 0, type = Screams And Whispers - Dance "
              "With the Dead,"
              " buffer = {MetadataEntry: 01 02 03 04 05}]");
}

}  // namespace
}  // namespace osf
}  // namespace ouster

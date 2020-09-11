#include "ouster/osf/file_info.h"

#include <gtest/gtest.h>

#include "osf_test.h"
#include "ouster/types.h"

namespace ouster {
namespace OSF {
namespace {

namespace os = ouster::sensor;

class OsfFileInfoTest : public OsfTestWithData {};

TEST_F(OsfFileInfoTest, GetBasicFields) {
    TEST_DATA_SKIP();

    OsfFile osf_file(test_data_dir() + "/lib-osf/fake_data_v12.osf");
    ASSERT_TRUE(osf_file);

    FileInfo file_info(osf_file);
    EXPECT_EQ("newSession", file_info.id());

    EXPECT_EQ(4, file_info.range_multiplier());
    EXPECT_EQ(109, file_info.start_ts().count());
    EXPECT_EQ(113, file_info.end_ts().count());

    EXPECT_EQ(1, file_info.mode());
    EXPECT_EQ(1, file_info.lidar_frame_mode());
}

TEST_F(OsfFileInfoTest, GetSensorsFromSession) {
    TEST_DATA_SKIP();

    sensor s;
    EXPECT_EQ("UNKNOWN", s.meta.name);

    OsfFile osf_file(test_data_dir() + "/lib-osf/fake_data_v12.osf");
    FileInfo file_info(osf_file);

    const FileInfo::sensors_map& sm = file_info.sensors();
    EXPECT_EQ(1, sm.size());
    EXPECT_TRUE(sm.count(1) == 1);
    EXPECT_EQ("sensor1_test", sm.at(1)->meta.name);
    EXPECT_EQ(os::MODE_1024x10, sm.at(1)->meta.mode);
    EXPECT_EQ(64, sm.at(1)->meta.beam_azimuth_angles.size());
    EXPECT_EQ(64, sm.at(1)->meta.beam_altitude_angles.size());
    EXPECT_FALSE(file_info.isFramed());
}

}  // namespace
}  // namespace OSF
}  // namespace ouster

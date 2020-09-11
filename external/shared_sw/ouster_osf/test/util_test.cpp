#include <gtest/gtest.h>
#include <sys/stat.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <iostream>

#include "../src/chunk.h"
#include "../src/util_impl.h"
#include "common.h"
#include "common_internals.h"
#include "osf_test.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/lidar_scan_stat.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream.h"
#include "ouster/osf/version.h"
#include "png_tools.h"

namespace ouster {
namespace OSF {
namespace {

class OsfUtilsTest : public OsfTest {};
class OsfUtilsWithDataTest : public OsfTestWithData {};

using ouster::sensor::sensor_info;

TEST_F(OsfUtilsTest, GpsDefaultToZeros) {
    Gps gps;
    EXPECT_EQ(gps.latitude, 0);
    EXPECT_EQ(gps.epy, 0);
    EXPECT_EQ(gps.longitude, 0);
    EXPECT_EQ(gps.epx, 0);
    EXPECT_EQ(gps.altitude, 0);
    EXPECT_EQ(gps.epv, 0);
    EXPECT_EQ(gps.ept, 0);
    EXPECT_EQ(gps.speed, 0);
    EXPECT_EQ(gps.eps, 0);
    EXPECT_EQ(gps.track, 0);
    EXPECT_EQ(gps.epd, 0);
    EXPECT_EQ(gps.climb, 0);
    EXPECT_EQ(gps.epc, 0);
}

// Check that we create the default sensor info
// with an expected field values
TEST_F(OsfUtilsTest, CreatesDefaultSensorInfo) {
    using ouster::sensor::default_sensor_info;
    using ouster::sensor::lidar_mode;
    const sensor_info si{default_sensor_info(lidar_mode::MODE_1024x10)};

    EXPECT_EQ(si.name, "UNKNOWN");
    EXPECT_EQ(si.sn, "000000000000");
    EXPECT_EQ(si.fw_rev, "UNKNOWN");
    EXPECT_EQ(si.mode, lidar_mode::MODE_1024x10);

    EXPECT_EQ(si.beam_azimuth_angles, ouster::sensor::gen1_azimuth_angles);
    EXPECT_EQ(si.beam_altitude_angles, ouster::sensor::gen1_altitude_angles);
    EXPECT_EQ(si.imu_to_sensor_transform,
              ouster::sensor::default_imu_to_sensor_transform);
    EXPECT_EQ(si.lidar_to_sensor_transform,
              ouster::sensor::default_lidar_to_sensor_transform);
}

// Check that we can make a test sensor with some known values
TEST_F(OsfUtilsTest, CreatesDefaultTestSensor) {
    namespace sensor = ouster::sensor;

    const OSF::sensor s{};

    ASSERT_EQ(s.id, 0);

    ASSERT_EQ(s.meta.format.columns_per_frame,
              sensor::n_cols_of_lidar_mode(sensor::MODE_1024x10));
    ASSERT_EQ(s.meta.name, "UNKNOWN");
}

// Check that we can make lidar scan and have fields with expected value inside
TEST_F(OsfUtilsTest, MakesLidarScan) {
    sensor s{};

    LidarScan ls = get_random_lidar_scan(s);

    const auto n =
        s.meta.format.columns_per_frame * s.meta.format.pixels_per_column;

    EXPECT_EQ(ls.w, s.meta.format.columns_per_frame);
    EXPECT_EQ(ls.h, s.meta.format.pixels_per_column);
    EXPECT_EQ(ls.range().size(), n);
    EXPECT_EQ(ls.noise().size(), n);
    EXPECT_EQ(ls.intensity().size(), n);
    EXPECT_EQ(ls.reflectivity().size(), n);
}

// Check that osfChunk stored with V_1_1 (with all message types)
// doesn't verify with the current version
TEST_F(OsfUtilsWithDataTest, DiesOnOpenOSF_V_1_1) {
    TEST_DATA_SKIP();

    OsfBufferOpener opener;
    ASSERT_EXIT(opener = ouster::OSF::openOsfBuffer(
                    test_data_dir() + "/lib-osf/fake_data_v11.osf"),
                ::testing::ExitedWithCode(EXIT_FAILURE),
                "osfChunk is not valid");
}

// Check that we can read OSF file created with OSF version V_1_1 code
// and just lidar_scan + traj messages (this is what we currently have
// running in prod on wardens with customers)
TEST_F(OsfUtilsWithDataTest, VerifiesOSF_ScanAndPoses_Only_V_1_1) {
    TEST_DATA_SKIP();

    const std::string input_osf_filename =
        test_data_dir() + "/lib-osf/fake_scan_and_traj_v11.osf";

    OsfFile osf_file(input_osf_filename);
    Reader reader(osf_file);

    EXPECT_EQ(
        2, std::distance(reader.messages().begin(), reader.messages().end()));

    // Read and check lidar_scan
    auto it = reader.messages().begin();
    auto msg1 = *it;
    auto ls_stored = msg1.as_lidar_scan();

    EXPECT_EQ(msg1.ts(), OSF::ts_t(113));

    LidarScanStat ls_stored_stat{*ls_stored};
    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_LT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0xffff);

    it++;
    auto msg2 = *it;
    EXPECT_EQ(msg2.type(), OSF::MessageType::TRAJECTORY);
    auto traj = msg2.as_trajectory();
    EXPECT_EQ(traj->size(), 1024);

    const ouster::OSF::pose p = traj->at(0);
    EXPECT_DOUBLE_EQ(0.24624180793762207, p.orientation.w());
    EXPECT_DOUBLE_EQ(-0.3149242103099823, p.orientation.x());
    EXPECT_DOUBLE_EQ(-0.89686661958694458, p.orientation.y());
    EXPECT_DOUBLE_EQ(0.18925660848617554, p.orientation.z());
    EXPECT_DOUBLE_EQ(21.306433762118481, p.position.x());
    EXPECT_DOUBLE_EQ(29.388608033159013, p.position.y());
    EXPECT_DOUBLE_EQ(-21.592618260815833, p.position.z());

    EXPECT_EQ(msg2.ts(), OSF::ts_t(113));
}

// Check that we can open and read V_1_2 that contains all message types inside
TEST_F(OsfUtilsWithDataTest, VerifiesOSF_V_1_2) {
    TEST_DATA_SKIP();

    const std::string input_osf_filename =
        test_data_dir() + "/lib-osf/fake_data_v12.osf";

    OsfFile osf_file(input_osf_filename);
    Reader reader(osf_file);

    EXPECT_GT(osf_file.size(), 100 * 1024);

    EXPECT_EQ(
        4, std::distance(reader.messages().begin(), reader.messages().end()));

    auto it = reader.messages().begin();

    ++it;  // go to the second msg
    auto msg1 = *it;
    auto ls_stored = msg1.as_lidar_scan();

    EXPECT_EQ(msg1.ts(), OSF::ts_t(112));

    LidarScanStat ls_stored_stat{*ls_stored};
    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_LT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0xffff);

    // Go to the 4th message
    ++it;
    ++it;

    auto msg2 = *it;
    EXPECT_EQ(msg2.type(), OSF::MessageType::GPS_WAYPOINT);
    auto gps_wp = msg2.as_gps();

    EXPECT_DOUBLE_EQ(gps_wp->latitude, 12.587171877060079);
}

// Check that PNG writer works from LidarScan to file (lossy, compressed)
TEST_F(OsfUtilsTest, SavesLidarScanToPng1) {
    sensor s{};
    LidarScan ls = get_random_lidar_scan(s);

    std::string osf_dest;
    if (!get_output_dir(&osf_dest)) FAIL();

    ScanData scan_data = scanEncodeOSF32(ls, s.meta.format.pixel_shift_by_row,
                                         OSF::RANGE_MULTIPLIER_DEFAULT);
    saveScanChannel(scan_data[0], osf_dest + "/test_frame.png");

    if (!path_exists(osf_dest + "/test_frame.png"))
        ADD_FAILURE() << "PNG file was not generated :(";
}

// Check that PNG writer works from LidarScan to file with different
// methods for range, intensity and noise channels. (baseline for OSF_56 frame)
TEST_F(OsfUtilsTest, SavesLidarScanToPng3) {
    sensor s{};
    LidarScan ls = get_random_lidar_scan(s);

    std::string osf_dest;
    if (!get_output_dir(&osf_dest)) FAIL();

    ScanChannelData res_buf;

    // No error returned
    EXPECT_FALSE(scanEncode24bitChannel(res_buf, ls,
                                        s.meta.format.pixel_shift_by_row,
                                        LidarScan::LidarScanIndex::RANGE));
    saveScanChannel(res_buf, osf_dest + "/r_test_frame.png");
    if (!path_exists(osf_dest + "/r_test_frame.png")) {
        ADD_FAILURE() << "Range PNG file was not generated";
    }

    res_buf.clear();

    // No error returned
    EXPECT_FALSE(scanEncode16bitChannel(res_buf, ls,
                                        s.meta.format.pixel_shift_by_row,
                                        LidarScan::LidarScanIndex::INTENSITY));
    saveScanChannel(res_buf, osf_dest + "/i_test_frame.png");
    if (!path_exists(osf_dest + "/i_test_frame.png")) {
        ADD_FAILURE() << "Intenisty PNG file was not generated";
    }

    res_buf.clear();

    // No error returned
    EXPECT_FALSE(scanEncode16bitChannel(res_buf, ls,
                                        s.meta.format.pixel_shift_by_row,
                                        LidarScan::LidarScanIndex::NOISE));
    saveScanChannel(res_buf, osf_dest + "/n_test_frame.png");
    if (!path_exists(osf_dest + "/n_test_frame.png")) {
        ADD_FAILURE() << "Noise PNG file was not generated";
    }
}

}  // namespace
}  // namespace OSF
}  // namespace ouster

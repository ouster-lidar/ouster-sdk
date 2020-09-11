#include "ouster/osf/stream.h"

#include <gtest/gtest.h>

#include <algorithm>

#include "../src/util_impl.h"
#include "common.h"
#include "common_internals.h"
#include "osf_test.h"
#include "ouster/osf/file.h"
#include "ouster/osf/lidar_scan_stat.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/util.h"

namespace ouster {
namespace OSF {
namespace {

class OsfStreamTest : public OsfTest {};

// Create the OSF file with 1 lidar_scan and 1 traj messages
// and read it back
TEST_F(OsfStreamTest, CreatesOSFStreamWithLidarScanAndPoses) {
    std::string osf_dest;
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(sensors, osf_dest);

    LidarScan ls = get_random_lidar_scan();
    osfs->logLidarScan(sensor_id, ls, 113);

    ouster::OSF::osf_poses poses = get_random_poses(*sensors[sensor_id]);
    osfs->logTrajectory(ouster::OSF::device_car, poses, 113);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("scan_and_poses.osf");

    delete osfs;

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    const ouster::OSF::OsfBufferOpener opener =
        ouster::OSF::openOsfBuffer(osf_dest + "/" + fname);

    ASSERT_TRUE(is_first_chunk_ok(opener));

    EXPECT_GT(opener.file_size, 10 * 1024);

    EXPECT_LT(opener.chunks_offset, opener.session_offset);

    const ouster::OSF::osfChunk* chunk = ouster::OSF::GetSizePrefixedosfChunk(
        opener.osf_file + opener.chunks_offset);

    EXPECT_EQ(chunk->frames()->size(), 2);
}

// Create OSF file with just one lidar_scan message and read it back
TEST_F(OsfStreamTest, CreatesOSFStreamWithLidarScan) {
    // TODO: Extract to SetUp method
    srand(std::time(nullptr));

    std::string osf_dest;
    // Should do FAIL here because it doesn't work
    // inside a helper function. See GTest Advanced doc for details.
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(sensors, osf_dest);

    ASSERT_EQ(osfs->ts_start, 0);
    ASSERT_EQ(osfs->ts_end, 0);

    // Get some lidar_scans
    LidarScan ls = get_random_lidar_scan();

    LidarScanStat ls_stat{ls};
    EXPECT_GT(ls_stat.mean(LidarScan::LidarScanIndex::REFLECTIVITY), 0);

    osfs->logLidarScan(sensor_id, ls, 100);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_lidar_m.osf");

    delete osfs;

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    Reader reader(output_file);

    auto ls_stored = reader.messages().begin()->as_lidar_scan();

    EXPECT_EQ(
        1, std::distance(reader.messages().begin(), reader.messages().end()));

    LidarScanStat ls_stored_stat{*ls_stored};
    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_LT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE),
              (1 << 16) - 1);

    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::INTENSITY), 0);
    EXPECT_LE(ls_stored_stat.mean(LidarScan::LidarScanIndex::INTENSITY), 255);

    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::NOISE), 0);
    EXPECT_LE(ls_stored_stat.mean(LidarScan::LidarScanIndex::NOISE), 255);

    EXPECT_EQ(ls_stored_stat.mean(LidarScan::LidarScanIndex::REFLECTIVITY), 0);
}

// Create OSF file with one gps message and read it back
TEST_F(OsfStreamTest, CreatesOSFStreamWithGps) {
    std::string osf_dest;
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(sensors, osf_dest);

    ouster::OSF::Gps gps = get_random_gps_waypoint();

    osfs->logGps(ouster::OSF::device_gps, gps, 100);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_gps_m.osf");

    delete osfs;

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    Reader reader(output_file);

    auto msg_it = reader.messages().begin();

    EXPECT_EQ(OSF::MessageType::GPS_WAYPOINT, msg_it->type());

    auto gps_wp = msg_it->as_gps();

    EXPECT_EQ(
        1, std::distance(reader.messages().begin(), reader.messages().end()));

    EXPECT_DOUBLE_EQ(gps_wp->latitude, gps.latitude);
    EXPECT_DOUBLE_EQ(gps_wp->longitude, gps.longitude);
    EXPECT_DOUBLE_EQ(gps_wp->altitude, gps.altitude);
    EXPECT_DOUBLE_EQ(gps_wp->latitude, gps.latitude);
    EXPECT_DOUBLE_EQ(gps_wp->epy, gps.epy);
    EXPECT_DOUBLE_EQ(gps_wp->epx, gps.epx);
    EXPECT_DOUBLE_EQ(gps_wp->epv, gps.epv);
    EXPECT_DOUBLE_EQ(gps_wp->ept, gps.ept);
    EXPECT_DOUBLE_EQ(gps_wp->speed, gps.speed);
    EXPECT_DOUBLE_EQ(gps_wp->track, gps.track);
    EXPECT_DOUBLE_EQ(gps_wp->epd, gps.epd);
    EXPECT_DOUBLE_EQ(gps_wp->climb, gps.climb);
    EXPECT_DOUBLE_EQ(gps_wp->epc, gps.epc);
}

// Create OSF file with one imu message and read it back
TEST_F(OsfStreamTest, CreatesOSFStreamWithImu) {
    std::string osf_dest;
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(sensors, osf_dest);

    std::array<double, 3> angular_vel = normal_arr<3>(15, 10);
    std::array<double, 3> linear_acc = normal_arr<3>(0.5, 0.3);

    osfs->logImu(sensor_id, angular_vel, linear_acc, {0, 0, 0}, 100);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_imu_m.osf");

    delete osfs;

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    Reader reader(output_file);

    auto msg_it = reader.messages().begin();

    EXPECT_EQ(
        1, std::distance(reader.messages().begin(), reader.messages().end()));

    EXPECT_EQ(OSF::MessageType::IMU, msg_it->type());

    auto imu = msg_it->as_imu();

    EXPECT_DOUBLE_EQ(imu->angular_vel[0], angular_vel[0]);
    EXPECT_DOUBLE_EQ(imu->angular_vel[1], angular_vel[1]);
    EXPECT_DOUBLE_EQ(imu->angular_vel[2], angular_vel[2]);

    EXPECT_DOUBLE_EQ(imu->linear_accel[0], linear_acc[0]);
    EXPECT_DOUBLE_EQ(imu->linear_accel[1], linear_acc[1]);
    EXPECT_DOUBLE_EQ(imu->linear_accel[2], linear_acc[2]);

    EXPECT_EQ(msg_it->ts().count(), 100);
}

// Create OSF file with one traj message and read it back
TEST_F(OsfStreamTest, CreatesOSFStreamWithTrajectory) {
    std::string osf_dest;
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(sensors, osf_dest);

    ouster::OSF::osf_poses poses = get_random_poses(*sensors[sensor_id]);

    osfs->logTrajectory(ouster::OSF::device_car, poses, 110);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_traj_m.osf");

    delete osfs;

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    Reader reader(output_file);

    auto msg_it = reader.messages().begin();

    EXPECT_EQ(
        1, std::distance(reader.messages().begin(), reader.messages().end()));

    EXPECT_EQ(OSF::MessageType::TRAJECTORY, msg_it->type());

    auto traj = msg_it->as_trajectory();

    EXPECT_EQ(poses.size(), traj->size());

    for (size_t i = 0; i < traj->size(); ++i) {
        const ouster::OSF::pose p = traj->at(i);
        EXPECT_EQ(p.orientation.w(), poses[i].orientation.w());
        EXPECT_EQ(p.orientation.x(), poses[i].orientation.x());
        EXPECT_EQ(p.orientation.y(), poses[i].orientation.y());
        EXPECT_EQ(p.orientation.z(), poses[i].orientation.z());
        EXPECT_EQ(p.position.x(), poses[i].position.x());
        EXPECT_EQ(p.position.y(), poses[i].position.y());
        EXPECT_EQ(p.position.z(), poses[i].position.z());
    }

    EXPECT_EQ(msg_it->ts().count(), 110);
}

// Create OSF with 4 different types and read it back
TEST_F(OsfStreamTest, CreatesOSFStreamWithEveryMessage) {
    /*
     * Record every message and recover them in the same order
     */

    std::string osf_dest;
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(sensors, osf_dest);

    std::array<double, 3> angular_vel = normal_arr<3>(15, 10);
    std::array<double, 3> linear_acc = normal_arr<3>(0.5, 0.3);
    osfs->logImu(sensor_id, angular_vel, linear_acc, {0, 0, 0}, 109);

    LidarScan ls = get_random_lidar_scan();
    osfs->logLidarScan(sensor_id, ls, 112);

    ouster::OSF::osf_poses poses = get_random_poses(*sensors[sensor_id]);
    osfs->logTrajectory(ouster::OSF::device_car, poses, 112);

    ouster::OSF::Gps gps = get_random_gps_waypoint();
    osfs->logGps(ouster::OSF::device_gps, gps, 113);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_every_m.osf");

    delete osfs;

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    Reader reader(output_file);

    EXPECT_EQ(
        4, std::distance(reader.messages().begin(), reader.messages().end()));

    // First is IMU
    auto msg_it = reader.messages().begin();

    EXPECT_EQ(OSF::MessageType::IMU, msg_it->type());
    auto imu = msg_it->as_imu();
    EXPECT_DOUBLE_EQ(imu->angular_vel[0], angular_vel[0]);
    EXPECT_DOUBLE_EQ(imu->angular_vel[1], angular_vel[1]);
    EXPECT_DOUBLE_EQ(imu->angular_vel[2], angular_vel[2]);

    EXPECT_DOUBLE_EQ(imu->linear_accel[0], linear_acc[0]);
    EXPECT_DOUBLE_EQ(imu->linear_accel[1], linear_acc[1]);
    EXPECT_DOUBLE_EQ(imu->linear_accel[2], linear_acc[2]);

    EXPECT_EQ(msg_it->ts().count(), 109);

    // Second is LIDAR_SCAN
    msg_it++;

    auto ls_stored = msg_it->as_lidar_scan();

    LidarScanStat ls_stored_stat{*ls_stored};
    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_LT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE),
              (1 << 16) - 1);

    // Third is TAJECTORY
    msg_it++;

    EXPECT_EQ(OSF::MessageType::TRAJECTORY, msg_it->type());
    auto traj = msg_it->as_trajectory();

    EXPECT_EQ(poses.size(), traj->size());

    for (size_t i = 0; i < traj->size(); ++i) {
        const ouster::OSF::pose p = traj->at(i);
        EXPECT_EQ(p.orientation.w(), poses[i].orientation.w());
        EXPECT_EQ(p.orientation.x(), poses[i].orientation.x());
        EXPECT_EQ(p.orientation.y(), poses[i].orientation.y());
        EXPECT_EQ(p.orientation.z(), poses[i].orientation.z());
        EXPECT_EQ(p.position.x(), poses[i].position.x());
        EXPECT_EQ(p.position.y(), poses[i].position.y());
        EXPECT_EQ(p.position.z(), poses[i].position.z());
    }
    EXPECT_EQ(msg_it->ts().count(), 112);

    // Fourth is GPS_WAYPOINT
    msg_it++;

    EXPECT_EQ(OSF::MessageType::GPS_WAYPOINT, msg_it->type());

    auto gps_wp = msg_it->as_gps();

    EXPECT_DOUBLE_EQ(gps_wp->latitude, gps.latitude);
    EXPECT_DOUBLE_EQ(gps_wp->longitude, gps.longitude);
    EXPECT_DOUBLE_EQ(gps_wp->altitude, gps.altitude);
    EXPECT_DOUBLE_EQ(gps_wp->latitude, gps.latitude);
    EXPECT_DOUBLE_EQ(gps_wp->epy, gps.epy);
    EXPECT_DOUBLE_EQ(gps_wp->epx, gps.epx);
    EXPECT_DOUBLE_EQ(gps_wp->epv, gps.epv);
    EXPECT_DOUBLE_EQ(gps_wp->ept, gps.ept);
    EXPECT_DOUBLE_EQ(gps_wp->speed, gps.speed);
    EXPECT_DOUBLE_EQ(gps_wp->track, gps.track);
    EXPECT_DOUBLE_EQ(gps_wp->epd, gps.epd);
    EXPECT_DOUBLE_EQ(gps_wp->climb, gps.climb);
    EXPECT_DOUBLE_EQ(gps_wp->epc, gps.epc);
}

// Create OSF file with just one lidar_scan OSF_56 encoded message and read it
// back
TEST_F(OsfStreamTest, Creates_OSF_56_OneLidarScan) {
    std::string osf_dest;
    // Should do FAIL here because it doesn't work
    // inside a helper function. See GTest Advanced doc for details.
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(sensors, osf_dest,
                                    OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_56);

    // Get some lidar_scans
    LidarScan ls = get_random_lidar_scan();

    // std::cout << "ls = " << ls << std::endl;

    LidarScanStat ls_stat{ls};
    EXPECT_GT(ls_stat.mean(LidarScan::LidarScanIndex::REFLECTIVITY), 0);

    osfs->logLidarScan(sensor_id, ls, 100);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_lidar_m.osf");

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    delete osfs;

    Reader reader(output_file);

    ASSERT_EQ(
        1, std::distance(reader.messages().begin(), reader.messages().end()));

    auto ls_stored = reader.messages().begin()->as_lidar_scan();

    ASSERT_TRUE(ls_stored) << "lidar scan should be recoverable";

    // std::cout << "ls_stored = " << *ls_stored << std::endl;

    LidarScanStat ls_stored_stat{*ls_stored};
    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_LT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0xfffff);

    // OSF_56 - should have all equal to original lidar_scan but reflectivity()
    EXPECT_TRUE(ls_stored->range().isApprox(ls.range()));
    EXPECT_TRUE(ls_stored->intensity().isApprox(ls.intensity()));
    EXPECT_TRUE(ls_stored->noise().isApprox(ls.noise()));

    EXPECT_TRUE(ls_stored->reflectivity().isApproxToConstant(0));

    EXPECT_EQ(ls_stored->ts, ls.ts);
}

// Create OSF file with just one lidar_scan OSF_72 encoded message and read it
// back
TEST_F(OsfStreamTest, Creates_OSF_72_OneLidarScan) {
    std::string osf_dest;
    // Should do FAIL here because it doesn't work
    // inside a helper function. See GTest Advanced doc for details.
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(sensors, osf_dest,
                                    OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_72);

    // Get some lidar_scans
    LidarScan ls = get_random_lidar_scan();

    LidarScanStat ls_stat{ls};
    EXPECT_GT(ls_stat.mean(LidarScan::LidarScanIndex::REFLECTIVITY), 0);

    osfs->logLidarScan(sensor_id, ls, 100);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_lidar_m.osf");

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    delete osfs;

    Reader reader(output_file);

    ASSERT_EQ(
        1, std::distance(reader.messages().begin(), reader.messages().end()));

    auto ls_stored = reader.messages().begin()->as_lidar_scan();

    ASSERT_TRUE(ls_stored) << "lidar scan should be recoverable";

    LidarScanStat ls_stored_stat{*ls_stored};
    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_LT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0xfffff);

    // OSF_72 - should have all equal data to original lidar_scan (ls)
    EXPECT_TRUE(ls_stored->range().isApprox(ls.range()));
    EXPECT_TRUE(ls_stored->intensity().isApprox(ls.intensity()));
    EXPECT_TRUE(ls_stored->noise().isApprox(ls.noise()));
    EXPECT_TRUE(ls_stored->reflectivity().isApprox(ls.reflectivity()));

    EXPECT_EQ(ls_stored->ts, ls.ts);
}

// Create OSF file with just one lidar_scan OSF_40RI encoded message and read it
// back
TEST_F(OsfStreamTest, Creates_OSF_40RI_OneLidarScan) {
    std::string osf_dest;
    // Should do FAIL here because it doesn't work
    // inside a helper function. See GTest Advanced doc for details.
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(
        sensors, osf_dest, OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_40RI);

    // Get some lidar_scans
    LidarScan ls = get_random_lidar_scan();

    LidarScanStat ls_stat{ls};
    EXPECT_GT(ls_stat.mean(LidarScan::LidarScanIndex::REFLECTIVITY), 0);

    osfs->logLidarScan(sensor_id, ls, 100);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_lidar_m.osf");

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    delete osfs;

    Reader reader(output_file);

    ASSERT_EQ(
        1, std::distance(reader.messages().begin(), reader.messages().end()));

    auto ls_stored = reader.messages().begin()->as_lidar_scan();

    ASSERT_TRUE(ls_stored) << "lidar scan should be recoverable";

    LidarScanStat ls_stored_stat{*ls_stored};
    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_LT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0xfffff);

    // OSF_40RI - should have all equal data for range and intensity
    EXPECT_TRUE(ls_stored->range().isApprox(ls.range()));
    EXPECT_TRUE(ls_stored->intensity().isApprox(ls.intensity()));

    EXPECT_TRUE(ls_stored->noise().isApproxToConstant(0));
    EXPECT_TRUE(ls_stored->reflectivity().isApproxToConstant(0));

    EXPECT_EQ(ls_stored->ts, ls.ts);
}

// Create OSF file with just one lidar_scan OSF_40RN encoded message and read it
// back
TEST_F(OsfStreamTest, Creates_OSF_40RN_OneLidarScan) {
    std::string osf_dest;
    // Should do FAIL here because it doesn't work
    // inside a helper function. See GTest Advanced doc for details.
    if (!get_output_dir(&osf_dest)) FAIL();

    const uint8_t sensor_id = 1;

    OSF::sensors_map sensors;
    sensors[sensor_id] = std::make_shared<sensor>(sensor_id);

    OSFStream* osfs = new OSFStream(
        sensors, osf_dest, OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_40RN);

    // Get some lidar_scans
    LidarScan ls = get_random_lidar_scan();

    LidarScanStat ls_stat{ls};
    EXPECT_GT(ls_stat.mean(LidarScan::LidarScanIndex::REFLECTIVITY), 0);

    osfs->logLidarScan(sensor_id, ls, 100);

    osfs->saveChunk();
    const std::string fname = osfs->createSession("one_lidar_m.osf");

    if (!path_exists(osf_dest + "/" + fname))
        ADD_FAILURE() << "OSF file was not generated :(";

    OsfFile output_file(osf_dest + "/" + fname);
    EXPECT_TRUE(output_file.good());
    EXPECT_TRUE(output_file.valid());

    delete osfs;

    Reader reader(output_file);

    ASSERT_EQ(
        1, std::distance(reader.messages().begin(), reader.messages().end()));

    auto ls_stored = reader.messages().begin()->as_lidar_scan();

    ASSERT_TRUE(ls_stored) << "lidar scan should be recoverable";

    LidarScanStat ls_stored_stat{*ls_stored};
    EXPECT_GT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_LT(ls_stored_stat.mean(LidarScan::LidarScanIndex::RANGE), 0xfffff);

    // OSF_40RI - should have all equal data for range and intensity
    EXPECT_TRUE(ls_stored->range().isApprox(ls.range()));
    EXPECT_TRUE(ls_stored->noise().isApprox(ls.noise()));

    EXPECT_TRUE(ls_stored->intensity().isApproxToConstant(0));
    EXPECT_TRUE(ls_stored->reflectivity().isApproxToConstant(0));

    EXPECT_EQ(ls_stored->ts, ls.ts);
}

}  // namespace
}  // namespace OSF
}  // namespace ouster
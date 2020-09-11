#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "osf_test.h"
#include "ouster/osf/file.h"
#include "ouster/osf/file_info.h"
#include "ouster/osf/lidar_scan_stat.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/util.h"
#include "png_tools.h"
#include "util_impl.h"

namespace ouster {
namespace OSF {
namespace {

class FrameChunkerTest : public OsfTestWithData {
   public:
    static std::string output_dir;
    static std::vector<std::string> files;

    std::string tmp_file(const std::string& basename) {
        std::string res = output_dir + basename;
        files.push_back(res);
        return res;
    }

    static void SetUpTestCase() {
        output_dir = tmp_dir() + "/";
    }

    // clean up temp files
    static void TearDownTestCase() {
        for (const auto& path : files) ::unlink(path.c_str());
        ::rmdir(output_dir.c_str());
    }
};

std::string FrameChunkerTest::output_dir = {};
std::vector<std::string> FrameChunkerTest::files = {};

void compare_sensors_meta(const FileInfo& in_file_info,
                          const FileInfo& out_file_info) {
    EXPECT_EQ(in_file_info.sensors().size(), out_file_info.sensors().size());
    for (const auto sel : out_file_info.sensors()) {
        const auto& sensor_out = sel.second;
        const auto& sensor_in = in_file_info.sensors().at(sel.first);
        EXPECT_EQ(sensor_in->id, sensor_out->id);
        EXPECT_EQ(sensor_in->meta.name, sensor_out->meta.name);
        EXPECT_EQ(sensor_in->meta.sn, sensor_out->meta.sn);
        EXPECT_EQ(sensor_in->meta.fw_rev, sensor_out->meta.fw_rev);
        EXPECT_EQ(sensor_in->meta.mode, sensor_out->meta.mode);
        EXPECT_EQ(sensor_in->meta.prod_line, sensor_out->meta.prod_line);
        EXPECT_EQ(sensor_in->meta.beam_azimuth_angles,
                  sensor_out->meta.beam_azimuth_angles);
        EXPECT_EQ(sensor_in->meta.beam_altitude_angles,
                  sensor_out->meta.beam_altitude_angles);
        EXPECT_EQ(sensor_in->meta.lidar_origin_to_beam_origin_mm,
                  sensor_out->meta.lidar_origin_to_beam_origin_mm);
        EXPECT_EQ(sensor_in->meta.imu_to_sensor_transform,
                  sensor_out->meta.imu_to_sensor_transform);
        EXPECT_EQ(sensor_in->meta.lidar_to_sensor_transform,
                  sensor_out->meta.lidar_to_sensor_transform);
        EXPECT_EQ(sensor_in->meta.extrinsic, sensor_out->meta.extrinsic);
    }
}

TEST_F(FrameChunkerTest, ChunkOSFFileSmoke) {
    TEST_DATA_SKIP();

    std::string input_osf_filename =
        test_data_dir() + "/lib-osf/ml_data_743_v12_fix_ds/0000.osf";

    std::string output_osf_filename = tmp_file("0000_chunked.osf");

    OsfFile osf_file(input_osf_filename);
    EXPECT_TRUE(osf_file.good());

    FileInfo file_info(osf_file);
    EXPECT_FALSE(file_info.isFramed());

    // Get one lidar scan from input OSF file
    Reader reader(osf_file, {MessageType::LIDAR_SCAN});
    OSF::MessageRef msg = (*reader.messages().begin());
    LidarScan ls = *msg.as_lidar_scan();

    // Check that we have at least one non-zero ts in lidar_scan
    EXPECT_TRUE(std::count_if(ls.ts.begin(), ls.ts.end(),
                              [](LidarScan::ts_t t) { return t.count() > 0; }));

    // Frame Chunker
    rechunk(input_osf_filename, output_osf_filename);

    EXPECT_TRUE(path_exists(output_osf_filename));

    OsfFile out_osf_file(output_osf_filename);
    EXPECT_TRUE(out_osf_file.good());

    FileInfo out_file_info(out_osf_file);
    EXPECT_TRUE(out_file_info.isFramed());

    // Check session and sensors info
    EXPECT_EQ(file_info.id(), out_file_info.id());
    EXPECT_EQ(OSF_FRAME_MODE_OSF_32, out_file_info.lidar_frame_mode());
    compare_sensors_meta(file_info, out_file_info);

    // Get the corresponding lidar scan from the output OSF
    Reader out_reader(out_osf_file, {MessageType::LIDAR_SCAN});
    auto out_msgs = out_reader.messages();
    auto it = std::find_if(
        out_msgs.begin(), out_msgs.end(),
        [&msg](const OSF::MessageRef& m) { return msg.ts() == m.ts(); });
    EXPECT_TRUE(it != out_msgs.end());
    LidarScan out_ls = *it->as_lidar_scan();

    // Check that we have at least one non-zero ts in lidar_scan
    EXPECT_TRUE(std::count_if(out_ls.ts.begin(), out_ls.ts.end(),
                              [](LidarScan::ts_t t) { return t.count() > 0; }));

    // Compare timestamps of input vs output lidar scans
    EXPECT_EQ(ls.ts, out_ls.ts);

    FileStatsFramed fst = calc_framed_file_stats(output_osf_filename);

    // Params of the good frame chunker outcome (but it depends on a file ...)
    EXPECT_LT(fst.dt_var, 0.0001);
    EXPECT_GT(fst.full_frames_ratio, 0.88);
    EXPECT_EQ(fst.dt_past_cnt, 0);
    EXPECT_EQ(fst.start_end_mismatch_cnt, 0);
    EXPECT_EQ(fst.overlapped_frames_cnt, 0);
}

TEST_F(FrameChunkerTest, ChunkOSFFileNewSmokeGpsImu) {
    TEST_DATA_SKIP();

    std::string input_osf_filename =
        test_data_dir() + "/lib-osf/small_with_gps_v12/small_with_gps_v12.osf";

    std::string output_osf_filename = tmp_file("small_with_gps_v12_chunked.osf");

    OsfFile osf_file(input_osf_filename);
    EXPECT_TRUE(osf_file.good());

    FileInfo file_info(osf_file);
    EXPECT_FALSE(file_info.isFramed());

    // Frame Chunker
    rechunk(input_osf_filename, output_osf_filename);

    EXPECT_TRUE(path_exists(output_osf_filename));

    OsfFile out_osf_file(output_osf_filename);
    FileInfo out_file_info(out_osf_file);
    EXPECT_TRUE(out_file_info.isFramed());

    // Check session and sensors info
    EXPECT_EQ(file_info.id(), out_file_info.id());
    EXPECT_EQ(OSF_FRAME_MODE_OSF_32, out_file_info.lidar_frame_mode());
    compare_sensors_meta(file_info, out_file_info);

    Reader out_reader(out_osf_file);

    // Check that we have imu and gps in the result
    auto gps_msgs = out_reader.messages({OSF::MessageType::GPS_WAYPOINT});
    EXPECT_EQ(10, std::distance(gps_msgs.begin(), gps_msgs.end()));

    auto imu_msgs = out_reader.messages({OSF::MessageType::IMU});
    EXPECT_EQ(1000, std::distance(imu_msgs.begin(), imu_msgs.end()));

    FileStatsFramed fst = calc_framed_file_stats(output_osf_filename);

    // Params of the good frame chunker outcome
    EXPECT_LT(fst.dt_var, 0.0001);
    EXPECT_GT(fst.full_frames_cnt, 90);
    EXPECT_EQ(fst.dt_past_cnt, 0);
    EXPECT_EQ(fst.start_end_mismatch_cnt, 0);
    EXPECT_EQ(fst.overlapped_frames_cnt, 0);
}

TEST_F(FrameChunkerTest, ChunkOSF_drive2577) {
    TEST_DATA_SKIP();

    std::string input_osf_filename =
        test_data_dir() + "/frame-chunker/drive-2577/0000.osf";

    OsfFile osf_file(input_osf_filename);
    EXPECT_TRUE(osf_file.good());

    FileInfo file_info(osf_file);
    EXPECT_FALSE(file_info.isFramed());

    std::string output_osf_filename = tmp_file("0000_chunked.osf");

    rechunk(input_osf_filename, output_osf_filename);

    EXPECT_TRUE(path_exists(output_osf_filename));

    OsfFile out_osf_file(output_osf_filename);
    FileInfo out_file_info(out_osf_file);
    EXPECT_TRUE(out_file_info.isFramed());

    // Check session and sensors info
    EXPECT_EQ(file_info.id(), out_file_info.id());
    EXPECT_EQ(OSF_FRAME_MODE_OSF_32, out_file_info.lidar_frame_mode());
    compare_sensors_meta(file_info, out_file_info);

    FileStatsFramed fst = calc_framed_file_stats(output_osf_filename);

    // FOR THE REFERENCE:
    // Previous frame chunker implementation had this results (which wanted
    // to improve upon):
    //
    // FileStatsFramed [ dt_mean = 0.0838735, dt_var = 0.00538443,
    // dt_past_cnt = 8, dt_jumps_cnt = 0, ls_total_cnt = 282,
    // ls_with_traj_cnt = 258, full_frames_cnt = 58, start_end_mismatch_cnt = 9,
    // overlapped_frames_cnt = 9, session_start_ts = 1586513698077991000,
    // session_end_ts = 1586513707807313000, full_frames_ratio = 0.495726,
    // frames_cnt = 117 ]

    // Params of the good frame chunker outcome for this drive-2577
    EXPECT_LT(fst.dt_var, 0.0001);
    EXPECT_GT(fst.full_frames_cnt, 80);
    EXPECT_EQ(fst.dt_past_cnt, 0);
    EXPECT_EQ(fst.start_end_mismatch_cnt, 0);
    EXPECT_EQ(fst.overlapped_frames_cnt, 0);
}

TEST_F(FrameChunkerTest, ChunkOSF_drive2588) {
    TEST_DATA_SKIP();

    std::string input_osf_filename =
        test_data_dir() + "/frame-chunker/drive-2588/0000.osf";

    OsfFile osf_file(input_osf_filename);
    EXPECT_TRUE(osf_file.good());

    FileInfo file_info(osf_file);
    EXPECT_FALSE(file_info.isFramed());

    std::string output_osf_filename = tmp_file("0000_chunked.osf");

    rechunk(input_osf_filename, output_osf_filename);

    EXPECT_TRUE(path_exists(output_osf_filename));

    OsfFile out_osf_file(output_osf_filename);
    FileInfo out_file_info(out_osf_file);
    EXPECT_TRUE(out_file_info.isFramed());

    // Check session and sensors info
    EXPECT_EQ(file_info.id(), out_file_info.id());
    EXPECT_EQ(OSF_FRAME_MODE_OSF_32, out_file_info.lidar_frame_mode());
    compare_sensors_meta(file_info, out_file_info);

    FileStatsFramed fst = calc_framed_file_stats(output_osf_filename);

    // FOR THE REFERENCE:
    // Previous frame chunker implementation had this results (which wanted
    // to improve upon):
    //
    // FileStatsFramed [ dt_mean = 0.0939498, dt_var = 0.00206194,
    // dt_past_cnt = 5, dt_jumps_cnt = 0, ls_total_cnt = 265,
    // ls_with_traj_cnt = 253, full_frames_cnt = 72, start_end_mismatch_cnt = 8,
    // overlapped_frames_cnt = 5, session_start_ts = 1586912660037880000,
    // session_end_ts = 1586912669118702000, full_frames_ratio = 0.742268,
    // frames_cnt = 97 ]

    // Params of the good frame chunker outcome for this drive-2588
    EXPECT_LT(fst.dt_var, 0.0001);
    EXPECT_GT(fst.full_frames_cnt, 77);
    EXPECT_EQ(fst.dt_past_cnt, 0);
    EXPECT_EQ(fst.start_end_mismatch_cnt, 0);
    EXPECT_EQ(fst.overlapped_frames_cnt, 0);
}

TEST_F(FrameChunkerTest, ChunkOSF_drive2625) {
    TEST_DATA_SKIP();

    std::string input_osf_filename =
        test_data_dir() + "/frame-chunker/drive-2625/0000.osf";

    OsfFile osf_file(input_osf_filename);
    EXPECT_TRUE(osf_file.good());

    FileInfo file_info(osf_file);
    EXPECT_FALSE(file_info.isFramed());

    std::string output_osf_filename = tmp_file("0000_chunked.osf");

    rechunk(input_osf_filename, output_osf_filename);

    EXPECT_TRUE(path_exists(output_osf_filename));

    OsfFile out_osf_file(output_osf_filename);
    FileInfo out_file_info(out_osf_file);
    EXPECT_TRUE(out_file_info.isFramed());

    // Check session and sensors info
    EXPECT_EQ(file_info.id(), out_file_info.id());
    EXPECT_EQ(OSF_FRAME_MODE_OSF_32, out_file_info.lidar_frame_mode());
    compare_sensors_meta(file_info, out_file_info);

    FileStatsFramed fst = calc_framed_file_stats(output_osf_filename);

    // FOR THE REFERENCE:
    // Previous frame chunker implementation had this results (which wanted
    // to improve upon):
    //
    // FileStatsFramed [ dt_mean = 0.0877152, dt_var = 0.00496408,
    // dt_past_cnt = 6, dt_jumps_cnt = 0, ls_total_cnt = 259,
    // ls_with_traj_cnt = 234, full_frames_cnt = 55, start_end_mismatch_cnt = 8,
    // overlapped_frames_cnt = 7, session_start_ts = 1587260020322781000,
    // session_end_ts = 1587260029357450000, full_frames_ratio = 0.528846,
    // frames_cnt = 104 ]

    // Params of the good frame chunker outcome for this drive-2625
    EXPECT_LT(fst.dt_var, 0.0001);
    EXPECT_GT(fst.full_frames_cnt, 72);
    EXPECT_EQ(fst.dt_past_cnt, 0);
    EXPECT_EQ(fst.start_end_mismatch_cnt, 0);
    EXPECT_EQ(fst.overlapped_frames_cnt, 0);
}

// frameChunker with recode option to the default OSF_32
TEST_F(FrameChunkerTest, ChunkOSF_drive3532_recoding) {
    TEST_DATA_SKIP();

    std::string input_osf_filename =
        test_data_dir() + "/fg-drives/3532-0.4.5/osf/0000_72.osf";

    OsfFile osf_file(input_osf_filename);
    EXPECT_TRUE(osf_file.good());

    FileInfo file_info(osf_file);
    EXPECT_FALSE(file_info.isFramed());

    EXPECT_EQ(file_info.lidar_frame_mode(),
              OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_72);

    // Get first LidarScan from input file (need to use SortedWindow because
    // order is not guaranteed)
    Reader in_reader(osf_file, {MessageType::LIDAR_SCAN});
    SortedWindow in_ls_queue(in_reader);
    auto in_msg = in_ls_queue.pop();
    auto in_ls = in_msg.as_lidar_scan();

    std::string output_osf_filename = tmp_file("0000_72_chunked.osf");

    rechunk(input_osf_filename, output_osf_filename);

    EXPECT_TRUE(path_exists(output_osf_filename));

    OsfFile out_osf_file(output_osf_filename);
    FileInfo out_file_info(out_osf_file);
    EXPECT_TRUE(out_file_info.isFramed());

    // Check session and sensors info
    EXPECT_EQ(file_info.id(), out_file_info.id());
    EXPECT_EQ(OSF_FRAME_MODE_OSF_32, out_file_info.lidar_frame_mode());
    compare_sensors_meta(file_info, out_file_info);

    // Get first LidarScan from output, order is guaranteed becvause it's after
    // frameChunker
    Reader out_reader(out_osf_file, {MessageType::LIDAR_SCAN});
    auto out_msg = *out_reader.messages().begin();
    auto out_ls = out_msg.as_lidar_scan();

    // Ensure that it's the same lidar scan (ts == ts && id == id)
    EXPECT_GT(in_msg.ts(), OSF::ts_t(0));
    EXPECT_EQ(in_msg.ts(), out_msg.ts());
    EXPECT_EQ(in_msg.id(), out_msg.id());

    LidarScanStat in_ls_stat(*in_ls), out_ls_stat(*out_ls);

    // Check that in_ls has all channels not zero
    EXPECT_GT(in_ls_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_GT(in_ls_stat.mean(LidarScan::LidarScanIndex::INTENSITY), 0);
    EXPECT_GT(in_ls_stat.mean(LidarScan::LidarScanIndex::NOISE), 0);
    EXPECT_GT(in_ls_stat.mean(LidarScan::LidarScanIndex::REFLECTIVITY), 0);

    // Check that out_ls zeroed out reflectivity
    EXPECT_GT(out_ls_stat.mean(LidarScan::LidarScanIndex::RANGE), 0);
    EXPECT_GT(out_ls_stat.mean(LidarScan::LidarScanIndex::INTENSITY), 0);
    EXPECT_GT(out_ls_stat.mean(LidarScan::LidarScanIndex::NOISE), 0);
    EXPECT_EQ(out_ls_stat.mean(LidarScan::LidarScanIndex::REFLECTIVITY), 0);

    // Check that INTENSITY is clamped
    EXPECT_GT(in_ls_stat.max(LidarScan::LidarScanIndex::INTENSITY), 255);
    EXPECT_LE(out_ls_stat.max(LidarScan::LidarScanIndex::INTENSITY), 255);
}

TEST_F(FrameChunkerTest, CarTrajectory_drive3532) {
    TEST_DATA_SKIP();

    // File contains car trajectory and per sensor trajectory
    std::string osf_filename =
        test_data_dir() + "/fg-drives/3532-0.4.5/osf/0000_72_with_car_traj.osf";

    OsfFile osf_file(osf_filename);
    EXPECT_TRUE(osf_file.good());

    // Read only lidar_scan and trajectory messages
    OSF::Reader reader(
        osf_file, {OSF::MessageType::LIDAR_SCAN, OSF::MessageType::TRAJECTORY});

    OSF::SortedWindow ls_msgs(reader, 200, {OSF::MessageType::LIDAR_SCAN});

    OSF::SortedWindow traj_msgs(reader, 200, {OSF::MessageType::TRAJECTORY});

    // Car trajectories with evaluate
    OSF::TrajectoryReader traj_reader(reader);

    int traj_with_gt = 0;

    // Iterate over all lidar scans, evaluate lidar scan trajectory from car
    // trajectory and compare it with existing per sensor trajectory (aka
    // ground truth)
    while (!ls_msgs.empty()) {
        auto ls_msg = ls_msgs.pop();
        auto ls = ls_msg.as_lidar_scan();
        auto sensor = reader.file_info().sensors().at(ls_msg.id());
        auto ls_traj = traj_reader.evaluate(ls->ts, sensor->meta.extrinsic);
        if (ls_traj) {
            // try to find gt traj and compare with generated one
            while (!traj_msgs.empty() && traj_msgs.peek().ts() <= ls_msg.ts()) {
                auto traj_msg = traj_msgs.pop();
                if (traj_msg.id() == ls_msg.id() &&
                    traj_msg.ts() == ls_msg.ts()) {
                    auto gt_traj = traj_msg.as_trajectory();
                    double rot_err, trans_err;
                    std::tie(rot_err, trans_err) =
                        calc_traj_error(*gt_traj, *ls_traj);
                    EXPECT_NEAR(rot_err, 0, 2e-6);
                    EXPECT_NEAR(trans_err, 0, 2e-6);
                    ++traj_with_gt;
                }
            }
        }
    }

    // Check that we have some amounts of trajectories paired and compared
    // with ground truth trajectory
    EXPECT_GT(traj_with_gt, 220);

}

}  // namespace
}  // namespace OSF
}  // namespace ouster

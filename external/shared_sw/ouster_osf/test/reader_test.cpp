#include "ouster/osf/reader.h"

#include <gtest/gtest.h>

#include "osf_test.h"
#include "ouster/osf/file.h"
#include "ouster/osf/lidar_scan_stat.h"

namespace ouster {
namespace OSF {
namespace {

class ReaderTest : public OsfTestWithData {};

TEST_F(ReaderTest, IteratorReadMessagesGenerated) {
    TEST_DATA_SKIP();

    OsfFile osf_file(test_data_dir() + "/lib-osf/fake_data_v12.osf");

    EXPECT_TRUE(osf_file.valid());

    Reader reader(osf_file);

    auto messages = reader.messages();
    EXPECT_EQ(4, std::distance(messages.begin(), messages.end()));

    int ls_cnt = 0;
    int tr_cnt = 0;
    int im_cnt = 0;
    int gp_cnt = 0;
    std::unique_ptr<LidarScan> ls;
    std::unique_ptr<Trajectory> traj;
    std::unique_ptr<Imu> imu;
    std::unique_ptr<Gps> gps;
    for (auto& m : reader.messages()) {
        switch (m.type()) {
            case MessageType::LIDAR_SCAN:
                ls = m.as_lidar_scan();
                EXPECT_EQ(1024, ls->w);
                EXPECT_EQ(64, ls->h);
                ++ls_cnt;
                break;
            case MessageType::TRAJECTORY:
                traj = m.as_trajectory();
                EXPECT_EQ(1024, traj->size());
                ++tr_cnt;
                break;
            case MessageType::IMU:
                imu = m.as_imu();
                EXPECT_EQ(3, imu->angular_vel.size());
                EXPECT_EQ(3, imu->linear_accel.size());
                EXPECT_EQ(3, imu->ts.size());
                ++im_cnt;
                break;
            case MessageType::GPS_WAYPOINT:
                gps = m.as_gps();
                EXPECT_DOUBLE_EQ(12.587171877060079, gps->latitude);
                ++gp_cnt;
                break;
            default:
                std::cout << " ---- OTHER MSG TYPE\n";
                break;
        }
    }

    EXPECT_EQ(1, ls_cnt);
    EXPECT_EQ(1, tr_cnt);
    EXPECT_EQ(1, im_cnt);
    EXPECT_EQ(1, gp_cnt);

    // Test algorithms with our iterator
    bool sorted = std::is_sorted(messages.begin(), messages.end(),
                                 [](const MessageRef& a, const MessageRef& b) {
                                     return a.ts().count() < b.ts().count();
                                 });
    EXPECT_TRUE(sorted);
}

TEST_F(ReaderTest, IteratorReadMessagesMore) {
    TEST_DATA_SKIP();

    OsfFile osf_file(test_data_dir() +
                     "/lib-osf/ml_data_743_v12_fix/0000fc.osf");

    // TODO: Extend tests to more cases ...
    // OsfFile osf_file(test_data_dir() +
    //                  "/lib-osf/small_with_gps_v12/small_with_gps_v12.osf");

    // OsfFile osf_file(test_data_dir() + "/lib-osf/fake_data_v12.osf");
    // OsfFile osf_file(test_data_dir() +
    //                  "/lib-osf/small_with_gps_v12/small_with_gps_v12.osf");
    // OsfFile osf_file(test_data_dir() + "/lib-osf/ml_data_743_v12/0000.osf");
    // OsfFile osf_file(test_data_dir() +
    // "/lib-osf/ml_data_743_v12/0000fc.osf");
    // OsfFile osf_file(test_data_dir() +
    // "/lib-osf/ml_data_743_v12_fix/0000.osf");

    // OsfFile osf_file(test_data_dir() + "/lib-osf/fake_data_v12.osf");

    EXPECT_TRUE(osf_file.valid());

    Reader reader(osf_file);

    auto messages = reader.messages();
    EXPECT_EQ(194, std::distance(messages.begin(), messages.end()));

    bool sorted = std::is_sorted(messages.begin(), messages.end(),
                                 [](const MessageRef& a, const MessageRef& b) {
                                     return a.ts().count() < b.ts().count();
                                 });
    // It's not guaranteed to be sorted, this is just true for the current file
    EXPECT_TRUE(sorted);

    const auto messagesc = reader.messages();
    int cnt = 0;
    for (const auto m : messagesc) {
        ++cnt;
        (void)m;  // do nothing with m ...
    }

    // Compare two distances to ensure that at our iterators works.
    EXPECT_EQ(cnt, std::distance(messages.begin(), messages.end()));
}

TEST_F(ReaderTest, IteratorReadMessagesFiltered) {
    TEST_DATA_SKIP();

    OsfFile osf_file(test_data_dir() +
                     "/lib-osf/small_with_gps_v12/small_with_gps_v12.osf");

    EXPECT_TRUE(osf_file.valid());

    {
        // Read all messages
        Reader reader(osf_file);
        auto messages = reader.messages();
        EXPECT_EQ(1203, std::distance(messages.begin(), messages.end()));
    }

    {
        // Read nothing (skip all essentially)
        Reader reader(osf_file, {OSF::MessageType::NONE});
        auto messages = reader.messages();
        auto messages_cnt = std::distance(messages.begin(), messages.end());
        EXPECT_EQ(0, messages_cnt);

        // The same as above but with query_filters
        Reader reader_all(osf_file);
        auto messages_q = reader_all.messages({OSF::MessageType::NONE});
        auto messages_q_cnt =
            std::distance(messages_q.begin(), messages_q.end());
        EXPECT_EQ(messages_q_cnt, messages_cnt);
    }

    {
        // Read only gps messages
        Reader reader(osf_file, {OSF::MessageType::GPS_WAYPOINT});
        for (const auto& m : reader.messages()) {
            EXPECT_EQ(OSF::MessageType::GPS_WAYPOINT, m.type());
        }

        // The same as above but with query_filters
        Reader reader_all(osf_file);
        for (const auto& m :
             reader_all.messages({OSF::MessageType::GPS_WAYPOINT})) {
            EXPECT_EQ(OSF::MessageType::GPS_WAYPOINT, m.type());
        }
    }

    {
        // Read lidar_scan messages only
        Reader reader(osf_file, {OSF::MessageType::LIDAR_SCAN});
        auto messages = reader.messages();
        EXPECT_EQ(100, std::distance(messages.begin(), messages.end()));
    }

    {
        // Read lidar_scan + traj messages
        Reader reader(osf_file, {OSF::MessageType::LIDAR_SCAN,
                                 OSF::MessageType::TRAJECTORY});
        auto messages = reader.messages();
        EXPECT_EQ(193, std::distance(messages.begin(), messages.end()));

        // and with combination of reader + query_filters
        Reader reader_all(osf_file, {OSF::MessageType::LIDAR_SCAN,
                                     OSF::MessageType::TRAJECTORY,
                                     OSF::MessageType::GPS_WAYPOINT});
        auto messages_q = reader_all.messages(
            {OSF::MessageType::LIDAR_SCAN, OSF::MessageType::TRAJECTORY});
        // Result should be the sum of just lidar_scan + trajectory messages
        EXPECT_EQ(193, std::distance(messages_q.begin(), messages_q.end()));
    }

    {
        // Try to read it by chunks with specified filters
        Reader reader(osf_file, {OSF::MessageType::LIDAR_SCAN,
                                 OSF::MessageType::TRAJECTORY});
        const auto f_keys = reader.file_info().frames_keys();

        // 5 chunks in a file
        EXPECT_EQ(5, f_keys.size());
        // Get messages of the first chunk
        auto messages = reader.messages(f_keys[0]);
        EXPECT_EQ(34, std::distance(messages.begin(), messages.end()));

        // it's not a framed OSF, so there is no chunk with index = 0
        auto messages0 = reader.messages(0);
        EXPECT_EQ(0, std::distance(messages0.begin(), messages0.end()));
    }

    // Sliced read equal to the whole read
    {
        Reader reader_ls_traj(osf_file, {OSF::MessageType::LIDAR_SCAN,
                                         OSF::MessageType::TRAJECTORY});

        Reader reader_ls(osf_file, {OSF::MessageType::LIDAR_SCAN});

        Reader reader_traj(osf_file, {OSF::MessageType::TRAJECTORY});

        const size_t ls_traj_cnt = std::distance(
            reader_ls_traj.messages().begin(), reader_ls_traj.messages().end());

        const size_t ls_cnt = std::distance(reader_ls.messages().begin(),
                                            reader_ls.messages().end());

        const size_t traj_cnt = std::distance(reader_traj.messages().begin(),
                                              reader_traj.messages().end());

        EXPECT_EQ(ls_traj_cnt, ls_cnt + traj_cnt);
    }
}

TEST_F(ReaderTest, SortedWindowReader) {
    TEST_DATA_SKIP();

    OsfFile osf_file(test_data_dir() +
                     "/lib-osf/small_with_gps_v12/small_with_gps_v12.osf");

    // Basic sorted reading
    {
        Reader reader(osf_file, {OSF::MessageType::LIDAR_SCAN,
                                 OSF::MessageType::TRAJECTORY});
        SortedWindow sorted_reader(reader);
        EXPECT_LT(ts_t(9 * 1000l * 1000l * 1000l), sorted_reader.duration());

        const OSF::MessageRef& m = sorted_reader.peek();
        EXPECT_EQ(ts_t(1567783229425152000l), m.ts());

        int sorted_cnt = 0;
        while (!sorted_reader.empty()) {
            sorted_reader.pop();
            sorted_cnt++;
        }
        EXPECT_EQ(193, sorted_cnt);
        EXPECT_EQ(0, sorted_reader.skipped());

        size_t all_cnt =
            std::distance(reader.messages().begin(), reader.messages().end());
        EXPECT_EQ(193, all_cnt);
    }

    // Sorted reading and consistent counts
    {
        Reader reader(osf_file);
        SortedWindow sorted_reader(reader, 1);
        EXPECT_EQ(ts_t(0), sorted_reader.duration());

        const OSF::MessageRef& m = sorted_reader.peek();
        EXPECT_EQ(ts_t(1567783229425152000l), m.ts());

        // Count sorted_reader messages
        int sorted_cnt = 0;
        while (!sorted_reader.empty()) {
            sorted_reader.pop();
            sorted_cnt++;
        }

        // Finished reading so window duration should be ZERO
        EXPECT_EQ(ts_t(0), sorted_reader.duration());

        // We should have some skipped messages
        EXPECT_LT(0, sorted_reader.skipped());

        // All messages count directly from the reader
        size_t all_cnt =
            std::distance(reader.messages().begin(), reader.messages().end());

        // All messages is equal
        EXPECT_EQ(all_cnt, sorted_cnt + sorted_reader.skipped());
    }
}

TEST_F(ReaderTest, ReadSensorsInfo) {
    TEST_DATA_SKIP();

    OsfFile osf_file(test_data_dir() +
                     "/lib-osf/ml_data_743_v12_fix/0000fc.osf");

    EXPECT_TRUE(osf_file.valid());

    Reader reader(osf_file);

    // Iterate through var
    auto sensors = reader.file_info().sensors();
    EXPECT_EQ(1, sensors.size());
    for (auto s : sensors) {
        EXPECT_EQ(s.first, s.second->id);
    }

    EXPECT_EQ(1, reader.file_info().sensors().size());

    // Iterate on rvalue
    for (auto s : reader.file_info().sensors()) {
        EXPECT_EQ(s.first, s.second->id);
    }

    EXPECT_EQ("sensor1", reader.file_info().sensors().at(1)->meta.name);
}

TEST_F(ReaderTest, SortedWindowFilteredReader) {
    TEST_DATA_SKIP();

    OsfFile osf_file(test_data_dir() +
                     "/lib-osf/small_with_gps_v12/small_with_gps_v12.osf");

    // Basic sorted reading
    {
        Reader reader(osf_file);
        SortedWindow sorted_reader(
            reader, 200,
            {OSF::MessageType::LIDAR_SCAN, OSF::MessageType::TRAJECTORY});
        EXPECT_LT(ts_t(9 * 1000l * 1000l * 1000l), sorted_reader.duration());

        const OSF::MessageRef& m = sorted_reader.peek();
        EXPECT_EQ(ts_t(1567783229425152000l), m.ts());

        int sorted_cnt = 0;
        while (!sorted_reader.empty()) {
            sorted_reader.pop();
            sorted_cnt++;
        }
        EXPECT_EQ(193, sorted_cnt);
        EXPECT_EQ(0, sorted_reader.skipped());

        size_t all_cnt =
            std::distance(reader.messages().begin(), reader.messages().end());
        EXPECT_EQ(1203, all_cnt);
    }
}

}  // namespace
}  // namespace OSF
}  // namespace ouster

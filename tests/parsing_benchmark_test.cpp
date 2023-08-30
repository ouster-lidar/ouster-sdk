/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <numeric>
#include <random>

#include "ouster/lidar_scan.h"
#include "ouster/pcap.h"
#include "ouster/types.h"
#include "util.h"

using ouster::sensor::ChanField;

using str_pair = std::pair<std::string, std::string>;
class ParsingBenchmarkTestFixture : public ::testing::TestWithParam<str_pair> {
};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    ParsingBenchmarkTests,
    ParsingBenchmarkTestFixture,
    ::testing::Values(
        str_pair{"OS-0-128-U1_v2.3.0_1024x10.pcap",
                 "OS-0-128-U1_v2.3.0_1024x10.json"},
        str_pair{"OS-0-32-U1_v2.2.0_1024x10.pcap",
                 "OS-0-32-U1_v2.2.0_1024x10.json"},
        str_pair{"OS-1-128_767798045_1024x10_20230712_120049.pcap",
                 "OS-1-128_767798045_1024x10_20230712_120049.json"},
        str_pair{"OS-1-128_v2.3.0_1024x10_lb_n3.pcap",
                 "OS-1-128_v2.3.0_1024x10.json"},
        str_pair{"OS-2-128-U1_v2.3.0_1024x10.pcap",
                 "OS-2-128-U1_v2.3.0_1024x10.json"})
);
// clang-format on

namespace ouster {
namespace sensor_utils {

struct parse_col {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField f,
                    const sensor::packet_format& pf,
                    const uint8_t* packet_buf) const {
        for (int icol = 0; icol < pf.columns_per_packet; icol++) {
            const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
            const uint16_t m_id = pf.col_measurement_id(col_buf);
            pf.col_field(col_buf, f, field.col(m_id).data(), field.cols());
        }
    }
};

template <int BlockDim>
struct parse_block {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField f,
                    const sensor::packet_format& pf,
                    const uint8_t* packet_buf) const {
        pf.block_field<T, BlockDim>(field, f, packet_buf);
    }
};

using HashMap = std::map<ChanField, size_t>;

// picked up from
// https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
struct matrix_hash {
    template <typename T>
    void operator()(Eigen::Ref<ouster::img_t<T>> matrix, ChanField f,
                    HashMap& map) const {
        size_t seed = 0;
        for (int i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^=
                std::hash<T>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        map[f] = seed;
    }
};

struct set_zero {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> matrix, ChanField) const {
        matrix.setZero();
    }
};

TEST_P(ParsingBenchmarkTestFixture, ParseBlockCorrectnessTest) {
    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info =
        ouster::sensor::metadata_from_json(data_dir + "/" + test_params.second);
    auto pf = ouster::sensor::packet_format(info);
    PcapReader pcap(data_dir + "/" + test_params.first);

    auto ls =
        LidarScan(info.format.columns_per_frame, info.format.pixels_per_column,
                  info.format.udp_profile_lidar);

    auto parse_and_hash = [&](auto parser) -> HashMap {
        // reset
        pcap.seek(0);
        impl::foreach_field(ls, set_zero{});
        // parse
        while (pcap.next_packet()) {
            if (pcap.current_info().dst_port == 7502)
                impl::foreach_field(ls, parser, pf, pcap.current_data());
        }

        HashMap map;
        impl::foreach_field(ls, matrix_hash{}, map);
        return map;
    };

    HashMap hashes = parse_and_hash(parse_col{});
    // sanity checks
    ASSERT_TRUE(hashes.size() > 0);
    for (auto pair : hashes) {
        ASSERT_TRUE(pair.second > 0);
    }

    EXPECT_TRUE(hashes == parse_and_hash(parse_block<16>{}));
    EXPECT_TRUE(hashes == parse_and_hash(parse_block<8>{}));
    EXPECT_TRUE(hashes == parse_and_hash(parse_block<4>{}));
}

TEST_P(ParsingBenchmarkTestFixture, ScanBatcherBenchTest) {
    std::map<std::string, std::string> styles = term_styles();
    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();

    auto info =
        ouster::sensor::metadata_from_json(data_dir + "/" + test_params.second);
    auto pf = ouster::sensor::packet_format(info);

    std::cout << styles["yellow"] << styles["bold"]
              << "CHECKING PARSING PERFORMANCE WITH: " << test_params.first
              << styles["reset"] << std::endl;

    // load up packets into memory to avoid I/O latency messing with benchmarks
    std::vector<std::vector<uint8_t>> packets;
    {
        PcapReader pcap(data_dir + "/" + test_params.first);
        while (pcap.next_packet()) {
            if (pcap.current_info().dst_port == 7502) {
                std::vector<uint8_t> packet(pcap.current_length());
                std::memcpy(packet.data(), pcap.current_data(),
                            pcap.current_length());
                packets.push_back(std::move(packet));
            }
        }
    }

    auto ls =
        LidarScan(info.format.columns_per_frame, info.format.pixels_per_column,
                  info.format.udp_profile_lidar);

    constexpr int N_SCANS = 1000;
    constexpr int MOVING_AVG_WINDOW = 100;
    using MovingAverage64 = MovingAverage<int64_t, int64_t, MOVING_AVG_WINDOW>;
    static std::map<std::string, MovingAverage64> mv;

    Timer t;
    std::stringstream ss;
    int output_ctr = 0;

    auto parse = [&](auto parser, std::string name) {
        t.start();
        for (const auto& packet : packets)
            impl::foreach_field(ls, parser, pf, packet.data());
        t.stop();
        mv[name](t.elapsed_microseconds());
    };

    std::vector<std::function<void()>> all_methods = {
        [&]() { parse(parse_col{}, "col"); },
        [&]() { parse(parse_block<4>{}, "block4"); },
        [&]() { parse(parse_block<8>{}, "block8"); },
        [&]() { parse(parse_block<16>{}, "block16"); }};

    std::default_random_engine g;
    std::vector<int> ids(all_methods.size());
    std::iota(std::begin(ids), std::end(ids), 0);

    for (auto i = 0; i < N_SCANS; ++i) {
        std::shuffle(std::begin(ids), std::end(ids), g);
        for (auto m : ids) all_methods[m]();

        if (++output_ctr % MOVING_AVG_WINDOW == 0) {
            ss.str("");
            ss << styles["bold"] << "runs: " << styles["reset"]
               << styles["magenta"] << std::setw(3) << output_ctr << " "
               << styles["reset"];
            ss << styles["bold"] << "col[time]: " << styles["reset"]
               << styles["cyan"] << std::setw(4) << lround(mv["col"]) << "μs, "
               << styles["reset"];

            auto best_time = std::min_element(
                mv.begin(), mv.end(), [](const auto& a, const auto& b) -> bool {
                    return a.second < b.second;
                });

            ss << styles["bold"] << "best " << best_time->first
               << "[time]:" << styles["reset"] << styles["cyan"] << std::setw(4)
               << lround(best_time->second) << "μs, " << styles["reset"];

            for (std::string name : {"col", "block4", "block8", "block16"}) {
                auto speedup = lround(100.0f * mv["col"] / mv[name]);
                auto color_modifier =
                    name == best_time->first
                        ? styles["blue"]
                        : (speedup >= 100 ? styles["green"] : styles["red"]);
                ss << styles["bold"] << name << ": " << color_modifier
                   << std::setw(4) << speedup << "%, " << styles["reset"];
            }
            std::cout << ss.str() << std::endl;
        }
    }
}

}  // namespace sensor_utils
}  // namespace ouster

#include <gtest/gtest.h>
#include <sys/stat.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "../util.h"
#include "benchmark_utils.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/packet.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/xyzlut.h"
#include "ouster/pcap/pcap_packet_source.h"

using ouster::sdk::core::cartesian;
using ouster::sdk::core::destagger;
using ouster::sdk::core::FrameBatcher;
using ouster::sdk::core::img_t;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::LidarPacket;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::PacketType;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::pcap::PcapPacketSource;

TEST(CoreBenchmark, Cartesian) {
    const int WIDTH = 1024;
    const int HEIGHT = 128;
    const int N_ITERS = benchmark_iterations(1000);

    auto gen = seeded_rng();
    auto lut = make_random_lut(HEIGHT, WIDTH, gen);

    const int POOL_SIZE = std::min(N_ITERS, 16);
    std::vector<img_t<uint32_t>> pool(POOL_SIZE, img_t<uint32_t>(WIDTH, HEIGHT));
    for (auto& range : pool) randomize_range(range, gen);

    Timer t;
    t.start();
    for (int i = 0; i < N_ITERS; ++i) {
        auto points = cartesian(pool[i % POOL_SIZE], lut);
        ASSERT_EQ(points.rows(), WIDTH * HEIGHT);
    }
    t.stop();

    report_benchmark(
        "cartesian(range, lut) [" + std::to_string(WIDTH) + "x" + std::to_string(HEIGHT) + "]",
        t.elapsed_nanoseconds(), N_ITERS);
}

TEST(CoreBenchmark, Destagger) {
    const int WIDTH = 1024;
    const int HEIGHT = 128;
    const int N_ITERS = benchmark_iterations(1000);

    auto gen = seeded_rng();

    std::uniform_int_distribution<int> shift_dist(-30, 30);
    std::vector<int> pixel_shift_by_row(HEIGHT);
    for (auto& s : pixel_shift_by_row) s = shift_dist(gen);

    const int POOL_SIZE = std::min(N_ITERS, 16);
    std::vector<img_t<uint32_t>> pool(POOL_SIZE, img_t<uint32_t>(HEIGHT, WIDTH));
    for (auto& range : pool) randomize_range(range, gen);

    Timer t;
    t.start();
    for (int i = 0; i < N_ITERS; ++i) {
        auto out = destagger<uint32_t>(pool[i % POOL_SIZE], pixel_shift_by_row);
        ASSERT_EQ(out.rows(), HEIGHT);
    }
    t.stop();

    report_benchmark(
        "destagger<uint32_t> [" + std::to_string(WIDTH) + "x" + std::to_string(HEIGHT) + "]",
        t.elapsed_nanoseconds(), N_ITERS);
}

TEST(CoreBenchmark, FrameBatcher) {
    const auto data_dir = getenvs("DATA_DIR");
    if (data_dir.empty()) {
        GTEST_SKIP() << "DATA_DIR not set — skipping FrameBatcher benchmark";
    }

    const std::vector<std::string> pcap_files = {
        "OS-0-32-U0_v2.3.0_1024x10_20220420_113012.pcap",
        "0ZGB1L6N0TP3R184_OS-0-32-U2_v2.2.0_1024x10_20220125_160640-000.pcap",
        "OS-1-128_v2.3.0_1024x10_20220419_161100.pcap",
        "OS-1-128_v2.5.0-rc.1_1024x10_20230328_063807.pcap",
        "OS-2-128_v2.3.0_1024x10_20220419_164406.pcap",
    };

    const int N_ITERS = benchmark_iterations(1000);

    for (const auto& pcap_filename : pcap_files) {
        const auto pcap_path = data_dir + "/" + pcap_filename;

        struct stat st {};
        if (stat(pcap_path.c_str(), &st) != 0) {
            std::cout << "Fixture not found, skipping: " << pcap_path << "\n";
            continue;
        }

        PcapPacketSource source(pcap_path);
        if (source.sensor_info().empty()) {
            GTEST_FAIL() << "No metadata found for: " << pcap_filename;
        }
        const SensorInfo& info = *source.sensor_info()[0];
        auto pf_shared = std::make_shared<PacketFormat>(info);

        // Load one frame worth of lidar packets from the pcap to use as a
        // template.
        std::vector<LidarPacket> template_packets;
        {
            FrameBatcher probe(info);
            LidarFrame probe_frame(info);
            for (auto it = source.begin(); it != source.end(); ++it) {
                const auto& [sensor_idx, pkt_ptr] = *it;
                if (pkt_ptr->type() != PacketType::Lidar) continue;
                template_packets.push_back(pkt_ptr->as<LidarPacket>());
                if (probe(template_packets.back(), probe_frame)) break;
            }
        }

        // Generate N_ITERS frames worth of packets
        std::vector<LidarPacket> all_packets;
        all_packets.reserve(template_packets.size() * N_ITERS);
        for (int i = 0; i < N_ITERS; ++i) {
            for (auto pkt : template_packets) {
                pf_shared->set_frame_id(pkt.buf.data(), static_cast<uint32_t>(i));
                all_packets.push_back(std::move(pkt));
            }
        }

        LidarFrame output_frames[2] = {LidarFrame(info), LidarFrame(info)};

        FrameBatcher batcher(info);
        int num_batched = 0;
        Timer t;
        t.start();
        for (const auto& pkt : all_packets) {
            if (batcher(pkt, output_frames[num_batched % 2])) {
                ++num_batched;
            }
        }
        t.stop();

        EXPECT_EQ(num_batched, N_ITERS);
        report_benchmark("FrameBatcher [" + pcap_filename + "]", t.elapsed_nanoseconds(), N_ITERS);
    }
}

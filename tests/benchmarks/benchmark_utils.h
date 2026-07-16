#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <ostream>
#include <random>
#include <string>
#include <vector>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/xyzlut.h"

/**
 * Returns true when OUSTER_PERFORMANCE_BENCHMARK is set.
 */
inline bool enable_benchmark_tests() {
    static const std::vector<std::string> yes = {"1", "y", "yes", "on"};
    const char* env = std::getenv("OUSTER_PERFORMANCE_BENCHMARK");
    if (!env) return false;
    std::string lower;
    for (const char* p = env; *p != '\0'; ++p) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(*p))));
    }
    return std::find(yes.begin(), yes.end(), lower) != yes.end();
}

/**
 * Returns the number of iterations to run.
 *
 * Priority order:
 *   1. OUSTER_BENCHMARK_ITERATIONS env var (override)
 *   2. default_full when OUSTER_PERFORMANCE_BENCHMARK is set
 *   3. 1 (smoke-test default)
 */
inline int benchmark_iterations(int default_full) {
    if (const char* env = std::getenv("OUSTER_BENCHMARK_ITERATIONS")) {
        int n = std::atoi(env);
        if (n > 0) return n;
    }
    return enable_benchmark_tests() ? default_full : 1;
}

/**
 * Records benchmark timing as GTest properties.
 *   test_runtime        — total elapsed nanoseconds for all iterations
 *   test_iteration_time — average nanoseconds per iteration
 *   test_iterations     — number of iterations run
 *
 * @param total_ns   Total elapsed nanoseconds from Timer::elapsed_nanoseconds()
 * @param iterations Number of timed iterations
 */
inline void record_benchmark(int64_t total_ns, int iterations) {
    const double iter_ns = iterations > 0 ? (static_cast<double>(total_ns) / iterations) : 0.0;
    ::testing::Test::RecordProperty("test_runtime", std::to_string(total_ns));
    ::testing::Test::RecordProperty("test_iteration_time", std::to_string(iter_ns));
    ::testing::Test::RecordProperty("test_iterations", std::to_string(iterations));
}

/**
 * Prints timing to stdout and records GTest properties.
 *
 * Output: "{label}: {N} iterations, {X.XX} ns/iter"
 */
inline void report_benchmark(const std::string& label, int64_t total_ns, int iterations) {
    const double ns_per_iter = iterations > 0 ? (static_cast<double>(total_ns) / iterations) : 0.0;
    std::cout << label << ": " << iterations << " iterations, " << ns_per_iter << " ns/iter"
              << std::endl;
    record_benchmark(total_ns, iterations);
}

/**
 * A pcap file used to parameterize file-based benchmarks.
 */
struct test_fixture {
    std::string pcap_filename;
};

/*
 * PrintTo is a Gtest hook thatenables GTest to emit the pcap
 * filename in XML/JSON value_param instead of a raw hex dump.
 */
inline void PrintTo(const test_fixture& f, std::ostream* os) {
    *os << f.pcap_filename;
}

/** Returns a seeded random number. Use a fixed seed for reproducibility. */
inline std::mt19937 seeded_rng(uint32_t seed = 42) {
    return std::mt19937{seed};
}

/**
 * Fills a range image with ~50 % valid returns using a fixed-distribution
 * RNG. Invalid pixels are set to 0; valid pixels to a value in [1, 10000].
 * Called inside the timing loop to prevent cache warm-up bias.
 */
inline void randomize_range(ouster::sdk::core::img_t<uint32_t>& range, std::mt19937& gen) {
    static std::uniform_int_distribution<uint32_t> val(1, 10000);
    static std::uniform_real_distribution<float> coin(0.0f, 1.0f);
    for (int r = 0; r < range.rows(); ++r)
        for (int c = 0; c < range.cols(); ++c) range(r, c) = coin(gen) < 0.5f ? 0u : val(gen);
}

/**
 * Builds an XYZLut from randomized direction and offset arrays.
 */
inline ouster::sdk::core::XYZLut make_random_lut(int height, int width, std::mt19937& gen) {
    const int rows = height * width;
    std::uniform_real_distribution<double> dir_dist(0.5, 1.5);
    std::uniform_real_distribution<double> off_dist(0.0, 0.01);

    ouster::sdk::core::ArrayX3dR direction(rows, 3);
    ouster::sdk::core::ArrayX3dR offset(rows, 3);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < 3; ++c) {
            direction(r, c) = dir_dist(gen);
            offset(r, c) = off_dist(gen);
        }
    }
    return ouster::sdk::core::XYZLut(direction, offset, height, width);
}

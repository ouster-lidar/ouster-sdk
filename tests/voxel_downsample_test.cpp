#include <gtest/gtest.h>
#include <ouster/core/eigen_hash.h>
#include <ouster/core/voxel_hash_map.h>
#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <random>
#include <unordered_set>
#include <vector>

#include "util.h"

using Voxel = Eigen::Vector3i;
using ouster::sdk::core::voxel_downsample;
using ouster::sdk::core::VoxelHashMap3d;

// V0: Original first-in-wins (no randomization).
static std::vector<Eigen::Vector3d> VoxelDownsampleLegacy(const std::vector<Eigen::Vector3d>& frame,
                                                          const double voxel_size) {
    tsl::robin_map<Voxel, Eigen::Vector3d> grid;
    grid.reserve(frame.size());
    for (const auto& point : frame) {
        const auto voxel = VoxelHashMap3d::point_to_voxel(point, 1.0 / voxel_size);
        if (!grid.contains(voxel)) grid.insert({voxel, point});
    }
    std::vector<Eigen::Vector3d> result;
    result.reserve(grid.size());
    for (const auto& kv : grid) result.emplace_back(kv.second);
    return result;
}

TEST(VoxelDownsampleTest, EmptyInput) {
    auto result = voxel_downsample({}, 1.0);
    EXPECT_TRUE(result.first.empty());
    EXPECT_TRUE(result.second.empty());
}

TEST(VoxelDownsampleTest, SinglePoint) {
    std::vector<Eigen::Vector3d> frame = {{1.5, 2.5, 3.5}};
    auto result = voxel_downsample(frame, 1.0);
    ASSERT_EQ(result.first.size(), 1u);
    EXPECT_EQ(result.first[0], frame[0]);
    ASSERT_EQ(result.second.size(), 1u);
    EXPECT_EQ(result.second[0], 0u);
}

TEST(VoxelDownsampleTest, PointsInDistinctVoxelsAllPreserved) {
    const double vs = 1.0;
    std::vector<Eigen::Vector3d> frame = {{0.5, 0.5, 0.5}, {1.5, 0.5, 0.5}, {0.5, 1.5, 0.5}};
    auto result = voxel_downsample(frame, vs);
    EXPECT_EQ(result.first.size(), frame.size());
    EXPECT_EQ(result.second.size(), frame.size());
}

TEST(VoxelDownsampleTest, OutputPointsComeFromInput) {
    const double vs = 1.0;
    std::mt19937 gen(123);
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    std::vector<Eigen::Vector3d> frame(500);
    for (auto& pt : frame) pt = Eigen::Vector3d(dist(gen), dist(gen), dist(gen));

    auto pts = voxel_downsample(frame, vs).first;
    for (const auto& rpt : pts) {
        bool found = std::any_of(frame.begin(), frame.end(),
                                 [&](const Eigen::Vector3d& p) { return p == rpt; });
        EXPECT_TRUE(found) << "Output point not found in input";
    }
}

TEST(VoxelDownsampleTest, OneRepresentativePerVoxel) {
    const double vs = 1.0;
    std::mt19937 gen(456);
    std::uniform_real_distribution<double> dist(-5.0, 5.0);
    std::vector<Eigen::Vector3d> frame(1000);
    for (auto& pt : frame) pt = Eigen::Vector3d(dist(gen), dist(gen), dist(gen));

    auto pts = voxel_downsample(frame, vs).first;
    std::unordered_set<Voxel> voxels;
    for (const auto& pt : pts) {
        auto v = VoxelHashMap3d::point_to_voxel(pt, 1.0 / vs);
        EXPECT_TRUE(voxels.insert(v).second) << "Duplicate voxel in output";
    }
}

TEST(VoxelDownsampleTest, DeterministicWithFixedSeed) {
    const double vs = 1.0;
    std::mt19937 gen(789);
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    std::vector<Eigen::Vector3d> frame(1000);
    for (auto& pt : frame) pt = Eigen::Vector3d(dist(gen), dist(gen), dist(gen));

    auto r1 = voxel_downsample(frame, vs);
    auto r2 = voxel_downsample(frame, vs);
    ASSERT_EQ(r1.first.size(), r2.first.size());
    for (size_t i = 0; i < r1.first.size(); ++i) {
        EXPECT_EQ(r1.first[i], r2.first[i]);
        EXPECT_EQ(r1.second[i], r2.second[i]);
    }
}

TEST(VoxelDownsampleTest, IndicesMatchOutputPoints) {
    const double vs = 1.0;
    std::mt19937 gen(321);
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    std::vector<Eigen::Vector3d> frame(1000);
    for (auto& pt : frame) pt = Eigen::Vector3d(dist(gen), dist(gen), dist(gen));

    auto result = voxel_downsample(frame, vs);
    const auto& pts = result.first;
    const auto& idx = result.second;

    ASSERT_EQ(pts.size(), idx.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        ASSERT_LT(idx[i], frame.size());
        EXPECT_EQ(pts[i], frame[idx[i]])
            << "Output point " << i << " doesn't match frame[" << idx[i] << "]";
    }
}

TEST(VoxelDownsampleTest, IndicesAreUnique) {
    const double vs = 1.0;
    std::mt19937 gen(654);
    std::uniform_real_distribution<double> dist(-5.0, 5.0);
    std::vector<Eigen::Vector3d> frame(500);
    for (auto& pt : frame) pt = Eigen::Vector3d(dist(gen), dist(gen), dist(gen));

    auto idx = voxel_downsample(frame, vs).second;

    std::unordered_set<uint32_t> index_set(idx.begin(), idx.end());
    EXPECT_EQ(index_set.size(), idx.size()) << "Duplicate indices found";
}

TEST(VoxelDownsampleTest, DoesNotAlwaysPickFirstPoint) {
    // Feed the same set of points in different orderings. With randomized
    // selection the chosen representative should vary across orderings.
    const double vs = 100.0;
    const int n_points = 200;
    std::vector<Eigen::Vector3d> base_frame;
    base_frame.reserve(n_points);
    for (int i = 0; i < n_points; ++i) base_frame.emplace_back(double(i) * 0.01, 0.0, 0.0);

    std::mt19937 shuffle_rng(777);
    std::unordered_set<int> selected;
    const int n_trials = 20;
    for (int t = 0; t < n_trials; ++t) {
        auto frame = base_frame;
        std::shuffle(frame.begin(), frame.end(), shuffle_rng);
        auto pts = voxel_downsample(frame, vs).first;
        ASSERT_EQ(pts.size(), 1u);
        for (int i = 0; i < n_points; ++i) {
            if (pts[0] == base_frame[i]) {
                selected.insert(i);
                break;
            }
        }
    }
    EXPECT_GT(selected.size(), 1u)
        << "Same point selected every time — randomization may not be working";
}

TEST(VoxelDownsampleTest, UniformSelectionAcrossVoxel) {
    const double vs = 100.0;
    const int n_points = 20;
    const int n_trials = 5000;
    std::vector<int> selection_count(n_points, 0);
    std::mt19937 shuffle_rng(999);

    std::vector<Eigen::Vector3d> base_frame;
    base_frame.reserve(n_points);
    for (int i = 0; i < n_points; ++i) base_frame.emplace_back(double(i) * 0.1, 0.0, 0.0);

    for (int trial = 0; trial < n_trials; ++trial) {
        auto frame = base_frame;
        std::shuffle(frame.begin(), frame.end(), shuffle_rng);
        auto pts = voxel_downsample(frame, vs).first;
        ASSERT_EQ(pts.size(), 1u);
        for (int i = 0; i < n_points; ++i) {
            if (pts[0] == base_frame[i]) {
                selection_count[i]++;
                break;
            }
        }
    }

    double expected = double(n_trials) / n_points;
    for (int i = 0; i < n_points; ++i) {
        double sigma = std::sqrt(expected * (1.0 - 1.0 / n_points));
        EXPECT_NEAR(selection_count[i], expected, 4.0 * sigma)
            << "Point " << i << " selected " << selection_count[i] << " times, expected ~"
            << expected;
    }
}

namespace {
struct PerfResult {
    double ms_per_call;
    size_t output_size;
};

template <typename Fn>
PerfResult bench(Fn fn, int warmup_iters, int measure_iters) {
    size_t output_size = 0;
    for (int i = 0; i < warmup_iters; ++i) {
        auto r = fn();
        output_size += r.size();
    }
    output_size = 0;

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < measure_iters; ++i) {
        auto result = fn();
        output_size += result.size();
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / measure_iters;
    return {ms, output_size / measure_iters};
}
}  // namespace

TEST(VoxelDownsampleTest, Performance) {
    int warmup = enable_perf_comparison_tests() ? 200 : 1;
    int n_iters = enable_perf_comparison_tests() ? 2000 : 2;

    std::mt19937 gen(42);

    struct Scenario {
        const char* label;
        int n_points;
        double voxel_size;
        double spatial_range;
    };
    // spatial_range controls how spread out points are.
    // ~pts / (2*range/vs)^3 gives avg points per occupied voxel.
    std::vector<Scenario> scenarios = {
        {"spread  (128K pts, vs=0.5, 100m)", 131072, 0.5, 50.0},
        {"lidar   (128K pts, vs=0.5, 15m)", 131072, 0.5, 7.5},
        {"cluster (128K pts, vs=0.5, 5m)", 131072, 0.5, 2.5},
        {"big     (524K pts, vs=1.0, 500m)", 524288, 1.0, 250.0},
    };

    using DsFn = std::vector<Eigen::Vector3d> (*)(const std::vector<Eigen::Vector3d>&, double);
    static auto CurrentImpl = [](const std::vector<Eigen::Vector3d>& f, double vs) {
        return voxel_downsample(f, vs).first;
    };
    struct Variant {
        const char* name;
        DsFn fn;
    };
    std::vector<Variant> variants = {
        {"Legacy (first-in)", VoxelDownsampleLegacy},
        {"Current (shuffle)", +CurrentImpl},
    };

    for (const auto& s : scenarios) {
        std::uniform_real_distribution<double> dist(-s.spatial_range, s.spatial_range);
        std::vector<Eigen::Vector3d> frame(s.n_points);
        for (auto& pt : frame) pt = Eigen::Vector3d(dist(gen), dist(gen), dist(gen));

        auto ref = VoxelDownsampleLegacy(frame, s.voxel_size);
        double pts_per_voxel = double(frame.size()) / ref.size();
        char hdr[128];
        std::snprintf(hdr, sizeof(hdr), "\n  %s  [%zu voxels, %.1f pts/voxel]\n", s.label,
                      ref.size(), pts_per_voxel);
        std::cout << hdr;
        std::cout << "  " << std::string(60, '-') << "\n";

        PerfResult baseline{};
        for (size_t v = 0; v < variants.size(); ++v) {
            auto r = bench([&] { return variants[v].fn(frame, s.voxel_size); }, warmup, n_iters);
            if (v == 0) baseline = r;
            double delta_pct =
                (r.ms_per_call - baseline.ms_per_call) / baseline.ms_per_call * 100.0;
            char buf[128];
            std::snprintf(buf, sizeof(buf), "    %-20s  %8.3f ms    %+.1f%%\n", variants[v].name,
                          r.ms_per_call, delta_pct);
            std::cout << buf;
            ASSERT_EQ(r.output_size, baseline.output_size)
                << "Output size mismatch: " << variants[v].name;
        }
    }
}

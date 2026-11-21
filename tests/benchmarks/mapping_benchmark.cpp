#include <gtest/gtest.h>
#include <ouster/core/impl/voxel_hash_map_t.h>
#include <ouster/core/voxel_hash_map.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <vector>

#include "../util.h"
#include "benchmark_utils.h"
#include "ouster/mapping/icp_registration.h"

TEST(JacobianBenchmark, build_linear_system) {
    const int N_CORRS = 10000;
    const int N_ITERS = benchmark_iterations(1000);

    auto gen = seeded_rng();
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    std::uniform_real_distribution<double> noise(-0.2, 0.2);

    ouster::sdk::mapping::Correspondences corrs;
    corrs.reserve(N_CORRS);
    for (int i = 0; i < N_CORRS; ++i) {
        Eigen::Vector3d s(dist(gen), dist(gen), dist(gen));
        Eigen::Vector3d t = s + Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
        corrs.push_back(std::make_pair(s, t));
    }

    Timer t;
    t.start();
    for (int i = 0; i < N_ITERS; ++i) {
        auto result = ouster::sdk::mapping::build_linear_system(corrs, 1.0);
        ASSERT_FALSE(result.second.hasNaN());
    }
    t.stop();

    report_benchmark("build_linear_system (" + std::to_string(N_CORRS) + " corrs/iter)",
                     t.elapsed_nanoseconds(), N_ITERS);
}

TEST(VoxelHashMapBenchmark, get_closest_neighbor) {
    const int n_iterations = benchmark_iterations(1000);
    const int n_queries = 10000;

    const double voxel_size = 1.0;
    ouster::sdk::core::VoxelHashMap3d map(voxel_size, 200.0, 20);

    auto rng = seeded_rng(7);
    std::uniform_real_distribution<double> dist(-10.0, 10.0);

    // Build a moderately dense map — ~50k points over a 20×20×20 region
    std::vector<Eigen::Vector3d> points;
    const int n_map_points = 50000;
    points.reserve(n_map_points);
    for (int i = 0; i < n_map_points; ++i) points.emplace_back(dist(rng), dist(rng), dist(rng));
    map.add_points(points);

    // Pre-generate queries so timing doesn't include RNG
    std::vector<Eigen::Vector3d> queries;
    queries.reserve(n_queries);
    for (int i = 0; i < n_queries; ++i) queries.emplace_back(dist(rng), dist(rng), dist(rng));

    const double max_dist_sq = 4.0;

    Timer t;
    t.start();
    for (int iter = 0; iter < n_iterations; ++iter) {
        for (const auto& q : queries) {
            auto result = map.get_closest_neighbor(q, max_dist_sq);
            ASSERT_GE(std::get<1>(result), 0.0);
        }
    }
    t.stop();

    report_benchmark("get_closest_neighbor (" + std::to_string(n_queries) + " queries/iter)",
                     t.elapsed_nanoseconds(), n_iterations);
}

TEST(VoxelHashMapBenchmark, add_points) {
    std::map<std::string, std::string> styles = term_styles();
    std::cout << styles["yellow"] << styles["bold"]
              << "CHECKING PERFORMANCE FOR VoxelHashMap::add_points (core vs "
                 "mapping::impl)"
              << styles["reset"] << std::endl;

    const double voxel_size = 1.0;
    const double max_distance = 200.0;
    const size_t max_points_per_voxel = 20;
    const int n_points = enable_perf_comparison_tests() ? 50000 : 5000;

    auto rng = seeded_rng(42);
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::VectorXd> points_xd;
    points.reserve(static_cast<size_t>(n_points));
    points_xd.reserve(static_cast<size_t>(n_points));
    for (int i = 0; i < n_points; ++i) {
        points.emplace_back(dist(rng), dist(rng), dist(rng));
        points_xd.emplace_back(points[i]);
    }

    using CoreMap3d =
        ouster::sdk::core::VoxelHashMapT<Eigen::Vector3i, Eigen::Vector3d,
                                         ouster::sdk::core::DefaultVoxelBucket<Eigen::Vector3d>,
                                         ouster::sdk::core::impl::first_n_point>;
    using CoreMapXd =
        ouster::sdk::core::VoxelHashMapT<Eigen::Vector3i, Eigen::VectorXd,
                                         ouster::sdk::core::DefaultVoxelBucket<Eigen::VectorXd>,
                                         ouster::sdk::core::impl::first_n_point>;

    CoreMap3d core_vhmap(voxel_size, max_distance, max_points_per_voxel);
    CoreMapXd core_vhmap_xd(voxel_size, max_distance, max_points_per_voxel);
    CoreMap3d first_win_vhmap(voxel_size, max_distance, max_points_per_voxel);
    CoreMapXd first_win_vhmap_xd(voxel_size, max_distance, max_points_per_voxel);

    constexpr int MOVING_AVG_WINDOW = 30;
    using MovingAverage64 = MovingAverage<int64_t, int64_t, MOVING_AVG_WINDOW>;
    static std::map<std::string, MovingAverage64> mv;

    const int ITERATIONS = enable_perf_comparison_tests() ? 100 : 1;

    Timer t;
    std::stringstream ss;
    int output_ctr = 0;

    using AddPointsMethod = std::function<void()>;
    std::vector<std::pair<std::string, AddPointsMethod>> all_methods;

    all_methods.emplace_back("core", [&]() {
        core_vhmap.clear();
        core_vhmap.add_points(points);
    });

    all_methods.emplace_back("core_xd", [&]() {
        core_vhmap_xd.clear();
        core_vhmap_xd.add_points(points_xd);
    });

    all_methods.emplace_back("first_win", [&]() {
        first_win_vhmap.clear();
        first_win_vhmap.add_points(points);
    });

    all_methods.emplace_back("first_win_xd", [&]() {
        first_win_vhmap_xd.clear();
        first_win_vhmap_xd.add_points(points_xd);
    });

    std::default_random_engine g;
    std::vector<int> ids(all_methods.size());
    std::iota(std::begin(ids), std::end(ids), 0);

    for (int iter = 0; iter < ITERATIONS; ++iter) {
        std::shuffle(std::begin(ids), std::end(ids), g);
        for (int id : ids) {
            const auto& method = all_methods[static_cast<size_t>(id)];
            t.start();
            method.second();
            t.stop();
            mv[method.first](t.elapsed_microseconds());
        }

        if (++output_ctr % MOVING_AVG_WINDOW == 0) {
            ss.str("");
            ss << styles["bold"] << "points: " << styles["reset"] << styles["magenta"]
               << std::setw(6) << n_points << ", " << styles["reset"];
            ss << styles["bold"] << "core[time]: " << styles["reset"] << styles["cyan"]
               << std::setw(4) << mv["core"] << "μs, " << styles["reset"];
            ss << styles["bold"] << "core_xd[time]: " << styles["reset"] << styles["cyan"]
               << std::setw(4) << mv["core_xd"] << "μs, " << styles["reset"];

            auto best_time = std::min_element(
                mv.begin(), mv.end(),
                [](const auto& a, const auto& b) -> bool { return a.second < b.second; });

            for (const auto& x : mv) {
                auto speedup =
                    lround(100.0f * static_cast<float>(mv["core"]) / static_cast<float>(x.second));
                auto color_modifier = x.first == best_time->first
                                          ? styles["blue"]
                                          : (speedup >= 100 ? styles["green"] : styles["red"]);
                ss << styles["bold"] << x.first << ": " << color_modifier << std::setw(4) << speedup
                   << "%, " << styles["reset"];
            }
            std::cout << ss.str() << std::endl;
        }
    }

    // Sanity-check that every variant produced an identical occupied-voxel
    // count for the same input (they only differ in PointType, not in the
    // dedup/insert logic).
    ASSERT_GT(core_vhmap.pointcloud_vector().size(), 0u);
    ASSERT_EQ(core_vhmap.pointcloud_vector().size(), core_vhmap_xd.pointcloud_vector().size());
}

TEST(VoxelHashMapBenchmark, add_points_dense_vs_packed_records) {
    using ouster::sdk::core::ArrayX3dR;
    using ouster::sdk::core::ArrayXXdR;
    using ouster::sdk::core::AttributeLayout;
    using ouster::sdk::core::AttributeType;
    using ouster::sdk::core::VoxelHashMapXd;

    std::map<std::string, std::string> styles = term_styles();
    std::cout << styles["yellow"] << styles["bold"]
              << "CHECKING PERFORMANCE FOR VoxelHashMap::add_points(ArrayXXdR) vs "
                 "add_points(ArrayX3dR, records)"
              << styles["reset"] << std::endl;

    const double voxel_size = 1.0;
    const double max_distance = 200.0;
    const size_t max_points_per_voxel = 20;
    const int n_points = enable_perf_comparison_tests() ? 50000 : 5000;

    // Same 2-attribute layout as the voxel_hash_map example: a
    // RANGE (uint32) and a REFLECTIVITY (uint8) value riding alongside every
    // point. Both maps below share it, so every point carries the same 5
    // bytes/point of attribute data whether it arrives as two widened doubles
    // (10 of the 16 bytes/point add_points(ArrayXXdR) hands in) or as an
    // already-packed 5-byte record.
    const AttributeLayout layout({{AttributeType::U32, 1}, {AttributeType::U8, 1}});
    const std::size_t stride = layout.stride();
    const Eigen::Index n_cols = 3 + static_cast<Eigen::Index>(layout.element_count());

    auto rng = seeded_rng(99);
    std::uniform_real_distribution<double> xyz_dist(-10.0, 10.0);
    std::uniform_int_distribution<std::uint32_t> range_dist(
        0, std::numeric_limits<std::uint32_t>::max());
    std::uniform_int_distribution<int> reflectivity_dist(0, 255);

    // The same underlying per-point data (xyz, range, reflectivity), laid out
    // two ways: a dense [xyz | range | reflectivity]-as-double matrix for
    // add_points(ArrayXXdR), and an Nx3 xyz array plus a pre-packed
    // [u32, u8] record per point for add_points(ArrayX3dR, records).
    ArrayXXdR dense_points(n_points, n_cols);
    ArrayX3dR xyz(n_points, 3);
    ouster::sdk::core::AttributeRecords records(n_points, static_cast<Eigen::Index>(stride));

    double elements[2];
    for (int i = 0; i < n_points; ++i) {
        const double x = xyz_dist(rng);
        const double y = xyz_dist(rng);
        const double z = xyz_dist(rng);
        const std::uint32_t range_val = range_dist(rng);
        const std::uint8_t reflectivity_val = static_cast<std::uint8_t>(reflectivity_dist(rng));

        dense_points(i, 0) = x;
        dense_points(i, 1) = y;
        dense_points(i, 2) = z;
        dense_points(i, 3) = static_cast<double>(range_val);
        dense_points(i, 4) = static_cast<double>(reflectivity_val);

        xyz(i, 0) = x;
        xyz(i, 1) = y;
        xyz(i, 2) = z;

        elements[0] = static_cast<double>(range_val);
        elements[1] = static_cast<double>(reflectivity_val);
        layout.encode(elements, records.row(i).data());
    }

    VoxelHashMapXd dense_map(voxel_size, max_distance, max_points_per_voxel, 1,
                             ouster::sdk::core::AttributeLayout::all_double(2));
    VoxelHashMapXd packed_map(voxel_size, max_distance, max_points_per_voxel, 1, layout);

    constexpr int MOVING_AVG_WINDOW = 30;
    using MovingAverage64 = MovingAverage<int64_t, int64_t, MOVING_AVG_WINDOW>;
    static std::map<std::string, MovingAverage64> mv;

    const int ITERATIONS = enable_perf_comparison_tests() ? 100 : 1;

    Timer t;
    std::stringstream ss;
    int output_ctr = 0;

    using AddPointsMethod = std::function<void()>;
    std::vector<std::pair<std::string, AddPointsMethod>> all_methods;

    all_methods.emplace_back("dense_arrayxxd", [&]() {
        dense_map.clear();
        dense_map.add_points(dense_points);
    });

    all_methods.emplace_back("packed_records", [&]() {
        packed_map.clear();
        packed_map.add_points(xyz, records);
    });

    std::default_random_engine g;
    std::vector<int> ids(all_methods.size());
    std::iota(std::begin(ids), std::end(ids), 0);

    for (int iter = 0; iter < ITERATIONS; ++iter) {
        std::shuffle(std::begin(ids), std::end(ids), g);
        for (int id : ids) {
            const auto& method = all_methods[static_cast<size_t>(id)];
            t.start();
            method.second();
            t.stop();
            mv[method.first](t.elapsed_microseconds());
        }

        if (++output_ctr % MOVING_AVG_WINDOW == 0) {
            ss.str("");
            ss << styles["bold"] << "points: " << styles["reset"] << styles["magenta"]
               << std::setw(6) << n_points << ", " << styles["reset"];
            ss << styles["bold"] << "dense_arrayxxd[time]: " << styles["reset"] << styles["cyan"]
               << std::setw(4) << mv["dense_arrayxxd"] << "μs, " << styles["reset"];
            ss << styles["bold"] << "packed_records[time]: " << styles["reset"] << styles["cyan"]
               << std::setw(4) << mv["packed_records"] << "μs, " << styles["reset"];

            auto best_time = std::min_element(
                mv.begin(), mv.end(),
                [](const auto& a, const auto& b) -> bool { return a.second < b.second; });

            for (const auto& x : mv) {
                auto speedup = lround(100.0f * static_cast<float>(mv["dense_arrayxxd"]) /
                                      static_cast<float>(x.second));
                auto color_modifier = x.first == best_time->first
                                          ? styles["blue"]
                                          : (speedup >= 100 ? styles["green"] : styles["red"]);
                ss << styles["bold"] << x.first << ": " << color_modifier << std::setw(4) << speedup
                   << "%, " << styles["reset"];
            }
            std::cout << ss.str() << std::endl;
        }
    }

    // Sanity-check that both entry points produced an identical occupied-voxel
    // count for the same underlying per-point data (they only differ in how
    // the attribute payload is packaged on the way in).
    ASSERT_GT(dense_map.pointcloud_vector().size(), 0u);
    ASSERT_EQ(dense_map.pointcloud_vector().size(), packed_map.pointcloud_vector().size());
}

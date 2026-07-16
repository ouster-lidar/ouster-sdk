/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * Unit and performance tests for VoxelHashMap::get_closest_neighbor.
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "ouster/core/voxel_hash_map.h"

using ouster::sdk::core::DefaultVoxelBucket;
using ouster::sdk::core::IndexedPoint;
using ouster::sdk::core::IndexedReservoirHashMap3d;
using ouster::sdk::core::VoxelHashMap;
using ouster::sdk::core::VoxelHashMap3d;
using ouster::sdk::core::impl::indexed_reservoir_strategy;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static VoxelHashMap3d make_map(double voxel_size, double max_distance = 100.0,
                               std::size_t max_points_per_voxel = 20) {
    return VoxelHashMap3d(voxel_size, max_distance, max_points_per_voxel);
}

// Brute-force NN over a flat list — used as the reference oracle.
static std::tuple<Eigen::Vector3d, double> brute_force_nn(
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& query) {
    Eigen::Vector3d best = Eigen::Vector3d::Zero();
    double best_sq = std::numeric_limits<double>::max();
    for (const auto& p : points) {
        double d = (p - query).squaredNorm();
        if (d < best_sq) {
            best = p;
            best_sq = d;
        }
    }
    return std::make_tuple(best, best_sq);
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

TEST(VoxelHashMapTest, EmptyMapReturnsMax) {
    auto map = make_map(1.0);
    Eigen::Vector3d query(0.5, 0.5, 0.5);
    auto result = map.get_closest_neighbor(query);
    EXPECT_DOUBLE_EQ(std::get<1>(result), std::numeric_limits<double>::max());
}

TEST(VoxelHashMapTest, ExactMatch) {
    auto map = make_map(1.0);
    Eigen::Vector3d pt(2.5, 3.5, 4.5);
    map.add_points(std::vector<Eigen::Vector3d>{pt});

    auto result = map.get_closest_neighbor(pt);
    EXPECT_DOUBLE_EQ(std::get<1>(result), 0.0);
    EXPECT_EQ(std::get<0>(result), pt);
}

TEST(VoxelHashMapTest, SameVoxelNeighbor) {
    auto map = make_map(1.0);
    Eigen::Vector3d pt(0.3, 0.3, 0.3);
    Eigen::Vector3d query(0.7, 0.7, 0.7);
    map.add_points(std::vector<Eigen::Vector3d>{pt});

    auto result = map.get_closest_neighbor(query);
    double expected_sq = (pt - query).squaredNorm();
    EXPECT_DOUBLE_EQ(std::get<1>(result), expected_sq);
    EXPECT_EQ(std::get<0>(result), pt);
}

TEST(VoxelHashMapTest, AdjacentVoxelNeighbor) {
    auto map = make_map(1.0);
    // Two points in adjacent voxels, query closer to the one across the border
    Eigen::Vector3d pt_a(0.2, 0.5, 0.5);  // voxel (0,0,0)
    Eigen::Vector3d pt_b(1.1, 0.5, 0.5);  // voxel (1,0,0)
    Eigen::Vector3d query(0.95, 0.5, 0.5);
    map.add_points(std::vector<Eigen::Vector3d>{pt_a, pt_b});

    auto result = map.get_closest_neighbor(query);
    EXPECT_EQ(std::get<0>(result), pt_b);
    EXPECT_DOUBLE_EQ(std::get<1>(result), (pt_b - query).squaredNorm());
}

TEST(VoxelHashMapTest, MaxDistanceSqPrunesAll) {
    auto map = make_map(1.0);
    Eigen::Vector3d pt(5.0, 5.0, 5.0);
    map.add_points(std::vector<Eigen::Vector3d>{pt});

    // Query is in the same neighborhood but impose a very tight bound
    Eigen::Vector3d query(5.5, 5.5, 5.5);
    double tight_bound = 0.001;
    auto result = map.get_closest_neighbor(query, tight_bound);
    // No point is within 0.001 sq dist, so should return the seeded bound
    EXPECT_GE(std::get<1>(result), tight_bound);
}

TEST(VoxelHashMapTest, MaxDistanceSqAllowsClose) {
    auto map = make_map(1.0);
    Eigen::Vector3d pt(5.0, 5.0, 5.0);
    map.add_points(std::vector<Eigen::Vector3d>{pt});

    Eigen::Vector3d query(5.1, 5.0, 5.0);
    double generous_bound = 1.0;
    auto result = map.get_closest_neighbor(query, generous_bound);
    EXPECT_DOUBLE_EQ(std::get<1>(result), (pt - query).squaredNorm());
    EXPECT_EQ(std::get<0>(result), pt);
}

TEST(VoxelHashMapTest, NegativeCoordinates) {
    auto map = make_map(1.0);
    Eigen::Vector3d pt(-2.3, -1.7, -0.5);
    Eigen::Vector3d query(-2.1, -1.9, -0.6);
    map.add_points(std::vector<Eigen::Vector3d>{pt});

    auto result = map.get_closest_neighbor(query);
    EXPECT_DOUBLE_EQ(std::get<1>(result), (pt - query).squaredNorm());
}

// The key correctness test: verify early-exit produces identical results to a
// brute-force iterate over all map points.
TEST(VoxelHashMapTest, MatchesBruteForce) {
    const double voxel_size = 1.0;
    auto map = make_map(voxel_size, /*max_distance=*/100.0,
                        /*max_points_per_voxel=*/20);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-5.0, 5.0);

    std::vector<Eigen::Vector3d> all_points;
    const int n_points = 2000;
    all_points.reserve(n_points);
    for (int i = 0; i < n_points; ++i) {
        all_points.emplace_back(dist(rng), dist(rng), dist(rng));
    }
    map.add_points(all_points);

    // Not all points may have been inserted (density limits), so extract what
    // the map actually contains.
    auto map_points = map.pointcloud_vector();

    const int n_queries = 500;
    for (int i = 0; i < n_queries; ++i) {
        Eigen::Vector3d query(dist(rng), dist(rng), dist(rng));

        auto voxel_result = map.get_closest_neighbor(query);
        auto brute_result = brute_force_nn(map_points, query);

        ASSERT_NEAR(std::get<1>(voxel_result), std::get<1>(brute_result), 1e-12)
            << "Mismatch at query " << i << ": " << query.transpose();
    }
}

// ---------------------------------------------------------------------------
// Footprint regression: DefaultVoxelBucket must stay the same size as
// std::vector<PointType>.
// ---------------------------------------------------------------------------

TEST(VoxelHashMapTest, EmptyMetadataBucketCollapsesToVectorSize) {
    using Bucket = VoxelHashMap3d::VoxelBucket;
    EXPECT_EQ(sizeof(Bucket), sizeof(std::vector<Eigen::Vector3d>));
}

// ---------------------------------------------------------------------------
// Constructor validation
// ---------------------------------------------------------------------------

TEST(VoxelHashMapTest, RejectsAttributesForFixedSizePointType) {
    EXPECT_THROW(VoxelHashMap3d(1.0, 100.0, 20, 1, /*num_attributes=*/5), std::invalid_argument);
}

TEST(VoxelHashMapTest, AcceptsAttributesForDynamicPointType) {
    EXPECT_NO_THROW(ouster::sdk::core::VoxelHashMapXd(1.0, 100.0, 20,
                                                      /*num_attributes=*/5));
}

// ---------------------------------------------------------------------------
// Regression: existing behavior must be identical after the refactor
// ---------------------------------------------------------------------------

// Same as above but with the max_distance_sq bound active.
TEST(VoxelHashMapTest, MatchesBruteForceWithBound) {
    const double voxel_size = 1.0;
    auto map = make_map(voxel_size, /*max_distance=*/100.0,
                        /*max_points_per_voxel=*/20);

    std::mt19937 rng(123);
    std::uniform_real_distribution<double> dist(-5.0, 5.0);

    std::vector<Eigen::Vector3d> all_points;
    for (int i = 0; i < 2000; ++i) {
        all_points.emplace_back(dist(rng), dist(rng), dist(rng));
    }
    map.add_points(all_points);
    auto map_points = map.pointcloud_vector();

    const double max_dist = 2.0;
    const double max_dist_sq = max_dist * max_dist;

    for (int i = 0; i < 500; ++i) {
        Eigen::Vector3d query(dist(rng), dist(rng), dist(rng));

        auto voxel_result = map.get_closest_neighbor(query, max_dist_sq);
        auto brute_result = brute_force_nn(map_points, query);

        double voxel_d = std::get<1>(voxel_result);
        double brute_d = std::get<1>(brute_result);

        if (brute_d < max_dist_sq) {
            // Both should agree on the closest point
            ASSERT_NEAR(voxel_d, brute_d, 1e-12)
                << "Mismatch at query " << i << ": " << query.transpose();
        } else {
            // Brute force has no neighbor within bound, voxel should also
            // report nothing (distance >= max_dist_sq)
            ASSERT_GE(voxel_d, max_dist_sq);
        }
    }
}

TEST(VoxelHashMapTest, IndexedReservoirVector3dRecordsInsertionOrder) {
    // Points placed 2 m apart so each lands in its own voxel â no replacement.
    // The strategy must overwrite the sentinel index (999) with 0, 1, 2, ...
    IndexedReservoirHashMap3d map(1.0, 1000.0, 5);

    const int N = 6;
    for (int i = 0; i < N; ++i) {
        IndexedPoint<Eigen::Vector3d> pt;
        pt.point = Eigen::Vector3d(i * 2.0, 0.0, 0.0);
        pt.index = 999u;  // sentinel â strategy must overwrite
        map.add_point(pt);
    }

    EXPECT_EQ(map.strategy_.call_count, static_cast<std::uint64_t>(N));

    const auto pts = map.pointcloud_vector();
    ASSERT_EQ(pts.size(), static_cast<std::size_t>(N));

    std::vector<std::uint32_t> indices;
    for (const auto& p : pts) {
        EXPECT_NE(p.index, 999u) << "strategy must overwrite input index";
        indices.push_back(p.index);
    }
    std::sort(indices.begin(), indices.end());
    for (int i = 0; i < N; ++i) {
        EXPECT_EQ(indices[i], static_cast<std::uint32_t>(i))
            << "insertion order rank " << i << " missing";
    }
}

TEST(VoxelHashMapTest, IndexedReservoirVector3dReplacementTracksCallCount) {
    // Single voxel, capacity 1.  After M insertions call_count == M and the
    // surviving point's index is in [0, M) â it is some insertion-order rank,
    // not necessarily the last one inserted.
    const int M = 50;
    IndexedReservoirHashMap3d map(100.0, 1000.0, 1);

    for (int i = 0; i < M; ++i) {
        IndexedPoint<Eigen::Vector3d> pt;
        pt.point = Eigen::Vector3d(0.5, 0.5, 0.5);
        pt.index = 0u;
        map.add_point(pt);
    }

    EXPECT_EQ(map.strategy_.call_count, static_cast<std::uint64_t>(M));

    const auto pts = map.pointcloud_vector();
    ASSERT_EQ(pts.size(), 1u);
    EXPECT_LT(pts[0].index, static_cast<std::uint32_t>(M))
        << "surviving index must be a valid insertion-order rank";
}

TEST(VoxelHashMapTest, IndexedReservoirVectorXdRecordsInsertionOrder) {
    // Same semantics as the Vector3d test but with a dynamic-size inner type.
    // Requires an explicit template instantiation for this specialisation in
    // voxel_hash_map.cpp (IndexedPoint<VectorXd> + indexed_reservoir_strategy).
    using IndexedReservoirHashMapXd =
        VoxelHashMap<Eigen::Vector3i, IndexedPoint<Eigen::VectorXd>,
                     DefaultVoxelBucket<IndexedPoint<Eigen::VectorXd>>, indexed_reservoir_strategy>;

    IndexedReservoirHashMapXd map(1.0, 1000.0, 5);

    const int N = 4;
    for (int i = 0; i < N; ++i) {
        IndexedPoint<Eigen::VectorXd> pt;
        pt.point = Eigen::VectorXd(3);
        pt.point << i * 2.0, 0.0, 0.0;
        pt.index = 999u;
        map.add_point(pt);
    }

    EXPECT_EQ(map.strategy_.call_count, static_cast<std::uint64_t>(N));

    const auto pts = map.pointcloud_vector();
    ASSERT_EQ(pts.size(), static_cast<std::size_t>(N));

    std::vector<std::uint32_t> indices;
    for (const auto& p : pts) {
        EXPECT_NE(p.index, 999u) << "strategy must overwrite input index";
        ASSERT_EQ(p.point.size(), 3) << "inner VectorXd must stay 3-D";
        indices.push_back(p.index);
    }
    std::sort(indices.begin(), indices.end());
    for (int i = 0; i < N; ++i) {
        EXPECT_EQ(indices[i], static_cast<std::uint32_t>(i));
    }
}

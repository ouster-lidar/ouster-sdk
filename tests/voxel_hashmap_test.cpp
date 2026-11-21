/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * Unit and performance tests for VoxelHashMapT::get_closest_neighbor.
 */

#include <gtest/gtest.h>
#include <ouster/core/impl/voxel_hash_map_t.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "ouster/core/attribute_layout.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/voxel_hash_map.h"

using ouster::sdk::core::ArrayX3dR;
using ouster::sdk::core::AttributeLayout;
using ouster::sdk::core::VHMInternalStorageMode;
using ouster::sdk::core::VoxelHashMap3d;
using ouster::sdk::core::VoxelHashMapT;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

struct StorageConfig {
    VHMInternalStorageMode mode;
    double def_voxel_size;
    double tol_abs;
};
namespace {
const StorageConfig g_storage_configs[] = {
    {VHMInternalStorageMode::ABS, 1.0, 0.0},
    {VHMInternalStorageMode::FX16, 1.0, 2.0 / ((1 << 16) - 1)},
    {VHMInternalStorageMode::FX8, 0.125, 2.0 / ((1 << 8) - 1)},
};

static VoxelHashMap3d make_map_v3d(double voxel_size, VHMInternalStorageMode storage_mode,
                                   double max_distance = 100.0,
                                   std::size_t max_points_per_voxel = 20) {
    return VoxelHashMap3d(voxel_size, max_distance, max_points_per_voxel,
                          VoxelHashMap3d::DEFAULT_MIN_POINTS_PER_VOXEL, AttributeLayout(),
                          storage_mode);
}

// Brute-force NN over a flat list — used as the reference oracle.
static std::tuple<Eigen::Vector3d, double> brute_force_nn(
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& query) {
    Eigen::Vector3d best = Eigen::Vector3d::Zero();
    double best_sq = std::numeric_limits<double>::max();
    for (const auto& point : points) {
        double dist_sq = (point - query).squaredNorm();
        if (dist_sq < best_sq) {
            best = point;
            best_sq = dist_sq;
        }
    }
    return std::make_tuple(best, best_sq);
}
}  // namespace

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

TEST(VoxelHashMapTest, EmptyMapReturnsMax) {
    for (const StorageConfig& config : g_storage_configs) {
        auto map = make_map_v3d(config.def_voxel_size, config.mode);
        Eigen::Vector3d query(0.5, 0.5, 0.5);
        auto result = map.get_closest_neighbor(query);
        EXPECT_DOUBLE_EQ(std::get<1>(result), std::numeric_limits<double>::max());
    }
}

TEST(VoxelHashMapTest, ExactMatch) {
    for (const StorageConfig& config : g_storage_configs) {
        auto map = make_map_v3d(config.def_voxel_size, config.mode);
        Eigen::Vector3d orig_pt(2.5, 3.5, 4.5);
        map.add_points(std::vector<Eigen::Vector3d>{orig_pt});

        auto result = map.get_closest_neighbor(orig_pt);
        if (config.mode == VHMInternalStorageMode::ABS) {
            EXPECT_EQ(std::get<0>(result), orig_pt);
            EXPECT_DOUBLE_EQ(std::get<1>(result), 0.0);
        } else {
            auto nnp = std::get<0>(result);
            for (int idx = 0; idx < 3; ++idx) {
                EXPECT_NEAR(orig_pt[idx], nnp[idx], config.tol_abs);
            }
            EXPECT_NEAR(std::get<1>(result), 0.0, config.tol_abs);
        }
    }
}

TEST(VoxelHashMapTest, SameVoxelNeighbor) {
    for (const StorageConfig& config : g_storage_configs) {
        auto map = make_map_v3d(config.def_voxel_size, config.mode);
        Eigen::Vector3d orig_pt(0.25, 0.25, 0.25);
        Eigen::Vector3d query = orig_pt + Eigen::Vector3d::Constant(0.5 * config.def_voxel_size);
        map.add_points(std::vector<Eigen::Vector3d>{orig_pt});

        auto result = map.get_closest_neighbor(query);
        double expected_sq = (orig_pt - query).squaredNorm();
        if (config.mode == VHMInternalStorageMode::ABS) {
            EXPECT_EQ(std::get<0>(result), orig_pt);
            EXPECT_DOUBLE_EQ(std::get<1>(result), expected_sq);
        } else {
            auto nnp = std::get<0>(result);
            for (int idx = 0; idx < 3; ++idx) {
                EXPECT_NEAR(orig_pt[idx], nnp[idx], config.tol_abs);
            }
            EXPECT_NEAR(std::get<1>(result), expected_sq, config.tol_abs);
        }
    }
}

TEST(VoxelHashMapTest, AdjacentVoxelNeighbor) {
    auto map = make_map_v3d(1.0, VHMInternalStorageMode::ABS);
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
    auto map = make_map_v3d(1.0, VHMInternalStorageMode::ABS);
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
    auto map = make_map_v3d(1.0, VHMInternalStorageMode::ABS);
    Eigen::Vector3d pt(5.0, 5.0, 5.0);
    map.add_points(std::vector<Eigen::Vector3d>{pt});

    Eigen::Vector3d query(5.1, 5.0, 5.0);
    double generous_bound = 1.0;
    auto result = map.get_closest_neighbor(query, generous_bound);
    EXPECT_DOUBLE_EQ(std::get<1>(result), (pt - query).squaredNorm());
    EXPECT_EQ(std::get<0>(result), pt);
}

TEST(VoxelHashMapTest, NegativeCoordinates) {
    auto map = make_map_v3d(1.0, VHMInternalStorageMode::ABS);
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
    auto map = make_map_v3d(voxel_size, VHMInternalStorageMode::ABS,
                            /*max_distance=*/100.0,
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
// Constructor validation
// ---------------------------------------------------------------------------

TEST(VoxelHashMapTest, RejectsAttributesForFixedSizePointType) {
    EXPECT_THROW(VoxelHashMap3d(1.0, 100.0, 20, 1, AttributeLayout::all_double(5)),
                 std::invalid_argument);
}

TEST(VoxelHashMapTest, AcceptsAttributesForDynamicPointType) {
    EXPECT_NO_THROW(
        ouster::sdk::core::VoxelHashMapXd(1.0, 100.0, 20, 1, AttributeLayout::all_double(5)));
}

// ---------------------------------------------------------------------------
// Regression: existing behavior must be identical after the refactor
// ---------------------------------------------------------------------------

// Same as above but with the max_distance_sq bound active.
TEST(VoxelHashMapTest, MatchesBruteForceWithBound) {
    const double voxel_size = 1.0;
    auto map = make_map_v3d(voxel_size, VHMInternalStorageMode::ABS,
                            /*max_distance=*/100.0,
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

//

// ---------------------------------------------------------------------------
// VectorXd map test - Checking auxiliary data is retrieved correctly
// ---------------------------------------------------------------------------
TEST(VoxelHashMapTest, AuxiliaryAttributesTest) {
    using namespace ouster::sdk::core;
    //
    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> uniform_f64(-1, 1);

    // Several points per voxel, so the attributes of each must stay paired.
    std::size_t max_points_per_voxel = 20;
    static int num_attribs_list[] = {0, 1, 2, 5};

    for (int num_attribs : num_attribs_list) {
        for (const StorageConfig& config : g_storage_configs) {
            double voxel_size = config.def_voxel_size;
            VoxelHashMapXd hm(voxel_size, VoxelHashMapXd::DEFAULT_MAX_DISTANCE,
                              max_points_per_voxel, VoxelHashMapXd::DEFAULT_MIN_POINTS_PER_VOXEL,
                              AttributeLayout::all_double(static_cast<std::size_t>(num_attribs)),
                              config.mode);

            std::vector<Eigen::VectorXd> points;

            // NOTE(Brett): Set this up such that we get multiple points per bucket, but that aren't
            // too close to one another
            constexpr int voxel_d = 10;  // Number of voxels to insert along each side
            static const Eigen::Vector3d voxel_offsets[] = {
                Eigen::Vector3d{0.25, 0.25, 0.25},
                Eigen::Vector3d{0.50, 0.50, 0.50},
                Eigen::Vector3d{0.75, 0.75, 0.75},
            };

            int min_coord = -voxel_d / 2;
            int max_coord = min_coord + voxel_d;
            for (int z_coord = min_coord; z_coord <= max_coord; ++z_coord) {
                for (int y_coord = min_coord; y_coord <= max_coord; ++y_coord) {
                    for (int x_coord = min_coord; x_coord <= max_coord; ++x_coord) {
                        for (const Eigen::Vector3d& offset : voxel_offsets) {
                            Eigen::VectorXd point(3 + num_attribs);
                            point.head<3>() =
                                voxel_size * Eigen::Vector3d(x_coord, y_coord, z_coord) +
                                voxel_size * offset;

                            for (int att_idx = 0; att_idx < num_attribs; ++att_idx) {
                                point(3 + att_idx) = uniform_f64(rng);
                            }

                            points.push_back(point);
                            hm.add_point(point);
                        }
                    }
                }
            }

            std::vector<Eigen::VectorXd> hmvector = hm.pointcloud_vector();
            EXPECT_EQ(hmvector.size(), points.size());
            std::size_t count = hmvector.size();
            for (std::size_t idx0 = 0; idx0 < count; ++idx0) {
                int found = 0;
                for (std::size_t idx1 = 0; idx1 < count; ++idx1) {
                    // NOTE(Brett): With quantization the XYZ coords might be slightly off, but the
                    // attributes should remain exact.
                    Eigen::Vector3d xyz_diff = points[idx0].head<3>() - hmvector[idx1].head<3>();
                    if ((xyz_diff.array().abs() <=
                         Eigen::Vector3d::Constant(config.tol_abs).array())
                            .all()) {
                        EXPECT_EQ(points[idx0].tail(num_attribs), hmvector[idx1].tail(num_attribs));
                        found += 1;
                    }
                }

                EXPECT_EQ(found, 1);
            }
        }
    }
}

TEST(VoxelHashMapTest, AverageVoxelHashMap3dCorrectlyAveragesAndRespectsThreshold) {
    using ouster::sdk::core::AverageVoxelHashMap3d;

    AverageVoxelHashMap3d map(1.0, 100.0, 20, 3);

    Eigen::Vector3d pt1(0.1, 0.2, 0.3);
    Eigen::Vector3d pt2(0.5, 0.4, 0.3);

    map.add_points(std::vector<Eigen::Vector3d>{pt1, pt2});

    // Since min_points_per_voxel is 3, these 2 points should not be exposed yet
    auto pts = map.pointcloud_vector();
    EXPECT_TRUE(pts.empty());

    Eigen::Vector3d pt3(0.3, 0.6, 0.6);
    map.add_points(std::vector<Eigen::Vector3d>{pt3});

    // Now we should have 1 point which is the average of pt1, pt2, and pt3
    pts = map.pointcloud_vector();
    ASSERT_EQ(pts.size(), 1u);

    Eigen::Vector3d expected_avg = (pt1 + pt2 + pt3) / 3.0;
    EXPECT_NEAR(pts[0].x(), expected_avg.x(), 1e-9);
    EXPECT_NEAR(pts[0].y(), expected_avg.y(), 1e-9);
    EXPECT_NEAR(pts[0].z(), expected_avg.z(), 1e-9);
}

TEST(VoxelHashMapTest, AverageVoxelHashMapXdCorrectlyAveragesAndRespectsThreshold) {
    using ouster::sdk::core::AverageVoxelHashMapXd;

    AverageVoxelHashMapXd map(1.0, 100.0, 20, 2, AttributeLayout::all_double(2));

    Eigen::VectorXd pt1(5);
    pt1 << 0.1, 0.2, 0.3, 10.0, 20.0;

    map.add_point(pt1);

    // min_points_per_voxel is 2, so 1 point should be hidden
    auto pts = map.pointcloud_vector();
    EXPECT_TRUE(pts.empty());

    Eigen::VectorXd pt2(5);
    pt2 << 0.5, 0.4, 0.3, 30.0, 40.0;

    map.add_point(pt2);

    // Now threshold is reached
    pts = map.pointcloud_vector();
    ASSERT_EQ(pts.size(), 1u);

    Eigen::VectorXd expected_avg(5);
    expected_avg << 0.3, 0.3, 0.3, 20.0, 30.0;
    EXPECT_EQ(pts[0].size(), 5);
    for (int i = 0; i < 5; ++i) {
        EXPECT_NEAR(pts[0][i], expected_avg[i], 1e-9);
    }
}

TEST(VoxelHashMapTest, VoxelDownsample3dRandomK1VariesAcrossShuffles) {
    using ouster::sdk::core::voxel_downsample_3d;
    using ouster::sdk::core::VoxelDownsampleStrategy;

    ArrayX3dR frame(50, 3);
    for (int i = 0; i < 50; ++i) {
        frame.row(i) << 1.0 + 0.01 * i, 1.0, 1.0;
    }

    std::mt19937 shuffle_rng(5678);
    std::unordered_set<int> kept_indices;
    for (int trial = 0; trial < 20; ++trial) {
        ArrayX3dR shuffled = frame;
        std::vector<int> order(shuffled.rows());
        std::iota(order.begin(), order.end(), 0);
        std::shuffle(order.begin(), order.end(), shuffle_rng);
        for (int r = 0; r < shuffled.rows(); ++r) {
            shuffled.row(r) = frame.row(order[r]);
        }

        auto out = voxel_downsample_3d(shuffled, /*voxel_size=*/10.0, /*max_points_per_voxel=*/1,
                                       /*min_points_per_voxel=*/1, VoxelDownsampleStrategy::RANDOM);
        ASSERT_EQ(out.rows(), 1);
        for (int i = 0; i < frame.rows(); ++i) {
            if ((out.row(0).transpose() - frame.row(i).transpose()).matrix().norm() < 1e-4) {
                kept_indices.insert(i);
                break;
            }
        }
    }
    EXPECT_GT(kept_indices.size(), 1u);
}

TEST(VoxelHashMapTest, LastNPointEvictsTheOldestOnceFull) {
    using namespace ouster::sdk::core;
    // voxel_size=10, max_points_per_voxel=2 -> near-duplicate rejection radius
    // is voxel_size / sqrt(max_points_per_voxel) ~= 7.07, so points placed near
    // the four corners of one 10-unit voxel (pairwise distance >= 9) are never
    // rejected as near-duplicates of one another.
    LastNPointVoxelHashMap3d map(10.0, VoxelHashMap::DEFAULT_MAX_DISTANCE,
                                 /*max_points_per_voxel=*/2,
                                 /*min_points_per_voxel=*/1, AttributeLayout(),
                                 VHMInternalStorageMode::ABS);
    const Eigen::Vector3d p0(0.5, 0.5, 0.5);
    const Eigen::Vector3d p1(9.5, 9.5, 9.5);
    const Eigen::Vector3d p2(0.5, 9.5, 0.5);
    const Eigen::Vector3d p3(9.5, 0.5, 9.5);
    const Eigen::Vector3d p4(0.5, 0.5, 9.5);

    auto kept = [&]() {
        std::vector<Eigen::Vector3d> pts = map.pointcloud_vector();
        std::sort(pts.begin(), pts.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return std::tie(a.x(), a.y(), a.z()) < std::tie(b.x(), b.y(), b.z());
        });
        return pts;
    };
    auto expect_kept = [&](std::vector<Eigen::Vector3d> expected) {
        std::sort(expected.begin(), expected.end(),
                  [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                      return std::tie(a.x(), a.y(), a.z()) < std::tie(b.x(), b.y(), b.z());
                  });
        const std::vector<Eigen::Vector3d> actual = kept();
        ASSERT_EQ(actual.size(), expected.size());
        for (std::size_t i = 0; i < actual.size(); ++i) {
            EXPECT_LT((actual[i] - expected[i]).norm(), 1e-6);
        }
    };

    map.add_point(p0);
    map.add_point(p1);
    expect_kept({p0, p1});  // Not yet full: both admitted normally.

    // Full: p2 evicts the oldest kept point (p0).
    map.add_point(p2);
    expect_kept({p1, p2});

    // Still full: p3 evicts the next-oldest (p1).
    map.add_point(p3);
    expect_kept({p2, p3});

    // p4 evicts p2, again the oldest kept point.
    map.add_point(p4);
    expect_kept({p3, p4});
}

TEST(VoxelHashMapTest, LastNPointKeepsLastAdmittedWhenDuplicatesAreInterspersed) {
    using namespace ouster::sdk::core;
    // A rejected near-duplicate must not disturb which point is evicted next:
    // the voxel always holds exactly the last max_points_per_voxel points that
    // passed the near-duplicate check.
    LastNPointVoxelHashMap3d map(10.0, VoxelHashMap::DEFAULT_MAX_DISTANCE,
                                 /*max_points_per_voxel=*/2,
                                 /*min_points_per_voxel=*/1, AttributeLayout(),
                                 VHMInternalStorageMode::ABS);
    const Eigen::Vector3d p0(0.5, 0.5, 0.5);
    const Eigen::Vector3d p1(9.5, 9.5, 9.5);
    const Eigen::Vector3d dup_of_p0(0.6, 0.5, 0.5);
    const Eigen::Vector3d p2(0.5, 9.5, 0.5);
    const Eigen::Vector3d p3(9.5, 0.5, 9.5);

    map.add_point(p0);
    map.add_point(p1);
    map.add_point(dup_of_p0);  // Rejected, but still counted by bucket.count.
    map.add_point(p2);         // Must evict p0 (oldest), not p1.

    auto expect_kept = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        const std::vector<Eigen::Vector3d> pts = map.pointcloud_vector();
        ASSERT_EQ(pts.size(), 2u);
        for (const Eigen::Vector3d& want : {a, b}) {
            bool found = false;
            for (const auto& p : pts) {
                if ((p - want).norm() < 1e-6) found = true;
            }
            EXPECT_TRUE(found) << want.transpose();
        }
    };
    expect_kept(p1, p2);

    map.add_point(Eigen::Vector3d(9.6, 9.5, 9.5));  // Near-duplicate of p1: dropped.
    map.add_point(p3);                              // Evicts p1, the oldest.
    expect_kept(p2, p3);
}

TEST(VoxelHashMapTest, LastNPointStillRejectsNearDuplicates) {
    using namespace ouster::sdk::core;
    // Unlike the corner points above, a point within the resolution radius of
    // one already kept must still be dropped, not cause an eviction: LAST
    // changes what happens once the voxel is full and a genuinely new point
    // arrives, not the existing near-duplicate filter.
    LastNPointVoxelHashMap3d map(10.0, VoxelHashMap::DEFAULT_MAX_DISTANCE,
                                 /*max_points_per_voxel=*/2,
                                 /*min_points_per_voxel=*/1, AttributeLayout(),
                                 VHMInternalStorageMode::ABS);
    map.add_point(Eigen::Vector3d(0.5, 0.5, 0.5));
    map.add_point(Eigen::Vector3d(9.5, 9.5, 9.5));
    // Within ~7.07 of the first point, so rejected as a near-duplicate; the
    // bucket is full, but this must not consume an eviction slot.
    map.add_point(Eigen::Vector3d(0.6, 0.5, 0.5));

    const std::vector<Eigen::Vector3d> pts = map.pointcloud_vector();
    ASSERT_EQ(pts.size(), 2u);
    bool saw_first = false;
    for (const auto& p : pts) {
        if ((p - Eigen::Vector3d(0.5, 0.5, 0.5)).norm() < 1e-6) saw_first = true;
    }
    EXPECT_TRUE(saw_first);
}

// ---------------------------------------------------------------------------
// AttributeLayout - packing, unpacking and the conversions it performs
// ---------------------------------------------------------------------------
TEST(AttributeLayoutTest, TypeNamesAndSizesFollowTheCppType) {
    using namespace ouster::sdk::core;
    const std::vector<std::pair<std::string, std::size_t>> expected = {
        {"u8", 1},  {"u16", 2}, {"u32", 4}, {"u64", 8}, {"i8", 1}, {"i16", 2},
        {"i32", 4}, {"i64", 8}, {"f16", 2}, {"f32", 4}, {"f64", 8}};
    ASSERT_EQ(sizeof(ALL_ATTRIBUTE_TYPES) / sizeof(ALL_ATTRIBUTE_TYPES[0]), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(attribute_type_name(ALL_ATTRIBUTE_TYPES[i]), expected[i].first);
        EXPECT_EQ(attribute_type_size(ALL_ATTRIBUTE_TYPES[i]), expected[i].second);
    }
    EXPECT_THROW(attribute_type_name(static_cast<AttributeType>(0xff)), std::invalid_argument);
}

TEST(AttributeLayoutTest, PacksWithoutPadding) {
    using namespace ouster::sdk::core;
    AttributeLayout layout({{AttributeType::U32, 1},
                            {AttributeType::U8, 1},
                            {AttributeType::F32, 3},
                            {AttributeType::U16, 1}});

    EXPECT_EQ(layout.stride(), 4u + 1u + 12u + 2u);
    EXPECT_EQ(layout.element_count(), 6u);
    EXPECT_EQ(layout.byte_offset(0), 0u);
    EXPECT_EQ(layout.byte_offset(1), 4u);
    EXPECT_EQ(layout.byte_offset(2), 5u);
    EXPECT_EQ(layout.byte_offset(3), 17u);
    EXPECT_EQ(layout.element_offset(2), 2u);
    EXPECT_EQ(layout.byte_size(2), 12u);
    // Every per-attribute accessor checks its index.
    EXPECT_THROW(layout.byte_offset(4), std::out_of_range);
    EXPECT_THROW(layout.byte_size(4), std::out_of_range);
    EXPECT_THROW(layout.element_offset(4), std::out_of_range);
    EXPECT_FALSE(layout.all_double());
    EXPECT_TRUE(AttributeLayout::all_double(4).all_double());
    EXPECT_EQ(AttributeLayout::all_double(4).stride(), 32u);
}

TEST(AttributeLayoutTest, RoundTripsValuesExactly) {
    using namespace ouster::sdk::core;
    AttributeLayout layout(
        {{AttributeType::U32, 1}, {AttributeType::U8, 1}, {AttributeType::U64, 1}});

    // 4294967295 and 9007199254740993 both survive here but would not survive a
    // trip through float, which is what these attributes used to be written as.
    const double values[3] = {4294967295.0, 255.0, 9007199254740992.0};
    std::vector<std::uint8_t> record(layout.stride());
    layout.encode(values, record.data());

    double decoded[3] = {0, 0, 0};
    layout.decode(record.data(), decoded);
    EXPECT_EQ(decoded[0], values[0]);
    EXPECT_EQ(decoded[1], values[1]);
    EXPECT_EQ(decoded[2], values[2]);

    layout.decode_add(record.data(), decoded);
    EXPECT_EQ(decoded[0], 2 * values[0]);
}

TEST(AttributeLayoutTest, RoundsAndClampsIntegerAttributes) {
    using namespace ouster::sdk::core;
    AttributeLayout layout({{AttributeType::U8, 1}, {AttributeType::I8, 1}});
    std::vector<std::uint8_t> record(layout.stride());

    const double over[2] = {300.0, -300.0};
    layout.encode(over, record.data());
    double decoded[2];
    layout.decode(record.data(), decoded);
    EXPECT_EQ(decoded[0], 255.0);
    EXPECT_EQ(decoded[1], -128.0);

    const double fractional[2] = {2.6, -2.6};
    layout.encode(fractional, record.data());
    layout.decode(record.data(), decoded);
    EXPECT_EQ(decoded[0], 3.0);
    EXPECT_EQ(decoded[1], -3.0);
}

TEST(AttributeLayoutTest, F16AttributesEncodeAndDecodeThroughTheDoubleApi) {
    using namespace ouster::sdk::core;
    AttributeLayout layout({{AttributeType::F16, 1}});
    EXPECT_EQ(attribute_type_size(AttributeType::F16), 2u);
    EXPECT_EQ(attribute_type_name(AttributeType::F16), "f16");
    EXPECT_EQ(layout.stride(), 2u);

    // Values exactly representable in binary16 round-trip exactly.
    const double exact[] = {1.5, -0.25, 0.0, 2048.0};
    for (const double value : exact) {
        std::vector<std::uint8_t> record(layout.stride());
        layout.encode(&value, record.data());
        double decoded = 0.0;
        layout.decode(record.data(), &decoded);
        EXPECT_EQ(decoded, value) << "value=" << value;
    }

    // A value with more precision than binary16 holds (11-bit mantissa) is
    // quantized on the way in -- proof this actually goes through half
    // precision rather than being stored as a wider float.
    const double precise = 1.0 / 3.0;
    std::vector<std::uint8_t> record(layout.stride());
    layout.encode(&precise, record.data());
    double decoded = 0.0;
    layout.decode(record.data(), &decoded);
    EXPECT_NE(decoded, precise);
    EXPECT_NEAR(decoded, precise, 1e-3);
    // Quantization happens once: encoding the already-quantized value again
    // reproduces the same bits.
    std::vector<std::uint8_t> record2(layout.stride());
    layout.encode(&decoded, record2.data());
    EXPECT_EQ(std::memcmp(record.data(), record2.data(), record.size()), 0);

    // decode_add() sums in double, same as every other attribute type.
    double sum = 0.0;
    layout.decode_add(record.data(), &sum);
    layout.decode_add(record.data(), &sum);
    EXPECT_EQ(sum, 2 * decoded);
}

TEST(AttributeLayoutTest, F16AttributesPackAndUnpackWithoutConversion) {
    using namespace ouster::sdk::core;
    AttributeLayout layout({{AttributeType::U8, 1}, {AttributeType::F16, 1}});
    AttributeRecords records =
        AttributeRecords::Zero(3, static_cast<Eigen::Index>(layout.stride()));

    // pack()/unpack() never convert -- unlike encode()/decode(), the exact
    // half bits given in survive untouched, whatever they are.
    const std::vector<Eigen::half> values = {Eigen::half(1.0f / 3.0f), Eigen::half(-2.5f),
                                             Eigen::half(0.0f)};
    layout.pack(1, values, records);
    const std::vector<Eigen::half> back = layout.unpack<Eigen::half>(1, records);
    ASSERT_EQ(back.size(), values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
        EXPECT_EQ(static_cast<float>(back[i]), static_cast<float>(values[i]));
    }

    // Same type, same bit width, still rejected: pack()/unpack() check the
    // exact stored type, not just the size.
    const std::vector<std::uint16_t> wrong_type = {0, 1, 2};
    EXPECT_THROW(layout.pack(1, wrong_type, records), std::invalid_argument);
    EXPECT_THROW(layout.unpack<std::uint16_t>(1, records), std::invalid_argument);
    EXPECT_THROW(layout.unpack<float>(1, records), std::invalid_argument);
}

TEST(AttributedPointCloudTest, F16AttributeAccessibleAsEigenHalf) {
    using namespace ouster::sdk::core;
    AttributedPointCloud cloud;
    cloud.layout = AttributeLayout({{AttributeType::F16, 1}, {AttributeType::U8, 1}});
    cloud.xyz = ArrayX3dR::Zero(2, 3);
    cloud.attributes = AttributeRecords::Zero(2, static_cast<Eigen::Index>(cloud.layout.stride()));
    const Eigen::half signal[2] = {Eigen::half(1.5f), Eigen::half(-3.25f)};
    const std::uint8_t tag[2] = {9u, 200u};
    for (Eigen::Index i = 0; i < 2; ++i) {
        std::uint8_t* rec = cloud.attributes.row(i).data();
        std::memcpy(rec + cloud.layout.byte_offset(0), &signal[i], 2);
        std::memcpy(rec + cloud.layout.byte_offset(1), &tag[i], 1);
    }

    const std::vector<Eigen::half> got = cloud.attribute<Eigen::half>(0);
    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(static_cast<float>(got[0]), 1.5f);
    EXPECT_EQ(static_cast<float>(got[1]), -3.25f);

    struct Attrs {
        Eigen::half signal;
        std::uint8_t tag;
    };
    const std::vector<Attrs> attrs = cloud.attributes_as<Attrs>(&Attrs::signal, &Attrs::tag);
    ASSERT_EQ(attrs.size(), 2u);
    EXPECT_EQ(static_cast<float>(attrs[0].signal), 1.5f);
    EXPECT_EQ(attrs[1].tag, 200u);

    // A same-sized but different type is still rejected.
    EXPECT_THROW(cloud.attribute<std::uint16_t>(0), std::invalid_argument);
    EXPECT_THROW(cloud.attribute<float>(0), std::invalid_argument);
}

TEST(AttributeLayoutTest, PacksAndUnpacksTypedValues) {
    using namespace ouster::sdk::core;
    AttributeLayout layout({{AttributeType::U32, 1}, {AttributeType::U16, 2}});
    constexpr Eigen::Index rows = 4;
    const auto stride = static_cast<Eigen::Index>(layout.stride());
    AttributeRecords records = AttributeRecords::Zero(rows, stride);

    // A multi-element attribute is count values per point, point by point.
    const std::vector<std::uint32_t> ranges = {1u, 70000u, 4000000000u, 7u};
    const std::vector<std::uint16_t> pairs = {1, 2, 3, 4, 5, 6, 7, 8};
    layout.pack(0, ranges, records);
    layout.pack(1, pairs, records);
    EXPECT_EQ(layout.unpack<std::uint32_t>(0, records), ranges);
    EXPECT_EQ(layout.unpack<std::uint16_t>(1, records), pairs);

    // The pointer overloads behave the same.
    std::uint16_t out_pairs[rows * 2] = {};
    layout.unpack(1, records, out_pairs, rows * 2);
    EXPECT_EQ(out_pairs[7], 8);

    // Packing one attribute leaves the other attributes' bytes alone.
    const std::uint32_t zeros[rows] = {};
    layout.pack(0, zeros, rows, records);
    EXPECT_EQ(layout.unpack<std::uint16_t>(1, records), pairs);

    // Records may be a view into a wider buffer (row pitch > stride).
    AttributeRecords wide = AttributeRecords::Zero(rows, stride + 3);
    layout.pack(0, ranges, wide.leftCols(stride));
    EXPECT_EQ(layout.unpack<std::uint32_t>(0, wide.leftCols(stride)), ranges);
}

namespace {
// A {U32, U8, F32 x 3} cloud of three points whose values break a float or
// double intermediate, built directly rather than through a map.
ouster::sdk::core::AttributedPointCloud accessor_test_cloud() {
    using namespace ouster::sdk::core;
    AttributedPointCloud cloud;
    cloud.layout =
        AttributeLayout({{AttributeType::U32, 1}, {AttributeType::U8, 1}, {AttributeType::F32, 3}});
    cloud.xyz = ArrayX3dR::Zero(3, 3);
    cloud.attributes = AttributeRecords::Zero(3, static_cast<Eigen::Index>(cloud.layout.stride()));
    const std::uint32_t ranges[3] = {4294967295u, 16777217u, 0u};
    const std::uint8_t reflectivity[3] = {255u, 128u, 7u};
    const float rgb[3][3] = {{0.1f, 0.2f, 0.3f}, {1.f, 2.f, 3.f}, {-1.f, 0.f, 1e30f}};
    for (Eigen::Index i = 0; i < 3; ++i) {
        std::uint8_t* rec = cloud.attributes.row(i).data();
        std::memcpy(rec + cloud.layout.byte_offset(0), &ranges[i], 4);
        std::memcpy(rec + cloud.layout.byte_offset(1), &reflectivity[i], 1);
        std::memcpy(rec + cloud.layout.byte_offset(2), rgb[i], 12);
    }
    return cloud;
}
}  // namespace

TEST(AttributedPointCloudTest, AttributeReturnsEachColumnInItsStoredType) {
    using namespace ouster::sdk::core;
    const AttributedPointCloud cloud = accessor_test_cloud();
    EXPECT_EQ(cloud.size(), 3u);
    EXPECT_EQ(cloud.attribute<std::uint32_t>(0),
              (std::vector<std::uint32_t>{4294967295u, 16777217u, 0u}));
    EXPECT_EQ(cloud.attribute<std::uint8_t>(1), (std::vector<std::uint8_t>{255u, 128u, 7u}));
    // A multi-element attribute comes back point by point, count values each.
    EXPECT_EQ(cloud.attribute<float>(2),
              (std::vector<float>{0.1f, 0.2f, 0.3f, 1.f, 2.f, 3.f, -1.f, 0.f, 1e30f}));

    // The pointer overload fills a caller-owned buffer the same way.
    std::uint32_t ranges[3] = {};
    cloud.attribute(0, ranges, 3);
    EXPECT_EQ(ranges[0], 4294967295u);
    EXPECT_EQ(ranges[1], 16777217u);
    // ... and checks the buffer holds exactly size() * count values.
    EXPECT_THROW(cloud.attribute(0, ranges, 2), std::invalid_argument);
    float rgb[9] = {};
    EXPECT_THROW(cloud.attribute(2, rgb, 3), std::invalid_argument);
    EXPECT_NO_THROW(cloud.attribute(2, rgb, 9));
}

TEST(AttributeLayoutTest, PackAndUnpackRejectWrongTypesAndSizes) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout({{AttributeType::U32, 1}, {AttributeType::F32, 3}});
    AttributeRecords records =
        AttributeRecords::Zero(2, static_cast<Eigen::Index>(layout.stride()));

    // Same-sized but different type.
    EXPECT_THROW(layout.pack(0, std::vector<float>{1.f, 2.f}, records), std::invalid_argument);
    EXPECT_THROW(layout.pack(0, std::vector<std::int32_t>{1, 2}, records), std::invalid_argument);
    EXPECT_THROW(layout.unpack<double>(1, records), std::invalid_argument);
    // Wrong number of values: one per record for a scalar, three for NORMALS.
    EXPECT_THROW(layout.pack(0, std::vector<std::uint32_t>{1u}, records), std::invalid_argument);
    EXPECT_THROW(layout.pack(1, std::vector<float>{1.f, 2.f}, records), std::invalid_argument);
    float normals[6] = {};
    EXPECT_THROW(layout.unpack(1, records, normals, 5), std::invalid_argument);
    EXPECT_NO_THROW(layout.unpack(1, records, normals, 6));
    // spec_index out of range.
    EXPECT_THROW(layout.pack(2, std::vector<std::uint32_t>{1u, 2u}, records),
                 std::invalid_argument);
    EXPECT_THROW(layout.unpack<std::uint32_t>(2, records), std::invalid_argument);
    // Records that are not stride bytes wide.
    AttributeRecords narrow = AttributeRecords::Zero(2, 4);
    EXPECT_THROW(layout.pack(0, std::vector<std::uint32_t>{1u, 2u}, narrow), std::invalid_argument);
    EXPECT_THROW(layout.unpack<std::uint32_t>(0, narrow), std::invalid_argument);
    // Zero rows is fine.
    AttributeRecords none(0, static_cast<Eigen::Index>(layout.stride()));
    EXPECT_NO_THROW(layout.pack(0, std::vector<std::uint32_t>{}, none));
    EXPECT_TRUE(layout.unpack<float>(1, none).empty());
}

TEST(AttributedPointCloudTest, AccessorsRejectPositionsAndRecordsThatDisagree) {
    using namespace ouster::sdk::core;
#pragma pack(push, 1)
    struct Packed {
        std::uint32_t range;
        std::uint8_t reflectivity;
        float rgb[3];
    };
#pragma pack(pop)
    struct Attrs {
        std::uint32_t range;
        std::uint8_t reflectivity;
        float rgb[3];
    };
    auto expect_all_throw = [](const AttributedPointCloud& cloud) {
        EXPECT_THROW(cloud.attribute<std::uint32_t>(0), std::invalid_argument);
        std::uint32_t ranges[64] = {};
        EXPECT_THROW(cloud.attribute(0, ranges, cloud.size()), std::invalid_argument);
        EXPECT_THROW(cloud.attributes_as<Packed>(), std::invalid_argument);
        EXPECT_THROW(cloud.attributes_as<Attrs>(&Attrs::range, &Attrs::reflectivity, &Attrs::rgb),
                     std::invalid_argument);
    };

    // More records than positions: a packed copy sized from size() would overrun.
    AttributedPointCloud more_records = accessor_test_cloud();
    more_records.xyz = ArrayX3dR::Zero(1, 3);
    expect_all_throw(more_records);

    // Fewer records than positions: a field-by-field copy would read past them.
    AttributedPointCloud fewer_records = accessor_test_cloud();
    fewer_records.xyz = ArrayX3dR::Zero(64, 3);
    expect_all_throw(fewer_records);

    // Records wider than the layout stride.
    AttributedPointCloud wide_records = accessor_test_cloud();
    wide_records.attributes =
        AttributeRecords::Zero(3, static_cast<Eigen::Index>(wide_records.layout.stride()) + 1);
    expect_all_throw(wide_records);
}

TEST(AttributedPointCloudTest, AttributeRejectsAnyTypeButTheStoredOne) {
    using namespace ouster::sdk::core;
    const AttributedPointCloud cloud = accessor_test_cloud();
    // Same size, different type: still rejected rather than reinterpreted.
    EXPECT_THROW(cloud.attribute<float>(0), std::invalid_argument);
    EXPECT_THROW(cloud.attribute<std::int32_t>(0), std::invalid_argument);
    EXPECT_THROW(cloud.attribute<std::int8_t>(1), std::invalid_argument);
    EXPECT_THROW(cloud.attribute<std::uint64_t>(0), std::invalid_argument);
    EXPECT_THROW(cloud.attribute<double>(2), std::invalid_argument);
    EXPECT_THROW(cloud.attribute<std::uint32_t>(3), std::invalid_argument);
}

TEST(AttributedPointCloudTest, AttributesAsFillsAStructFieldByField) {
    using namespace ouster::sdk::core;
    const AttributedPointCloud cloud = accessor_test_cloud();

    // Natural (padded) struct, with an extra member the layout knows nothing of.
    struct Attrs {
        std::uint8_t unrelated;
        std::uint32_t range;
        std::uint8_t reflectivity;
        std::array<float, 3> rgb;
    };
    const auto attrs = cloud.attributes_as<Attrs>(&Attrs::range, &Attrs::reflectivity, &Attrs::rgb);
    ASSERT_EQ(attrs.size(), 3u);
    EXPECT_EQ(attrs[0].range, 4294967295u);
    EXPECT_EQ(attrs[1].range, 16777217u);
    EXPECT_EQ(attrs[0].reflectivity, 255u);
    EXPECT_EQ(attrs[2].reflectivity, 7u);
    EXPECT_EQ(attrs[1].rgb, (std::array<float, 3>{1.f, 2.f, 3.f}));
    EXPECT_EQ(attrs[2].rgb[2], 1e30f);

    // C arrays work as multi-element fields too.
    struct CArrayAttrs {
        std::uint32_t range;
        std::uint8_t reflectivity;
        float rgb[3];
    };
    const auto c_attrs = cloud.attributes_as<CArrayAttrs>(
        &CArrayAttrs::range, &CArrayAttrs::reflectivity, &CArrayAttrs::rgb);
    EXPECT_EQ(c_attrs[2].rgb[0], -1.f);
}

TEST(AttributedPointCloudTest, AttributesAsRejectsFieldsThatDoNotMatchTheLayout) {
    using namespace ouster::sdk::core;
    const AttributedPointCloud cloud = accessor_test_cloud();
    struct Attrs {
        std::uint32_t range;
        std::int32_t signed_range;
        std::uint8_t reflectivity;
        float rgb[3];
        float rg[2];
    };
    // Too few fields.
    EXPECT_THROW(cloud.attributes_as<Attrs>(&Attrs::range, &Attrs::reflectivity),
                 std::invalid_argument);
    // Wrong order: the {U32, U8} pair swapped.
    EXPECT_THROW(cloud.attributes_as<Attrs>(&Attrs::reflectivity, &Attrs::range, &Attrs::rgb),
                 std::invalid_argument);
    // Same size, different type.
    EXPECT_THROW(
        cloud.attributes_as<Attrs>(&Attrs::signed_range, &Attrs::reflectivity, &Attrs::rgb),
        std::invalid_argument);
    // Wrong element count.
    EXPECT_THROW(cloud.attributes_as<Attrs>(&Attrs::range, &Attrs::reflectivity, &Attrs::rg),
                 std::invalid_argument);
}

TEST(AttributedPointCloudTest, AttributesAsCopiesRecordsIntoAPackedStruct) {
    using namespace ouster::sdk::core;
    const AttributedPointCloud cloud = accessor_test_cloud();
#pragma pack(push, 1)
    struct Packed {
        std::uint32_t range;
        std::uint8_t reflectivity;
        float rgb[3];
    };
#pragma pack(pop)
    static_assert(sizeof(Packed) == 17, "packed to the record size");
    const auto packed = cloud.attributes_as<Packed>();
    ASSERT_EQ(packed.size(), 3u);
    EXPECT_EQ(packed[0].range, 4294967295u);
    EXPECT_EQ(packed[1].reflectivity, 128u);
    EXPECT_EQ(packed[1].rgb[2], 3.f);

    // An unpacked struct is wider than a record and is rejected.
    struct Unpacked {
        std::uint32_t range;
        std::uint8_t reflectivity;
        float rgb[3];
    };
    EXPECT_THROW(cloud.attributes_as<Unpacked>(), std::invalid_argument);

    // An empty cloud decodes to nothing.
    AttributedPointCloud empty;
    empty.layout = cloud.layout;
    empty.attributes.resize(0, static_cast<Eigen::Index>(cloud.layout.stride()));
    EXPECT_TRUE(empty.attributes_as<Packed>().empty());
    EXPECT_TRUE(empty.attribute<std::uint32_t>(0).empty());
}

// ---------------------------------------------------------------------------
// VoxelHashMapT with a typed attribute layout
// ---------------------------------------------------------------------------
namespace {

// One point's attributes for the {U32, U8, F32} layout used by the tests below.
struct TypedAttribs {
    std::uint32_t range;
    std::uint8_t reflectivity;
    float signal;
};

ouster::sdk::core::AttributeLayout typed_layout() {
    using namespace ouster::sdk::core;
    return AttributeLayout(
        {{AttributeType::U32, 1}, {AttributeType::U8, 1}, {AttributeType::F32, 1}});
}

ouster::sdk::core::AttributeRecords encode_typed(const ouster::sdk::core::AttributeLayout& layout,
                                                 const std::vector<TypedAttribs>& attribs) {
    std::vector<std::uint32_t> ranges;
    std::vector<std::uint8_t> reflectivity;
    std::vector<float> signal;
    for (const TypedAttribs& a : attribs) {
        ranges.push_back(a.range);
        reflectivity.push_back(a.reflectivity);
        signal.push_back(a.signal);
    }
    ouster::sdk::core::AttributeRecords records(static_cast<Eigen::Index>(attribs.size()),
                                                static_cast<Eigen::Index>(layout.stride()));
    layout.pack(0, ranges, records);
    layout.pack(1, reflectivity, records);
    layout.pack(2, signal, records);
    return records;
}

// Every point's attributes of a cloud with typed_layout().
std::vector<TypedAttribs> decode_typed(const ouster::sdk::core::AttributedPointCloud& cloud) {
    return cloud.attributes_as<TypedAttribs>(&TypedAttribs::range, &TypedAttribs::reflectivity,
                                             &TypedAttribs::signal);
}

}  // namespace

TEST(VoxelHashMapTest, RejectsMismatchedRecordsSize) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    ASSERT_EQ(layout.stride(), 9u);
    VoxelHashMapXd map(1.0, VoxelHashMapXd::DEFAULT_MAX_DISTANCE, 10, 1, layout);

    ArrayX3dR xyz(2, 3);
    xyz << 0.05, 0.05, 0.05,  //
        1.05, 0.05, 0.05;

    // add_points: too few and too many bytes for 2 points * 9 bytes/point.
    // Wrong width, wrong row count, and empty records are all rejected.
    EXPECT_THROW(map.add_points(xyz, AttributeRecords::Zero(2, 8)), std::invalid_argument);
    EXPECT_THROW(map.add_points(xyz, AttributeRecords::Zero(2, 10)), std::invalid_argument);
    EXPECT_THROW(map.add_points(xyz, AttributeRecords::Zero(1, 9)), std::invalid_argument);
    EXPECT_THROW(map.add_points(xyz, AttributeRecords()), std::invalid_argument);
    // Exactly the right size still works.
    EXPECT_NO_THROW(map.add_points(xyz, AttributeRecords::Zero(2, 9)));
    // A view over an existing buffer binds without a copy, even when its rows
    // are wider than a record (here: the leftmost 9 of 12 bytes per row).
    std::vector<std::uint8_t> buffer(2 * 12, 0);
    const Eigen::Map<const AttributeRecords> wide(buffer.data(), 2, 12);
    EXPECT_NO_THROW(map.add_points(xyz, wide.leftCols(9)));

    // add_point: a single record must be exactly one stride.
    const Eigen::Vector3d pt(2.05, 0.05, 0.05);
    EXPECT_THROW(map.add_point(pt, AttributeRecord::Zero(8)), std::invalid_argument);
    EXPECT_THROW(map.add_point(pt, AttributeRecord::Zero(10)), std::invalid_argument);
    EXPECT_NO_THROW(map.add_point(pt, AttributeRecord::Zero(9)));

    // A map without attributes takes empty records.
    VoxelHashMapXd plain(1.0, VoxelHashMapXd::DEFAULT_MAX_DISTANCE, 10, 1, AttributeLayout());
    EXPECT_NO_THROW(plain.add_points(xyz, AttributeRecords()));
    EXPECT_NO_THROW(plain.add_points(xyz, AttributeRecords(2, 0)));
    EXPECT_THROW(plain.add_points(xyz, AttributeRecords::Zero(2, 1)), std::invalid_argument);
    EXPECT_NO_THROW(plain.add_point(pt, AttributeRecord()));
    EXPECT_EQ(plain.pointcloud_double().rows(), 3);
}

TEST(VoxelHashMapTest, TypedAttributesSurviveTheMapExactly) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    EXPECT_EQ(layout.stride(), 9u);

    // 4294967295 is not representable as a float and 16777217 is the first
    // integer a float cannot hold, so both would be corrupted by the old
    // "every attribute is a double, written as a float" path.
    const std::vector<TypedAttribs> attribs = {
        {4294967295u, 255u, 1.5f}, {16777217u, 1u, -2.25f}, {0u, 0u, 0.0f}, {12345u, 42u, 7.5f}};
    ArrayX3dR xyz(4, 3);
    xyz << 0.05, 0.05, 0.05,  //
        1.05, 0.05, 0.05,     //
        2.05, 0.05, 0.05,     //
        3.05, 0.05, 0.05;

    for (const StorageConfig& config : g_storage_configs) {
        VoxelHashMapXd map(config.def_voxel_size, VoxelHashMapXd::DEFAULT_MAX_DISTANCE,
                           /*max_points_per_voxel=*/10, /*min_points_per_voxel=*/1, layout,
                           config.mode);
        EXPECT_EQ(map.point_cols(), 6);

        const AttributeRecords records = encode_typed(layout, attribs);
        map.add_points(xyz, records);

        const AttributedPointCloud out = map.pointcloud_attributed();
        ASSERT_EQ(out.xyz.rows(), 4);
        ASSERT_EQ(out.attributes.rows(), 4);
        ASSERT_EQ(out.attributes.cols(), static_cast<Eigen::Index>(layout.stride()));

        // Match each output row back to its input by position, then compare the
        // attributes bit-for-bit.
        const std::vector<TypedAttribs> decoded = decode_typed(out);
        for (Eigen::Index row = 0; row < out.xyz.rows(); ++row) {
            const TypedAttribs& got = decoded[static_cast<std::size_t>(row)];
            bool matched = false;
            for (std::size_t i = 0; i < attribs.size(); ++i) {
                if (std::abs(out.xyz(row, 0) - xyz(static_cast<Eigen::Index>(i), 0)) <=
                    config.tol_abs) {
                    EXPECT_EQ(got.range, attribs[i].range);
                    EXPECT_EQ(got.reflectivity, attribs[i].reflectivity);
                    EXPECT_EQ(got.signal, attribs[i].signal);
                    matched = true;
                    break;
                }
            }
            EXPECT_TRUE(matched);
        }
    }
}

TEST(VoxelHashMapTest, TypedAttributesAreVisibleThroughTheDoubleApi) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    const std::vector<TypedAttribs> attribs = {{12345u, 42u, 7.5f}};
    ArrayX3dR xyz(1, 3);
    xyz << 0.05, 0.05, 0.05;

    VoxelHashMapXd map(1.0, VoxelHashMapXd::DEFAULT_MAX_DISTANCE, 10, 1, layout,
                       VHMInternalStorageMode::ABS);
    const AttributeRecords records = encode_typed(layout, attribs);
    map.add_points(xyz, records);

    // pointcloud_double() and get_closest_neighbor() keep speaking in doubles, so
    // existing callers (e.g. registration) see the attributes
    // exactly as they did when everything was stored as a double.
    const ArrayXXdR cloud = map.pointcloud_double();
    ASSERT_EQ(cloud.rows(), 1);
    ASSERT_EQ(cloud.cols(), 6);
    EXPECT_EQ(cloud(0, 3), 12345.0);
    EXPECT_EQ(cloud(0, 4), 42.0);
    EXPECT_EQ(cloud(0, 5), 7.5);

    Eigen::VectorXd query = Eigen::VectorXd::Zero(6);
    query.head<3>() = Eigen::Vector3d(0.05, 0.05, 0.05);
    Eigen::VectorXd neighbor;
    double dist_sq = 0;
    std::tie(neighbor, dist_sq) = map.get_closest_neighbor(query);
    ASSERT_EQ(neighbor.size(), 6);
    EXPECT_EQ(neighbor(3), 12345.0);
    EXPECT_EQ(neighbor(4), 42.0);
    EXPECT_EQ(neighbor(5), 7.5);
}

TEST(VoxelHashMapTest, TypedAttributesAreRoundedWhenAveraged) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    // Two points in one voxel whose ranges average to 10.5; the integer
    // attribute has to come back rounded, not truncated or wrapped.
    const std::vector<TypedAttribs> attribs = {{10u, 4u, 1.0f}, {11u, 5u, 2.0f}};
    ArrayX3dR xyz(2, 3);
    xyz << 0.10, 0.10, 0.10,  //
        0.20, 0.20, 0.20;

    AverageVoxelHashMapXd map(1.0, AverageVoxelHashMapXd::DEFAULT_MAX_DISTANCE, 10, 1, layout,
                              VHMInternalStorageMode::ABS);
    const AttributeRecords records = encode_typed(layout, attribs);
    map.add_points(xyz, records);

    const AttributedPointCloud out = map.pointcloud_attributed();
    ASSERT_EQ(out.xyz.rows(), 1);
    ASSERT_EQ(out.layout, layout);
    const TypedAttribs got = decode_typed(out)[0];
    EXPECT_EQ(got.range, 11u);          // 10.5 rounded to nearest
    EXPECT_EQ(got.reflectivity, 5u);    // 4.5 rounded to nearest
    EXPECT_FLOAT_EQ(got.signal, 1.5f);  // float attributes keep the exact mean
}

TEST(VoxelHashMapTest, ExtractVoxelsFarFromLocationAttributed) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    const std::vector<TypedAttribs> attribs = {{1u, 1u, 1.0f}, {2u, 2u, 2.0f}};
    ArrayX3dR xyz(2, 3);
    xyz << 0.5, 0.5, 0.5,  //
        500.5, 0.5, 0.5;

    VoxelHashMapXd map(1.0, /*max_distance=*/10.0, 10, 1, layout, VHMInternalStorageMode::ABS);
    const AttributeRecords records = encode_typed(layout, attribs);
    map.add_points(xyz, records);

    Eigen::VectorXd origin = Eigen::VectorXd::Zero(6);
    const AttributedPointCloud evicted = map.extract_voxels_far_from_location_attributed(origin);
    ASSERT_EQ(evicted.xyz.rows(), 1);
    EXPECT_NEAR(evicted.xyz(0, 0), 500.5, 1e-9);
    ASSERT_EQ(evicted.layout, layout);
    EXPECT_EQ(decode_typed(evicted)[0].range, 2u);

    // The near voxel stays behind.
    const AttributedPointCloud remaining = map.pointcloud_attributed();
    ASSERT_EQ(remaining.xyz.rows(), 1);
    EXPECT_EQ(decode_typed(remaining)[0].range, 1u);

    // The output records are what add_points() takes, so they feed another
    // map with the same layout directly.
    VoxelHashMapXd copy(1.0, /*max_distance=*/10.0, 10, 1, layout, VHMInternalStorageMode::ABS);
    copy.add_points(evicted);
    const AttributedPointCloud copied = copy.pointcloud_attributed();
    ASSERT_EQ(copied.xyz.rows(), 1);
    EXPECT_EQ(copied.attributes, evicted.attributes);
    EXPECT_EQ(copied.layout, evicted.layout);

    // Same 9-byte stride, different types: the (xyz, records) overload can
    // only check the width, but the cloud overload also compares the layouts.
    const AttributeLayout reordered(
        {{AttributeType::F32, 1}, {AttributeType::U8, 1}, {AttributeType::U32, 1}});
    ASSERT_EQ(reordered.stride(), layout.stride());
    VoxelHashMapXd other(1.0, /*max_distance=*/10.0, 10, 1, reordered, VHMInternalStorageMode::ABS);
    EXPECT_THROW(other.add_points(evicted), std::invalid_argument);
    EXPECT_TRUE(other.empty());
}

TEST(VoxelHashMapTest, RejectsAttributeLayoutForFixedSizePointType) {
    using namespace ouster::sdk::core;
    EXPECT_THROW(VoxelHashMap3d(1.0, 100.0, 20, 1, AttributeLayout({{AttributeType::U8, 1}})),
                 std::invalid_argument);
}

TEST(VoxelHashMapTest, BucketsWithoutAttributesKeepTheirFootprint) {
    using namespace ouster::sdk::core;
    // VoxelHashMap3d's buckets cannot carry attributes, so they must not pay
    // for the attribute-record storage that the VectorXd buckets need.
    using Bucket3d = DefaultVoxelBucketTarget<Eigen::Vector3d, VHMInternalStorageMode::FX8>;
    EXPECT_EQ(sizeof(Bucket3d),
              sizeof(std::vector<typename Bucket3d::StorageType>) + sizeof(std::size_t));
}

TEST(VoxelHashMapTest, RejectsAttributeLayoutForBucketsThatCannotStoreIt) {
    using namespace ouster::sdk::core;
    // PointNormalBucket spends the tail of each point on the normal, so it has
    // nowhere to keep an attribute record: accumulate_point_normal_strategy
    // would drop it while attribute_layout() still advertised attributes.
    EXPECT_THROW(PointNormalVoxelHashMap3d(1.0, 100.0, 1, 1, AttributeLayout::all_double(2)),
                 std::invalid_argument);
    EXPECT_THROW(
        PointNormalVoxelHashMap3d(1.0, 100.0, 1, 1, AttributeLayout({{AttributeType::U32, 1}})),
        std::invalid_argument);
    // An empty layout is still fine -- that is how voxel_downsample_with_normals
    // builds the map.
    EXPECT_NO_THROW(PointNormalVoxelHashMap3d(1.0, 100.0, 1, 1, AttributeLayout()));
}

TEST(VoxelHashMapTest, PointNormalMapReturnsFullSixColumnPoints) {
    using namespace ouster::sdk::core;
    using Point6d = PointNormalVoxelHashMap3d::point_type;
    static_assert(Point6d::SizeAtCompileTime == 6, "point-normal maps carry a fixed 6-vector");

    PointNormalVoxelHashMap3d map(1.0, 2.0, 1, 1);
    EXPECT_EQ(map.point_cols(), 6);

    // Two points in one voxel near the origin, one far away.
    Point6d p;
    p << 0.2, 0.2, 0.2, 0.0, 0.0, 1.0;
    map.add_point(p);
    p << 0.4, 0.4, 0.4, 0.0, 0.0, 1.0;
    map.add_point(p);
    p << 50.5, 0.5, 0.5, 1.0, 0.0, 0.0;
    map.add_point(p);

    // Every row must hold the averaged position and normal, with no row
    // overrunning into its neighbor.
    const ArrayXXdR cloud = map.pointcloud_double();
    ASSERT_EQ(cloud.rows(), 2);
    ASSERT_EQ(cloud.cols(), 6);
    bool saw_near = false;
    bool saw_far = false;
    for (Eigen::Index row = 0; row < cloud.rows(); ++row) {
        if (cloud(row, 0) < 1.0) {
            EXPECT_NEAR(cloud(row, 0), 0.3, 1e-12);
            EXPECT_NEAR(cloud(row, 5), 1.0, 1e-12);
            saw_near = true;
        } else {
            EXPECT_NEAR(cloud(row, 0), 50.5, 1e-12);
            EXPECT_NEAR(cloud(row, 3), 1.0, 1e-12);
            saw_far = true;
        }
    }
    EXPECT_TRUE(saw_near);
    EXPECT_TRUE(saw_far);

    Point6d query = Point6d::Zero();
    query.head<3>() << 0.3, 0.3, 0.3;
    Point6d neighbor;
    double dist_sq = 0;
    std::tie(neighbor, dist_sq) = map.get_closest_neighbor(query);
    EXPECT_NEAR(dist_sq, 0.0, 1e-12);
    EXPECT_NEAR(neighbor(5), 1.0, 1e-12);

    const ArrayXXdR far = map.extract_voxels_far_from_location_double(query);
    ASSERT_EQ(far.rows(), 1);
    ASSERT_EQ(far.cols(), 6);
    EXPECT_NEAR(far(0, 0), 50.5, 1e-12);
    EXPECT_NEAR(far(0, 3), 1.0, 1e-12);
    EXPECT_EQ(map.pointcloud_double().rows(), 1);
}

TEST(VoxelHashMapTest, PointNormalMapSkipsVoxelsWithCancellingNormals) {
    using namespace ouster::sdk::core;
    using Point6d = PointNormalVoxelHashMap3d::point_type;

    PointNormalVoxelHashMap3d map(1.0, 100.0, 1, 1);
    // Opposite normals in one voxel sum to zero, so that voxel emits no point;
    // the output must be sized for the one remaining voxel, not two.
    Point6d p;
    p << 0.2, 0.2, 0.2, 0.0, 0.0, 1.0;
    map.add_point(p);
    p << 0.4, 0.4, 0.4, 0.0, 0.0, -1.0;
    map.add_point(p);
    p << 5.5, 0.5, 0.5, 1.0, 0.0, 0.0;
    map.add_point(p);

    const ArrayXXdR cloud = map.pointcloud_double();
    ASSERT_EQ(cloud.rows(), 1);
    EXPECT_NEAR(cloud(0, 0), 5.5, 1e-12);
    EXPECT_NEAR(cloud(0, 3), 1.0, 1e-12);
    EXPECT_EQ(map.pointcloud_vector().size(), 1u);
    EXPECT_EQ(map.pointcloud_attributed().xyz.rows(), 1);

    Point6d origin = Point6d::Zero();
    origin.head<3>() << 200.0, 0.0, 0.0;
    const ArrayXXdR far = map.extract_voxels_far_from_location_double(origin);
    ASSERT_EQ(far.rows(), 1);
    EXPECT_NEAR(far(0, 0), 5.5, 1e-12);
    EXPECT_TRUE(map.empty());
}

TEST(VoxelDownsampleXdTest, AttributedOverloadKeepsAttributeTypes) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    // Two voxels: points 0 and 1 share one, point 2 sits alone in another.
    // 4294967295 is the largest uint32 -- it must come back bit-exact.
    const std::vector<TypedAttribs> attribs = {
        {4294967295u, 250u, 0.25f}, {4294967293u, 4u, 0.75f}, {7u, 9u, 1.5f}};
    ArrayX3dR xyz(3, 3);
    xyz << 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 5.1, 5.1, 5.1;
    const AttributeRecords records = encode_typed(layout, attribs);

    auto by_x = [](const AttributedPointCloud& cloud, double x) {
        for (Eigen::Index row = 0; row < cloud.xyz.rows(); ++row) {
            if (std::abs(cloud.xyz(row, 0) - x) < 0.3) return row;
        }
        return Eigen::Index{-1};
    };

    // FIRST_N_POINT keeps the first point of each voxel, record untouched.
    const AttributedPointCloud first = voxel_downsample_xd(
        xyz, 1.0, 1, 1, VoxelDownsampleStrategy::FIRST_N_POINT, layout, records);
    ASSERT_EQ(first.layout, layout);
    ASSERT_EQ(first.xyz.rows(), 2);
    ASSERT_EQ(first.attributes.cols(), static_cast<Eigen::Index>(layout.stride()));
    const Eigen::Index near_first = by_x(first, 0.1);
    ASSERT_GE(near_first, 0);
    EXPECT_EQ(first.attributes.row(near_first), records.row(0));
    EXPECT_EQ(first.attributes.row(by_x(first, 5.1)), records.row(2));

    // RANDOM keeps one of the inputs, again record-for-record.
    const AttributedPointCloud random =
        voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::RANDOM, layout, records);
    ASSERT_EQ(random.xyz.rows(), 2);
    const Eigen::Index near_random = by_x(random, 0.15);
    ASSERT_GE(near_random, 0);
    EXPECT_TRUE(random.attributes.row(near_random) == records.row(0) ||
                random.attributes.row(near_random) == records.row(1));

    // AVERAGE_POINT: integer means are rounded, floats keep the exact mean.
    const AttributedPointCloud average = voxel_downsample_xd(
        xyz, 1.0, 1, 1, VoxelDownsampleStrategy::AVERAGE_POINT, layout, records);
    ASSERT_EQ(average.xyz.rows(), 2);
    const TypedAttribs mean = decode_typed(average)[static_cast<std::size_t>(by_x(average, 0.15))];
    EXPECT_EQ(mean.range, 4294967294u);
    EXPECT_EQ(mean.reflectivity, 127u);  // 127.0 exactly
    EXPECT_FLOAT_EQ(mean.signal, 0.5f);

    // Without attributes it downsamples positions only, keeping the same
    // points as voxel_downsample_3d. Explicitly empty 0-wide records work too.
    const AttributedPointCloud positions_only =
        voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::FIRST_N_POINT);
    const AttributedPointCloud positions_only_explicit =
        voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::FIRST_N_POINT,
                            AttributeLayout(), AttributeRecords(3, 0));
    EXPECT_TRUE((positions_only_explicit.xyz == positions_only.xyz).all());
    EXPECT_EQ(positions_only.attributes.rows(), positions_only.xyz.rows());
    EXPECT_TRUE(positions_only.layout.empty());
    EXPECT_EQ(positions_only.attributes.cols(), 0);
    const ArrayX3dR plain =
        voxel_downsample_3d(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::FIRST_N_POINT);
    ASSERT_EQ(plain.rows(), positions_only.xyz.rows());
    ASSERT_EQ(plain.rows(), first.xyz.rows());
    for (Eigen::Index row = 0; row < plain.rows(); ++row) {
        EXPECT_GE(by_x(positions_only, plain(row, 0)), 0);
    }

    // Empty input yields an empty cloud that still carries the layout.
    const AttributedPointCloud empty =
        voxel_downsample_xd(ArrayX3dR(0, 3), 1.0, 1, 1, VoxelDownsampleStrategy::RANDOM, layout,
                            AttributeRecords(0, static_cast<Eigen::Index>(layout.stride())));
    EXPECT_EQ(empty.xyz.rows(), 0);
    EXPECT_EQ(empty.attributes.rows(), 0);
    EXPECT_EQ(empty.layout, layout);

    // Mis-sized records are rejected.
    EXPECT_THROW(voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::RANDOM, layout,
                                     AttributeRecords::Zero(2, 9)),
                 std::invalid_argument);
    // Records without a layout to describe them are rejected, not dropped.
    EXPECT_THROW(voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::RANDOM,
                                     AttributeLayout(), records),
                 std::invalid_argument);
}

TEST(VoxelDownsampleTest, VoxelDownsampleRoutesOnTheAttributeLayout) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout({{AttributeType::U32, 1}, {AttributeType::U8, 1}});
    ArrayX3dR xyz(5, 3);
    xyz << 0.1, 0.1, 0.1, 0.2, 0.3, 0.1, 5.1, 5.1, 5.1, 5.4, 5.2, 5.3, -3.5, 2.5, 0.5;
    AttributeRecords records(5, static_cast<Eigen::Index>(layout.stride()));
    for (Eigen::Index row = 0; row < records.rows(); ++row) {
        const double values[2] = {static_cast<double>(1000 + row), static_cast<double>(row)};
        layout.encode(values, records.row(row).data());
    }

    for (const VoxelDownsampleStrategy strategy :
         {VoxelDownsampleStrategy::FIRST_N_POINT, VoxelDownsampleStrategy::AVERAGE_POINT,
          VoxelDownsampleStrategy::RANDOM}) {
        // With attributes it is voxel_downsample_xd().
        const AttributedPointCloud attributed =
            voxel_downsample(xyz, 1.0, 2, 1, strategy, layout, records);
        const AttributedPointCloud expected_xd =
            voxel_downsample_xd(xyz, 1.0, 2, 1, strategy, layout, records);
        EXPECT_EQ(attributed.layout, layout);
        EXPECT_TRUE((attributed.xyz == expected_xd.xyz).all());
        EXPECT_TRUE(attributed.attributes == expected_xd.attributes);

        // Positions only, it is voxel_downsample_3d(), with zero-width records.
        const AttributedPointCloud positions_only = voxel_downsample(xyz, 1.0, 2, 1, strategy);
        const ArrayX3dR expected_3d = voxel_downsample_3d(xyz, 1.0, 2, 1, strategy);
        EXPECT_TRUE((positions_only.xyz == expected_3d).all());
        EXPECT_TRUE(positions_only.layout.empty());
        EXPECT_EQ(positions_only.attributes.rows(), expected_3d.rows());
        EXPECT_EQ(positions_only.attributes.cols(), 0);
    }

    // Explicitly empty 0-wide records are accepted; records without a layout are not.
    EXPECT_EQ(voxel_downsample(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::FIRST_N_POINT,
                               AttributeLayout(), AttributeRecords(5, 0))
                  .xyz.rows(),
              3);
    EXPECT_THROW(voxel_downsample(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::RANDOM,
                                  AttributeLayout(), records),
                 std::invalid_argument);
    EXPECT_THROW(voxel_downsample(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::RANDOM, layout,
                                  AttributeRecords::Zero(2, 9)),
                 std::invalid_argument);
}

TEST(VoxelDownsampleXdTest, LastNPointKeepsTheLastAddedPointPerVoxel) {
    using namespace ouster::sdk::core;
    // Each point carries its own insertion-order sequence number (0, 1, 2,
    // ...) as a U32 attribute, so "which point survived" can be read directly
    // off the output rather than inferred from position. Two independent
    // voxels (around the origin and far away) each get their own 0..4
    // sequence, so a pass isn't just a fluke of one voxel.
    //
    // voxel_size=10 with max_points_per_voxel=1 sets the near-duplicate
    // rejection radius to voxel_size / sqrt(1) = voxel_size (the full voxel
    // width), so every point here sits near a different corner of its voxel:
    // consecutive insertions are always a face or space diagonal apart
    // (12.7 or 15.6) and never a plain edge (9), which would fall under the
    // radius and be rejected as a near-duplicate instead of evicting anything.
    ArrayX3dR xyz(10, 3);
    xyz << 0.5, 0.5, 0.5,     // voxel A, seq 0
        9.5, 9.5, 9.5,        // voxel A, seq 1
        0.5, 9.5, 0.5,        // voxel A, seq 2
        9.5, 0.5, 9.5,        // voxel A, seq 3
        9.5, 9.5, 0.5,        // voxel A, seq 4 (expected survivor)
        100.5, 100.5, 100.5,  // voxel B, seq 0
        109.5, 109.5, 109.5,  // voxel B, seq 1
        100.5, 109.5, 100.5,  // voxel B, seq 2
        109.5, 100.5, 109.5,  // voxel B, seq 3
        109.5, 109.5, 100.5;  // voxel B, seq 4 (expected survivor)
    const AttributeLayout layout({{AttributeType::U32, 1}});
    const std::vector<std::uint32_t> sequence = {0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
    AttributeRecords records(10, static_cast<Eigen::Index>(layout.stride()));
    layout.pack(0, sequence, records);

    // Sanity check: FIRST_N_POINT (the strategy last_n_point was refactored
    // out of) keeps the opposite end -- the first point added to each voxel.
    const AttributedPointCloud first = voxel_downsample(
        xyz, /*voxel_size=*/10.0, /*max_points_per_voxel=*/1, /*min_points_per_voxel=*/1,
        VoxelDownsampleStrategy::FIRST_N_POINT, layout, records, VHMInternalStorageMode::ABS);
    ASSERT_EQ(first.size(), 2u);
    std::vector<std::uint32_t> first_seqs = first.attribute<std::uint32_t>(0);
    std::sort(first_seqs.begin(), first_seqs.end());
    EXPECT_EQ(first_seqs, (std::vector<std::uint32_t>{0u, 0u}));

    // LAST_N_POINT: exactly one survivor per voxel (max_points_per_voxel=1),
    // and it must be the last point added to that voxel -- sequence 4, for
    // both voxels.
    const AttributedPointCloud last = voxel_downsample(
        xyz, /*voxel_size=*/10.0, /*max_points_per_voxel=*/1, /*min_points_per_voxel=*/1,
        VoxelDownsampleStrategy::LAST_N_POINT, layout, records, VHMInternalStorageMode::ABS);
    ASSERT_EQ(last.size(), 2u);
    std::vector<std::uint32_t> last_seqs = last.attribute<std::uint32_t>(0);
    std::sort(last_seqs.begin(), last_seqs.end());
    EXPECT_EQ(last_seqs, (std::vector<std::uint32_t>{4u, 4u}));
}

TEST(VoxelDownsampleXdTest, SmallestKeepsSmallestAttributeValue) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    // Four points share one voxel with RANGE 100, 50, 200, 10 (insertion
    // order); a fifth sits alone in a second voxel with RANGE 999.
    const std::vector<TypedAttribs> attribs = {
        {100u, 1u, 0.0f}, {50u, 2u, 0.0f}, {200u, 3u, 0.0f}, {10u, 4u, 0.0f}, {999u, 5u, 0.0f}};
    ArrayX3dR xyz(5, 3);
    xyz << 0.1, 0.1, 0.1, 0.15, 0.15, 0.15, 0.2, 0.2, 0.2, 0.25, 0.25, 0.25, 5.1, 5.1, 5.1;
    const AttributeRecords records = encode_typed(layout, attribs);

    // RANGE is spec index 0 in typed_layout(). With max_points_per_voxel == 2,
    // the crowded voxel should keep only its two smallest ranges (10, 50).
    const AttributedPointCloud out = voxel_downsample_xd(
        xyz, 1.0, /*max_points_per_voxel=*/2, /*min_points_per_voxel=*/1,
        VoxelDownsampleStrategy::SMALLEST, layout, records, VHMInternalStorageMode::AUTO,
        /*attribute_index=*/0);
    ASSERT_EQ(out.xyz.rows(), 3);

    std::vector<std::uint32_t> kept_ranges;
    for (const TypedAttribs& attribs_out : decode_typed(out)) {
        kept_ranges.push_back(attribs_out.range);
    }
    std::sort(kept_ranges.begin(), kept_ranges.end());
    EXPECT_EQ(kept_ranges, (std::vector<std::uint32_t>{10u, 50u, 999u}));
}

TEST(VoxelDownsampleXdTest, LargestKeepsLargestAttributeValue) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    // Same layout as SmallestKeepsSmallestAttributeValue, but ranking for the
    // largest values instead: the crowded voxel should keep 100 and 200.
    const std::vector<TypedAttribs> attribs = {
        {100u, 1u, 0.0f}, {50u, 2u, 0.0f}, {200u, 3u, 0.0f}, {10u, 4u, 0.0f}, {999u, 5u, 0.0f}};
    ArrayX3dR xyz(5, 3);
    xyz << 0.1, 0.1, 0.1, 0.15, 0.15, 0.15, 0.2, 0.2, 0.2, 0.25, 0.25, 0.25, 5.1, 5.1, 5.1;
    const AttributeRecords records = encode_typed(layout, attribs);

    const AttributedPointCloud out = voxel_downsample_xd(
        xyz, 1.0, /*max_points_per_voxel=*/2, /*min_points_per_voxel=*/1,
        VoxelDownsampleStrategy::LARGEST, layout, records, VHMInternalStorageMode::AUTO,
        /*attribute_index=*/0);
    ASSERT_EQ(out.xyz.rows(), 3);

    std::vector<std::uint32_t> kept_ranges;
    for (const TypedAttribs& attribs_out : decode_typed(out)) {
        kept_ranges.push_back(attribs_out.range);
    }
    std::sort(kept_ranges.begin(), kept_ranges.end());
    EXPECT_EQ(kept_ranges, (std::vector<std::uint32_t>{100u, 200u, 999u}));
}

TEST(VoxelDownsampleXdTest, ExtremumStrategiesRankByAnyScalarAttributeType) {
    using namespace ouster::sdk::core;
    // typed_layout() is {U32 range, U8 reflectivity, F32 signal}; rank by the
    // float32 "signal" attribute (spec index 2) to confirm the strategy is
    // not tied to uint32/RANGE.
    const AttributeLayout layout = typed_layout();
    const std::vector<TypedAttribs> attribs = {{1u, 1u, 3.5f}, {2u, 2u, 1.5f}, {3u, 3u, 2.5f}};
    ArrayX3dR xyz(3, 3);
    xyz << 0.1, 0.1, 0.1, 0.15, 0.15, 0.15, 0.2, 0.2, 0.2;
    const AttributeRecords records = encode_typed(layout, attribs);

    const AttributedPointCloud out = voxel_downsample_xd(
        xyz, 1.0, /*max_points_per_voxel=*/1, 1, VoxelDownsampleStrategy::SMALLEST, layout, records,
        VHMInternalStorageMode::AUTO, /*attribute_index=*/2);
    ASSERT_EQ(out.xyz.rows(), 1);
    EXPECT_FLOAT_EQ(decode_typed(out)[0].signal, 1.5f);
}

TEST(VoxelDownsampleXdTest, ExtremumStrategiesDefaultToTheFirstAttribute) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    // Same crowded-voxel scenario as SmallestKeepsSmallestAttributeValue /
    // LargestKeepsLargestAttributeValue, but attribute_index is omitted; RANGE
    // is spec index 0, so the result should be identical either way.
    const std::vector<TypedAttribs> attribs = {
        {100u, 1u, 0.0f}, {50u, 2u, 0.0f}, {200u, 3u, 0.0f}, {10u, 4u, 0.0f}, {999u, 5u, 0.0f}};
    ArrayX3dR xyz(5, 3);
    xyz << 0.1, 0.1, 0.1, 0.15, 0.15, 0.15, 0.2, 0.2, 0.2, 0.25, 0.25, 0.25, 5.1, 5.1, 5.1;
    const AttributeRecords records = encode_typed(layout, attribs);

    auto kept_ranges_of = [](const AttributedPointCloud& out) {
        std::vector<std::uint32_t> ranges;
        for (const TypedAttribs& attribs_out : decode_typed(out)) {
            ranges.push_back(attribs_out.range);
        }
        std::sort(ranges.begin(), ranges.end());
        return ranges;
    };

    const AttributedPointCloud smallest =
        voxel_downsample_xd(xyz, 1.0, /*max_points_per_voxel=*/2, /*min_points_per_voxel=*/1,
                            VoxelDownsampleStrategy::SMALLEST, layout, records);
    EXPECT_EQ(kept_ranges_of(smallest), (std::vector<std::uint32_t>{10u, 50u, 999u}));

    const AttributedPointCloud largest =
        voxel_downsample_xd(xyz, 1.0, /*max_points_per_voxel=*/2, /*min_points_per_voxel=*/1,
                            VoxelDownsampleStrategy::LARGEST, layout, records);
    EXPECT_EQ(kept_ranges_of(largest), (std::vector<std::uint32_t>{100u, 200u, 999u}));
}

TEST(VoxelDownsampleXdTest, ExtremumStrategiesTreatNanAsTheWorstValue) {
    using namespace ouster::sdk::core;
    // Ranking by a float32 "signal" attribute (typed_layout()'s spec index 2),
    // one of whose values is NaN. Without special handling, a NaN that ends
    // up tracked as a voxel's "worst" value can never be displaced -- every
    // comparison against it is false -- freezing that slot regardless of what
    // arrives afterward.
    const AttributeLayout layout = typed_layout();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::vector<TypedAttribs> attribs = {
        {1u, 1u, nan}, {2u, 2u, 2.5f}, {3u, 3u, 1.5f}, {4u, 4u, 3.5f}};
    ArrayX3dR xyz(4, 3);
    xyz << 0.1, 0.1, 0.1, 0.15, 0.15, 0.15, 0.2, 0.2, 0.2, 0.25, 0.25, 0.25;
    const AttributeRecords records = encode_typed(layout, attribs);

    // SMALLEST, max_points_per_voxel=1: whichever order the NaN point arrives
    // in, it must never be the one kept over a real value.
    const AttributedPointCloud out = voxel_downsample_xd(
        xyz, 1.0, /*max_points_per_voxel=*/1, 1, VoxelDownsampleStrategy::SMALLEST, layout, records,
        VHMInternalStorageMode::AUTO, /*attribute_index=*/2);
    ASSERT_EQ(out.xyz.rows(), 1);
    EXPECT_FLOAT_EQ(decode_typed(out)[0].signal, 1.5f);

    // LARGEST too: the NaN point must still lose to every real value.
    const AttributedPointCloud out2 = voxel_downsample_xd(
        xyz, 1.0, /*max_points_per_voxel=*/1, 1, VoxelDownsampleStrategy::LARGEST, layout, records,
        VHMInternalStorageMode::AUTO, /*attribute_index=*/2);
    ASSERT_EQ(out2.xyz.rows(), 1);
    EXPECT_FLOAT_EQ(decode_typed(out2)[0].signal, 3.5f);
}

TEST(VoxelDownsampleXdTest, ExtremumStrategiesRequireAScalarAttribute) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    const std::vector<TypedAttribs> attribs = {{10u, 1u, 0.0f}, {20u, 2u, 0.0f}};
    ArrayX3dR xyz(2, 3);
    xyz << 0.1, 0.1, 0.1, 0.15, 0.15, 0.15;
    const AttributeRecords records = encode_typed(layout, attribs);

    // attribute_index explicitly set to the "unset" sentinel.
    EXPECT_THROW(voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::SMALLEST, layout,
                                     records, VHMInternalStorageMode::AUTO, NO_ATTRIBUTE_INDEX),
                 std::invalid_argument);
    EXPECT_THROW(voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::LARGEST, layout,
                                     records, VHMInternalStorageMode::AUTO, NO_ATTRIBUTE_INDEX),
                 std::invalid_argument);

    // A multi-element attribute (RGB, count 3) has no single smallest/largest value.
    // Also placed first (spec index 0) to confirm the default attribute_index
    // still gets validated rather than silently accepted.
    const AttributeLayout rgb_layout({{AttributeType::F32, 3}, {AttributeType::U32, 1}});
    const AttributeRecords rgb_records =
        AttributeRecords::Zero(2, static_cast<Eigen::Index>(rgb_layout.stride()));
    EXPECT_THROW(voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::SMALLEST, rgb_layout,
                                     rgb_records),
                 std::invalid_argument);

    // An out-of-range attribute_index must raise the same std::invalid_argument
    // as the other misuse cases above.
    EXPECT_THROW(voxel_downsample_xd(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::SMALLEST, layout,
                                     records, VHMInternalStorageMode::AUTO,
                                     /*attribute_index=*/5),
                 std::invalid_argument);
}

TEST(VoxelDownsampleXdTest, VoxelDownsample3dRejectsSmallestAndLargest) {
    using namespace ouster::sdk::core;
    ArrayX3dR xyz(2, 3);
    xyz << 0.1, 0.1, 0.1, 5.1, 5.1, 5.1;
    EXPECT_THROW(voxel_downsample_3d(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::SMALLEST),
                 std::invalid_argument);
    EXPECT_THROW(voxel_downsample_3d(xyz, 1.0, 1, 1, VoxelDownsampleStrategy::LARGEST),
                 std::invalid_argument);
}

TEST(VoxelHashMapTest, SinglePointPathRejectsPointsOfTheWrongWidth) {
    using namespace ouster::sdk::core;
    auto point_of = [](Eigen::Index size) {
        Eigen::VectorXd point = Eigen::VectorXd::Zero(size);
        point.head<3>() << 0.5, 0.5, 0.5;
        return point;
    };

    // Empty layout on an averaging map: a wider point used to reach the
    // bucket's `point += value` with mismatched sizes.
    AverageVoxelHashMapXd average(1.0, 100.0, 10, 1, AttributeLayout());
    EXPECT_THROW(average.add_point(point_of(5)), std::invalid_argument);
    EXPECT_TRUE(average.empty());
    EXPECT_NO_THROW(average.add_point(point_of(3)));

    // Typed layout (two attributes, point_cols() == 5): both directions fail.
    const AttributeLayout layout({{AttributeType::U32, 1}, {AttributeType::U8, 1}});
    for (const auto mode : {VHMInternalStorageMode::ABS, VHMInternalStorageMode::FX16}) {
        VoxelHashMapXd map(1.0, 100.0, 10, 1, layout, mode);
        ASSERT_EQ(map.point_cols(), 5);
        EXPECT_THROW(map.add_point(point_of(4)), std::invalid_argument);
        EXPECT_THROW(map.add_point(point_of(6)), std::invalid_argument);
        EXPECT_TRUE(map.empty());
        EXPECT_NO_THROW(map.add_point(point_of(5)));
        EXPECT_EQ(map.pointcloud_double().rows(), 1);
    }

    // add_points(std::vector) and update() check every point before inserting
    // any, so one bad point leaves the map untouched.
    RandomVoxelHashMapXd random(1.0, 100.0, 10, 1, AttributeLayout::all_double(1));
    Eigen::VectorXd far = point_of(4);
    far[0] = 50.5;
    const std::vector<Eigen::VectorXd> mixed = {point_of(4), far, point_of(3)};
    EXPECT_THROW(random.add_points(mixed), std::invalid_argument);
    EXPECT_TRUE(random.empty());
    EXPECT_THROW(random.update(mixed, point_of(4)), std::invalid_argument);
    EXPECT_TRUE(random.empty());
    random.update({point_of(4), far}, point_of(4));
    EXPECT_EQ(random.pointcloud_double().rows(), 2);
}

// The migration recipe documented in docs/migration/migration-1.0.0-1.1.0.rst for
// callers of the removed `ArrayXXdR voxel_downsample_xd(frame, ...)`: view an
// Nx(3+K) double frame as positions plus all-double records without copying,
// and turn the result back into an Nx(3+K) double array.
TEST(VoxelDownsampleXdTest, MigrationFromTheNxDDoubleForm) {
    using namespace ouster::sdk::core;
    ArrayXXdR frame(4, 5);
    frame << 0.1, 0.1, 0.1, 10.0, 100.0,  //
        0.2, 0.2, 0.2, 12.0, 102.0,       //
        5.1, 5.1, 5.1, 20.0, 200.0,       //
        5.2, 5.2, 5.2, 22.0, 202.0;

    // [doc-stag-voxel-downsample-xd-migration]
    const Eigen::Index k = frame.cols() - 3;  // attribute columns after x, y, z
    const AttributeLayout layout = AttributeLayout::all_double(static_cast<std::size_t>(k));
    // Each row's attribute tail, viewed as layout.stride() bytes -- no copy.
    const Eigen::Map<const AttributeRecords, 0, Eigen::OuterStride<>> records(
        reinterpret_cast<const std::uint8_t*>(frame.data() + 3), frame.rows(),
        static_cast<Eigen::Index>(layout.stride()),
        Eigen::OuterStride<>(frame.cols() * static_cast<Eigen::Index>(sizeof(double))));

    const AttributedPointCloud out = voxel_downsample_xd(
        frame.leftCols<3>(), 1.0, 1, 1, VoxelDownsampleStrategy::AVERAGE_POINT, layout, records);

    // Back to the old Nx(3+K) double array, if the caller still needs it.
    ArrayXXdR result(out.xyz.rows(), 3 + k);
    for (Eigen::Index row = 0; row < result.rows(); ++row) {
        result.row(row).head<3>() = out.xyz.row(row);
        out.layout.decode(out.attributes.row(row).data(), result.row(row).data() + 3);
    }
    // [doc-etag-voxel-downsample-xd-migration]

    ASSERT_EQ(result.rows(), 2);
    ASSERT_EQ(result.cols(), 5);
    for (Eigen::Index row = 0; row < result.rows(); ++row) {
        const bool near = result(row, 0) < 1.0;
        EXPECT_NEAR(result(row, 0), near ? 0.15 : 5.15, 1e-4);
        EXPECT_DOUBLE_EQ(result(row, 3), near ? 11.0 : 21.0);
        EXPECT_DOUBLE_EQ(result(row, 4), near ? 101.0 : 201.0);
    }
}

// ---------------------------------------------------------------------------
// VoxelHashMap
// ---------------------------------------------------------------------------
namespace {

const ouster::sdk::core::VoxelDownsampleStrategy g_all_strategies[] = {
    ouster::sdk::core::VoxelDownsampleStrategy::FIRST_N_POINT,
    ouster::sdk::core::VoxelDownsampleStrategy::LAST_N_POINT,
    ouster::sdk::core::VoxelDownsampleStrategy::AVERAGE_POINT,
    ouster::sdk::core::VoxelDownsampleStrategy::RANDOM,
};

// Fills a proxy and the concrete map it should have picked with the same points
// and checks they produce the same pointcloud, which only holds when the proxy
// really forwards to that specialization (the strategies disagree on which
// point, or what average, represents a voxel).
template <typename MapT>
void expect_proxy_matches(ouster::sdk::core::VoxelDownsampleStrategy strategy,
                          const ouster::sdk::core::AttributeLayout& layout, const ArrayX3dR& xyz,
                          const ouster::sdk::core::AttributeRecords& records) {
    using namespace ouster::sdk::core;
    const double voxel_size = 1.0;
    const std::size_t max_points_per_voxel = 2;
    const std::size_t min_points_per_voxel = 1;

    auto proxy = VoxelHashMap::create(voxel_size, max_points_per_voxel, min_points_per_voxel,
                                      strategy, layout);
    MapT concrete(voxel_size, VoxelHashMap::DEFAULT_MAX_DISTANCE, max_points_per_voxel,
                  min_points_per_voxel, layout);
    EXPECT_EQ(proxy->strategy(), strategy);
    EXPECT_EQ(proxy->point_cols(), concrete.point_cols());
    EXPECT_EQ(proxy->attribute_layout(), layout);
    EXPECT_EQ(proxy->storage_mode(), concrete.storage_mode());
    EXPECT_EQ(proxy->voxel_size(), voxel_size);
    EXPECT_EQ(proxy->max_points_per_voxel(), max_points_per_voxel);
    EXPECT_EQ(proxy->min_points_per_voxel(), min_points_per_voxel);
    EXPECT_EQ(proxy->max_distance(), VoxelHashMap::DEFAULT_MAX_DISTANCE);
    EXPECT_TRUE(proxy->empty());

    proxy->add_points(xyz, records);
    concrete.add_points(xyz, records);
    EXPECT_FALSE(proxy->empty());

    const AttributedPointCloud got = proxy->pointcloud();
    const AttributedPointCloud want = concrete.pointcloud_attributed();
    EXPECT_EQ(got.layout, want.layout);
    EXPECT_TRUE(got.xyz.isApprox(want.xyz));
    EXPECT_TRUE((got.attributes.array() == want.attributes.array()).all());

    proxy->clear();
    EXPECT_TRUE(proxy->empty());
}

}  // namespace

TEST(VoxelHashMapProxyTest, CreatePicksTheSpecializationForEachStrategy) {
    using namespace ouster::sdk::core;
    std::mt19937 gen(7);
    std::uniform_real_distribution<double> coord(-4.0, 4.0);
    std::uniform_int_distribution<std::uint32_t> value(0, 1000);
    const Eigen::Index n = 500;
    ArrayX3dR xyz(n, 3);
    std::vector<TypedAttribs> attribs(static_cast<std::size_t>(n));
    for (Eigen::Index i = 0; i < n; ++i) {
        xyz.row(i) << coord(gen), coord(gen), coord(gen);
        attribs[static_cast<std::size_t>(i)] = {value(gen),
                                                static_cast<std::uint8_t>(value(gen) % 256),
                                                static_cast<float>(value(gen)) / 8.0f};
    }
    const AttributeLayout layout = typed_layout();
    const AttributeRecords records = encode_typed(layout, attribs);
    const AttributeRecords no_records(n, 0);

    // No attributes: the positions-only (3d) maps.
    expect_proxy_matches<VoxelHashMap3d>(VoxelDownsampleStrategy::FIRST_N_POINT, AttributeLayout(),
                                         xyz, no_records);
    expect_proxy_matches<LastNPointVoxelHashMap3d>(VoxelDownsampleStrategy::LAST_N_POINT,
                                                   AttributeLayout(), xyz, no_records);
    expect_proxy_matches<AverageVoxelHashMap3d>(VoxelDownsampleStrategy::AVERAGE_POINT,
                                                AttributeLayout(), xyz, no_records);
    expect_proxy_matches<RandomVoxelHashMap3d>(VoxelDownsampleStrategy::RANDOM, AttributeLayout(),
                                               xyz, no_records);
    // With attributes: the attributed (Xd) maps.
    expect_proxy_matches<VoxelHashMapXd>(VoxelDownsampleStrategy::FIRST_N_POINT, layout, xyz,
                                         records);
    expect_proxy_matches<LastNPointVoxelHashMapXd>(VoxelDownsampleStrategy::LAST_N_POINT, layout,
                                                   xyz, records);
    expect_proxy_matches<AverageVoxelHashMapXd>(VoxelDownsampleStrategy::AVERAGE_POINT, layout, xyz,
                                                records);
    expect_proxy_matches<RandomVoxelHashMapXd>(VoxelDownsampleStrategy::RANDOM, layout, xyz,
                                               records);
}

TEST(VoxelHashMapProxyTest, CreateRejectsBadArguments) {
    using namespace ouster::sdk::core;
    EXPECT_THROW(VoxelHashMap::create(1.0, 1, 1, static_cast<VoxelDownsampleStrategy>(99)),
                 std::invalid_argument);
    // The underlying map's own validation still applies.
    for (const VoxelDownsampleStrategy strategy : g_all_strategies) {
        EXPECT_THROW(VoxelHashMap::create(0.0, 1, 1, strategy), std::invalid_argument);
        EXPECT_THROW(VoxelHashMap::create(1.0, 0, 1, strategy), std::invalid_argument);
        EXPECT_THROW(VoxelHashMap::create(1.0, 1, 1, strategy, typed_layout(), 0.0),
                     std::invalid_argument);
    }
}

TEST(VoxelHashMapProxyTest, PositionsOnlyMapSpeaksVectorXd) {
    using namespace ouster::sdk::core;
    for (const VoxelDownsampleStrategy strategy : g_all_strategies) {
        auto map = VoxelHashMap::create(1.0, 1, 1, strategy);
        ASSERT_EQ(map->point_cols(), 3);
        EXPECT_TRUE(map->attribute_layout().empty());

        // Points must be exactly xyz; a bad one in a batch inserts nothing.
        EXPECT_THROW(map->add_point(Eigen::VectorXd::Zero(4)), std::invalid_argument);
        const std::vector<Eigen::VectorXd> mixed = {Eigen::Vector3d(0.5, 0.5, 0.5),
                                                    Eigen::VectorXd::Zero(2)};
        EXPECT_THROW(map->add_points(mixed), std::invalid_argument);
        EXPECT_TRUE(map->empty());
        EXPECT_THROW(map->update(mixed, Eigen::Vector3d::Zero()), std::invalid_argument);
        EXPECT_TRUE(map->empty());
        // A bad trim position is rejected before any point goes in.
        EXPECT_THROW(map->update(std::vector<Eigen::VectorXd>{Eigen::Vector3d(0.5, 0.5, 0.5)},
                                 Eigen::VectorXd::Zero(2)),
                     std::invalid_argument);
        EXPECT_TRUE(map->empty());
        // Positions need only their leading xyz.
        EXPECT_THROW(map->remove_voxels_far_from_location(Eigen::VectorXd::Zero(2)),
                     std::invalid_argument);

        map->update(std::vector<Eigen::VectorXd>{Eigen::Vector3d(0.5, 0.5, 0.5),
                                                 Eigen::Vector3d(250.5, 0.5, 0.5)},
                    Eigen::Vector3d::Zero());
        // The far point was trimmed against the default max_distance.
        const std::vector<Eigen::VectorXd> points = map->pointcloud_vector();
        ASSERT_EQ(points.size(), 1u);
        EXPECT_EQ(points[0].size(), 3);

        Eigen::VectorXd query(4);
        query << 0.5, 0.5, 0.5, 123.0;  // trailing values are ignored
        const auto neighbor = map->get_closest_neighbor(query);
        EXPECT_EQ(std::get<0>(neighbor).size(), 3);
        EXPECT_LT(std::get<1>(neighbor), 1e-3);

        map->add_point(Eigen::Vector3d(200.5, 0.5, 0.5));
        const AttributedPointCloud far =
            map->extract_voxels_far_from_location(Eigen::Vector3d::Zero());
        ASSERT_EQ(far.size(), 1u);
        EXPECT_NEAR(far.xyz(0, 0), 200.5, 0.5);
        EXPECT_EQ(map->pointcloud().size(), 1u);
    }
}

TEST(VoxelHashMapProxyTest, AddPointsVector3dConvenienceOverload) {
    using namespace ouster::sdk::core;

    // Empty input: a no-op, not a crash on points.front().
    {
        auto map = VoxelHashMap::create(1.0);
        map->add_points(std::vector<Eigen::Vector3d>{});
        EXPECT_TRUE(map->empty());
    }

    // Zero-copy insert into a positions-only map, for every strategy.
    for (const VoxelDownsampleStrategy strategy : g_all_strategies) {
        auto map = VoxelHashMap::create(1.0, /*max_points_per_voxel=*/4, /*min_points_per_voxel=*/1,
                                        strategy);
        // Each point lands in its own voxel, well apart, so the result is the
        // same for every strategy regardless of per-voxel reduction.
        const std::vector<Eigen::Vector3d> points = {Eigen::Vector3d(0.5, 0.5, 0.5),
                                                     Eigen::Vector3d(5.5, 5.5, 5.5),
                                                     Eigen::Vector3d(10.5, 10.5, 10.5)};
        map->add_points(points);
        EXPECT_EQ(map->pointcloud().size(), 3u);
    }

    // An attributed map expects attribute_layout().element_count() extra
    // columns per point; this convenience overload supplies only xyz, so it
    // throws exactly as add_points(ArrayXXdR) does for a column mismatch.
    {
        const AttributeLayout layout = AttributeLayout::all_double(2);
        auto map = VoxelHashMap::create(1.0, 1, 1, VoxelDownsampleStrategy::FIRST_N_POINT, layout);
        EXPECT_THROW(map->add_points(std::vector<Eigen::Vector3d>{Eigen::Vector3d(0.5, 0.5, 0.5)}),
                     std::invalid_argument);
    }
}

TEST(VoxelHashMapProxyTest, AttributedMapKeepsAttributeTypes) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    const std::vector<TypedAttribs> attribs = {{4294967295u, 255u, 1.5f}, {16777217u, 1u, -2.25f}};
    ArrayX3dR xyz(2, 3);
    xyz << 0.5, 0.5, 0.5,  //
        200.5, 0.5, 0.5;

    for (const VoxelDownsampleStrategy strategy :
         {VoxelDownsampleStrategy::FIRST_N_POINT, VoxelDownsampleStrategy::RANDOM}) {
        auto map = VoxelHashMap::create(1.0, 1, 1, strategy, layout);
        ASSERT_EQ(map->point_cols(), 6);
        map->add_points(xyz, encode_typed(layout, attribs));

        const auto neighbor = map->get_closest_neighbor(Eigen::Vector3d(0.5, 0.5, 0.5));
        ASSERT_EQ(std::get<0>(neighbor).size(), 6);
        EXPECT_EQ(std::get<0>(neighbor)[3], 4294967295.0);

        const AttributedPointCloud far =
            map->extract_voxels_far_from_location(Eigen::Vector3d::Zero());
        ASSERT_EQ(far.xyz.rows(), 1);
        const TypedAttribs got = decode_typed(far)[0];
        EXPECT_EQ(got.range, 16777217u);
        EXPECT_EQ(got.reflectivity, 1u);
        EXPECT_EQ(got.signal, -2.25f);

        // The double-valued API sees the attributes too.
        Eigen::VectorXd point(6);
        point << 1.5, 0.5, 0.5, 7.0, 8.0, 9.5;
        map->add_point(point);
        EXPECT_THROW(map->add_point(Eigen::VectorXd::Zero(3)), std::invalid_argument);
        EXPECT_EQ(map->pointcloud().size(), 2u);
    }
}

TEST(VoxelHashMapProxyTest, SmallestAndLargestStrategiesRankByAttributeIndex) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout = typed_layout();
    // Two points share one voxel with RANGE 100, 20; a third sits alone in a
    // second voxel with RANGE 999.
    const std::vector<TypedAttribs> attribs = {{100u, 1u, 0.0f}, {20u, 2u, 0.0f}, {999u, 3u, 0.0f}};
    ArrayX3dR xyz(3, 3);
    xyz << 0.1, 0.1, 0.1, 0.15, 0.15, 0.15, 10.5, 0.5, 0.5;
    const AttributeRecords records = encode_typed(layout, attribs);

    auto smallest = VoxelHashMap::create(1.0, 1, 1, VoxelDownsampleStrategy::SMALLEST, layout);
    EXPECT_EQ(smallest->strategy(), VoxelDownsampleStrategy::SMALLEST);
    smallest->add_points(xyz, records);
    std::vector<std::uint32_t> smallest_ranges;
    for (const TypedAttribs& a : decode_typed(smallest->pointcloud())) {
        smallest_ranges.push_back(a.range);
    }
    std::sort(smallest_ranges.begin(), smallest_ranges.end());
    EXPECT_EQ(smallest_ranges, (std::vector<std::uint32_t>{20u, 999u}));

    auto largest = VoxelHashMap::create(1.0, 1, 1, VoxelDownsampleStrategy::LARGEST, layout);
    EXPECT_EQ(largest->strategy(), VoxelDownsampleStrategy::LARGEST);
    largest->add_points(xyz, records);
    std::vector<std::uint32_t> largest_ranges;
    for (const TypedAttribs& a : decode_typed(largest->pointcloud())) {
        largest_ranges.push_back(a.range);
    }
    std::sort(largest_ranges.begin(), largest_ranges.end());
    EXPECT_EQ(largest_ranges, (std::vector<std::uint32_t>{100u, 999u}));

    // Needs a non-empty attribute_layout -- there is nothing to rank
    // positions-only points by.
    EXPECT_THROW(VoxelHashMap::create(1.0, 1, 1, VoxelDownsampleStrategy::SMALLEST),
                 std::invalid_argument);
    EXPECT_THROW(VoxelHashMap::create(1.0, 1, 1, VoxelDownsampleStrategy::LARGEST),
                 std::invalid_argument);

    // attribute_index must name a scalar attribute of attribute_layout:
    // REFLECTIVITY (spec index 1) is scalar, so this one succeeds...
    EXPECT_NO_THROW(VoxelHashMap::create(1.0, 1, 1, VoxelDownsampleStrategy::SMALLEST, layout,
                                         100.0, VHMInternalStorageMode::AUTO,
                                         /*attribute_index=*/1));
    // ... but an out-of-range index doesn't.
    EXPECT_THROW(VoxelHashMap::create(1.0, 1, 1, VoxelDownsampleStrategy::SMALLEST, layout, 100.0,
                                      VHMInternalStorageMode::AUTO, /*attribute_index=*/5),
                 std::invalid_argument);
}

TEST(VoxelHashMapProxyTest, UpdateFromMatrixMatchesTheVectorOverload) {
    using namespace ouster::sdk::core;

    // Positions-only: an Nx3 matrix.
    {
        auto map = VoxelHashMap::create(1.0, 10, 1, VoxelDownsampleStrategy::FIRST_N_POINT);
        ArrayXXdR points(2, 3);
        points << 0.5, 0.5, 0.5,  //
            250.5, 0.5, 0.5;

        // Wrong column count: rejected before any point is inserted.
        EXPECT_THROW(map->update(ArrayXXdR::Zero(2, 4), Eigen::Vector3d::Zero()),
                     std::invalid_argument);
        EXPECT_TRUE(map->empty());
        // A bad trim position is rejected the same way as for the std::vector overload.
        EXPECT_THROW(map->update(points, Eigen::VectorXd::Zero(2)), std::invalid_argument);
        EXPECT_TRUE(map->empty());

        map->update(points, Eigen::Vector3d::Zero());
        // The far point was trimmed against the default max_distance, same as
        // the std::vector overload's test above.
        const std::vector<Eigen::VectorXd> kept = map->pointcloud_vector();
        ASSERT_EQ(kept.size(), 1u);
        EXPECT_TRUE(kept[0].isApprox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    }

    // Attributed: an Nx6 matrix (xyz + 3 double attributes), compared against
    // the equivalent std::vector<Eigen::VectorXd> call.
    {
        const AttributeLayout layout = AttributeLayout::all_double(3);
        auto from_matrix =
            VoxelHashMap::create(1.0, 10, 1, VoxelDownsampleStrategy::FIRST_N_POINT, layout);
        auto from_vector =
            VoxelHashMap::create(1.0, 10, 1, VoxelDownsampleStrategy::FIRST_N_POINT, layout);
        ArrayXXdR points(2, 6);
        points << 0.5, 0.5, 0.5, 1.0, 2.0, 3.0,  //
            250.5, 0.5, 0.5, 4.0, 5.0, 6.0;
        const std::vector<Eigen::VectorXd> as_vector = {points.row(0), points.row(1)};

        from_matrix->update(points, Eigen::Vector3d::Zero());
        from_vector->update(as_vector, Eigen::Vector3d::Zero());

        EXPECT_TRUE(from_matrix->pointcloud().xyz.isApprox(from_vector->pointcloud().xyz));
    }
}

TEST(VoxelHashMapProxyTest, F16AttributeSurvivesTheMapAndAveraging) {
    using namespace ouster::sdk::core;
    const AttributeLayout layout({{AttributeType::F16, 1}});

    // FIRST_N_POINT / RANDOM: the exact half bits added come back untouched.
    for (const VoxelDownsampleStrategy strategy :
         {VoxelDownsampleStrategy::FIRST_N_POINT, VoxelDownsampleStrategy::RANDOM}) {
        auto map = VoxelHashMap::create(1.0, 1, 1, strategy, layout);
        ASSERT_EQ(map->point_cols(), 4);
        ArrayX3dR xyz(2, 3);
        xyz << 0.5, 0.5, 0.5,  //
            200.5, 0.5, 0.5;
        const Eigen::half signal[2] = {Eigen::half(1.5f), Eigen::half(-7.25f)};
        AttributeRecords records(2, static_cast<Eigen::Index>(layout.stride()));
        layout.pack(0, std::vector<Eigen::half>{signal[0], signal[1]}, records);
        map->add_points(xyz, records);

        const AttributedPointCloud far =
            map->extract_voxels_far_from_location(Eigen::Vector3d::Zero());
        ASSERT_EQ(far.size(), 1u);
        EXPECT_EQ(static_cast<float>(far.attribute<Eigen::half>(0)[0]), -7.25f);

        const AttributedPointCloud remaining = map->pointcloud();
        ASSERT_EQ(remaining.size(), 1u);
        EXPECT_EQ(static_cast<float>(remaining.attribute<Eigen::half>(0)[0]), 1.5f);

        // The double-valued API sees the same remaining point, widened to double.
        const std::vector<Eigen::VectorXd> doubles = map->pointcloud_vector();
        ASSERT_EQ(doubles.size(), 1u);
        EXPECT_NEAR(doubles[0][3], 1.5, 1e-9);
    }

    // AVERAGE_POINT: the map accumulates in double and re-encodes to half on
    // read, exercising encode()'s F16 path rather than pack()'s raw copy.
    {
        auto map = VoxelHashMap::create(1.0, 10, 1, VoxelDownsampleStrategy::AVERAGE_POINT, layout);
        ArrayX3dR xyz(2, 3);
        xyz << 0.5, 0.5, 0.5,  //
            0.6, 0.6, 0.6;
        AttributeRecords records(2, static_cast<Eigen::Index>(layout.stride()));
        layout.pack(0, std::vector<Eigen::half>{Eigen::half(1.0f), Eigen::half(2.0f)}, records);
        map->add_points(xyz, records);

        const AttributedPointCloud out = map->pointcloud();
        ASSERT_EQ(out.size(), 1u);
        EXPECT_EQ(static_cast<float>(out.attribute<Eigen::half>(0)[0]), 1.5f);  // exact in binary16
    }
}

TEST(VoxelDownsampleTest, StorageModeSelectsExactOrQuantizedPositions) {
    using namespace ouster::sdk::core;
    // Irregular coordinates, one point per voxel, so every point is kept and
    // any quantization of its position shows.
    ArrayX3dR xyz(3, 3);
    xyz << 0.01234567, 0.04567891, 0.07891234,  //
        0.51234567, 0.54567891, 0.57891234,     //
        1.01234567, 1.04567891, 1.07891234;
    const AttributeLayout layout({{AttributeType::U32, 1}});
    const AttributeRecords records = AttributeRecords::Zero(3, 4);
    const double voxel_size = 0.1;  // AUTO resolves to FX8 at this size

    auto sorted = [](ArrayX3dR points) {
        std::vector<Eigen::Index> order(static_cast<std::size_t>(points.rows()));
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                  [&](Eigen::Index a, Eigen::Index b) { return points(a, 0) < points(b, 0); });
        ArrayX3dR out(points.rows(), 3);
        for (std::size_t i = 0; i < order.size(); ++i) {
            out.row(static_cast<Eigen::Index>(i)) = points.row(order[i]);
        }
        return out;
    };

    for (const VoxelDownsampleStrategy strategy :
         {VoxelDownsampleStrategy::FIRST_N_POINT, VoxelDownsampleStrategy::RANDOM}) {
        // ABS keeps the input positions exactly.
        const ArrayX3dR exact_3d =
            voxel_downsample_3d(xyz, voxel_size, 1, 1, strategy, VHMInternalStorageMode::ABS);
        EXPECT_TRUE((sorted(exact_3d) == xyz).all());
        const AttributedPointCloud exact_xd = voxel_downsample_xd(
            xyz, voxel_size, 1, 1, strategy, layout, records, VHMInternalStorageMode::ABS);
        EXPECT_TRUE((sorted(exact_xd.xyz) == xyz).all());

        // The AUTO default quantizes to within a fraction of the voxel.
        const ArrayX3dR quantized = sorted(voxel_downsample_3d(xyz, voxel_size, 1, 1, strategy));
        ASSERT_EQ(quantized.rows(), 3);
        EXPECT_FALSE((quantized == xyz).all());
        EXPECT_LT((quantized - xyz).abs().maxCoeff(), voxel_size / 64.0);
    }
}

TEST(VoxelDownsampleTest, EmptyInputStillValidatesArguments) {
    using namespace ouster::sdk::core;
    const ArrayX3dR empty(0, 3);

    EXPECT_THROW(voxel_downsample_3d(empty, 0.0), std::invalid_argument);
    EXPECT_THROW(voxel_downsample_3d(empty, 1.0, 0), std::invalid_argument);
    EXPECT_THROW(voxel_downsample_xd(empty, 0.0), std::invalid_argument);
    EXPECT_THROW(voxel_downsample_xd(empty, 1.0, 0), std::invalid_argument);
    EXPECT_THROW(voxel_downsample(empty, 0.0), std::invalid_argument);
    EXPECT_THROW(voxel_downsample(empty, 1.0, 0), std::invalid_argument);

    // Valid arguments still give an empty 0x3 cloud.
    const ArrayX3dR out = voxel_downsample_3d(empty, 1.0);
    EXPECT_EQ(out.rows(), 0);
    EXPECT_EQ(out.cols(), 3);
    EXPECT_EQ(voxel_downsample_xd(empty, 1.0).xyz.rows(), 0);
    EXPECT_EQ(voxel_downsample(empty, 1.0).xyz.rows(), 0);
}

namespace {
// get_closest_neighbor_position() must find exactly what get_closest_neighbor()
// does -- same position, same squared distance, same misses -- just without
// building a full point. `cols` is the map's point_cols().
template <typename MapT>
void expect_position_search_matches_full_search(
    std::size_t num_attributes, std::initializer_list<VHMInternalStorageMode> modes) {
    using PointType = typename MapT::point_type;
    const Eigen::Index cols =
        3 + static_cast<Eigen::Index>(num_attributes) + (PointType::SizeAtCompileTime == 6 ? 3 : 0);
    std::mt19937 gen(11);
    for (const StorageConfig& config : g_storage_configs) {
        if (std::find(modes.begin(), modes.end(), config.mode) == modes.end()) continue;
        const double vs = config.def_voxel_size;
        std::uniform_real_distribution<double> coord(-2.0 * vs, 2.0 * vs);
        MapT map(vs, 100.0, /*max_points_per_voxel=*/4, /*min_points_per_voxel=*/1,
                 AttributeLayout::all_double(num_attributes), config.mode);
        ASSERT_EQ(map.point_cols(), cols);

        ouster::sdk::core::ArrayXXdR points(300, cols);
        for (Eigen::Index row = 0; row < points.rows(); ++row) {
            for (Eigen::Index col = 0; col < cols; ++col) points(row, col) = coord(gen);
        }
        map.add_points(points);

        for (const double max_distance_sq :
             {std::numeric_limits<double>::max(), 0.3 * vs * 0.3 * vs}) {
            for (int i = 0; i < 200; ++i) {
                const Eigen::Vector3d query(1.25 * coord(gen), 1.25 * coord(gen),
                                            1.25 * coord(gen));
                PointType full_query =
                    ouster::sdk::core::impl::PointDefaultValue<PointType>::make(cols);
                full_query.template head<3>() = query;

                const auto full = map.get_closest_neighbor(full_query, max_distance_sq);
                const auto position = map.get_closest_neighbor_position(query, max_distance_sq);
                EXPECT_EQ(position.second, std::get<1>(full));
                if (std::get<1>(full) < max_distance_sq) {
                    EXPECT_TRUE(
                        (position.first.array() == std::get<0>(full).template head<3>().array())
                            .all());
                } else {
                    EXPECT_TRUE(position.first.isZero());
                }
            }
        }
    }
}
}  // namespace

TEST(VoxelHashMapTest, PositionSearchMatchesFullSearchForEveryMapType) {
    using namespace ouster::sdk::core;
    const auto all = {VHMInternalStorageMode::ABS, VHMInternalStorageMode::FX16,
                      VHMInternalStorageMode::FX8};
    // The averaging and point-normal buckets always store ABS.
    const auto abs_only = {VHMInternalStorageMode::ABS};
    expect_position_search_matches_full_search<VoxelHashMap3d>(0, all);
    expect_position_search_matches_full_search<VoxelHashMapXd>(2, all);
    expect_position_search_matches_full_search<RandomVoxelHashMap3d>(0, all);
    expect_position_search_matches_full_search<RandomVoxelHashMapXd>(2, all);
    expect_position_search_matches_full_search<AverageVoxelHashMap3d>(0, abs_only);
    expect_position_search_matches_full_search<AverageVoxelHashMapXd>(2, abs_only);
    expect_position_search_matches_full_search<PointNormalVoxelHashMap3d>(0, abs_only);
}

TEST(VoxelHashMapProxyTest, PositionSearchOnAttributedMaps) {
    using namespace ouster::sdk::core;
    ArrayX3dR xyz(2, 3);
    xyz << 0.5, 0.5, 0.5,  //
        3.5, 0.5, 0.5;
    for (const VoxelDownsampleStrategy strategy :
         {VoxelDownsampleStrategy::FIRST_N_POINT, VoxelDownsampleStrategy::AVERAGE_POINT,
          VoxelDownsampleStrategy::RANDOM}) {
        auto map = VoxelHashMap::create(1.0, 1, 1, strategy, typed_layout(), 100.0,
                                        VHMInternalStorageMode::ABS);
        map->add_points(xyz, encode_typed(typed_layout(), {{1u, 2u, 3.0f}, {4u, 5u, 6.0f}}));
        const auto hit = map->get_closest_neighbor_position(Eigen::Vector3d(0.6, 0.5, 0.5), 1.0);
        EXPECT_TRUE(hit.first.isApprox(Eigen::Vector3d(0.5, 0.5, 0.5)));
        EXPECT_NEAR(hit.second, 0.01, 1e-12);
        // Nothing within range: squared distance >= the bound, position zero.
        const auto miss = map->get_closest_neighbor_position(Eigen::Vector3d(2.0, 0.5, 0.5), 0.25);
        EXPECT_GE(miss.second, 0.25);
        EXPECT_TRUE(miss.first.isZero());
    }
}

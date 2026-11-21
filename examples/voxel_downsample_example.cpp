/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * Minimal example of downsampling a pointcloud whose attributes keep their
 * own types.
 *
 * The C++ counterpart of python/src/ouster/sdk/examples/voxel_downsample.py.
 * voxel_downsample() takes and returns positions plus packed attribute
 * records -- a RANGE (uint32), a REFLECTIVITY (uint8), a NORMALS vector
 * (3 x float32) here -- each kept in its own type, rather than widening them
 * all into a single double array. Its parameters follow
 * VoxelHashMap::create(), with xyz first and the attribute records in place
 * of max_distance (downsampling never trims); see voxel_hash_map_example.cpp
 * for the map that create() builds.
 */

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include "ouster/core/attribute_layout.h"
#include "ouster/core/voxel_hash_map.h"

// NOLINTBEGIN(google-build-using-namespace)
using namespace ouster::sdk::core;
// NOLINTEND(google-build-using-namespace)

int main() {
    // [doc-stag-attribute-cpp-downsample-input]
    // A synthetic cloud: positions plus three attributes, each in the type a
    // sensor would produce it in. A scalar attribute is one value per point;
    // a multi-element one, NORMALS here, is count values per point.
    const AttributeLayout layout(
        {{AttributeType::U32, 1}, {AttributeType::U8, 1}, {AttributeType::F32, 3}});
    constexpr int n_points = 1000;
    // A fixed seed, not a random_device, so this example's output is
    // reproducible across runs; not a security-sensitive use of the RNG.
    std::mt19937 rng(0);  // NOLINT(cert-msc32-c,cert-msc51-cpp)
    std::uniform_real_distribution<double> coord(-5.0, 5.0);
    ArrayX3dR xyz(n_points, 3);
    std::vector<std::uint32_t> ranges(static_cast<std::size_t>(n_points));
    std::vector<std::uint8_t> reflectivity(static_cast<std::size_t>(n_points));
    std::vector<float> normals(static_cast<std::size_t>(n_points) * 3);
    for (int i = 0; i < n_points; ++i) {
        xyz.row(i) << coord(rng), coord(rng), coord(rng);
        ranges[static_cast<std::size_t>(i)] = static_cast<std::uint32_t>(rng());
        reflectivity[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(rng() & 0xFF);
        const Eigen::Vector3f normal =
            Eigen::Vector3d(coord(rng), coord(rng), coord(rng)).cast<float>().normalized();
        std::copy(normal.data(), normal.data() + 3, &normals[static_cast<std::size_t>(i) * 3]);
    }
    AttributeRecords records(n_points, static_cast<Eigen::Index>(layout.stride()));
    layout.pack(0, ranges, records);
    layout.pack(1, reflectivity, records);
    layout.pack(2, normals, records);
    // [doc-etag-attribute-cpp-downsample-input]

    // [doc-stag-attribute-cpp-downsample]
    // voxel_downsample()'s parameters follow VoxelHashMap::create(), in the
    // same order, with xyz first and the attribute layout plus its packed
    // records in place of max_distance (downsampling never trims).
    const AttributedPointCloud out = voxel_downsample(
        xyz, /*voxel_size=*/1.0, /*max_points_per_voxel=*/1,
        /*min_points_per_voxel=*/1, VoxelDownsampleStrategy::RANDOM, layout, records);
    std::cout << n_points << " points downsampled to " << out.size() << "\n";

    // RANDOM (like FIRST_N_POINT) keeps actual input points, so each kept
    // point's attributes are exactly those it was given -- no rounding
    // through a double. (Positions, by contrast, may come back within a tiny
    // fraction of the voxel size of the input: the default AUTO storage mode
    // keeps them in fixed point. Hence finding the source point by nearest
    // position rather than exact equality.)
    const std::vector<std::uint32_t> out_ranges = out.attribute<std::uint32_t>(0);
    const std::vector<std::uint8_t> out_reflectivity = out.attribute<std::uint8_t>(1);
    const std::vector<float> out_normals = out.attribute<float>(2);
    for (std::size_t row = 0; row < out.size(); ++row) {
        double best_dist_sq = std::numeric_limits<double>::max();
        std::size_t kept = 0;
        for (int i = 0; i < n_points; ++i) {
            const double dist_sq =
                (xyz.row(i) - out.xyz.row(static_cast<Eigen::Index>(row))).square().sum();
            if (dist_sq < best_dist_sq) {
                best_dist_sq = dist_sq;
                kept = static_cast<std::size_t>(i);
            }
        }
        if (out_ranges[row] != ranges[kept] || out_reflectivity[row] != reflectivity[kept] ||
            !std::equal(out_normals.begin() + static_cast<std::ptrdiff_t>(3 * row),
                        out_normals.begin() + static_cast<std::ptrdiff_t>(3 * row + 3),
                        normals.begin() + static_cast<std::ptrdiff_t>(3 * kept))) {
            std::cerr << "downsampled attributes disagree at row " << row << "\n";
            return 1;
        }
    }
    // [doc-etag-attribute-cpp-downsample]

    // [doc-stag-attribute-cpp-downsample-average]
    // AVERAGE_POINT computes each voxel's mean. Float attributes keep the
    // exact mean; integer attributes come back in their own type with the
    // mean rounded to the nearest representable value.
    const AttributedPointCloud avg = voxel_downsample(
        xyz, /*voxel_size=*/1.0, /*max_points_per_voxel=*/1, /*min_points_per_voxel=*/1,
        VoxelDownsampleStrategy::AVERAGE_POINT, layout, records);
    const std::vector<std::uint32_t> avg_ranges = avg.attribute<std::uint32_t>(0);
    const std::vector<std::uint8_t> avg_reflectivity = avg.attribute<std::uint8_t>(1);
    const std::vector<float> avg_normals = avg.attribute<float>(2);
    std::cout << "AVERAGE_POINT: " << avg.size() << " voxels\n";
    for (std::size_t row = 0; row < std::min<std::size_t>(3, avg.size()); ++row) {
        std::cout << "  xyz=(" << avg.xyz(static_cast<Eigen::Index>(row), 0) << ", "
                  << avg.xyz(static_cast<Eigen::Index>(row), 1) << ", "
                  << avg.xyz(static_cast<Eigen::Index>(row), 2) << ") range=" << avg_ranges[row]
                  << " reflectivity=" << static_cast<unsigned>(avg_reflectivity[row]) << " normal=("
                  << avg_normals[3 * row] << ", " << avg_normals[3 * row + 1] << ", "
                  << avg_normals[3 * row + 2] << ")\n";
    }
    // [doc-etag-attribute-cpp-downsample-average]
    return 0;
}

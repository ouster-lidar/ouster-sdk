/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * Minimal example of a VoxelHashMap with a typed attribute layout, and of
 * reading its attributes back through the AttributedPointCloud accessors.
 *
 * The C++ counterpart of python/src/ouster/sdk/examples/voxel_hash_map.py.
 * pointcloud() returns the attributes as packed byte records; the
 * accessors on AttributedPointCloud decode them into typed values, checking
 * the requested types against the cloud's layout at runtime.
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <random>
#include <vector>

#include "ouster/core/attribute_layout.h"
#include "ouster/core/voxel_hash_map.h"

// NOLINTBEGIN(google-build-using-namespace)
using namespace ouster::sdk::core;
// NOLINTEND(google-build-using-namespace)

int main() {
    // [doc-stag-attribute-cpp-construct]
    // A scalar RANGE (uint32), a scalar REFLECTIVITY (uint8) and a
    // three-element NORMALS (3 x float32), packed back-to-back:
    // 4 + 1 + 3 * 4 = 17 bytes per point.
    const AttributeLayout layout(
        {{AttributeType::U32, 1}, {AttributeType::U8, 1}, {AttributeType::F32, 3}});
    auto voxel_map = VoxelHashMap::create(
        /*voxel_size=*/0.5, /*max_points_per_voxel=*/10, /*min_points_per_voxel=*/1,
        VoxelDownsampleStrategy::FIRST_N_POINT, layout, /*max_distance=*/100.0);
    std::cout << "layout stride: " << voxel_map->attribute_layout().stride() << " bytes/point\n";
    // [doc-etag-attribute-cpp-construct]

    // [doc-stag-attribute-cpp-add-points]
    // Points go in as an Nx3 array of positions plus one packed record per
    // point.
    constexpr int n_points = 6;
    // A fixed seed, not a random_device, so this example's output is
    // reproducible across runs; not a security-sensitive use of the RNG.
    std::mt19937 rng(0);  // NOLINT(cert-msc32-c,cert-msc51-cpp)
    std::uniform_real_distribution<double> coord(-2.0, 2.0);
    ArrayX3dR xyz(n_points, 3);
    std::vector<std::uint32_t> ranges(n_points);
    std::vector<std::uint8_t> reflectivity(n_points);
    // A multi-element attribute is stored point by point: point i's three
    // components are normals[3 * i], normals[3 * i + 1] and normals[3 * i + 2].
    std::vector<float> normals(static_cast<std::size_t>(n_points) * 3);
    for (int i = 0; i < n_points; ++i) {
        xyz.row(i) << coord(rng), coord(rng), coord(rng);
        ranges[i] = static_cast<std::uint32_t>(rng());
        reflectivity[i] = static_cast<std::uint8_t>(rng() & 0xFF);
        const Eigen::Vector3f normal =
            Eigen::Vector3d(coord(rng), coord(rng), coord(rng)).cast<float>().normalized();
        std::copy(normal.data(), normal.data() + 3, &normals[static_cast<std::size_t>(i) * 3]);
    }

    // AttributeLayout::pack() writes one typed attribute into the records,
    // checking the value type and count against its AttributeSpec.
    AttributeRecords records(n_points, static_cast<Eigen::Index>(layout.stride()));
    layout.pack(0, ranges, records);
    layout.pack(1, reflectivity, records);
    layout.pack(2, normals, records);  // 3 floats per point
    voxel_map->add_points(xyz, records);
    // [doc-etag-attribute-cpp-add-points]

    const AttributedPointCloud cloud = voxel_map->pointcloud();
    std::cout << cloud.size() << " points retained after voxelization\n";

    // [doc-stag-attribute-cpp-read-columns]
    // One attribute at a time: the template argument is the type the
    // attribute is stored as. A mismatch -- cloud.attribute<float>(0), say,
    // even though float is also 4 bytes -- throws std::invalid_argument.
    const std::vector<std::uint32_t> out_ranges = cloud.attribute<std::uint32_t>(0);
    const std::vector<std::uint8_t> out_reflectivity = cloud.attribute<std::uint8_t>(1);
    // A three-element attribute is requested by its scalar type too, and comes
    // back as size() * 3 values, point by point. An Eigen::Map views them as
    // an Nx3 matrix, one normal per row, without copying.
    const std::vector<float> out_normals = cloud.attribute<float>(2);
    const Eigen::Map<const MatrixXXR<float>> normal_rows(
        out_normals.data(), static_cast<Eigen::Index>(cloud.size()), 3);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto row = static_cast<Eigen::Index>(i);
        std::cout << "  xyz=(" << cloud.xyz(row, 0) << ", " << cloud.xyz(row, 1) << ", "
                  << cloud.xyz(row, 2) << ") range=" << out_ranges[i]
                  << " reflectivity=" << static_cast<unsigned>(out_reflectivity[i]) << " normal=("
                  << normal_rows(row, 0) << ", " << normal_rows(row, 1) << ", "
                  << normal_rows(row, 2) << ")\n";
    }
    // [doc-etag-attribute-cpp-read-columns]

    // [doc-stag-attribute-cpp-read-struct]
    // All attributes at once, as a struct of your own. Listing the members in
    // layout order lets each one be checked against its AttributeSpec, so the
    // struct needs no packing and may carry padding or extra members. A
    // three-element attribute maps onto a std::array (or C array) of 3.
    struct PointAttributes {
        std::uint32_t range;
        std::uint8_t reflectivity;
        std::array<float, 3> normal;
    };
    const std::vector<PointAttributes> attrs = cloud.attributes_as<PointAttributes>(
        &PointAttributes::range, &PointAttributes::reflectivity, &PointAttributes::normal);
    // [doc-etag-attribute-cpp-read-struct]

    // [doc-stag-attribute-cpp-read-packed]
    // Or, without listing members, a packed struct that mirrors the record
    // byte for byte. Only its size can be checked, against layout.stride().
#pragma pack(push, 1)
    struct PackedPointAttributes {
        std::uint32_t range;
        std::uint8_t reflectivity;
        float normal[3];
    };
#pragma pack(pop)
    const std::vector<PackedPointAttributes> packed = cloud.attributes_as<PackedPointAttributes>();
    // [doc-etag-attribute-cpp-read-packed]

    // All three views decode the same values.
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        bool same =
            attrs[i].range == out_ranges[i] && attrs[i].reflectivity == out_reflectivity[i] &&
            packed[i].range == out_ranges[i] && packed[i].reflectivity == out_reflectivity[i];
        for (int channel = 0; channel < 3; ++channel) {
            same = same && attrs[i].normal[channel] == out_normals[3 * i + channel] &&
                   packed[i].normal[channel] == out_normals[3 * i + channel];
        }
        if (!same) {
            std::cerr << "decoded attributes disagree at point " << i << "\n";
            return 1;
        }
    }
    return 0;
}

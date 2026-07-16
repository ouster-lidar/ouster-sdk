#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <functional>

#include "visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Hash functor for Eigen::Vector3i (voxel indices).
 *
 * Mixes the three int32 components into a 64-bit value, then runs a
 * splitmix64 finalizer so the upper bits of size_t are populated on
 * 64-bit hosts.
 */
struct OUSTER_API_CLASS Vector3iHash {
    /// Compute hash of a voxel index.
    /// @param[in] voxel The voxel index to hash.
    /// @return Hash value.
    OUSTER_API_FUNCTION
    std::size_t operator()(const Eigen::Vector3i& voxel) const noexcept {
        const auto* v = reinterpret_cast<const std::uint32_t*>(voxel.data());
        std::uint64_t h = static_cast<std::uint64_t>(v[0]) * 73856093ULL;
        h ^= static_cast<std::uint64_t>(v[1]) * 19349669ULL;
        h ^= static_cast<std::uint64_t>(v[2]) * 83492791ULL;
        // splitmix64 finalizer for better avalanche on 64-bit hosts.
        h ^= h >> 33;
        h *= 0xff51afd7ed558ccdULL;
        h ^= h >> 33;
        h *= 0xc4ceb9fe1a85ec53ULL;
        h ^= h >> 33;
        return static_cast<std::size_t>(h);
    }
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster

/// Compatibility hash specialization for `Eigen::Vector3i`.
///
/// This keeps `std::unordered_{set,map}<Eigen::Vector3i, ...>` working in
/// tests and downstream code. New code in this library should prefer
/// `ouster::sdk::core::Vector3iHash` passed explicitly to the container.
namespace std {

/// Hash specialization for `Eigen::Vector3i`.
///
/// Delegates to `ouster::sdk::core::Vector3iHash` for consistent hashing
/// across the library.
template <>
struct hash<Eigen::Vector3i> {
    /// Compute hash of a voxel index.
    /// @param[in] voxel The voxel index to hash.
    /// @return Hash value.
    std::size_t operator()(const Eigen::Vector3i& voxel) const noexcept {
        return ouster::sdk::core::Vector3iHash{}(voxel);
    }
};

}  // namespace std

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace ouster {
namespace sdk {
namespace algorithm {
namespace impl {

template <typename T>
inline void hash_combine(std::size_t& seed, const T& value) {
    seed ^= std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct SpatialHashCell2D {
    int64_t x = 0;
    int64_t y = 0;

    bool operator==(const SpatialHashCell2D& other) const {
        return x == other.x && y == other.y;
    }
};

struct SpatialHashCell2DHash {
    std::size_t operator()(const SpatialHashCell2D& key) const {
        std::size_t seed = std::hash<int64_t>{}(key.x);
        hash_combine(seed, key.y);
        return seed;
    }
};

struct SpatialHashCell3D {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator==(const SpatialHashCell3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    /// Lexicographic ordering so SpatialHashCell3D can be used as a
    /// std::map key, giving deterministic iteration across all platforms.
    bool operator<(const SpatialHashCell3D& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

struct SpatialHashCell3DHash {
    std::size_t operator()(const SpatialHashCell3D& key) const {
        std::size_t seed = std::hash<int64_t>{}(key.x);
        hash_combine(seed, key.y);
        hash_combine(seed, key.z);
        return seed;
    }
};

inline SpatialHashCell2D compute_cell_xy_from_inverse_cell_size(const Eigen::Vector3d& pos,
                                                                double inv_cell_size) {
    return {static_cast<int64_t>(std::floor(pos.x() * inv_cell_size)),
            static_cast<int64_t>(std::floor(pos.y() * inv_cell_size))};
}

inline SpatialHashCell2D compute_cell_xy(const Eigen::Vector3d& pos, double cell_size) {
    return compute_cell_xy_from_inverse_cell_size(pos, 1.0 / cell_size);
}

inline SpatialHashCell3D compute_cell_xyz_from_inverse_cell_size(const Eigen::Vector3d& pos,
                                                                 double inv_cell_size) {
    return {static_cast<int64_t>(std::floor(pos.x() * inv_cell_size)),
            static_cast<int64_t>(std::floor(pos.y() * inv_cell_size)),
            static_cast<int64_t>(std::floor(pos.z() * inv_cell_size))};
}

inline SpatialHashCell3D compute_cell_xyz(const Eigen::Vector3d& pos, double cell_size) {
    return compute_cell_xyz_from_inverse_cell_size(pos, 1.0 / cell_size);
}

template <typename Func>
inline void for_each_neighbor_cell(const SpatialHashCell2D& center, Func&& fn) {
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            fn(SpatialHashCell2D{center.x + dx, center.y + dy});
        }
    }
}

template <typename Func>
inline void for_each_neighbor_cell(const SpatialHashCell3D& center, Func&& fn) {
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                fn(SpatialHashCell3D{center.x + dx, center.y + dy, center.z + dz});
            }
        }
    }
}

}  // namespace impl
}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster

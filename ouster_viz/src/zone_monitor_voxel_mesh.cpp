#include "ouster/impl/zone_monitor_voxel_mesh.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "ouster/lidar_scan.h"  // NOLINT(misc-include-cleaner,unused-includes) needed for destagger
#include "ouster/point_viz.h"
#include "ouster/typedefs.h"
#include "ouster/types.h"

using ouster::sdk::core::SensorInfo;
using std::array;
using std::map;
using std::pair;
using std::vector;

namespace ouster {
namespace sdk {
namespace viz {

// Interpolates a vector at a floating point index. Clamps to the ends of the
// vector if the index is out of range. Throws std::invalid_argument if the
// vector is empty.
template <typename Val, typename Idx, typename>
Val lerp_vector(const std::vector<Val>& vec, Idx idx) {
    if (vec.empty()) {
        throw std::invalid_argument("Cannot interpolate an empty vector.");
    }
    if (idx < 0.0) {
        idx = 0.0;
    }
    if (idx >= vec.size() - 1) {
        idx = vec.size() - 1.0;
    }
    int idx0 = static_cast<int>(std::floor(idx));
    int idx1 = static_cast<int>(std::ceil(idx));
    if (idx0 == idx1) {
        return vec[idx0];
    }
    Idx weight = idx - static_cast<Idx>(idx0);
    // Pretty much everyone understands operator precedence. The extra
    // parentheses are just to keep Clang-Tidy happy.
    return ((1.0 - weight) * vec[idx0]) + (weight * vec[idx1]);
}

// Returns a direction vector for a given pixel (row, col) in the sensor frame,
// without accounting for the lidar to sensor transform. The beam angles are
// interpolated for sub-pixel accuracy, something that is necessary to compute
// the voxel face corners. The voxels themselves have no physical meaning, so
// we're not concerned that this interpolation would not correspond to a real
// beam angle.
Eigen::Vector3f direction_for_pixel(const SensorInfo& sensor_info, double row,
                                    double col) {
    double theta_encoder = 2.0 * M_PI * (1.0 - col / sensor_info.w());
    double phi =
        M_PI * lerp_vector(sensor_info.beam_altitude_angles, row) / 180.0;
    Eigen::Vector3f direction(
        static_cast<float>(std::cos(theta_encoder) * std::cos(phi)),
        static_cast<float>(std::sin(theta_encoder) * std::cos(phi)),
        static_cast<float>(std::sin(phi)));
    return direction;
}

// Returns a direction vector for a given pixel (row, col) in the sensor frame,
// accounting for the lidar to sensor transform.
Eigen::Vector3f direction_rot(const SensorInfo& sensor_info, double row,
                              double col) {
    auto dir = direction_for_pixel(sensor_info, row, col);
    auto& rot =
        sensor_info.lidar_to_sensor_transform.topLeftCorner(3, 3).cast<float>();
    return rot * dir / 1000.0;  // convert mm to meters
}

// Returns an offset vector for a given pixel (row, col) in the sensor frame,
// accounting for the beam to lidar and lidar to sensor transforms. Once again
// this is interpolated for sub-pixel accuracy to compute voxel face corners.
Eigen::Vector3f offset(const SensorInfo& sensor_info, double row, double col) {
    double beam_to_lidar_euclidean_distance_mm =
        std::sqrt((sensor_info.beam_to_lidar_transform(0, 3) *
                   sensor_info.beam_to_lidar_transform(0, 3)) +
                  (sensor_info.beam_to_lidar_transform(2, 3) *
                   sensor_info.beam_to_lidar_transform(2, 3)));

    double theta_encoder = 2.0 * M_PI * (1.0 - col / sensor_info.w());
    Eigen::Vector3f dir = direction_for_pixel(sensor_info, row, col);

    Eigen::Vector3f offset_v(
        static_cast<float>((std::cos(theta_encoder) *
                            sensor_info.beam_to_lidar_transform(0, 3)) -
                           (dir.x() * beam_to_lidar_euclidean_distance_mm)),
        static_cast<float>((std::sin(theta_encoder) *
                            sensor_info.beam_to_lidar_transform(0, 3)) -
                           (dir.y() * beam_to_lidar_euclidean_distance_mm)),
        static_cast<float>((-dir.z() * beam_to_lidar_euclidean_distance_mm +
                            sensor_info.beam_to_lidar_transform(2, 3))));
    auto translation =
        sensor_info.lidar_to_sensor_transform.topRightCorner(3, 1)
            .cast<float>();
    auto res = (offset_v + translation) / 1000.0f;
    return res;
}

// Pre-computes direction and offset vectors for every voxel corner.
VertexLookupTable precompute_voxel_vertices(const SensorInfo& metadata) {
    // The grid of vertices is one larger than the grid of pixels in each
    // dimension.
    const int n_rows = static_cast<int>(metadata.h());
    const int n_cols = static_cast<int>(metadata.w());
    VertexLookupTable lookup_table(n_rows + 1,
                                   std::vector<VoxelVertexData>(n_cols + 1));

    for (int row = 0; row <= n_rows; ++row) {
        for (int col = 0; col <= n_cols; ++col) {
            // Each (row, col) in this grid corresponds to the geometric point
            // (j-0.5, i-0.5)
            lookup_table[row][col] = {
                direction_rot(metadata, row - 0.5, col - 0.5),
                offset(metadata, row - 0.5, col - 0.5)};
        }
    }
    return lookup_table;
}

// Generate the voxels for each pixel in the range images, and stitch them
// together. The vertex positions are calculated using the pre-computed
// direction and offset vectors from the lookup table, scaled by the range value
// for that pixel. Faces and edges are optionally added to the mesh. The mesh is
// constructed in two stages:
// 1. Populate Voxel Caps: For each pixel in the range images, create the
//    top and bottom faces of the voxel using the pre-computed vertex data.
// 2. Stitch Neighbors: Connect adjacent voxels by creating faces and edges
//    between them. This is done for both the near and far faces, as well as
//    between the near and far faces.
// The result is a watertight mesh with shared vertices between adjacent voxels.
std::tuple<std::vector<Vertex3f>, std::vector<uint32_t>, std::vector<uint32_t>>
voxel_style_mesh_components_from_range_images_with_lut(
    ouster::sdk::core::img_t<uint32_t> near_range_image_mm,
    ouster::sdk::core::img_t<uint32_t> far_range_image_mm,
    const SensorInfo& metadata, const VertexLookupTable& vertex_lookup,
    bool add_faces, bool add_edges) {
    auto normalize = [](const Eigen::Vector3f& vec) {
        return vec / vec.norm();
    };
    vector<Vertex3f> vertices;
    vector<uint32_t> edges;
    vector<uint32_t> faces;
    uint32_t num_vertices = 0;

    // This map keeps track of the vertices created for each pixel in each face.
    // We'll use it to stitch together adjacent voxels.
    vector<map<pair<int, int>, vector<Vertex3f>>> verts = {
        map<pair<int, int>, vector<Vertex3f>>(),
        map<pair<int, int>, vector<Vertex3f>>()};

    auto near_destaggered = ouster::sdk::core::destagger<uint32_t>(
        near_range_image_mm, metadata.format.pixel_shift_by_row, false);
    auto far_destaggered = ouster::sdk::core::destagger<uint32_t>(
        far_range_image_mm, metadata.format.pixel_shift_by_row, false);
    const array<decltype(near_destaggered), 2> destaggered_faces = {
        near_destaggered, far_destaggered};

    const int n_rows = static_cast<int>(metadata.h());
    const int n_cols = static_cast<int>(metadata.w());

    // STAGE 1: Populate Voxel Caps
    for (int row = 1; row < n_rows - 1; row++) {
        for (int col = 1; col < n_cols - 1; col++) {
            if (destaggered_faces[1](row, col) == 0) {
                continue;
            }

            // The per-pixel center direction is not shared, so calculate it
            // here.
            const auto center_direction = direction_rot(metadata, row, col);
            const auto face_normal = -normalize(center_direction);

            // Get the four corners for pixel (row,col) from the shared vertex
            // lookup table. Pixel (row,col) is defined by vertices (row,col),
            // (row,i+1), (j+1,i+1), (j+1,col).
            const array<const VoxelVertexData*, 4> corners = {
                &vertex_lookup[row][col],          // Top-left
                &vertex_lookup[row][col + 1],      // Top-right
                &vertex_lookup[row + 1][col + 1],  // Bottom-right
                &vertex_lookup[row + 1][col]       // Bottom-left
            };

            for (size_t face_idx = 0; face_idx < 2; ++face_idx) {
                auto rng = static_cast<float>(destaggered_faces[face_idx](
                    row,
                    col));  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

                vector<Vertex3f> new_verts = {
                    Vertex3f(rng * corners[0]->direction + corners[0]->offset,
                             face_normal),
                    Vertex3f(rng * corners[1]->direction + corners[1]->offset,
                             face_normal),
                    Vertex3f(rng * corners[2]->direction + corners[2]->offset,
                             face_normal),
                    Vertex3f(rng * corners[3]->direction + corners[3]->offset,
                             face_normal)};

                vertices.insert(vertices.end(), new_verts.begin(),
                                new_verts.end());
                verts[face_idx][{row, col}] = new_verts;

                if (add_faces) {
                    faces.insert(
                        faces.end(),
                        {num_vertices + 3, num_vertices + 1, num_vertices + 0,
                         num_vertices + 1, num_vertices + 3, num_vertices + 2});
                }

                if (add_edges) {
                    edges.insert(
                        edges.end(),
                        {num_vertices + 0, num_vertices + 1, num_vertices + 1,
                         num_vertices + 2, num_vertices + 2, num_vertices + 3,
                         num_vertices + 3, num_vertices + 0});
                }
                num_vertices += static_cast<uint32_t>(new_verts.size());
            }
        }
    }

    // STAGE 2: Stitch Neighbors
    // Now that all voxels are created, connect adjacent neighbors.
    // Part A: Stitch neighbors on the same face (near-to-near, far-to-far)
    for (size_t face_idx = 0; face_idx < 2; ++face_idx) {
        for (const auto& pixel_and_vertex : verts[face_idx]) {
            auto& pixel = pixel_and_vertex.first;
            auto& a = pixel_and_vertex.second;
            const int row = pixel.first;
            const int col = pixel.second;

            if (verts[face_idx].count({row, col + 1})) {  // Right neighbor
                const auto& b = verts[face_idx].at({row, col + 1});
                auto normal =
                    normalize((a[2].position - a[1].position)
                                  .cross(b[0].position - a[1].position));
                vector<Vertex3f> new_verts = {Vertex3f(a[1].position, normal),
                                              Vertex3f(b[0].position, normal),
                                              Vertex3f(b[3].position, normal),
                                              Vertex3f(a[2].position, normal)};
                vertices.insert(vertices.end(), new_verts.begin(),
                                new_verts.end());
                if (add_faces) {
                    faces.insert(
                        faces.end(),
                        {num_vertices + 0, num_vertices + 3, num_vertices + 1,
                         num_vertices + 3, num_vertices + 2, num_vertices + 1});
                }
                if (add_edges) {
                    edges.insert(edges.end(),
                                 {num_vertices + 0, num_vertices + 1,
                                  num_vertices + 2, num_vertices + 3});
                }
                num_vertices += static_cast<uint32_t>(new_verts.size());
            }

            if (verts[face_idx].count({row + 1, col})) {  // Bottom neighbor
                const auto& b = verts[face_idx].at({row + 1, col});
                auto normal =
                    normalize((b[0].position - a[3].position)
                                  .cross(b[1].position - a[3].position));
                vector<Vertex3f> new_verts = {Vertex3f(a[3].position, normal),
                                              Vertex3f(a[2].position, normal),
                                              Vertex3f(b[0].position, normal),
                                              Vertex3f(b[1].position, normal)};
                vertices.insert(vertices.end(), new_verts.begin(),
                                new_verts.end());
                if (add_faces) {
                    faces.insert(
                        faces.end(),
                        {num_vertices + 0, num_vertices + 1, num_vertices + 2,
                         num_vertices + 0, num_vertices + 2, num_vertices + 3});
                }
                if (add_edges) {
                    edges.insert(edges.end(),
                                 {num_vertices + 0, num_vertices + 2,
                                  num_vertices + 1, num_vertices + 3});
                }
                num_vertices += static_cast<uint32_t>(new_verts.size());
            }
        }
    }

    // Part B: Stitch near faces to far faces
    auto add_connecting_wall = [&](const vector<Vertex3f>& new_verts) {
        vertices.insert(vertices.end(), new_verts.begin(), new_verts.end());
        if (add_edges) {
            edges.insert(edges.end(), {num_vertices + 0, num_vertices + 2});
        }
        if (add_faces) {
            faces.insert(faces.end(), {num_vertices + 0, num_vertices + 1,
                                       num_vertices + 2, num_vertices + 2,
                                       num_vertices + 3, num_vertices + 1});
        }
        num_vertices += static_cast<uint32_t>(new_verts.size());
    };

    for (const auto& pixel_and_vertex : verts[0]) {
        auto& pixel = pixel_and_vertex.first;
        auto& a = pixel_and_vertex.second;
        auto it = verts[1].find(pixel);
        if (it == verts[1].end()) {
            continue;
        }
        const auto& b = it->second;
        const int row = pixel.first;
        const int col = pixel.second;

        if (verts[0].count({row - 1, col}) == 0) {  // Top wall
            auto normal = normalize((a[1].position - a[0].position)
                                        .cross(b[1].position - a[0].position));
            add_connecting_wall({Vertex3f(a[0].position, normal),
                                 Vertex3f(a[1].position, normal),
                                 Vertex3f(b[0].position, normal),
                                 Vertex3f(b[1].position, normal)});
        }
        if (verts[0].count({row + 1, col}) == 0) {  // Bottom wall
            auto normal = normalize((a[2].position - a[3].position)
                                        .cross(b[2].position - a[3].position));
            add_connecting_wall({Vertex3f(b[2].position, normal),
                                 Vertex3f(b[3].position, normal),
                                 Vertex3f(a[2].position, normal),
                                 Vertex3f(a[3].position, normal)});
        }
        if (verts[0].count({row, col - 1}) == 0) {  // Left wall
            auto normal = normalize((a[3].position - a[0].position)
                                        .cross(b[3].position - a[0].position));
            add_connecting_wall({Vertex3f(a[0].position, normal),
                                 Vertex3f(a[3].position, normal),
                                 Vertex3f(b[0].position, normal),
                                 Vertex3f(b[3].position, normal)});
        }
        if (verts[0].count({row, col + 1}) == 0) {  // Right wall
            auto normal = normalize((a[2].position - a[1].position)
                                        .cross(b[2].position - a[1].position));
            add_connecting_wall({Vertex3f(a[1].position, normal),
                                 Vertex3f(a[2].position, normal),
                                 Vertex3f(b[1].position, normal),
                                 Vertex3f(b[2].position, normal)});
        }
    }

    return {vertices, faces, edges};
}

ouster::sdk::viz::Mesh voxel_style_mesh_from_zone_image_pair(
    ouster::sdk::core::Zrb& zrb,  // TODO[tws] const
    const SensorInfo& sensor_info, const VertexLookupTable& vertex_lookup) {
    auto res = voxel_style_mesh_components_from_range_images_with_lut(
        zrb.near_range_mm, zrb.far_range_mm, sensor_info, vertex_lookup, false,
        true);
    return ouster::sdk::viz::Mesh(std::move(std::get<0>(res)),
                                  std::move(std::get<1>(res)),
                                  std::move(std::get<2>(res)));
}

}  // namespace viz
}  // namespace sdk
}  // namespace ouster

#pragma once

#include <tuple>
#include <vector>

#include "ouster/point_viz.h"
#include "ouster/typedefs.h"
#include "ouster/types.h"
#include "ouster/zrb.h"

namespace ouster {
namespace sdk {
namespace viz {

/**
 * Simple linear interpolation of a vector at a floating point index.
 * Clamps to the ends of the vector if the index is out of range.
 * @param[in] v the vector to interpolate.
 * @param[in] idx the floating point index to interpolate at.
 * @throws std::invalid_argument if the vector is empty.
 * @returns the interpolated value.
 */
template <typename Val, typename Idx,
          typename = std::enable_if_t<std::is_floating_point<Val>::value &&
                                      std::is_floating_point<Idx>::value>>
Val lerp_vector(const std::vector<Val>& v, Idx idx);

/**
 * Interpolate the beam altitude angle for sub-pixel accuracy.
 * @param[in] sensor_info the sensor info containing beam altitude angles.
 * @param[in] row the floating point row index to interpolate at.
 * @return the interpolated altitude angle in degrees.
 */
double slow_altitude_angle(const ouster::sdk::core::SensorInfo& sensor_info,
                           double row);

/** Get the sub-pixel direction vector accounting for the lidar to sensor
 * transform.
 * @param[in] sensor_info the SensorInfo containing beam angles and transforms.
 * @param[in] row the floating point row index to interpolate at.
 * @param[in] col the floating point column index to interpolate at.
 * @return the direction vector for the pixel (row, col).
 */
Eigen::Vector3f direction_rot(const ouster::sdk::core::SensorInfo& sensor_info,
                              double row, double col);

/** Get the sub-pixel offset vector accounting for the beam to lidar and
 * lidar to sensor transforms.
 * @param[in] sensor_info the SensorInfo containing beam angles and transforms.
 * @param[in] row the floating point row index to interpolate at.
 * @param[in] col the floating point column index to interpolate at.
 * @return the offset vector for the pixel (row, col).
 */
Eigen::Vector3f offset(const ouster::sdk::core::SensorInfo& sensor_info,
                       double row, double col);

/** Direction and offset data for a voxel vertex. */
struct VoxelVertexData {
    Eigen::Vector3f direction;
    Eigen::Vector3f offset;
};

// Lookup table of pre-computed vertex data for each pixel corner.
using VertexLookupTable = std::vector<std::vector<VoxelVertexData>>;

/**
 * Pre-computes direction and offset vectors for every vertex in the pixel grid,
 * allowing us to re-use them for creating voxel meshes for multiple image
 * pairs.
 * @param[in] metadata the sensor metadata.
 * @return a lookup table of vertex data for each pixel corner.
 */
VertexLookupTable precompute_voxel_vertices(
    const ouster::sdk::core::SensorInfo& metadata);

/** Generate the voxels for each pixel in the range images, and stitch them
 * together. The vertex positions are calculated using the pre-computed
 * direction and offset vectors from the lookup table, scaled by the range value
 * for that pixel. The mesh is constructed in two stages:
 * 1. Populate Voxel Caps: Create the near and far faces of each voxel using
 *    the pre-computed vertex data.
 * 2. Stitch Neighbors: Connect adjacent voxels by creating faces and edges
 *    between them. This is done for both the near and far faces, as well as
 *    between the near and far faces.
 * Faces and edges are optionally added to the mesh.
 * @param[in] near_range_image_mm the near range image in mm.
 * @param[in] far_range_image_mm the far range image in mm.
 * @param[in] metadata the sensor metadata.
 * @param[in] vertex_lookup the pre-computed vertex lookup table.
 * @param[in] add_faces whether to add faces to the mesh.
 * @param[in] add_edges whether to add edges to the mesh.
 * @return a tuple of (vertices, faces, edges) for the mesh that can be used to
 * construct a ouster::sdk::viz::Mesh.
 */
std::tuple<std::vector<Vertex3f>, std::vector<uint32_t>, std::vector<uint32_t>>
voxel_style_mesh_components_from_range_images_with_lut(
    ouster::sdk::core::img_t<uint32_t> near_range_image_mm,
    ouster::sdk::core::img_t<uint32_t> far_range_image_mm,
    const ouster::sdk::core::SensorInfo& metadata,
    const VertexLookupTable& vertex_lookup, bool add_faces, bool add_edges);

/**
 * Create a voxel-style mesh from a Zrb using pre-computed vertex
 * data.
 * @param[in] zone_image_pair the ZoneImagePair containing near and far range
 * images.
 * @param[in] sensor_info the sensor metadata.
 * @param[in] vertex_lookup the pre-computed vertex lookup table.
 * @return a ouster::sdk::viz::Mesh representing the voxel-style mesh.
 */
ouster::sdk::viz::Mesh voxel_style_mesh_from_zone_image_pair(
    ouster::sdk::core::Zrb& zone_image_pair,  // TODO[tws] const
    const ouster::sdk::core::SensorInfo& sensor_info,
    const VertexLookupTable& vertex_lookup);

}  // namespace viz
}  // namespace sdk
}  // namespace ouster

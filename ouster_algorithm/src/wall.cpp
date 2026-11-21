/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Wall extraction from a frame, and the wall support gate
 */

#include "ouster/algorithm/impl/wall.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster/algorithm/impl/spatial_hash.h"
#include "ouster/algorithm/normals.h"
#include "ouster/algorithm/plumb.h"
#include "ouster/core/chanfield.h"
#include "ouster/core/field.h"
#include "ouster/core/frame.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/impl/dewarp_impl.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/pose_util.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"

namespace ChanField = ouster::sdk::core::ChanField;
using ouster::sdk::algorithm::impl::cell_key;
using ouster::sdk::core::DEG_TO_RAD;
using ouster::sdk::core::logger;
using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::MatrixX3dR;
using ouster::sdk::core::PointCloudXYZd;
using ouster::sdk::core::impl::PoseH;

namespace ouster {
namespace sdk {
namespace algorithm {

namespace {

// Cell size in meters of the top-down grid the support check rasterizes
// the wall points into.
constexpr double SUPPORT_CELL_M = 0.25;
constexpr double SUPPORT_INV_CELL_M = 1.0 / SUPPORT_CELL_M;
// The largest continuous patch of wall cells covers at least this many
// square meters before the points count as a wall. Seen through gaps in
// foliage at twenty meters a real facade often shows only a two by two
// meter slice of itself, so this pairs with the minimum horizontal run
// below rather than carrying the whole gate alone.
constexpr double MIN_SUPPORT_AREA_M2 = 4.0;

// A cell must span this much vertically before it counts as wall, so ground
// clutter and low objects never raise one.
constexpr double WALL_SUPPORT_MIN_CELL_HEIGHT_M = 0.5;
// ... and holds at least this many points, so a few stray returns strung
// out in height cannot raise one either.
constexpr double WALL_SUPPORT_MIN_CELL_POINTS = 3.0;
// Radius in cells of the window the local footprint of a cell is measured
// over (2 spans 5 x 5 cells, 1.25 m at the default cell size).
constexpr int WALL_SUPPORT_WINDOW_CELLS = 2;
// The surface normals over that window agree at least this well, measured
// as the length of their mean: one surface carries a mean near 1, while
// foliage and other volume-filling clutter scatters its normals and
// carries a mean near 0. Asking the normals rather than the shape of the
// top-down shadow is what lets the eaves, bay window and setback faces of
// one building each count as the wall they are, instead of smearing
// together into a footprint too thick to be one.
constexpr double WALL_SUPPORT_MIN_NORMAL_COHERENCE = 0.88;
// The footprint over that window reaches at least this far (RMS, in
// meters) along its dominant direction, so a pole or a lone corner of
// returns is not mistaken for a wall.
constexpr double WALL_SUPPORT_MIN_EXTENT_M = 0.2;
// The largest continuous patch also runs at least this far horizontally,
// measured as the length of the uniform segment with the same second
// moment. A wall runs along the ground; a pole, a tree trunk or a lone
// corner of returns does not, and area alone cannot tell them apart -- a
// trunk seen over seven meters of height covers as much surface as a real
// wall fragment two meters wide.
constexpr double WALL_SUPPORT_MIN_RUN_M = 2.0;
// Neighborhood parameters handed to normals().
constexpr int NORMALS_PIXEL_SEARCH_RANGE = 3;
constexpr double NORMALS_TARGET_DISTANCE_M = 0.1;

// Running xy moments of a set of points: enough to recover the RMS
// extents of their footprint, and summable so the footprint of a union of
// sets is the sum of their moments.
struct FootprintMoments {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double sum_yy = 0.0;
    double count = 0.0;

    void add(double x, double y) {
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_yy += y * y;
        count += 1.0;
    }

    void add(const FootprintMoments& other) {
        sum_x += other.sum_x;
        sum_y += other.sum_y;
        sum_xx += other.sum_xx;
        sum_xy += other.sum_xy;
        sum_yy += other.sum_yy;
        count += other.count;
    }
};

// One cell of the top-down support grid.
// The xy moments and the summed unit normals are kept so the footprint and
// the normal agreement around a cell can both be measured by summing them
// over a window, without a second pass over the points.
struct SupportCell {
    int64_t cell_x = 0;
    int64_t cell_y = 0;
    double z_min = 0.0;
    double z_max = 0.0;
    // Moments of the points in the cell; its count is the point count.
    FootprintMoments footprint;
    Eigen::Vector3d sum_normal = Eigen::Vector3d::Zero();
    bool visited = false;
    bool qualified = false;
};

// RMS extent along the footprint's dominant direction: the square root
// of the largest eigenvalue of its 2D covariance.
double footprint_extent(const FootprintMoments& moments) {
    const double count = moments.count;
    const double mean_x = moments.sum_x / count;
    const double mean_y = moments.sum_y / count;
    const double var_x = moments.sum_xx / count - mean_x * mean_x;
    const double var_y = moments.sum_yy / count - mean_y * mean_y;
    const double cov_xy = moments.sum_xy / count - mean_x * mean_y;
    const double half_trace = 0.5 * (var_x + var_y);
    const double half_gap = 0.5 * (var_x - var_y);
    const double spread = std::sqrt(half_gap * half_gap + cov_xy * cov_xy);
    return std::sqrt(std::max(half_trace + spread, 0.0));
}

// Wall segmentation runs on any Frame that carries a RANGE field. The two
// helpers below read the lidar specifics off the frame's metadata when it has
// them and otherwise answer what a camera (or any other non-lidar sensor) is:
// a single return on an unstaggered image.

// Only a lidar is staggered -- its columns are captured at swept azimuths. A
// camera's pixels already lie on a regular grid, so a zero shift per row makes
// every destagger/restagger step a no-op.
std::vector<int> frame_pixel_shift_by_row(const core::Frame& frame) {
    const auto* lidar_info = dynamic_cast<const core::LidarInfo*>(frame.sensor_info().get());
    if (lidar_info != nullptr) {
        return lidar_info->format.pixel_shift_by_row;
    }
    return std::vector<int>(frame.h, 0);
}

// A lidar exposes one or more returns; anything else has a single one.
int frame_num_returns(const core::Frame& frame) {
    const auto* lidar_info = dynamic_cast<const core::LidarInfo*>(frame.sensor_info().get());
    return lidar_info != nullptr ? lidar_info->num_returns() : 1;
}

// Load and destagger a stored global-coordinate NORMALS field. Returns false
// when the field is missing or unusable.
bool try_load_global_normals(const core::Frame& frame, const std::string& field_name,
                             MatrixX3dR& destaggered_normals) {
    if (!frame.has_field(field_name)) {
        return false;
    }
    const auto& field = frame.field(field_name);
    const auto& shape = field.shape();
    const std::vector<int> pixel_shifts = frame_pixel_shift_by_row(frame);
    if (field.tag() != core::ChanFieldType::FLOAT32 || shape.size() != 3 || shape[0] != frame.h ||
        shape[1] != frame.w || shape[2] != 3u || pixel_shifts.size() != frame.h) {
        return false;
    }
    const auto values = static_cast<core::ConstArrayView3<float>>(field);
    if (values.sparse()) {
        return false;
    }

    const Eigen::Index height = static_cast<Eigen::Index>(frame.h);
    const Eigen::Index width = static_cast<Eigen::Index>(frame.w);
    core::MatrixX3fR shifted(height * width, 3);
    const Eigen::TensorMap<const Eigen::Tensor<float, 3, Eigen::RowMajor>> input(values.data(),
                                                                                 height, width, 3);
    Eigen::TensorMap<Eigen::Tensor<float, 3, Eigen::RowMajor>> output(shifted.data(), height, width,
                                                                      3);
    core::destagger_into<float, 3>(input, pixel_shifts, false, output);

    destaggered_normals = shifted.cast<double>();
    return true;
}

// Destaggered cloud, range image and normals of one return.
struct ReturnGeometry {
    PointCloudXYZd points;
    core::img_t<uint32_t> range;
    MatrixX3dR normals;
};

// Dewarp, destagger and estimate normals for the returns of one frame.
std::vector<ReturnGeometry> prepare_returns(const core::Frame& frame,
                                            const Matrix4dR& gravity_rotation) {
    const auto& info = *frame.sensor_info();
    const int frame_width = static_cast<int>(frame.w);
    const std::vector<int> pixel_shifts = frame_pixel_shift_by_row(frame);

    std::vector<std::string> range_fields{ChanField::RANGE};
    if (frame.has_field(ChanField::RANGE2)) {
        range_fields.emplace_back(ChanField::RANGE2);
    }

    // Use the same global-to-wall transform for points and stored normals. A
    // lidar frame carries one pose per column; a camera frame carries a single
    // pose (index 0), so every pixel shares it.
    const int first_col = frame.get_first_valid_column();
    const auto& pose_shape = frame.body_to_world().shape();
    const int pose_count = pose_shape.empty() ? 0 : static_cast<int>(pose_shape[0]);
    const bool per_column_poses = pose_count == frame_width;
    const Matrix4dR global_to_wall =
        gravity_rotation * Matrix4dR(PoseH(frame.get_column_pose(first_col))).inverse();
    core::MatrixX16dR rel_poses(frame_width, 16);
    for (int col = 0; col < frame_width; ++col) {
        const int pose_index = per_column_poses ? col : 0;
        Eigen::Map<Matrix4dR>(rel_poses.row(col).data()) =
            global_to_wall * Matrix4dR(PoseH(frame.get_column_pose(pose_index)));
    }

    // The sensor's cached table, so a streaming caller does not rebuild it
    // for every frame.
    const auto xyz_lut = info.xyzlut<double>();
    std::vector<ReturnGeometry> returns(range_fields.size());
    for (size_t ret = 0; ret < range_fields.size(); ++ret) {
        Eigen::Ref<const core::img_t<uint32_t>> range = frame.field(range_fields[ret]);
        const Eigen::Index height = range.rows();
        const Eigen::Index width = range.cols();
        const PointCloudXYZd points = (*xyz_lut)(range);

        PointCloudXYZd dewarped(points.rows(), points.cols());
        core::dewarp<double>(dewarped, points, rel_poses);

        // Destagger the range image and the dewarped cloud so normal
        // estimation sees spatially adjacent neighbours.
        returns[ret].range = core::destagger<uint32_t>(range, pixel_shifts);
        returns[ret].points.resize(dewarped.rows(), 3);
        const Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>> dewarped_view(
            dewarped.data(), height, width, 3);
        Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>> destaggered_view(
            returns[ret].points.data(), height, width, 3);
        core::destagger_into<double, 3>(dewarped_view, pixel_shifts, false, destaggered_view);
    }

    // A frame that already carries normals is taken at its word rather
    // than estimated again -- that estimation is the most expensive step
    // here. A dual-return frame missing one of the two fields is recomputed
    // whole, so both returns stay coherent with each other.
    const bool loaded_first =
        try_load_global_normals(frame, ChanField::NORMALS, returns[0].normals);
    const bool loaded_second =
        loaded_first && (returns.size() < 2 ||
                         try_load_global_normals(frame, ChanField::NORMALS2, returns[1].normals));
    if (loaded_first && loaded_second) {
        // Wall points are expressed relative to the first valid column and
        // gravity-aligned, so convert the stored global normals to match.
        for (ReturnGeometry& geometry : returns) {
            geometry.normals *= global_to_wall.block<3, 3>(0, 0).transpose();
        }
        return returns;
    }

    // The dewarp is rotation-only about the frame anchor, so every column
    // uses the corrected extrinsic translation as its sensor origin.
    MatrixX3dR sensor_origins(frame_width, 3);
    sensor_origins.rowwise() =
        (gravity_rotation * info.sensor_to_body).block<3, 1>(0, 3).transpose();

    if (returns.size() > 1) {
        // One estimation for both returns: the vertical subtent is measured
        // once and the neighbour search sees either return, so the two
        // normal maps stay consistent with each other.
        auto normal_pair = normals(returns[0].points, returns[0].range, returns[1].points,
                                   returns[1].range, sensor_origins, NORMALS_PIXEL_SEARCH_RANGE,
                                   DEFAULT_MIN_ANGLE_INCIDENCE_RAD, NORMALS_TARGET_DISTANCE_M);
        returns[0].normals = std::move(normal_pair.first);
        returns[1].normals = std::move(normal_pair.second);
    } else {
        returns[0].normals =
            normals(returns[0].points, returns[0].range, sensor_origins, NORMALS_PIXEL_SEARCH_RANGE,
                    DEFAULT_MIN_ANGLE_INCIDENCE_RAD, NORMALS_TARGET_DISTANCE_M);
    }
    return returns;
}

// The wall candidates of one prepared return: points and unit normals,
// dewarped and destaggered, in the frame's body coordinates rotated by the
// gravity alignment, with the destaggered flat pixel index (row * w + col)
// of each so an answer about the points can be written back per pixel.
struct WallCandidates {
    core::ArrayX3dR points;
    core::ArrayX3dR normals;
    Eigen::ArrayXi pixel_indices;
};

// The wall candidates of one prepared return.
WallCandidates find_wall_candidates(const ReturnGeometry& geometry, double tol_rad) {
    const Eigen::Index height = geometry.range.rows();
    const Eigen::Index width = geometry.range.cols();

    // Wall points have normals within tol_rad of horizontal.
    const double sin_tol = std::sin(tol_rad);
    std::vector<Eigen::Index> wall_rows;
    wall_rows.reserve(static_cast<size_t>(height * width) / 4);
    for (Eigen::Index u = 0; u < height; ++u) {
        for (Eigen::Index v = 0; v < width; ++v) {
            if (geometry.range(u, v) == 0) {
                continue;
            }
            const Eigen::Index flat = u * width + v;
            const Eigen::Vector3d normal = geometry.normals.row(flat).transpose();
            const double normal_norm = normal.norm();
            if (!geometry.points.row(flat).allFinite() || !std::isfinite(normal_norm) ||
                normal_norm <= 0.0) {
                continue;
            }
            if (std::abs(normal.z()) / normal_norm >= sin_tol) {
                continue;
            }
            wall_rows.push_back(flat);
        }
    }

    WallCandidates result;
    const Eigen::Index wall_count = static_cast<Eigen::Index>(wall_rows.size());
    result.points.resize(wall_count, 3);
    result.normals.resize(wall_count, 3);
    result.pixel_indices.resize(wall_count);
    for (Eigen::Index row = 0; row < wall_count; ++row) {
        const Eigen::Index flat = wall_rows[static_cast<size_t>(row)];
        result.points.row(row) = geometry.points.row(flat);
        // Normalized here, so the result is unit whatever normals() returned.
        result.normals.row(row) = geometry.normals.row(flat) / geometry.normals.row(flat).norm();
        result.pixel_indices[row] = static_cast<int>(flat);
    }
    return result;
}

// Index along one axis of the support cell a coordinate falls in.
int64_t support_cell_index(double coordinate) {
    return static_cast<int64_t>(std::floor(coordinate * SUPPORT_INV_CELL_M));
}

// Support cells indexed by their packed grid coordinates.
using SupportGrid = std::unordered_map<int64_t, SupportCell>;

// Rasterize the voting points into a top-down grid, keeping the height
// range, the point count, the xy moments and the summed unit normal of
// every cell.
SupportGrid rasterize_support_grid(const Eigen::Ref<const core::ArrayX3dR>& points,
                                   const Eigen::Ref<const core::ArrayX3dR>& normals) {
    SupportGrid grid;
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const double x = points(i, 0);
        const double y = points(i, 1);
        const double z = points(i, 2);
        const Eigen::Vector3d normal(normals(i, 0), normals(i, 1), normals(i, 2));
        const int64_t cell_x = support_cell_index(x);
        const int64_t cell_y = support_cell_index(y);
        SupportCell& cell = grid[cell_key(cell_x, cell_y)];
        if (cell.footprint.count == 0.0) {
            cell.cell_x = cell_x;
            cell.cell_y = cell_y;
            cell.z_min = z;
            cell.z_max = z;
        }
        cell.z_min = std::min(cell.z_min, z);
        cell.z_max = std::max(cell.z_max, z);
        cell.footprint.add(x, y);
        cell.sum_normal += normal;
    }
    return grid;
}

// Every cell within WALL_SUPPORT_WINDOW_CELLS of `center`, itself
// included, pooled into one: the footprint and the normal agreement
// around a cell are read off these sums.
SupportCell pooled_window(const SupportGrid& grid, const SupportCell& center) {
    SupportCell pooled;
    constexpr int64_t window = WALL_SUPPORT_WINDOW_CELLS;
    for (int64_t dx = -window; dx <= window; ++dx) {
        for (int64_t dy = -window; dy <= window; ++dy) {
            const auto nearby = grid.find(cell_key(center.cell_x + dx, center.cell_y + dy));
            if (nearby == grid.end()) {
                continue;
            }
            const SupportCell& other = nearby->second;
            pooled.footprint.add(other.footprint);
            pooled.sum_normal += other.sum_normal;
        }
    }
    return pooled;
}

// The cells of the grid that are wall: a cell is wall when enough points
// stand tall enough in it, the normals around it agree on one surface,
// and that surface reaches far enough to be more than a pole. The normals
// are what tell a wall from a bush: a building answers a 1.25 m window
// with parallel faces half a meter apart -- wall, eaves, bay window --
// whose pooled top-down shadow is far thicker than any single one of
// them, while their normals point the same way and a bush's do not.
SupportGrid select_wall_cells(const SupportGrid& grid) {
    SupportGrid walls;
    for (const auto& entry : grid) {
        const SupportCell& cell = entry.second;
        if (cell.footprint.count < WALL_SUPPORT_MIN_CELL_POINTS ||
            cell.z_max - cell.z_min < WALL_SUPPORT_MIN_CELL_HEIGHT_M) {
            continue;
        }
        const SupportCell pooled = pooled_window(grid, cell);
        // The mean of unit normals: length 1 when they all point one way,
        // length 0 when they cancel.
        if (pooled.sum_normal.norm() / pooled.footprint.count < WALL_SUPPORT_MIN_NORMAL_COHERENCE) {
            continue;
        }
        if (footprint_extent(pooled.footprint) < WALL_SUPPORT_MIN_EXTENT_M) {
            continue;
        }
        walls.emplace(entry.first, cell);
    }
    return walls;
}

// Push the unvisited 8-neighbors of `cell` onto `stack`, marking them
// visited; a neighbor joins only when its height range overlaps the cell's.
void push_connected_neighbors(SupportGrid& walls, const SupportCell& cell,
                              std::vector<int64_t>& stack) {
    for (int64_t dx = -1; dx <= 1; ++dx) {
        for (int64_t dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            const auto found = walls.find(cell_key(cell.cell_x + dx, cell.cell_y + dy));
            if (found == walls.end() || found->second.visited) {
                continue;
            }
            const SupportCell& neighbor = found->second;
            if (std::min(cell.z_max, neighbor.z_max) <= std::max(cell.z_min, neighbor.z_min)) {
                continue;
            }
            found->second.visited = true;
            stack.push_back(found->first);
        }
    }
}

// Find cells that belong to an 8-connected patch large enough and long
// enough to be a wall. Neighboring cells join only when their height ranges
// overlap.
void mark_qualified_wall_cells(SupportGrid& walls) {
    std::vector<int64_t> stack;
    std::vector<int64_t> patch_cells;
    for (auto& entry : walls) {
        if (entry.second.visited) {
            continue;
        }
        entry.second.visited = true;
        stack.assign(1, entry.first);
        patch_cells.clear();
        double area = 0.0;
        FootprintMoments centers;
        while (!stack.empty()) {
            const int64_t key = stack.back();
            stack.pop_back();
            patch_cells.push_back(key);
            const SupportCell& cell = walls.at(key);
            area += SUPPORT_CELL_M * (cell.z_max - cell.z_min);
            centers.add((static_cast<double>(cell.cell_x) + 0.5) * SUPPORT_CELL_M,
                        (static_cast<double>(cell.cell_y) + 0.5) * SUPPORT_CELL_M);
            push_connected_neighbors(walls, cell, stack);
        }

        // Length of a uniform segment with the same variance along its axis.
        const double run = 2.0 * std::sqrt(3.0) * footprint_extent(centers);
        if (area >= MIN_SUPPORT_AREA_M2 && run >= WALL_SUPPORT_MIN_RUN_M) {
            for (const int64_t key : patch_cells) {
                walls.at(key).qualified = true;
            }
        }
    }
}

// Wall, or not, for each input point: 1 where the point stands on a wall
// cell whose patch cleared the gate, 0 everywhere else.
Eigen::Array<uint8_t, Eigen::Dynamic, 1> label_wall_points(
    const Eigen::Ref<const core::ArrayX3dR>& points, const SupportGrid& walls) {
    Eigen::Array<uint8_t, Eigen::Dynamic, 1> point_is_wall =
        Eigen::Array<uint8_t, Eigen::Dynamic, 1>::Zero(points.rows());
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const auto found = walls.find(
            cell_key(support_cell_index(points(i, 0)), support_cell_index(points(i, 1))));
        if (found != walls.end() && found->second.qualified) {
            point_is_wall[i] = 1;
        }
    }
    return point_is_wall;
}

// Label candidate points that stand on a real, continuous wall rather than
// foliage or clutter.
Eigen::Array<uint8_t, Eigen::Dynamic, 1> wall_support_mask(
    Eigen::Ref<const core::ArrayX3dR> points, Eigen::Ref<const core::ArrayX3dR> normals) {
    const SupportGrid grid = rasterize_support_grid(points, normals);
    SupportGrid walls = select_wall_cells(grid);
    mark_qualified_wall_cells(walls);
    return label_wall_points(points, walls);
}

// Wall, or not, for every pixel of one frame: the candidates of every
// return the frame carries, gated together on one support grid, scattered
// back per pixel and restaggered into the frame's own layout so each mask
// can be stored as a field. What WallSegEngine writes. Throws
// std::invalid_argument on a frame with no sensor_info(), or no RANGE image to
// project (a color-only camera has no geometry to stand a wall on).
std::vector<core::img_t<uint8_t>> wall_mask(const core::Frame& frame,
                                            const Matrix4dR& gravity_rotation) {
    if (!frame.sensor_info()) {
        throw std::invalid_argument("frame.sensor_info() is required for wall segmentation");
    }
    if (!frame.has_field(ChanField::RANGE)) {
        throw std::invalid_argument("frame must contain a RANGE field for wall segmentation");
    }
    const std::vector<ReturnGeometry> returns = prepare_returns(frame, gravity_rotation);
    const double tol_rad = PLUMB_WALLS_ANGLE_TOL_DEG * DEG_TO_RAD;

    std::vector<WallCandidates> candidates;
    candidates.reserve(returns.size());
    Eigen::Index pooled_count = 0;
    for (const ReturnGeometry& geometry : returns) {
        candidates.push_back(find_wall_candidates(geometry, tol_rad));
        pooled_count += candidates.back().points.rows();
    }

    // Both returns are gated together, on one grid: a facade answers with
    // both of them, and pooling makes the patch it stands on denser rather
    // than splitting the same wall into two thinner cases.
    core::ArrayX3dR pooled_points(pooled_count, 3);
    core::ArrayX3dR pooled_normals(pooled_count, 3);
    Eigen::Index pooled_row = 0;
    for (const WallCandidates& candidate : candidates) {
        const Eigen::Index rows = candidate.points.rows();
        pooled_points.middleRows(pooled_row, rows) = candidate.points;
        pooled_normals.middleRows(pooled_row, rows) = candidate.normals;
        pooled_row += rows;
    }
    const Eigen::Array<uint8_t, Eigen::Dynamic, 1> point_is_wall =
        wall_support_mask(pooled_points, pooled_normals);

    // The pipeline works destaggered, so each answer is scattered back in
    // that layout and restaggered into the frame's own.
    const Eigen::Index height = static_cast<Eigen::Index>(frame.h);
    const Eigen::Index width = static_cast<Eigen::Index>(frame.w);
    const std::vector<int> pixel_shifts = frame_pixel_shift_by_row(frame);
    std::vector<core::img_t<uint8_t>> masks;
    masks.reserve(candidates.size());
    Eigen::Index pooled_offset = 0;
    for (const WallCandidates& candidate : candidates) {
        core::img_t<uint8_t> destaggered_mask = core::img_t<uint8_t>::Zero(height, width);
        for (Eigen::Index row = 0; row < candidate.pixel_indices.size(); ++row) {
            if (point_is_wall[pooled_offset + row] == 0) {
                continue;
            }
            const Eigen::Index flat = static_cast<Eigen::Index>(candidate.pixel_indices[row]);
            destaggered_mask(flat / width, flat % width) = 1;
        }
        pooled_offset += candidate.pixel_indices.size();
        masks.push_back(core::stagger<uint8_t>(destaggered_mask, pixel_shifts));
    }
    return masks;
}

// WallSegEngine over wall_mask(): one task per sensor, masks handed back
// per slot, and written as fields only by update().
class MaskWallSegEngine : public WallSegEngine {
   public:
    explicit MaskWallSegEngine(WallSegConfig config) : config_(std::move(config)) {}

    std::vector<std::vector<core::img_t<uint8_t>>> compute_walls(
        const core::FrameSet& frames) const override {
        std::vector<std::vector<core::img_t<uint8_t>>> masks(frames.size());
        std::vector<size_t> valid_indices;
        for (const size_t sensor_idx : frames.valid_indices()) {
            valid_indices.push_back(sensor_idx);
        }
        if (valid_indices.size() <= 1u) {
            for (const size_t idx : valid_indices) {
                masks[idx] = segment_sensor(*frames[idx], idx);
            }
            return masks;
        }

        // Sensors in a FrameSet are independent. Run them concurrently so a
        // multi-sensor source pays approximately the cost of its slowest
        // sensor rather than the sum of all sensor runtimes. Each task owns
        // its own slot of `masks`, which is sized up front.
        std::vector<std::future<void>> tasks;
        tasks.reserve(valid_indices.size());
        for (const size_t idx : valid_indices) {
            try {
                tasks.emplace_back(std::async(std::launch::async, [this, &frames, &masks, idx]() {
                    masks[idx] = segment_sensor(*frames[idx], idx);
                }));
            } catch (const std::system_error& error) {
                throw std::runtime_error("Failed to launch wall-segmentation task: " +
                                         std::string(error.what()));
            }
        }
        for (auto& task : tasks) {
            task.get();
        }
        return masks;
    }

    void update(core::FrameSet& frames) override {
        const std::vector<std::vector<core::img_t<uint8_t>>> masks = compute_walls(frames);
        for (const size_t idx : frames.valid_indices()) {
            write_fields(*frames[idx], masks[idx]);
        }
    }

   private:
    // One sensor's masks. A sensor that cannot be segmented answers empty
    // and is logged rather than failing the set.
    std::vector<core::img_t<uint8_t>> segment_sensor(const core::Frame& frame, size_t idx) const {
        try {
            return wall_mask(frame, config_.gravity_rotation);
        } catch (const std::exception& error) {
            logger().warn("WallSegEngine: no wall mask for sensor {}: {}", idx, error.what());
            return {};
        }
    }

    // Write one sensor's masks as its WALL fields. Empty masks mean the
    // sensor could not be segmented, and its fields are left as they were.
    static void write_fields(core::Frame& frame, const std::vector<core::img_t<uint8_t>>& masks) {
        if (masks.empty()) {
            return;
        }
        const std::vector<size_t> shape{frame.h, frame.w};
        const int max_returns = frame_num_returns(frame);
        for (int ret = 0; ret < max_returns; ++ret) {
            const std::string field_name = ChanField::return_field_name(ChanField::WALL, ret);
            if (frame.has_field(field_name)) {
                frame.del_field(field_name);
            }
            // Returns the frame does not carry get no field rather than a
            // mask of zeros that reads as "no wall here" when nothing was
            // measured at all.
            if (static_cast<size_t>(ret) >= masks.size()) {
                continue;
            }
            frame.add_field(field_name, core::FieldDescriptor::array<uint8_t>(shape),
                            core::FieldClass::PIXEL_FIELD);
            frame.field<uint8_t>(field_name) = masks[static_cast<size_t>(ret)];
        }
    }

    WallSegConfig config_;
};

}  // namespace

std::unique_ptr<WallSegEngine> WallSegEngine::create(const WallSegConfig& config) {
    return std::make_unique<MaskWallSegEngine>(config);
}

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster

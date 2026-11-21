#include "ouster/algorithm/normals.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ouster {
namespace sdk {
namespace algorithm {

using core::img_t;
using core::MatrixX3dR;
using core::PointCloudXYZd;

namespace {

// Range delta threshold to flag 1-pixel-thin foreground objects during neighbor
// checks (used in consider_neighbor).
constexpr uint32_t FOREGROUND_SALIENCE_MM = 500;

void validate_normal_parameters(double min_angle_of_incidence_rad, double target_distance_m) {
    if (!(target_distance_m > 0.0)) {
        throw std::runtime_error("normals: target_distance_m must be positive");
    }
    if (!(min_angle_of_incidence_rad > 0.0)) {
        throw std::runtime_error("normals: min_angle_of_incidence_rad must be positive");
    }
}

// From mid_column outward, find the first column with valid range data at
// both top and bottom, compute the angle between their normalized beam
// vectors, and return the angle divided by the pixel distance between them.
// This gives an estimate of the vertical pixel subtent in radians. If no
// valid pairs are found, fall back to assuming a 90 degree vertical FOV
template <typename RangeValidFn, typename BeamFn>
double compute_vertical_subtent(size_t width, size_t height, const RangeValidFn& has_range,
                                const BeamFn& normalized_beam) {
    const size_t mid_col = width / 2;
    // Search from center column outward (left and right alternately)
    for (size_t col_offset = 0; col_offset <= mid_col; ++col_offset) {
        for (int sign : {-1, 1}) {
            const int col_i = static_cast<int>(mid_col) + sign * static_cast<int>(col_offset);

            // Check column index bounds
            if (col_i < 0 || col_i >= static_cast<int>(width)) {
                continue;
            }
            const size_t col = static_cast<size_t>(col_i);

            // Start from top and bottom of the column
            size_t top = height > 0 ? height - 1 : 0;
            size_t bottom = 0;

            // Move top and bottom inward until both have valid range data
            while (top > bottom) {
                const bool top_valid = has_range(top, col);
                const bool bottom_valid = has_range(bottom, col);
                if (top_valid && bottom_valid) {
                    const auto v_top = normalized_beam(top, col);
                    const auto v_bottom = normalized_beam(bottom, col);

                    double dot_product = v_top.dot(v_bottom);
                    dot_product = std::max(-1.0, std::min(1.0, dot_product));
                    double angle = std::acos(dot_product);

                    // Return the per-pixel vertical subtent. For example, a
                    // 33.75° FOV over rows 10..118 gives 33.75° / 108.
                    return angle / static_cast<double>(top - bottom);
                }
                // Skip invalid points by moving inward
                top -= top_valid ? 0 : 1;
                bottom += bottom_valid ? 0 : 1;
            }
        }
    }
    // Fallback if no valid pair found: assume 90 deg vertical FOV spread across
    // the image height.
    const size_t intervals = height > 1 ? height - 1 : 1;
    return (0.5 * M_PI) / static_cast<double>(intervals);
}

// Templated on the presence of a second return so the per-neighbor checks
// compile to two specialized versions without per-pixel branching.
template <bool HasSecondReturn>
void compute_unit_normals(const double* xyz, const uint32_t* range, const double* xyz2,
                          const uint32_t* range2, size_t height, size_t width,
                          const Eigen::Ref<const MatrixX3dR> sensor_origins, double* normals_data,
                          size_t pixel_search_range, double min_angle_of_incidence_rad,
                          double target_distance_m, double vertical_pixel_subtent_override = 0.0) {
    // Helpers for computing flat indices into the range (2D) and xyz (3D)
    auto flat_index = [&](size_t row, size_t col) -> size_t {
        // Row-major index into the range image.
        return row * width + col;
    };
    auto flat_index_3d = [&](size_t row, size_t col) -> size_t {
        // Row-major index into the XYZ array (3 doubles per pixel).
        return flat_index(row, col) * 3;
    };

    const double horizontal_pixel_subtent_rad = 2.0 * M_PI / static_cast<double>(width);
    const double desired_neighbor_distance_sq = target_distance_m * target_distance_m;
    const double safe_incidence_rad = std::max(min_angle_of_incidence_rad, 1e-6);

    // Normalize beams on demand while estimating vertical pixel subtent.
    // compute_vertical_subtent() normalizes only its selected top/bottom pair;
    // the main loop separately reuses center_point and normalizes a center beam
    // only when needed. A full H*W buffer would add unused work and memory
    // traffic.
    auto normalized_beam = [&](size_t row, size_t col) -> Eigen::Vector3d {
        const double* point_xyz = xyz + flat_index_3d(row, col);
        Eigen::Vector3d direction(point_xyz[0], point_xyz[1], point_xyz[2]);
        const Eigen::Index col_idx = static_cast<Eigen::Index>(col);
        direction.x() -= sensor_origins(col_idx, 0);
        direction.y() -= sensor_origins(col_idx, 1);
        direction.z() -= sensor_origins(col_idx, 2);
        return direction.normalized();
    };

    const double vertical_pixel_subtent_rad =
        vertical_pixel_subtent_override > 0.0
            ? vertical_pixel_subtent_override
            : compute_vertical_subtent(
                  width, height,
                  [&](size_t row, size_t col) { return range[flat_index(row, col)] != 0; },
                  normalized_beam);

    // Maximum allowable range delta for a given incidence angle and pixel
    // subtent; acts as an AOI-based gating threshold. It simplifies to
    // range_mm * 0.001 * pixel_subtent / tan(incidence), so precompute the two
    // per-axis scales and leave one multiply in the per-pixel hot path.
    const double tan_safe_incidence = std::tan(safe_incidence_rad);
    const double horizontal_distance_per_range_mm =
        0.001 * horizontal_pixel_subtent_rad / tan_safe_incidence;
    const double vertical_distance_per_range_mm =
        0.001 * vertical_pixel_subtent_rad / tan_safe_incidence;

    enum class NeighborAxis { VERTICAL, HORIZONTAL };

    // Scan outward by pixel radius and mark a neighbor "good" once it passes
    // the angle-of-incidence threshold at the radius where it was found.
    auto find_best_neighbor = [&](NeighborAxis axis, size_t row_index, size_t col_index,
                                  double neighbor_distance_sq, const Eigen::Vector3d& center_point,
                                  uint32_t center_range_mm, Eigen::Vector3d& diff,
                                  bool& requires_flip, bool& thin_foreground_flag,
                                  size_t max_up_radius, size_t max_down_radius) -> bool {
        Eigen::Vector3d best_diff = Eigen::Vector3d::Zero();
        double min_distance_sq = std::numeric_limits<double>::infinity();
        double best_error = std::numeric_limits<double>::infinity();
        size_t best_radius = 1;
        bool best_requires_flip = false;
        bool good_neighbor = false;

        // Record the best neighbor candidate (closest to target distance) if it
        // passes validity checks for range/non-foreground; updates shared
        // state.
        auto consider_neighbor = [&](size_t row, size_t col, const double* xyz_base,
                                     const uint32_t* rng_base, bool flip_flag,
                                     size_t current_radius) {
            const size_t neighbor_idx = flat_index(row, col);
            uint32_t neighbor_range = rng_base[neighbor_idx];
            if (neighbor_range == 0) {
                return;
            }

            Eigen::Map<const Eigen::Vector3d> neighbor_vec(xyz_base + flat_index_3d(row, col));
            Eigen::Vector3d diff_vec = neighbor_vec - center_point;
            double distance_sq = diff_vec.squaredNorm();

            // if all neighbors are more than threshold, then it's a thin pole
            if (static_cast<int64_t>(neighbor_range) - static_cast<int64_t>(center_range_mm) <
                static_cast<int64_t>(FOREGROUND_SALIENCE_MM)) {
                thin_foreground_flag = false;
            }

            double candidate_error = std::abs(distance_sq - desired_neighbor_distance_sq);
            if (candidate_error < best_error) {
                best_diff = diff_vec;
                min_distance_sq = distance_sq;
                best_error = candidate_error;
                best_requires_flip = flip_flag;
                best_radius = current_radius;
            }
        };

        const size_t final_search_radius = axis == NeighborAxis::VERTICAL
                                               ? std::max(max_up_radius, max_down_radius)
                                               : std::min(pixel_search_range, width / 2);
        for (size_t radius = 1; radius <= final_search_radius; ++radius) {
            if (good_neighbor && !thin_foreground_flag) {
                break;
            }

            if (axis == NeighborAxis::VERTICAL) {
                if (radius <= max_up_radius) {
                    size_t row_up = row_index - radius;
                    consider_neighbor(row_up, col_index, xyz, range, true, radius);
                }
                if (radius <= max_down_radius) {
                    size_t row_down = row_index + radius;
                    consider_neighbor(row_down, col_index, xyz, range, false, radius);
                }
                if (HasSecondReturn) {
                    if (radius <= max_up_radius) {
                        size_t row_up = row_index - radius;
                        consider_neighbor(row_up, col_index, xyz2, range2, true, radius);
                    }
                    if (radius <= max_down_radius) {
                        size_t row_down = row_index + radius;
                        consider_neighbor(row_down, col_index, xyz2, range2, false, radius);
                    }
                }
            } else {
                // Horizontal sweep: wrap around the 360° image. Beyond
                // width / 2 the columns repeat, so cap the search there and
                // visit each distinct non-center column at most once.
                const size_t col_left =
                    col_index >= radius ? col_index - radius : col_index + width - radius;
                const size_t col_right_unwrapped = col_index + radius;
                const size_t col_right =
                    col_right_unwrapped < width ? col_right_unwrapped : col_right_unwrapped - width;
                consider_neighbor(row_index, col_left, xyz, range, true, radius);
                if (HasSecondReturn) {
                    consider_neighbor(row_index, col_left, xyz2, range2, true, radius);
                }

                // For even widths, both directions reach the same opposite
                // column at radius == width / 2.
                if (col_right != col_left) {
                    consider_neighbor(row_index, col_right, xyz, range, false, radius);
                    if (HasSecondReturn) {
                        consider_neighbor(row_index, col_right, xyz2, range2, false, radius);
                    }
                }
            }

            if (desired_neighbor_distance_sq <= min_distance_sq &&
                min_distance_sq < (static_cast<double>(best_radius) *
                                   static_cast<double>(best_radius) * neighbor_distance_sq)) {
                good_neighbor = true;
            } else if (radius == final_search_radius) {
                // If we ran out of search radius, accept the closest candidate
                // that stayed within the distance threshold.
                if (min_distance_sq > 0 &&
                    min_distance_sq < (static_cast<double>(best_radius) *
                                       static_cast<double>(best_radius) * neighbor_distance_sq)) {
                    good_neighbor = true;
                }
            }
        }

        if (good_neighbor) {
            diff = best_diff;
            requires_flip = best_requires_flip;
            return true;
        }
        return false;
    };

    for (size_t u = 0; u < height; ++u) {
        const size_t max_up_radius = std::min(pixel_search_range, u);
        const size_t max_down_radius = std::min(pixel_search_range, height - 1 - u);
        for (size_t v = 0; v < width; ++v) {
            uint32_t center_range = range[flat_index(u, v)];
            double* normal_slot = normals_data + flat_index_3d(u, v);
            // Zero up front so every skip path below is safe by construction
            // (the output matrix itself is not zero-initialized); compute
            // paths overwrite the slot.
            normal_slot[0] = normal_slot[1] = normal_slot[2] = 0.0;
            if (center_range == 0) {
                continue;
            }

            const double* center_xyz = xyz + flat_index_3d(u, v);
            Eigen::Vector3d center_point(center_xyz[0], center_xyz[1], center_xyz[2]);
            Eigen::Vector3d beam_vec = center_point;
            const Eigen::Index col_idx = static_cast<Eigen::Index>(v);
            beam_vec -= sensor_origins.row(col_idx).transpose();
            const double beam_magnitude_sq = beam_vec.squaredNorm();
            if (beam_magnitude_sq == 0.0) {
                continue;
            }

            const double neighbor_distance_horizontal =
                static_cast<double>(center_range) * horizontal_distance_per_range_mm;
            const double neighbor_distance_horizontal_sq =
                neighbor_distance_horizontal * neighbor_distance_horizontal;

            const double neighbor_distance_vertical =
                static_cast<double>(center_range) * vertical_distance_per_range_mm;
            const double neighbor_distance_vertical_sq =
                neighbor_distance_vertical * neighbor_distance_vertical;

            Eigen::Vector3d vertical_diff = Eigen::Vector3d::Zero();
            Eigen::Vector3d horizontal_diff = Eigen::Vector3d::Zero();
            // We need use one neighbors for up/down, right/left so we
            // may need to flip the sign before the cross product
            bool vertical_requires_flip = false;    // true if picked vertical neighbor lies above
            bool horizontal_requires_flip = false;  // true if picked horizontal neighbor
                                                    // lies to the left
            bool vertical_thin_foreground_object = true;
            bool horizontal_thin_foreground_object = true;

            const bool vertical_found = find_best_neighbor(
                NeighborAxis::VERTICAL, u, v, neighbor_distance_vertical_sq, center_point,
                center_range, vertical_diff, vertical_requires_flip,
                vertical_thin_foreground_object, max_up_radius, max_down_radius);
            const bool horizontal_found = find_best_neighbor(
                NeighborAxis::HORIZONTAL, u, v, neighbor_distance_horizontal_sq, center_point,
                center_range, horizontal_diff, horizontal_requires_flip,
                horizontal_thin_foreground_object, pixel_search_range, pixel_search_range);

            const bool thin_both =
                vertical_thin_foreground_object && horizontal_thin_foreground_object;

            // Case A: if a pixel has range but misses both neighbors,
            // or both were classified thin. Use the beam vector as the
            // normal vector.
            if ((!vertical_found && !horizontal_found) || thin_both) {
                Eigen::Vector3d beam = beam_vec * (-1.0 / std::sqrt(beam_magnitude_sq));
                normal_slot[0] = beam.x();
                normal_slot[1] = beam.y();
                normal_slot[2] = beam.z();
                continue;
            }

            // Case B: if a pixel only has one neighbor OR the opposite
            // neighbor was marked thin, use that good neighbor (vector
            // A) and decompose the beam (sensor to pixel vector) into
            // components perpendicular/parallel to A. Use the
            // perpendicular component as the normal vector.
            const bool vertical_only =
                vertical_found && (!horizontal_found || horizontal_thin_foreground_object);
            const bool horizontal_only =
                horizontal_found && (!vertical_found || vertical_thin_foreground_object);
            if (vertical_only || horizontal_only) {
                const Eigen::Vector3d& single_diff =
                    vertical_only ? vertical_diff : horizontal_diff;
                const Eigen::Vector3d beam_dir = beam_vec * (1.0 / std::sqrt(beam_magnitude_sq));
                double denom = single_diff.squaredNorm();
                if (denom < std::numeric_limits<double>::epsilon()) {
                    continue;
                }
                double dot = single_diff.dot(beam_dir);
                Eigen::Vector3d projected = beam_dir - (dot / denom) * single_diff;
                double n_sq = projected.squaredNorm();
                if (n_sq < std::numeric_limits<double>::epsilon()) {
                    continue;
                }
                projected *= -1.0 / std::sqrt(n_sq);
                normal_slot[0] = projected.x();
                normal_slot[1] = projected.y();
                normal_slot[2] = projected.z();
                continue;
            }

            // Case C: if a pixel has one of both up/down, left/right
            // valid neighbors
            if (horizontal_requires_flip != vertical_requires_flip) {
                vertical_diff = -vertical_diff;
            }

            Eigen::Vector3d normal = vertical_diff.cross(horizontal_diff);
            double norm_magnitude = normal.norm();
            if (norm_magnitude != 0.0) {
                normal *= 1.0 / norm_magnitude;
                normal_slot[0] = normal.x();
                normal_slot[1] = normal.y();
                normal_slot[2] = normal.z();
            }
        }
    }
}

}  // namespace

MatrixX3dR normals(const Eigen::Ref<const PointCloudXYZd> xyz,
                   const Eigen::Ref<const img_t<uint32_t>> range,
                   const Eigen::Ref<const MatrixX3dR> sensor_origins_xyz, size_t pixel_search_range,
                   double min_angle_of_incidence_rad, double target_distance_m) {
    const size_t h = static_cast<size_t>(range.rows());
    const size_t w = static_cast<size_t>(range.cols());

    if (static_cast<size_t>(xyz.rows()) != h * w) {
        throw std::runtime_error("normals: xyz dimensions mismatch");
    }
    if (sensor_origins_xyz.rows() != static_cast<Eigen::Index>(w)) {
        throw std::runtime_error("normals: sensor_origins size must match image width");
    }
    validate_normal_parameters(min_angle_of_incidence_rad, target_distance_m);

    const Eigen::Index num_pixels = static_cast<Eigen::Index>(h * w);
    // No zero-initialization needed: compute_unit_normals writes every slot.
    MatrixX3dR normals(num_pixels, 3);
    compute_unit_normals<false>(xyz.data(), range.data(), nullptr, nullptr, h, w,
                                sensor_origins_xyz, normals.data(), pixel_search_range,
                                min_angle_of_incidence_rad, target_distance_m);

    return normals;
}

std::pair<MatrixX3dR, MatrixX3dR> normals(
    const Eigen::Ref<const PointCloudXYZd> xyz, const Eigen::Ref<const img_t<uint32_t>> range,
    const Eigen::Ref<const PointCloudXYZd> xyz2, const Eigen::Ref<const img_t<uint32_t>> range2,
    const Eigen::Ref<const MatrixX3dR> sensor_origins_xyz, size_t pixel_search_range,
    double min_angle_of_incidence_rad, double target_distance_m) {
    const size_t h = static_cast<size_t>(range.rows());
    const size_t w = static_cast<size_t>(range.cols());

    if (static_cast<size_t>(xyz.rows()) != h * w || static_cast<size_t>(xyz2.rows()) != h * w) {
        throw std::runtime_error("normals: xyz dimensions mismatch");
    }
    if (static_cast<size_t>(range2.rows()) != h || static_cast<size_t>(range2.cols()) != w) {
        throw std::runtime_error("normals: range2 dimensions mismatch");
    }
    if (sensor_origins_xyz.rows() != static_cast<Eigen::Index>(w)) {
        throw std::runtime_error("normals: sensor_origins size must match image width");
    }
    validate_normal_parameters(min_angle_of_incidence_rad, target_distance_m);

    // Compute vertical pixel subtent once from the first return. Reuse it for
    // both returns so thresholds stay consistent.
    auto has_range = [&](size_t row, size_t col) {
        return range(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) != 0;
    };
    double vertical_pixel_subtent_rad =
        compute_vertical_subtent(w, h, has_range, [&](size_t row, size_t col) {
            const Eigen::Index idx = static_cast<Eigen::Index>(row * w + col);

            Eigen::Vector3d beam_vec(xyz(idx, 0), xyz(idx, 1), xyz(idx, 2));

            // subtract per-column sensor origin before normalization
            beam_vec -= sensor_origins_xyz.row(static_cast<Eigen::Index>(col)).transpose();

            double magnitude = beam_vec.norm();
            if (magnitude > 0.0) {
                beam_vec *= 1.0 / magnitude;
            } else {
                beam_vec.setZero();
            }
            return beam_vec;
        });

    const Eigen::Index num_pixels = static_cast<Eigen::Index>(h * w);
    // No zero-initialization needed: compute_unit_normals writes every slot.
    MatrixX3dR first(num_pixels, 3);
    compute_unit_normals<true>(xyz.data(), range.data(), xyz2.data(), range2.data(), h, w,
                               sensor_origins_xyz, first.data(), pixel_search_range,
                               min_angle_of_incidence_rad, target_distance_m,
                               vertical_pixel_subtent_rad);

    MatrixX3dR second(num_pixels, 3);
    compute_unit_normals<true>(xyz2.data(), range2.data(), xyz.data(), range.data(), h, w,
                               sensor_origins_xyz, second.data(), pixel_search_range,
                               min_angle_of_incidence_rad, target_distance_m,
                               vertical_pixel_subtent_rad);

    return {std::move(first), std::move(second)};
}

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster

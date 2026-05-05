#include "ouster/normals.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

namespace {

// Range delta threshold to flag 1-pixel-thin foreground objects during neighbor
// checks (used in check_neighbor_to_center).
constexpr uint32_t FOREGROUND_SALIENCE_MM = 500;

// From mid_column outward, find the first column with valid range data at
// both top and bottom, compute the angle between their normalized beam
// vectors, and return the angle divided by the pixel distance between them.
// This gives an estimate of the vertical pixel subtent in radians. If no
// valid pairs are found, fall back to assuming a 90 degree vertical FOV
template <typename RangeValidFn, typename BeamFn>
double compute_vertical_subtent(size_t width, size_t height,
                                const RangeValidFn& has_range,
                                const BeamFn& normalized_beam) {
    const size_t mid_col = width / 2;
    // Search from center column outward (left and right alternately)
    for (size_t col_offset = 0; col_offset <= mid_col; ++col_offset) {
        for (int sign : {-1, 1}) {
            const int col_i =
                static_cast<int>(mid_col) + sign * static_cast<int>(col_offset);

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
                if (has_range(top, col) && has_range(bottom, col)) {
                    const auto& v_top = normalized_beam(top, col);
                    const auto& v_bottom = normalized_beam(bottom, col);

                    double dot_product = v_top.dot(v_bottom);
                    dot_product = std::max(-1.0, std::min(1.0, dot_product));
                    double angle = std::acos(dot_product);

                    if (top != bottom) {
                        // return per pixel vertical subtent (angle)
                        // For example, vertical FOV is 33.75° over 108 pixels
                        // 33.75° / (118 - 10) = 0.3125° per pixel vertically
                        return angle / static_cast<double>(top - bottom);
                    }
                }
                // Skip invalid points by moving inward
                top -= (has_range(top, col) ? 0 : 1);
                bottom += (has_range(bottom, col) ? 0 : 1);
            }
        }
    }
    // Fallback if no valid pair found: assume 90 deg vertical FOV spread across
    // the image height.
    const size_t intervals = std::max<size_t>(1, height - 1);
    return (0.5 * M_PI) / static_cast<double>(intervals);
}

void compute_unit_normals(const double* xyz, const uint32_t* range,
                          const double* xyz2, const uint32_t* range2,
                          size_t height, size_t width,
                          const Eigen::Ref<const MatrixX3dR> sensor_origins,
                          double* normals_data, size_t pixel_search_range,
                          double min_angle_of_incidence_rad,
                          double target_distance_m,
                          double vertical_pixel_subtent_override = 0.0) {
    if (target_distance_m <= 0.0) {
        throw std::runtime_error("normals: target_distance_m must be positive");
    }
    if (min_angle_of_incidence_rad <= 0.0) {
        throw std::runtime_error(
            "normals: min_angle_of_incidence_rad must be positive");
    }
    // Helpers for computing flat indices into the range (2D) and xyz (3D)
    auto flat_index = [&](size_t row, size_t col) -> size_t {
        // Row-major index into the range image.
        return row * width + col;
    };
    auto flat_index_3d = [&](size_t row, size_t col) -> size_t {
        // Row-major index into the XYZ array (3 doubles per pixel).
        return flat_index(row, col) * 3;
    };

    // Tile size that helps keep range/XYZ data hot in cache.
    constexpr size_t block_size = 32;

    const double horizontal_pixel_subtent_rad =
        2.0 * M_PI / static_cast<double>(width);
    const double desired_neighbor_distance_sq =
        target_distance_m * target_distance_m;
    const double safe_incidence_rad =
        std::max(min_angle_of_incidence_rad, 1e-6);

    // Precompute normalized beam vectors for each pixel (destaggered xyz minus
    // per-column sensor origin, normalized) to avoid recomputing inside the
    // neighbor loops.
    std::vector<Eigen::Vector3d> beams(height * width, Eigen::Vector3d::Zero());
    for (size_t row = 0; row < height; ++row) {
        for (size_t col = 0; col < width; ++col) {
            const double* point_xyz = xyz + flat_index_3d(row, col);
            Eigen::Vector3d direction(point_xyz[0], point_xyz[1], point_xyz[2]);
            if (sensor_origins.rows() > 0 &&
                col < static_cast<size_t>(sensor_origins.rows())) {
                const Eigen::Index col_idx = static_cast<Eigen::Index>(col);
                direction.x() -= sensor_origins(col_idx, 0);
                direction.y() -= sensor_origins(col_idx, 1);
                direction.z() -= sensor_origins(col_idx, 2);
            }
            const double magnitude = direction.norm();
            if (magnitude > 0.0) {
                beams[flat_index(row, col)] = direction / magnitude;
            } else {
                beams[flat_index(row, col)] = Eigen::Vector3d::Zero();
            }
        }
    }

    auto normalized_beam = [&](size_t row,
                               size_t col) -> const Eigen::Vector3d& {
        return beams[flat_index(row, col)];
    };

    const double vertical_pixel_subtent_rad =
        vertical_pixel_subtent_override > 0.0
            ? vertical_pixel_subtent_override
            : compute_vertical_subtent(
                  width, height,
                  [&](size_t row, size_t col) {
                      return range[flat_index(row, col)] != 0;
                  },
                  normalized_beam);
    const bool has_second_return = (xyz2 != nullptr && range2 != nullptr);

    // Maximum allowable range delta for a given incidence angle and pixel
    // subtent; acts as an AOI-based gating threshold.
    auto calc_max_distance_threshold =
        [safe_incidence_rad](uint32_t range_mm, double pixel_subtent_rad) {
            const double perimeter_m =
                2.0 * M_PI * (static_cast<double>(range_mm) * 0.001);
            const double px_res = (2.0 * M_PI) / pixel_subtent_rad;
            const double min_px_spacing_m = perimeter_m / px_res;
            const double aoi_range_threshold_m =
                min_px_spacing_m / std::tan(safe_incidence_rad);
            return aoi_range_threshold_m;
        };

    enum class NeighborAxis { VERTICAL, HORIZONTAL };

    // Scan outward by pixel radius and mark a neighbor "good" once it passes
    // the angle-of-incidence threshold at the radius where it was found.
    auto find_best_neighbor =
        [&](NeighborAxis axis, size_t row_index, size_t col_index,
            double neighbor_distance_sq, const Eigen::Vector3d& center_point,
            uint32_t center_range_mm, Eigen::Vector3d& diff,
            bool& requires_flip, bool& thin_foreground_flag,
            size_t max_up_radius, size_t max_down_radius) -> bool {
        Eigen::Vector3d best_diff = Eigen::Vector3d::Zero();
        double min_distance_sq = std::numeric_limits<double>::infinity();
        size_t best_radius = 1;
        bool best_requires_flip = false;
        bool good_neighbor = false;

        // Record the best neighbor candidate (closest to target distance) if it
        // passes validity checks for range/non-foreground; updates shared
        // state.
        auto consider_neighbor = [&](size_t row, size_t col,
                                     const double* xyz_base,
                                     const uint32_t* rng_base, bool flip_flag,
                                     size_t current_radius) {
            const size_t neighbor_idx = flat_index(row, col);
            uint32_t neighbor_range = rng_base[neighbor_idx];
            if (neighbor_range == 0) {
                return;
            }

            Eigen::Map<const Eigen::Vector3d> neighbor_vec(
                xyz_base + flat_index_3d(row, col));
            Eigen::Vector3d diff_vec = neighbor_vec - center_point;
            double distance_sq = diff_vec.squaredNorm();

            // if all neighors are more than threshold, then it's a thin pole
            if (static_cast<int64_t>(neighbor_range) -
                    static_cast<int64_t>(center_range_mm) <
                static_cast<int64_t>(FOREGROUND_SALIENCE_MM)) {
                thin_foreground_flag = false;
            }

            double candidate_error =
                std::abs(distance_sq - desired_neighbor_distance_sq);
            if (candidate_error <
                std::abs(min_distance_sq - desired_neighbor_distance_sq)) {
                best_diff = diff_vec;
                min_distance_sq = distance_sq;
                best_requires_flip = flip_flag;
                best_radius = current_radius;
            }
        };

        for (size_t radius = 1; radius <= pixel_search_range; ++radius) {
            if (axis == NeighborAxis::VERTICAL && radius > max_up_radius &&
                radius > max_down_radius) {
                break;
            }
            if (good_neighbor && !thin_foreground_flag) {
                break;
            }

            if (axis == NeighborAxis::VERTICAL) {
                if (radius <= max_up_radius) {
                    size_t row_up = row_index - radius;
                    consider_neighbor(row_up, col_index, xyz, range, true,
                                      radius);
                }
                if (radius <= max_down_radius) {
                    size_t row_down = row_index + radius;
                    consider_neighbor(row_down, col_index, xyz, range, false,
                                      radius);
                }
                if (has_second_return && range2 != nullptr) {
                    if (radius <= max_up_radius) {
                        size_t row_up = row_index - radius;
                        consider_neighbor(row_up, col_index, xyz2, range2, true,
                                          radius);
                    }
                    if (radius <= max_down_radius) {
                        size_t row_down = row_index + radius;
                        consider_neighbor(row_down, col_index, xyz2, range2,
                                          false, radius);
                    }
                }
            } else {
                // Horizontal sweep: wrap around for 360° images
                int col_left_unwrapped =
                    static_cast<int>(col_index) - static_cast<int>(radius);
                size_t col_left = static_cast<size_t>(
                    ((col_left_unwrapped % static_cast<int>(width)) +
                     static_cast<int>(width)) %
                    static_cast<int>(width));
                consider_neighbor(row_index, col_left, xyz, range, true,
                                  radius);
                if (has_second_return && range2 != nullptr) {
                    consider_neighbor(row_index, col_left, xyz2, range2, true,
                                      radius);
                }

                int col_right_unwrapped =
                    static_cast<int>(col_index) + static_cast<int>(radius);
                size_t col_right = static_cast<size_t>(col_right_unwrapped %
                                                       static_cast<int>(width));
                consider_neighbor(row_index, col_right, xyz, range, false,
                                  radius);
                if (has_second_return && range2 != nullptr) {
                    consider_neighbor(row_index, col_right, xyz2, range2, false,
                                      radius);
                }
            }

            if (desired_neighbor_distance_sq <= min_distance_sq &&
                min_distance_sq <
                    (static_cast<double>(best_radius) *
                     static_cast<double>(best_radius) * neighbor_distance_sq)) {
                good_neighbor = true;
            } else if (radius == pixel_search_range) {
                // If we ran out of search radius, accept the closest candidate
                // that stayed within the distance threshold.
                if (min_distance_sq > 0 &&
                    min_distance_sq < (static_cast<double>(best_radius) *
                                       static_cast<double>(best_radius) *
                                       neighbor_distance_sq)) {
                    good_neighbor = true;
                }
            }
        }

        if (good_neighbor &&
            min_distance_sq < std::numeric_limits<double>::infinity()) {
            diff = best_diff;
            requires_flip = best_requires_flip;
            return true;
        }
        return false;
    };

    for (size_t block_u = 0; block_u < height; block_u += block_size) {
        const size_t end_u = std::min(block_u + block_size, height);

        for (size_t block_v = 0; block_v < width; block_v += block_size) {
            const size_t end_v = std::min(block_v + block_size, width);

            for (size_t u = block_u; u < end_u; ++u) {
                const size_t max_up_radius = std::min(pixel_search_range, u);
                const size_t max_down_radius =
                    std::min(pixel_search_range, height - 1 - u);
                for (size_t v = block_v; v < end_v; ++v) {
                    uint32_t center_range = range[flat_index(u, v)];
                    double* normal_slot = normals_data + flat_index_3d(u, v);
                    normal_slot[0] = normal_slot[1] = normal_slot[2] = 0.0;
                    if (center_range == 0) {
                        continue;
                    }

                    const double* center_xyz = xyz + flat_index_3d(u, v);
                    Eigen::Vector3d center_point(center_xyz[0], center_xyz[1],
                                                 center_xyz[2]);
                    const Eigen::Vector3d& beam_dir = normalized_beam(u, v);
                    if (beam_dir.squaredNorm() <=
                        std::numeric_limits<double>::epsilon()) {
                        continue;
                    }

                    const double neighbor_distance_horizontal =
                        calc_max_distance_threshold(
                            center_range, horizontal_pixel_subtent_rad);
                    const double neighbor_distance_horizontal_sq =
                        neighbor_distance_horizontal *
                        neighbor_distance_horizontal;

                    const double neighbor_distance_vertical =
                        calc_max_distance_threshold(center_range,
                                                    vertical_pixel_subtent_rad);
                    const double neighbor_distance_vertical_sq =
                        neighbor_distance_vertical * neighbor_distance_vertical;

                    Eigen::Vector3d vertical_diff = Eigen::Vector3d::Zero();
                    Eigen::Vector3d horizontal_diff = Eigen::Vector3d::Zero();
                    // We need use one neighbors for up/down, right/left so we
                    // may need to flip the sign before the cross product
                    bool vertical_requires_flip =
                        false;  // true if picked vertical neighbor lies above
                    bool horizontal_requires_flip =
                        false;  // true if picked horizontal neighbor lies to
                                // the left
                    bool vertical_thin_foreground_object = true;
                    bool horizontal_thin_foreground_object = true;

                    const bool vertical_found = find_best_neighbor(
                        NeighborAxis::VERTICAL, u, v,
                        neighbor_distance_vertical_sq, center_point,
                        center_range, vertical_diff, vertical_requires_flip,
                        vertical_thin_foreground_object, max_up_radius,
                        max_down_radius);
                    const bool horizontal_found = find_best_neighbor(
                        NeighborAxis::HORIZONTAL, u, v,
                        neighbor_distance_horizontal_sq, center_point,
                        center_range, horizontal_diff, horizontal_requires_flip,
                        horizontal_thin_foreground_object, pixel_search_range,
                        pixel_search_range);

                    const bool thin_both = vertical_thin_foreground_object &&
                                           horizontal_thin_foreground_object;

                    // Case A: if a pixel has range but misses both neighbors,
                    // or both were classified thin. Use the beam vector as the
                    // normal vector.
                    if ((!vertical_found && !horizontal_found) || thin_both) {
                        Eigen::Vector3d beam = -beam_dir;
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
                    if (vertical_found && (!horizontal_found ||
                                           horizontal_thin_foreground_object)) {
                        double denom = vertical_diff.squaredNorm();
                        if (std::abs(denom) <
                            std::numeric_limits<double>::epsilon()) {
                            continue;
                        }
                        double dot = vertical_diff.dot(beam_dir);
                        Eigen::Vector3d projected =
                            beam_dir - (dot / denom) * vertical_diff;
                        double n_sq = projected.squaredNorm();
                        if (std::abs(n_sq) <
                            std::numeric_limits<double>::epsilon()) {
                            continue;
                        }
                        projected /= std::sqrt(n_sq);
                        projected = -projected;
                        normal_slot[0] = projected.x();
                        normal_slot[1] = projected.y();
                        normal_slot[2] = projected.z();
                        continue;
                    } else if (horizontal_found &&
                               (!vertical_found ||
                                vertical_thin_foreground_object)) {
                        double denom = horizontal_diff.squaredNorm();
                        if (std::abs(denom) <
                            std::numeric_limits<double>::epsilon()) {
                            continue;
                        }

                        double dot = horizontal_diff.dot(beam_dir);
                        Eigen::Vector3d projected =
                            beam_dir - (dot / denom) * horizontal_diff;
                        double n_sq = projected.squaredNorm();
                        if (std::abs(n_sq) <
                            std::numeric_limits<double>::epsilon()) {
                            continue;
                        }
                        projected /= std::sqrt(n_sq);
                        projected = -projected;
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

                    Eigen::Vector3d normal =
                        vertical_diff.cross(horizontal_diff);
                    double norm_magnitude = normal.norm();
                    if (norm_magnitude != 0.0) {
                        normal /= norm_magnitude;
                        normal_slot[0] = normal.x();
                        normal_slot[1] = normal.y();
                        normal_slot[2] = normal.z();
                    }
                }
            }
        }
    }
}

}  // namespace

MatrixX3dR normals(const Eigen::Ref<const PointCloudXYZd> xyz,
                   const Eigen::Ref<const img_t<uint32_t>> range,
                   const Eigen::Ref<const MatrixX3dR> sensor_origins_xyz,
                   size_t pixel_search_range, double min_angle_of_incidence_rad,
                   double target_distance_m) {
    const size_t h = static_cast<size_t>(range.rows());
    const size_t w = static_cast<size_t>(range.cols());

    if (static_cast<size_t>(xyz.rows()) != h * w || xyz.cols() != 3) {
        throw std::runtime_error("normals: xyz dimensions mismatch");
    }
    if (sensor_origins_xyz.rows() != static_cast<Eigen::Index>(w)) {
        throw std::runtime_error(
            "normals: sensor_origins size must match image width");
    }

    const Eigen::Index num_pixels = static_cast<Eigen::Index>(h * w);
    MatrixX3dR normals = MatrixX3dR::Zero(num_pixels, 3);
    compute_unit_normals(xyz.data(), range.data(), nullptr, nullptr, h, w,
                         sensor_origins_xyz, normals.data(), pixel_search_range,
                         min_angle_of_incidence_rad, target_distance_m);

    return normals;
}

std::pair<MatrixX3dR, MatrixX3dR> normals(
    const Eigen::Ref<const PointCloudXYZd> xyz,
    const Eigen::Ref<const img_t<uint32_t>> range,
    const Eigen::Ref<const PointCloudXYZd> xyz2,
    const Eigen::Ref<const img_t<uint32_t>> range2,
    const Eigen::Ref<const MatrixX3dR> sensor_origins_xyz,
    size_t pixel_search_range, double min_angle_of_incidence_rad,
    double target_distance_m) {
    const size_t h = static_cast<size_t>(range.rows());
    const size_t w = static_cast<size_t>(range.cols());

    if (static_cast<size_t>(xyz.rows()) != h * w || xyz.cols() != 3 ||
        static_cast<size_t>(xyz2.rows()) != h * w || xyz2.cols() != 3) {
        throw std::runtime_error("normals: xyz dimensions mismatch");
    }
    if (static_cast<size_t>(range2.rows()) != h ||
        static_cast<size_t>(range2.cols()) != w) {
        throw std::runtime_error("normals: range2 dimensions mismatch");
    }
    if (sensor_origins_xyz.rows() != static_cast<Eigen::Index>(w)) {
        throw std::runtime_error(
            "normals: sensor_origins size must match image width");
    }

    // Compute vertical pixel subtent once from the first return. Reuse it for
    // both returns so thresholds stay consistent.
    auto has_range = [&](size_t row, size_t col) {
        return range(static_cast<Eigen::Index>(row),
                     static_cast<Eigen::Index>(col)) != 0;
    };
    double vertical_pixel_subtent_rad =
        compute_vertical_subtent(w, h, has_range, [&](size_t row, size_t col) {
            const Eigen::Index idx = static_cast<Eigen::Index>(row * w + col);

            Eigen::Vector3d beam_vec(xyz(idx, 0), xyz(idx, 1), xyz(idx, 2));

            // subtract per-column sensor origin before normalization
            beam_vec -= sensor_origins_xyz.row(static_cast<Eigen::Index>(col))
                            .transpose();

            double magnitude = beam_vec.norm();
            if (magnitude > 0.0) {
                beam_vec /= static_cast<double>(magnitude);
            } else {
                beam_vec.setZero();
            }
            return beam_vec;
        });

    const Eigen::Index num_pixels = static_cast<Eigen::Index>(h * w);
    MatrixX3dR first = MatrixX3dR::Zero(num_pixels, 3);
    compute_unit_normals(xyz.data(), range.data(), xyz2.data(), range2.data(),
                         h, w, sensor_origins_xyz, first.data(),
                         pixel_search_range, min_angle_of_incidence_rad,
                         target_distance_m, vertical_pixel_subtent_rad);

    MatrixX3dR second = MatrixX3dR::Zero(num_pixels, 3);
    compute_unit_normals(xyz2.data(), range2.data(), xyz.data(), range.data(),
                         h, w, sensor_origins_xyz, second.data(),
                         pixel_search_range, min_angle_of_incidence_rad,
                         target_distance_m, vertical_pixel_subtent_rad);

    return {first, second};
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

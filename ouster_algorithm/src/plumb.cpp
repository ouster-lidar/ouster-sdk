#include "ouster/algorithm/plumb.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/algorithm/impl/spatial_hash.h"
#include "ouster/algorithm/impl/wall.h"
#include "ouster/core/chanfield.h"
#include "ouster/core/field.h"
#include "ouster/core/frame.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/impl/dewarp_impl.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/pose_util.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"

namespace ChanField = ouster::sdk::core::ChanField;
using ouster::sdk::algorithm::impl::cell_key;
using ouster::sdk::core::DEG_TO_RAD;
using ouster::sdk::core::logger;
using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::PointCloudXYZd;
using ouster::sdk::core::impl::PoseH;

namespace ouster {
namespace sdk {
namespace algorithm {

namespace {

// Coordinate descent schedule: (step, half range) in degrees, coarse then
// fine. The coarse pass bounds how much residual tilt is correctable.
constexpr double COARSE_STEP_DEG = 1.0;
constexpr double COARSE_RANGE_DEG = 25.0;
constexpr double FINE_STEP_DEG = 0.1;
constexpr double FINE_RANGE_DEG = 2.0;

Eigen::Matrix3d rotation_from_rotvec(const Eigen::Vector3d& rotvec) {
    const double theta = rotvec.norm();
    if (theta < 1e-12) {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(theta, rotvec / theta).toRotationMatrix();
}

// Evaluate independent rotation candidates on the TBB pool the rest of the
// SDK shares. Each candidate projects and sorts every wall point to count
// occupied cells; `keys` is scratch reused across the candidates one task runs.
template <typename Body>
void for_each_candidate(size_t candidate_count, Body&& body) {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, candidate_count),
                      [&](const tbb::blocked_range<size_t>& range) {
                          std::vector<int64_t> keys;
                          for (size_t index = range.begin(); index != range.end(); ++index) {
                              body(index, keys);
                          }
                      });
}

Eigen::Matrix3d rotation_xy(double x_deg, double y_deg) {
    return (Eigen::AngleAxisd(x_deg * DEG_TO_RAD, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(y_deg * DEG_TO_RAD, Eigen::Vector3d::UnitY()))
        .toRotationMatrix();
}

// Count occupied 2D grid cells of the points projected along z after
// rotating by Rx(x_deg) * Ry(y_deg). Scratch buffer is reused per call.
size_t occupied_cells(const Eigen::ArrayXd& px, const Eigen::ArrayXd& py, const Eigen::ArrayXd& pz,
                      double x_deg, double y_deg, double inv_resolution,
                      std::vector<int64_t>& keys) {
    const double xr = x_deg * DEG_TO_RAD;
    const double yr = y_deg * DEG_TO_RAD;
    const double cx = std::cos(xr);
    const double sx = std::sin(xr);
    const double cy = std::cos(yr);
    const double sy = std::sin(yr);

    // q = Rx(x) @ Ry(y) @ p; only the projected x and y coordinates matter.
    const Eigen::ArrayXd proj_x = px * cy + pz * sy;
    const Eigen::ArrayXd proj_y = sx * (px * sy - pz * cy) + cx * py;

    keys.clear();
    keys.reserve(static_cast<size_t>(px.size()));
    for (Eigen::Index i = 0; i < px.size(); ++i) {
        keys.push_back(cell_key(static_cast<int64_t>(std::llround(proj_x[i] * inv_resolution)),
                                static_cast<int64_t>(std::llround(proj_y[i] * inv_resolution))));
    }
    std::sort(keys.begin(), keys.end());
    return static_cast<size_t>(std::unique(keys.begin(), keys.end()) - keys.begin());
}

// Gyro-derotated mean of the valid accelerometer samples: each sample is
// rotated into the frame of the first valid one by integrating the gyro
// between samples. Returns the number of samples averaged; zero means no
// usable data.
size_t derotated_accel_mean(Eigen::Ref<const core::ArrayX3fR> acc,
                            Eigen::Ref<const core::ArrayX3fR> gyro,
                            Eigen::Ref<const Eigen::ArrayX<uint64_t>> timestamps,
                            Eigen::Ref<const Eigen::ArrayX<uint16_t>> status,
                            Eigen::Vector3d& mean) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_rate = Eigen::Vector3d::Zero();
    size_t count = 0;
    double last_timestamp = 0.0;
    for (Eigen::Index k = 0; k < acc.rows(); ++k) {
        if ((status[k] & 0x01) == 0) {
            continue;
        }
        const Eigen::Vector3d sample = acc.row(k).cast<double>().transpose();
        const Eigen::Vector3d rate = gyro.row(k).cast<double>().transpose();
        if (!sample.allFinite() || !rate.allFinite()) {
            continue;
        }
        const double timestamp = static_cast<double>(timestamps[k]) * 1e-9;
        if (count > 0 && timestamp > last_timestamp) {
            rot = rot * rotation_from_rotvec(previous_rate * (timestamp - last_timestamp));
        }
        sum += rot * sample;
        previous_rate = rate;
        last_timestamp = timestamp;
        ++count;
    }
    if (count > 0) {
        mean = sum / static_cast<double>(count);
    }
    return count;
}

// Sweep one rotation axis over [center - range, center + range] in `step`
// increments with the other axis held at `other_deg`, and return the angle
// minimizing the occupied cell count. `sweep_x` selects whether the swept
// angle is the rotation about X or about Y.
double sweep_axis(const Eigen::ArrayXd& px, const Eigen::ArrayXd& py, const Eigen::ArrayXd& pz,
                  bool sweep_x, double center, double other_deg, double step, double range,
                  double inv_resolution) {
    const size_t candidate_count = static_cast<size_t>(std::lround(2.0 * range / step)) + 1;
    const auto angle_at = [center, range, step](size_t index) {
        return center - range + step * static_cast<double>(index);
    };

    // The candidates are independent, so they are counted concurrently and
    // the winner is picked afterwards -- in index order, so the answer does
    // not depend on which worker finished first.
    std::vector<size_t> counts(candidate_count, 0);
    for_each_candidate(candidate_count, [&](size_t index, std::vector<int64_t>& keys) {
        const double angle = angle_at(index);
        const double x_deg = sweep_x ? angle : other_deg;
        const double y_deg = sweep_x ? other_deg : angle;
        counts[index] = occupied_cells(px, py, pz, x_deg, y_deg, inv_resolution, keys);
    });

    const size_t best_index =
        static_cast<size_t>(std::min_element(counts.begin(), counts.end()) - counts.begin());
    return angle_at(best_index);
}

// Per-column poses relative to the first valid column, so only intra-frame
// motion remains, composed with the gravity alignment. A lidar frame carries
// one pose per column; a camera frame carries a single pose (index 0), so
// every pixel shares it and the relative dewarp reduces to the gravity
// rotation.
core::MatrixX16dR relative_column_poses(const core::Frame& frame,
                                        const Matrix4dR& gravity_rotation) {
    const int first_col = frame.get_first_valid_column();
    const int frame_width = static_cast<int>(frame.w);
    const auto& pose_shape = frame.body_to_world().shape();
    const int pose_count = pose_shape.empty() ? 0 : static_cast<int>(pose_shape[0]);
    const bool per_column_poses = pose_count == frame_width;
    const Matrix4dR anchor_inverse = Matrix4dR(PoseH(frame.get_column_pose(first_col))).inverse();
    const Matrix4dR global_to_wall = gravity_rotation * anchor_inverse;
    core::MatrixX16dR rel_poses(frame_width, 16);
    for (int col = 0; col < frame_width; ++col) {
        const int pose_index = per_column_poses ? col : 0;
        const Matrix4dR pose_c =
            global_to_wall * Matrix4dR(PoseH(frame.get_column_pose(pose_index)));
        Eigen::Map<Matrix4dR>(rel_poses.row(col).data()) = pose_c;
    }
    return rel_poses;
}

// A frame's own WALL fields copied out, one per return that carries one
// ("WALL" first, "WALL2" second), in the shape compute_walls() answers.
std::vector<core::img_t<uint8_t>> wall_masks_of(const core::Frame& frame) {
    std::vector<core::img_t<uint8_t>> masks;
    for (int ret = 0; ret < 2; ++ret) {
        const std::string mask_name = ChanField::return_field_name(ChanField::WALL, ret);
        if (!frame.has_field(mask_name)) {
            break;
        }
        masks.emplace_back(Eigen::Ref<const core::img_t<uint8_t>>(frame.field(mask_name)));
    }
    return masks;
}

// The points `masks` mark as wall in one frame, one mask per return
// (masks[0] against "RANGE", masks[1] against "RANGE2"), as N x 3 XYZ
// rows dewarped by the frame's poses relative to its first valid column and
// rotated by gravity_rotation -- the frame WallSegEngine took the masks in
// when it was handed the same rotation. 0 x 3 for no masks. Throws
// std::invalid_argument on a frame with no sensor_info() or valid column, or
// a mask whose shape does not match its range image.
core::ArrayX3dR wall_points_of(const core::Frame& frame,
                               const std::vector<core::img_t<uint8_t>>& masks,
                               const Matrix4dR& gravity_rotation) {
    if (!frame.sensor_info()) {
        throw std::invalid_argument("frame.sensor_info() is required for plumb_walls");
    }
    const auto& info = *frame.sensor_info();
    const core::MatrixX16dR rel_poses = relative_column_poses(frame, gravity_rotation);
    const auto xyz_lut = info.xyzlut<double>();

    // The masks are in the frame's own layout, and so is the cloud the table
    // gives, so a pixel indexes both directly and the dewarp reads its
    // column's pose without any destaggering. That holds whether the layout is
    // a lidar's staggered one or a camera's regular grid.
    std::vector<Eigen::Vector3d> pooled;
    for (size_t ret = 0; ret < masks.size(); ++ret) {
        const std::string range_name =
            ChanField::return_field_name(ChanField::RANGE, static_cast<int>(ret));
        if (!frame.has_field(range_name)) {
            continue;
        }
        const core::img_t<uint8_t>& mask = masks[ret];
        Eigen::Ref<const core::img_t<uint32_t>> range = frame.field(range_name);
        if (mask.rows() != range.rows() || mask.cols() != range.cols()) {
            std::stringstream message;
            message << "plumb_walls: wall mask " << ret << " is " << mask.rows() << "x"
                    << mask.cols() << ", but '" << range_name << "' is " << range.rows() << "x"
                    << range.cols();
            throw std::invalid_argument(message.str());
        }
        const PointCloudXYZd points = (*xyz_lut)(range);
        PointCloudXYZd dewarped(points.rows(), points.cols());
        core::dewarp<double>(dewarped, points, rel_poses);
        const Eigen::Index width = range.cols();
        for (Eigen::Index u = 0; u < range.rows(); ++u) {
            for (Eigen::Index v = 0; v < width; ++v) {
                if (mask(u, v) == 0 || range(u, v) == 0) {
                    continue;
                }
                pooled.emplace_back(dewarped.row(u * width + v).transpose());
            }
        }
    }

    core::ArrayX3dR result(static_cast<Eigen::Index>(pooled.size()), 3);
    for (size_t i = 0; i < pooled.size(); ++i) {
        result.row(static_cast<Eigen::Index>(i)) = pooled[i].transpose();
    }
    return result;
}

}  // namespace

nonstd::optional<Matrix4dR> plumb_imu(const core::FrameSet& frames) {
    bool found = false;
    double best_fraction = -1.0;
    size_t best_count = 0;
    Eigen::Vector3d gravity_direction = Eigen::Vector3d::UnitZ();
    size_t anchor_sensor_index = 0;
    for (size_t sensor_idx : frames.valid_indices()) {
        const auto& frame_ptr = frames[sensor_idx];
        if (!frame_ptr) {
            continue;
        }

        // Resolve the two rotations that carry an accelerometer reading into the
        // body frame. Both lidar and camera frames carry sensor->body and
        // imu->sensor extrinsics on their info (imu_to_sensor_transform is a
        // SensorInfo base field); a camera's IMU defaults to coincident with the
        // camera frame (identity) unless calibration overrides it.
        // Both transforms are shared SensorInfo base fields, so read them
        // directly from the frame's sensor_info regardless of sensor kind. A
        // camera's IMU defaults to coincident with the camera frame (identity)
        // unless calibration overrides imu_to_sensor_transform.
        if (!frame_ptr->sensor_info()) {
            continue;
        }
        const Matrix4dR sensor_to_body = frame_ptr->sensor_info()->sensor_to_body;
        const Matrix4dR imu_to_sensor = frame_ptr->sensor_info()->imu_to_sensor_transform;

        const core::Frame& frame = *frame_ptr;
        if (!frame.has_field(ChanField::IMU_ACC) || !frame.has_field(ChanField::IMU_STATUS) ||
            !frame.has_field(ChanField::IMU_GYRO) || !frame.has_field(ChanField::IMU_TIMESTAMP)) {
            continue;
        }
        Eigen::Ref<const core::ArrayX3fR> acc = frame.field(ChanField::IMU_ACC);
        Eigen::Ref<const core::ArrayX3fR> gyro = frame.field(ChanField::IMU_GYRO);
        Eigen::Ref<const Eigen::ArrayX<uint64_t>> timestamps =
            frame.field(ChanField::IMU_TIMESTAMP);
        Eigen::Ref<const Eigen::ArrayX<uint16_t>> status = frame.field(ChanField::IMU_STATUS);
        if (acc.rows() != status.rows() || acc.rows() == 0 || gyro.rows() != acc.rows() ||
            timestamps.rows() != acc.rows()) {
            continue;
        }

        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        const size_t sample_count = derotated_accel_mean(acc, gyro, timestamps, status, mean);
        if (sample_count == 0) {
            continue;
        }
        // Anchor on the sensor with the cleanest IMU stream: highest valid
        // fraction, ties broken by sample count and then lowest index. A
        // single valid sample still anchors when nothing better exists.
        const double fraction = static_cast<double>(sample_count) / static_cast<double>(acc.rows());
        if (found && (fraction < best_fraction ||
                      (fraction == best_fraction && sample_count <= best_count))) {
            continue;
        }
        const Eigen::Vector3d gravity =
            sensor_to_body.block<3, 3>(0, 0) * imu_to_sensor.block<3, 3>(0, 0) * mean;
        const double norm = gravity.norm();
        if (!std::isfinite(norm) || norm <= 0.0) {
            continue;
        }
        best_fraction = fraction;
        best_count = sample_count;
        found = true;
        gravity_direction = gravity / norm;
        anchor_sensor_index = sensor_idx;
    }
    if (!found) {
        logger().warn(
            "plumb_imu: no sensor has usable IMU data (requires the FW 3.2+ IMU profile)");
        return nonstd::nullopt;
    }
    const Eigen::Vector3d& g = gravity_direction;
    Matrix4dR correction = Matrix4dR::Identity();
    correction.block<3, 3>(0, 0) =
        core::get_rot_matrix_to_align_to_gravity(g.x(), g.y(), g.z(), /*fix_yaw=*/true);
    logger().info(
        "plumb_imu: IMU of sensor {} gives the body frame gravity direction ({:.4f}, {:.4f}, "
        "{:.4f})",
        frames[anchor_sensor_index]->sensor_info()->sn, g.x(), g.y(), g.z());
    return correction;
}

namespace {

// The wall-density fit behind plumb_walls(): the roll/pitch whose projection
// along z occupies the fewest grid cells, and how sharp that minimum was.
struct WallDensityFit {
    bool valid = false;
    Matrix4dR correction = Matrix4dR::Identity();
    double x_rotation_deg = 0.0;
    double y_rotation_deg = 0.0;
    double contrast = 0.0;
    bool boundary_pinned = false;
};

WallDensityFit fit_wall_density(Eigen::Ref<const core::ArrayX3dR> points, double grid_resolution) {
    WallDensityFit result;
    if (!std::isfinite(grid_resolution) || grid_resolution <= 0.0 || points.rows() < 3) {
        return result;
    }

    const Eigen::ArrayXd px = points.col(0);
    const Eigen::ArrayXd py = points.col(1);
    const Eigen::ArrayXd pz = points.col(2);
    const double inv_resolution = 1.0 / grid_resolution;

    // Coordinate descent over rotations about X then Y, coarse then fine.
    const double schedule[2][2] = {{COARSE_STEP_DEG, COARSE_RANGE_DEG},
                                   {FINE_STEP_DEG, FINE_RANGE_DEG}};
    double best_x = 0.0;
    double best_y = 0.0;
    for (const auto& pass : schedule) {
        best_x = sweep_axis(px, py, pz, /*sweep_x=*/true, best_x, best_y, pass[0], pass[1],
                            inv_resolution);
        best_y = sweep_axis(px, py, pz, /*sweep_x=*/false, best_y, best_x, pass[0], pass[1],
                            inv_resolution);
    }

    result.x_rotation_deg = best_x;
    result.y_rotation_deg = best_y;
    result.boundary_pinned = std::max(std::abs(best_x), std::abs(best_y)) >= PLUMB_SEARCH_EDGE_DEG;
    // Probe PLUMB_CONTRAST_PROBE_DEG off the minimum along the x and y axis
    // directions only and keep the weakest ratio, so a scene that constrains
    // only one axis (a single wall or fence) cannot report a sharp minimum.
    // The first offset is the minimum itself, batched with the probes.
    const double probe = PLUMB_CONTRAST_PROBE_DEG;
    const double probe_offsets[5][2] = {
        {0.0, 0.0}, {probe, 0.0}, {-probe, 0.0}, {0.0, probe}, {0.0, -probe}};
    std::array<size_t, 5> probe_counts{};
    for_each_candidate(probe_counts.size(), [&](size_t index, std::vector<int64_t>& keys) {
        probe_counts[index] =
            occupied_cells(px, py, pz, best_x + probe_offsets[index][0],
                           best_y + probe_offsets[index][1], inv_resolution, keys);
    });
    const size_t at_minimum = probe_counts.front();
    const size_t off_minimum = *std::min_element(probe_counts.begin() + 1, probe_counts.end());
    result.contrast =
        static_cast<double>(off_minimum) / static_cast<double>(std::max<size_t>(at_minimum, 1));
    result.correction.block<3, 3>(0, 0) = rotation_xy(best_x, best_y);
    result.valid = !result.boundary_pinned;
    return result;
}

}  // namespace

nonstd::optional<Matrix4dR> plumb_walls(const core::FrameSet& frames,
                                        const Matrix4dR& gravity_rotation) {
    // A frame that already carries a WALL field is read as it lies. Only the
    // others are segmented, and only in memory: nothing on the set is ever
    // written, so there is nothing to clean up on any path out.
    std::vector<std::shared_ptr<core::Frame>> to_segment(frames.size());
    size_t segment_count = 0;
    size_t reuse_count = 0;
    for (size_t sensor_idx : frames.valid_indices()) {
        if (frames[sensor_idx]->has_field(ChanField::WALL)) {
            ++reuse_count;
        } else {
            to_segment[sensor_idx] = frames[sensor_idx];
            ++segment_count;
        }
    }
    logger().info("plumb_walls: segmenting {} frames, reusing existing WALL fields from {}",
                  segment_count, reuse_count);
    std::vector<std::vector<core::img_t<uint8_t>>> computed(frames.size());
    if (segment_count > 0) {
        WallSegConfig config;
        config.gravity_rotation = gravity_rotation;
        // The null slots are the frames the engine must leave alone.
        computed = WallSegEngine::create(config)->compute_walls(core::FrameSet(to_segment));
    }

    // Every marked pixel of every sensor and return, as points in the
    // aligned frame. A sensor that cannot answer is left out rather than
    // allowed to fail the whole set: the others may still carry a wall.
    std::vector<core::ArrayX3dR> per_sensor;
    per_sensor.reserve(frames.size());
    Eigen::Index pooled_count = 0;
    for (size_t sensor_idx : frames.valid_indices()) {
        const core::Frame& frame = *frames[sensor_idx];
        try {
            std::vector<core::img_t<uint8_t>> masks;
            if (to_segment[sensor_idx]) {
                masks = std::move(computed[sensor_idx]);
            } else {
                masks = wall_masks_of(frame);
            }
            per_sensor.push_back(wall_points_of(frame, masks, gravity_rotation));
            pooled_count += per_sensor.back().rows();
        } catch (const std::exception& error) {
            logger().warn("plumb_walls: no wall points from sensor {}: {}", sensor_idx,
                          error.what());
        }
    }

    const size_t wall_point_count = static_cast<size_t>(pooled_count);
    if (wall_point_count < PLUMB_WALLS_MIN_WALL_POINTS) {
        logger().warn(
            "plumb_walls: only {} points stand on a wall surface, minimum {}; no correction",
            wall_point_count, PLUMB_WALLS_MIN_WALL_POINTS);
        return nonstd::nullopt;
    }
    core::ArrayX3dR pooled_points(pooled_count, 3);
    Eigen::Index pooled_row = 0;
    for (const core::ArrayX3dR& points : per_sensor) {
        pooled_points.middleRows(pooled_row, points.rows()) = points;
        pooled_row += points.rows();
    }
    const WallDensityFit fit = fit_wall_density(pooled_points, PLUMB_WALLS_GRID_RESOLUTION_M);
    if (!fit.valid) {
        logger().warn(
            "plumb_walls: {} wall points, solution ({:+.1f}, {:+.1f}) deg{}; no correction",
            wall_point_count, fit.x_rotation_deg, fit.y_rotation_deg,
            fit.boundary_pinned ? " pinned at the search boundary" : " not found");
        return nonstd::nullopt;
    }
    if (fit.contrast < PLUMB_WALLS_LOW_CONTRAST) {
        logger().warn(
            "plumb_walls: low wall-structure confidence, contrast {:.2f}; applying the "
            "refinement anyway",
            fit.contrast);
    }
    logger().info(
        "plumb_walls: {} wall points, correction {:+.2f} deg about X, {:+.2f} deg about Y, "
        "contrast {:.2f}",
        wall_point_count, fit.x_rotation_deg, fit.y_rotation_deg, fit.contrast);
    return fit.correction;
}

nonstd::optional<Matrix4dR> plumb(const core::FrameSet& frames, bool use_imu, bool use_walls) {
    nonstd::optional<Matrix4dR> imu;
    if (use_imu) {
        imu = plumb_imu(frames);
        if (!imu && use_walls) {
            logger().warn(
                "plumb: refining from wall geometry alone, which assumes the current "
                "orientation is already within the wall search range of level");
        }
    }
    const Matrix4dR identity = Matrix4dR::Identity();
    nonstd::optional<Matrix4dR> walls;
    if (use_walls) {
        walls = plumb_walls(frames, imu.value_or(identity));
        if (!walls && imu) {
            logger().info("plumb: keeping the IMU levelling without the wall refinement");
        }
    }
    if (!imu && !walls) {
        return nonstd::nullopt;
    }
    Matrix4dR correction = walls.value_or(identity) * imu.value_or(identity);
    return correction;
}

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster

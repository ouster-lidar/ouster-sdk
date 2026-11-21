#include "ouster/mapping/deskew_method.h"

#include <ouster/core/chanfield.h>
#include <ouster/core/defaults.h>
#include <ouster/core/frame_set.h>
#include <ouster/core/impl/logging.h>
#include <ouster/core/lidar_info.h>
#include <ouster/core/pose_util.h>
#include <ouster/core/sensor_info.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <utility>

#include "ouster/algorithm/plumb.h"
#include "slam_util.h"

using namespace ouster::sdk::core;  // NOLINT(google-build-using-namespace)

#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
const auto EIGEN_INDEXING_ALL = Eigen::indexing::all;
#else
const auto EIGEN_INDEXING_ALL = Eigen::all;
#endif

namespace ouster {
namespace sdk {
namespace mapping {

namespace {

/// Plumbed gravity more than this many degrees off world -Z under a supplied
/// initial_pose warns; the pose is still used.
constexpr double INITIAL_POSE_LEVEL_TOL_DEG = 10.0;

double degrees_from_level(const Eigen::Vector3d& up_world_frame_xyz) {
    const double cosine = std::max(-1.0, std::min(1.0, up_world_frame_xyz.normalized().z()));
    return std::acos(cosine) / DEG_TO_RAD;
}

}  // namespace

namespace impl {

// NOLINTBEGIN(readability-identifier-length)
void interp_pose(LidarFrame& frame, double t0, Eigen::Ref<const Matrix4dR> x0, double t1,
                 Eigen::Ref<const Matrix4dR> x1) {
    std::vector<int> valid_cols = impl::get_valid_columns<uint32_t>(frame.status());
    std::vector<double> frame_ts_vec = impl::get_valid_timestamps(frame.timestamp(), valid_cols);
    std::vector<Matrix4dR> interp = ouster::sdk::core::interp_pose(frame_ts_vec, t0, x0, t1, x1);
    for (size_t k = 0; k < valid_cols.size(); ++k) {
        frame.set_column_pose(valid_cols[k], interp[k]);
    }
}
// NOLINTEND(readability-identifier-length)

/// Whether frame @p sidx of @p frame_set should feed the IMU deskew.
///
/// imu_deskew is a lidar deskew: its per-sensor state (imu_to_body_transform_,
/// biases) shares the FrameSet index space (one slot per sensor, in FrameSet
/// order). Camera frames may carry Ouster-format IMU fields (e.g. a ZED saved
/// alongside a lidar), so IMU consumption is restricted to lidar frames; camera
/// IMU never contributes to lidar deskew.
namespace {
bool is_lidar_imu_source(const FrameSet& frame_set, size_t sidx, size_t sensor_count) {
    return sidx < sensor_count && std::dynamic_pointer_cast<LidarFrame>(frame_set[sidx]) != nullptr;
}
}  // namespace

}  // namespace impl

namespace {

// Split the IMU interval containing target with a node at target, driven by the sample that owns
// that interval, so integration can stop exactly there. Returns the index of that node.
size_t insert_imu_node_at(std::vector<double>& timestamps, MatrixX3dR& gyro, MatrixX3dR& accel,
                          double target) {
    auto upper_it = std::lower_bound(timestamps.begin(), timestamps.end(), target);
    const auto row = static_cast<Eigen::Index>(std::distance(timestamps.begin(), upper_it));
    if (upper_it != timestamps.end() && *upper_it == target) {
        return static_cast<size_t>(row);
    }
    const Eigen::Index owner = std::min<Eigen::Index>(row, gyro.rows() - 1);
    auto insert_row = [row, owner](MatrixX3dR& samples) {
        MatrixX3dR out(samples.rows() + 1, 3);
        out.topRows(row) = samples.topRows(row);
        out.row(row) = samples.row(owner);
        out.bottomRows(samples.rows() - row) = samples.bottomRows(samples.rows() - row);
        samples = std::move(out);
    };
    insert_row(gyro);
    insert_row(accel);
    timestamps.insert(upper_it, target);
    return static_cast<size_t>(row);
}

}  // namespace

// Matrix4dR is a fixed-size Eigen type, which the codebase passes by const ref
// rather than by value (passing fixed-size Eigen objects by value has a known
// stack-alignment caveat), so suppress the pass-by-value suggestion here.
// NOLINTBEGIN(modernize-pass-by-value)
DeskewMethod::DeskewMethod(const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
                           const nonstd::optional<Matrix4dR>& initial_pose)
    : initial_pose_(initial_pose) {
    if (sensor_infos.empty()) {
        throw std::invalid_argument("No sensor info provided for slam");
    }
}
// NOLINTEND(modernize-pass-by-value)

void ConstantVelocityDeskewMethod::update(FrameSet& frame_set) {
    // Not enough poses to perform constant velocity deskewing. Need at least
    // two poses to start applying deskewing.
    if (ts_list_.size() < 2) {
        // Not enough history to interpolate yet. Anchor the frame at the
        // configured initial pose (identity by default) so the trajectory
        // starts there instead of always at the map origin.
        impl::init_valid_column_poses(frame_set, initial_pose_.value_or(Matrix4dR::Identity()));
        return;
    }

    for (auto& frame : frame_set.valid_lidar_frames()) {
        impl::interp_pose(*frame, ts_list_.front(), pose_list_.front(), ts_list_.back(),
                          pose_list_.back());
    }
}

InertialIntegrationImuDeskewMethod::InertialIntegrationImuDeskewMethod(
    const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
    const nonstd::optional<Matrix4dR>& initial_pose, const std::string& plumb_method,
    bool gravity_from_initial_pose)
    : DeskewMethod(sensor_infos, initial_pose),
      gravity_from_initial_pose_(initial_pose && gravity_from_initial_pose),
      level_world_frame_(!initial_pose && gravity_from_initial_pose) {
    if (plumb_method != "imu" && plumb_method != "walls" && plumb_method != "all") {
        throw std::invalid_argument("Invalid plumb_method: " + plumb_method);
    }
    plumb_imu_ = plumb_method != "walls";
    plumb_walls_ = plumb_method != "imu";
    if (gravity_from_initial_pose_) {
        // The world frame of a supplied pose is level; the IMU plumb only checks it.
        gravity_vector_world_frame_ = Eigen::Vector3d::UnitZ();
        plumb_imu_ = true;
        plumb_walls_ = false;
    }
    // Per-sensor state is sized to the full sensor list and indexed by frameset
    // position; only lidar entries carry an IMU-to-body transform. Non-lidar
    // slots stay at identity/zero and are never consumed (see
    // is_lidar_imu_source), so the list may mix lidar and camera in any order.
    accel_bias_imu_frame_.resize(sensor_infos.size(), Eigen::Vector3d::Zero());
    gyro_bias_imu_frame_.resize(sensor_infos.size(), Eigen::Vector3d::Zero());
    imu_to_body_transform_.resize(sensor_infos.size(), Matrix4dR::Identity());
    for (size_t i = 0; i < sensor_infos.size(); ++i) {
        auto lidar = std::dynamic_pointer_cast<LidarInfo>(sensor_infos[i]);
        if (!lidar) {
            continue;
        }
        Matrix4dR imu_transform = lidar->imu_to_sensor_transform;
        imu_transform.block<3, 1>(0, 3) *= 1e-3;  // mm -> m
        imu_to_body_transform_[i] = lidar->sensor_to_body * imu_transform;
    }
}

void InertialIntegrationImuDeskewMethod::collect_imu_poses(const FrameSet& frame_set,
                                                           std::vector<double>& timestamps,
                                                           std::vector<Matrix4dR>& poses) {
    std::vector<double> sample_ts;
    std::vector<Matrix4dR> sample_poses;
    for (const auto& frame : frame_set.valid_frames()) {
        // Lidar IMU only: camera frames may carry IMU fields but do not
        // participate in lidar deskew (see is_lidar_imu_source).
        if (!std::dynamic_pointer_cast<LidarFrame>(frame) ||
            !frame->has_field(ChanField::IMU_STATUS)) {
            continue;
        }

        Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status = frame->field(ChanField::IMU_STATUS);
        Eigen::Ref<const Eigen::ArrayX<uint64_t>> imu_ts_arr =
            frame->field(ChanField::IMU_TIMESTAMP);
        Eigen::Ref<const Eigen::ArrayX<uint16_t>> meas_id =
            frame->field(ChanField::IMU_MEASUREMENT_ID);
        for (int imu_idx : impl::get_valid_columns<uint16_t>(imu_status)) {
            sample_ts.push_back(static_cast<double>(imu_ts_arr[imu_idx]) * 1e-9);
            sample_poses.push_back(frame->get_column_pose(meas_id(imu_idx)));
        }
    }

    std::vector<size_t> order(sample_ts.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(),
                     [&](size_t lhs, size_t rhs) { return sample_ts[lhs] < sample_ts[rhs]; });
    timestamps.clear();
    poses.clear();
    for (size_t idx : order) {
        if (timestamps.empty() || sample_ts[idx] > timestamps.back()) {
            timestamps.push_back(sample_ts[idx]);
            poses.push_back(sample_poses[idx]);
        }
    }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void InertialIntegrationImuDeskewMethod::update(FrameSet& frame_set) {
    // Only SLAM without an initial_pose levels the world frame.
    const bool level_world_frame = !world_frame_determined_ && level_world_frame_;
    size_t total_possible_imu_data = 0;
    std::vector<std::vector<int>> imu_valid_per_sensor(frame_set.size());
    for (size_t sidx : frame_set.valid_indices()) {
        if (!impl::is_lidar_imu_source(frame_set, sidx, imu_to_body_transform_.size())) {
            continue;
        }
        const auto& frame = *frame_set[sidx];
        if (!frame.has_field(ChanField::IMU_STATUS)) {
            continue;
        }
        Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status = frame.field(ChanField::IMU_STATUS);
        std::vector<int> imu_valid = impl::get_valid_columns<uint16_t>(imu_status);
        total_possible_imu_data += imu_valid.size();
        imu_valid_per_sensor[sidx] = std::move(imu_valid);
    }

    // Combine imu data from all sensors. Each per-sensor contribution is
    // appended as a contiguous, monotonically-increasing sub-range (IMU
    // timestamps are monotonic within a frame and get_valid_columns returns
    // ascending indices). The ranges are merged in timestamp order below.
    std::vector<double> combined_imu_ts;
    combined_imu_ts.reserve(total_possible_imu_data);
    std::vector<Eigen::Vector3d> combined_gyro_body_frame;
    combined_gyro_body_frame.reserve(total_possible_imu_data);
    std::vector<Eigen::Vector3d> combined_accel_body_frame;
    combined_accel_body_frame.reserve(total_possible_imu_data);
    std::vector<size_t> cursors;
    std::vector<size_t> ends;
    cursors.reserve(frame_set.size());
    ends.reserve(frame_set.size());

    for (size_t sidx : frame_set.valid_indices()) {
        if (!impl::is_lidar_imu_source(frame_set, sidx, imu_to_body_transform_.size())) {
            continue;
        }
        const auto& frame = *frame_set[sidx];
        if (!frame.has_field(ChanField::IMU_STATUS)) {
            continue;
        }

        const std::vector<int>& imu_valid = imu_valid_per_sensor[sidx];

        if (imu_valid.empty()) {
            continue;
        }

        Eigen::Ref<const Eigen::ArrayX<uint64_t>> imu_timestamps =
            frame.field(ChanField::IMU_TIMESTAMP);
        Eigen::Ref<const ArrayX3fR> gyro = frame.field(ChanField::IMU_GYRO);
        Eigen::Ref<const ArrayX3fR> acc = frame.field(ChanField::IMU_ACC);
        MatrixX3dR valid_gyro = gyro(imu_valid, EIGEN_INDEXING_ALL).cast<double>();
        MatrixX3dR valid_acc = acc(imu_valid, EIGEN_INDEXING_ALL).cast<double>();
        valid_gyro.rowwise() -= gyro_bias_imu_frame_[sidx].transpose();
        valid_acc.rowwise() -= accel_bias_imu_frame_[sidx].transpose();

        MatrixX3dR gyro_body_frame;
        MatrixX3dR accel_body_frame;
        transform_imu_data_to_body_frame(imu_to_body_transform_[sidx], valid_gyro, valid_acc,
                                         gyro_body_frame, accel_body_frame);

        // Track where this sensor's data begins in the combined buffer so the
        // k-way merge can iterate each sensor's slice via cursors/ends.
        const size_t range_begin = combined_imu_ts.size();
        for (size_t k = 0; k < imu_valid.size(); ++k) {
            combined_imu_ts.push_back(static_cast<double>(imu_timestamps(imu_valid[k])) * 1e-9);
            combined_gyro_body_frame.emplace_back(
                gyro_body_frame.row(static_cast<Eigen::Index>(k)));
            combined_accel_body_frame.emplace_back(
                accel_body_frame.row(static_cast<Eigen::Index>(k)));
        }
        if (combined_imu_ts.size() > range_begin) {
            cursors.push_back(range_begin);
            ends.push_back(combined_imu_ts.size());
        }
    }

    // No usable IMU data: fall back to pose interpolation when SLAM poses are
    // available, otherwise write identity to avoid downstream errors.
    auto deskew_without_imu = [&]() {
        if (ts_list_.size() >= 2) {
            for (auto& frame : frame_set.valid_lidar_frames()) {
                impl::interp_pose(*frame, ts_list_.front(), pose_list_.front(), ts_list_.back(),
                                  pose_list_.back());
            }
        } else {
            impl::init_valid_column_poses(frame_set, initial_pose_.value_or(Matrix4dR::Identity()));
        }
        // No IMU samples here, so walls are the only stage that can measure
        // gravity. With "imu" the world frame stays undetermined for the next frame set.
        if (plumb_walls_ && !world_frame_determined_) {
            plumb_gravity_from_walls(frame_set, level_world_frame);
        }
        last_frame_set_ = frame_set;
    };

    if (combined_imu_ts.empty()) {
        deskew_without_imu();
        return;
    }

    // k-way merge: each per-sensor sub-range of combined_imu_ts is already
    // monotonically increasing. Walk all cursors simultaneously, always
    // advancing the one with the smallest current timestamp.
    // Duplicate timestamps (across or within sensors) are dropped, matching
    // the previous std::unique behavior.
    const size_t k_sensors = cursors.size();
    std::vector<size_t> indices;
    indices.reserve(combined_imu_ts.size());
    while (true) {
        // k_sensors is used as a sentinel meaning "no valid cursor found yet".
        size_t best_k = k_sensors;
        for (size_t k = 0; k < k_sensors; ++k) {
            if (cursors[k] >= ends[k]) {
                continue;
            }
            if (best_k == k_sensors ||
                combined_imu_ts[cursors[k]] < combined_imu_ts[cursors[best_k]]) {
                best_k = k;
            }
        }
        // All cursors exhausted — merge complete.
        if (best_k == k_sensors) {
            break;
        }
        // Drop duplicate timestamps across sensors (matches previous
        // std::unique behavior on the concatenated, unsorted buffer).
        if (indices.empty() ||
            combined_imu_ts[cursors[best_k]] != combined_imu_ts[indices.back()]) {
            indices.push_back(cursors[best_k]);
        }
        ++cursors[best_k];
    }
    // assemble the merged, deduplicated imu measurements in timestamp order
    std::vector<double> sorted_imu_ts(indices.size());
    MatrixX3dR sorted_gyro(indices.size(), 3);
    MatrixX3dR sorted_accel(indices.size(), 3);

    for (size_t i = 0; i < indices.size(); ++i) {
        sorted_imu_ts[i] = combined_imu_ts[indices[i]];
        sorted_gyro.row(static_cast<Eigen::Index>(i)) = combined_gyro_body_frame[indices[i]];
        sorted_accel.row(static_cast<Eigen::Index>(i)) = combined_accel_body_frame[indices[i]];
    }

    if (rolling_imu_timestamps_.empty()) {
        rolling_imu_timestamps_ = sorted_imu_ts;
        rolling_gyro_body_frame_ = sorted_gyro;
        rolling_accel_body_frame_ = sorted_accel;
    } else {
        auto append_start = std::upper_bound(sorted_imu_ts.begin(), sorted_imu_ts.end(),
                                             rolling_imu_timestamps_.back());
        const Eigen::Index append_count =
            static_cast<Eigen::Index>(std::distance(append_start, sorted_imu_ts.end()));
        if (append_count > 0) {
            const Eigen::Index old_rows = rolling_gyro_body_frame_.rows();
            const Eigen::Index new_rows = old_rows + append_count;
            const Eigen::Index append_row =
                static_cast<Eigen::Index>(std::distance(sorted_imu_ts.begin(), append_start));
            rolling_gyro_body_frame_.conservativeResize(new_rows, Eigen::NoChange);
            rolling_accel_body_frame_.conservativeResize(new_rows, Eigen::NoChange);
            rolling_gyro_body_frame_.bottomRows(append_count) =
                sorted_gyro.middleRows(append_row, append_count);
            rolling_accel_body_frame_.bottomRows(append_count) =
                sorted_accel.middleRows(append_row, append_count);
            rolling_imu_timestamps_.insert(rolling_imu_timestamps_.end(), append_start,
                                           sorted_imu_ts.end());
        }
    }

    double first_col_ts = sorted_imu_ts.front();
    double last_col_ts = sorted_imu_ts.back();
    for (const auto& frame : frame_set.valid_lidar_frames()) {
        if (!impl::has_valid_columns(*frame)) {
            continue;
        }
        first_col_ts = std::min(
            first_col_ts,
            static_cast<double>(frame->timestamp()[frame->get_first_valid_column()]) * 1e-9);
        last_col_ts = std::max(
            last_col_ts,
            static_cast<double>(frame->timestamp()[frame->get_last_valid_column()]) * 1e-9);
    }

    // Anchor at the previous frame set's latest IMU sample at or before this frame set's first
    // column, or at its earliest one if all are later. With none cached, fall back to SLAM
    // history, and on the first frame to the initial pose at the first column.
    double last_ts = first_col_ts;
    Matrix4dR last_pose = initial_pose_.value_or(Matrix4dR::Identity());
    const bool first_frame = ts_list_.empty();
    if (!imu_anchor_ts_.empty()) {
        const auto anchor_it =
            std::upper_bound(imu_anchor_ts_.begin(), imu_anchor_ts_.end(), first_col_ts);
        const size_t anchor_idx =
            anchor_it == imu_anchor_ts_.begin()
                ? 0
                : static_cast<size_t>(std::distance(imu_anchor_ts_.begin(), anchor_it)) - 1;
        last_ts = imu_anchor_ts_[anchor_idx];
        last_pose = imu_anchor_poses_[anchor_idx];
    } else if (!ts_list_.empty()) {
        last_ts = ts_list_.back();
        last_pose = pose_list_.back();
    }

    // Plumbing measures gravity in the body frame; the anchor rotates it into
    // the world frame.
    if (plumb_imu_ && (!world_frame_determined_ || !plumbed_up_world_frame_vector_)) {
        plumb_gravity_from_imu(frame_set, last_pose, level_world_frame);
        // Once the world frame is determined this only retries a failed plumb,
        // so check its first success.
        if (gravity_from_initial_pose_ && world_frame_determined_ &&
            plumbed_up_world_frame_vector_) {
            validate_initial_pose_gravity();
        }
    }

    // Initial linear velocity. Use the finite-difference between the two most
    // recent SLAM poses once they are available. Before that, zero is a
    // reasonable approximation: rotational deskew from gyro is accurate
    // regardless, and approximate translational deskew beats none at all.
    Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
    if (last_linear_velocity_world_frame_) {
        initial_velocity = *last_linear_velocity_world_frame_;
        // That is the velocity at the previous frame set's last IMU sample; an earlier anchor
        // interpolates the velocity solve's nodes around it, clamped to the first node.
        const auto node_it =
            std::upper_bound(velocity_nodes_ts_.begin(), velocity_nodes_ts_.end(), last_ts);
        if (!velocity_nodes_ts_.empty() && node_it == velocity_nodes_ts_.begin()) {
            initial_velocity = velocity_nodes_world_frame_.front();
        } else if (node_it != velocity_nodes_ts_.begin() && node_it != velocity_nodes_ts_.end()) {
            const auto next =
                static_cast<size_t>(std::distance(velocity_nodes_ts_.begin(), node_it));
            const double alpha = (last_ts - velocity_nodes_ts_[next - 1]) /
                                 (velocity_nodes_ts_[next] - velocity_nodes_ts_[next - 1]);
            initial_velocity =
                velocity_nodes_world_frame_[next - 1] +
                alpha * (velocity_nodes_world_frame_[next] - velocity_nodes_world_frame_[next - 1]);
        }
    } else if (ts_list_.size() >= 2) {
        initial_velocity =
            (pose_list_.back().block<3, 1>(0, 3) - pose_list_.front().block<3, 1>(0, 3)) /
            (ts_list_.back() - ts_list_.front());
    }

    // Integrate forward from the anchor, which becomes the first node, so every column after it
    // is interpolated; samples at or before it are skipped. Past the last sample its reading is
    // held up to the last column.
    const auto resume_it = std::upper_bound(sorted_imu_ts.begin(), sorted_imu_ts.end(), last_ts);
    const auto resume_rows =
        static_cast<Eigen::Index>(std::distance(resume_it, sorted_imu_ts.end()));
    std::vector<double> nodes_ts(resume_it, sorted_imu_ts.end());
    MatrixX3dR nodes_gyro = sorted_gyro.bottomRows(resume_rows);
    MatrixX3dR nodes_accel = sorted_accel.bottomRows(resume_rows);
    const double last_node_ts = nodes_ts.empty() ? last_ts : nodes_ts.back();
    if (last_col_ts > last_node_ts) {
        nodes_ts.push_back(last_col_ts);
        nodes_gyro.conservativeResize(resume_rows + 1, Eigen::NoChange);
        nodes_accel.conservativeResize(resume_rows + 1, Eigen::NoChange);
        nodes_gyro.bottomRows(1) = sorted_gyro.bottomRows(1);
        nodes_accel.bottomRows(1) = sorted_accel.bottomRows(1);
    }
    if (nodes_ts.empty()) {
        deskew_without_imu();
        return;
    }

    MatrixX3dR nodes_velocities(nodes_gyro.rows(), 3);
    std::vector<Matrix4dR> imu_poses_world_frame = calc_poses_with_motion_model(
        last_ts, last_pose, initial_velocity, nodes_ts, nodes_gyro, nodes_accel, nodes_velocities);

    nodes_ts.insert(nodes_ts.begin(), last_ts);
    imu_poses_world_frame.insert(imu_poses_world_frame.begin(), last_pose);
    // set_last_pose isn't always called, so seed the next update with the open-loop anchor and
    // velocity; set_last_pose and finalize_after_registration overwrite them.
    if (!first_frame) {
        velocity_nodes_ts_ = nodes_ts;
        velocity_nodes_world_frame_.assign(1, initial_velocity);
        for (Eigen::Index row = 0; row < nodes_velocities.rows(); ++row) {
            velocity_nodes_world_frame_.emplace_back(nodes_velocities.row(row).transpose());
        }
        last_linear_velocity_world_frame_ = velocity_nodes_world_frame_.back();
        imu_anchor_ts_ = nodes_ts;
        imu_anchor_poses_ = imu_poses_world_frame;
    }
    interpolate_imu_poses_to_frame_set(frame_set, nodes_ts, imu_poses_world_frame);

    if (plumb_walls_ && !world_frame_determined_) {
        // Refine on the deskewed poses, before registration builds the map on them.
        plumb_gravity_from_walls(frame_set, level_world_frame);
    }
    last_frame_set_ = frame_set;
}

void InertialIntegrationImuDeskewMethod::finalize_after_registration(
    FrameSet& frame_set, int64_t anchor_timestamp_ns, const Matrix4dR& corrected_anchor_pose) {
    const double target_ts = static_cast<double>(anchor_timestamp_ns) * 1e-9;
    velocity_nodes_ts_.clear();
    velocity_nodes_world_frame_.clear();

    auto finish_with_current_poses = [&]() {
        last_frame_set_ = frame_set;
        set_last_pose(anchor_timestamp_ns, corrected_anchor_pose);
    };

    auto prune_rolling_imu_before = [&](double timestamp) {
        auto first_at_or_after_timestamp = std::lower_bound(
            rolling_imu_timestamps_.begin(), rolling_imu_timestamps_.end(), timestamp);
        const Eigen::Index drop_rows = static_cast<Eigen::Index>(
            std::distance(rolling_imu_timestamps_.begin(), first_at_or_after_timestamp));
        if (drop_rows == 0) {
            return;
        }
        const Eigen::Index remaining_rows = rolling_gyro_body_frame_.rows() - drop_rows;
        if (remaining_rows <= 0) {
            rolling_imu_timestamps_.clear();
            rolling_gyro_body_frame_.resize(0, 3);
            rolling_accel_body_frame_.resize(0, 3);
            return;
        }
        rolling_imu_timestamps_.erase(rolling_imu_timestamps_.begin(), first_at_or_after_timestamp);
        rolling_gyro_body_frame_ = rolling_gyro_body_frame_.bottomRows(remaining_rows).eval();
        rolling_accel_body_frame_ = rolling_accel_body_frame_.bottomRows(remaining_rows).eval();
    };

    if (rolling_imu_timestamps_.empty() || ts_list_.empty()) {
        finish_with_current_poses();
        return;
    }

    const double last_ts = ts_list_.back();
    const Matrix4dR& last_pose = pose_list_.back();

    if (target_ts <= last_ts || target_ts > rolling_imu_timestamps_.back()) {
        logger().warn(
            "Skipping post-ICP IMU deskew refinement because the target "
            "timestamp is outside the rolling IMU integration interval.");
        finish_with_current_poses();
        return;
    }

    auto anchor_it =
        std::lower_bound(rolling_imu_timestamps_.begin(), rolling_imu_timestamps_.end(), last_ts);
    const Eigen::Index start_row =
        static_cast<Eigen::Index>(std::distance(rolling_imu_timestamps_.begin(), anchor_it));
    const Eigen::Index remaining_rows = rolling_gyro_body_frame_.rows() - start_row;
    if (remaining_rows <= 0) {
        finish_with_current_poses();
        return;
    }
    std::vector<double> refined_ts(anchor_it, rolling_imu_timestamps_.end());
    MatrixX3dR refined_gyro = rolling_gyro_body_frame_.bottomRows(remaining_rows);
    MatrixX3dR refined_accel = rolling_accel_body_frame_.bottomRows(remaining_rows);

    nonstd::optional<VelocitySolveResult> solve =
        solve_initial_velocity(last_ts, last_pose, target_ts, corrected_anchor_pose, refined_ts,
                               refined_gyro, refined_accel);
    if (!solve) {
        const double delta_t = target_ts - last_ts;
        if (delta_t > 0.0) {
            const Eigen::Vector3d delta_pos =
                corrected_anchor_pose.block<3, 1>(0, 3) - last_pose.block<3, 1>(0, 3);
            last_linear_velocity_world_frame_ = delta_pos / delta_t;
        }
        finish_with_current_poses();
        return;
    }

    if (!std::isfinite(solve->initial_velocity_world_frame.norm())) {
        logger().warn(
            "Skipping post-ICP IMU deskew refinement: solved velocity is "
            "non-finite.");
        prune_rolling_imu_before(target_ts);
        finish_with_current_poses();
        return;
    }

    last_linear_velocity_world_frame_ = solve->ending_velocity_world_frame;
    velocity_nodes_ts_ = std::move(solve->nodes_ts);
    velocity_nodes_world_frame_ = std::move(solve->nodes_velocities_world_frame);
    prune_rolling_imu_before(target_ts);
    finish_with_current_poses();
}

void InertialIntegrationImuDeskewMethod::set_last_pose(int64_t timestamp_ns,
                                                       const Matrix4dR& pose) {
    // The next update integrates from this pose, whether registration
    // corrected it or ICP was skipped, so it determines the world frame. Levelling
    // afterwards would put later frame sets in a different world frame than
    // this one. Frame sets dropped before any pose came back start over from
    // the initial pose and are plumbed again.
    if (!world_frame_determined_) {
        world_frame_determined_ = true;
        if (gravity_from_initial_pose_) {
            // Validate only now that the world frame is determined,
            // to avoid repeated warnings.
            validate_initial_pose_gravity();
        } else if (level_world_frame_ && !plumbed_up_world_frame_vector_) {
            logger().warn(
                "Plumbing could not level the FrameSet the trajectory starts "
                "from, so the world frame starts at the identity pose instead "
                "of being levelled with gravity");
        }
    }

    // Store the previous frame set's IMU-aligned poses before active time
    // correction resets timestamps. This keeps the next integration anchor in
    // the corrected timestamp domain used by update().
    std::vector<double> anchor_ts;
    std::vector<Matrix4dR> anchor_poses;
    collect_imu_poses(last_frame_set_, anchor_ts, anchor_poses);
    if (!anchor_ts.empty()) {
        imu_anchor_ts_ = std::move(anchor_ts);
        imu_anchor_poses_ = std::move(anchor_poses);
    }
    DeskewMethod::set_last_pose(timestamp_ns, pose);
    estimate_gravity_vector(last_frame_set_);
}

Eigen::Vector3d InertialIntegrationImuDeskewMethod::gravity_direction_from_pose(
    Eigen::Ref<const Matrix4dR> body_to_world_pose) {
    // R^T * [0, 0, -1] is the negated third row of R, expressed as a column.
    return -body_to_world_pose.block<1, 3>(2, 0).transpose().normalized();
}

void InertialIntegrationImuDeskewMethod::validate_initial_pose_gravity() {
    if (!plumbed_up_world_frame_vector_) {
        logger().warn(
            "Plumbing failed on the frame set the trajectory starts from; "
            "keeping world -Z as the gravity direction and validating "
            "initial_pose once a later IMU plumb succeeds");
        return;
    }

    // Plumbed up should be world +Z; yaw and translation cannot affect it.
    const double angle_deg = degrees_from_level(*plumbed_up_world_frame_vector_);
    if (angle_deg > INITIAL_POSE_LEVEL_TOL_DEG) {
        logger().warn(
            "Plumbed gravity is {:.2f} degrees off world -Z (threshold: {:.1f} degrees); the "
            "initial_pose may not be levelled, the extrinsics may be wrong, or the sensor may be "
            "moving. Keeping the supplied initial_pose",
            angle_deg, INITIAL_POSE_LEVEL_TOL_DEG);
    }
}

void InertialIntegrationImuDeskewMethod::plumb_gravity_from_imu(FrameSet& frame_set,
                                                                Matrix4dR& anchor_pose,
                                                                bool level_anchor) {
    const auto plumb = algorithm::plumb_imu(frame_set);
    if (!plumb) {
        return;
    }

    // The correction aligns gravity with world -Z. Store the opposite
    // direction to match the stationary accelerometer signal.
    const Eigen::Vector3d up_body = -gravity_direction_from_pose(*plumb);
    if (level_anchor) {
        anchor_pose.block<3, 3>(0, 0) = plumb->block<3, 3>(0, 0);
    }
    set_plumbed_up((anchor_pose.block<3, 3>(0, 0) * up_body).normalized());

    logger().info(
        "Plumbed the gravity direction from the IMU{}: world frame gravity direction "
        "({:.4f}, {:.4f}, {:.4f})",
        level_anchor ? " and levelled the first frame" : "", plumbed_up_world_frame_vector_->x(),
        plumbed_up_world_frame_vector_->y(), plumbed_up_world_frame_vector_->z());
}

void InertialIntegrationImuDeskewMethod::plumb_gravity_from_walls(FrameSet& frame_set,
                                                                  bool level_poses) {
    // plumb_walls dewarps relative to each frame's first valid column; hand back the
    // world orientation at the earliest one, since a late anchor has moved on.
    Matrix3dR first_column_rotation = Matrix3dR::Identity();
    uint64_t earliest_ts = std::numeric_limits<uint64_t>::max();
    for (const auto& frame : frame_set.valid_frames()) {
        if (!impl::has_valid_columns(*frame)) {
            continue;
        }
        const int first_col = frame->get_first_valid_column();
        const uint64_t col_ts = frame->timestamp()[first_col];
        if (col_ts < earliest_ts) {
            earliest_ts = col_ts;
            first_column_rotation = frame->get_column_pose(first_col).block<3, 3>(0, 0);
        }
    }
    // Level with the IMU seed so the search starts near zero.
    const Eigen::Vector3d up_world = plumbed_up_world_frame_vector_.value_or(
        gravity_vector_world_frame_.value_or(Eigen::Vector3d::UnitZ()));
    const Eigen::Matrix3d level = get_rot_matrix_to_align_to_gravity(
        up_world.x(), up_world.y(), up_world.z(), /*fix_yaw=*/true);
    Matrix4dR gravity_rotation = Matrix4dR::Identity();
    gravity_rotation.block<3, 3>(0, 0) = level * first_column_rotation;

    const auto walls = algorithm::plumb_walls(frame_set, gravity_rotation);
    if (!walls) {
        return;
    }

    // correction * level maps world into the plumb frame.
    Matrix4dR world_to_plumb = Matrix4dR::Identity();
    world_to_plumb.block<3, 3>(0, 0) = walls->block<3, 3>(0, 0) * level;
    if (level_poses) {
        // Nothing is registered yet, so the plumb frame simply becomes the world frame.
        impl::transform_inplace(frame_set, world_to_plumb);
        set_plumbed_up(Eigen::Vector3d::UnitZ());
    } else {
        // Store the direction opposite gravity for acceleration correction.
        set_plumbed_up(-gravity_direction_from_pose(world_to_plumb));
    }
    logger().info(
        "Refined the gravity direction from wall geometry{}: world frame gravity "
        "direction ({:.4f}, {:.4f}, {:.4f})",
        level_poses ? " and levelled the first frame" : "", plumbed_up_world_frame_vector_->x(),
        plumbed_up_world_frame_vector_->y(), plumbed_up_world_frame_vector_->z());
}

void InertialIntegrationImuDeskewMethod::set_plumbed_up(const Eigen::Vector3d& up_world_frame_xyz) {
    plumbed_up_world_frame_vector_ = up_world_frame_xyz;
    if (!gravity_from_initial_pose_) {
        gravity_vector_world_frame_ = up_world_frame_xyz;
    }
}

void InertialIntegrationImuDeskewMethod::transform_imu_data_to_body_frame(
    Eigen::Ref<const Matrix4dR> imu_to_body_transform, Eigen::Ref<const MatrixX3dR> gyro_imu_frame,
    Eigen::Ref<const MatrixX3dR> accel_imu_frame, MatrixX3dR& gyro_body_frame,
    MatrixX3dR& accel_body_frame) {
    assert(gyro_imu_frame.rows() == accel_imu_frame.rows());
    // Extract rotation and translation
    const Eigen::Matrix3d rotation = imu_to_body_transform.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = imu_to_body_transform.block<3, 1>(0, 3);
    // Body origin relative to the IMU, in the IMU frame.
    const Eigen::Vector3d lever_arm = -rotation.transpose() * translation;
    Eigen::Index n_rows = gyro_imu_frame.rows();
    gyro_body_frame.resize(n_rows, 3);
    accel_body_frame.resize(n_rows, 3);
    // Transform Gyro: w_body = R * w_imu
    gyro_body_frame.noalias() = gyro_imu_frame * rotation.transpose();
    // Transform Acc: a_body = R * (a_imu + w_imu x (w_imu x r))
    for (Eigen::Index i = 0; i < n_rows; ++i) {
        Eigen::Vector3d gyro = gyro_imu_frame.row(i);
        Eigen::Vector3d acc = accel_imu_frame.row(i);
        // Centripetal acceleration in IMU frame: w x (w x r)
        Eigen::Vector3d centripetal = gyro.cross(gyro.cross(lever_arm));
        accel_body_frame.row(i) = rotation * (acc + centripetal);
    }
}

double InertialIntegrationImuDeskewMethod::angle_between_poses(Eigen::Ref<const Matrix4dR> pose_a,
                                                               Eigen::Ref<const Matrix4dR> pose_b) {
    double trace = (pose_a.block<3, 3>(0, 0) * pose_b.block<3, 3>(0, 0).transpose()).trace();
    auto clamp = [](double val, double low, double high) {
        return val < low ? low : (val > high ? high : val);
    };
    double cos_theta = clamp((trace - 1.0) / 2.0, -1.0, 1.0);
    return std::acos(cos_theta);
}

// estimate gyro and accel biases and gravity vector (world frame) when
// detecting the lidar sensors are rather static (low motion)
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void InertialIntegrationImuDeskewMethod::estimate_gravity_vector(FrameSet& frames) {
    if (ts_list_.size() < 2) {
        return;
    }

    constexpr double lowpass_fraction = 0.1;
    constexpr double max_linear_motion_m = 0.01;                   // m
    constexpr double max_angular_motion_rad = 0.1 * M_PI / 180.0;  // rad
    constexpr double max_expected_gyro_bias = 2.0 * M_PI / 180.0;  // rad/s
    constexpr double max_expected_accel_bias = 0.5;                // m/s^2
    constexpr int min_stationary_frames = 3;

    double pose_ang_delta = angle_between_poses(pose_list_.back(), pose_list_.front());
    double pose_lin_delta =
        (pose_list_.back().block<3, 1>(0, 3) - pose_list_.front().block<3, 1>(0, 3)).norm();

    // Gyro bias needs only a rotation-free streak; accel bias and gravity also need no translation.
    bool no_rotation = pose_ang_delta < max_angular_motion_rad;
    bool stationary = no_rotation && pose_lin_delta < max_linear_motion_m;
    no_rotation_frame_count_ = no_rotation ? no_rotation_frame_count_ + 1 : 0;
    stationary_frame_count_ = stationary ? stationary_frame_count_ + 1 : 0;
    if (no_rotation_frame_count_ < min_stationary_frames) {
        return;
    }

    for (size_t sidx : frames.valid_indices()) {
        if (!impl::is_lidar_imu_source(frames, sidx, imu_to_body_transform_.size())) {
            continue;
        }
        auto& frame = *frames[sidx];
        if (!frame.has_field(ChanField::IMU_STATUS)) {
            continue;
        }

        Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status = frame.field(ChanField::IMU_STATUS);
        std::vector<int> valid = impl::get_valid_columns<uint16_t>(imu_status);

        if (valid.size() != static_cast<unsigned>(imu_status.size())) {
            continue;
        }

        Eigen::Ref<const ArrayX3fR> gyro = frame.field(ChanField::IMU_GYRO);
        MatrixX3dR valid_gyro = gyro(valid, EIGEN_INDEXING_ALL).cast<double>();
        Eigen::Vector3d mean_gyro = valid_gyro.colwise().mean();
        if (mean_gyro.norm() < max_expected_gyro_bias) {
            if (gyro_bias_imu_frame_[sidx].isZero()) {
                gyro_bias_imu_frame_[sidx] = mean_gyro;
            } else {
                gyro_bias_imu_frame_[sidx] = (1 - lowpass_fraction) * gyro_bias_imu_frame_[sidx] +
                                             lowpass_fraction * mean_gyro;
            }
        }

        Eigen::Ref<const ArrayX3fR> acc = frame.field(ChanField::IMU_ACC);
        MatrixX3dR valid_acc = acc(valid, EIGEN_INDEXING_ALL).cast<double>();
        Eigen::Vector3d mean_acc = valid_acc.colwise().mean();
        if (stationary_frame_count_ >= min_stationary_frames &&
            std::abs(mean_acc.norm() - GRAVITY_MPERSEC2) < max_expected_accel_bias) {
            Eigen::Vector3d gravity_vector_imu_frame_xyz = mean_acc / mean_acc.norm();
            Eigen::Vector3d accel_bias = mean_acc - gravity_vector_imu_frame_xyz * GRAVITY_MPERSEC2;
            if (accel_bias_imu_frame_[sidx].isZero()) {
                accel_bias_imu_frame_[sidx] = accel_bias;
            } else {
                accel_bias_imu_frame_[sidx] = (1 - lowpass_fraction) * accel_bias_imu_frame_[sidx] +
                                              lowpass_fraction * accel_bias;
            }

            // Transform to sensor frame then world
            valid_gyro.rowwise() -= gyro_bias_imu_frame_[sidx].transpose();
            valid_acc.rowwise() -= accel_bias_imu_frame_[sidx].transpose();
            MatrixX3dR gyro_body_frame;
            MatrixX3dR accel_body_frame;
            transform_imu_data_to_body_frame(imu_to_body_transform_[sidx], valid_gyro, valid_acc,
                                             gyro_body_frame, accel_body_frame);

            Eigen::Ref<const Eigen::ArrayX<uint16_t>> meas_id =
                frame.field(ChanField::IMU_MEASUREMENT_ID);
            std::vector<Matrix4dR> world_poses;
            for (int idx : valid) {
                int id = meas_id(idx);
                world_poses.push_back(frame.get_column_pose(id));
            }
            MatrixX3dR imu_acc_world(accel_body_frame.rows(), 3);
            for (int row_idx = 0; row_idx < accel_body_frame.rows(); ++row_idx) {
                imu_acc_world.row(row_idx) = world_poses[row_idx].block<3, 3>(0, 0) *
                                             accel_body_frame.row(row_idx).transpose();
            }
            Eigen::Vector3d mean_imu_acc_world_frame = imu_acc_world.colwise().mean();
            Eigen::Vector3d gravity_vector = mean_imu_acc_world_frame.normalized();
            if (!gravity_vector_world_frame_) {
                gravity_vector_world_frame_ = gravity_vector;
            } else {
                gravity_vector_world_frame_ =
                    (1 - lowpass_fraction) * (*gravity_vector_world_frame_) +
                    lowpass_fraction * gravity_vector;
            }
            gravity_vector_world_frame_->normalize();
        }
    }
}

std::vector<Matrix4dR> InertialIntegrationImuDeskewMethod::calc_poses_with_motion_model(
    double last_timestamp, Eigen::Ref<const Matrix4dR> last_body_to_world_pose,
    const Eigen::Vector3d& initial_linear_velocity_world_frame,
    const std::vector<double>& timestamps, Eigen::Ref<const MatrixX3dR> angular_velocity_body_frame,
    Eigen::Ref<const MatrixX3dR> linear_accel_body_frame,
    Eigen::Ref<MatrixX3dR> nodes_velocities_world_frame) {
    size_t num_timestamps = timestamps.size();
    std::vector<Matrix4dR> body_to_world_poses(num_timestamps, Matrix4dR::Identity());
    Eigen::Vector3d current_linear_velocity_world_frame = initial_linear_velocity_world_frame;

    for (size_t i = 0; i < num_timestamps; ++i) {
        double delta_t = 0.0;
        Eigen::Matrix3d prev_world_orientation;
        Eigen::Vector3d prev_world_position;
        if (i == 0) {
            delta_t = timestamps[i] - last_timestamp;
            prev_world_orientation = last_body_to_world_pose.block<3, 3>(0, 0);
            prev_world_position = last_body_to_world_pose.block<3, 1>(0, 3);
        } else {
            delta_t = timestamps[i] - timestamps[i - 1];
            prev_world_orientation = body_to_world_poses[i - 1].block<3, 3>(0, 0);
            prev_world_position = body_to_world_poses[i - 1].block<3, 1>(0, 3);
        }

        // 2. Update the orientation
        Eigen::Vector3d w = angular_velocity_body_frame.row(static_cast<Eigen::Index>(i));
        double angle = w.norm() * delta_t;
        Eigen::Matrix3d delta_rotation;
        if (angle < 1e-12) {
            delta_rotation.setIdentity();
        } else {
            Eigen::Vector3d axis = w.normalized();
            delta_rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        }
        Eigen::Matrix3d new_world_orientation = prev_world_orientation * delta_rotation;

        // 3. Gravity-free world acceleration (if accelerations are provided)
        Eigen::Vector3d world_acc_no_g = Eigen::Vector3d::Zero();
        if (gravity_vector_world_frame_) {
            // Use the average of the old and new orientation to transform the
            // velocity According to the internet, this is fine for small angles
            // and is more accurate
            Eigen::Vector3d acc_local = linear_accel_body_frame.row(static_cast<Eigen::Index>(i));
            Eigen::Vector3d world_acc =
                (prev_world_orientation + new_world_orientation) * 0.5 * acc_local;
            world_acc_no_g = world_acc - (*gravity_vector_world_frame_) * GRAVITY_MPERSEC2;
        }

        // 4. Constant acceleration over the interval: exact for the sample that drives it. Must
        // match calc_corrected_poses_with_motion_model so the velocity solve stays exact.
        Eigen::Vector3d new_world_position = prev_world_position +
                                             current_linear_velocity_world_frame * delta_t +
                                             0.5 * world_acc_no_g * delta_t * delta_t;
        current_linear_velocity_world_frame += world_acc_no_g * delta_t;
        nodes_velocities_world_frame.row(static_cast<Eigen::Index>(i)) =
            current_linear_velocity_world_frame;

        // 5. Assemble and store the new pose
        body_to_world_poses[i].block<3, 3>(0, 0) = new_world_orientation;
        body_to_world_poses[i].block<3, 1>(0, 3) = new_world_position;
    }

    return body_to_world_poses;
}

std::vector<Matrix4dR> InertialIntegrationImuDeskewMethod::calc_corrected_poses_with_motion_model(
    double last_timestamp, Eigen::Ref<const Matrix4dR> last_body_to_world_pose, size_t target_idx,
    Eigen::Ref<const Matrix4dR> target_pose,
    const Eigen::Vector3d& initial_linear_velocity_world_frame,
    const std::vector<double>& timestamps, Eigen::Ref<const MatrixX3dR> angular_velocity_body_frame,
    Eigen::Ref<const MatrixX3dR> linear_accel_body_frame,
    Eigen::Ref<Eigen::Vector3d> ending_velocity_world_frame,
    Eigen::Ref<MatrixX3dR> nodes_velocities_world_frame) const {
    const size_t num_timestamps = timestamps.size();
    std::vector<Matrix4dR> body_to_world_poses(num_timestamps, Matrix4dR::Identity());
    if (num_timestamps == 0) {
        ending_velocity_world_frame = initial_linear_velocity_world_frame;
        return body_to_world_poses;
    }

    auto integrate_orientation = [](const Eigen::Matrix3d& prev_orientation,
                                    const Eigen::Vector3d& angular_velocity,
                                    double delta_t) -> Eigen::Matrix3d {
        const double angle = angular_velocity.norm() * delta_t;
        if (angle < 1e-12) {
            return prev_orientation;
        }
        const Eigen::Vector3d axis = angular_velocity.normalized();
        return prev_orientation * Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    };

    std::vector<Eigen::Matrix3d> predicted_orientations(target_idx + 1);
    Eigen::Matrix3d prev_orientation = last_body_to_world_pose.block<3, 3>(0, 0);
    double prev_ts = last_timestamp;
    for (size_t i = 0; i <= target_idx; ++i) {
        const double delta_t = timestamps[i] - prev_ts;
        const Eigen::Vector3d angular_velocity =
            angular_velocity_body_frame.row(static_cast<Eigen::Index>(i));
        predicted_orientations[i] =
            integrate_orientation(prev_orientation, angular_velocity, delta_t);
        prev_orientation = predicted_orientations[i];
        prev_ts = timestamps[i];
    }

    const Eigen::Matrix3d target_orientation = target_pose.block<3, 3>(0, 0);
    Eigen::Matrix3d rotation_error =
        target_orientation * predicted_orientations[target_idx].transpose();
    Eigen::Quaterniond error_quat(rotation_error);
    error_quat.normalize();
    if (error_quat.w() < 0.0) {
        error_quat.coeffs() *= -1.0;
    }

    std::vector<Eigen::Matrix3d> corrected_orientations(num_timestamps);
    const double target_duration = timestamps[target_idx] - last_timestamp;
    const Eigen::Quaterniond identity_quat = Eigen::Quaterniond::Identity();
    for (size_t i = 0; i <= target_idx; ++i) {
        const double alpha =
            target_duration > 0.0 ? (timestamps[i] - last_timestamp) / target_duration : 1.0;
        Eigen::Quaterniond partial_error =
            identity_quat.slerp(std::max(0.0, std::min(1.0, alpha)), error_quat);
        corrected_orientations[i] = partial_error.toRotationMatrix() * predicted_orientations[i];
    }
    corrected_orientations[target_idx] = target_orientation;

    prev_orientation = target_orientation;
    prev_ts = timestamps[target_idx];
    for (size_t i = target_idx + 1; i < num_timestamps; ++i) {
        const double delta_t = timestamps[i] - prev_ts;
        const Eigen::Vector3d angular_velocity =
            angular_velocity_body_frame.row(static_cast<Eigen::Index>(i));
        corrected_orientations[i] =
            integrate_orientation(prev_orientation, angular_velocity, delta_t);
        prev_orientation = corrected_orientations[i];
        prev_ts = timestamps[i];
    }

    Eigen::Vector3d current_linear_velocity_world_frame = initial_linear_velocity_world_frame;
    Eigen::Vector3d prev_world_position = last_body_to_world_pose.block<3, 1>(0, 3);
    prev_orientation = last_body_to_world_pose.block<3, 3>(0, 0);
    prev_ts = last_timestamp;
    for (size_t i = 0; i < num_timestamps; ++i) {
        const double delta_t = timestamps[i] - prev_ts;
        const Eigen::Matrix3d& new_world_orientation = corrected_orientations[i];

        Eigen::Vector3d world_acc_no_g = Eigen::Vector3d::Zero();
        if (gravity_vector_world_frame_) {
            const Eigen::Vector3d acc_local =
                linear_accel_body_frame.row(static_cast<Eigen::Index>(i));
            const Eigen::Vector3d world_acc =
                (prev_orientation + new_world_orientation) * 0.5 * acc_local;
            world_acc_no_g = world_acc - (*gravity_vector_world_frame_) * GRAVITY_MPERSEC2;
        }

        // Constant acceleration over the interval, as in calc_poses_with_motion_model. Position
        // stays linear in the initial velocity with coefficient sum(delta_t), which is what makes
        // solve_initial_velocity exact.
        const Eigen::Vector3d new_world_position = prev_world_position +
                                                   current_linear_velocity_world_frame * delta_t +
                                                   0.5 * world_acc_no_g * delta_t * delta_t;
        current_linear_velocity_world_frame += world_acc_no_g * delta_t;
        nodes_velocities_world_frame.row(static_cast<Eigen::Index>(i)) =
            current_linear_velocity_world_frame;
        body_to_world_poses[i].block<3, 3>(0, 0) = new_world_orientation;
        body_to_world_poses[i].block<3, 1>(0, 3) = new_world_position;

        prev_orientation = new_world_orientation;
        prev_world_position = new_world_position;
        prev_ts = timestamps[i];
    }

    ending_velocity_world_frame = current_linear_velocity_world_frame;
    return body_to_world_poses;
}

nonstd::optional<InertialIntegrationImuDeskewMethod::VelocitySolveResult>
InertialIntegrationImuDeskewMethod::solve_initial_velocity(
    double last_timestamp, Eigen::Ref<const Matrix4dR> last_body_to_world_pose,
    double target_timestamp, Eigen::Ref<const Matrix4dR> target_pose,
    const std::vector<double>& timestamps, Eigen::Ref<const MatrixX3dR> angular_velocity_body_frame,
    Eigen::Ref<const MatrixX3dR> linear_accel_body_frame) const {
    const double delta_t = target_timestamp - last_timestamp;
    if (delta_t <= 0.0 || timestamps.empty()) {
        return nonstd::nullopt;
    }

    std::vector<double> nodes_ts = timestamps;
    MatrixX3dR nodes_gyro = angular_velocity_body_frame;
    MatrixX3dR nodes_accel = linear_accel_body_frame;
    const size_t target_idx =
        insert_imu_node_at(nodes_ts, nodes_gyro, nodes_accel, target_timestamp);

    Eigen::Vector3d accel_only_ending_velocity;
    MatrixX3dR accel_only_velocities(nodes_gyro.rows(), 3);
    std::vector<Matrix4dR> accel_only_poses = calc_corrected_poses_with_motion_model(
        last_timestamp, last_body_to_world_pose, target_idx, target_pose, Eigen::Vector3d::Zero(),
        nodes_ts, nodes_gyro, nodes_accel, accel_only_ending_velocity, accel_only_velocities);

    const Eigen::Vector3d anchor_position = last_body_to_world_pose.block<3, 1>(0, 3);
    const Eigen::Vector3d target_position = target_pose.block<3, 1>(0, 3);
    const Eigen::Vector3d accel_displacement =
        accel_only_poses[target_idx].block<3, 1>(0, 3) - anchor_position;

    VelocitySolveResult result;
    result.initial_velocity_world_frame =
        (target_position - anchor_position - accel_displacement) / delta_t;
    result.ending_velocity_world_frame =
        result.initial_velocity_world_frame + accel_only_ending_velocity;
    for (Eigen::Index row = 0; row < accel_only_velocities.rows(); ++row) {
        result.nodes_velocities_world_frame.emplace_back(
            result.initial_velocity_world_frame + accel_only_velocities.row(row).transpose());
    }
    result.nodes_ts = std::move(nodes_ts);
    return result;
}

void InertialIntegrationImuDeskewMethod::interpolate_imu_poses_to_frame_set(
    FrameSet& frame_set, const std::vector<double>& imu_timestamps,
    const std::vector<Matrix4dR>& imu_poses) {
    for (size_t sidx : frame_set.valid_indices()) {
        auto lidar = std::dynamic_pointer_cast<core::LidarFrame>(frame_set[sidx]);
        if (!lidar) {
            continue;
        }
        auto& frame = *lidar;
        Eigen::ArrayX<uint64_t> frame_ts_copy = frame.timestamp();
        Eigen::ArrayX<uint32_t> frame_status_copy = frame.status();

        // Lidar columns may be missing while colocated IMU measurements exist.
        // Use the IMU measurement id to preserve those poses for downstream
        // state estimation.
        if (frame.has_field(ChanField::IMU_STATUS)) {
            Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status =
                frame.field(ChanField::IMU_STATUS);
            Eigen::Ref<const Eigen::ArrayX<uint64_t>> imu_ts =
                frame.field(ChanField::IMU_TIMESTAMP);
            Eigen::Ref<const Eigen::ArrayX<uint16_t>> meas_id =
                frame.field(ChanField::IMU_MEASUREMENT_ID);
            std::vector<int> imu_valid = impl::get_valid_columns<uint16_t>(imu_status);
            for (int imu_col : imu_valid) {
                int frame_col = meas_id[imu_col];
                frame_ts_copy[frame_col] = imu_ts[imu_col];
                frame_status_copy[frame_col] = 1;
            }
        }

        std::vector<int> frame_valid = impl::get_valid_columns<uint32_t>(frame_status_copy);
        std::vector<double> frame_ts_vec = impl::get_valid_timestamps(frame_ts_copy, frame_valid);
        auto interp = interp_pose(frame_ts_vec, imu_timestamps, imu_poses);
        for (size_t k = 0; k < frame_valid.size(); ++k) {
            frame.set_column_pose(frame_valid[k], interp[k]);
        }
    }
}

std::unique_ptr<DeskewMethod> DeskewMethodFactory::create(
    const std::string& method, const std::vector<std::shared_ptr<SensorInfo>>& sensor_infos,
    const nonstd::optional<Matrix4dR>& initial_pose, const std::string& plumb_method,
    bool gravity_from_initial_pose) {
    // check if any of the lidar sensors is on FW 3.2 and has IMU data
    bool has_imu_data = false;
    for (const auto& info : sensor_infos) {
        auto lidar = std::dynamic_pointer_cast<LidarInfo>(info);
        if (!lidar) {
            continue;
        }
        uint32_t imu_measurements_per_frame =
            lidar->format.imu_measurements_per_packet * lidar->format.imu_packets_per_frame;
        if (imu_measurements_per_frame > 0) {
            has_imu_data = true;
            break;
        }
    }

    if (method == "none") {
        logger().info("No deskewing will be applied");
        return nullptr;
    } else if (method == "constant_velocity") {
        logger().info("Using ConstantVelocityDeskewMethod");
        return std::make_unique<ConstantVelocityDeskewMethod>(sensor_infos, initial_pose);
    } else if (method == "imu_deskew") {
        logger().info("Using InertialIntegrationImuDeskewMethod");
        return std::make_unique<InertialIntegrationImuDeskewMethod>(
            sensor_infos, initial_pose, plumb_method, gravity_from_initial_pose);
    } else if (method == "auto") {
        if (!has_imu_data) {
            logger().info(
                "Synchronous IMU data not available (requires FW 3.2+ and "
                "ACCEL32_GYRO32_NMEA imu profile),"
                " falling back to ConstantVelocityDeskewMethod.\n"
                " Suppress this warning by adding '--deskew-method "
                "constant_velocity' to the 'slam' or 'localize' command.");
            return std::make_unique<ConstantVelocityDeskewMethod>(sensor_infos, initial_pose);
        } else {
            logger().info("Using InertialIntegrationImuDeskewMethod");
            return std::make_unique<InertialIntegrationImuDeskewMethod>(
                sensor_infos, initial_pose, plumb_method, gravity_from_initial_pose);
        }
    } else {
        throw std::invalid_argument("Invalid deskew_method: " + method);
    }
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

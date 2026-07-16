#include "ouster/mapping/deskew_method.h"

#include <ouster/core/impl/logging.h>
#include <ouster/core/pose_util.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

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

}  // namespace impl

// Matrix4dR is a fixed-size Eigen type, which the codebase passes by const ref
// rather than by value (passing fixed-size Eigen objects by value has a known
// stack-alignment caveat), so suppress the pass-by-value suggestion here.
// NOLINTBEGIN(modernize-pass-by-value)
DeskewMethod::DeskewMethod(const std::vector<std::shared_ptr<SensorInfo>>& infos,
                           const Matrix4dR& initial_pose)
    : initial_pose_(initial_pose) {
    if (infos.empty()) {
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
        impl::init_valid_column_poses(frame_set, initial_pose_);
        return;
    }

    for (size_t idx : frame_set.valid_indices()) {
        auto& frame = *frame_set[idx];
        impl::interp_pose(frame, ts_list_.front(), pose_list_.front(), ts_list_.back(),
                          pose_list_.back());
    }
}

constexpr double InertialIntegrationImuDeskewMethod::GRAVITY_MPERSEC2;

InertialIntegrationImuDeskewMethod::InertialIntegrationImuDeskewMethod(
    const std::vector<std::shared_ptr<SensorInfo>>& infos, const Matrix4dR& initial_pose)
    : DeskewMethod(infos, initial_pose) {
    accel_bias_imu_frame_.resize(infos.size(), Eigen::Vector3d::Zero());
    gyro_bias_imu_frame_.resize(infos.size(), Eigen::Vector3d::Zero());
    imu_to_body_transform_.resize(infos.size());
    for (size_t i = 0; i < infos.size(); ++i) {
        const auto& info = *infos[i];
        Matrix4dR imu_transform = info.imu_to_sensor_transform;
        imu_transform.block<3, 1>(0, 3) *= 1e-3;  // mm -> m
        imu_to_body_transform_[i] = info.sensor_to_body * imu_transform;
    }
}

bool InertialIntegrationImuDeskewMethod::pick_last_valid_imu_pose(const FrameSet& frame_set,
                                                                  double& last_ts,
                                                                  Matrix4dR& last_pose) {
    bool imu_data_found = false;
    for (const auto& frame : frame_set.valid_frames()) {
        if (!frame->has_field(ChanField::IMU_STATUS)) {
            continue;
        }

        Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status = frame->field(ChanField::IMU_STATUS);
        std::vector<int> imu_valid = impl::get_valid_columns<uint16_t>(imu_status);
        if (imu_valid.empty()) {
            continue;
        }

        int last_valid_imu_idx = imu_valid.back();
        Eigen::Ref<const Eigen::ArrayX<uint64_t>> imu_ts_arr =
            frame->field(ChanField::IMU_TIMESTAMP);
        double candidate_ts = static_cast<double>(imu_ts_arr[last_valid_imu_idx]) * 1e-9;
        if (!imu_data_found || candidate_ts > last_ts) {
            last_ts = candidate_ts;
            Eigen::Ref<const Eigen::ArrayX<uint16_t>> meas_id =
                frame->field(ChanField::IMU_MEASUREMENT_ID);
            uint16_t last_pose_col = meas_id(last_valid_imu_idx);
            last_pose = frame->get_column_pose(last_pose_col);
        }
        imu_data_found = true;
    }
    return imu_data_found;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void InertialIntegrationImuDeskewMethod::update(FrameSet& frame_set) {
    size_t total_possible_imu_data = 0;
    std::vector<std::vector<int>> imu_valid_per_sensor(frame_set.size());
    for (size_t sidx : frame_set.valid_indices()) {
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
    std::vector<double> combined_ts;
    combined_ts.reserve(total_possible_imu_data);
    std::vector<Eigen::Vector3d> combined_gyro_body_frame;
    combined_gyro_body_frame.reserve(total_possible_imu_data);
    std::vector<Eigen::Vector3d> combined_accel_body_frame;
    combined_accel_body_frame.reserve(total_possible_imu_data);
    std::vector<size_t> cursors;
    std::vector<size_t> ends;
    cursors.reserve(frame_set.size());
    ends.reserve(frame_set.size());

    for (size_t sidx : frame_set.valid_indices()) {
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
        const size_t range_begin = combined_ts.size();
        for (size_t k = 0; k < imu_valid.size(); ++k) {
            combined_ts.push_back(static_cast<double>(imu_timestamps(imu_valid[k])) * 1e-9);
            combined_gyro_body_frame.emplace_back(
                gyro_body_frame.row(static_cast<Eigen::Index>(k)));
            combined_accel_body_frame.emplace_back(
                accel_body_frame.row(static_cast<Eigen::Index>(k)));
        }
        if (combined_ts.size() > range_begin) {
            cursors.push_back(range_begin);
            ends.push_back(combined_ts.size());
        }
    }

    if (combined_ts.empty()) {
        // No IMU data: fall back to pose interpolation when SLAM poses are
        // available, otherwise write identity to avoid downstream errors.
        if (ts_list_.size() >= 2) {
            for (auto& frame : frame_set.valid_frames()) {
                impl::interp_pose(*frame, ts_list_.front(), pose_list_.front(), ts_list_.back(),
                                  pose_list_.back());
            }
        } else {
            impl::init_valid_column_poses(frame_set, initial_pose_);
        }
        last_frame_set_ = frame_set;
        return;
    }

    // k-way merge: each per-sensor sub-range of combined_ts is already
    // monotonically increasing. Walk all cursors simultaneously, always
    // advancing the one with the smallest current timestamp.
    // Duplicate timestamps (across or within sensors) are dropped, matching
    // the previous std::unique behavior.
    const size_t k_sensors = cursors.size();
    std::vector<size_t> indices;
    indices.reserve(combined_ts.size());
    while (true) {
        // k_sensors is used as a sentinel meaning "no valid cursor found yet".
        size_t best_k = k_sensors;
        for (size_t k = 0; k < k_sensors; ++k) {
            if (cursors[k] >= ends[k]) {
                continue;
            }
            if (best_k == k_sensors || combined_ts[cursors[k]] < combined_ts[cursors[best_k]]) {
                best_k = k;
            }
        }
        // All cursors exhausted — merge complete.
        if (best_k == k_sensors) {
            break;
        }
        // Drop duplicate timestamps across sensors (matches previous
        // std::unique behavior on the concatenated, unsorted buffer).
        if (indices.empty() || combined_ts[cursors[best_k]] != combined_ts[indices.back()]) {
            indices.push_back(cursors[best_k]);
        }
        ++cursors[best_k];
    }
    // assemble the merged, deduplicated imu measurements in timestamp order
    std::vector<double> sorted_ts(indices.size());
    MatrixX3dR sorted_gyro(indices.size(), 3);
    MatrixX3dR sorted_accel(indices.size(), 3);

    for (size_t i = 0; i < indices.size(); ++i) {
        sorted_ts[i] = combined_ts[indices[i]];
        sorted_gyro.row(static_cast<Eigen::Index>(i)) = combined_gyro_body_frame[indices[i]];
        sorted_accel.row(static_cast<Eigen::Index>(i)) = combined_accel_body_frame[indices[i]];
    }

    if (rolling_imu_timestamps_.empty()) {
        rolling_imu_timestamps_ = sorted_ts;
        rolling_gyro_body_frame_ = sorted_gyro;
        rolling_accel_body_frame_ = sorted_accel;
    } else {
        auto append_start =
            std::upper_bound(sorted_ts.begin(), sorted_ts.end(), rolling_imu_timestamps_.back());
        const Eigen::Index append_count =
            static_cast<Eigen::Index>(std::distance(append_start, sorted_ts.end()));
        if (append_count > 0) {
            const Eigen::Index old_rows = rolling_gyro_body_frame_.rows();
            const Eigen::Index new_rows = old_rows + append_count;
            const Eigen::Index append_row =
                static_cast<Eigen::Index>(std::distance(sorted_ts.begin(), append_start));
            rolling_gyro_body_frame_.conservativeResize(new_rows, Eigen::NoChange);
            rolling_accel_body_frame_.conservativeResize(new_rows, Eigen::NoChange);
            rolling_gyro_body_frame_.bottomRows(append_count) =
                sorted_gyro.middleRows(append_row, append_count);
            rolling_accel_body_frame_.bottomRows(append_count) =
                sorted_accel.middleRows(append_row, append_count);
            rolling_imu_timestamps_.insert(rolling_imu_timestamps_.end(), append_start,
                                           sorted_ts.end());
        }
    }

    // Prefer the previous frame's IMU pose as the integration anchor. If that
    // is unavailable, fall back to SLAM history before using first-frame
    // identity initialization.
    double last_ts = sorted_ts.front();
    Matrix4dR last_pose = initial_pose_;
    if (imu_integration_anchor_ts_) {
        last_ts = *imu_integration_anchor_ts_;
        last_pose = *imu_integration_anchor_pose_;
    } else if (!ts_list_.empty()) {
        last_ts = ts_list_.back();
        last_pose = pose_list_.back();
    }
    // Initial linear velocity. Use the finite-difference between the two most
    // recent SLAM poses once they are available. Before that, zero is a
    // reasonable approximation: rotational deskew from gyro is accurate
    // regardless, and approximate translational deskew beats none at all.
    Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
    if (last_linear_velocity_world_frame_) {
        initial_velocity = *last_linear_velocity_world_frame_;
    } else if (ts_list_.size() >= 2) {
        initial_velocity =
            (pose_list_.back().block<3, 1>(0, 3) - pose_list_.front().block<3, 1>(0, 3)) /
            (ts_list_.back() - ts_list_.front());
    }

    std::vector<Matrix4dR> imu_poses_world_frame = calc_poses_with_motion_model(
        last_ts, last_pose, initial_velocity, sorted_ts, sorted_gyro, sorted_accel);

    interpolate_imu_poses_to_frame_set(frame_set, sorted_ts, imu_poses_world_frame);

    last_frame_set_ = frame_set;
}

void InertialIntegrationImuDeskewMethod::finalize_after_registration(
    FrameSet& frame_set, int64_t anchor_timestamp_ns, const Matrix4dR& corrected_anchor_pose) {
    const double target_ts = static_cast<double>(anchor_timestamp_ns) * 1e-9;

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
    prune_rolling_imu_before(target_ts);
    finish_with_current_poses();
}

void InertialIntegrationImuDeskewMethod::set_last_pose(int64_t timestamp_ns,
                                                       const Matrix4dR& pose) {
    // Store the last valid IMU-aligned pose from the previous frame set before
    // active time correction resets timestamps. This keeps the next integration
    // anchor in the corrected timestamp domain used by update().
    double last_ts = 0.0;
    Matrix4dR last_pose = Matrix4dR::Identity();
    if (pick_last_valid_imu_pose(last_frame_set_, last_ts, last_pose)) {
        imu_integration_anchor_ts_ = last_ts;
        imu_integration_anchor_pose_ = last_pose;
    }
    DeskewMethod::set_last_pose(timestamp_ns, pose);
    estimate_gravity_vector(last_frame_set_);
}

void InertialIntegrationImuDeskewMethod::transform_imu_data_to_body_frame(
    Eigen::Ref<const Matrix4dR> imu_to_body_transform, Eigen::Ref<const MatrixX3dR> gyro_imu_frame,
    Eigen::Ref<const MatrixX3dR> accel_imu_frame, MatrixX3dR& gyro_body_frame,
    MatrixX3dR& accel_body_frame) {
    assert(gyro_imu_frame.rows() == accel_imu_frame.rows());
    // Extract rotation and translation
    const Eigen::Matrix3d rotation = imu_to_body_transform.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = imu_to_body_transform.block<3, 1>(0, 3);
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
        Eigen::Vector3d centripetal = gyro.cross(gyro.cross(translation));
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
    constexpr double max_angular_motion_rad = M_PI / 180.0;        // rad
    constexpr double max_expected_gyro_bias = 2.0 * M_PI / 180.0;  // rad/s
    constexpr double max_expected_accel_bias = 0.5;                // m/s^2

    double pose_ang_delta = angle_between_poses(pose_list_.back(), pose_list_.front());
    double pose_lin_delta =
        (pose_list_.back().block<3, 1>(0, 3) - pose_list_.front().block<3, 1>(0, 3)).norm();

    for (size_t sidx : frames.valid_indices()) {
        auto& frame = *frames[sidx];
        if (!frame.has_field(ChanField::IMU_STATUS)) {
            continue;
        }

        Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status = frame.field(ChanField::IMU_STATUS);
        std::vector<int> valid = impl::get_valid_columns<uint16_t>(imu_status);

        if (valid.size() != static_cast<unsigned>(imu_status.size())) {
            continue;
        }

        if (pose_ang_delta >= max_angular_motion_rad) {
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
        if (pose_lin_delta < max_linear_motion_m &&
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
            if (!gravity_vector_world_frame_xyz_) {
                gravity_vector_world_frame_xyz_ = gravity_vector;
            } else {
                gravity_vector_world_frame_xyz_ =
                    (1 - lowpass_fraction) * (*gravity_vector_world_frame_xyz_) +
                    lowpass_fraction * gravity_vector;
            }
            gravity_vector_world_frame_xyz_->normalize();
        }
    }
}

std::vector<Matrix4dR> InertialIntegrationImuDeskewMethod::calc_poses_with_motion_model(
    double last_timestamp, Eigen::Ref<const Matrix4dR> last_body_to_world_pose,
    const Eigen::Vector3d& initial_linear_velocity_world_frame,
    const std::vector<double>& timestamps, Eigen::Ref<const MatrixX3dR> angular_velocity_body_frame,
    Eigen::Ref<const MatrixX3dR> linear_accel_body_frame) {
    size_t num_timestamps = timestamps.size();
    std::vector<Matrix4dR> body_to_world_poses(num_timestamps, Matrix4dR::Identity());
    Eigen::Vector3d current_linear_velocity_world_frame = initial_linear_velocity_world_frame;

    // TODO[UN]: we need to skip imu measurements for timestamps earlier than
    // last_timestamp and feed in poses from last frame set or alter the logic
    // of pick_last_valid_imu_pose to get the last imu pose before current
    // timestamp[0]. Alternatively, in cases of an imu measurements overlap one
    // could simply average the results between the overlapping measurements.
    // I will defer the choice to the next iteration.
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

        // 3. Update local velocity (if accelerations are provided)
        if (gravity_vector_world_frame_xyz_) {
            // Use the average of the old and new orientation to transform the
            // velocity According to the internet, this is fine for small angles
            // and is more accurate
            Eigen::Vector3d acc_local = linear_accel_body_frame.row(static_cast<Eigen::Index>(i));
            Eigen::Vector3d world_acc =
                (prev_world_orientation + new_world_orientation) * 0.5 * acc_local;
            Eigen::Vector3d world_acc_no_g =
                world_acc - (*gravity_vector_world_frame_xyz_) * GRAVITY_MPERSEC2;
            current_linear_velocity_world_frame += world_acc_no_g * delta_t;
        }

        // 4. Calculate new position
        Eigen::Vector3d new_world_position =
            prev_world_position + current_linear_velocity_world_frame * delta_t;

        // 5. Assemble and store the new pose
        body_to_world_poses[i].block<3, 3>(0, 0) = new_world_orientation;
        body_to_world_poses[i].block<3, 1>(0, 3) = new_world_position;
    }

    return body_to_world_poses;
}

std::vector<Matrix4dR> InertialIntegrationImuDeskewMethod::calc_corrected_poses_with_motion_model(
    double last_timestamp, Eigen::Ref<const Matrix4dR> last_body_to_world_pose,
    double target_timestamp, Eigen::Ref<const Matrix4dR> target_pose,
    const Eigen::Vector3d& initial_linear_velocity_world_frame,
    const std::vector<double>& timestamps, Eigen::Ref<const MatrixX3dR> angular_velocity_body_frame,
    Eigen::Ref<const MatrixX3dR> linear_accel_body_frame,
    Eigen::Vector3d* ending_velocity_world_frame) const {
    const size_t num_timestamps = timestamps.size();
    std::vector<Matrix4dR> body_to_world_poses(num_timestamps, Matrix4dR::Identity());
    if (num_timestamps == 0) {
        if (ending_velocity_world_frame != nullptr) {
            *ending_velocity_world_frame = initial_linear_velocity_world_frame;
        }
        return body_to_world_poses;
    }

    auto target_it = std::lower_bound(timestamps.begin(), timestamps.end(), target_timestamp);
    size_t target_idx = static_cast<size_t>(std::distance(timestamps.begin(), target_it));
    if (target_it == timestamps.end() || std::abs(*target_it - target_timestamp) > 1e-9) {
        target_idx = num_timestamps - 1;
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
    const double target_duration = target_timestamp - last_timestamp;
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

        if (gravity_vector_world_frame_xyz_) {
            const Eigen::Vector3d acc_local =
                linear_accel_body_frame.row(static_cast<Eigen::Index>(i));
            const Eigen::Vector3d world_acc =
                (prev_orientation + new_world_orientation) * 0.5 * acc_local;
            const Eigen::Vector3d world_acc_no_g =
                world_acc - (*gravity_vector_world_frame_xyz_) * GRAVITY_MPERSEC2;
            current_linear_velocity_world_frame += world_acc_no_g * delta_t;
        }

        const Eigen::Vector3d new_world_position =
            prev_world_position + current_linear_velocity_world_frame * delta_t;
        body_to_world_poses[i].block<3, 3>(0, 0) = new_world_orientation;
        body_to_world_poses[i].block<3, 1>(0, 3) = new_world_position;

        prev_orientation = new_world_orientation;
        prev_world_position = new_world_position;
        prev_ts = timestamps[i];
    }

    if (ending_velocity_world_frame != nullptr) {
        *ending_velocity_world_frame = current_linear_velocity_world_frame;
    }
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

    auto upper_it = std::lower_bound(timestamps.begin(), timestamps.end(), target_timestamp);
    auto target_it = upper_it;
    if (upper_it == timestamps.end()) {
        target_it = std::prev(upper_it);
    } else if (upper_it != timestamps.begin()) {
        auto prev_it = std::prev(upper_it);
        if (std::abs(*prev_it - target_timestamp) < std::abs(*upper_it - target_timestamp)) {
            target_it = prev_it;
        }
    }
    if (target_it == timestamps.end()) {
        return nonstd::nullopt;
    }
    const size_t target_idx = static_cast<size_t>(std::distance(timestamps.begin(), target_it));

    Eigen::Vector3d accel_only_ending_velocity;
    std::vector<Matrix4dR> accel_only_poses = calc_corrected_poses_with_motion_model(
        last_timestamp, last_body_to_world_pose, target_timestamp, target_pose,
        Eigen::Vector3d::Zero(), timestamps, angular_velocity_body_frame, linear_accel_body_frame,
        &accel_only_ending_velocity);

    const Eigen::Vector3d anchor_position = last_body_to_world_pose.block<3, 1>(0, 3);
    const Eigen::Vector3d target_position = target_pose.block<3, 1>(0, 3);
    const Eigen::Vector3d accel_displacement =
        accel_only_poses[target_idx].block<3, 1>(0, 3) - anchor_position;

    VelocitySolveResult result;
    result.initial_velocity_world_frame =
        (target_position - anchor_position - accel_displacement) / delta_t;
    result.ending_velocity_world_frame =
        result.initial_velocity_world_frame + accel_only_ending_velocity;
    return result;
}

void InertialIntegrationImuDeskewMethod::interpolate_imu_poses_to_frame_set(
    FrameSet& frame_set, const std::vector<double>& imu_timestamps,
    const std::vector<Matrix4dR>& imu_poses) {
    for (size_t sidx : frame_set.valid_indices()) {
        auto& frame = *frame_set[sidx];
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
    const std::string& method, const std::vector<std::shared_ptr<SensorInfo>>& infos,
    const Matrix4dR& initial_pose) {
    // check if any of the sensors is on FW 3.2 and has IMU data
    bool has_imu_data = false;
    for (const auto& info : infos) {
        uint32_t imu_measurements_per_frame =
            info->format.imu_measurements_per_packet * info->format.imu_packets_per_frame;
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
        return std::make_unique<ConstantVelocityDeskewMethod>(infos, initial_pose);
    } else if (method == "imu_deskew") {
        logger().info("Using InertialIntegrationImuDeskewMethod");
        return std::make_unique<InertialIntegrationImuDeskewMethod>(infos, initial_pose);
    } else if (method == "auto") {
        if (!has_imu_data) {
            logger().info(
                "Synchronous IMU data not available (requires FW 3.2+ and "
                "ACCEL32_GYRO32_NMEA imu profile),"
                " falling back to ConstantVelocityDeskewMethod.\n"
                " Suppress this warning by adding '--deskew-method "
                "constant_velocity' to the 'slam' or 'localize' command.");
            return std::make_unique<ConstantVelocityDeskewMethod>(infos, initial_pose);
        } else {
            logger().info("Using InertialIntegrationImuDeskewMethod");
            return std::make_unique<InertialIntegrationImuDeskewMethod>(infos, initial_pose);
        }
    } else {
        throw std::invalid_argument("Invalid deskew_method: " + method);
    }
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

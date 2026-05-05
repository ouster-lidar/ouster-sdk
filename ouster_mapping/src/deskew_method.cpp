#include "ouster/deskew_method.h"

#include <ouster/impl/logging.h>
#include <ouster/pose_util.h>

#include <numeric>

#include "slam_util.h"

using namespace ouster::sdk::core;

#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
const auto EIGEN_INDEXING_ALL = Eigen::indexing::all;
#else
const auto EIGEN_INDEXING_ALL = Eigen::all;
#endif

namespace ouster {
namespace sdk {
namespace mapping {

DeskewMethod::DeskewMethod(
    const std::vector<std::shared_ptr<SensorInfo>>& infos) {
    if (infos.empty()) {
        throw std::invalid_argument("No sensor info provided for slam");
    }
}

void ConstantVelocityDeskewMethod::update(LidarScanSet& scan_set) {
    // Not enough poses to perform constant velocity deskewing. Need at least
    // two poses to start applying deskewing.
    if (ts_list_.size() < 2) {
        return;
    }

    for (size_t idx : scan_set.valid_indices()) {
        auto& scan = *scan_set[idx];
        impl::interp_pose(scan, ts_list_.front(), pose_list_.front(),
                          ts_list_.back(), pose_list_.back());
    }
}

constexpr double InertialIntegrationImuDeskewMethod::GRAVITY_MPERSEC2;

InertialIntegrationImuDeskewMethod::InertialIntegrationImuDeskewMethod(
    const std::vector<std::shared_ptr<SensorInfo>>& infos)
    : DeskewMethod(infos) {
    accel_bias_imu_frame_.resize(infos.size(), Eigen::Vector3d::Zero());
    gyro_bias_imu_frame_.resize(infos.size(), Eigen::Vector3d::Zero());
    imu_to_body_transform_.resize(infos.size());
    for (size_t i = 0; i < infos.size(); ++i) {
        const auto& info = *infos[i];
        Matrix4dR Tr = info.imu_to_sensor_transform;
        Tr.block<3, 1>(0, 3) *= 1e-3;  // mm -> m
        imu_to_body_transform_[i] = info.extrinsic * Tr;
    }
}

bool InertialIntegrationImuDeskewMethod::pick_last_valid_imu_pose(
    const LidarScanSet& scan_set, double& last_ts, Matrix4dR& last_pose) {
    bool imu_data_found = false;
    for (const auto& scan : scan_set.valid_scans()) {
        if (!scan.has_field(ChanField::IMU_STATUS)) {
            continue;
        }

        Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status =
            scan.field(ChanField::IMU_STATUS);
        std::vector<int> imu_valid =
            impl::get_valid_columns<uint16_t>(imu_status);
        if (imu_valid.empty()) {
            continue;
        }

        int last_valid_imu_idx = imu_valid.back();
        Eigen::Ref<const Eigen::ArrayX<uint64_t>> scan_ts =
            scan.field(ChanField::IMU_TIMESTAMP);
        double candidate_ts = scan_ts[last_valid_imu_idx] * 1e-9;
        if (!imu_data_found || candidate_ts > last_ts) {
            last_ts = candidate_ts;
            Eigen::Ref<const Eigen::ArrayX<uint16_t>> meas_id =
                scan.field(ChanField::IMU_MEASUREMENT_ID);
            uint16_t last_pose_col = meas_id(last_valid_imu_idx);
            last_pose = scan.get_column_pose(last_pose_col);
        }
        imu_data_found = true;
    }

    return imu_data_found;
}

void InertialIntegrationImuDeskewMethod::update(LidarScanSet& scan_set) {
    if (ts_list_.size() < 2) {
        return;
    }

    size_t total_possible_imu_data = 0;
    std::vector<std::vector<int>> imu_valid_per_sensor(scan_set.size());
    for (size_t sidx : scan_set.valid_indices()) {
        const auto& scan = *scan_set[sidx];
        if (!scan.has_field(ChanField::IMU_STATUS)) {
            continue;
        }
        Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status =
            scan.field(ChanField::IMU_STATUS);
        std::vector<int> imu_valid =
            impl::get_valid_columns<uint16_t>(imu_status);
        total_possible_imu_data += imu_valid.size();
        imu_valid_per_sensor[sidx] = std::move(imu_valid);
    }

    // Combine and and sort all the imu data from all sensors by time stamp
    std::vector<double> combined_ts;
    combined_ts.reserve(total_possible_imu_data);
    std::vector<Eigen::Vector3d> combined_gyro_body_frame;
    combined_gyro_body_frame.reserve(total_possible_imu_data);
    std::vector<Eigen::Vector3d> combined_accel_body_frame;
    combined_accel_body_frame.reserve(total_possible_imu_data);

    for (size_t sidx : scan_set.valid_indices()) {
        const auto& scan = *scan_set[sidx];
        if (!scan.has_field(ChanField::IMU_STATUS)) {
            continue;
        }

        const std::vector<int>& imu_valid = imu_valid_per_sensor[sidx];

        if (imu_valid.empty()) {
            continue;
        }

        Eigen::Ref<const Eigen::ArrayX<uint64_t>> ts =
            scan.field(ChanField::IMU_TIMESTAMP);
        Eigen::Ref<const ArrayX3fR> gyro = scan.field(ChanField::IMU_GYRO);
        Eigen::Ref<const ArrayX3fR> acc = scan.field(ChanField::IMU_ACC);
        MatrixX3dR valid_gyro =
            gyro(imu_valid, EIGEN_INDEXING_ALL).cast<double>();
        MatrixX3dR valid_acc =
            acc(imu_valid, EIGEN_INDEXING_ALL).cast<double>();
        valid_gyro.rowwise() -= gyro_bias_imu_frame_[sidx].transpose();
        valid_acc.rowwise() -= accel_bias_imu_frame_[sidx].transpose();

        MatrixX3dR gyro_body_frame;
        MatrixX3dR accel_body_frame;
        transform_imu_data_to_body_frame(imu_to_body_transform_[sidx],
                                         valid_gyro, valid_acc, gyro_body_frame,
                                         accel_body_frame);

        for (size_t k = 0; k < imu_valid.size(); ++k) {
            combined_ts.push_back(ts(imu_valid[k]) * 1e-9);
            combined_gyro_body_frame.emplace_back(gyro_body_frame.row(k));
            combined_accel_body_frame.emplace_back(accel_body_frame.row(k));
        }
    }

    if (combined_ts.empty() || last_scan_set_last_ts_ == nonstd::nullopt) {
        for (auto& scan : scan_set.valid_scans()) {
            impl::interp_pose(scan, ts_list_.front(), pose_list_.front(),
                              ts_list_.back(), pose_list_.back());
        }
        last_scan_set_ = scan_set;
        return;
    }

    double last_ts = *last_scan_set_last_ts_;
    Matrix4dR last_pose = *last_scan_set_last_pose_;

    // TODO[UN]: optimize this sorting given the timestamps are monotonically
    // increasing for each sensor
    std::vector<size_t> indices(combined_ts.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&combined_ts](size_t left, size_t right) {
                  return combined_ts[left] < combined_ts[right];
              });
    auto last = std::unique(indices.begin(), indices.end(),
                            [&combined_ts](size_t left, size_t right) {
                                return combined_ts[left] == combined_ts[right];
                            });
    indices.erase(last, indices.end());
    // selected unique (time wise) imu measurement data after sort
    std::vector<double> sorted_ts(indices.size());
    MatrixX3dR sorted_gyro(indices.size(), 3);
    MatrixX3dR sorted_accel(indices.size(), 3);

    for (size_t i = 0; i < indices.size(); ++i) {
        sorted_ts[i] = combined_ts[indices[i]];
        sorted_gyro.row(i) = combined_gyro_body_frame[indices[i]];
        sorted_accel.row(i) = combined_accel_body_frame[indices[i]];
    }

    std::vector<Matrix4dR> imu_poses_world_frame = calc_poses_with_motion_model(
        last_ts, last_pose, sorted_ts, sorted_gyro, sorted_accel);

    for (size_t sidx : scan_set.valid_indices()) {
        auto& scan = *scan_set[sidx];
        if (!scan.has_field(ChanField::IMU_STATUS)) {
            impl::interp_pose(scan, ts_list_.front(), pose_list_.front(),
                              ts_list_.back(), pose_list_.back());
            continue;
        }

        // This block deals with the edge cases where lidar packets may have
        // been dropped but corresponding imu packet exist and can be use to
        // subsitute the missing lidar timestamps
        const std::vector<int>& imu_valid = imu_valid_per_sensor[sidx];
        Eigen::Ref<const Eigen::ArrayX<uint64_t>> imu_ts =
            scan.field(ChanField::IMU_TIMESTAMP);
        Eigen::Ref<const Eigen::ArrayX<uint16_t>> meas_id =
            scan.field(ChanField::IMU_MEASUREMENT_ID);
        Eigen::ArrayX<uint64_t> scan_ts_copy = scan.timestamp();
        Eigen::ArrayX<uint32_t> scan_status_copy = scan.status();
        // override scan status and timestamps with imu where available
        for (size_t idx = 0; idx < imu_valid.size(); ++idx) {
            int imu_col = imu_valid[idx];
            int scan_col = meas_id[imu_col];
            scan_ts_copy[scan_col] = imu_ts[imu_col];
            scan_status_copy[scan_col] = 1;
        }

        std::vector<int> scan_valid =
            impl::get_valid_columns<uint32_t>(scan_status_copy);
        std::vector<double> scan_ts_vec =
            impl::get_valid_timestamps(scan_ts_copy, scan_valid);
        auto interp =
            interp_pose(scan_ts_vec, sorted_ts, imu_poses_world_frame);
        for (size_t k = 0; k < scan_valid.size(); ++k) {
            Matrix4dR pose = Eigen::Map<Matrix4dR>(interp.row(k).data());
            scan.set_column_pose(scan_valid[k], pose);
        }
    }

    last_scan_set_ = scan_set;
}

void InertialIntegrationImuDeskewMethod::set_last_pose(int64_t ts,
                                                       const Matrix4dR& pose) {
    // retrieve the last imu pose from the last scan set before the time get
    // reset by the active time correction in case of an unsynchronized
    // multi-sensor setup
    double last_ts = 0.0;
    Matrix4dR last_pose = Matrix4dR::Identity();
    if (pick_last_valid_imu_pose(last_scan_set_, last_ts, last_pose)) {
        last_scan_set_last_ts_ = last_ts;
        last_scan_set_last_pose_ = last_pose;
    }
    DeskewMethod::set_last_pose(ts, pose);
    estimate_gravity_vector(last_scan_set_);
}

void InertialIntegrationImuDeskewMethod::transform_imu_data_to_body_frame(
    Eigen::Ref<const Matrix4dR> imu_to_body_transform,
    Eigen::Ref<const MatrixX3dR> gyro_imu_frame,
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

double InertialIntegrationImuDeskewMethod::angle_between_poses(
    Eigen::Ref<const Matrix4dR> pose_a, Eigen::Ref<const Matrix4dR> pose_b) {
    double trace =
        (pose_a.block<3, 3>(0, 0) * pose_b.block<3, 3>(0, 0).transpose())
            .trace();
    auto clamp = [](double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };
    double cos_theta = clamp((trace - 1.0) / 2.0, -1.0, 1.0);
    return std::acos(cos_theta);
}

// estimate gyro and accel biases and gravity vector (world frame) when
// detecting the lidar sensors are rather static (low motion)
void InertialIntegrationImuDeskewMethod::estimate_gravity_vector(
    LidarScanSet& scans) {
    if (ts_list_.size() < 2) {
        return;
    }

    constexpr double LOWPASS_FRACTION = 0.1;
    constexpr double MAX_LINEAR_MOTION_M = 0.01;                   // m
    constexpr double MAX_ANGULAR_MOTION_RAD = M_PI / 180.0;        // rad
    constexpr double MAX_EXPECTED_GYRO_BIAS = 2.0 * M_PI / 180.0;  // rad/s
    constexpr double MAX_EXPECTED_ACCEL_BIAS = 0.5;                // m/s^2

    double pose_ang_delta =
        angle_between_poses(pose_list_.back(), pose_list_.front());
    double pose_lin_delta = (pose_list_.back().block<3, 1>(0, 3) -
                             pose_list_.front().block<3, 1>(0, 3))
                                .norm();

    for (size_t sidx : scans.valid_indices()) {
        auto& scan = *scans[sidx];
        if (!scan.has_field(ChanField::IMU_STATUS)) {
            continue;
        }

        Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status =
            scan.field(ChanField::IMU_STATUS);
        std::vector<int> valid = impl::get_valid_columns<uint16_t>(imu_status);

        if (valid.size() != static_cast<unsigned>(imu_status.size())) {
            continue;
        }

        if (pose_ang_delta >= MAX_ANGULAR_MOTION_RAD) {
            continue;
        }

        Eigen::Ref<const ArrayX3fR> gyro = scan.field(ChanField::IMU_GYRO);
        MatrixX3dR valid_gyro = gyro(valid, EIGEN_INDEXING_ALL).cast<double>();
        Eigen::Vector3d mean_gyro = valid_gyro.colwise().mean();
        if (mean_gyro.norm() < MAX_EXPECTED_GYRO_BIAS) {
            if (gyro_bias_imu_frame_[sidx].isZero()) {
                gyro_bias_imu_frame_[sidx] = mean_gyro;
            } else {
                gyro_bias_imu_frame_[sidx] =
                    (1 - LOWPASS_FRACTION) * gyro_bias_imu_frame_[sidx] +
                    LOWPASS_FRACTION * mean_gyro;
            }
        }

        Eigen::Ref<const ArrayX3fR> acc = scan.field(ChanField::IMU_ACC);
        MatrixX3dR valid_acc = acc(valid, EIGEN_INDEXING_ALL).cast<double>();
        Eigen::Vector3d mean_acc = valid_acc.colwise().mean();
        if (pose_lin_delta < MAX_LINEAR_MOTION_M &&
            std::abs(mean_acc.norm() - GRAVITY_MPERSEC2) <
                MAX_EXPECTED_ACCEL_BIAS) {
            Eigen::Vector3d gravity_vector_imu_frame_xyz =
                mean_acc / mean_acc.norm();
            Eigen::Vector3d accel_bias =
                mean_acc - gravity_vector_imu_frame_xyz * GRAVITY_MPERSEC2;
            if (accel_bias_imu_frame_[sidx].isZero()) {
                accel_bias_imu_frame_[sidx] = accel_bias;
            } else {
                accel_bias_imu_frame_[sidx] =
                    (1 - LOWPASS_FRACTION) * accel_bias_imu_frame_[sidx] +
                    LOWPASS_FRACTION * accel_bias;
            }

            // Transform to sensor frame then world
            valid_gyro.rowwise() -= gyro_bias_imu_frame_[sidx].transpose();
            valid_acc.rowwise() -= accel_bias_imu_frame_[sidx].transpose();
            MatrixX3dR gyro_body_frame;
            MatrixX3dR accel_body_frame;
            transform_imu_data_to_body_frame(imu_to_body_transform_[sidx],
                                             valid_gyro, valid_acc,
                                             gyro_body_frame, accel_body_frame);

            Eigen::Ref<const Eigen::ArrayX<uint16_t>> meas_id =
                scan.field(ChanField::IMU_MEASUREMENT_ID);
            std::vector<Matrix4dR> world_poses;
            for (int idx : valid) {
                int id = meas_id(idx);
                world_poses.push_back(scan.get_column_pose(id));
            }
            MatrixX3dR imu_acc_world(accel_body_frame.rows(), 3);
            for (int r = 0; r < accel_body_frame.rows(); ++r) {
                imu_acc_world.row(r) = world_poses[r].block<3, 3>(0, 0) *
                                       accel_body_frame.row(r).transpose();
            }
            Eigen::Vector3d mean_imu_acc_world_frame =
                imu_acc_world.colwise().mean();
            Eigen::Vector3d gravity_vector =
                mean_imu_acc_world_frame.normalized();
            if (!gravity_vector_world_frame_xyz_) {
                gravity_vector_world_frame_xyz_ = gravity_vector;
            } else {
                gravity_vector_world_frame_xyz_ =
                    (1 - LOWPASS_FRACTION) *
                        (*gravity_vector_world_frame_xyz_) +
                    LOWPASS_FRACTION * gravity_vector;
            }
            gravity_vector_world_frame_xyz_->normalize();
        }
    }
}

std::vector<Matrix4dR>
InertialIntegrationImuDeskewMethod::calc_poses_with_motion_model(
    double last_timestamp, Eigen::Ref<const Matrix4dR> last_body_to_world_pose,
    const std::vector<double>& timestamps,
    Eigen::Ref<const MatrixX3dR> angular_velocity_body_frame,
    Eigen::Ref<const MatrixX3dR> linear_accel_body_frame) {
    size_t N = timestamps.size();
    std::vector<Matrix4dR> body_to_world_poses(N, Matrix4dR::Identity());
    // Initial linear velocity (world frame)
    Eigen::Vector3d current_linear_velocity_world_frame =
        (pose_list_.back().block<3, 1>(0, 3) -
         pose_list_.front().block<3, 1>(0, 3)) /
        (ts_list_.back() - ts_list_.front());

    // TODO[UN]: we need to skip imu measurements for timestamps earlier than
    // last_timestamp and feed in poses from last scan set or alter the logic
    // of pick_last_valid_imu_pose to get the last imu pose before current
    // timestamp[0]. Alternatively, in cases of an imu measurements overlap one
    // could simply average the results between the overlapping measurements.
    // I will defer the choice to the next iteration.
    for (size_t i = 0; i < N; ++i) {
        double dt;
        Eigen::Matrix3d prev_world_orientation;
        Eigen::Vector3d prev_world_position;
        if (i == 0) {
            dt = timestamps[i] - last_timestamp;
            prev_world_orientation = last_body_to_world_pose.block<3, 3>(0, 0);
            prev_world_position = last_body_to_world_pose.block<3, 1>(0, 3);
        } else {
            dt = timestamps[i] - timestamps[i - 1];
            prev_world_orientation =
                body_to_world_poses[i - 1].block<3, 3>(0, 0);
            prev_world_position = body_to_world_poses[i - 1].block<3, 1>(0, 3);
        }

        // 2. Update the orientation
        Eigen::Vector3d w = angular_velocity_body_frame.row(i);
        double angle = w.norm() * dt;
        Eigen::Matrix3d delta_rotation;
        if (angle < 1e-12) {
            delta_rotation.setIdentity();
        } else {
            Eigen::Vector3d axis = w.normalized();
            delta_rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        }
        Eigen::Matrix3d new_world_orientation =
            prev_world_orientation * delta_rotation;

        // 3. Update local velocity (if accelerations are provided)
        if (gravity_vector_world_frame_xyz_) {
            // Use the average of the old and new orientation to transform the
            // velocity According to the internet, this is fine for small angles
            // and is more accurate
            Eigen::Vector3d acc_local = linear_accel_body_frame.row(i);
            Eigen::Vector3d world_acc =
                (prev_world_orientation + new_world_orientation) * 0.5 *
                acc_local;
            Eigen::Vector3d world_acc_no_g =
                world_acc -
                (*gravity_vector_world_frame_xyz_) * GRAVITY_MPERSEC2;
            current_linear_velocity_world_frame += world_acc_no_g * dt;
        }

        // 4. Calculate new position
        Eigen::Vector3d new_world_position =
            prev_world_position + current_linear_velocity_world_frame * dt;

        // 5. Assemble and store the new pose
        body_to_world_poses[i].block<3, 3>(0, 0) = new_world_orientation;
        body_to_world_poses[i].block<3, 1>(0, 3) = new_world_position;
    }

    return body_to_world_poses;
}

std::unique_ptr<DeskewMethod> DeskewMethodFactory::create(
    const std::string& method,
    const std::vector<std::shared_ptr<SensorInfo>>& infos) {
    // check if any of the sensors is on FW 3.2 and has IMU data
    bool has_imu_data = false;
    for (const auto& info : infos) {
        PacketFormat packet_format(*info);
        size_t imu_measurements_per_scan =
            packet_format.imu_measurements_per_packet *
            packet_format.imu_packets_per_frame;
        if (imu_measurements_per_scan > 0) {
            has_imu_data = true;
            break;
        }
    }

    if (method == "none") {
        logger().info("No deskewing will be applied");
        return nullptr;
    } else if (method == "constant_velocity") {
        logger().info("Using ConstantVelocityDeskewMethod");
        return std::make_unique<ConstantVelocityDeskewMethod>(infos);
    } else if (method == "imu_deskew") {
        logger().info("Using InertialIntegrationImuDeskewMethod");
        return std::make_unique<InertialIntegrationImuDeskewMethod>(infos);
    } else if (method == "auto") {
        if (!has_imu_data) {
            logger().info(
                "Synchronous IMU data not available (requires FW 3.2+ and "
                "ACCEL32_GYRO32_NMEA imu profile),"
                " falling back to ConstantVelocityDeskewMethod.\n"
                " Suppress this warning by adding '--deskew-method "
                "constant_velocity' to the 'slam' or 'localize' command.");
            return std::make_unique<ConstantVelocityDeskewMethod>(infos);
        } else {
            logger().info("Using InertialIntegrationImuDeskewMethod");
            return std::make_unique<InertialIntegrationImuDeskewMethod>(infos);
        }
    } else {
        throw std::invalid_argument("Invalid deskew_method: " + method);
    }
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

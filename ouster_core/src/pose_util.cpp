#include <ouster/core/pose_util.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

namespace impl {

size_t max_number_of_valid_points(const FrameSet& frame_set) {
    size_t count = 0;
    for (const auto& frame : frame_set.valid_frames()) {
        // A frame with no valid columns contributes no points; skip it rather
        // than let get_first_valid_column() throw (matches dewarp_impl).
        try {
            const int start_col = frame->get_first_valid_column();
            const int stop_col = frame->get_last_valid_column();
            count += static_cast<size_t>(stop_col - start_col + 1) * frame->h;
        } catch (const std::runtime_error& /*e*/) {
        }
    }

    return count;
}

}  // namespace impl

namespace {

Eigen::Vector3d normalize(const Eigen::Vector3d& vec) {
    const double norm = vec.norm();
    if (norm <= std::numeric_limits<double>::epsilon()) {
        return vec;
    }
    return vec / norm;
}

}  // namespace

Eigen::Matrix3d get_rot_matrix_to_align_to_gravity(double accel_x, double accel_y, double accel_z,
                                                   bool fix_yaw) {
    const Eigen::Vector3d gravity_vector = Eigen::Vector3d::UnitZ();

    Eigen::Vector3d accel_vector = normalize(Eigen::Vector3d(accel_x, accel_y, accel_z));

    // Build the shortest rotation that maps the measured acceleration
    // direction onto +Z axis by Rodrigues' formula.
    Eigen::Vector3d axis = accel_vector.cross(gravity_vector);
    if (axis.norm() > 0.0) {
        axis = normalize(axis);
    }

    const double dot = std::max(-1.0, std::min(1.0, accel_vector.dot(gravity_vector)));
    const double angle = std::acos(dot);

    Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
    skew(0, 1) = -axis(2);
    skew(0, 2) = axis(1);
    skew(1, 0) = axis(2);
    skew(1, 2) = -axis(0);
    skew(2, 0) = -axis(1);
    skew(2, 1) = axis(0);

    Eigen::Matrix3d rot_align_gravity = Eigen::Matrix3d::Identity() + std::sin(angle) * skew +
                                        (1.0 - std::cos(angle)) * (skew * skew);

    if (!fix_yaw) {
        return rot_align_gravity;
    }

    // If fix_yaw is true, we want to remove any yaw component from the
    // rotation. Neutralize the yaw introduced by the gravity-alignment step so
    // the result only changes roll/pitch while keeping the aligned Z axis.
    const Eigen::Vector3d forward = rot_align_gravity * Eigen::Vector3d::UnitX();
    const double yaw_angle = std::atan2(forward(1), forward(0));
    const double cos_yaw = std::cos(-yaw_angle);
    const double sin_yaw = std::sin(-yaw_angle);
    Eigen::Matrix3d rot_counter_yaw = Eigen::Matrix3d::Identity();
    rot_counter_yaw(0, 0) = cos_yaw;
    rot_counter_yaw(0, 1) = -sin_yaw;
    rot_counter_yaw(1, 0) = sin_yaw;
    rot_counter_yaw(1, 1) = cos_yaw;

    return rot_counter_yaw * rot_align_gravity;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

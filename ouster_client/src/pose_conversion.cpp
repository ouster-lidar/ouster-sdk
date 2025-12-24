#include <ouster/pose_conversion.h>

namespace ouster {
namespace sdk {
namespace core {

Eigen::Matrix4d euler_pose_to_matrix(const Eigen::Matrix<double, 6, 1>& pose) {
    double roll = pose(0);
    double pitch = pose(1);
    double yaw = pose(2);
    double x = pose(3);
    double y = pose(4);
    double z = pose(5);

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
    transformation_matrix(0, 3) = x;
    transformation_matrix(1, 3) = y;
    transformation_matrix(2, 3) = z;

    return transformation_matrix;
}

Eigen::Matrix4d quaternion_pose_to_matrix(
    const Eigen::Matrix<double, 7, 1>& pose) {
    double quat_w = pose(0);
    double quat_x = pose(1);
    double quat_y = pose(2);
    double quat_z = pose(3);
    double x = pose(4);
    double y = pose(5);
    double z = pose(6);

    Eigen::Quaterniond quat(quat_w, quat_x, quat_y, quat_z);

    quat.normalize();

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

    return transform;
}

Matrix3dR euler_to_rotation_matrix(double roll, double pitch, double yaw) {
    // clang-format off
    Matrix3dR R_x;
    R_x << 1, 0, 0,
           0, std::cos(roll), -std::sin(roll),
           0, std::sin(roll), std::cos(roll);
    
    Matrix3dR R_y;
    R_y << std::cos(pitch), 0, std::sin(pitch),
           0, 1, 0,
           -std::sin(pitch), 0, std::cos(pitch);
    
    Matrix3dR R_z;
    R_z << std::cos(yaw), -std::sin(yaw), 0,
           std::sin(yaw), std::cos(yaw), 0,
           0, 0, 1;
    // clang-format on
    return R_z * (R_y * R_x);
}

Eigen::Vector3d matrix_to_euler(const Matrix3dR& matrix) {
    double sy =
        std::sqrt(matrix(0, 0) * matrix(0, 0) + matrix(1, 0) * matrix(1, 0));
    bool singular = sy < 1e-6;

    double roll, pitch, yaw;
    if (!singular) {
        roll = std::atan2(matrix(2, 1), matrix(2, 2));
        pitch = std::atan2(-matrix(2, 0), sy);
        yaw = std::atan2(matrix(1, 0), matrix(0, 0));
    } else {
        roll = std::atan2(-matrix(1, 2), matrix(1, 1));
        pitch = std::atan2(-matrix(2, 0), sy);
        yaw = 0.0;
    }

    return Eigen::Vector3d(roll, pitch, yaw);
}

Matrix4dR xyzrpy_to_matrix(double px, double py, double pz, double roll,
                           double pitch, double yaw) {
    Matrix4dR out = Matrix4dR::Identity();
    out.block<3, 1>(0, 3) = Eigen::Vector3d(px, py, pz);
    out.block<3, 3>(0, 0) = euler_to_rotation_matrix(roll, pitch, yaw);
    return out;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

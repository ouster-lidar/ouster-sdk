#include <ouster/pose_conversion.h>

namespace ouster {
namespace core {

Eigen::Matrix4d euler_pose_to_matrix(const Eigen::Matrix<double, 6, 1>& pose) {
    double roll = pose(0);
    double pitch = pose(1);
    double yaw = pose(2);
    double x = pose(3);
    double y = pose(4);
    double z = pose(5);

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;

    return T;
}

Eigen::Matrix4d quaternion_pose_to_matrix(
    const Eigen::Matrix<double, 7, 1>& pose) {
    double qw = pose(0);
    double qx = pose(1);
    double qy = pose(2);
    double qz = pose(3);
    double x = pose(4);
    double y = pose(5);
    double z = pose(6);

    Eigen::Quaterniond quat(qw, qx, qy, qz);

    quat.normalize();

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

    return transform;
}

}  // namespace core
}  // namespace ouster

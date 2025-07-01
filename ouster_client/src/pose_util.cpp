#include <ouster/pose_util.h>

namespace ouster {
namespace core {

Poses interp_pose(const std::vector<double>& x_interp,
                  const std::vector<double>& x_known,
                  const Poses& poses_known) {
    std::vector<Eigen::Matrix<double, 4, 4>> poses_known_4x4;
    const size_t W = poses_known.rows();
    for (size_t w = 0; w < W; w++) {
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
            pose_matrix(poses_known.row(w).data());
        poses_known_4x4.emplace_back(pose_matrix);
    }

    std::vector<Eigen::Matrix<double, 4, 4>> inqury_pose_4x4 =
        interp_pose(x_interp, x_known, poses_known_4x4);

    const size_t num_interp = inqury_pose_4x4.size();
    Poses inqury_pose(num_interp, 16);
    for (size_t i = 0; i < num_interp; i++) {
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> row_4x4 =
            inqury_pose_4x4[i];
        Eigen::Map<const Eigen::Matrix<double, 1, 16, Eigen::RowMajor>> row_16(
            row_4x4.data());
        inqury_pose.row(i) = row_16;
    }

    return inqury_pose;
}

}  // namespace core
}  // namespace ouster

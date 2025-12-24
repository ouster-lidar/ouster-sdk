#include <ouster/pose_util.h>

#include <cstddef>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

Poses interp_pose(const std::vector<double>& x_interp,
                  const std::vector<double>& x_known,
                  const Poses& poses_known) {
    // Convert std::vector inputs to Eigen::Matrix for the templated function
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_interp_eigen(x_interp.size());
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_known_eigen(x_known.size());

    for (size_t i = 0; i < x_interp.size(); ++i) {
        x_interp_eigen(i) = x_interp[i];
    }
    for (size_t i = 0; i < x_known.size(); ++i) {
        x_known_eigen(i) = x_known[i];
    }

    // Call the templated interp_pose function
    return interp_pose<double, double>(x_interp_eigen, x_known_eigen,
                                       poses_known);
}

Poses interp_pose(const std::vector<double>& x_interp,
                  const std::vector<double>& x_known,
                  const std::vector<Matrix4dR>& poses_known) {
    // Convert std::vector inputs to Eigen::Matrix for the templated function
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_interp_eigen(x_interp.size());
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_known_eigen(x_known.size());

    for (size_t i = 0; i < x_interp.size(); ++i) {
        x_interp_eigen(i) = x_interp[i];
    }
    for (size_t i = 0; i < x_known.size(); ++i) {
        x_known_eigen(i) = x_known[i];
    }

    MatrixX16dR poses_known_eigen(poses_known.size(), 16);
    for (size_t i = 0; i < poses_known.size(); ++i) {
        poses_known_eigen.row(i) =
            Eigen::Map<const Eigen::Matrix<double, 1, 16>>(
                poses_known[i].data(), 16);
    }

    // Call the templated interp_pose function
    return interp_pose<double, double>(x_interp_eigen, x_known_eigen,
                                       poses_known_eigen);
}

namespace impl {

size_t max_number_of_valid_points(const LidarScanSet& lidar_scan_set) {
    size_t count = 0;
    for (const auto& scan : lidar_scan_set.valid_scans()) {
        const int start_col = scan.get_first_valid_column();
        const int stop_col = scan.get_last_valid_column();
        count += static_cast<size_t>(stop_col - start_col + 1) * scan.h;
    }

    return count;
}

}  // namespace impl

}  // namespace core
}  // namespace sdk
}  // namespace ouster

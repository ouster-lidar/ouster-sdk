#pragma once

#include <ouster/types.h>

#include <kiss_icp/core/Registration.hpp>
#include <kiss_icp/core/Threshold.hpp>
#include <kiss_icp/core/VoxelHashMap.hpp>
#include <kiss_icp/core/VoxelUtils.hpp>
#include <kiss_icp/pipeline/KissICP.hpp>
#include <sophus/se3.hpp>

namespace ouster {
namespace sdk {
namespace mapping {
namespace impl {

struct KissConfig {
    // map params
    double voxel_size = 1.0;
    double max_range = 100.0;
    double min_range = 5.0;
    int max_points_per_voxel = 20;

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;

    // registration params
    int max_num_iterations = 500;
    double convergence_criterion = 0.0001;
    int max_num_threads = 0;

    // Motion compensation
    bool deskew = false;
};

class KissICP {
   public:
    explicit KissICP(const KissConfig& config);

    ~KissICP() = default;

    KissICP(KissICP&&) noexcept = default;

    KissICP& operator=(KissICP&&) noexcept = default;

    Sophus::SE3d register_frame(const std::vector<Eigen::Vector3d>& frame,
                                const Sophus::SE3d& initial_guess,
                                bool update_map = true);

    std::vector<Eigen::Vector3d> get_local_map() const {
        return local_map_.Pointcloud();
    }

    void add_points_to_local_map(const std::vector<Eigen::Vector3d>& points) {
        local_map_.AddPoints(points);
    }

   private:
    kiss_icp::pipeline::KissICP::Vector3dVectorTuple voxelize(
        const std::vector<Eigen::Vector3d>& iframe);

    KissConfig config_;
    kiss_icp::AdaptiveThreshold adaptive_threshold_;
    kiss_icp::VoxelHashMap local_map_;
    kiss_icp::Registration registration_;

    size_t frame_count_ = 0;
};

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

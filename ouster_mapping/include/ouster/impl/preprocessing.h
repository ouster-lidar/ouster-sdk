#pragma once

#include <Eigen/Core>
#include <vector>

namespace ouster {

class Preprocessor {
   public:
    Preprocessor(const double max_range, const double min_range,
                 const bool deskew, const int max_num_threads);

    std::vector<Eigen::Vector3d> Preprocess(
        const std::vector<Eigen::Vector3d>& frame,
        const std::vector<double>& timestamps,
        const Eigen::Matrix4d& relative_motion) const;

   private:
    double max_range_;
    double min_range_;
    bool deskew_;
    int max_num_threads_;
};

}  // namespace ouster

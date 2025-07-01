#include "ouster/impl/preprocessing.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <functional>
#include <sophus/se3.hpp>
#include <vector>

// Users can enable OpenMP within ouster_client by first enabling omp through
// the compiler and then adding '-DOUSTER_OMP' option to the DCMAKE_CXX_FLAGS
#ifdef OUSTER_OMP
#define __OUSTER_UTILIZE_OPENMP__
#endif

// get the number of threads through OpenMP
#ifdef __OUSTER_UTILIZE_OPENMP__
#include <omp.h>
#define MAX_NUM_THREADS (omp_get_max_threads())
#else
#define MAX_NUM_THREADS (1)
#endif

namespace ouster {

Preprocessor::Preprocessor(const double max_range, const double min_range,
                           const bool deskew, const int max_num_threads)
    : max_range_(max_range),
      min_range_(min_range),
      deskew_(deskew),
      max_num_threads_(max_num_threads > 0 ? max_num_threads
                                           : MAX_NUM_THREADS) {}

std::vector<Eigen::Vector3d> Preprocessor::Preprocess(
    const std::vector<Eigen::Vector3d>& frame,
    const std::vector<double>& timestamps,
    const Eigen::Matrix4d& relative_motion) const {
    if (!deskew_ || timestamps.empty()) {
        return frame;
    }

    const auto min_max =
        std::minmax_element(timestamps.cbegin(), timestamps.cend());
    const double min_time = *min_max.first;
    const double max_time = *min_max.second;
    const auto normalize = [&](const double t) {
        return (t - min_time) / (max_time - min_time);
    };

    // convert to a Sophus object
    const Sophus::SE3d rel_motion(relative_motion);
    const auto& omega = rel_motion.log();

    std::vector<Eigen::Vector3d> deskewed_frame(frame.size());

#ifdef __OUSTER_UTILIZE_OPENMP__
    omp_set_num_threads(max_num_threads_);
#pragma omp parallel for
#endif
    for (int idx = 0; idx < static_cast<int>(frame.size()); ++idx) {
        const auto& point = frame.at(idx);
        const auto& stamp = normalize(timestamps.at(idx));
        const auto pose = Sophus::SE3d::exp((stamp - 0.5) * omega);
        deskewed_frame.at(idx) = pose * point;
    }

    std::vector<Eigen::Vector3d> preprocessed_frame;
    preprocessed_frame.reserve(deskewed_frame.size());
    for (const auto& point : deskewed_frame) {
        const double point_range = point.norm();
        if (point_range < max_range_ && point_range > min_range_) {
            preprocessed_frame.emplace_back(point);
        }
    }
    preprocessed_frame.shrink_to_fit();
    return preprocessed_frame;
}
}  // namespace ouster

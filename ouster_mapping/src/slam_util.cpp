#include "slam_util.h"

#include <ouster/impl/logging.h>
#include <ouster/pose_util.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <nonstd/optional.hpp>
#include <stdexcept>

using namespace ouster::sdk::core;

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
namespace sdk {
namespace mapping {
namespace impl {

nonstd::optional<double> determine_voxel_size(const LidarScanSet& scans,
                                              double start_pct,
                                              double end_pct) {
    if (start_pct < 0.0 || start_pct > 1.0 || end_pct < 0.0 || end_pct > 1.0 ||
        start_pct >= end_pct) {
        throw std::invalid_argument(
            "start_pct and end_pct must be in the range [0, 1] and "
            "start_pct must be less than end_pct.");
    }

    std::vector<uint32_t> selected_ranges;

    for (const auto& scan : scans.valid_scans()) {
        auto range = scan.field<uint32_t>(ChanField::RANGE);
        const int start_col = scan.get_first_valid_column();
        const int stop_col = scan.get_last_valid_column();
        Eigen::Ref<const LidarScan::Header<uint32_t>> status = scan.status();

        // Flatten valid range values into a vector
        std::vector<uint32_t> scan_range;
        scan_range.reserve((stop_col - start_col + 1) * range.rows());
        for (int x = start_col; x <= stop_col; ++x) {
            if (status[x] == 0) continue;
            for (int y = 0; y < range.rows(); ++y) {
                if (range(y, x) != 0) {
                    scan_range.push_back(range(y, x));
                }
            }
        }

        if (scan_range.empty()) {
            continue;
        }

        // Sort and select the percentile range
        std::sort(scan_range.begin(), scan_range.end());
        size_t n = scan_range.size();
        size_t start_idx =
            static_cast<size_t>(std::ceil(start_pct * 100.0 / 100.0 * n));
        size_t end_idx =
            static_cast<size_t>(std::floor(end_pct * 100.0 / 100.0 * n));
        if (end_idx >= n) {
            end_idx = n - 1;
        }

        for (size_t i = start_idx; i <= end_idx; ++i) {
            selected_ranges.push_back(scan_range[i]);
        }
    }

    if (selected_ranges.empty()) {
        return nonstd::nullopt;
    }

    // Compute mean and convert to meters, then divide by 46.0
    double sum = 0.0;
    for (auto v : selected_ranges) sum += static_cast<double>(v);
    double average = 0.001 * (sum / selected_ranges.size()) / 46.0;
    return 2.0 * average;
}

Eigen::Matrix3d make_ortho(const Eigen::Matrix3d& matrix) {
    // Perform SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Calculate the rotation matrix R = U * V^T
    Eigen::Matrix3d matrix_u = svd.matrixU();
    Eigen::Matrix3d matrix_v = svd.matrixV();
    Eigen::Matrix3d rotation = matrix_u * matrix_v.transpose();
    // Check if the determinant is negative (indicating a reflection)
    if (rotation.determinant() < 0.0) {
        // Flip the sign of the column of U corresponding to the smallest
        // singular value
        matrix_u.col(2) *= -1.0;
        rotation = matrix_u * matrix_v.transpose();
    }

    return rotation;
}

Eigen::Matrix4d make_ortho(const Eigen::Matrix4d& matrix) {
    Eigen::Matrix4d result = matrix;
    Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
    result.block<3, 3>(0, 0) = make_ortho(rotation);
    return result;
}

void transform_inplace(LidarScan& scan, const Matrix4dR& transform) {
    Eigen::Ref<const LidarScan::Header<uint32_t>> status = scan.status();
    auto poses = Eigen::Map<MatrixX16dR>(scan.pose().get<double>(), scan.w, 16);
    for (size_t j = 0; j < scan.w; ++j) {
        if (status[j] == 0) {
            continue;
        }
        auto pose = Eigen::Map<Matrix4dR>(poses.row(j).data());
        pose = transform * pose;
    }
}

void transform_inplace(LidarScanSet& scan_set, const Matrix4dR& transform) {
    for (auto& scan : scan_set.valid_scans()) {
        transform_inplace(scan, transform);
    }
}

std::vector<double> get_valid_timestamps(
    Eigen::Ref<const Eigen::ArrayX<uint64_t>> ts_field,
    const std::vector<int>& valid) {
    std::vector<double> ts;
    ts.reserve(valid.size());
    std::transform(valid.begin(), valid.end(), std::back_inserter(ts),
                   [&ts_field](int idx) {
                       return static_cast<double>(ts_field(idx)) * 1e-9;
                   });
    return ts;
};

// Returns the adjusted timestamp range for a LidarScanSet
std::pair<uint64_t, uint64_t> compute_frame_ts_range(
    const LidarScanSet& scans) {
    uint64_t frame_start_ts = std::numeric_limits<uint64_t>::max();
    uint64_t frame_stop_ts = std::numeric_limits<uint64_t>::min();

    for (size_t idx : scans.valid_indices()) {
        const auto& scan = scans[idx];
        Eigen::Ref<const LidarScan::Header<uint64_t>> c_timestamps =
            scan->timestamp();
        uint64_t start_ts = c_timestamps[scan->get_first_valid_column()];
        uint64_t stop_ts = c_timestamps[scan->get_last_valid_column()];
        frame_start_ts = std::min(frame_start_ts, start_ts);
        frame_stop_ts = std::max(frame_stop_ts, stop_ts);
    }

    return {frame_start_ts, frame_stop_ts};
}

std::pair<size_t, size_t> find_scan_set_mid(
    const LidarScanSet& scans, const std::pair<int64_t, int64_t>& ts_range) {
    // the scans set middle point should be ~ (frame_ts_range.first +
    // frame_duration / 2)
    const int64_t mid_point_ts = (ts_range.first + ts_range.second) / 2;
    size_t mid_scan_idx = 0;
    size_t mid_column_idx = 0;
    int64_t min_diff = std::numeric_limits<int64_t>::max();
    for (size_t idx : scans.valid_indices()) {
        const auto& scan = *scans[idx];
        Eigen::Ref<const LidarScan::Header<uint64_t>> c_timestamps =
            scan.timestamp();
        Eigen::Ref<const LidarScan::Header<uint32_t>> scan_status =
            scan.status();
        for (int col = scan.get_first_valid_column();
             col <= scan.get_last_valid_column(); ++col) {
            if ((scan_status[col] & 0x01) == 0) {
                continue;
            }
            int64_t col_ts = c_timestamps[col];
            int64_t diff_from_mid_point_ts = std::abs(col_ts - mid_point_ts);
            if (diff_from_mid_point_ts < min_diff) {
                min_diff = diff_from_mid_point_ts;
                mid_scan_idx = idx;
                mid_column_idx = col;
            }
        }
    }

    return {mid_scan_idx, mid_column_idx};
}

void interp_pose(LidarScan& scan, double t0, Eigen::Ref<const Matrix4dR> x0,
                 double t1, Eigen::Ref<const Matrix4dR> x1) {
    std::vector<int> valid_cols =
        impl::get_valid_columns<uint32_t>(scan.status());
    std::vector<double> scan_ts =
        impl::get_valid_timestamps(scan.timestamp(), valid_cols);
    std::vector<Matrix4dR> interp =
        ouster::sdk::core::interp_pose(scan_ts, t0, x0, t1, x1);
    for (size_t k = 0; k < valid_cols.size(); ++k) {
        scan.set_column_pose(valid_cols[k], interp[k]);
    }
}

}  // namespace impl
}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <limits>

#include "ouster/lidar_scan.h"

#include <cstdio>

namespace ouster {

struct LidarScanStat {
    using row_t = Eigen::Array<double, 1, 4>;
    using cov_t = Eigen::Array<double, 4, 4>;
    using data_t = Eigen::Array<double, Eigen::Dynamic, 4>;
    using index_t = LidarScan::index_t;
    using ts_t = LidarScan::ts_t;

    explicit LidarScanStat(const LidarScan& ls)
        : ts_min_{0},
          ts_max_{0},
          duration_{0},
          lidar_scan_{ls},
          data_{ls.data.cast<double>()} {
        if (lidar_scan_.w > 0) {
            auto minmax =
                minmax_element(lidar_scan_.ts.begin(), lidar_scan_.ts.end());
            ts_min_ = *minmax.first;
            ts_max_ = *minmax.second;
            duration_ = (ts_max_ - ts_min_);
        }

        // TODO[pb]: Fix it for the empty LidarScan case
        mean_ = data_.colwise().mean();
        min_ = data_.colwise().minCoeff();
        max_ = data_.colwise().maxCoeff();

        auto d = data_.rowwise() - mean_;

        cov_ = d.transpose().matrix() * d.matrix();
        cov_ /= ls.w * ls.h;
    }

    row_t mean() const { return mean_; }
    double mean(const LidarScan::LidarScanIndex& channel) const {
        return mean_(channel);
    }

    row_t min() const { return min_; }
    double min(const LidarScan::LidarScanIndex& channel) const {
        return min_(channel);
    }


    row_t max() const { return max_; }
    double max(const LidarScan::LidarScanIndex& channel) const {
        return max_(channel);
    }

    ts_t ts_min() const { return ts_min_; }
    ts_t ts_max() const { return ts_max_; }
    ts_t duration() const { return duration_; }

    cov_t covMatrix() const { return cov_; }
    double cov(const LidarScan::LidarScanIndex& channel) const {
        return cov_(channel, channel);
    }

    index_t w() const { return lidar_scan_.w; }
    index_t h() const { return lidar_scan_.h; }

   private:
    row_t mean_, min_, max_;
    cov_t cov_;
    ts_t ts_min_, ts_max_;  // nanoseconds
    ts_t duration_;
    const LidarScan& lidar_scan_;
    const data_t data_;
};

}  // namespace ouster
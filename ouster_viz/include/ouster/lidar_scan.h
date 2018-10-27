#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace ouster {

/**
 * LidarScan class
 * by convention:
 * row coordinate: 0 <= u < H
 * col coordinate: 0 <= v < W
 */

class LidarScan {
   public:
    const size_t W;
    const size_t H;
    std::vector<double> range;
    std::vector<double> intensity;
    std::vector<double> reflectivity;
    std::vector<double> noise;

    LidarScan(size_t w = 2048, size_t h = 64)
        : W(w),
          H(h),
          range(W * H, 0),
          intensity(W * H, 0),
          reflectivity(W * H, 0),
          noise(W * H, 0) {}

    void reset() {
        std::fill(range.begin(), range.end(), 0);
        std::fill(intensity.begin(), intensity.end(), 0);
        std::fill(reflectivity.begin(), reflectivity.end(), 0);
        std::fill(noise.begin(), noise.end(), 0);
    }
};

}  // namespace ouster

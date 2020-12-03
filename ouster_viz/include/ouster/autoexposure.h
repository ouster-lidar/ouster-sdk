/**
 * @file
 * @brief Adjust brightness of the image
 *
 * Functor that adjusts brightness so that 3rd percentile pixel is black
 * and 97th percentile pixel is white.
 */

#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace ouster {
namespace viz {
struct AutoExposure {
   private:
    const double percentile = 0.1;
    // damping makes the autoexposure smooth and avoids flickering.
    // however, it becomes slower to update.
    // 1.0 --> slowest, smoothest
    // 0.0 --> fastest, prone to flickering
    const double damping = 0.90;
    // for performance reasons, we may not want to update every frame
    // but rather every few frames
    const int update_every = 3;
    // for performance reasons, only consider a subset of points
    const size_t stride = 4;
    // if there are too few points, do nothing
    const size_t min_nonzero_points = 100;

    double lo_state = -1.0;
    double hi_state = -1.0;
    double lo = -1.0;
    double hi = -1.0;

    bool initialized = false;
    int counter = 0;

   public:
    /**
     * scales the image so that contrast is stretched between 0 and 1,
     * so that the top percentile is 1 - percentile
     * and the bottom percentile is percentile.
     * Analogous to imagemagick's -contrast-stretch operation
     *
     * @param key_eigen Reference to image, modified in place
     */
    void operator()(Eigen::Ref<Eigen::ArrayXd> key_eigen) {
        if (counter == 0) {
            const size_t n = key_eigen.rows();
            std::vector<size_t> indices;
            indices.reserve(n);
            for (size_t i = 0; i < n; i += stride) {
                // ignore 0 values, which are often due to dropped packets etc
                if (key_eigen[i] > 0) {
                    indices.push_back(i);
                }
            }
            if (indices.size() < min_nonzero_points) {
                // too few nonzero values, nothing to do
                return;
            }
            const size_t kth_extreme = indices.size() * percentile;
            auto cmp = [&](const size_t a, const size_t b) {
                return key_eigen(a) < key_eigen(b);
            };
            std::nth_element(indices.begin(), indices.begin() + kth_extreme,
                             indices.end(), cmp);
            lo = key_eigen[*(indices.begin() + kth_extreme)];
            std::nth_element(indices.begin() + kth_extreme,
                             indices.end() - kth_extreme - 1, indices.end(),
                             cmp);
            hi = key_eigen[*(indices.end() - kth_extreme - 1)];

            if (!initialized) {
                initialized = true;
                lo_state = lo;
                hi_state = hi;
            }
        }
        if (!initialized) {
            return;
        }
        // we use exponential smoothing
        lo_state = damping * lo_state + (1.0 - damping) * lo;
        hi_state = damping * hi_state + (1.0 - damping) * hi;
        counter = (counter + 1) % update_every;
        key_eigen += percentile - lo_state;
        key_eigen *= (1.0 - 2 * percentile) / (hi_state - lo_state);

        // clamp
        key_eigen = key_eigen.max(0.0).min(1.0);
    }
};
}  // namespace viz
}  // namespace ouster

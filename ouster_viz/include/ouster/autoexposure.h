/**
 * @file
 * @brief Adjust brightness image brightness and apply gamma correction
 *
 * Functor that adjusts brightness so that 1st percentile pixel is black
 * and 99th percentile pixel is white, while applying basic gamma correction
 * of 2.0.
 * Stores state of the black and white points so that it does not flicker
 * rapidly.
 */

#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <cstdlib>

struct AutoExposure {
   private:
    double lo_state = -1.0;
    double hi_state = -1.0;

   public:
    void operator()(Eigen::Ref<Eigen::ArrayXd> key_eigen) {
        const size_t n = key_eigen.rows();
        const size_t kth_extreme = n / 100;
        std::vector<size_t> indices(n);
        for (size_t i = 0; i < n; i++) {
            indices[i] = i;
        }
        auto cmp = [&](const size_t a, const size_t b) {
            return key_eigen(a) < key_eigen(b);
        };
        std::nth_element(indices.begin(), indices.begin() + kth_extreme,
                         indices.end(), cmp);
        const double lo = key_eigen[*(indices.begin() + kth_extreme)];
        std::nth_element(indices.begin() + kth_extreme,
                         indices.end() - kth_extreme, indices.end(), cmp);
        const double hi = key_eigen[*(indices.end() - kth_extreme)];
        if (lo_state < 0) {
            lo_state = lo;
            hi_state = hi;
        }
        lo_state = 0.9 * lo_state + 0.1 * lo;
        hi_state = 0.9 * hi_state + 0.1 * hi;
        key_eigen -= lo;
        key_eigen *= 1.0 / (hi - lo);

        // gamma correction
        key_eigen = key_eigen.max(0.0).sqrt().min(1.0);
    }
};

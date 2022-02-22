/**
 * @file
 * @brief Utilities for post-processing image data
 */

#pragma once

#include <Eigen/Core>
#include <vector>

#include "ouster/types.h"

namespace ouster {
namespace viz {

/**
 * Functor that adjusts brightness to between 0 and 1 */
class AutoExposure {
    const double lo_percentile, hi_percentile;  // percentiles used for scaling
    const int ae_update_every;

    double lo_state = -1.0;
    double hi_state = -1.0;
    double lo = -1.0;
    double hi = -1.0;

    bool initialized = false;
    int counter = 0;

   public:
    /* default constructor using default percentile and udpate values */
    AutoExposure();

    /* constructor specifying update modulo, and using default percentiles */
    AutoExposure(int update_every);

    /* constructor specifying low and high percentiles, and update modulo */
    AutoExposure(double lo_percentile, double hi_percentile, int update_every);

    /**
     * Scales the image so that contrast is stretched between 0 and 1.
     *
     * The top percentile is 1 - hi_percentile and the bottom percentile is
     * lo_percentile. Analogous to linear 'contrast-stretch', i.e.
     * normalization.
     *
     * @param image Reference to image, modified in place
     */
    void operator()(Eigen::Ref<img_t<double>> image, bool update_state = true);
};

/**
 * Corrects beam uniformity by minimizing median difference between rows,
 * thereby correcting subtle horizontal line artifacts in images, especially the
 * ambient image.
 */
class BeamUniformityCorrector {
   private:
    int counter = 0;
    std::vector<double> dark_count;

   public:
    /**
     * Applies dark count correction to an image, modifying it in-place to have
     * reduced horizontal line artifacts.
     *
     * @param image Mutable reference to an image as a 2D Eigen array,
     *              to be modified in-place
     */
    void operator()(Eigen::Ref<img_t<double>> image);
};
}  // namespace viz
}  // namespace ouster

/**
 * @file
 * @brief Utilities for post-processing image data
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

#include "ouster/types.h"

namespace ouster {
namespace viz {

/**
 * Corrects beam uniformity by minimizing median difference between rows,
 * thereby correcting subtle horizontal line artifacts in images, especially the
 * ambient image.
 */
class AutoExposure {
    double lo_state = -1.0;
    double hi_state = -1.0;
    double lo = -1.0;
    double hi = -1.0;

    bool initialized = false;
    int counter = 0;

   public:
    /**
     * Scales the image so that contrast is stretched between 0 and 1.
     *
     * The top percentile is 1 - percentile and the bottom percentile is
     * percentile. Analogous to imagemagick's -contrast-stretch operation.
     *
     * @param image Reference to image, modified in place
     */
    void operator()(Eigen::Ref<img_t<double>> image);
};

/**
 * Functor that adjusts brightness so that 3rd percentile pixel is black
 * and 97th percentile pixel is white.
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

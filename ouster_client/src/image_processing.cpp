/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/image_processing.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "ouster/deprecation.h"

namespace ouster {
namespace sdk {
namespace core {

namespace {

/*
 * damping makes the autoexposure smooth and avoids flickering however, it
 * becomes slower to update.
 * 1.0 --> slowest, smoothest
 * 0.0 --> fastest, prone to flickering
 */
const double AE_DAMPING = 0.90;
/*
 * for performance reasons, we may not want to update every frame but rather
 * every few frames
 */
const int AE_DEFAULT_UPDATE_EVERY = 3;

/* for performance reasons, only consider a subset of points */
const size_t AE_STRIDE = 4;

/* if there are too few points, do nothing */
const size_t AE_MIN_NONZERO_POINTS = 100;

/* default percentile for scaling in autoexposure */
const double AE_DEFAULT_PERCENTILE = 0.1;

}  // namespace

AutoExposure::AutoExposure()
    : lo_percentile_(AE_DEFAULT_PERCENTILE),
      hi_percentile_(AE_DEFAULT_PERCENTILE),
      ae_update_every_(AE_DEFAULT_UPDATE_EVERY) {}

AutoExposure::AutoExposure(int update_every)
    : lo_percentile_(AE_DEFAULT_PERCENTILE),
      hi_percentile_(AE_DEFAULT_PERCENTILE),
      ae_update_every_(update_every) {}

AutoExposure::AutoExposure(double lo_percentile, double hi_percentile,
                           int update_every)
    : lo_percentile_(lo_percentile),
      hi_percentile_(hi_percentile),
      ae_update_every_(update_every) {}

template <typename T>
void AutoExposure::update(Eigen::Ref<img_t<T>> image, bool update_state) {
    Eigen::Map<Eigen::Array<T, -1, 1>> key_eigen(image.data(), image.size());

    // int a;
    if (counter_ == 0 && update_state) {
        const size_t num_elements = key_eigen.rows();
        std::vector<size_t> indices;
        indices.reserve(num_elements);
        for (size_t i = 0; i < num_elements; i += AE_STRIDE) {
            // ignore 0 values, which are often due to dropped packets etc
            if (key_eigen[i] > 0) {
                indices.push_back(i);
            }
        }
        if (indices.size() < AE_MIN_NONZERO_POINTS) {
            // too few nonzero values, nothing to do
            return;
        }
        auto cmp = [&](const size_t a, const size_t b) {
            return key_eigen(a) < key_eigen(b);
        };

        const size_t lo_kth_extreme =
            static_cast<size_t>(indices.size() * lo_percentile_);
        std::nth_element(indices.begin(), indices.begin() + lo_kth_extreme,
                         indices.end(), cmp);
        lo_ = key_eigen[*(indices.begin() + lo_kth_extreme)];

        const size_t hi_kth_extreme =
            static_cast<size_t>(indices.size() * hi_percentile_);
        std::nth_element(indices.begin() + lo_kth_extreme,
                         indices.end() - hi_kth_extreme - 1, indices.end(),
                         cmp);
        hi_ = key_eigen[*(indices.end() - hi_kth_extreme - 1)];

        if (!initialized_) {
            initialized_ = true;
            lo_state_ = lo_;
            hi_state_ = hi_;
        }
    }
    if (!initialized_) {
        return;
    }

    // we use the simplest form of exponential smoothing
    if (update_state) {
        lo_state_ = AE_DAMPING * lo_state_ + (1.0 - AE_DAMPING) * lo_;
        hi_state_ = AE_DAMPING * hi_state_ + (1.0 - AE_DAMPING) * hi_;
    }

    // Apply affine transformation mapping lo_state to lo_percentile and
    // hi_state to 1 - hi_percentile. If it would map 0 to positive number,
    // instead map using only hi_state
    double lo_hi_scale =
        (1.0 - (lo_percentile_ + hi_percentile_)) / (hi_state_ - lo_state_);

    if (std::isinf(lo_hi_scale) || std::isnan(lo_hi_scale)) {
        // map everything relative to hi_state being 0.5 due to small spread or
        // nan
        key_eigen *= 0.5 / hi_state_;
    } else if (lo_hi_scale * (0.0 - lo_state_) + lo_percentile_ <= 0.00) {
        // apply affine transformation
        key_eigen -= lo_state_;
        key_eigen *= lo_hi_scale;
        key_eigen += lo_percentile_;
    } else {
        // lo_hi_state transformation would map 0 to positive number
        // instead, map using only hi_state
        key_eigen *= (1.0 - hi_percentile_) / (hi_state_);
    }

    // clamp
    key_eigen = key_eigen.max(static_cast<T>(0)).min(static_cast<T>(1));

    if (update_state) {
        counter_ = (counter_ + 1) % ae_update_every_;
    }
}

// use overloads vs templates so implicit conversion to Eigen::Ref still works
void AutoExposure::operator()(Eigen::Ref<img_t<float>> image,
                              bool update_state) {
    update(image, update_state);
}

void AutoExposure::operator()(Eigen::Ref<img_t<double>> image,
                              bool update_state) {
    update(image, update_state);
}

namespace {

/*
 * damping makes the correction smooth and avoids flickering.
 * however, it becomes slower to update.
 * 1.0 --> slowest, smoothest
 * 0.0 --> fastest, prone to flickering
 */
const double BUC_DAMPING = 0.92;

/*
 * for performance reasons, we may not want to update every frame
 * but rather every 8 or so frames.
 */
const int BUC_UPDATE_EVERY = 8;

}  // namespace

/*
 * computes the dark count, i.e. an additive offset in the brightness of the
 * image, to smoothe the difference between rows
 */
template <typename T>
static Eigen::Array<T, -1, 1> compute_dark_count(
    const Eigen::Ref<img_t<T>>& image) {
    const size_t image_h = image.rows();
    const size_t image_w = image.cols();

    Eigen::Array<T, -1, 1> tmp = Eigen::Array<T, -1, 1>::Zero(image_w);
    Eigen::Array<T, -1, 1> new_dark_count =
        Eigen::Array<T, -1, 1>::Zero(image_h);

    // probably computed lazily when used below?
    auto row_diffs = image.bottomRows(image_h - 1) - image.topRows(image_h - 1);

    // to handle azimuth-masked data, only consider columns with nonzero values
    Eigen::Array<bool, -1, 1> col_mask =
        image.template cast<bool>().colwise().any();
    const size_t n_cols = col_mask.count();

    if (n_cols == 0) {
        return new_dark_count;
    }

    img_t<T> row_diffs_nonzero{image_h - 1, n_cols};
    for (size_t i = 0, j = 0; i < image_w && j < n_cols; i++) {
        if (col_mask[i]) {
            row_diffs_nonzero.col(j) = row_diffs.col(i);
            j++;
        }
    }

    // compute the median of differences between rows
    for (size_t i = 1; i < image_h; i++) {
        tmp = row_diffs_nonzero.row(i - 1);
        std::nth_element(tmp.data(), tmp.data() + (n_cols / 2),
                         tmp.data() + n_cols);
        new_dark_count[i] = new_dark_count[i - 1] + tmp[n_cols / 2];
    }

    // remove gradients in the entire height of image by doing linear fit
    Eigen::Matrix<T, -1, 2> image_array(image_h, 2);
    for (size_t i = 0; i < image_h; i++) {
        image_array(i, 0) = 1;
        image_array(i, 1) = static_cast<T>(i);
    }
    Eigen::Matrix<T, 2, 1> x =
        image_array.fullPivLu().solve(new_dark_count.matrix());
    new_dark_count -= (image_array * x).array();

    // subtract minimum value
    new_dark_count -= new_dark_count.minCoeff();
    return new_dark_count;
}

template <typename T>
void BeamUniformityCorrector::update(Eigen::Ref<img_t<T>> image,
                                     bool update_state) {
    const auto image_h = image.rows();

    // compute dark counts, if necessary
    if (dark_count_.size() != image_h) {
        dark_count_ = compute_dark_count(image).template cast<double>();
    } else if (update_state && counter_ == 0) {
        // if previous state exists, update using exponential smoothing:
        Eigen::ArrayXd new_dark_count =
            compute_dark_count(image).template cast<double>();
        dark_count_ *= BUC_DAMPING;
        dark_count_ += new_dark_count * (1.0 - BUC_DAMPING);
    }
    counter_ = (counter_ + 1) % BUC_UPDATE_EVERY;

    // apply the dark count correction
    image.colwise() -= dark_count_.cast<T>();

    // clamp any negative values
    image = image.cwiseMax(static_cast<T>(0));
}

void BeamUniformityCorrector::operator()(Eigen::Ref<img_t<float>> image,
                                         bool update_state) {
    update(image, update_state);
}

void BeamUniformityCorrector::operator()(Eigen::Ref<img_t<double>> image,
                                         bool update_state) {
    update(image, update_state);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster

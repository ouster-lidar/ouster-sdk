#include "ouster/image_processing.h"

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace ouster {
namespace viz {

namespace {
const double ae_percentile = 0.1;

/*
 * damping makes the autoexposure smooth and avoids flickering however, it
 * becomes slower to update.
 * 1.0 --> slowest, smoothest
 * 0.0 --> fastest, prone to flickering
 */
const double ae_damping = 0.90;

/*
 * for performance reasons, we may not want to update every frame but rather
 * every few frames
 */
const int ae_update_every = 3;

/* for performance reasons, only consider a subset of points */
const size_t ae_stride = 4;

/* if there are too few points, do nothing */
const size_t ae_min_nonzero_points = 100;

}  // namespace

void AutoExposure::operator()(Eigen::Ref<img_t<double>> image) {
    Eigen::Map<Eigen::ArrayXd> key_eigen(image.data(), image.size());

    if (counter == 0) {
        const size_t n = key_eigen.rows();
        std::vector<size_t> indices;
        indices.reserve(n);
        for (size_t i = 0; i < n; i += ae_stride) {
            // ignore 0 values, which are often due to dropped packets etc
            if (key_eigen[i] > 0) {
                indices.push_back(i);
            }
        }
        if (indices.size() < ae_min_nonzero_points) {
            // too few nonzero values, nothing to do
            return;
        }
        const size_t kth_extreme = indices.size() * ae_percentile;
        auto cmp = [&](const size_t a, const size_t b) {
            return key_eigen(a) < key_eigen(b);
        };
        std::nth_element(indices.begin(), indices.begin() + kth_extreme,
                         indices.end(), cmp);
        lo = key_eigen[*(indices.begin() + kth_extreme)];
        std::nth_element(indices.begin() + kth_extreme,
                         indices.end() - kth_extreme - 1, indices.end(), cmp);
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
    lo_state = ae_damping * lo_state + (1.0 - ae_damping) * lo;
    hi_state = ae_damping * hi_state + (1.0 - ae_damping) * hi;
    counter = (counter + 1) % ae_update_every;
    key_eigen += ae_percentile - lo_state;
    key_eigen *= (1.0 - 2 * ae_percentile) / (hi_state - lo_state);

    // clamp
    key_eigen = key_eigen.max(0.0).min(1.0);
}

namespace {

/*
 * damping makes the correction smooth and avoids flickering.
 * however, it becomes slower to update.
 * 1.0 --> slowest, smoothest
 * 0.0 --> fastest, prone to flickering
 */
const double buc_damping = 0.92;

/*
 * for performance reasons, we may not want to update every frame
 * but rather every 8 or so frames.
 */
const int buc_update_every = 8;

}  // namespace

/*
 * computes the dark count, i.e. an additive offset in the brightness of the
 * image, to smoothe the difference between rows
 */
static std::vector<double> compute_dark_count(
    const Eigen::Ref<const img_t<double>>& image) {
    const size_t image_h = image.rows();
    const size_t image_w = image.cols();

    std::vector<double> tmp(image_w);
    std::vector<double> new_dark_count(image_h, 0);

    img_t<double> row_diffs =
        image.bottomRows(image_h - 1) - image.topRows(image_h - 1);

    // compute the median of differences between rows
    for (size_t i = 1; i < image_h; i++) {
        Eigen::Map<Eigen::Matrix<double, -1, 1>> tmp_map(tmp.data(), image_w);
        tmp_map = row_diffs.row(i - 1);
        std::nth_element(tmp.begin(), tmp.begin() + image_w / 2, tmp.end());
        new_dark_count[i] = new_dark_count[i - 1] + tmp[image_w / 2];
    }

    // remove gradients in the entire height of image by doing linear fit
    Eigen::Matrix<double, -1, 2> A(image_h, 2);
    for (size_t i = 0; i < image_h; i++) {
        A(i, 0) = 1;
        A(i, 1) = i;
    }

    Eigen::Vector2d x = A.fullPivLu().solve(
        Eigen::Map<Eigen::VectorXd>(new_dark_count.data(), image_h, 1));

    Eigen::Map<Eigen::ArrayXd>(new_dark_count.data(), image_h, 1) -=
        (A * x).array();

    // subtract minimum value
    double min_el =
        *std::min_element(new_dark_count.begin(), new_dark_count.end());
    Eigen::Map<Eigen::ArrayXd>(new_dark_count.data(), image_h, 1) -= min_el;
    return new_dark_count;
}

/*
 * updates the dark count by using exponential smoothing:
 * https://en.wikipedia.org/wiki/Exponential_smoothing
 * and then updates the dark_count member variable with the new result
 */
static void update_dark_count(const Eigen::Ref<const img_t<double>>& image,
                              std::vector<double>& dark_count) {
    const auto new_dark_count = compute_dark_count(image);
    const size_t image_h = image.rows();
    auto dark_count_map =
        Eigen::Map<Eigen::ArrayXd>(dark_count.data(), image_h);
    const auto new_dark_count_map =
        Eigen::Map<const Eigen::ArrayXd>(new_dark_count.data(), image_h);
    dark_count_map *= buc_damping;
    dark_count_map += new_dark_count_map * (1.0 - buc_damping);
}

void BeamUniformityCorrector::operator()(Eigen::Ref<img_t<double>> image) {
    const size_t image_h = image.rows();

    if (counter == 0) {
        if (dark_count.size() == 0) {
            dark_count = compute_dark_count(image);
        } else {
            update_dark_count(image, dark_count);
        }
    }
    counter = (counter + 1) % buc_update_every;

    // apply the dark count correction row by row
    for (size_t i = 0; i < image_h; i++) {
        // contains a view of the current row
        image.row(i) -= dark_count[i];
        image.row(i) = image.row(i).unaryExpr([](double x) {
            x = std::max(x, 0.0);
            x = std::min(x, (double)UINT32_MAX);
            return x;
        });
    }
}

}  // namespace viz
}  // namespace ouster

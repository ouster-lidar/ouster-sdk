/**
 * @file
 * @brief Corrects beam uniformity by minimizing median difference between rows,
 *        thereby correcting subtle horizontal line artifacts in images,
 *        especially the ambient image.
 */

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <vector>

namespace ouster {
namespace viz {

class BeamUniformityCorrector {
   private:
    // damping makes the correction smooth and avoids flickering.
    // however, it becomes slower to update.
    // 1.0 --> slowest, smoothest
    // 0.0 --> fastest, prone to flickering
    const double damping = 0.92;
    // for performance reasons, we may not want to update every frame
    // but rather every 8 or so frames.
    const int update_every = 8;
    int counter = 0;

    std::vector<double> dark_count;

    using im_t =
        Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    /**
     * computes the dark count, i.e. an additive offset in the brightness of the
     * image, to smoothe the difference between rows
     *
     * @param image Const reference to an image as a 2D Eigen array
     */
    static std::vector<double> compute_dark_count(
        const Eigen::Ref<const im_t>& image) {
        const size_t image_h = image.rows();
        const size_t image_w = image.cols();

        std::vector<double> tmp(image_w);
        std::vector<double> new_dark_count(image_h, 0);

        im_t row_diffs =
            image.bottomRows(image_h - 1) - image.topRows(image_h - 1);

        // compute the median of differences between rows
        for (size_t i = 1; i < image_h; i++) {
            Eigen::Map<Eigen::Matrix<double, -1, 1>> tmp_map(tmp.data(),
                                                             image_w);
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

    /**
     * updates the dark count by using exponential smoothing:
     * https://en.wikipedia.org/wiki/Exponential_smoothing
     * and then updates the dark_count member variable with the new result
     *
     * @param image Const reference to an image as a 2D Eigen array
     */
    void update_dark_count(const Eigen::Ref<const im_t>& image) {
        const auto new_dark_count = compute_dark_count(image);
        const size_t image_h = image.rows();
        auto dark_count_map =
            Eigen::Map<Eigen::ArrayXd>(dark_count.data(), image_h);
        const auto new_dark_count_map =
            Eigen::Map<const Eigen::ArrayXd>(new_dark_count.data(), image_h);
        dark_count_map *= damping;
        dark_count_map += new_dark_count_map * (1.0 - damping);
    }

   public:
    /**
     * Applies dark count correction to an image, modifying it in-place
     * to have reduced horizontal line artifacts.
     *
     * @param image Mutable reference to an image as a 2D Eigen array,
     *              to be modified in-place
     */
    void correct(Eigen::Ref<im_t> image) {
        const size_t image_h = image.rows();

        if (counter == 0) {
            if (dark_count.size() == 0) {
                dark_count = compute_dark_count(image);
            } else {
                update_dark_count(image);
            }
        }
        counter = (counter + 1) % update_every;

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
};
}  // namespace viz
}  // namespace ouster

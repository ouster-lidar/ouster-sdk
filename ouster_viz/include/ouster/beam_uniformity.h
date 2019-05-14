/**
 * @file
 * @brief Corrects beam uniformity by minimizing median difference between rows
 *
 */

#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <algorithm>

class BeamUniformityCorrector {
   private:
    std::vector<double> dark_count;

    std::vector<double> compute_dark_count(
        const Eigen::Ref<Eigen::ArrayXXd> image) {
        const size_t image_h = image.rows();
        const size_t image_w = image.cols();

        std::vector<double> tmp(image_w);
        std::vector<double> new_dark_count(image_h, 0);

        Eigen::ArrayXXd row_diffs =
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

   public:
    void correct(Eigen::Ref<Eigen::ArrayXXd> image) {
        const size_t image_h = image.rows();

        if (dark_count.size() == 0) {
            dark_count = compute_dark_count(image);
        } else {
            // update dark_count with a decaying weighted average
            const auto new_dark_count = compute_dark_count(image);
            Eigen::Map<Eigen::ArrayXd>(dark_count.data(), image_h) *= 0.95;
            Eigen::Map<Eigen::ArrayXd>(dark_count.data(), image_h) +=
                Eigen::Map<const Eigen::ArrayXd>(new_dark_count.data(),
                                                 image_h) *
                0.05;
        }

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

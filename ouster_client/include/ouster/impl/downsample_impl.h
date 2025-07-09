#pragma once

#include <Eigen/Core>

namespace ouster {
namespace core {
namespace impl {
template <typename PointScalar, typename AttribScalar>
class AccumulatedPoint {
   public:
    using AttribVecT = Eigen::Matrix<AttribScalar, 1, Eigen::Dynamic>;
    using PointT = Eigen::Matrix<PointScalar, 3, 1>;

   private:
    int num_points_;
    PointT point_;
    AttribVecT attrib_;

   public:
    AccumulatedPoint()
        : num_points_(0),
          point_(PointScalar(0.0), PointScalar(0.0), PointScalar(0.0)) {}

    void add_point(const PointT& point, const AttribVecT& attrib) {
        point_ += point;
        if (attrib.cols() > 0) {
            if (attrib_.cols() == 0) {
                attrib_.resize(1, attrib.cols());
                attrib_.setZero();
            }
            attrib_ += attrib;
        }
        num_points_ += 1;
    }

    PointT average_point() const { return point_ / double(num_points_); }

    AttribVecT average_attrib() const {
        return attrib_ / AttribScalar(num_points_);
    }

    int num_points() const { return num_points_; }
};

// Implementation adapted from:
// https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
template <typename T>
struct hash_eigen {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (int i = 0; i < static_cast<int>(matrix.size()); i++) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 +
                    (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
}  // namespace impl
}  // namespace core
}  // namespace ouster

/**
 * @file
 * @brief Holds lidar data by field in column-major order
 */

#pragma once

#include <Eigen/Eigen>
#include <iterator>
#include <utility>
#include <vector>
#include <cstdlib>

namespace ouster {

struct LidarScan {
    using Data = Eigen::Array<double, Eigen::Dynamic, 6>;
    using Point = Eigen::Array<double, 1, 6>;

    const ssize_t W;
    const ssize_t H;
    Data data_;

    LidarScan(size_t w, size_t h) : W(w), H(h), data_{w * h, 6} {};

    Eigen::Ref<Eigen::ArrayXd> x() { return data_.col(0); }
    Eigen::Ref<Eigen::ArrayXd> y() { return data_.col(1); }
    Eigen::Ref<Eigen::ArrayXd> z() { return data_.col(2); }
    Eigen::Ref<Eigen::ArrayXd> intensity() { return data_.col(3); }
    Eigen::Ref<Eigen::ArrayXd> noise() { return data_.col(4); }
    Eigen::Ref<Eigen::ArrayXd> range() { return data_.col(5); }

    struct iterator;

    iterator begin() { return iterator(0, &this->data_); }

    static inline Point make_val(float x, float y, float z, float intensity,
                                 uint32_t, uint16_t, uint8_t, uint16_t noise,
                                 uint32_t range) {
        Point p;
        p << x, y, z, intensity, noise, range;
        return p;
    }

    // Minimal set of operations to support os1_util.h:batch_to_iter; not really
    // a proper iterator. Remove when Eigen support for STL iterators lands.
    struct iterator {
        using iterator_category = std::output_iterator_tag;
        using value_type = LidarScan::Point;
        using difference_type = void;
        using pointer = void;
        using reference = void;

        inline iterator operator++() {
            idx_++;
            return *this;
        }
        inline Data::RowXpr operator*() { return data_->row(idx_); }

        inline Data::RowXpr operator[](int i) { return data_->row(idx_ + i); }

        friend iterator operator+(iterator lhs, int i) {
            return iterator{lhs.idx_ + i, lhs.data_};
        }

        friend bool operator==(const iterator& lhs, const iterator& rhs) {
            return lhs.idx_ == rhs.idx_;
        }

        friend bool operator!=(const iterator& lhs, const iterator& rhs) {
            return !(lhs == rhs);
        }

       private:
        iterator(int idx, Data* data) : idx_{idx}, data_{data} {}
        int idx_;
        Data* data_;

        friend class LidarScan;
    };
};
}

/**
 * @file
 * @brief Holds lidar data by field in row-major order
 */

#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include <cmath>
#include <iterator>
#include <utility>
#include <vector>

#include "ouster/compat.h"

namespace ouster {

using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

struct LidarScan {
    enum LidarScanIndex { RANGE, INTENSITY, NOISE, REFLECTIVITY };
    using ts_t = std::chrono::nanoseconds;
    using raw_t = uint32_t;
    using data_t = Eigen::Array<raw_t, Eigen::Dynamic, 4>;
    using index_t = std::ptrdiff_t;

    struct Pixel {
        raw_t range;
        raw_t intensity;
        raw_t noise;
        raw_t reflectivity;
        ts_t ts;
        static inline Pixel empty_val() { return Pixel{0, 0, 0, 0, ts_t(0)}; }
    };

    index_t w;
    index_t h;

    data_t data;
    std::vector<ts_t> ts;

    LidarScan(size_t w, size_t h) : w(w), h(h), data{w * h, 4}, ts(w){};

    inline index_t ind(const index_t u, const index_t v) { return u * w + v; }

    data_t::ColXpr range() { return data.col(LidarScanIndex::RANGE); }
    data_t::ColXpr intensity() { return data.col(LidarScanIndex::INTENSITY); }
    data_t::ColXpr noise() { return data.col(LidarScanIndex::NOISE); }
    data_t::ColXpr reflectivity() {
        return data.col(LidarScanIndex::REFLECTIVITY);
    }

    data_t::ConstColXpr range() const {
        return data.col(LidarScanIndex::RANGE);
    }
    data_t::ConstColXpr intensity() const {
        return data.col(LidarScanIndex::INTENSITY);
    }
    data_t::ConstColXpr noise() const {
        return data.col(LidarScanIndex::NOISE);
    }
    data_t::ConstColXpr reflectivity() const {
        return data.col(LidarScanIndex::REFLECTIVITY);
    }

    struct iterator;
    struct PixelRef;

    iterator begin() { return iterator(0, this); }

    static inline Pixel pixel(index_t u, index_t v, ts_t ts, ts_t scan_ts,
                              uint32_t range, uint16_t intensity,
                              uint16_t noise, uint16_t reflectivity) {
        return Pixel{static_cast<raw_t>(range), static_cast<raw_t>(intensity),
                     static_cast<raw_t>(noise),
                     static_cast<raw_t>(reflectivity), ts};
    }

    // Minimal set of operations to support os1_util.h:batch_to_iter; not
    // really a proper iterator.
    // Please note that the ordering of the iterator is column major
    // and not row major like the actual underlying data.
    struct iterator {
        using iterator_category = std::output_iterator_tag;
        using value_type = LidarScan::Pixel;
        using difference_type = void;
        using pointer = void;
        using reference = LidarScan::PixelRef;

        inline iterator operator++() {
            pixel_index_++;
            return *this;
        }
        inline PixelRef operator*() { return PixelRef(pixel_index_, li_); }

        inline PixelRef operator[](index_t i) {
            return PixelRef(pixel_index_ + i, li_);
        }

        friend iterator operator+(iterator lhs, index_t i) {
            return iterator{lhs.pixel_index_ + i, lhs.li_};
        }

        friend bool operator==(const iterator& lhs, const iterator& rhs) {
            return lhs.pixel_index_ == rhs.pixel_index_;
        }

        friend bool operator!=(const iterator& lhs, const iterator& rhs) {
            return !(lhs == rhs);
        }

       private:
        iterator(index_t pixel_index, LidarScan* li)
            : pixel_index_{pixel_index}, li_{li} {}
        index_t pixel_index_;
        LidarScan* li_;
        friend class LidarScan;
    };

    struct PixelRef {
        void operator=(const Pixel& p) {
            const index_t hh = li_->h;
            index_t u = pixel_index_ % hh;
            index_t v = pixel_index_ / hh;
            li_->range()(li_->ind(u, v)) = p.range;
            li_->intensity()(li_->ind(u, v)) = p.intensity;
            li_->noise()(li_->ind(u, v)) = p.noise;
            li_->reflectivity()(li_->ind(u, v)) = p.reflectivity;
            li_->ts[v] = p.ts;
        }

       private:
        PixelRef(index_t pixel_index, LidarScan* li)
            : pixel_index_{pixel_index}, li_{li} {}
        index_t pixel_index_;
        LidarScan* li_;
        friend class LidarScan;
    };
};

/**
 * Generate a matrix of unit vectors pointing radially outwards, useful for
 * efficiently computing cartesian coordinates from ranges.  The result is a
 * n x 3 array of doubles stored in column-major order where each row is the
 * unit vector corresponding to the nth point in a lidar scan, with 0 <= n <
 * h * w.
 * The ordering of the rows is consistent with LidarScan::ind(u, v) for the
 * 3D point corresponding to the pixel at row u, column v in the LidarScan.
 * @param w number of columns in the lidar scan. e.g. 512, 1024, or 2048.
 * @param h number of rows in the lidar scan. e.g. 64 for the OS1-64
 * @param range_unit the unit, in meters, of the range,  e.g. OS1::range_unit
 * @param azimuth_angles_deg azimuth offsets in degrees for each of h beams
 * @param altitude_angles_Deg altitude in degrees for each of h beams
 * @return xyz direction unit vectors for each point in the lidar scan
 */
inline Points make_xyz_lut(const LidarScan::index_t w,
                           const LidarScan::index_t h, const double range_unit,
                           const std::vector<double>& azimuth_angles_deg,
                           const std::vector<double>& altitude_angles_deg) {
    Eigen::ArrayXd azimuth(w * h);
    Eigen::ArrayXd altitude(w * h);
    for (LidarScan::index_t v = 0; v < w; v++) {
        for (LidarScan::index_t u = 0; u < h; u++) {
            LidarScan::index_t i = u * w + v;
            azimuth(i) =
                azimuth_angles_deg[u] * M_PI / 180.0 + 2.0 * M_PI * v / w;
            altitude(i) = altitude_angles_deg[u] * M_PI / 180.0;
        }
    }
    Points xyz(w * h, 3);
    xyz.col(0) = altitude.cos() * azimuth.cos();
    xyz.col(1) = -altitude.cos() * azimuth.sin();
    xyz.col(2) = altitude.sin();
    return xyz * range_unit;
}

/**
 * Convert LidarScan to cartesian points.
 * @param scan a LidarScan
 * @param xyz_lut a lookup table of unit vectors generated by make_xyz_lut
 * @return cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = scan.ind(u, v) for a pixel
 *         at row u, column v.
 */
inline Points cartesian(const LidarScan& scan, const Points& xyz_lut) {
    return xyz_lut.colwise() * scan.range().cast<double>();
}

}  // namespace ouster

/**
 * @file
 * @brief Holds lidar data by field in column-major order
 */

#pragma once

#include <utility>
#include <vector>

namespace ouster {

struct LidarScan {
    const ssize_t W;
    const ssize_t H;
    std::vector<double> range;
    std::vector<double> intensity;
    std::vector<double> reflectivity;
    std::vector<double> noise;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    LidarScan(size_t w, size_t h) : W(w), H(h) {
        range.reserve(W * H);
        intensity.reserve(W * H);
        reflectivity.reserve(W * H);
        noise.reserve(W * H);
        x.reserve(W * H);
        y.reserve(W * H);
        z.reserve(W * H);
    };

    void clear() {
        range.clear();
        intensity.clear();
        reflectivity.clear();
        noise.clear();
        x.clear();
        y.clear();
        z.clear();
    }

    // x, y, z, (padding), i, ts, reflectivity, ring, noise, range (mm)
    using value_type = std::tuple<float, float, float, float, float, float,
                                  uint16_t, uint8_t, uint16_t, uint32_t>;

    /**
     * For use with os1_util.h:batch_to_iter
     **/
    void push_back(const value_type& data) {
        x.push_back(std::get<0>(data));
        y.push_back(std::get<1>(data));
        z.push_back(std::get<2>(data));
        intensity.push_back(std::get<4>(data));
        reflectivity.push_back(std::get<6>(data));
        noise.push_back(std::get<8>(data));
        range.push_back(std::get<9>(data));
    }
};
}

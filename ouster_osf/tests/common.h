/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <random>
#include <string>

#include "compat_ops.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/stream_lidar_scan.h"

namespace ouster {
namespace osf {

constexpr double TEST_EPS = 1e-8;
constexpr char OSF_OUTPUT_DIR[] = "test_osf";

using idx = std::ptrdiff_t;

inline bool get_test_data_dir(std::string& test_data_dir) {
    std::string test_data_dir_var;
    if (get_env_var("DATA_DIR", test_data_dir_var)) {
        // printf("DATA_DIR: %s\n", test_data_dir_var);
        if (!test_data_dir_var.empty()) {
            if (!is_dir(test_data_dir_var)) {
                printf("WARNING: DATA_DIR: %s doesn't exist\n",
                       test_data_dir_var.c_str());
                return false;
            }
            test_data_dir = test_data_dir_var;
            return true;
        }
    }
    printf("ERROR: DATA_DIR env var: is not set for OSF tests\n");
    return false;
}

/// Get output dir for test files
bool get_output_dir(std::string& output_dir) {
    std::string build_dir;
    if (get_env_var("BUILD_DIR", build_dir)) {
        if (!build_dir.empty()) {
            if (!is_dir(build_dir)) {
                printf("ERROR: BUILD_DIR: %s doesn't exist yet\n",
                       build_dir.c_str());
                return false;
            }
            output_dir = std::string{build_dir} + "/" + OSF_OUTPUT_DIR;
        }
    } else {
        output_dir = OSF_OUTPUT_DIR;
    }
    if (!is_dir(output_dir)) {
        if (!make_dir(output_dir)) {
            printf("ERROR: Can't create output_dir: %s\n", output_dir.c_str());
            return false;
        }
    }
    return true;
}

inline double normal_d(const double m, const double s) {
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<double> d{m, s};
    return d(gen);
}

inline uint32_t normal_d_bounded(const double m, const double s,
                                 const double b = 1 << 20) {
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<double> d{m, s};
    double g = d(gen);
    while (g < 0 || g >= b) g = d(gen);
    return static_cast<uint32_t>(g);
}

// TODO: Extract all this to data generator with rd, gen etc cached
template <size_t N>
std::array<double, N> normal_arr(const double& m, const double& s) {
    std::array<double, N> arr;
    for (size_t i = 0; i < N; ++i) {
        arr[i] = normal_d(m, s);
    }
    return arr;
}

// set field to random values, with mask_bits specifienging the number of
// bits to mask
struct set_to_random {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field_dest, size_t mask_bits = 0) {
        field_dest = field_dest.unaryExpr([=](T) {
            double sr = static_cast<double>(std::rand()) / RAND_MAX;
            return static_cast<T>(
                sr * static_cast<double>(std::numeric_limits<T>::max()));
        });
        if (mask_bits && sizeof(T) * 8 > mask_bits) {
            field_dest = field_dest.unaryExpr([=](T a) {
                return static_cast<T>(a & ((1LL << mask_bits) - 1));
            });
        }
    }
};

inline void random_lidar_scan_data(LidarScan& ls) {
    using sensor::ChanField;
    using sensor::ChanFieldType;

    for (auto f : ls) {
        if (f.first == sensor::ChanField::RANGE ||
            f.first == sensor::ChanField::RANGE2) {
            // Closer to reality that RANGE is just 20bits and not all 32
            ouster::impl::visit_field(ls, f.first, set_to_random(), 20);
        } else {
            ouster::impl::visit_field(ls, f.first, set_to_random());
        }
    }

    ls.frame_id = 5;

    const int32_t columns_per_packet = ls.w / ls.packet_timestamp().size();
    const int64_t t_start = 100;
    const int64_t t_start_p = 100000000000;
    const int64_t dt = 100 * 1000 / (ls.w - 1);
    const int64_t dt_p = 100 * 1000 / (ls.packet_timestamp().size() - 1);
    for (ptrdiff_t i = 0; i < ls.w; ++i) {
        if (i == 0)
            ls.timestamp()[i] = t_start;
        else
            ls.timestamp()[i] = ls.timestamp()[i - 1] + dt;
        ls.status()[i] = static_cast<uint32_t>(
            (std::numeric_limits<uint32_t>::max() / ls.w) * i);
        ls.measurement_id()[i] = static_cast<uint16_t>(
            (std::numeric_limits<uint16_t>::max() / ls.w) * i);
        ls.pose()[i] = ouster::mat4d::Random();

        const int32_t pi = i / columns_per_packet;
        if (pi == 0)
            ls.packet_timestamp()[pi] = t_start_p;
        else
            ls.packet_timestamp()[pi] = ls.packet_timestamp()[pi - 1] + dt_p;
    }
}

inline LidarScan get_random_lidar_scan(
    const size_t w = 1024, const size_t h = 64,
    sensor::UDPProfileLidar profile =
        sensor::UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL) {
    LidarScan ls{w, h, profile};
    random_lidar_scan_data(ls);
    return ls;
}

inline LidarScan get_random_lidar_scan(const size_t w = 1024,
                                       const size_t h = 64,
                                       LidarScanFieldTypes field_types = {}) {
    LidarScan ls{w, h, field_types.begin(), field_types.end()};
    random_lidar_scan_data(ls);
    return ls;
}

inline LidarScan get_random_lidar_scan(const sensor::sensor_info& si) {
    return get_random_lidar_scan(si.format.columns_per_frame,
                                 si.format.pixels_per_column,
                                 si.format.udp_profile_lidar);
}

}  // namespace osf
}  // namespace ouster

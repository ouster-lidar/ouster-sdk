#pragma once

#include <gtest/gtest.h>
#include <sys/stat.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <random>
#include <string>

#include "ouster/lidar_scan.h"
#include "ouster/osf/util.h"

namespace ouster {
namespace OSF {

constexpr double TEST_EPS = 1e-8;
constexpr char OSF_OUTPUT_DIR[] = "test_osf";

using idx = std::ptrdiff_t;

bool is_dir(const std::string& path) {
    struct stat statbuf;
    if (stat(path.c_str(), &statbuf) != 0) {
        if (errno != ENOENT) printf("ERROR: stat: %s", std::strerror(errno));
        return false;
    }
    return S_ISDIR(statbuf.st_mode);
}

bool path_exists(const std::string& path) {
    struct stat sb;
    if (stat(path.c_str(), &sb) != 0) return false;
    return true;
}

bool get_test_data_dir(std::string* test_data_dir) {
    char* test_data_dir_var;
    if ((test_data_dir_var = std::getenv("TEST_DATA_DIR")) != nullptr) {
        // printf("TEST_DATA_DIR: %s\n", test_data_dir_var);
        if (strlen(test_data_dir_var) > 0) {
            if (!is_dir(test_data_dir_var)) {
                printf("WARNING: TEST_DATA_DIR: %s doesn't exist\n",
                       test_data_dir_var);
                return false;
            }
            *test_data_dir = std::string{test_data_dir_var};
            return true;
        }
    }
    printf("ERROR: TEST_DATA_DIR env var: is not set\n");
    return false;
}

bool get_output_dir(std::string* output_dir) {
    char* build_dir;
    if ((build_dir = std::getenv("BUILD_DIR")) != nullptr) {
        if (strlen(build_dir) > 0) {
            if (!is_dir(build_dir)) {
                printf("ERROR: BUILD_DIR: %s doesn't exist\n", build_dir);
                return false;
            }
            *output_dir = std::string{build_dir} + "/" + OSF_OUTPUT_DIR;
        }
    } else {
        *output_dir = OSF_OUTPUT_DIR;
    }
    if (!is_dir(*output_dir)) {
        if (mkdir(output_dir->c_str(), 0777) != 0) {
            printf("ERROR: Can't create output_dir: %s\n", output_dir->c_str());
            return false;
        }
    }

    return true;
}

inline std::string tmp_dir() {
    char tmpdir[] = "/tmp/osf-test.XXXXXX";
    if (::mkdtemp(tmpdir) == nullptr) {
        std::cerr << "Can't create temp dir\n";
        ADD_FAILURE();
    };
    return {tmpdir};
}

double normal_d(const double m, const double s) {
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<double> d{m, s};
    return d(gen);
}

uint32_t normal_d_bounded(const double m, const double s,
                          const double b = 1 << 20) {
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<double> d{m, s};
    double g = d(gen);
    while (g < 0 || g >= b) g = d(gen);
    return g;
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

LidarScan get_random_lidar_scan(const size_t w = 1024, const size_t h = 64) {
    LidarScan ls{w, h};

    for (size_t i = 0; i < static_cast<size_t>(ls.w * ls.h); ++i) {
        ls.data(i, LidarScan::LidarScanIndex::RANGE) =
            normal_d_bounded(10000, 2000);
        ls.data(i, LidarScan::LidarScanIndex::INTENSITY) =
            normal_d_bounded(10000, 5000);
        ls.data(i, LidarScan::LidarScanIndex::NOISE) =
            normal_d_bounded(8000, 1000);
        ls.data(i, LidarScan::LidarScanIndex::REFLECTIVITY) =
            normal_d_bounded(2000, 500);
    }

    const int64_t t_start = 100;
    const int64_t dt = 100 * 1000 / (ls.w - 1);
    for (size_t i = 0; i < static_cast<size_t>(ls.w); ++i) {
        if (i == 0)
            ls.ts[i] = std::chrono::nanoseconds(t_start);
        else
            ls.ts[i] = std::chrono::nanoseconds(ls.ts[i - 1].count() + dt);
    }

    return ls;
}

LidarScan get_random_lidar_scan(const sensor& s) {
    return get_random_lidar_scan(s.meta.format.columns_per_frame,
                                 s.meta.format.pixels_per_column);
}

ouster::OSF::Gps get_random_gps_waypoint() {
    ouster::OSF::Gps gps{};
    gps.latitude = normal_d(10, 2);
    gps.longitude = normal_d(20, 4);
    gps.altitude = normal_d(1, 0.2);
    gps.epy = normal_d(50, 2);
    gps.epx = normal_d(60, 2);
    gps.epv = normal_d(70, 2);
    gps.ept = normal_d(100, 20);
    gps.speed = normal_d(25, 5);
    gps.eps = normal_d(150, 5);
    gps.track = normal_d(15, 5);
    gps.epd = normal_d(7, 3);
    gps.climb = normal_d(70, 3);
    gps.epc = normal_d(2, 0.1);
    return gps;
}

ouster::OSF::osf_poses get_random_poses(const idx w) {
    ouster::OSF::osf_poses poses;
    for (idx i = 0; i < w; ++i) {
        ouster::OSF::pose p{};
        p.orientation = Eigen::Quaternionf::UnitRandom();
        p.position = Eigen::Translation<double, 3>(Eigen::Vector3d::Random() *
                                                   normal_d(100, 40));
        poses.emplace_back(p);
    }
    return poses;
}

ouster::OSF::osf_poses get_random_poses(const sensor& s) {
    return get_random_poses(s.meta.format.columns_per_frame);
}

}  // namespace OSF
}  // namespace ouster

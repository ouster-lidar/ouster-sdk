#include <cmath>
#include <vector>

#include "ouster/os1_packet.h"

namespace ouster {
namespace OS1 {

extern const std::vector<double> beam_altitude_angles = {
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

extern const std::vector<double> beam_azimuth_angles = {
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};

extern const std::vector<double> imu_to_sensor_transform = {
    1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1};

extern const std::vector<double> lidar_to_sensor_transform = {
    -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1};

std::vector<double> make_xyz_lut(int W, int H,
                                 const std::vector<double>& azimuth_angles,
                                 const std::vector<double>& altitude_angles) {
    const int n = W * H;
    std::vector<double> xyz = std::vector<double>(3 * n, 0);

    for (int icol = 0; icol < W; icol++) {
        double h_angle_0 = 2.0 * M_PI * icol / W;
        for (int ipx = 0; ipx < H; ipx++) {
            int ind = 3 * (icol * H + ipx);
            double h_angle =
                (azimuth_angles.at(ipx) * 2 * M_PI / 360.0) + h_angle_0;

            xyz[ind + 0] = std::cos(altitude_angles[ipx] * 2 * M_PI / 360.0) *
                           std::cos(h_angle);
            xyz[ind + 1] = -std::cos(altitude_angles[ipx] * 2 * M_PI / 360.0) *
                           std::sin(h_angle);
            xyz[ind + 2] = std::sin(altitude_angles[ipx] * 2 * M_PI / 360.0);
        }
    }
    return xyz;
}

std::vector<int> get_px_offset(int lidar_mode) {
    auto repeat = [](int n, const std::vector<int>& v) {
        std::vector<int> res{};
        for (int i = 0; i < n; i++) res.insert(res.end(), v.begin(), v.end());
        return res;
    };

    switch (lidar_mode) {
        case 512:
            return repeat(16, {0, 3, 6, 9});
        case 1024:
            return repeat(16, {0, 6, 12, 18});
        case 2048:
            return repeat(16, {0, 12, 24, 36});
        default:
            return std::vector<int>{64, 0};
    }
}
}
}

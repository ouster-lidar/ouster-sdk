#include "representations.cpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/packet.h"
#include "ouster/core/types.h"
#include "ouster/pcap/pcap_frame_set_source.h"

using namespace ouster::sdk;

namespace {

std::string env_or_empty(const char* key) {
    const char* value = std::getenv(key);
    return (value && *value) ? std::string(value) : std::string{};
}

std::string data_root() {
    const auto from_env = env_or_empty("DATA_DIR");
    if (!from_env.empty()) {
        return from_env;
    }
#ifdef OUSTER_SDK_SOURCE_DIR
    return std::string(OUSTER_SDK_SOURCE_DIR) + "/tests";
#else
    throw std::runtime_error(
        "DATA_DIR environment variable not set and "
        "OUSTER_SDK_SOURCE_DIR is undefined.");
#endif
}

struct FixturePaths {
    std::string pcap;
    std::string json;
};

FixturePaths test_fixture() {
    const auto root = data_root();
    return {
        root + "/pcaps/OS-1-128_v2.3.0_1024x10_lb_n3.pcap",
        root + "/pcaps/OS-1-128_v2.3.0_1024x10.json",
    };
}

core::LidarFrame load_full_frame(const FixturePaths& paths) {
    auto source = ouster::sdk::open_source(paths.pcap);
    auto it = source.begin();
    return *(*it)[0];
}

}  // namespace

TEST(ProcessingRepresentationsCpp, MetadataProjectionDoesNotThrow) {
    const auto paths = test_fixture();
    auto info = core::metadata_from_json(paths.json);
    auto frame = load_full_frame(paths);

    ASSERT_NO_THROW(ouster::docs::project_with_metadata_extrinsics(*frame.sensor_info, frame));
}

TEST(ProcessingRepresentationsCpp, MetadataTransformMatchesManualComputation) {
    const auto paths = test_fixture();
    auto frame = load_full_frame(paths);

    const auto transformed =
        ouster::docs::apply_metadata_transform_xyzlut(*frame.sensor_info, frame);

    core::mat4d transform = core::mat4d::Identity();
    transform(2, 2) = -1.0;
    transform(1, 1) = -1.0;
    transform(2, 3) = 20000;  // millimetres
    transform(0, 3) = 1500;   // millimetres

    const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>().transpose();
    const Eigen::Vector3d translation_m = transform.topRightCorner<3, 1>() * core::RANGE_UNIT;

    auto base_lut = core::XYZLut(*frame.sensor_info, /*use_extrinsics=*/false);
    core::PointCloudXYZd manual = base_lut(frame);
    manual = manual * rotation;
    manual.rowwise() += translation_m.transpose();

    const auto range_field = frame.field<uint32_t>(core::ChanField::RANGE);
    const size_t h = frame.h;
    const size_t w = frame.w;
    for (size_t row = 0; row < h; ++row) {
        for (size_t col = 0; col < w; ++col) {
            if (range_field(row, col) == 0) {
                const Eigen::Index idx = static_cast<Eigen::Index>(row * w + col);
                manual.row(idx).setZero();
            }
        }
    }

    const Eigen::MatrixXd diff = (transformed - manual).cwiseAbs();
    const double max_diff = diff.maxCoeff();
    std::cout << "[DEBUG] MetadataTransform max diff: " << max_diff << std::endl;

    if (max_diff >= 1e-6) {
        Eigen::Index max_row, max_col;
        diff.maxCoeff(&max_row, &max_col);
        std::cout << "[DEBUG] Largest difference at idx " << max_row << ", component " << max_col
                  << std::endl;
        std::cout << "[DEBUG] transformed: " << transformed.row(max_row) << std::endl;
        std::cout << "[DEBUG] manual     : " << manual.row(max_row) << std::endl;
    }

    EXPECT_LT(max_diff, 1e-6);
}

TEST(ProcessingRepresentationsCpp, FilterPointsByRangeAndAzimuth) {
    const auto paths = test_fixture();
    auto frame = load_full_frame(paths);

    const double range_min_m = 2.0;
    auto filtered =
        ouster::docs::filter_points_by_range_and_azimuth(*frame.sensor_info, frame, range_min_m);

    // 1. Verify we have output data
    ASSERT_GT(filtered.rows(), 0);

    // 2. Verify Azimuth filtering (Size Check)
    // The function keeps the first 75% of columns.
    const size_t h = frame.h;
    const size_t w = frame.w;
    const size_t max_expected_points = static_cast<size_t>(w * 0.75 * h);

    ASSERT_LE(filtered.rows(), static_cast<Eigen::Index>(max_expected_points))
        << "Result contains more points than the azimuth window allows";

    // 3. Verify Range filtering (Value Check)
    // Iterate through the result and ensure every point meets the criteria.
    for (Eigen::Index i = 0; i < filtered.rows(); ++i) {
        const Eigen::Vector3d point = filtered.row(i).transpose();
        const double range = point.norm();

        // Skip zero points (masked out)
        if (range < 1e-6) continue;

        // Check if point distance is greater than or equal to the threshold
        // (Using a small epsilon for floating point comparisons)
        EXPECT_GE(range, range_min_m - 1e-6)
            << "Point at index " << i << " is closer (" << range
            << "m) than the minimum threshold (" << range_min_m << "m)";
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

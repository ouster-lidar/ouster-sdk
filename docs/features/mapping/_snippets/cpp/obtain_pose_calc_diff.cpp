#include <ouster/core/open_source.h>
#include <ouster/mapping/slam_engine.h>

#include <Eigen/Dense>
#include <cstdint>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace docs {

using namespace ::ouster::sdk;

using PoseDiff = std::tuple<Eigen::Matrix3d, Eigen::Vector3d>;

// clang-format off
//! [doc-stag-pose-diff]
PoseDiff pose_differences(const core::Matrix4dR& last_frame_pose,
                          const core::Matrix4dR& frame_pose) {
    auto pose_diff = last_frame_pose.inverse() * frame_pose;
    auto rotation_diff = pose_diff.block<3, 3>(0, 0);
    auto translation_diff = pose_diff.block<3, 1>(0, 3);
    return {rotation_diff, translation_diff}; }
//! [doc-etag-pose-diff]
// clang-format on

struct PoseDelta {
    int idx;
    std::int64_t timestamp;
    core::Matrix4dR pose;
    Eigen::Matrix3d rotation_diff;
    Eigen::Vector3d translation_diff;
};

std::vector<PoseDelta> iterate_pose_differences(core::AnyFrameSetSource& data_source,
                                                mapping::SlamEngine& slam) {
    std::vector<PoseDelta> deltas;
    core::Matrix4dR last_frame_pose = core::Matrix4dR::Identity();
    int idx = 0;

    // clang-format off
    //! [doc-stag-pose-diff-call]
    for (auto frame_set : data_source) {
        slam.update(frame_set);

        const auto& frame = frame_set[0];
        int col = 0;
        try {
            col = frame->get_first_valid_column();
        } catch (const std::exception&) {
            ++idx;
            continue;
        }

        core::Matrix4dR frame_pose = frame->get_column_pose(col);
        auto rotation_and_translation_diff = pose_differences(last_frame_pose, frame_pose);
        auto rotation_diff = std::get<0>(rotation_and_translation_diff);
        auto translation_diff = std::get<1>(rotation_and_translation_diff);
        last_frame_pose = frame_pose;
        //! [doc-etag-pose-diff-call]
        // clang-format on

        auto frame_ts = frame->timestamp()[col];
        deltas.push_back({idx, static_cast<std::int64_t>(frame_ts), frame_pose, rotation_diff,
                          translation_diff});
        ++idx;
    }

    return deltas;
}

std::pair<std::unique_ptr<mapping::SlamEngine>, core::AnyFrameSetSource> build_default_slam(
    const std::string& source_file_path) {
    auto source = open_source(source_file_path, {}, true, -1);
    mapping::LIOSlamConfig config;
    config.min_range = 1.0;
    config.max_range = 50.0;
    config.voxel_size = 0.5;
    auto slam = mapping::SlamEngine::create(source.sensor_info(), config);
    return {std::move(slam), std::move(source)};
}

void run_pose_difference_report(const std::string& source_file_path) {
    auto built = build_default_slam(source_file_path);
    auto& slam = *built.first;
    auto& source = built.second;

    auto deltas = iterate_pose_differences(source, slam);

    for (const auto& delta : deltas) {
        std::cout << "idx = " << delta.idx << " at timestamp " << delta.timestamp
                  << " has the pose\n"
                  << delta.pose << '\n';
        std::cout << "Rotation difference:\n" << delta.rotation_diff << '\n';
        std::cout << "Translation difference: " << delta.translation_diff.transpose() << "\n";
    }
}

}  // namespace docs
}  // namespace sdk
}  // namespace ouster

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: obtain_pose_calc_diff <source_file>" << '\n';
        return (argc == 1) ? 0 : 1;
    }

    ouster::sdk::docs::run_pose_difference_report(argv[1]);
    return 0;
}

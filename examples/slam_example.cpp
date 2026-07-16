#ifdef OUSTER_OSF
#include "ouster/osf/osf_frame_set_source.h"
#endif
#ifdef OUSTER_PCAP
#include "ouster/pcap/pcap_frame_set_source.h"
#endif
#ifdef OUSTER_SENSOR
#include "ouster/sensor/sensor_frame_set_source.h"
#endif
// NOLINTBEGIN(google-build-using-namespace)
//! [doc-stag-slam-imports]
#include <ouster/core/open_source.h>
#include <ouster/mapping/slam_engine.h>
using namespace ouster::sdk;
//! [doc-etag-slam-imports]
// NOLINTEND(google-build-using-namespace)

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: slam_example <source_file>" << std::endl;
        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string source_file = argv[1];

    // clang-format off
    //! [doc-stag-slam-open]
    mapping::LIOSlamConfig slam_config;
    slam_config.min_range = 0.5;          // Minimum range 0.5 meters
    slam_config.max_range = 100.0;        // Maximum range 100.0 meters
    slam_config.deskew_method = "auto";   // Let the system choose the deskewing method
    //! [doc-etag-slam-open]

    //! [doc-stag-slam-engine]
    auto source = open_source(source_file);
    auto slam_engine = mapping::SlamEngine::create(
        source.sensor_info(),
        slam_config);
    //! [doc-etag-slam-engine]
    // clang-format on

    // Initialize the SLAM engine
    // The SlamEngine processes the frames to estimate the new state (pose)
    // and updates the column poses for each frame.
    auto to_degrees = [](double rad) { return rad * 180.0 / M_PI; };
    // clang-format off
    //! [doc-stag-slam-loop]
    //! [doc-stag-slam-loop-update]
    for (auto frame_set : source) {
        slam_engine->update(frame_set);
        //! [doc-etag-slam-loop-update]
        //! [doc-stag-slam-loop-printpose]
        const auto& frame = frame_set[0];
        // Get last valid column (closest to the current pose)
        int col = frame->get_last_valid_column();
        // Get timestamp and pose for the column
        auto frame_pose = frame->get_column_pose(col);
        auto frame_ts = frame->timestamp()[col];
        //! [doc-etag-slam-loop-printpose]
        // Extract translation
        Eigen::Vector3d translation = frame_pose.block<3, 1>(0, 3);
        // ZYX euler: yaw, pitch, roll
        Eigen::Matrix3d rot = frame_pose.block<3, 3>(0, 0);
        auto angles = rot.eulerAngles(2, 1, 0);
        double yaw = angles[0];
        double pitch = angles[1];
        double roll = angles[2];
        std::cout << "idx = " << frame->frame_id << "; frame_ts = " << frame_ts
                  << "; Translation: " << std::fixed << std::setprecision(2)
                  << translation[0] << ", " << translation[1] << ", " << translation[2]
                  << " (Roll: " << to_degrees(roll)
                  << ", Pitch: " << to_degrees(pitch)
                  << ", Yaw: " << to_degrees(yaw) << ")\n"; }
    //! [doc-etag-slam-loop]
    // clang-format on
    std::cout << "SLAM processing complete." << std::endl;
    return EXIT_SUCCESS;
}

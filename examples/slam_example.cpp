#include <ouster/open_source.h>
#ifdef OUSTER_OSF
#include "ouster/osf/osf_scan_source.h"
#endif
#ifdef OUSTER_PCAP
#include "ouster/pcap_scan_source.h"
#endif
#ifdef OUSTER_SENSOR
#include "ouster/sensor_scan_source.h"
#endif
#include <ouster/slam_engine.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace ouster::sdk;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: slam_example <source_file>" << std::endl;
        return (argc == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string source_file = argv[1];

    // open source file non-collated
    auto source = open_source(
        source_file, [](auto& source_options) { source_options.index = true; },
        true);

    mapping::SlamConfig slam_config;
    slam_config.backend = "kiss";  // Currently only "kiss" backend is available
    slam_config.deskew_method =
        "auto";                   // Let the system choose the deskewing method
    slam_config.min_range = 0.5;  // Minimum range 0.5 meters
    slam_config.max_range = 100.0;  // Maximum range 100.0 meters

    // Initialize the SLAM engine
    mapping::SlamEngine slam_engine(source.sensor_info(), slam_config);

    for (auto scans : source) {
        // The SlamEngine processes the scans to estimate the new state (pose)
        // and updates the column poses for each scan.
        slam_engine.update(scans);
        const auto& scan = scans[0];
        // Get last valid column (closest to the current pose)
        int col = scan->get_last_valid_column();
        // Get timestamp and pose for the column
        auto scan_ts = scan->timestamp()[col];
        core::Matrix4dR scan_pose = scan->get_column_pose(col);

        // Extract translation
        Eigen::Vector3d translation = scan_pose.block<3, 1>(0, 3);
        // Extract roll, pitch, yaw from rotation matrix
        Eigen::Matrix3d rot = scan_pose.block<3, 3>(0, 0);

        Eigen::Vector3d angles =
            rot.eulerAngles(2, 1, 0);  // ZYX order: yaw, pitch, roll
        double yaw = angles[0];
        double pitch = angles[1];
        double roll = angles[2];
        auto to_degrees = [](double rad) { return rad * 180.0 / M_PI; };
        std::cout << "idx = " << scan->frame_id << "; scan_ts = " << scan_ts
                  << "; Translation: " << std::fixed << std::setprecision(2)
                  << translation[0] << ", " << translation[1] << ", "
                  << translation[2] << " (Roll: " << to_degrees(roll)
                  << ", Pitch: " << to_degrees(pitch)
                  << ", Yaw: " << to_degrees(yaw) << ")\n";
    }
    std::cout << "SLAM processing complete." << std::endl;
    return EXIT_SUCCESS;
}

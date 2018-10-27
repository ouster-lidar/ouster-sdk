#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include "ouster/lidar_scan.h"

namespace ouster {
namespace viz {

using Points = Eigen::ArrayX3d;

/**
 * Handle to visualizer state
 **/
struct VizHandle;

/**
 * Struct used to store user-specified information for visualization
 **/
struct UserConfig {
    double intensity_scale = 1.0;
    double range_scale = 1.0;
    double noise_scale = 1.0;
    bool image_noise = false;
};

/**
 * Struct used to store specifics about a particular sensor's intrinsics
 **/
struct SensorSpecifics {
    int col_per_rev;
    int H;
    std::vector<float> beam_azimuth_angles;
    std::vector<float> beam_altitude_angles;
};

/**
 * Initialize an instance of the visualizer
 * @param xyz_lut, a table for computing xyz coordinates from a lidar scan
 * @param uc a struct containing user-specified data about the visualization
 * @param ss a struct containing information about the format of the data
 * @return a handle to the state of the visualizer
 **/
std::shared_ptr<VizHandle> init_viz(const std::vector<double>& xyz_lut,
                                    const UserConfig& uc,
                                    const SensorSpecifics& ss);

/**
 * Update the lidar scan being displayed by the visualizer
 * @param vh a handle to a visualizer returned by init_viz()
 * @param lidar_scan the lidar scan which is added to continiously by the
 * polling thread
 * @return void
 **/
void update_poll(viz::VizHandle& vh,
                 std::unique_ptr<ouster::LidarScan>& lidar_scan);

/**
 * Run visualizer render loop
 * @param vh handle to visualizer state returned by init_viz()
 * @return void when the user exits the visualizer
 **/
void run_viz(VizHandle& vh);
}
}

/**
 * @file
 * @brief Sample OS-1 data visualizer
 */

#pragma once

#include <memory>
#include <vector>

#include "ouster/lidar_scan.h"

namespace ouster {
namespace viz {

const std::vector<double> default_range_radii = {5.0,  10.0, 15.0,
                                                 20.0, 40.0, 80.0};

/**
 * Handle to visualizer state
 */
struct VizHandle;

/**
 * Initialize an instance of the visualizer
 * @param w horizontal resolution, usually one of 512, 1024, or 2048
 * @param h vertical resolution, always 64 for OS1 family
 * @return a handle to the state of the visualizer
 */
std::shared_ptr<VizHandle> init_viz(
    const LidarScan::index_t w, const LidarScan::index_t h,
    const Points* xyz_lut, const std::string& prod_line,
    const std::vector<double>& range_radii = default_range_radii);

/**
 * Update the lidar scan being displayed by the visualizer
 * @param vh a handle to a visualizer returned by init_viz()
 * @param lidar_scan the lidar scan to visualize on the next frame
 */
void update(viz::VizHandle& vh, std::unique_ptr<LidarScan>& lidar_scan);

/**
 * Run visualizer render loop
 * @param vh handle to visualizer state returned by init_viz()
 * @return void when the user exits the visualizer
 */
void run_viz(VizHandle& vh);

/**
 * Shut down visualizer, making previous call to run_viz return(). Thread safe.
 * @param vh handle to visualizer state returned by init_viz()
 */
void shutdown(VizHandle& vh);
}  // namespace viz
}  // namespace ouster

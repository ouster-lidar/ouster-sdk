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

/**
 * Handle to visualizer state
 */
struct VizHandle;

/**
 * Initialize an instance of the visualizer
 * @param W horizontal resolution, usually one of 512, 1024, or 2048
 * @param H vertical resolution, always 64 for OS1 family
 * @return a handle to the state of the visualizer
 */
std::shared_ptr<VizHandle> init_viz(int W, int H);

/**
 * Update the lidar scan being displayed by the visualizer
 * @param vh a handle to a visualizer returned by init_viz()
 * @param lidar_scan the lidar scan to visualize on the next frame
 */
void update(viz::VizHandle& vh, std::unique_ptr<ouster::LidarScan>& lidar_scan);

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
}
}

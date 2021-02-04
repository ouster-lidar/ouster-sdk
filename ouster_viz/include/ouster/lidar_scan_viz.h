/**
 * @file
 * @brief Update function for simple point cloud visualizer
 */
#pragma once

#include <GL/glew.h>

#include <atomic>
#include <vector>

#include "ouster/image_processing.h"
#include "ouster/lidar_scan.h"
#include "ouster/point_viz.h"
#include "ouster/types.h"
#include "ouster/version.h"

namespace ouster {
namespace viz {

/**
 * Helper class to visualize LidarScan.
 */
class LidarScanViz {
    AutoExposure range_ae;
    AutoExposure intensity_ae;
    AutoExposure ambient_ae;
    AutoExposure reflectivity_ae;
    BeamUniformityCorrector ambient_buc;
    util::version firmware_version;
    const std::vector<int> px_offset;
    const double aspect_ratio;
    const size_t h, w;
    std::vector<GLfloat> imdata;
    std::atomic_bool show_ambient;
    std::atomic_int display_mode;
    std::atomic_bool cycle_range;

    PointViz& point_viz;

   public:
    /**
     * Set up the LidarScanViz with a reference to the PointViz.
     *
     * Please note that the point_viz_ is stored as a reference and should be
     * kept in scope for the lifetime of the LidarScanViz.
     *
     * @param info Sensor metadata, e.g. from parse_metadata, containing
     * @param point_viz_ Reference to PointViz
     */
    LidarScanViz(const sensor::sensor_info& info, PointViz& point_viz_);

    /**
     * Render both image view and point cloud view to the point_viz.
     * 
     * If poses are available, set bool cloud_swap to false and then
     * call point_viz.cloudSwap() manually
     *
     * @param ls the scan to visualize
     * @param which_cloud index of the cloud to update with ls data
     * @param cloud_swap swap in new cloud immediately
     * @param show_image display the image view for this cloud
     */
    void draw(const LidarScan& ls, const size_t which_cloud = 0,
              const bool cloud_swap = true, const bool show_image = true);
};

}  // namespace viz
}  // namespace ouster

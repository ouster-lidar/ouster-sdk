/**
 * @file
 * @brief Update function for simple point cloud visualizer
 */
#pragma once

#include <map>
#include <mutex>
#include <set>
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
    using field_proc = std::function<void(Eigen::Ref<img_t<double>>)>;

    std::mutex mx;
    util::version firmware_version;
    const std::vector<int> px_offset;
    const double aspect_ratio;
    const size_t h, w;
    img_t<uint32_t> range1, range2;
    std::map<sensor::ChanField, img_t<double>> field_data;
    std::map<sensor::ChanField, std::vector<field_proc>> field_procs;
    std::set<sensor::ChanField> active_fields;
    std::vector<sensor::ChanField> available_fields;
    std::vector<float> imdata1;
    std::vector<float> imdata2;
    bool cloud1_enabled;
    bool cloud2_enabled;
    float point_size;
    int display_mode;
    bool display_mode_changed;
    int image_ind1;
    int image_ind2;
    int size_fraction{6};
    AutoExposure range_ae;
    AutoExposure intensity_ae;
    AutoExposure ambient_ae;
    AutoExposure reflectivity_ae;
    BeamUniformityCorrector ambient_buc;

    std::shared_ptr<Cloud> cloud1;
    std::shared_ptr<Cloud> cloud2;
    std::shared_ptr<Image> image1;
    std::shared_ptr<Image> image2;

    PointViz& point_viz;

    bool key_handler(const viz::PointViz::HandlerCtx& wc, int key, int mods);
    void cycle_display_mode();
    void cycle_field_2d(int& ind);
    void change_size_fraction(int amount);

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
     */
    void draw(const LidarScan& ls, const size_t which_cloud = 0);
};

}  // namespace viz
}  // namespace ouster

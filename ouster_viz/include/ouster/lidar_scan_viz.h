/**
 * @file
 * @brief Update function for simple point cloud visualizer
 */

#pragma once
#include "ouster/autoexposure.h"
#include "ouster/beam_uniformity.h"
#include "ouster/lidar_scan.h"
#include "ouster/point_viz.h"
#include "ouster/types.h"

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
    const std::vector<int> px_offset;
    const double aspect_ratio;
    const size_t h, w;
    std::vector<GLfloat> imdata;
    std::atomic_bool show_ambient;
    std::atomic_int display_mode;
    std::atomic_bool cycle_range;

    enum CloudDisplayMode {
        MODE_RANGE = 0,
        MODE_INTENSITY = 1,
        MODE_AMBIENT = 2,
        MODE_REFLECTIVITY = 3,
        NUM_MODES = 4
    };
    PointViz& point_viz;

   public:
    /**
     * set up the LidarScanViz with a reference to the PointViz
     * and sensor metadata. Please note that the point_viz_ is stored as a
     * reference and should be kept in scope for the lifetime of the
     * LidarScanViz.
     *
     * @param info Sensor metadata, e.g. from parse_metadata, containing
     * @param point_viz_ Reference to PointViz
     */
    template <class Metadata>
    LidarScanViz(const Metadata& info, PointViz& point_viz_)
        : px_offset(info.format.pixel_shift_by_row),
          aspect_ratio((info.beam_altitude_angles.front() -
                        info.beam_altitude_angles.back()) /
                       360.0),  // beam angles are in degrees
          h(info.format.pixels_per_column),
          w(info.format.columns_per_frame),
          imdata(3 * h * w),
          show_ambient(true),
          display_mode(MODE_INTENSITY),
          cycle_range(false),
          point_viz(point_viz_) {
        point_viz.attachKeyHandler(
            GLFW_KEY_N, [this]() { this->show_ambient = !this->show_ambient; });
        point_viz.attachKeyHandler(GLFW_KEY_M, [this]() {
            this->display_mode = (this->display_mode + 1) % NUM_MODES;

            // for C++11 compatibility, the plus sign is needed due to
            // https://stackoverflow.com/questions/25143860/implicit-conversion-from-class-to-enumeration-type-in-switch-conditional
            switch (+display_mode) {
                case MODE_INTENSITY:
                    std::cerr << "Coloring point cloud by intensity"
                              << std::endl;
                    break;
                case MODE_RANGE:
                    std::cerr << "Coloring point cloud by range" << std::endl;
                    break;
                case MODE_AMBIENT:
                    std::cerr << "Coloring point cloud by ambient" << std::endl;
                    break;
                case MODE_REFLECTIVITY:
                    std::cerr << "Coloring point cloud by reflectivity"
                              << std::endl;
            }
        });
        point_viz.attachKeyHandler(GLFW_KEY_V, [this]() {
            this->cycle_range = !this->cycle_range;
            if (this->cycle_range) {
                std::cerr << "Cycling range every 2.0 m" << std::endl;
            } else {
                std::cerr << "Cycling range disabled" << std::endl;
            }
        });
    }

    /**
     * render both image view and point cloud view to the point_viz.
     * If poses are available, set bool cloud_swap to false and then
     * call point_viz.cloudSwap() manually
     */
    void draw(const LidarScan& ls, const size_t which_cloud = 0,
              const bool cloud_swap = true, const bool show_image = true) {
        using glmap_t =
            Eigen::Map<Eigen::Array<GLfloat, Eigen::Dynamic, Eigen::Dynamic,
                                    Eigen::RowMajor>>;

        img_t<double> range = ls.field(LidarScan::RANGE).cast<double>();
        if (show_image || display_mode == MODE_RANGE) {
            if (!cycle_range) {
                range_ae(
                    Eigen::Map<Eigen::ArrayXd>(range.data(), range.size()));
            }
        }
        img_t<double> intensity = ls.field(LidarScan::INTENSITY).cast<double>();
        if (show_image || display_mode == MODE_INTENSITY) {
            intensity_ae(
                Eigen::Map<Eigen::ArrayXd>(intensity.data(), intensity.size()));
        }
        img_t<double> ambient = ls.field(LidarScan::AMBIENT).cast<double>();
        auto intensity_destaggered = destagger<double>(intensity, px_offset);
        auto range_destaggered = destagger<double>(range, px_offset);
        if (cycle_range) {
            glmap_t(imdata.data(), h, w) =
                range_destaggered.cast<GLfloat>().unaryExpr(
                    [](const GLfloat x) -> GLfloat {
                        return std::fmod(x * sensor::range_unit, 2.0) * 0.5;
                    });
        } else {
            glmap_t(imdata.data(), h, w) = range_destaggered.cast<GLfloat>();
        }
        glmap_t(imdata.data() + w * h, h, w) =
            intensity_destaggered.cast<GLfloat>();

        img_t<double> ambient_destaggered{h, w};
        if ((show_image && show_ambient) || display_mode == MODE_AMBIENT) {
            // we need to destagger ambient because the
            // BeamUniformityCorrector only works on destaggered stuff
            ambient_destaggered = destagger<double>(ambient, px_offset);
            ambient_buc.correct(ambient_destaggered);
            ambient_ae(Eigen::Map<Eigen::ArrayXd>(ambient_destaggered.data(),
                                                  ambient_destaggered.size()));

            if (show_image && show_ambient) {
                glmap_t(imdata.data() + 2 * w * h, h, w) =
                    ambient_destaggered.cast<GLfloat>();
            }
        }

        if (show_image) {
            if (show_ambient) {
                point_viz.resizeImage(w, 3 * h);
                point_viz.setImageAspectRatio(3 * aspect_ratio);
            } else {
                point_viz.resizeImage(w, 2 * h);
                point_viz.setImageAspectRatio(2 * aspect_ratio);
            }
            point_viz.setImage(imdata.data());
            point_viz.imageSwap();
        }

        auto range_data = ls.field(LidarScan::RANGE).data();

        switch (+display_mode) {
            case MODE_INTENSITY:
                point_viz.setRangeAndKey(which_cloud, range_data,
                                         intensity.data());
                break;
            case MODE_RANGE:
                point_viz.setRangeAndKey(which_cloud, range_data, range.data());
                break;
            case MODE_AMBIENT:
                ambient = stagger<double>(ambient_destaggered, px_offset);
                point_viz.setRangeAndKey(which_cloud, range_data,
                                         ambient.data());
                break;
            case MODE_REFLECTIVITY:
                img_t<double> reflectivity =
                    ls.field(LidarScan::REFLECTIVITY).cast<double>();
                reflectivity_ae(Eigen::Map<Eigen::ArrayXd>(
                    reflectivity.data(), reflectivity.size()));
                point_viz.setRangeAndKey(which_cloud, range_data,
                                         reflectivity.data());
                break;
        }
        if (cloud_swap) point_viz.cloudSwap(which_cloud);
    }
};
}  // namespace viz
}  // namespace ouster

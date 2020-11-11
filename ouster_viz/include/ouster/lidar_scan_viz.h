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
    AutoExposure noise_ae;
    AutoExposure reflectivity_ae;
    BeamUniformityCorrector noise_buc;
    const std::vector<int> px_offset;
    const double aspect_ratio;
    const size_t h, w;
    std::vector<GLfloat> imdata;
    std::atomic_bool show_noise;
    std::atomic_int display_mode;
    std::atomic_bool cycle_range;

    enum CloudDisplayMode {
        MODE_RANGE = 0,
        MODE_INTENSITY = 1,
        MODE_NOISE = 2,
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
          show_noise(true),
          display_mode(MODE_INTENSITY),
          cycle_range(false),
          point_viz(point_viz_) {
        point_viz.attachKeyHandler(
            GLFW_KEY_N, [this]() { this->show_noise = !this->show_noise; });
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
                case MODE_NOISE:
                    std::cerr << "Coloring point cloud by noise" << std::endl;
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
        Eigen::ArrayXd range = ls.range().cast<double>();
        if (show_image || display_mode == MODE_RANGE) {
            if (!cycle_range) {
                range_ae(range);
            }
        }
        Eigen::ArrayXd intensity = ls.intensity().cast<double>();
        if (show_image || display_mode == MODE_INTENSITY) {
            intensity_ae(intensity);
        }
        Eigen::ArrayXd noise = ls.noise().cast<double>();
        Eigen::ArrayXXd noise_destaggered(h, w);
        for (size_t u = 0; u < h; u++) {
            for (size_t v = 0; v < w; v++) {
                const size_t vv = (v + px_offset[u]) % w;
                if (cycle_range) {
                    imdata[u * w + vv] =
                        std::fmod(range(u * w + v) * sensor::range_unit, 2.0) /
                        2.0;
                } else {
                    imdata[u * w + vv] = range(u * w + v);
                }
                imdata[(u + h) * w + vv] = intensity(u * w + v);
            }
        }

        // TODO: optimize and move destaggering logic to ouster::LidarScan
        if ((show_image && show_noise) || display_mode == MODE_NOISE) {
            // we need to destagger noise because the
            // BeamUniformityCorrector only works on destaggered stuff
            for (size_t u = 0; u < h; u++) {
                for (size_t v = 0; v < w; v++) {
                    const size_t vv = (v + px_offset[u]) % w;
                    noise_destaggered(u, vv) = noise(u * w + v);
                }
            }
            noise_buc.correct(noise_destaggered);
            noise_ae(
                Eigen::Map<Eigen::ArrayXd>(noise_destaggered.data(), w * h));
            for (size_t u = 0; u < h; u++) {
                for (size_t v = 0; v < w; v++) {
                    imdata[2 * h * w + u * w + v] = noise_destaggered(u, v);
                }
            }
        }

        if (show_image) {
            if (show_noise) {
                point_viz.resizeImage(w, 3 * h);
                point_viz.setImageAspectRatio(3 * aspect_ratio);
            } else {
                point_viz.resizeImage(w, 2 * h);
                point_viz.setImageAspectRatio(2 * aspect_ratio);
            }
            point_viz.setImage(imdata.data());
            point_viz.imageSwap();
        }

        switch (+display_mode) {
            case MODE_INTENSITY:
                point_viz.setRangeAndKey(which_cloud, ls.range().data(),
                                         intensity.data());
                break;
            case MODE_RANGE:
                point_viz.setRangeAndKey(which_cloud, ls.range().data(),
                                         range.data());
                break;
            case MODE_NOISE:
                // zzz...
                for (size_t u = 0; u < h; u++) {
                    for (size_t v = 0; v < w; v++) {
                        const size_t vv = (v + px_offset[u]) % w;
                        noise(u * w + v) = noise_destaggered(u, vv);
                    }
                }
                point_viz.setRangeAndKey(which_cloud, ls.range().data(),
                                         noise.data());
                break;
            case MODE_REFLECTIVITY:
                Eigen::ArrayXd reflectivity = ls.reflectivity().cast<double>();
                reflectivity_ae(reflectivity);
                point_viz.setRangeAndKey(which_cloud, ls.range().data(),
                                         reflectivity.data());
                break;
        }
        if (cloud_swap) point_viz.cloudSwap(which_cloud);
    }
};
}  // namespace viz
}  // namespace ouster

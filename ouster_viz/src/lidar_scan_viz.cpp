#include "ouster/lidar_scan_viz.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>

#include "ouster/types.h"
#include "ouster/colormaps.h"

namespace ouster {
namespace viz {

namespace {
enum CloudDisplayMode {
    MODE_RANGE = 0,
    MODE_INTENSITY = 1,
    MODE_AMBIENT = 2,
    MODE_REFLECTIVITY = 3,
    NUM_MODES = 4
};
}

const util::version calref_min_version = {2, 1, 0};

LidarScanViz::LidarScanViz(const sensor::sensor_info& info,
                           PointViz& point_viz_)
    : firmware_version(ouster::util::version_of_string(info.fw_rev)),
      px_offset(info.format.pixel_shift_by_row),
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
                std::cerr << "Coloring point cloud by intensity" << std::endl;
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
        if(display_mode == MODE_REFLECTIVITY &&
	       firmware_version >= calref_min_version) {
           point_viz.setPointCloudPalette(calref, calref_n);
        } else {
           point_viz.setPointCloudPalette(spezia, spezia_n);
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

void LidarScanViz::draw(const LidarScan& ls, const size_t which_cloud,
                        const bool cloud_swap, const bool show_image) {
    using glmap_t = Eigen::Map<
        Eigen::Array<GLfloat, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

    img_t<double> range = ls.field(LidarScan::RANGE).cast<double>();
    if (show_image || display_mode == MODE_RANGE) {
        if (!cycle_range) {
            range_ae(range);
        }
    }
    img_t<double> intensity = ls.field(LidarScan::INTENSITY).cast<double>();
    if (show_image || display_mode == MODE_INTENSITY) {
        intensity_ae(intensity);
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
        ambient_buc(ambient_destaggered);
        ambient_ae(ambient_destaggered);
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
            point_viz.setRangeAndKey(which_cloud, range_data, intensity.data());
            break;
        case MODE_RANGE:
            point_viz.setRangeAndKey(which_cloud, range_data, range.data());
            break;
        case MODE_AMBIENT:
            ambient = stagger<double>(ambient_destaggered, px_offset);
            point_viz.setRangeAndKey(which_cloud, range_data, ambient.data());
            break;
        case MODE_REFLECTIVITY:
            img_t<double> reflectivity =
                ls.field(LidarScan::REFLECTIVITY).cast<double>();
            if(firmware_version >= calref_min_version) {  
                // Scale directly from 0-255 to 0-1
                reflectivity /= 255.0;
            } else {
                // Apply autoexposure algorithm          
                reflectivity_ae(reflectivity);
            }
            
            point_viz.setRangeAndKey(which_cloud, range_data,
                                     reflectivity.data());
            break;
    }
    if (cloud_swap) point_viz.cloudSwap(which_cloud);
}

}  // namespace viz
}  // namespace ouster

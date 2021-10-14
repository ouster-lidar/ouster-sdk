#include "ouster/lidar_scan_viz.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>
#include <mutex>
#include <set>
#include <utility>

#include "ouster/colormaps.h"
#include "ouster/types.h"

namespace ouster {
namespace viz {

namespace {

using sensor::ChanField;

enum CloudDisplayMode {
    MODE_RANGE = 0,
    MODE_SIGNAL = 1,
    MODE_NEAR_IR = 2,
    MODE_REFLECTIVITY = 3,
    NUM_MODES = 4
};

ChanField field_of_mode(CloudDisplayMode display_mode, bool second = false) {
    switch (display_mode) {
        case MODE_SIGNAL:
            return second ? ChanField::SIGNAL2 : ChanField::SIGNAL;
        case MODE_RANGE:
            return second ? ChanField::RANGE2 : ChanField::RANGE;
        case MODE_NEAR_IR:
            return ChanField::NEAR_IR;
        case MODE_REFLECTIVITY:
            return second ? ChanField::REFLECTIVITY2 : ChanField::REFLECTIVITY;
        default:
            throw std::runtime_error("Unreachable");
    }
}

enum CullingMode { CULLING_OFF = 0, CULLING_ON = 1, NUM_CULLING_MODES = 2 };

using glmap_t = Eigen::Map<img_t<GLfloat>>;

const util::version calref_min_version = {2, 1, 0};

struct read_and_cast {
    template <typename T, typename U>
    void operator()(Eigen::Ref<const img_t<T>> field, img_t<U>& dest) {
        dest = field.template cast<U>();
    }
};

}  // namespace

/* key handlers called from point viz draw look thread */

void LidarScanViz::cycle_field_2d_1() {
    std::lock_guard<std::mutex> lock{mx};
    image_ind1 = (image_ind1 + 1) % available_fields.size();
    std::cerr << "2D image: " << to_string(available_fields.at(image_ind1))
              << "/" << to_string(available_fields.at(image_ind2)) << std::endl;
}

void LidarScanViz::cycle_field_2d_2() {
    std::lock_guard<std::mutex> lock{mx};
    image_ind2 = (image_ind2 + 1) % available_fields.size();
    std::cerr << "2D image: " << to_string(available_fields.at(image_ind1))
              << "/" << to_string(available_fields.at(image_ind2)) << std::endl;
}

void LidarScanViz::cycle_display_mode() {
    std::lock_guard<std::mutex> lock{mx};
    this->display_mode = (this->display_mode + 1) % NUM_MODES;

    // TODO: displays a few frames with the wrong palette :p
    if (this->display_mode == MODE_REFLECTIVITY &&
        firmware_version >= calref_min_version) {
        point_viz.setPointCloudPalette(calref, calref_n);
    } else {
        point_viz.setPointCloudPalette(spezia, spezia_n);
    }

    switch (display_mode) {
        case MODE_SIGNAL:
            std::cerr << "Point cloud: SIGNAL" << std::endl;
            break;
        case MODE_RANGE:
            std::cerr << "Point cloud: RANGE" << std::endl;
            break;
        case MODE_NEAR_IR:
            std::cerr << "Point cloud: NEAR_IR" << std::endl;
            break;
        case MODE_REFLECTIVITY:
            std::cerr << "Point cloud: REFLECTIVITY" << std::endl;
            break;
    }
}

LidarScanViz::LidarScanViz(const sensor::sensor_info& info,
                           PointViz& point_viz_)
    : firmware_version(ouster::util::version_of_string(info.fw_rev)),
      px_offset(info.format.pixel_shift_by_row),
      aspect_ratio((info.beam_altitude_angles.front() -
                    info.beam_altitude_angles.back()) /
                   360.0),  // beam angles are in degrees
      h(info.format.pixels_per_column),
      w(info.format.columns_per_frame),
      range1{h, w},
      range2{h, w},
      field_data{},
      field_procs{},
      active_fields{},
      imdata(3 * h * w),
      display_mode(MODE_SIGNAL),
      image_ind1(0),
      image_ind2(2),
      // intensity is more likely than range to gray out
      // blacks out when using AE with higher lo_percentile
      intensity_ae(0.02, 0.1, 3),
      point_viz(point_viz_) {
    // extra key bindings
    point_viz.attachKeyHandler(
        GLFW_KEY_M, std::bind(&LidarScanViz::cycle_display_mode, this));

    point_viz.attachKeyHandler(
        GLFW_KEY_B, std::bind(&LidarScanViz::cycle_field_2d_1, this));

    point_viz.attachKeyHandler(
        GLFW_KEY_N, std::bind(&LidarScanViz::cycle_field_2d_2, this));

    // normalization functions for each field
    //
    // TODO: passing *_ae without wrapping in a lambda is nice but causes a
    //       copy, which prevents sharing state with 2nd return processing
    field_procs[ChanField::RANGE] = {
        [this](Eigen::Ref<img_t<double>> f) { range_ae(f); }};

    field_procs[ChanField::RANGE2] = {
        [this](Eigen::Ref<img_t<double>> f) { range_ae(f, false); }};

    field_procs[ChanField::SIGNAL] = {
        [this](Eigen::Ref<img_t<double>> f) { intensity_ae(f); }};

    field_procs[ChanField::SIGNAL2] = {
        [this](Eigen::Ref<img_t<double>> f) { intensity_ae(f, false); }};

    if (firmware_version >= calref_min_version)
        field_procs[ChanField::REFLECTIVITY] = {
            [](Eigen::Ref<img_t<double>> f) { f /= 255.0; }};
    else
        field_procs[ChanField::REFLECTIVITY] = {
            [this](Eigen::Ref<img_t<double>> f) { reflectivity_ae(f); }};

    if (firmware_version >= calref_min_version)
        field_procs[ChanField::REFLECTIVITY2] = {
            [](Eigen::Ref<img_t<double>> f) { f /= 255.0; }};
    else
        field_procs[ChanField::REFLECTIVITY2] = {
            [this](Eigen::Ref<img_t<double>> f) { reflectivity_ae(f, false); }};

    field_procs[ChanField::NEAR_IR] = {
        [this](Eigen::Ref<img_t<double>> f) {
            f = destagger<double>(f, px_offset);
            ambient_buc(f);
            f = stagger<double>(f, px_offset);
        },
        [this](Eigen::Ref<img_t<double>> f) { ambient_ae(f); }};
}

void LidarScanViz::draw(const LidarScan& ls, const size_t which_cloud) {
    // protect against concurrent access in key handlers
    std::lock_guard<std::mutex> lock{mx};

    point_viz.resizeImage(w, 2 * h);
    point_viz.setImageAspectRatio(2 * aspect_ratio);

    // select fields to display by index
    available_fields.clear();
    for (auto kv : ls) available_fields.push_back(kv.first);
    auto img_field_1 = available_fields.at(image_ind1);
    auto img_field_2 = available_fields.at(image_ind2);

    // figure out which fields are actually being displayed
    auto mode = static_cast<CloudDisplayMode>(display_mode);
    auto cloud_field_1 = field_of_mode(mode);
    auto cloud_field_2 = field_of_mode(mode, true);
    active_fields.clear();
    active_fields.insert(
        {cloud_field_1, cloud_field_2, img_field_1, img_field_2});

    // populate field buffers and apply normalization
    for (const auto field : active_fields) {
        if (ls.field_type(field)) {
            impl::visit_field(ls, field, read_and_cast(), field_data[field]);
            for (auto& proc : field_procs.at(field)) proc(field_data[field]);
        }
    }

    // update first cloud
    impl::visit_field(ls, ChanField::RANGE, read_and_cast(), range1);
    double* key1_data = field_data.at(cloud_field_1).data();
    point_viz.setRangeAndKey(which_cloud, range1.data(), key1_data);
    point_viz.cloudSwap(which_cloud);

    // TODO: hacked to use two cloud indices per scan for second return
    if (ls.field_type(ChanField::RANGE2) && ls.field_type(cloud_field_2)) {
        impl::visit_field(ls, ChanField::RANGE2, read_and_cast(), range2);
        double* key2_data = field_data.at(cloud_field_2).data();
        point_viz.setRangeAndKey(which_cloud + 1, range2.data(), key2_data);
        point_viz.cloudSwap(which_cloud + 1);
    }

    // update 2d image data
    glmap_t(imdata.data(), h, w) =
        destagger<double>(field_data.at(img_field_1), px_offset)
            .cast<GLfloat>();
    glmap_t(imdata.data() + w * h, h, w) =
        destagger<double>(field_data.at(img_field_2), px_offset)
            .cast<GLfloat>();
    point_viz.setImage(imdata.data());
    point_viz.imageSwap();
}

}  // namespace viz
}  // namespace ouster

#include "ouster/lidar_scan_viz.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>
#include <memory>
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

void LidarScanViz::cycle_field_2d(int& ind) {
    ind = (ind + 1) % available_fields.size();
    std::cerr << "2D image: " << to_string(available_fields.at(image_ind1))
              << "/" << to_string(available_fields.at(image_ind2)) << std::endl;
}

void LidarScanViz::cycle_display_mode() {
    this->display_mode = (this->display_mode + 1) % NUM_MODES;
    display_mode_changed = true;

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

void LidarScanViz::change_size_fraction(int amount) {
    int size_fraction_max = 20;
    size_fraction = (size_fraction + amount + (size_fraction_max + 1)) %
                    (size_fraction_max + 1);
    std::cerr << "Image size: " << size_fraction << std::endl;

    // fraction of vertical window space used by the images
    float vfrac = size_fraction / static_cast<float>(size_fraction_max);
    float hfrac = vfrac / 2 / aspect_ratio;

    // update image screen position
    image1->set_position({-hfrac, hfrac, 1, 1 - vfrac});
    image2->set_position({-hfrac, hfrac, 1 - vfrac, 1 - vfrac * 2});

    // center camera target in area not taken up by image
    point_viz.camera().set_proj_offset(0, vfrac);
}

bool LidarScanViz::key_handler(const PointViz::HandlerCtx&, int key, int mods) {
    std::lock_guard<std::mutex> lock{mx};
    if (mods == 0) {
        switch (key) {
            case GLFW_KEY_O:
                point_size = std::max(0.0f, point_size - 1);
                cloud1->set_point_size(point_size);
                cloud2->set_point_size(point_size);
                point_viz.update();
                break;
            case GLFW_KEY_P:
                point_size = std::min(10.0f, point_size + 1);
                cloud1->set_point_size(point_size);
                cloud2->set_point_size(point_size);
                point_viz.update();
                break;
            case GLFW_KEY_1:
                cloud1_enabled = !cloud1_enabled;
                if (cloud1_enabled)
                    point_viz.add(cloud1);
                else
                    point_viz.remove(cloud1);
                point_viz.update();
                break;
            case GLFW_KEY_2:
                cloud2_enabled = !cloud2_enabled;
                if (cloud2_enabled)
                    point_viz.add(cloud2);
                else
                    point_viz.remove(cloud2);
                point_viz.update();
                break;
            case GLFW_KEY_SEMICOLON:
                point_viz.target_display().update_ring_size(1);
                point_viz.update();
                break;
            case GLFW_KEY_APOSTROPHE:
                point_viz.target_display().update_ring_size(-1);
                point_viz.update();
                break;
            case GLFW_KEY_E:
                change_size_fraction(1);
                point_viz.update();
                break;
            case GLFW_KEY_M:
                cycle_display_mode();
                break;
            case GLFW_KEY_B:
                cycle_field_2d(image_ind1);
                break;
            case GLFW_KEY_N:
                cycle_field_2d(image_ind2);
                break;
            default:
                break;
        }
    } else if (mods == GLFW_MOD_SHIFT) {
        switch (key) {
            case GLFW_KEY_E:
                change_size_fraction(-1);
                point_viz.update();
                break;
            default:
                break;
        }
    }
    return false;
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
      imdata1(h * w),
      imdata2(h * w),
      cloud1_enabled(true),
      cloud2_enabled(true),
      point_size{2},
      display_mode(MODE_SIGNAL),
      image_ind1(0),
      image_ind2(2),
      // intensity is more likely than range to gray out
      // blacks out when using AE with higher lo_percentile
      intensity_ae(0.02, 0.1, 3),
      point_viz(point_viz_) {
    // extra key bindings
    using namespace std::placeholders;

    viz::add_default_controls(point_viz, mx);

    point_viz.push_key_handler(
        std::bind(&LidarScanViz::key_handler, this, _1, _2, _3));

    // normalization functions for each field
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

    const auto xyz_lut = make_xyz_lut(info);

    // two clouds to optionally display second return
    cloud1 =
        std::make_shared<Cloud>(w, h, xyz_lut.direction.data(),
                                xyz_lut.offset.data(), info.extrinsic.data());
    cloud2 =
        std::make_shared<Cloud>(w, h, xyz_lut.direction.data(),
                                xyz_lut.offset.data(), info.extrinsic.data());

    point_viz.add(cloud1);
    point_viz.add(cloud2);

    // two images
    image1 = std::make_shared<Image>();
    image2 = std::make_shared<Image>();

    point_viz.add(image1);
    point_viz.add(image2);

    // initialize image sizes
    change_size_fraction(0);
}

void LidarScanViz::draw(const LidarScan& ls, const size_t /*which_cloud*/) {
    // protect against concurrent access in key handlers
    std::lock_guard<std::mutex> lock{mx};

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
            ouster::impl::visit_field(ls, field, read_and_cast(),
                                      field_data[field]);
            for (auto& proc : field_procs.at(field)) proc(field_data[field]);
        }
    }

    // update first cloud
    ouster::impl::visit_field(ls, ChanField::RANGE, read_and_cast(), range1);
    double* key1_data = field_data.at(cloud_field_1).data();
    cloud1->set_range(range1.data());
    cloud1->set_key(key1_data);

    // TODO: hacked to use two cloud indices per scan for second return
    if (ls.field_type(ChanField::RANGE2) && ls.field_type(cloud_field_2)) {
        ouster::impl::visit_field(ls, ChanField::RANGE2, read_and_cast(),
                                  range2);
        double* key2_data = field_data.at(cloud_field_2).data();
        cloud2->set_range(range2.data());
        cloud2->set_key(key2_data);
    }

    // update 2d image data
    glmap_t(imdata1.data(), h, w) =
        destagger<double>(field_data.at(img_field_1), px_offset)
            .cast<GLfloat>();
    glmap_t(imdata2.data(), h, w) =
        destagger<double>(field_data.at(img_field_2), px_offset)
            .cast<GLfloat>();
    image1->set_image(w, h, imdata1.data());
    image2->set_image(w, h, imdata2.data());

    // update point cloud palette
    if (display_mode_changed) {
        if (this->display_mode == MODE_REFLECTIVITY &&
            firmware_version >= calref_min_version) {
            cloud1->set_palette(&calref[0][0], calref_n);
            cloud2->set_palette(&calref[0][0], calref_n);
        } else {
            cloud1->set_palette(&spezia[0][0], spezia_n);
            cloud2->set_palette(&spezia[0][0], spezia_n);
        }
    }

    // display new data on next frame
    point_viz.update();
}

}  // namespace viz
}  // namespace ouster

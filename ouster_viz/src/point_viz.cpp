/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/point_viz.h"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "camera.h"
#include "cloud.h"
#include "colormaps.h"
#include "glfw.h"
#include "image.h"
#include "misc.h"

static_assert(std::is_same<GLfloat, float>::value,
              "Platform has unexpected definition of GLfloat");

namespace ouster {
namespace viz {

namespace {

/*
 * Helper for addable / removable drawable objects
 */
template <typename GL, typename T>
class Indexed {
    struct Front {
        std::unique_ptr<GL> gl;
        std::unique_ptr<T> state;
    };
    using Back = std::shared_ptr<T>;

    std::vector<Front> front;
    std::vector<Back> back;

   public:
    Indexed() : front{}, back{} {}

    void add(const std::shared_ptr<T>& t) {
        // find and use first empty slot, or grow
        auto res = std::find_if(back.begin(), back.end(),
                                [](const Back& b) { return !b; });
        if (res == back.end()) {
            back.push_back(t);
        } else {
            *res = t;
        }
    }

    bool remove(const std::shared_ptr<T>& t) {
        auto res = std::find(back.begin(), back.end(), t);
        if (res == back.end()) {
            return false;
        } else {
            res->reset();
            return true;
        }
    }

    void draw(const WindowCtx& ctx, const impl::CameraData& camera) {
        for (auto& f : front) {
            if (!f.state) {
                continue;  // skip deleted
            }
            if (!f.gl) {
                f.gl = std::make_unique<GL>(*f.state);  // init GL for added
            }
            f.gl->draw(ctx, camera, *f.state);
        }
    }

    void swap() {
        assert(front.size() <= back.size());

        // in case back grew
        if (front.size() < back.size()) front.resize(back.size());

        // send updated, added or deleted state to the front
        for (size_t i = 0; i < front.size(); i++) {
            if (back[i] && front[i].state) {
                front[i].state->update_from(*back[i]);
                back[i]->clear();
            } else if (back[i] && !front[i].state) {
                front[i].state = std::make_unique<T>(*back[i]);
                back[i]->clear();
            } else if (!back[i] && front[i].state) {
                front[i].state.reset();
            }
        }
    }
};

}  // namespace

/*
 * PointViz implementation
 */
struct PointViz::Impl {
    std::unique_ptr<GLFWContext> glfw;
    GLuint vao;

    // state for drawing
    std::mutex update_mx;
    bool front_changed{false};

    Camera camera_back, camera_front;

    TargetDisplay target;
    impl::GLRings rings;

    Indexed<impl::GLCloud, Cloud> clouds;
    Indexed<impl::GLCuboid, Cuboid> cuboids;
    Indexed<impl::GLLabel, Label> labels;
    Indexed<impl::GLImage, Image> images;

    template <typename T>
    using Handlers = std::list<std::function<T>>;

    Handlers<bool(const WindowCtx&, int, int)> key_handlers;
    Handlers<bool(const WindowCtx&, ouster::viz::MouseButton,
                  ouster::viz::MouseButtonEvent,
                  ouster::viz::EventModifierKeys)>
        mouse_button_handlers;
    Handlers<bool(const WindowCtx&, double, double)> scroll_handlers;
    Handlers<bool(const WindowCtx&, double, double)> mouse_pos_handlers;

    Handlers<bool(const std::vector<uint8_t>& fb_data, int viewport_width,
                  int viewport_height)>
        frame_buffer_handlers;

    Handlers<bool(const WindowCtx&)> window_resize_handlers;

    // temp storage for frame_buffer_handlers
    std::vector<uint8_t> frame_buffer_data_{};

    double fps_last_time_{0};
    uint64_t fps_frame_counter_{0};
    double fps_{0};

    Impl(std::unique_ptr<GLFWContext>&& glfw) : glfw{std::move(glfw)} {}
};

/*
 * PointViz interface
 */

PointViz::PointViz(const std::string& name, bool fix_aspect, int window_width,
                   int window_height) {
    auto glfw = std::make_unique<GLFWContext>(name, fix_aspect, window_width,
                                              window_height);

    // set context for GL initialization
    glfwMakeContextCurrent(glfw->window);

    pimpl = std::make_unique<Impl>(std::move(glfw));

    // top-level gl state for point viz
    glGenVertexArrays(1, &pimpl->vao);
    glBindVertexArray(pimpl->vao);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // TODO: need to check if these were already called?
    impl::GLCloud::initialize();
    impl::GLImage::initialize();
    impl::GLRings::initialize();
    impl::GLCuboid::initialize();

    // release context in case subsequent calls are done from another thread
    glfwMakeContextCurrent(nullptr);

    // add user-setable input handlers
    pimpl->glfw->key_handler = [this](const WindowCtx& ctx, int key, int mods) {
        for (auto& f : pimpl->key_handlers)
            if (!f(ctx, key, mods)) {
                break;
            }
    };
    pimpl->glfw->mouse_button_handler = [this](const WindowCtx& ctx, int button,
                                               int action, int mods) {
        for (auto& f : pimpl->mouse_button_handlers)
            if (!f(ctx, ouster::viz::MouseButton(button),
                   ouster::viz::MouseButtonEvent(action),
                   ouster::viz::EventModifierKeys(mods))) {
                break;
            }
    };
    pimpl->glfw->scroll_handler = [this](const WindowCtx& ctx, double x,
                                         double y) {
        for (auto& f : pimpl->scroll_handlers)
            if (!f(ctx, x, y)) {
                break;
            }
    };
    pimpl->glfw->mouse_pos_handler = [this](const WindowCtx& ctx, double x,
                                            double y) {
        for (auto& f : pimpl->mouse_pos_handlers)
            if (!f(ctx, x, y)) {
                break;
            }
    };

    // glfwPollEvents blocks during resize on macos. Keep rendering to avoid
    // artifacts during resize
    pimpl->glfw->resize_handler = [this](const WindowCtx& ctx) {
        for (auto& f : pimpl->window_resize_handlers) {
            if (!f(ctx)) {
                break;
            }
        }
#ifdef __APPLE__
        draw();
#endif
    };
}

PointViz::~PointViz() { glDeleteVertexArrays(1, &pimpl->vao); }

void PointViz::run() {
    pimpl->glfw->running(true);
    pimpl->glfw->visible(true);
    while (running()) run_once();
    pimpl->glfw->visible(false);
}

void PointViz::run_once() {
    if (glfwGetCurrentContext() != pimpl->glfw->window)
        glfwMakeContextCurrent(pimpl->glfw->window);
    draw();
    glfwPollEvents();
}

bool PointViz::running() { return pimpl->glfw->running(); }

void PointViz::running(bool state) { pimpl->glfw->running(state); }

void PointViz::visible(bool state) { pimpl->glfw->visible(state); }

void PointViz::update() {
    std::lock_guard<std::mutex> guard{pimpl->update_mx};

    // TWS 20241014: note we used to return here if
    // the last frame hasn't been drawn yet.
    // However, this can cause problem if the API user calls
    // update(), especially if more than once, before
    // calling run().
    // We're preserving front_changed in case we want
    // to expose this value via the API later on.
    // if (pimpl->front_changed) return false;

    // propagate camera changes
    pimpl->camera_front = pimpl->camera_back;

    pimpl->clouds.swap();
    pimpl->cuboids.swap();
    pimpl->labels.swap();
    pimpl->images.swap();
    pimpl->rings.update(pimpl->target);

    pimpl->front_changed = true;
}

int PointViz::viewport_width() const {
    return pimpl->glfw->window_context.viewport_width;
}

int PointViz::viewport_height() const {
    return pimpl->glfw->window_context.viewport_height;
}

int PointViz::window_width() const {
    return pimpl->glfw->window_context.window_width;
}

int PointViz::window_height() const {
    return pimpl->glfw->window_context.window_height;
}

void PointViz::draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBindVertexArray(pimpl->vao);

    // fps counting
    ++pimpl->fps_frame_counter_;
    double now_t = glfwGetTime();
    if (pimpl->fps_last_time_ == 0 || now_t - pimpl->fps_last_time_ >= 1.0) {
        pimpl->fps_ =
            pimpl->fps_frame_counter_ / (now_t - pimpl->fps_last_time_);
        pimpl->fps_last_time_ = now_t;
        pimpl->fps_frame_counter_ = 0;
    }

    // draw images
    {
        std::lock_guard<std::mutex> guard{pimpl->update_mx};
        const auto& ctx = pimpl->glfw->window_context;

        // calculate camera matrices
        auto camera_data =
            pimpl->camera_front.matrices(impl::window_aspect(ctx));

        // draw clouds
        impl::GLCloud::beginDraw();
        pimpl->clouds.draw(ctx, camera_data);
        impl::GLCloud::endDraw();

        // draw rings
        pimpl->rings.draw(ctx, camera_data);

        // draw cuboids
        impl::GLCuboid::beginDraw();
        pimpl->cuboids.draw(ctx, camera_data);
        impl::GLCuboid::endDraw();

        // draw labels and images on top of everything
        glClear(GL_DEPTH_BUFFER_BIT);

        // draw image
        impl::GLImage::beginDraw();
        pimpl->images.draw(ctx, camera_data);
        impl::GLImage::endDraw();

        // draw labels
        impl::GLLabel::beginDraw();
        pimpl->labels.draw(ctx, camera_data);
        impl::GLLabel::endDraw();

        // switch back to point viz vao
        glBindVertexArray(pimpl->vao);

        // mark front buffers no longer dirty
        pimpl->front_changed = false;
    }

    if (!pimpl->frame_buffer_handlers.empty()) {
        int width = viewport_width();
        int height = viewport_height();
        pimpl->frame_buffer_data_.resize(width * height * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE,
                     pimpl->frame_buffer_data_.data());
        for (auto& f : pimpl->frame_buffer_handlers)
            if (!f(pimpl->frame_buffer_data_, width, height)) break;
    }

    glfwSwapBuffers(pimpl->glfw->window);
}

double PointViz::fps() const { return pimpl->fps_; }

/*
 * Input handling
 */
void PointViz::push_key_handler(
    std::function<bool(const WindowCtx&, int, int)>&& callback) {
    // TODO: not thread safe: called in glfwPollEvents()
    pimpl->key_handlers.push_front(std::move(callback));
}

void PointViz::push_mouse_button_handler(
    std::function<bool(const WindowCtx&, ouster::viz::MouseButton,
                       ouster::viz::MouseButtonEvent,
                       ouster::viz::EventModifierKeys)>&& callback) {
    pimpl->mouse_button_handlers.push_front(std::move(callback));
}

void PointViz::push_scroll_handler(
    std::function<bool(const WindowCtx&, double, double)>&& callback) {
    pimpl->scroll_handlers.push_front(std::move(callback));
}

void PointViz::push_mouse_pos_handler(
    std::function<bool(const WindowCtx&, double, double)>&& callback) {
    pimpl->mouse_pos_handlers.push_front(std::move(callback));
}

void PointViz::push_frame_buffer_handler(
    std::function<bool(const std::vector<uint8_t>&, int, int)>&& callback) {
    pimpl->frame_buffer_handlers.push_front(std::move(callback));
}

void PointViz::pop_key_handler() { pimpl->key_handlers.pop_front(); }

void PointViz::pop_mouse_button_handler() {
    pimpl->mouse_button_handlers.pop_front();
}
void PointViz::pop_scroll_handler() { pimpl->scroll_handlers.pop_front(); }

void PointViz::pop_mouse_pos_handler() {
    pimpl->mouse_pos_handlers.pop_front();
}

void PointViz::pop_frame_buffer_handler() {
    pimpl->frame_buffer_handlers.pop_front();
}

void PointViz::push_frame_buffer_resize_handler(
    std::function<bool(const WindowCtx&)>&& callback) {
    pimpl->window_resize_handlers.push_front(std::move(callback));
}

void PointViz::pop_frame_buffer_resize_handler() {
    pimpl->window_resize_handlers.pop_front();
}

/*
 * Add / remove / access objects in the scene
 */
Camera& PointViz::camera() { return pimpl->camera_back; }

Camera& PointViz::current_camera() { return pimpl->camera_front; }

TargetDisplay& PointViz::target_display() { return pimpl->target; }

void PointViz::add(const std::shared_ptr<Cloud>& cloud) {
    cloud->dirty();
    pimpl->clouds.add(cloud);
}

void PointViz::add(const std::shared_ptr<Cuboid>& cuboid) {
    pimpl->cuboids.add(cuboid);
}

void PointViz::add(const std::shared_ptr<Label>& label) {
    label->dirty();
    pimpl->labels.add(label);
}

void PointViz::add(const std::shared_ptr<Image>& image) {
    pimpl->images.add(image);
}

bool PointViz::remove(const std::shared_ptr<Cloud>& cloud) {
    return pimpl->clouds.remove(cloud);
}

bool PointViz::remove(const std::shared_ptr<Cuboid>& cuboid) {
    return pimpl->cuboids.remove(cuboid);
}

bool PointViz::remove(const std::shared_ptr<Label>& label) {
    return pimpl->labels.remove(label);
}

bool PointViz::remove(const std::shared_ptr<Image>& image) {
    return pimpl->images.remove(image);
}

Cloud::Cloud(size_t w, size_t h, const mat4d& extrinsic)
    : n_{w * h},
      w_{w},
      extrinsic_{extrinsic},
      range_data_{std::make_shared<std::vector<float>>(n_, 0)},
      key_data_{std::make_shared<std::vector<float>>(4 * n_, 0)},
      mask_data_{std::make_shared<std::vector<float>>(4 * n_, 0)},
      xyz_data_{std::make_shared<std::vector<float>>(3 * n_, 0)},
      off_data_{std::make_shared<std::vector<float>>(3 * n_, 0)},
      transform_data_{std::make_shared<std::vector<float>>(12 * w, 0)},
      palette_data_{std::make_shared<std::vector<float>>(
          &spezia_palette[0][0], &spezia_palette[0][0] + spezia_n * 3)} {
    // initialize per-column poses to identity
    for (size_t v = 0; v < w; v++) {
        (*transform_data_)[3 * v] = 1;
        (*transform_data_)[3 * (v + w) + 1] = 1;
        (*transform_data_)[3 * (v + 2 * w) + 2] = 1;
    }
    transform_changed_ = true;

    // set initial rgba alpha to 1.0
    for (size_t i = 3; i < 4 * n_; i += 4) {
        (*key_data_)[i] = 1.0;
    }

    Eigen::Map<Eigen::Matrix4d>{pose_.data()}.setIdentity();
    pose_changed_ = true;
}

/*
 * Drawable types exposed to the user
 */
Cloud::Cloud(size_t n, const mat4d& extrinsic) : Cloud{1, n, extrinsic} {
    // initialize ranges to 1, set_xyz() used for data
    Eigen::Array<uint32_t, 1, -1> ones =
        Eigen::Array<uint32_t, 1, -1>::Ones(n * 3);
    set_range(ones.data());
}

Cloud::Cloud(size_t w, size_t h, const float* dir, const float* off,
             const mat4d& extrinsic)
    : Cloud::Cloud{w, h, extrinsic} {
    // initialize unit vectors and offsets, set_range() used for data
    set_xyz(dir);
    set_offset(off);
}

void Cloud::update_from(const Cloud& other) {
    // TODO[tws] This is ugly.
    // VAOs should have some encapsulation that allows better management.
    bool range_changed = other.range_changed_ || range_changed_;
    bool key_changed = other.key_changed_ || key_changed_;
    bool mask_changed = other.mask_changed_ || mask_changed_;
    bool pose_changed = other.pose_changed_ || pose_changed_;
    bool xyz_changed = other.xyz_changed_ || xyz_changed_;
    bool offset_changed = other.offset_changed_ || offset_changed_;
    bool transform_changed = other.transform_changed_ || transform_changed_;
    bool palette_changed = other.palette_changed_ || palette_changed_;
    bool point_size_changed = other.point_size_changed_ || point_size_changed_;
    *this = other;
    this->range_changed_ = range_changed;
    this->key_changed_ = key_changed;
    this->mask_changed_ = mask_changed;
    this->pose_changed_ = pose_changed;
    this->xyz_changed_ = xyz_changed;
    this->offset_changed_ = offset_changed;
    this->transform_changed_ = transform_changed;
    this->palette_changed_ = palette_changed;
    this->point_size_changed_ = point_size_changed;
}

void Cloud::clear() {
    range_changed_ = false;
    key_changed_ = false;
    mask_changed_ = false;
    xyz_changed_ = false;
    offset_changed_ = false;
    transform_changed_ = false;
    palette_changed_ = false;
    pose_changed_ = false;
    point_size_changed_ = false;
}

void Cloud::dirty() {
    range_changed_ = true;
    key_changed_ = true;
    mask_changed_ = true;
    xyz_changed_ = true;
    offset_changed_ = true;
    transform_changed_ = true;
    palette_changed_ = true;
    pose_changed_ = true;
    point_size_changed_ = true;
}

void Cloud::set_range(const uint32_t* x) {
    range_data_ = std::make_shared<std::vector<float>>(n_, 0);
    std::transform(x, x + n_, std::begin(*range_data_),
                   [](uint32_t i) { return static_cast<float>(i); });
    range_changed_ = true;
}

void Cloud::set_key(const float* key_data) {
    key_data_ = std::make_shared<std::vector<float>>(4 * n_, 1.0f);
    const float* end = key_data + n_;
    float* dst = key_data_->data();
    while (key_data != end) {
        *dst = *(key_data++);
        dst += 4;
    }
    key_changed_ = true;
    mono_ = true;
}

void Cloud::set_key_rgb(const float* key_rgb_data) {
    // 3 color channels per point (RGB)
    key_data_ = std::make_shared<std::vector<float>>(4 * n_, 1.0f);
    const float* end = key_rgb_data + 3 * n_;
    float* dst = key_data_->data();
    while (key_rgb_data != end) {
        *(dst++) = *(key_rgb_data++);
        *(dst++) = *(key_rgb_data++);
        *(dst++) = *(key_rgb_data++);
        dst++;
    }
    key_changed_ = true;
    mono_ = false;
}

void Cloud::set_key_rgba(const float* key_rgba_data) {
    // 4 color channels per point (RGBA)
    key_data_ = std::make_shared<std::vector<float>>(4 * n_, 0);
    const float* end = key_rgba_data + 4 * n_;
    float* dst = key_data_->data();
    while (key_rgba_data != end) {
        *(dst++) = *(key_rgba_data++);
        *(dst++) = *(key_rgba_data++);
        *(dst++) = *(key_rgba_data++);
        *(dst++) = *(key_rgba_data++);
    }
    key_changed_ = true;
    mono_ = false;
}

void Cloud::set_mask(const float* mask_data) {
    mask_data_ = std::make_shared<std::vector<float>>(4 * n_, 0);
    std::copy(mask_data, mask_data + 4 * n_, mask_data_->begin());
    mask_changed_ = true;
}

void Cloud::set_xyz(const float* xyz) {
    xyz_data_ = std::make_shared<std::vector<float>>(3 * n_, 0);
    for (size_t i = 0; i < n_; i++) {
        for (size_t k = 0; k < 3; k++) {
            (*xyz_data_)[3 * i + k] = xyz[i + n_ * k];
        }
    }
    xyz_changed_ = true;
}

void Cloud::set_offset(const float* offset) {
    off_data_ = std::make_shared<std::vector<float>>(3 * n_, 0);
    for (size_t i = 0; i < n_; i++) {
        for (size_t k = 0; k < 3; k++) {
            (*off_data_)[3 * i + k] = offset[i + n_ * k];
        }
    }
    offset_changed_ = true;
}

void Cloud::set_point_size(float size) {
    point_size_ = size;
    point_size_changed_ = true;
}

void Cloud::set_pose(const mat4d& pose) {
    pose_ = pose;
    pose_changed_ = true;
}

void Cloud::set_column_poses(const float* rotation, const float* translation) {
    transform_data_ = std::make_shared<std::vector<float>>(12 * w_, 0);
    for (size_t v = 0; v < w_; v++) {
        for (size_t u = 0; u < 3; u++) {
            for (size_t rgb = 0; rgb < 3; rgb++) {
                (*transform_data_)[(u * w_ + v) * 3 + rgb] =
                    rotation[v + u * w_ + 3 * rgb * w_];
            }
        }
        for (size_t rgb = 0; rgb < 3; rgb++) {
            (*transform_data_)[9 * w_ + 3 * v + rgb] =
                translation[v + rgb * w_];
        }
    }
    transform_changed_ = true;
}

void Cloud::set_column_poses(const float* column_poses) {
    // columns_poses: is [Wx4x4] and column-major storage
    transform_data_ = std::make_shared<std::vector<float>>(12 * w_, 0);
    for (size_t v = 0; v < w_; v++) {
        for (size_t u = 0; u < 3; u++) {
            for (size_t r = 0; r < 3; r++) {
                (*transform_data_)[(r * w_ + v) * 3 + u] =
                    column_poses[(r * 4 + u) * w_ + v];
            }
        }
        for (size_t u = 0; u < 3; u++) {
            (*transform_data_)[9 * w_ + 3 * v + u] =
                column_poses[(3 * 4 + u) * w_ + v];
        }
    }
    transform_changed_ = true;
}

void Cloud::set_palette(const float* palette, size_t palette_size) {
    palette_data_ = std::make_shared<std::vector<float>>(palette_size * 3);
    std::copy(palette, palette + (palette_size * 3), palette_data_->begin());
    palette_changed_ = true;
}

Image::Image() = default;

void Image::update_from(const Image& other) {
    bool position_changed = other.position_changed_ || position_changed_;
    bool image_changed = other.image_changed_ || image_changed_;
    bool mask_changed = other.mask_changed_ || mask_changed_;
    bool palette_changed = other.palette_changed_ || palette_changed_;
    *this = other;
    this->position_changed_ = position_changed;
    this->image_changed_ = image_changed;
    this->mask_changed_ = mask_changed;
    this->palette_changed_ = palette_changed;
}

void Image::clear() {
    position_changed_ = false;
    image_changed_ = false;
    mask_changed_ = false;
    palette_changed_ = false;
}

void Image::set_image(size_t width, size_t height, const float* image_data) {
    if (width < 1 || height < 1) {
        throw std::invalid_argument("invalid image size");
    }
    if (!image_data) {
        throw std::invalid_argument("null image data");
    }
    const size_t n = width * height;
    image_data_.resize(4 * n);
    image_width_ = width;
    image_height_ = height;
    const float* end = image_data + n;
    float* dst = image_data_.data();
    while (image_data != end) {
        *(dst + 0) = *(image_data++);
        *(dst + 3) = 1.0;  // alpha
        dst += 4;
    }
    image_changed_ = true;
    mono_ = true;
}

void Image::set_image_rgb(size_t width, size_t height,
                          const float* image_data_rgb) {
    if (width < 1 || height < 1) {
        throw std::invalid_argument("invalid image size");
    }
    if (!image_data_rgb) {
        throw std::invalid_argument("null image data");
    }
    const size_t n = width * height;
    image_data_.resize(4 * n);
    image_width_ = width;
    image_height_ = height;
    const float* end = image_data_rgb + 3 * n;
    float* dst = image_data_.data();
    while (image_data_rgb != end) {
        *(dst++) = *(image_data_rgb++);
        *(dst++) = *(image_data_rgb++);
        *(dst++) = *(image_data_rgb++);
        *(dst++) = 1.0;  // alpha
    }
    image_changed_ = true;
    mono_ = false;
}

void Image::set_image_rgba(size_t width, size_t height,
                           const float* image_data_rgba) {
    if (width < 1 || height < 1) {
        throw std::invalid_argument("invalid image size");
    }
    if (!image_data_rgba) {
        throw std::invalid_argument("null image data");
    }
    const size_t n = width * height;
    image_data_.resize(4 * n);
    image_width_ = width;
    image_height_ = height;
    const float* end = image_data_rgba + 4 * n;
    float* dst = image_data_.data();
    while (image_data_rgba != end) {
        *(dst++) = *(image_data_rgba++);
        *(dst++) = *(image_data_rgba++);
        *(dst++) = *(image_data_rgba++);
        *(dst++) = *(image_data_rgba++);
    }
    image_changed_ = true;
    mono_ = false;
}

void Image::set_mask(size_t width, size_t height, const float* mask_data) {
    if (width < 1 || height < 1) {
        throw std::invalid_argument("invalid mask size");
    }
    if (!mask_data) {
        throw std::invalid_argument("null mask data");
    }
    size_t n = width * height * 4;
    mask_data_.resize(n);
    mask_width_ = width;
    mask_height_ = height;
    std::copy(mask_data, mask_data + n, mask_data_.begin());
    mask_changed_ = true;
}

void Image::set_position(float x_min, float x_max, float y_min, float y_max) {
    position_ = {x_min, x_max, y_max, y_min};
    position_changed_ = true;
}

void Image::set_hshift(float hshift) {
    hshift_ = hshift;
    position_changed_ = true;
}

void Image::set_palette(const float* palette, size_t palette_size) {
    if (!palette) {
        throw std::invalid_argument("null palette");
    }
    palette_data_.resize(palette_size * 3);
    std::copy(palette, palette + (palette_size * 3), palette_data_.begin());
    palette_changed_ = true;
    use_palette_ = true;
}

void Image::clear_palette() {
    palette_data_.clear();
    palette_changed_ = true;
    use_palette_ = false;
}

void WindowCtx::check_invariants() const {
    if (window_width < 1 || window_height < 1) {
        throw std::logic_error("invalid window size");
    }
    if (viewport_width < 1 || viewport_height < 1) {
        throw std::logic_error("invalid viewport size");
    }
}

double WindowCtx::aspect_ratio() const {
    check_invariants();
    return static_cast<double>(window_width) / window_height;
}

std::pair<double, double> WindowCtx::normalized_coordinates(double x,
                                                            double y) const {
    check_invariants();
    double world_x = (2.0 / window_width * x - 1.0) * aspect_ratio();
    double world_y = 2.0 * (1.0 - (y / window_height)) - 1.0;
    return std::pair<double, double>(world_x, world_y);
}

std::pair<double, double> WindowCtx::window_coordinates(
    double normalized_x, double normalized_y) const {
    check_invariants();
    double window_x = (normalized_x + aspect_ratio()) * window_height / 2.0;
    double window_y = window_height * (1 - normalized_y) / 2.0;
    return std::pair<double, double>(window_x, window_y);
}

nonstd::optional<std::pair<int, int>> Image::window_coordinates_to_image_pixel(
    const WindowCtx& ctx, double x, double y) const {
    ctx.check_invariants();
    auto world = ctx.normalized_coordinates(x, y);

    // compute image pixel coordinates, accounting for hshift
    nonstd::optional<std::pair<int, int>> pixel;
    double mx = world.first - hshift_ * ctx.aspect_ratio();
    double my = world.second;

    // Important! position_ values are in a different order than the parameters
    // to set_position. :-/
    if (mx >= position_[0] && mx <= position_[1] && my >= position_[3] &&
        my <= position_[2]) {
        double img_rel_x = (mx - position_[0]) / (position_[1] - position_[0]);
        double img_rel_y = (position_[2] - my) / (position_[2] - position_[3]);
        int px = static_cast<int>(img_rel_x * image_width_);
        int py = static_cast<int>(img_rel_y * image_height_);
        pixel = std::pair<int, int>(px, py);
    }
    return pixel;
}

std::pair<double, double> Image::image_pixel_to_window_coordinates(
    const WindowCtx& ctx, int px, int py) const {
    ctx.check_invariants();
    double img_rel_x = static_cast<double>(px) / image_width_;
    double img_rel_y = static_cast<double>(py) / image_height_;

    // Important! position_ values are in a different order than the parameters
    // to set_position. :-/
    double mx = img_rel_x * (position_[1] - position_[0]) + position_[0];
    double my = position_[2] - img_rel_y * (position_[2] - position_[3]);
    double wx = mx + hshift_ * ctx.aspect_ratio();
    auto psize = pixel_size(ctx);
    auto wcoords = ctx.window_coordinates(wx, my);

    // return the window pixel in the center of the image pixel
    return std::pair<double, double>(wcoords.first + psize.first / 2,
                                     wcoords.second + psize.second / 2);
}

std::pair<double, double> Image::pixel_size(const WindowCtx& ctx) const {
    ctx.check_invariants();
    auto lower_left = ctx.window_coordinates(position_[0], position_[3]);
    auto upper_right = ctx.window_coordinates(position_[1], position_[2]);
    return std::pair<double, double>(
        fabs(upper_right.first - lower_left.first) / image_width_,
        fabs(upper_right.second - lower_left.second) / image_height_);
}

Cuboid::Cuboid(const mat4d& pose, const std::array<float, 4>& rgba) {
    set_transform(pose);
    set_rgba(rgba);
}

void Cuboid::update_from(const Cuboid& other) {
    bool transform_changed = other.transform_changed_ || transform_changed_;
    bool rgba_changed = other.rgba_changed_ || rgba_changed_;
    *this = other;
    this->transform_changed_ = transform_changed;
    this->rgba_changed_ = rgba_changed;
}

void Cuboid::clear() {
    transform_changed_ = false;
    rgba_changed_ = false;
}

void Cuboid::set_transform(const mat4d& pose) {
    transform_ = pose;
    transform_changed_ = true;
}

void Cuboid::set_rgba(const std::array<float, 4>& rgba) {
    rgba_ = rgba;
    rgba_changed_ = true;
}

Label::Label(const std::string& text, const vec3d& position) {
    set_text(text);
    set_position(position);
}

Label::Label(const std::string& text, float x, float y, bool align_right,
             bool align_top) {
    set_text(text);
    set_position(x, y, align_right, align_top);
}

void Label::update_from(const Label& other) {
    bool pos_changed = other.pos_changed_ || pos_changed_;
    bool scale_changed = other.scale_changed_ || scale_changed_;
    bool text_changed = other.text_changed_ || text_changed_;
    bool rgba_changed = other.rgba_changed_ || rgba_changed_;
    *this = other;
    this->pos_changed_ = pos_changed;
    this->scale_changed_ = scale_changed;
    this->text_changed_ = text_changed;
    this->rgba_changed_ = rgba_changed;
}

void Label::clear() {
    text_changed_ = false;
    pos_changed_ = false;
    scale_changed_ = false;
    rgba_changed_ = false;
}

void Label::dirty() {
    text_changed_ = true;
    pos_changed_ = true;
    scale_changed_ = true;
    rgba_changed_ = true;
}

void Label::set_position(const vec3d& position) {
    position_ = position;
    pos_changed_ = true;
    is_3d_ = true;
}

void Label::set_position(float x, float y, bool align_right, bool align_top) {
    position_ = {x, y, 0};
    align_right_ = align_right;
    align_top_ = align_top;
    pos_changed_ = true;
    is_3d_ = false;
}

void Label::set_scale(float scale) {
    scale_ = scale;
    scale_changed_ = true;
}

void Label::set_text(const std::string& text) {
    text_ = text;
    text_changed_ = true;
}

void Label::set_rgba(const std::array<float, 4>& rgba) {
    rgba_ = rgba;
    rgba_changed_ = true;
}

void TargetDisplay::enable_rings(bool state) { rings_enabled_ = state; }

void TargetDisplay::set_ring_size(int n) { ring_size_ = n; }
void TargetDisplay::set_ring_line_width(int line_width) {
    ring_line_width_ = line_width;
}

void add_default_controls(viz::PointViz& viz, std::mutex* mx) {
    bool orthographic = false;

    viz.push_key_handler(
        [=, &viz](const WindowCtx&, int key, int mods) mutable {
            auto lock = mx ? std::unique_lock<std::mutex>{*mx}
                           : std::unique_lock<std::mutex>{};
            if (mods == 0) {
                switch (key) {
                    case GLFW_KEY_W:
                        viz.camera().pitch(5);
                        viz.current_camera().pitch(5);
                        break;
                    case GLFW_KEY_S:
                        viz.camera().pitch(-5);
                        viz.current_camera().pitch(-5);
                        break;
                    case GLFW_KEY_A:
                        viz.camera().yaw(5);
                        viz.current_camera().yaw(5);
                        break;
                    case GLFW_KEY_D:
                        viz.camera().yaw(-5);
                        viz.current_camera().yaw(-5);
                        break;
                    case GLFW_KEY_EQUAL:
                        viz.camera().dolly(5);
                        viz.current_camera().dolly(5);
                        break;
                    case GLFW_KEY_MINUS:
                        viz.camera().dolly(-5);
                        viz.current_camera().dolly(-5);
                        break;
                    case GLFW_KEY_0:
                        orthographic = !orthographic;
                        viz.camera().set_orthographic(orthographic);
                        viz.current_camera().set_orthographic(orthographic);
                        break;
                    case GLFW_KEY_ESCAPE:
                        viz.running(false);
                        break;
                    default:
                        break;
                }
            } else if (mods == GLFW_MOD_SHIFT) {
                switch (key) {
                    case GLFW_KEY_R:
                        viz.camera().reset();
                        viz.current_camera().reset();
                        break;
                    default:
                        break;
                }
            } else if (mods == GLFW_MOD_CONTROL) {
                switch (key) {
                    case GLFW_KEY_R:
                        viz.camera().birds_eye_view();
                        viz.current_camera().birds_eye_view();
                        break;
                    default:
                        break;
                }
            }
            return true;
        });

    viz.push_scroll_handler([=, &viz](const WindowCtx&, double, double yoff) {
        auto lock = mx ? std::unique_lock<std::mutex>{*mx}
                       : std::unique_lock<std::mutex>{};
        viz.camera().dolly(static_cast<int>(yoff * 5));
        viz.current_camera().dolly(static_cast<int>(yoff * 5));
        return true;
    });

    viz.push_mouse_pos_handler(
        [=, &viz](const WindowCtx& wc, double xpos, double ypos) {
            auto lock = mx ? std::unique_lock<std::mutex>{*mx}
                           : std::unique_lock<std::mutex>{};
            double dx = (xpos - wc.mouse_x);
            double dy = (ypos - wc.mouse_y);
            // orbit or dolly in xy
            if (wc.lbutton_down) {
                constexpr double sensitivity = 0.3;
                viz.camera().yaw(sensitivity * dx);
                viz.camera().pitch(sensitivity * dy);
                viz.current_camera().yaw(sensitivity * dx);
                viz.current_camera().pitch(sensitivity * dy);
            } else if (wc.mbutton_down) {
                // convert from screen coordinated to fractions of window size
                // TODO: factor out conversion?
                const double window_diagonal =
                    std::sqrt(wc.window_width * wc.window_width +
                              wc.window_height * wc.window_height);
                dx *= 2.0 / window_diagonal;
                dy *= 2.0 / window_diagonal;
                viz.camera().dolly_xy(dx, dy);
                viz.current_camera().dolly_xy(dx, dy);
            }
            return true;
        });
}

}  // namespace viz
}  // namespace ouster

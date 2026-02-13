/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/point_viz.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <nonstd/optional.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "camera.h"
#include "cloud.h"
#include "colormaps.h"
#include "framebuffer.h"
#include "glfw.h"
#include "image.h"
#include "indexed.h"
#include "mesh.h"
#include "misc.h"
#include "screenshot_utils.h"

static_assert(std::is_same<GLfloat, float>::value,
              "Platform has unexpected definition of GLfloat");

namespace ouster {
namespace sdk {
namespace viz {

namespace {

/***
 * @brief holds the state of a screenshot request when get_screenshot
 * is called from a thread different than the render thread. It is meant
 * for the pimpl object that holds the viz state and the data access is
 * protected by a mutex. In short, this class holds the data being passed from
 * thread to thread back and forth.
 */
class ScreenshotRequest {
   public:
    ScreenshotRequest(uint32_t width, uint32_t height)
        : width_(width), height_(height), pixels_(width * height * 3) {}

    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; };
    bool is_complete() const { return complete_; }
    void set_pixels(const std::vector<uint8_t>& pixels) {
        pixels_ = pixels;
        complete_ = true;
    }
    std::vector<uint8_t> release_pixels() { return std::move(pixels_); }

   private:
    uint32_t width_;
    uint32_t height_;
    bool complete_{false};
    std::vector<uint8_t> pixels_;
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
    bool camera_changed{false};
    bool drew_this_update{false};

    // state of the screen recording framebuffer
    std::mutex screen_recording_fb_mutex;
    std::unique_ptr<impl::Framebuffer> screen_recording_fb{nullptr};

    // state for screenshots
    std::mutex screenshot_request_mutex;
    std::unique_ptr<impl::Framebuffer> screenshot_fb{nullptr};
    std::unique_ptr<ScreenshotRequest> screenshot_request{nullptr};
    std::condition_variable screenshot_condition_variable;
    std::thread::id rendering_thread_id;

    Camera camera_back, camera_front;

    TargetDisplay target;
    impl::GLRings rings;

    Indexed<impl::GLCloud, Cloud> clouds;
    Indexed<impl::GLMesh, Mesh> meshes;
    Indexed<impl::GLCuboid, Cuboid> cuboids;
    Indexed<impl::GLLabel, Label> labels;
    Indexed<impl::GLImage, Image> images;
    Indexed<impl::GLLines, Lines> lines;
    impl::GLLabel notification_gllabel;
    Label notification_label{"", 1 - 0.025f, 0.075f, true, false};

    /// Time when the notification should stop being displayed
    std::chrono::time_point<std::chrono::steady_clock> notification_end_time{};
    double notification_duration_sec{};

    template <typename T>
    using Handlers = std::list<std::function<T>>;

    Handlers<bool(const WindowCtx&, int, int)> key_handlers;
    Handlers<bool(const WindowCtx&, ouster::sdk::viz::MouseButton,
                  ouster::sdk::viz::MouseButtonEvent,
                  ouster::sdk::viz::EventModifierKeys)>
        mouse_button_handlers;
    Handlers<bool(const WindowCtx&, double, double)> scroll_handlers;
    Handlers<bool(const WindowCtx&, double, double)> mouse_pos_handlers;

    Handlers<bool(const std::vector<uint8_t>& fb_data, int viewport_width,
                  int viewport_height)>
        frame_buffer_handlers;

    Handlers<bool(const WindowCtx&)> window_resize_handlers;

    double fps_last_time{0};
    uint64_t fps_frame_counter{0};
    double fps{0};
    vec4f background_color{0, 0, 0, 1};

    explicit Impl(std::unique_ptr<GLFWContext>&& glfw)
        : glfw{std::move(glfw)} {}
};

/*
 * PointViz interface
 */

PointViz::PointViz(const std::string& name, bool fix_aspect, int window_width,
                   int window_height, bool maximized, bool fullscreen,
                   bool borderless) {
    auto glfw = std::make_unique<GLFWContext>(name, fix_aspect, window_width,
                                              window_height, maximized,
                                              fullscreen, borderless);

    // set context for GL initialization
    glfwMakeContextCurrent(glfw->window);

    pimpl_ = std::make_unique<Impl>(std::move(glfw));

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // These two lines are confugurations necessary for every
    // glReadPixels call. If any of these settings is changed somewhere else
    // in the program, special attention needs to be put on the screenshot
    // functionality
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);

    // TODO: need to check if these were already called?
    impl::GLCloud::initialize();
    impl::GLImage::initialize();
    impl::GLRings::initialize();
    impl::GLMesh::initialize();
    impl::GLCuboid::initialize();
    impl::GLLines::initialize();

    // release context in case subsequent calls are done from another thread
    glfwMakeContextCurrent(nullptr);

    // add user-setable input handlers
    pimpl_->glfw->key_handler = [this](const WindowCtx& ctx, int key,
                                       int mods) {
        for (auto& handler : pimpl_->key_handlers) {
            if (!handler(ctx, key, mods)) {
                break;
            }
        }
    };
    pimpl_->glfw->mouse_button_handler =
        [this](const WindowCtx& ctx, int button, int action, int mods) {
            for (auto& handler : pimpl_->mouse_button_handlers) {
                if (!handler(ctx, ouster::sdk::viz::MouseButton(button),
                             ouster::sdk::viz::MouseButtonEvent(action),
                             ouster::sdk::viz::EventModifierKeys(mods))) {
                    break;
                }
            }
        };
    pimpl_->glfw->scroll_handler = [this](const WindowCtx& ctx, double x,
                                          double y) {
        for (auto& handler : pimpl_->scroll_handlers) {
            if (!handler(ctx, x, y)) {
                break;
            }
        }
    };
    pimpl_->glfw->mouse_pos_handler = [this](const WindowCtx& ctx, double x,
                                             double y) {
        for (auto& handler : pimpl_->mouse_pos_handlers) {
            if (!handler(ctx, x, y)) {
                break;
            }
        }
    };

    // glfwPollEvents blocks during resize on macos. Keep rendering to avoid
    // artifacts during resize
    pimpl_->glfw->resize_handler = [this](const WindowCtx& ctx) {
        for (auto& handler : pimpl_->window_resize_handlers) {
            if (!handler(ctx)) {
                break;
            }
        }
        pimpl_->camera_changed = true;  // force a render
#ifdef __APPLE__
        process_frame();
#endif
    };
}

PointViz::~PointViz() = default;

void PointViz::add_default_controls(std::mutex* mx) {
    bool orthographic = false;

    auto move_camera = [this](int key, float amount) {
        switch (key) {
            case GLFW_KEY_A:
                this->camera().yaw(amount);
                this->current_camera().yaw(amount);
                return true;
            case GLFW_KEY_D:
                this->camera().yaw(-amount);
                this->current_camera().yaw(-amount);
                return true;
            case GLFW_KEY_W:
                this->camera().pitch(amount);
                this->current_camera().pitch(amount);
                return true;
            case GLFW_KEY_S:
                this->camera().pitch(-amount);
                this->current_camera().pitch(-amount);
                return true;
            case GLFW_KEY_EQUAL:
                this->camera().dolly(amount);
                this->current_camera().dolly(amount);
                return true;
            case GLFW_KEY_MINUS:
                this->camera().dolly(-amount);
                this->current_camera().dolly(-amount);
                return true;
            case GLFW_KEY_Q:
                this->camera().roll(amount);
                this->current_camera().roll(amount);
                return true;
            case GLFW_KEY_E:
                this->camera().roll(-amount);
                this->current_camera().roll(-amount);
                return true;
            default:
                return false;
        }
    };

    this->push_key_handler([this, mx, orthographic, move_camera](
                               const WindowCtx&, int key, int mods) mutable {
        auto lock = mx ? std::unique_lock<std::mutex>{*mx}
                       : std::unique_lock<std::mutex>{};
        if (mods == 0) {
            if (move_camera(key, 5)) {
                pimpl_->camera_changed = true;
                return true;
            }
            switch (key) {
                case GLFW_KEY_0:
                    orthographic = !orthographic;
                    this->camera().set_orthographic(orthographic);
                    this->current_camera().set_orthographic(orthographic);
                    pimpl_->camera_changed = true;
                    set_notification("Camera mode: " +
                                     std::string(orthographic ? "ORTHOGRAPHIC"
                                                              : "PERSPECTIVE"));
                    break;
                case GLFW_KEY_ESCAPE:
                    this->running(false);
                    break;
                default:
                    break;
            }
        } else if (mods == GLFW_MOD_SHIFT) {
            if (move_camera(key, 0.3)) {
                pimpl_->camera_changed = true;
                return true;
            }
            switch (key) {
                case GLFW_KEY_R:
                    this->camera().reset();
                    this->current_camera().reset();
                    pimpl_->camera_changed = true;
                    break;
                default:
                    break;
            }
        } else if (mods == GLFW_MOD_CONTROL) {
            switch (key) {
                case GLFW_KEY_R:
                    this->camera().birds_eye_view();
                    this->current_camera().birds_eye_view();
                    pimpl_->camera_changed = true;
                    break;
                default:
                    break;
            }
        }
        return true;
    });

    this->push_scroll_handler(
        [this, mx](const WindowCtx&, double, double yoff) {
            auto lock = mx ? std::unique_lock<std::mutex>{*mx}
                           : std::unique_lock<std::mutex>{};
            this->camera().dolly(static_cast<int>(yoff * 5));
            this->current_camera().dolly(static_cast<int>(yoff * 5));
            pimpl_->camera_changed = true;
            return true;
        });

    this->push_mouse_pos_handler(
        [this, mx](const WindowCtx& wc, double xpos, double ypos) {
            auto lock = mx ? std::unique_lock<std::mutex>{*mx}
                           : std::unique_lock<std::mutex>{};
            double dx = (xpos - wc.mouse_x);
            double dy = (ypos - wc.mouse_y);
            // orbit or dolly in xy
            if (wc.lbutton_down) {
                constexpr double sensitivity = 0.3;
                this->camera().yaw(sensitivity * dx);
                this->camera().pitch(sensitivity * dy);
                this->current_camera().yaw(sensitivity * dx);
                this->current_camera().pitch(sensitivity * dy);
                pimpl_->camera_changed = true;
            } else if (wc.mbutton_down) {
                // convert from screen coordinated to fractions of window size
                // TODO: factor out conversion?
                const double window_diagonal =
                    std::sqrt((wc.window_width * wc.window_width) +
                              (wc.window_height * wc.window_height));
                dx *= 2.0 / window_diagonal;
                dy *= 2.0 / window_diagonal;
                this->camera().dolly_xy(dx, dy);
                this->current_camera().dolly_xy(dx, dy);
                pimpl_->camera_changed = true;
            }
            return true;
        });
}

void PointViz::run() {
    pimpl_->glfw->running(true);
    pimpl_->glfw->visible(true);
    while (running()) {
        run_once();
    }
    pimpl_->glfw->visible(false);
}

void PointViz::run_once() {
    if (glfwGetCurrentContext() != pimpl_->glfw->window) {
        glfwMakeContextCurrent(pimpl_->glfw->window);
    }
    process_frame();
    glfwPollEvents();
}

void PointViz::cursor_visible(bool state) {
    glfwSetInputMode(pimpl_->glfw->window, GLFW_CURSOR,
                     state ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_HIDDEN);
}

bool PointViz::running() { return pimpl_->glfw->running(); }

void PointViz::running(bool state) {
    std::unique_lock<std::mutex> lock(pimpl_->screenshot_request_mutex);
    pimpl_->glfw->running(state);
}

void PointViz::visible(bool state) { pimpl_->glfw->visible(state); }

void PointViz::update() {
    std::lock_guard<std::mutex> guard{pimpl_->update_mx};

    // TWS 20241014: note we used to return here if
    // the last frame hasn't been drawn yet.
    // However, this can cause problem if the API user calls
    // update(), especially if more than once, before
    // calling run().
    // We're preserving front_changed in case we want
    // to expose this value via the API later on.
    // if (pimpl_->front_changed) return false;

    // propagate camera changes
    pimpl_->camera_front = pimpl_->camera_back;

    pimpl_->clouds.update_new_and_existing_drawables();
    pimpl_->meshes.update_new_and_existing_drawables();
    pimpl_->cuboids.update_new_and_existing_drawables();
    pimpl_->labels.update_new_and_existing_drawables();
    pimpl_->images.update_new_and_existing_drawables();
    pimpl_->lines.update_new_and_existing_drawables();
    pimpl_->rings.update(pimpl_->target);

    pimpl_->front_changed = true;
}

int PointViz::viewport_width() const {
    return pimpl_->glfw->window_context.viewport_width;
}

int PointViz::viewport_height() const {
    return pimpl_->glfw->window_context.viewport_height;
}

int PointViz::window_width() const {
    return pimpl_->glfw->window_context.window_width;
}

int PointViz::window_height() const {
    return pimpl_->glfw->window_context.window_height;
}

void PointViz::count_fps() {
    // fps counting
    ++pimpl_->fps_frame_counter;
    double now_t = glfwGetTime();
    if (pimpl_->fps_last_time == 0 || now_t - pimpl_->fps_last_time >= 1.0) {
        pimpl_->fps =
            pimpl_->fps_frame_counter / (now_t - pimpl_->fps_last_time);
        pimpl_->fps_last_time = now_t;
        pimpl_->fps_frame_counter = 0;
    }
}

void PointViz::draw() {
    glClearColor(pimpl_->background_color[0], pimpl_->background_color[1],
                 pimpl_->background_color[2], pimpl_->background_color[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const auto& ctx = pimpl_->glfw->window_context;

    // calculate camera matrices
    auto camera_data = pimpl_->camera_front.matrices(impl::window_aspect(ctx));

    // draw rings
    pimpl_->rings.draw(ctx, camera_data);

    // draw clouds
    impl::GLCloud::beginDraw();
    pimpl_->clouds.draw(ctx, camera_data);
    impl::GLCloud::endDraw();

    // draw meshes
    impl::GLMesh::beginDraw();
    pimpl_->meshes.draw(ctx, camera_data);
    impl::GLMesh::endDraw();

    // draw cuboids
    impl::GLCuboid::beginDraw();
    pimpl_->cuboids.draw(ctx, camera_data);
    impl::GLCuboid::endDraw();

    // draw lines
    impl::GLLines::beginDraw();
    pimpl_->lines.draw(ctx, camera_data);
    impl::GLLines::endDraw();

    // draw labels and images on top of everything
    glClear(GL_DEPTH_BUFFER_BIT);

    // draw image
    impl::GLImage::beginDraw();
    pimpl_->images.draw(ctx, camera_data);
    impl::GLImage::endDraw();

    // draw labels
    impl::GLLabel::beginDraw();
    pimpl_->labels.draw(ctx, camera_data);

    // draw notification label
    auto time_remaining =
        std::chrono::duration<float>(pimpl_->notification_end_time -
                                     std::chrono::steady_clock::now())
            .count();
    if (time_remaining > 0.0f) {
        constexpr float fade_out_time = 0.75f;  // seconds
        if (time_remaining < fade_out_time) {
            // fade out in the last second
            float alpha =
                (std::cos(M_PI * (time_remaining / fade_out_time + 1.0f)) +
                 1.0f) /
                2.0f;
            pimpl_->notification_label.set_rgba({1.0f, 1.0f, 1.0f, alpha});
        }
        pimpl_->notification_gllabel.draw(ctx, camera_data,
                                          pimpl_->notification_label);
    }
    impl::GLLabel::endDraw();
}

std::vector<uint8_t> PointViz::capture_framebuffer_pixels(
    impl::Framebuffer& framebuffer) {
    int current_fb_width{0};
    int current_fb_height{0};
    glfwGetFramebufferSize(pimpl_->glfw->window, &current_fb_width,
                           &current_fb_height);

    if (framebuffer.width() == current_fb_width &&
        framebuffer.height() == current_fb_height) {
        if (!pimpl_->drew_this_update) {
            draw();
            pimpl_->drew_this_update = true;
        }

        // Desired size is equal to window framebuffer size, just read
        // from the default framebuffer and return.
        // Note: on a Mac it is likely that this framebuffer size will be equal
        // to 2 times the window logical width and height
        std::vector<uint8_t> result(static_cast<std::size_t>(
            framebuffer.width() * framebuffer.height() * 3));

        glReadPixels(0, 0, static_cast<GLsizei>(framebuffer.width()),
                     static_cast<GLsizei>(framebuffer.height()), GL_RGB,
                     GL_UNSIGNED_BYTE, result.data());

        return result;
    }

    std::vector<uint8_t> result(static_cast<std::size_t>(
        framebuffer.width() * framebuffer.height() * 3));

    // Resolution is different to window size
    framebuffer.bind();
    if (framebuffer.is_complete()) {
        glViewport(0, 0, framebuffer.width(), framebuffer.height());

        draw();
        framebuffer.read_pixels_into(result);

        // Restore the viewport to the window size
        glViewport(0, 0, static_cast<GLsizei>(current_fb_width),
                   static_cast<GLsizei>(current_fb_height));
    }
    framebuffer.unbind();

    return result;
}

void PointViz::set_background_color(const vec4f& rgba) {
    pimpl_->background_color = rgba;
}

void PointViz::process_frame() {
    // Store this thread id to later identify the rendering thread
    // for methods that could be run from any thread, like get_screenshot()
    pimpl_->rendering_thread_id = std::this_thread::get_id();

    // Draw
    {
        std::lock_guard<std::mutex> guard{pimpl_->update_mx};
        if (pimpl_->front_changed || pimpl_->camera_changed) {
            count_fps();
        }
        pimpl_->clouds.clean_up_removed_drawables();
        pimpl_->meshes.clean_up_removed_drawables();
        pimpl_->cuboids.clean_up_removed_drawables();
        pimpl_->labels.clean_up_removed_drawables();
        pimpl_->images.clean_up_removed_drawables();
        pimpl_->lines.clean_up_removed_drawables();
        if (pimpl_->front_changed || pimpl_->camera_changed ||
            (notifications_enabled && pimpl_->notification_end_time >=
                                          std::chrono::steady_clock::now())) {
            pimpl_->drew_this_update = true;
            draw();
        } else {
            pimpl_->drew_this_update = false;
        }
        // mark front buffers no longer dirty
        pimpl_->front_changed = false;
        pimpl_->camera_changed = false;
    }
    handle_screenshot_request();
    handle_recording();
    if (pimpl_->drew_this_update) {
        glfwSwapBuffers(pimpl_->glfw->window);
    } else {
        // sleep a bit to not use 100% cpu
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void PointViz::handle_screenshot_request() {
    std::unique_lock<std::mutex> lock(pimpl_->screenshot_request_mutex);
    if (pimpl_->screenshot_request) {
        // Create the screenshot framebuffer with its desired texture size
        pimpl_->screenshot_fb = std::make_unique<impl::Framebuffer>(
            pimpl_->screenshot_request->width(),
            pimpl_->screenshot_request->height());

        auto pixels = capture_framebuffer_pixels(*pimpl_->screenshot_fb);

        // Dispose the screenshot framebuffer and its data
        pimpl_->screenshot_fb = nullptr;

        pimpl_->screenshot_request->set_pixels(pixels);
        pimpl_->screenshot_condition_variable.notify_one();
    }
}

void PointViz::handle_recording() {
    std::lock_guard<std::mutex> guard(pimpl_->screen_recording_fb_mutex);

    // Save screenshots from the recording framebuffer if recording
    if (pimpl_->screen_recording_fb) {
        // Save the screenshot
        auto pixels = capture_framebuffer_pixels(*pimpl_->screen_recording_fb);
        impl::screenshot_utils::flip_pixels(
            pixels, pimpl_->screen_recording_fb->width(),
            pimpl_->screen_recording_fb->height());
        impl::screenshot_utils::write_png(
            "", pixels, pimpl_->screen_recording_fb->width(),
            pimpl_->screen_recording_fb->height());
    }
}

double PointViz::fps() const { return pimpl_->fps; }

/*
 * Input handling
 */
void PointViz::push_key_handler(
    std::function<bool(const WindowCtx&, int, int)>&& callback) {
    // TODO: not thread safe: called in glfwPollEvents()
    pimpl_->key_handlers.push_front(std::move(callback));
}

void PointViz::push_mouse_button_handler(
    std::function<bool(const WindowCtx&, ouster::sdk::viz::MouseButton,
                       ouster::sdk::viz::MouseButtonEvent,
                       ouster::sdk::viz::EventModifierKeys)>&& callback) {
    pimpl_->mouse_button_handlers.push_front(std::move(callback));
}

void PointViz::push_scroll_handler(
    std::function<bool(const WindowCtx&, double, double)>&& callback) {
    pimpl_->scroll_handlers.push_front(std::move(callback));
}

void PointViz::push_mouse_pos_handler(
    std::function<bool(const WindowCtx&, double, double)>&& callback) {
    pimpl_->mouse_pos_handlers.push_front(std::move(callback));
}

void PointViz::pop_key_handler() { pimpl_->key_handlers.pop_front(); }

void PointViz::pop_mouse_button_handler() {
    pimpl_->mouse_button_handlers.pop_front();
}
void PointViz::pop_scroll_handler() { pimpl_->scroll_handlers.pop_front(); }

void PointViz::pop_mouse_pos_handler() {
    pimpl_->mouse_pos_handlers.pop_front();
}

void PointViz::push_frame_buffer_resize_handler(
    std::function<bool(const WindowCtx&)>&& callback) {
    pimpl_->window_resize_handlers.push_front(std::move(callback));
}

void PointViz::pop_frame_buffer_resize_handler() {
    pimpl_->window_resize_handlers.pop_front();
}

/*
 * Add / remove / access objects in the scene
 */
Camera& PointViz::camera() { return pimpl_->camera_back; }

Camera& PointViz::current_camera() { return pimpl_->camera_front; }

TargetDisplay& PointViz::target_display() { return pimpl_->target; }

void PointViz::add(const std::shared_ptr<Cloud>& cloud) {
    pimpl_->clouds.add(cloud);
}

void PointViz::add(const std::shared_ptr<Mesh>& mesh) {
    pimpl_->meshes.add(mesh);
}

void PointViz::add(const std::shared_ptr<Cuboid>& cuboid) {
    pimpl_->cuboids.add(cuboid);
}

void PointViz::add(const std::shared_ptr<Label>& label) {
    label->dirty();
    pimpl_->labels.add(label);
}

void PointViz::add(const std::shared_ptr<Image>& image) {
    pimpl_->images.add(image);
}

void PointViz::add(const std::shared_ptr<Lines>& lines) {
    pimpl_->lines.add(lines);
}

bool PointViz::remove(const std::shared_ptr<Cloud>& cloud) {
    return pimpl_->clouds.remove(cloud);
}

bool PointViz::remove(const std::shared_ptr<Mesh>& mesh) {
    return pimpl_->meshes.remove(mesh);
}

bool PointViz::remove(const std::shared_ptr<Cuboid>& cuboid) {
    return pimpl_->cuboids.remove(cuboid);
}

bool PointViz::remove(const std::shared_ptr<Label>& label) {
    return pimpl_->labels.remove(label);
}

bool PointViz::remove(const std::shared_ptr<Image>& image) {
    return pimpl_->images.remove(image);
}

bool PointViz::remove(const std::shared_ptr<Lines>& lines) {
    return pimpl_->lines.remove(lines);
}

std::pair<uint32_t, uint32_t> PointViz::get_scaled_viewport_size(
    double scale_factor) {
    if (scale_factor <= 0.0) {
        throw std::runtime_error("Invalid scale factor");
    }

    auto width =
        static_cast<uint32_t>(std::lround(viewport_width() * scale_factor));
    auto height =
        static_cast<uint32_t>(std::lround(viewport_height() * scale_factor));
    return std::make_pair(width, height);
}

std::vector<uint8_t> PointViz::get_screenshot(uint32_t width, uint32_t height) {
    std::vector<uint8_t> pixels;

    // determine if we have processed any frame yet by checking
    // if the rendering_thread_id value is not the default
    bool has_rendered_before =
        !(pimpl_->rendering_thread_id == std::thread::id());

    if (std::this_thread::get_id() == pimpl_->rendering_thread_id ||
        !has_rendered_before) {
        // This is the either the rendering thread or we have not yet rendered
        // anything, get the screenshot synchronously

        // If we have not yet rendered anything, we want to do a first render
        // before the screenshot happens
        if (!has_rendered_before) {
            if (glfwGetCurrentContext() != pimpl_->glfw->window) {
                glfwMakeContextCurrent(pimpl_->glfw->window);
            }
            draw();
        }

        // Create the screenshot framebuffer with its desired texture size
        pimpl_->screenshot_fb =
            std::make_unique<impl::Framebuffer>(width, height);

        pixels = capture_framebuffer_pixels(*pimpl_->screenshot_fb);

        // Dispose the screenshot framebuffer and its data
        pimpl_->screenshot_fb = nullptr;
    } else {
        // get_screenshot is being called from a different thread than the main
        // thread. Request the screenshot asynchronously.

        // Submit screenshot request
        {
            std::unique_lock<std::mutex> lock(pimpl_->screenshot_request_mutex);
            if (!running()) {
                throw PointVizNotRunningError();
            }
            if (!pimpl_->screenshot_request) {
                pimpl_->screenshot_request =
                    std::make_unique<ScreenshotRequest>(width, height);
            } else {
                throw std::runtime_error(
                    "A screenshot request is already in progress. Only one "
                    "screenshot request can be processed at a time.");
            }

            // Block until the screenshot is complete
            pimpl_->screenshot_condition_variable.wait(lock, [this]() {
                if (!pimpl_->screenshot_request) {
                    return false;
                }

                return pimpl_->screenshot_request->is_complete();
            });
            pixels = pimpl_->screenshot_request->release_pixels();

            // Delete the screenshot request to signal no pending requests
            // exist.
            pimpl_->screenshot_request = nullptr;
        }
    }

    impl::screenshot_utils::flip_pixels(pixels, width, height);
    return pixels;
}

std::vector<uint8_t> PointViz::get_screenshot(double scale_factor) {
    auto size = get_scaled_viewport_size(scale_factor);
    return get_screenshot(size.first, size.second);
}

std::string PointViz::save_screenshot(const std::string& path, uint32_t width,
                                      uint32_t height) {
    auto pixels = get_screenshot(width, height);
    return impl::screenshot_utils::write_png(path, pixels, width, height);
}

std::string PointViz::save_screenshot(const std::string& path,
                                      double scale_factor) {
    auto size = get_scaled_viewport_size(scale_factor);
    return save_screenshot(path, size.first, size.second);
}

bool PointViz::toggle_screen_recording(uint32_t width, uint32_t height) {
    if (glfwGetCurrentContext() != pimpl_->glfw->window) {
        glfwMakeContextCurrent(pimpl_->glfw->window);
    }

    std::lock_guard<std::mutex> guard(pimpl_->screen_recording_fb_mutex);

    // If we were already recording, destroy the screen_recording_fb
    // and therefore stop recording
    if (pimpl_->screen_recording_fb) {
        pimpl_->screen_recording_fb = nullptr;
        return false;
    }

    // Create the recording framebuffer
    pimpl_->screen_recording_fb =
        std::make_unique<impl::Framebuffer>(width, height);
    return true;
}

bool PointViz::toggle_screen_recording(double scale_factor) {
    auto size = get_scaled_viewport_size(scale_factor);
    return toggle_screen_recording(size.first, size.second);
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
          &SPEZIA_PALETTE[0][0], &SPEZIA_PALETTE[0][0] + (SPEZIA_N * 3))} {
    // initialize per-column poses to identity
    for (size_t v = 0; v < w; v++) {
        (*transform_data_)[3 * v] = 1;
        (*transform_data_)[(3 * (v + w)) + 1] = 1;
        (*transform_data_)[(3 * (v + 2 * w)) + 2] = 1;
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
Cloud::Cloud(size_t num_points, const mat4d& extrinsic)
    : Cloud{1, num_points, extrinsic} {
    // initialize ranges to 1, set_xyz() used for data
    Eigen::Array<uint32_t, 1, -1> ones =
        Eigen::Array<uint32_t, 1, -1>::Ones(num_points * 3);
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
    std::transform(x, x + n_, std::begin(*range_data_), [](uint32_t range_val) {
        return static_cast<float>(range_val);
    });
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
    const float* end = key_rgb_data + (3 * n_);
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
    const float* end = key_rgba_data + (4 * n_);
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
    std::copy(mask_data, mask_data + (4 * n_), mask_data_->begin());
    mask_changed_ = true;
}

void Cloud::set_xyz(const float* xyz) {
    xyz_data_ = std::make_shared<std::vector<float>>(3 * n_, 0);
    for (size_t i = 0; i < n_; i++) {
        for (size_t k = 0; k < 3; k++) {
            (*xyz_data_)[(3 * i) + k] = xyz[i + (n_ * k)];
        }
    }
    xyz_changed_ = true;
}

void Cloud::set_offset(const float* offset) {
    off_data_ = std::make_shared<std::vector<float>>(3 * n_, 0);
    for (size_t i = 0; i < n_; i++) {
        for (size_t k = 0; k < 3; k++) {
            (*off_data_)[(3 * i) + k] = offset[i + (n_ * k)];
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
                (*transform_data_)[((u * w_ + v) * 3) + rgb] =
                    rotation[v + (u * w_) + (3 * rgb * w_)];
            }
        }
        for (size_t rgb = 0; rgb < 3; rgb++) {
            (*transform_data_)[(9 * w_) + (3 * v) + rgb] =
                translation[v + (rgb * w_)];
        }
    }
    transform_changed_ = true;
}

void Cloud::set_column_poses(const float* column_poses) {
    // columns_poses: is [Wx4x4] and column-major storage
    transform_data_ = std::make_shared<std::vector<float>>(12 * w_, 0);
    for (size_t v = 0; v < w_; v++) {
        for (size_t u = 0; u < 3; u++) {
            for (size_t component = 0; component < 3; component++) {
                (*transform_data_)[((component * w_ + v) * 3) + u] =
                    column_poses[((component * 4 + u) * w_) + v];
            }
        }
        for (size_t u = 0; u < 3; u++) {
            (*transform_data_)[(9 * w_) + (3 * v) + u] =
                column_poses[((3 * 4 + u) * w_) + v];
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

void Image::dirty() {
    position_changed_ = true;
    image_changed_ = true;
    mask_changed_ = true;
    palette_changed_ = true;
}

void Image::set_image(size_t width, size_t height, const float* image_data) {
    if (width < 1 || height < 1) {
        throw std::invalid_argument("invalid image size");
    }
    if (image_data == nullptr) {
        throw std::invalid_argument("null image data");
    }
    const size_t num_pixels = width * height;
    image_data_.resize(4 * num_pixels);
    image_width_ = width;
    image_height_ = height;
    const float* end = image_data + num_pixels;
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
    if (image_data_rgb == nullptr) {
        throw std::invalid_argument("null image data");
    }
    const size_t num_pixels = width * height;
    image_data_.resize(4 * num_pixels);
    image_width_ = width;
    image_height_ = height;
    const float* end = image_data_rgb + (3 * num_pixels);
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
    if (image_data_rgba == nullptr) {
        throw std::invalid_argument("null image data");
    }
    const size_t num_pixels = width * height;
    image_data_.resize(4 * num_pixels);
    image_width_ = width;
    image_height_ = height;
    const float* end = image_data_rgba + (4 * num_pixels);
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
    if (mask_data == nullptr) {
        throw std::invalid_argument("null mask data");
    }
    size_t mask_data_size = width * height * 4;
    mask_data_.resize(mask_data_size);
    mask_width_ = width;
    mask_height_ = height;
    std::copy(mask_data, mask_data + mask_data_size, mask_data_.begin());
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
    if (palette == nullptr) {
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
    double world_y = (2.0 * (1.0 - (y / window_height))) - 1.0;
    return std::pair<double, double>(world_x, world_y);
}

std::pair<double, double> WindowCtx::viewport_coordinates(
    double normalized_x, double normalized_y) const {
    check_invariants();
    double window_x = (normalized_x + aspect_ratio()) * viewport_height / 2.0;
    double window_y = viewport_height * (1 - normalized_y) / 2.0;
    return std::pair<double, double>(window_x, window_y);
}

std::pair<int, int> Image::viewport_coordinates_to_image_pixel(
    const WindowCtx& ctx, double x, double y) const {
    ctx.check_invariants();
    if (image_width_ == 0 || image_height_ == 0) {
        throw std::runtime_error("image data has zero width or height");
    }
    // the image width or height in window coordinates is zero
    if (position_[1] - position_[0] == 0.0 ||
        position_[2] - position_[3] == 0.0) {
        throw std::runtime_error("image has an invalid position");
    }
    auto world = ctx.normalized_coordinates(x, y);

    // compute image pixel coordinates, accounting for hshift
    nonstd::optional<std::pair<int, int>> pixel;
    double mx = world.first - (hshift_ * ctx.aspect_ratio());
    double my = world.second;

    // Important! position_ values are in a different order than the parameters
    // to set_position. :-/
    double img_rel_x = (mx - position_[0]) / (position_[1] - position_[0]);
    double img_rel_y = (position_[2] - my) / (position_[2] - position_[3]);
    int px = static_cast<int>(img_rel_x * image_width_);
    int py = static_cast<int>(img_rel_y * image_height_);
    return std::make_pair(px, py);
}

std::pair<double, double> Image::image_pixel_to_viewport_coordinates(
    const WindowCtx& ctx, int px, int py) const {
    ctx.check_invariants();
    double img_rel_x = static_cast<double>(px) / image_width_;
    double img_rel_y = static_cast<double>(py) / image_height_;

    // Important! position_ values are in a different order than the parameters
    // to set_position. :-/
    double mx = (img_rel_x * (position_[1] - position_[0])) + position_[0];
    double my = position_[2] - (img_rel_y * (position_[2] - position_[3]));
    double wx = mx + (hshift_ * ctx.aspect_ratio());
    auto psize = pixel_size(ctx);
    auto vcoords = ctx.viewport_coordinates(wx, my);

    // return the window pixel in the center of the image pixel
    return std::pair<double, double>(vcoords.first + (psize.first / 2),
                                     vcoords.second + (psize.second / 2));
}

std::pair<double, double> Image::pixel_size(const WindowCtx& ctx) const {
    ctx.check_invariants();
    auto lower_left = ctx.viewport_coordinates(position_[0], position_[3]);
    auto upper_right = ctx.viewport_coordinates(position_[1], position_[2]);
    return std::pair<double, double>(
        fabs(upper_right.first - lower_left.first) / image_width_,
        fabs(upper_right.second - lower_left.second) / image_height_);
}

Mesh::Mesh(std::vector<Vertex3f>&& vertices,
           std::vector<unsigned int>&& face_indices,
           std::vector<unsigned int>&& edge_indices)
    : transform_{IDENTITY4D},
      vertices_(std::move(vertices)),
      face_indices_(std::move(face_indices)),
      edge_indices_(std::move(edge_indices)),
      face_rgba_{0.8f, 0.8f, 0.8f, 1.0f},
      edge_rgba_{0.8f, 0.8f, 0.8f, 1.0f} {}

Mesh::Mesh(const std::vector<Vertex3f>& vertices,
           const std::vector<unsigned int>& face_indices,
           const std::vector<unsigned int>& edge_indices, vec4f face_rgba,
           vec4f edge_rgba)
    : transform_{IDENTITY4D},
      vertices_(vertices),
      face_indices_(face_indices),
      edge_indices_(edge_indices),
      face_rgba_(face_rgba),
      edge_rgba_(edge_rgba) {}

Mesh Mesh::from_simple_mesh(const ouster::sdk::core::Mesh& mesh) {
    std::vector<Vertex3f> vertices;
    std::vector<unsigned int> face_indices;
    std::vector<unsigned int> edge_indices;
    unsigned int t_idx = 0;
    for (const auto& triangle : mesh.triangles()) {
        for (int c_idx = 0; c_idx < 3; c_idx++) {
            Eigen::Vector3f position{triangle.coords[c_idx].x(),
                                     triangle.coords[c_idx].y(),
                                     triangle.coords[c_idx].z()};
            Eigen::Vector3f normal{triangle.normal.x(), triangle.normal.y(),
                                   triangle.normal.z()};
            vertices.emplace_back(position, normal);
        }

        edge_indices.insert(edge_indices.end(),
                            {t_idx * 3, t_idx * 3 + 1, t_idx * 3 + 1,
                             t_idx * 3 + 2, t_idx * 3 + 2, t_idx * 3});
        face_indices.insert(face_indices.end(),
                            {t_idx * 3, t_idx * 3 + 1, t_idx * 3 + 2});
        t_idx++;
    }
    return Mesh(std::move(vertices), std::move(face_indices),
                std::move(edge_indices));
}

void Mesh::clear() {
    rgba_changed_ = false;
    transform_changed_ = false;
}

void Mesh::dirty() {
    rgba_changed_ = true;
    transform_changed_ = true;
}

void Mesh::set_transform(const mat4d& pose) {
    transform_ = pose;
    transform_changed_ = true;
}

void Mesh::update_from(const Mesh& other) {
    // TODO[tws] optimize
    *this = other;
}

void Mesh::set_face_rgba(const vec4f& rgba) {
    face_rgba_ = rgba;
    rgba_changed_ = true;
}

void Mesh::set_edge_rgba(const vec4f& rgba) {
    edge_rgba_ = rgba;
    rgba_changed_ = true;
}

Cuboid::Cuboid(const mat4d& pose, const vec4f& rgba) {
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

void Cuboid::dirty() {
    transform_changed_ = true;
    rgba_changed_ = true;
}

void Cuboid::set_transform(const mat4d& pose) {
    transform_ = pose;
    transform_changed_ = true;
}

void Cuboid::set_rgba(const vec4f& rgba) {
    rgba_ = rgba;
    rgba_changed_ = true;
}

Lines::Lines(const mat4d& pose, const vec4f& rgba) {
    set_transform(pose);
    set_rgba(rgba);
}

void Lines::update_from(const Lines& other) {
    bool transform_changed = other.transform_changed_ || transform_changed_;
    bool rgba_changed = other.rgba_changed_ || rgba_changed_;
    bool points_changed = other.points_changed_ || points_changed_;
    *this = other;
    this->transform_changed_ = transform_changed;
    this->rgba_changed_ = rgba_changed;
    this->points_changed_ = points_changed;
}

void Lines::clear() {
    transform_changed_ = false;
    rgba_changed_ = false;
    points_changed_ = false;
}

void Lines::dirty() {
    transform_changed_ = true;
    rgba_changed_ = true;
    points_changed_ = true;
}

void Lines::set_transform(const mat4d& pose) {
    transform_ = pose;
    transform_changed_ = true;
}

void Lines::set_rgba(const vec4f& rgba) {
    rgba_ = rgba;
    rgba_changed_ = true;
}

void Lines::set_points(size_t num_points, const float* points) {
    point_data_.resize(num_points * 3);
    memcpy(point_data_.data(), points, sizeof(float) * 3 * num_points);
    points_changed_ = true;
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

void Label::set_rgba(const vec4f& rgba) {
    rgba_ = rgba;
    rgba_changed_ = true;
}

float Label::get_text_height() const {
    return impl::GLLabel::get_text_height(*this);
}

void TargetDisplay::enable_rings(bool state) { rings_enabled_ = state; }

void TargetDisplay::set_ring_size(int ring_size) { ring_size_ = ring_size; }
void TargetDisplay::set_ring_line_width(int line_width) {
    ring_line_width_ = line_width;
}

void add_default_controls(PointViz& viz, std::mutex* mx) {
    viz.add_default_controls(mx);
}

void PointViz::set_notification(const std::string& msg, double duration_sec) {
    std::lock_guard<std::mutex> guard{pimpl_->update_mx};
    if (!notifications_enabled) {
        return;
    }
    pimpl_->notification_label.set_text(msg);
    pimpl_->notification_label.set_rgba({1.0f, 1.0f, 1.0f, 1.0f});
    pimpl_->notification_label.set_scale(1.5f);
    pimpl_->notification_duration_sec = duration_sec;
    pimpl_->notification_end_time =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int>(1000 * duration_sec));
}

bool PointViz::notification_active() const {
    std::lock_guard<std::mutex> guard{pimpl_->update_mx};
    return notifications_enabled &&
           (pimpl_->notification_end_time >= std::chrono::steady_clock::now());
}

}  // namespace viz
}  // namespace sdk
}  // namespace ouster

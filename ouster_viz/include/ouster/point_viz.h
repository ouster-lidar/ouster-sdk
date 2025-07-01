/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Point cloud and image visualizer for Ouster Lidar using OpenGL
 */
#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/visibility.h"

namespace ouster {
namespace viz {

/**
 * 4x4 matrix of doubles to represent transformations
 */
using mat4d = std::array<double, 16>;

/**
 * 4x4 identity matrix
 */
constexpr mat4d identity4d = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

using vec4f = std::array<float, 4>;
using vec3d = std::array<double, 3>;
using vec2d = std::array<double, 2>;

// TODO: messes up lidar_scan_viz
namespace impl {
struct CameraData;
class Framebuffer;
class GLCloud;
class GLImage;
class GLCuboid;
class GLLabel;
class GLRings;
class GLLines;
}  // namespace impl

/**
 * @brief A mouse button enum, for use in mouse button event handlers
 */
enum class MouseButton : int {
    // Note, this purposefully duplicates GLFW's enum
    // so that we can provide our own symbols without
    // having to include glfw.h here.
    // https://www.glfw.org/docs/latest/group__buttons.html
    MOUSE_BUTTON_1 = 0,
    MOUSE_BUTTON_2 = 1,
    MOUSE_BUTTON_3 = 2,
    MOUSE_BUTTON_4 = 3,
    MOUSE_BUTTON_5 = 4,
    MOUSE_BUTTON_6 = 5,
    MOUSE_BUTTON_7 = 6,
    MOUSE_BUTTON_8 = 7,
    MOUSE_BUTTON_LAST = MOUSE_BUTTON_8,
    MOUSE_BUTTON_LEFT = MOUSE_BUTTON_1,
    MOUSE_BUTTON_RIGHT = MOUSE_BUTTON_2,
    MOUSE_BUTTON_MIDDLE = MOUSE_BUTTON_3,
};

/**
 * @brief A mouse button event enum, for use in mouse button event handlers
 */
enum class MouseButtonEvent : int {
    MOUSE_BUTTON_RELEASED = 0,
    MOUSE_BUTTON_PRESSED = 1,
};

/**
 * @brief An enum for modifier keys, for use in mouse button event handlers
 */
enum class EventModifierKeys : int {
    MOD_NONE = 0,
    MOD_SHIFT = 0x0001,
    MOD_CONTROL = 0x0002,
    MOD_ALT = 0x0004,
    MOD_SUPER = 0x0008,
    MOD_CAPS_LOCK = 0x0010,
    MOD_NUM_LOCK = 0x0020,
};

class Camera;
class Cloud;
class Cuboid;
class Image;
class Label;
class Lines;
class TargetDisplay;
struct WindowCtx;

/**
 * Sets the default_window_width to 800
 */
constexpr int default_window_width = 800;

/**
 * Sets the default_window_height to 600
 */
constexpr int default_window_height = 600;

/**
 * @brief A basic visualizer for sensor data
 *
 * Displays a set of point clouds, images, cuboids, and text labels with a few
 * options for coloring and handling input.
 *
 * All operations are thread safe when running rendering (run() or run_once())
 * in a separate thread. This is the intended way to use the visualizer library
 * when a nontrivial amount of processing needs to run concurrently with
 * rendering (e.g. when streaming data from a running sensor).
 */
class OUSTER_API_CLASS PointViz {
   protected:
    /**
     * Get a reference to the current camera controls
     *
     * @return Handle to the current camera
     */
    Camera& current_camera();

   public:
    /**
     * Add default keyboard and mouse bindings to a visualizer instance
     *
     * Controls will modify the camera from the thread that calls run() or
     * run_once(), which will require synchronization when using multiple
     * threads.
     *
     * @param[in] mx mutex to lock while modifying camera
     */
    OUSTER_API_FUNCTION
    void add_default_controls(std::mutex* mx);

    struct Impl;

    /**
     * Creates a window and initializes the rendering context
     *
     * @param[in] name name of the visualizer, shown in the title bar
     * @param[in] fix_aspect Window aspect to set
     * @param[in] window_width Window width to set,
     *            else uses the default_window_width
     * @param[in] window_height Window height to set,
     *            else uses the default_window_height
     * @param[in] maximized If true, the window will be maximized to fit the
     * screen.
     */
    OUSTER_API_FUNCTION
    explicit PointViz(const std::string& name, bool fix_aspect = false,
                      int window_width = default_window_width,
                      int window_height = default_window_height,
                      bool maximized = false);

    // Because PointViz uses the PIMPL pattern
    // and the Impl owns the window context,
    // we can't realistically copy or move instances of PointViz.
    OUSTER_API_FUNCTION
    PointViz(const PointViz&) = delete;
    OUSTER_API_FUNCTION
    PointViz(PointViz&&) = delete;
    OUSTER_API_FUNCTION
    PointViz& operator=(PointViz&) = delete;
    OUSTER_API_FUNCTION
    PointViz& operator=(PointViz&&) = delete;

    /**
     * Tears down the rendering context and closes the viz window
     */
    OUSTER_API_FUNCTION
    ~PointViz();

    /**
     * Main drawing loop, keeps drawing things until running(false)
     *
     * Should be called from the main thread for macos compatibility
     */
    OUSTER_API_FUNCTION
    void run();

    /**
     * Run one iteration of the main loop for rendering and input handling
     *
     * Should be called from the main thread for macos compatibility
     */
    OUSTER_API_FUNCTION
    void run_once();

    /**
     * Check if the run() has been signaled to exit
     *
     * @return true if the run() loop is currently executing
     */
    OUSTER_API_FUNCTION
    bool running();

    /**
     * Set the running flag. Will signal run() to exit
     *
     * @param[in] state new value of the flag
     */
    OUSTER_API_FUNCTION
    void running(bool state);

    /**
     * Show or hide the visualizer window
     *
     * @param[in] state true to show
     */
    OUSTER_API_FUNCTION
    void visible(bool state);

    /**
     * Update visualization state
     *
     * Send state updates to be rendered on the next frame.
     */
    OUSTER_API_FUNCTION
    void update();

    /**
     * Add a callback for handling keyboard input
     *
     * @param[in] callback the callback. The second argument is the ascii value
     * of the key pressed. Third argument is a bitmask of the modifier keys The
     * callback's return value determines whether the remaining key callbacks
     * should be called.
     */
    OUSTER_API_FUNCTION
    void push_key_handler(
        std::function<bool(const WindowCtx&, int, int)>&& callback);

    /**
     * Add a callback for handling mouse button input
     *
     * @param[in] callback the callback. The callback's arguments are
     *   ctx: the context containing information about the buttons
     *   pressed, the mouse position, and the viewport;
     *   button: the mouse button pressed;
     *   mods: representing which modifier keys are pressed
     *   during the mouse click.
     *   The callback's return value determines whether
     *   the remaining mouse button callbacks should be called.
     */
    OUSTER_API_FUNCTION
    void push_mouse_button_handler(
        std::function<bool(const WindowCtx& ctx,
                           ouster::viz::MouseButton button,
                           ouster::viz::MouseButtonEvent event,
                           ouster::viz::EventModifierKeys mods)>&& callback);

    /**
     * Add a callback for handling mouse scrolling input
     *
     * @param[in] callback the callback. The callback's arguments are
     *   ctx: the context containing information about the buttons
     *   pressed, the mouse position, and the viewport;
     *   x: the amount of scrolling in the x direction;
     *   y: the amount of scrolling in the y direction.
     *   The callback's return value determines whether
     *   the remaining mouse scroll callbacks should be called.
     */
    OUSTER_API_FUNCTION
    void push_scroll_handler(
        std::function<bool(const WindowCtx&, double x, double y)>&& callback);

    /**
     * Add a callback for handling mouse movement
     *
     * @param[in] callback the callback. The callback's arguments are
     *   ctx: the context containing information about the buttons
     *   pressed, the mouse position, and the viewport;
     *   x: the mouse position in the x direction;
     *   y: the mouse position in the y direction.
     *   The callback's return value determines whether
     *   the remaining mouse position callbacks should be called.
     */
    OUSTER_API_FUNCTION
    void push_mouse_pos_handler(
        std::function<bool(const WindowCtx&, double x, double y)>&& callback);

    /**
     * Add a callback for processing every new draw frame buffer.
     *
     * NOTE: Any processing in the callback slows the renderer update loop
     *       dramatically. Primary use to store frame buffer images to disk
     *       for further processing.
     *
     * @param[in] callback function callback of a form f(fb_data, fb_width,
     * fb_height) The callback's return value determines whether the remaining
     * frame buffer callbacks should be called.
     */
    // clang-format off
    [[deprecated(
        "For screenshots or screen recording, use get_screenshot(), "
        "save_screenshot() or toggle_screen_recording(). "
        "push_frame_buffer_handler() is planned to be removed in Q2 2025.")]]
    OUSTER_API_FUNCTION void push_frame_buffer_handler(
    std::function<bool(const std::vector<uint8_t>&, int, int)>&& callback);
    // clang-format on

    /**
     * Remove the last added callback for handling keyboard events
     */
    OUSTER_API_FUNCTION
    void pop_key_handler();

    /**
     * Remove the last added callback for handling mouse button events
     */
    OUSTER_API_FUNCTION
    void pop_mouse_button_handler();

    /**
     * Remove the last added callback for handling mouse scroll events
     */
    OUSTER_API_FUNCTION
    void pop_scroll_handler();

    /**
     * Remove the last added callback for handling mouse position events
     */
    OUSTER_API_FUNCTION
    void pop_mouse_pos_handler();

    /**
     * Remove the last added callback for handling frame buffer events
     */
    // clang-format off
    [[deprecated(
        "For screenshots or screen recording, use get_screenshot(), "
        "save_screenshot() or toggle_screen_recording(). "
        "pop_frame_buffer_handler() is planned to be removed in Q2 2025.")]]
    OUSTER_API_FUNCTION 
    void pop_frame_buffer_handler();
    // clang-format on

    /**
     * Add a callback for handling frame buffer resize events.
     *
     * @param[in] callback function callback of the form f(const WindowCtx&).
     * The callback's return value determines whether the remaining frame buffer
     * resize callbacks should be called.
     */
    OUSTER_API_FUNCTION
    void push_frame_buffer_resize_handler(
        std::function<bool(const WindowCtx&)>&& callback);

    /**
     * Remove the last added callback for handling frame buffer resize events.
     */
    OUSTER_API_FUNCTION
    void pop_frame_buffer_resize_handler();

    /**
     * Get a reference to the camera controls
     *
     * @return Handler to the camera object
     */
    OUSTER_API_FUNCTION
    Camera& camera();

    /**
     * Get a reference to the target display controls
     *
     * @return Handler to the target display controls
     */
    OUSTER_API_FUNCTION
    TargetDisplay& target_display();

    /**
     * Add an object to the scene
     *
     * @param[in] cloud Adds a point cloud to the scene
     */
    OUSTER_API_FUNCTION
    void add(const std::shared_ptr<Cloud>& cloud);

    /**
     * Add an object to the scene
     *
     * @param[in] image Adds an image to the scene
     */
    OUSTER_API_FUNCTION
    void add(const std::shared_ptr<Image>& image);

    /**
     * Add an object to the scene
     *
     * @param[in] cuboid Adds a cuboid to the scene
     */
    OUSTER_API_FUNCTION
    void add(const std::shared_ptr<Cuboid>& cuboid);

    /**
     * Add an object to the scene
     *
     * @param[in] lines Adds a lines to the scene
     */
    OUSTER_API_FUNCTION
    void add(const std::shared_ptr<Lines>& lines);

    /**
     * Add an object to the scene
     *
     * @param[in] label Adds a label to the scene
     */
    OUSTER_API_FUNCTION
    void add(const std::shared_ptr<Label>& label);

    /**
     * Remove an object from the scene
     *
     * @param[in] cloud Remove a point cloud from the scene
     *
     * @return true if successfully removed else false
     */
    OUSTER_API_FUNCTION
    bool remove(const std::shared_ptr<Cloud>& cloud);

    /**
     * Remove an object from the scene
     *
     * @param[in] image Remove an image from the scene
     *
     * @return true if successfully removed else false
     */
    OUSTER_API_FUNCTION
    bool remove(const std::shared_ptr<Image>& image);

    /**
     * Remove an object from the scene
     *
     * @param[in] cuboid Remove a cuboid from the scene
     *
     * @return true if successfully removed else false
     */
    OUSTER_API_FUNCTION
    bool remove(const std::shared_ptr<Cuboid>& cuboid);

    /**
     * Remove an object from the scene
     *
     * @param[in] lines Remove a lines from the scene
     *
     * @return true if successfully removed else false
     */
    OUSTER_API_FUNCTION
    bool remove(const std::shared_ptr<Lines>& lines);

    /**
     * Remove an object from the scene
     *
     * @param[in] label Remove a label from the scene
     *
     * @return true if successfully removed else false
     */
    OUSTER_API_FUNCTION
    bool remove(const std::shared_ptr<Label>& label);

    /**
     * Get a viewport width in pixels.
     *
     * @return viewport width reported by glfw
     */
    OUSTER_API_FUNCTION
    int viewport_width() const;

    /**
     * Get a viewport height in pixels.
     *
     * @return viewport height reported by glfw
     */
    OUSTER_API_FUNCTION
    int viewport_height() const;

    /**
     * Get a window width in screen coordinates.
     *
     * NOTE: this value maybe different from the viewport size on retina
     * displays
     *
     * @return window width reported by glfw
     */
    OUSTER_API_FUNCTION
    int window_width() const;

    /**
     * Get a window height in screen coordinates.
     *
     * @note this value maybe different from the viewport size on retina
     * displays
     *
     * @return window height reported by glfw
     */
    OUSTER_API_FUNCTION
    int window_height() const;

    /**
     * Get frames per second (FPS) value, updated every second
     *
     * Updated every second in the draw() function
     *
     * @return fps value,
     */
    OUSTER_API_FUNCTION
    double fps() const;

    /**
     * @brief Gets a screenshot with the specified size
     * @param[in] width Screenshot width in pixels.
     * @param[in] height Screenshot height in pixels.
     * @return A vector of bytes containing the screenshot's pixel data in RGB
     * format. Each pixel is represented by three consecutive bytes (RGB),
     * stored in row-major order.
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> get_screenshot(uint32_t width, uint32_t height);

    /**
     * @brief Gets a screenshot with the specified size
     * @param[in] scale_factor A multiplier to the current window width and
     * height for determining screenshot size.
     * @return A vector of bytes containing the screenshot's pixel data in RGB
     * format. Each pixel is represented by three consecutive bytes (RGB),
     * stored in row-major order.
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> get_screenshot(double scale_factor = 1.0);

    /**
     * @brief Saves a screenshot with the specified size to the specified path
     * @param[in] path The path where the screenshot should be saved, the
     * filename is auto generated. An empty string means current path.
     * @param[in] width Screenshot width in pixels.
     * @param[in] height Screenshot height in pixels.
     * @return The resulting path and filename.
     */
    OUSTER_API_FUNCTION
    std::string save_screenshot(const std::string& path, uint32_t width,
                                uint32_t height);

    /**
     * @brief Saves a screenshot to the specified path
     * @param[in] path The path where the screenshot should be saved, the
     * filename is auto generated. An empty string means current path.
     * @param[in] scale_factor A multiplier to the window viewport width and
     * height for determining screenshot size.
     * @return the resulting path and filename, empty string if error.
     */
    OUSTER_API_FUNCTION
    std::string save_screenshot(const std::string& path,
                                double scale_factor = 1.0);

    /**
     * @brief Starts or stops saving screenshots continuously
     * @param[in] width screenshot width in pixels.
     * @param[in] height screenshot height in pixels.
     * @return true if recording started, false otherwise,
     */
    OUSTER_API_FUNCTION
    bool toggle_screen_recording(uint32_t width, uint32_t height);

    /**
     * @brief Starts or stops saving screenshots continuously
     * @param[in] scale_factor a multiplier to the window viewport width and
     * height for determining screenshot size, at the time of recording start.
     * @return true if recording started, false otherwise,
     */
    OUSTER_API_FUNCTION
    bool toggle_screen_recording(double scale_factor = 1.0);

    /**
     * @brief Computes the width and height that result from muliplying the
     * current window viewport width and height for a scale_factor.
     * @param[in] scale_factor A multiplier to the current window viewport
     * width and height
     * @return A pair that has width fist and height second
     */
    OUSTER_API_FUNCTION
    std::pair<uint32_t, uint32_t> get_scaled_viewport_size(double scale_factor);

   private:
    std::unique_ptr<Impl> pimpl;

    /**
     * @brief Counts the fps while on the rendering loop.
     */
    void count_fps();

    /**
     * @brief Calls all registered framebuffer handlers if any.
     */
    void call_framebuffer_handlers();

    /**
     * @brief Draws to the currently bound framebuffer.
     */
    void draw();

    /**
     * @brief Does all of the per-frame logic in one call. count_fps, draw to
     * framebuffers, call event handlers, etc.
     */
    void process_frame();

    /**
     * @brief Captures the pixels from the input framebuffer. If the framebuffer
     * size is the same as the window, the pixels are obtained from the default
     * framebuffer.
     * @param[in] framebuffer The framebuffer object from which the pixels will
     * be read.
     */
    std::vector<uint8_t> capture_framebuffer_pixels(
        impl::Framebuffer& framebuffer);

    /***
     * @brief Handles the screenshot requests, if any, in the rendering thread.
     */
    void handle_screenshot_request();

    /***
     * @brief Handles recording in the rendering thread(
     */
    void handle_recording();
};

/**
 * Add default keyboard and mouse bindings to a visualizer instance
 *
 * Controls will modify the camera from the thread that calls run() or
 * run_once(), which will require synchronization when using multiple threads.
 *
 * @param[in] viz the visualizer instance
 * @param[in] mx mutex to lock while modifying camera
 */
OUSTER_API_FUNCTION
void add_default_controls(viz::PointViz& viz, std::mutex* mx = nullptr);

/**
 * @brief Context for input callbacks.
 */
struct OUSTER_API_CLASS WindowCtx {
    bool lbutton_down{false};  ///< True if the left mouse button is held
    bool mbutton_down{false};  ///< True if the middle mouse button is held
    double mouse_x{0};         ///< Current mouse x position
    double mouse_y{0};         ///< Current mouse y position
    int viewport_width{0};     ///< Current viewport width in pixels
    int viewport_height{0};    ///< Current viewport height in pixels
    int window_width{0};       ///< Current window width in screen coordinates
    int window_height{0};      ///< Current window height in screen coordinates

    /**
     * @brief return the aspect ratio of the viewport.
     *
     * @return The aspect ratio of the viewport.
     */
    OUSTER_API_FUNCTION
    double aspect_ratio() const;

    /**
     * @brief return 2d coordinates normalized to (-1, 1) in the y-axis, given
     * viewport coordinates.
     *
     * PointViz renders 2d images and labels in this coordinate system, and as
     * such this method can be used to determine if a pixel in viewport
     * coordinates falls within a 2d image.
     *
     * @param[in] x X axis value of 2D viewport coordinate
     * @param[in] y Y axis value of 2D viewport coordinate
     *
     * @return 2d coordinates normalized to (-1, 1) in the y-axis
     */
    OUSTER_API_FUNCTION
    std::pair<double, double> normalized_coordinates(double x, double y) const;

    /**
     * @brief the inverse of "normalized_coordinates".
     *
     * Given 2d coordinates normalized to (-1, 1) in the y-axis, return viewport
     * coordinates.
     * @param[in] normalized_x X axis value of 2D normalized coordinate
     * @param[in] normalized_y Y axis value of 2D normalized coordinate
     *
     * @return 2d coordinates in viewport pixel space.
     */
    OUSTER_API_FUNCTION
    std::pair<double, double> viewport_coordinates(double normalized_x,
                                                   double normalized_y) const;

    /**
     * @brief raises std::logic_error if this WindowCtx doesn't satisfy class
     * invariants.
     */
    OUSTER_API_FUNCTION
    void check_invariants() const;
};

/**
 * @brief Controls the camera view and projection.
 */
class OUSTER_API_CLASS Camera {
    /* view parameters */
    mat4d target_;
    vec3d view_offset_;
    int roll_;
    int pitch_;
    int yaw_;              // decidegrees
    double log_distance_;  // 0 means 50m

    /* projection parameters */
    bool orthographic_;
    int fov_;
    double proj_offset_x_, proj_offset_y_;

    double view_distance() const;

   public:
    /**
     * @todo document me
     */
    OUSTER_API_FUNCTION
    Camera();

    /**
     * Calculate camera matrices.
     *
     * @param[in] aspect aspect ratio of the viewport
     *
     * @return projection, view matrices and target location.
     */
    OUSTER_API_FUNCTION
    impl::CameraData matrices(double aspect) const;

    /**
     * Reset the camera view and fov.
     */
    OUSTER_API_FUNCTION
    void reset();

    /**
     * Set camera view as looking from the top as a bird (Birds Eye View).
     */
    OUSTER_API_FUNCTION
    void birds_eye_view();

    /**
     * Roll the camera with respect to its current roll angle.
     *
     * @param[in] degrees offset to the current roll angle
     */
    OUSTER_API_FUNCTION
    void roll(float degrees);

    /**
     * Set roll in degrees.
     *
     * @param[in] degrees roll angle
     */
    OUSTER_API_FUNCTION
    void set_roll(float degrees);

    /**
     * Get the roll in degrees.
     *
     * @return roll in degrees
     */
    OUSTER_API_FUNCTION
    float get_roll() const;

    /**
     * Orbit the camera left or right about the camera target.
     *
     * @param[in] degrees offset to the current yaw angle
     */
    OUSTER_API_FUNCTION
    void yaw(float degrees);

    /**
     * Set yaw in degrees.
     *
     * @param[in] degrees yaw angle
     */
    OUSTER_API_FUNCTION
    void set_yaw(float degrees);

    /**
     * Get the yaw in degrees.
     *
     * @return yaw in degrees
     */
    OUSTER_API_FUNCTION
    float get_yaw() const;

    /**
     * Pitch the camera up or down.
     *
     * @param[in] degrees offset to the current pitch angle
     */
    OUSTER_API_FUNCTION
    void pitch(float degrees);

    /**
     * Set pitch in degrees.
     *
     * @param[in] degrees pitch angle
     */
    OUSTER_API_FUNCTION
    void set_pitch(float degrees);

    /**
     * Get the camera pitch in degrees.
     *
     * @return pitch in degrees
     */
    OUSTER_API_FUNCTION
    float get_pitch() const;

    /**
     * Move the camera towards or away from the target.
     *
     * @param[in] amount offset to the current camera distance from the target
     */
    OUSTER_API_FUNCTION
    void dolly(double amount);

    /**
     * Set dolly (i.e. log distance) from the target to the camera.
     *
     * @param[in] log_distance log of the distance from the target
     */
    OUSTER_API_FUNCTION
    void set_dolly(double log_distance);

    /**
     * Get the log distance from the target to the camera.
     *
     * @return log_distance
     */
    OUSTER_API_FUNCTION
    double get_dolly() const;

    /**
     * Move the camera in the XY plane of the camera view.
     *
     * Coordinates are normalized so that 1 is the length of the diagonal of the
     * view plane at the target. This is useful for implementing controls that
     * work intuitively regardless of the camera distance.
     *
     * @param[in] x horizontal offset
     * @param[in] y vertical offset
     */
    OUSTER_API_FUNCTION
    void dolly_xy(double x, double y);

    /**
     * Set view offset.
     *
     * @param[in] view_offset view offset of the camera
     */
    OUSTER_API_FUNCTION
    void set_view_offset(const vec3d& view_offset);

    /**
     * Get view offset.
     *
     * @return view offset of the camera
     */
    OUSTER_API_FUNCTION
    vec3d get_view_offset() const;

    /**
     * Set the diagonal field of view.
     *
     * @param[in] degrees the diagonal field of view, in degrees
     */
    OUSTER_API_FUNCTION
    void set_fov(float degrees);

    /**
     * Get field of fiew of a camera in degrees
     *
     * @return fov in degrees
     */
    OUSTER_API_FUNCTION
    float get_fov() const;

    /**
     * Use an orthographic or perspective projection.
     *
     * @param[in] state true for orthographic, false for perspective
     */
    OUSTER_API_FUNCTION
    void set_orthographic(bool state);

    /**
     * Get orthographic state.
     *
     * @return true if orthographic, false if perspective
     */
    OUSTER_API_FUNCTION
    bool is_orthographic() const;

    /**
     * Set the 2d position of camera target in the viewport.
     *
     * @param[in] x horizontal position in in normalized coordinates [-1, 1]
     * @param[in] y vertical position in in normalized coordinates [-1, 1]
     */
    OUSTER_API_FUNCTION
    void set_proj_offset(float x, float y);

    /**
     * Get the 2d position of camera target in the viewport.
     *
     * @return (x, y) position of a camera target in the viewport
     */
    OUSTER_API_FUNCTION
    vec2d get_proj_offset() const;

    /**
     * Directly set camera target object pose
     *
     * @param[in] target target where camera is looking at
     */
    OUSTER_API_FUNCTION
    void set_target(const mat4d& target);

    /**
     * Get the pose of a camera target.
     *
     * @return target camera pose
     */
    OUSTER_API_FUNCTION
    mat4d get_target() const;
};

/**
 * @brief Manages the state of the camera target display.
 */
class OUSTER_API_CLASS TargetDisplay {
    int ring_size_{1};
    bool rings_enabled_{false};
    int ring_line_width_{1};

   public:
    /**
     * Enable or disable distance ring display.
     *
     * @param[in] state true to display rings
     */
    OUSTER_API_FUNCTION
    void enable_rings(bool state);

    /**
     * Set the distance between rings.
     *
     * @param[in] n space between rings will be 10^n meters
     */
    OUSTER_API_FUNCTION
    void set_ring_size(int n);

    /**
     * Set the line width of the rings.
     *
     * @param[in] line_width of the rings line
     */
    OUSTER_API_FUNCTION
    void set_ring_line_width(int line_width);

    friend class impl::GLRings;
};

/**
 * @brief Manages the state of a point cloud.
 *
 * Each point cloud consists of n points with w poses. The ith point will be
 * transformed by the (i % w)th pose. For example for 2048 x 64 Ouster lidar
 * point cloud, we may have w = 2048 poses and n = 2048 * 64 = 131072 points.
 *
 * We also keep track of a per-cloud pose to efficiently transform the
 * whole point cloud without having to update all ~2048 poses.
 */
class OUSTER_API_CLASS Cloud {
   protected:
    size_t n_{0};
    size_t w_{0};
    mat4d extrinsic_{};

    // set everything to changed so on GLCloud object reuse we properly draw
    // everything first time
    bool range_changed_{true};
    bool key_changed_{true};
    bool mask_changed_{true};
    bool xyz_changed_{true};
    bool offset_changed_{true};
    bool transform_changed_{true};
    bool palette_changed_{true};
    bool pose_changed_{true};
    bool point_size_changed_{true};

    std::shared_ptr<std::vector<float>> range_data_{};
    std::shared_ptr<std::vector<float>> key_data_{};
    std::shared_ptr<std::vector<float>> mask_data_{};
    std::shared_ptr<std::vector<float>> xyz_data_{};
    std::shared_ptr<std::vector<float>> off_data_{};
    std::shared_ptr<std::vector<float>> transform_data_{};
    std::shared_ptr<std::vector<float>> palette_data_{};
    mat4d pose_{};
    float point_size_{2};
    bool mono_{true};

    Cloud(size_t w, size_t h, const mat4d& extrinsic);

   public:
    /**
     * Unstructured point cloud for visualization.
     *
     * Call set_xyz() to update
     *
     * @param[in] n number of points
     * @param[in] extrinsic sensor extrinsic calibration. 4x4 column-major
     *        homogeneous transformation matrix
     */
    OUSTER_API_FUNCTION
    explicit Cloud(size_t n, const mat4d& extrinsic = identity4d);

    /**
     * Structured point cloud for visualization.
     *
     * Call set_range() to update
     *
     * @param[in] w number of columns
     * @param[in] h number of pixels per column
     * @param[in] dir unit vectors for projection
     * @param[in] off offsets for xyz projection
     * @param[in] extrinsic sensor extrinsic calibration. 4x4 column-major
     *        homogeneous transformation matrix
     */
    OUSTER_API_FUNCTION
    Cloud(size_t w, size_t h, const float* dir, const float* off,
          const mat4d& extrinsic = identity4d);

    /**
     * Updates this cloud's state with the state of other,
     * accounting for prior changes to this objects's state.
     *
     * @param[in] other the object to update the state from.
     */
    OUSTER_API_FUNCTION
    void update_from(const Cloud& other);

    /**
     * Clear dirty flags.
     *
     * Resets any changes since the last call to PointViz::update()
     */
    OUSTER_API_FUNCTION
    void clear();

    /**
     * Set all dirty flags.
     *
     * Re-sets everything so the object is always redrawn.
     */
    OUSTER_API_FUNCTION
    void dirty();

    /**
     * Get the size of the point cloud.
     *
     * @return @todo document me
     */
    OUSTER_API_FUNCTION
    size_t get_size() const { return n_; }

    /**
     * Get the columns of the point cloud.
     *
     * @return number of columns in point cloud. (1 - for unstructured)
     */
    OUSTER_API_FUNCTION
    size_t get_cols() const { return w_; }

    /**
     * Set the range values.
     *
     * @param[in] range pointer to array of at least as many elements as there
     * are points, representing the range of the points
     */
    OUSTER_API_FUNCTION
    void set_range(const uint32_t* range);

    /**
     * Set the key values, used for coloring.
     *
     * @param[in] key pointer to array of at least as many elements as there are
     *                points, preferably normalized between 0 and 1
     */
    OUSTER_API_FUNCTION
    void set_key(const float* key);

    /**
     * Set the key alpha values, leaving the color the same.
     *
     * @param[in] key_alpha pointer to array of at least as many elements as
     *                      there are points, normalized between 0 and 1
     */
    OUSTER_API_FUNCTION
    void set_key_alpha(const float* key_alpha);

    /**
     * Set the key values in RGB format, used for coloring.
     *
     * @param[in] key_rgb pointer to array of at least 3x as many elements as
     * there are points, normalized between 0 and 1
     */
    OUSTER_API_FUNCTION
    void set_key_rgb(const float* key_rgb);

    /**
     * Set the key values in RGBA format, used for coloring.
     *
     * @param[in] key_rgba pointer to array of at least 4x as many elements as
     * there are points, normalized between 0 and 1
     */
    OUSTER_API_FUNCTION
    void set_key_rgba(const float* key_rgba);

    /**
     * Set the RGBA mask values, used as an overlay on top of the key.
     *
     * @param[in] mask pointer to array of at least 4x as many elements as there
     * are points, preferably normalized between 0 and 1
     */
    OUSTER_API_FUNCTION
    void set_mask(const float* mask);

    /**
     * Set the XYZ values.
     *
     * @param[in] xyz pointer to array of exactly 3n where n is number of
     * points, so that the xyz position of the ith point is i, i + n, i + 2n
     */
    OUSTER_API_FUNCTION
    void set_xyz(const float* xyz);

    /**
     * Set the offset values.
     *
     * TODO: no real reason to have this. Set in constructor, if at all
     *
     * @param[in] offset pointer to array of exactly 3n where n is number of
     * points, so that the xyz position of the ith point is i, i + n, i + 2n
     */
    OUSTER_API_FUNCTION
    void set_offset(const float* offset);

    /**
     * Set the ith point cloud pose.
     *
     * @param[in] pose 4x4 column-major homogeneous transformation matrix
     */
    OUSTER_API_FUNCTION
    void set_pose(const mat4d& pose);

    /**
     * Set point size.
     *
     * @param[in] size point size
     */
    OUSTER_API_FUNCTION
    void set_point_size(float size);

    /**
     * Set the per-column poses, so that the point corresponding to the
     * pixel at row u, column v in the staggered lidar scan is transformed
     * by the vth pose, given as a homogeneous transformation matrix.
     *
     * @param[in] rotation array of rotation matrices, total size 9 * w, where
     * the vth rotation matrix is: r[v], r[w + v], r[2 * w + v], r[3 * w + v],
     * r[4 * w + v], r[5 * w + v], r[6 * w + v], r[7 * w + v], r[8 * w + v]
     * @param[in] translation translation vector array, column major, where each
     * row is a translation vector. That is, the vth translation is t[v], t[w +
     * v], t[2 * w + v]
     */
    OUSTER_API_FUNCTION
    void set_column_poses(const float* rotation, const float* translation);

    /**
     * Set the per-column poses, so that the point corresponding to the
     * pixel at row u, column v in the staggered lidar scan is transformed
     * by the vth pose, given as a homogeneous transformation matrix.
     *
     * @param[in] column_poses array of 4x4 pose elements and length w
     * (i.e. [wx4x4]) column-storage
     */
    OUSTER_API_FUNCTION
    void set_column_poses(const float* column_poses);

    /**
     * Set the point cloud color palette.
     *
     * @param[in] palette the new palette to use, must have size 3*palette_size
     * @param[in] palette_size the number of colors in the new palette
     */
    OUSTER_API_FUNCTION
    void set_palette(const float* palette, size_t palette_size);

    friend class impl::GLCloud;
};

/**
 * @brief Manages the state of an image.
 */
class OUSTER_API_CLASS Image {
    bool position_changed_{false};
    bool image_changed_{false};
    bool mask_changed_{false};
    bool palette_changed_{false};

    vec4f position_{};
    size_t image_width_{0};
    size_t image_height_{0};
    std::vector<float> image_data_{};
    size_t mask_width_{0};
    size_t mask_height_{0};
    std::vector<float> mask_data_{};
    std::vector<float> palette_data_{};
    float hshift_{0};  // in normalized screen coordinates [-1. 1]
    bool mono_{true};
    bool use_palette_{false};

   public:
    /**
     * @todo document me
     */
    OUSTER_API_FUNCTION
    Image();

    /**
     * Updates this image's state with the state of other,
     * accounting for prior changes to this objects's state.
     *
     * @param[in] other the object to update the state from.
     */
    OUSTER_API_FUNCTION
    void update_from(const Image& other);

    /**
     * Clear dirty flags.
     *
     * Resets any changes since the last call to PointViz::update()
     */
    OUSTER_API_FUNCTION
    void clear();

    /**
     * Set the image data.
     *
     * @param[in] width width of the image data in pixels
     * @param[in] height height of the image data in pixels
     * @param[in] image_data pointer to an array of width * height elements
     *        interpreted as a row-major monochrome image
     */
    OUSTER_API_FUNCTION
    void set_image(size_t width, size_t height, const float* image_data);

    /**
     * Set the image data (RGB).
     *
     * @param[in] width width of the image data in pixels
     * @param[in] height height of the image data in pixels
     * @param[in] image_data_rgb pointer to an array of width * height elements
     *        interpreted as a row-major RGB image
     */
    OUSTER_API_FUNCTION
    void set_image_rgb(size_t width, size_t height,
                       const float* image_data_rgb);

    /**
     * Set the image data (RGBA).
     *
     * @param[in] width width of the image data in pixels
     * @param[in] height height of the image data in pixels
     * @param[in] image_data_rgba pointer to an array of width * height elements
     *        interpreted as a row-major RGBA image
     */
    OUSTER_API_FUNCTION
    void set_image_rgba(size_t width, size_t height,
                        const float* image_data_rgba);

    /**
     * Set the RGBA mask.
     *
     * Not required to be the same resolution as the image data
     *
     * @param[in] width width of the image data in pixels
     * @param[in] height height of the image data in pixels
     * @param[in] mask_data pointer to array of 4 * width * height elements
     * interpreted as a row-major rgba image
     */
    OUSTER_API_FUNCTION
    void set_mask(size_t width, size_t height, const float* mask_data);

    /**
     * Set the display position of the image.
     *
     * @param[in] x_min
     * @param[in] x_max
     * @param[in] y_min
     * @param[in] y_max
     *
     * @todo:  this is super weird.
     * Coordinates are {x_min, x_max, y_max, y_min}
     * in sort-of normalized screen coordinates:
     * y is in [-1, 1], and x uses the same scale
     * (i.e. window width is ignored).
     * This is currently just the same method the previous
     * hard-coded 'image_frac' logic was using;
     * I believe it was done this way to allow scaling
     * with the window while maintaining the aspect ratio.
     */
    OUSTER_API_FUNCTION
    void set_position(float x_min, float x_max, float y_min, float y_max);

    /**
     * Set horizontal shift in normalized viewport screen width coordinate.
     *
     * This may be used to "snap" images to the left/right screen edges.
     *
     * Some example values:
     *    0 - default, image is centered horizontally on the screen
     * -0.5 - image moved to the left for the 1/4 of a horizontal viewport
     *   -1 - image moved to the left for the 1/2 of a horizontal viewport
     *   +1 - image moved to the right for the 1/2 of a horizontal viewport
     * +0.5 - image moved to the right for the 1/4 of a horizontal viewport
     *
     * @param[in] hshift shift in normalized by width coordinates from 0 at
     * the center [-1.0..1.0]
     *
     */
    OUSTER_API_FUNCTION
    void set_hshift(float hshift);

    /**
     * Set the image color palette.
     *
     * @param[in] palette the new palette to use, must have size 3*palette_size
     * @param[in] palette_size the number of colors in the new palette
     */
    OUSTER_API_FUNCTION
    void set_palette(const float* palette, size_t palette_size);

    /**
     * Removes the image color palette.
     *
     */
    OUSTER_API_FUNCTION
    void clear_palette();

    /**
     * Returns the image pixel corresponding to the provided viewport
     * coordinates.
     *
     * @param[in] ctx the WindowCtx.
     * @param[in] x X coordinate for the viewport
     * @param[in] y Y coordinate for the viewport
     *
     * @throw std::runtime_error if the image data has not been set,
     * or if the image size (set by set_position) is zero in either dimension.
     *
     * @return Image pixel corresponding to the provided viewport coordinates.
     * IMPORTANT: the pixel is outside the image if either the (x, y) are less
     * than zero or greater than the image (width, height).
     */
    OUSTER_API_FUNCTION
    std::pair<int, int> viewport_coordinates_to_image_pixel(
        const WindowCtx& ctx, double x, double y) const;

    /**
     * @brief Returns the inverse of "viewport_coordinates_to_image_pixel"
     * This is useful for computing positions relative to an image pixel.
     *
     * @param[in] ctx the WindowCtx.
     * @param[in] px X pixel coordinate
     * @param[in] py Y pixel coordinate
     *
     * @return a coordinate normalized to -1, 1 in the window's Y axis
     */
    OUSTER_API_FUNCTION
    std::pair<double, double> image_pixel_to_viewport_coordinates(
        const WindowCtx& ctx, int px, int py) const;

    /**
     * @brief Returns the pixel size as a pair representing width and height in
     * window pixels.
     *
     * @param[in] ctx the WindowCtx.
     *
     * @return a pair representing the image pixel size in window pixels.
     */
    OUSTER_API_FUNCTION
    std::pair<double, double> pixel_size(const WindowCtx& ctx) const;

    friend class impl::GLImage;
};

/**
 * @brief Manages the state of a single cuboid.
 */
class OUSTER_API_CLASS Cuboid {
    bool transform_changed_{false};
    bool rgba_changed_{false};

    mat4d transform_{};
    vec4f rgba_{};

   public:
    /**
     * Constructor to initialize a cuboid
     *
     * @param[in] transform 4x4 matrix representing a transform
     *            defining the cuboid
     * @param[in] rgba 4x1 float matrix representing cuboid color in RGBA format
     *
     */
    OUSTER_API_FUNCTION
    Cuboid(const mat4d& transform, const vec4f& rgba);

    /**
     * Updates this cuboid's state with the state of other,
     * accounting for prior changes to this objects's state.
     *
     * @param[in] other the object to update the state from.
     */
    OUSTER_API_FUNCTION
    void update_from(const Cuboid& other);

    /**
     * Clear dirty flags.
     *
     * Resets any changes since the last call to PointViz::update()
     */
    OUSTER_API_FUNCTION
    void clear();

    /**
     * Set the transform defining the cuboid.
     *
     * Applied to a unit cube centered at the origin.
     *
     * @param[in] pose @todo document me
     */
    OUSTER_API_FUNCTION
    void set_transform(const mat4d& pose);

    /**
     * Set the color of the cuboid.
     *
     * @param[in] rgba @todo document me
     */
    OUSTER_API_FUNCTION
    void set_rgba(const vec4f& rgba);

    friend class impl::GLCuboid;
};

/**
 * @brief Manages the state of a single set of lines.
 */
class OUSTER_API_CLASS Lines {
    bool transform_changed_{false};
    bool rgba_changed_{false};
    bool points_changed_{false};

    mat4d transform_{};
    vec4f rgba_{};
    std::vector<float> point_data_;

   public:
    /**
     * Constructor to initialize a set of lines
     *
     * @param[in] transform 4x4 matrix representing a transform for the points
     * @param[in] rgba 4x1 float matrix representing lines color in RGBA format
     *
     */
    OUSTER_API_FUNCTION
    Lines(const mat4d& transform, const vec4f& rgba);

    /**
     * Updates this objects's state with the state of other,
     * accounting for prior changes to this objects's state.
     *
     * @param[in] other the object to update the state from.
     */
    OUSTER_API_FUNCTION
    void update_from(const Lines& other);

    /**
     * Clear dirty flags.
     *
     * Resets any changes since the last call to PointViz::update()
     */
    OUSTER_API_FUNCTION
    void clear();

    /**
     * Set the 3D points defining the lines. Each line is defined by 2 points, a
     * start and an end.
     *
     * @param[in] num_points number of points
     * @param[in] points num_points*3 array of doubles representing the xyz of
     *                   each point
     */
    OUSTER_API_FUNCTION
    void set_points(size_t num_points, const float* points);

    /**
     * Set the transform for the lines.
     *
     * @param[in] pose new pose for the lines
     */
    OUSTER_API_FUNCTION
    void set_transform(const mat4d& pose);

    /**
     * Set the color of the lines.
     *
     * @param[in] rgba color
     */
    OUSTER_API_FUNCTION
    void set_rgba(const vec4f& rgba);

    friend class impl::GLLines;
};

/**
 * @brief Manages the state of a text label.
 */
class OUSTER_API_CLASS Label {
    bool pos_changed_{false};
    bool scale_changed_{true};
    bool text_changed_{false};
    bool is_3d_{false};
    bool align_right_{false};
    bool align_top_{false};
    bool rgba_changed_{true};

    vec3d position_{};
    float scale_{1.0};
    std::string text_{};

    vec4f rgba_{1.0, 1.0, 1.0, 1.0};

   public:
    /**
     * Constructs a Label object
     *
     * @param[in] text Text to display
     * @param[in] position Set 3D position of label as x,y,z coordinates
     *            of type double
     */
    OUSTER_API_FUNCTION
    Label(const std::string& text, const vec3d& position);
    /**
     * Sets the 2D position of the lavel
     *
     * @param[in] text Label text to display
     * @param[in] x horizontal position [0, 1]
     * @param[in] y vertical position [0, 1]
     * @param[in] align_right interpret position as right of the label
     * @param[in] align_top interpret position as top of the label
     */
    OUSTER_API_FUNCTION
    Label(const std::string& text, float x, float y, bool align_right = false,
          bool align_top = false);

    /**
     * Updates this label's state with the state of other,
     * accounting for prior changes to this objects's state.
     *
     * @param[in] other the object to update the state from.
     */
    OUSTER_API_FUNCTION
    void update_from(const Label& other);

    /**
     * Clear dirty flags.
     *
     * Resets any changes since the last call to PointViz::update()
     */
    OUSTER_API_FUNCTION
    void clear();

    /**
     * Set all dirty flags.
     *
     * Re-sets everything so the object is always redrawn.
     */
    OUSTER_API_FUNCTION
    void dirty();

    /**
     * Update label text.
     *
     * @param[in] text new text to display
     */
    OUSTER_API_FUNCTION
    void set_text(const std::string& text);

    /**
     * Set label position.
     *
     * @param[in] position 3d position of the label
     */
    OUSTER_API_FUNCTION
    void set_position(const vec3d& position);

    /**
     * Set position of the 2D label.
     *
     * @param[in] x horizontal position [0, 1]
     * @param[in] y vertical position [0, 1]
     * @param[in] align_right interpret position as right of the label
     * @param[in] align_top interpret position as top of the label
     */
    OUSTER_API_FUNCTION
    void set_position(float x, float y, bool align_right = false,
                      bool align_top = false);

    /**
     * Set scaling factor of the label.
     *
     * @param[in] scale text scaling factor
     */
    OUSTER_API_FUNCTION
    void set_scale(float scale);

    /**
     * Set the color of the label.
     *
     * @param[in] rgba color in RGBA format
     */
    OUSTER_API_FUNCTION
    void set_rgba(const vec4f& rgba);

    friend class impl::GLLabel;
};
/**
 * Spezia palette size in number of colors.
 */
extern const size_t spezia_n;
/**
 * Spezia palette, RGB values per element.
 */
extern const float spezia_palette[][3];

/**
 * Spezia Cal Ref palette size in number of colors.
 */
extern const size_t spezia_cal_ref_n;
/**
 * Spezia Cal Ref palette, RGB values per element.
 */
extern const float spezia_cal_ref_palette[][3];

/**
 * Calibrated reflectifiy palette size in number of colors.
 */
extern const size_t calref_n;
/**
 * Calibrated reflectifiy, RGB values per element.
 */
extern const float calref_palette[][3];

/**
 * Greyscale palette size in number of colors.
 */
extern const size_t grey_n;
/**
 * Greyscale palette, RGB values per element.
 */
extern const float grey_palette[][3];

/**
 * Greyscale Cal Ref palette size in number of colors.
 */
extern const size_t grey_cal_ref_n;
/**
 * Greyscale Cal Ref palette, RGB values per element.
 */
extern const float grey_cal_ref_palette[][3];

/**
 * Viridis palette size in number of colors.
 */
extern const size_t viridis_n;
/**
 * Viridis palette, RGB values per element.
 */
extern const float viridis_palette[][3];

/**
 * Viridis Cal Ref palette size in number of colors.
 */
extern const size_t viridis_cal_ref_n;
/**
 * Viridis Cal Ref palette, RGB values per element.
 */
extern const float viridis_cal_ref_palette[][3];

/**
 * Magma palette size in number of colors.
 */
extern const size_t magma_n;
/**
 * Magma palette, RGB values per element.
 */
extern const float magma_palette[][3];

/**
 * Magma Cal Ref palette size in number of colors.
 */
extern const size_t magma_cal_ref_n;
/**
 * Magma Cal Ref palette, RGB values per element.
 */
extern const float magma_cal_ref_palette[][3];

}  // namespace viz
}  // namespace ouster

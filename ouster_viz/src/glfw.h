/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

// clang-format off
// GLAD must be included before GLFW
#include "glad.h"
#include <GLFW/glfw3.h>
// clang-format on

#include <functional>
#include <string>

#include "ouster/viz/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {

struct GLFWContext {
    explicit GLFWContext(const std::string& name, bool fix_aspect, int window_width,
                         int window_height, bool maximized, bool fullscreen, bool borderless);

    // manages glfw window pointer lifetime
    GLFWContext(const GLFWContext&) = delete;
    GLFWContext& operator=(const GLFWContext&) = delete;

    // pointer used for glfw callback context; can't move
    GLFWContext(GLFWContext&&) = delete;
    GLFWContext& operator=(GLFWContext&&) = delete;

    ~GLFWContext();

    // tear down global glfw context
    static void terminate();

    // manipulate glfwWindowShouldClose flag
    bool running() const;
    // Mutates GLFW window state; not logically const despite no member writes.
    // NOLINTNEXTLINE(readability-make-member-function-const)
    void running(bool);

    // Mutates GLFW window state; not logically const despite no member writes.
    // NOLINTNEXTLINE(readability-make-member-function-const)
    void visible(bool);

    static bool is_opengl_es();

    static float ui_scale();

    GLFWwindow* window;

    // state set by GLFW callbacks
    WindowCtx window_context;

    bool emulated_mbutton_down = false;

    std::function<void(const WindowCtx&, int, int)> key_handler;
    std::function<void(const WindowCtx&, int, int, int)> mouse_button_handler;
    std::function<void(const WindowCtx&, double, double)> scroll_handler;
    std::function<void(const WindowCtx&, double, double)> mouse_pos_handler;
    std::function<void(const WindowCtx&)> resize_handler;
};

}  // namespace viz
}  // namespace sdk
}  // namespace ouster

/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#ifdef OUSTER_VIZ_GLEW
#include <GL/glew.h>
#else
#include <glad/glad.h>
#endif

#include <GLFW/glfw3.h>

#include <functional>
#include <string>

#include "ouster/point_viz.h"

namespace ouster {
namespace viz {

struct GLFWContext {
    explicit GLFWContext(const std::string& name, bool fix_aspect,
                         int window_width, int window_height);

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
    bool running();
    void running(bool);

    void visible(bool);

    GLFWwindow* window;

    // state set by GLFW callbacks
    WindowCtx window_context;

    std::function<void(const WindowCtx&, int, int)> key_handler;
    std::function<void(const WindowCtx&, int, int)> mouse_button_handler;
    std::function<void(const WindowCtx&, double, double)> scroll_handler;
    std::function<void(const WindowCtx&, double, double)> mouse_pos_handler;

    std::function<void()> resize_handler;
};

}  // namespace viz
}  // namespace ouster

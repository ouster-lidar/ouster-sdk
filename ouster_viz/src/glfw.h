#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <functional>
#include <string>

#include "ouster/point_viz.h"

namespace ouster {
namespace viz {

struct GLFWContext {
    explicit GLFWContext(const std::string& name);

    // manages glfw window pointer lifetime
    GLFWContext(const GLFWContext&) = delete;
    GLFWContext& operator=(const GLFWContext&) = delete;

    // pointer used for glfw callback context; can't move
    GLFWContext(GLFWContext&&) = delete;
    GLFWContext& operator=(GLFWContext&&) = delete;

    ~GLFWContext();

    void quit();
    bool running();

    GLFWwindow* window;

    // state set by GLFW callbacks
    PointViz::HandlerCtx window_context;

    using HandlerCtx = PointViz::HandlerCtx;
    std::function<void(const HandlerCtx&, int, int)> key_handler;
    std::function<void(const HandlerCtx&, int, int)> mouse_button_handler;
    std::function<void(const HandlerCtx&, double, double)> scroll_handler;
    std::function<void(const HandlerCtx&, double, double)> mouse_pos_handler;

    std::function<void()> resize_handler;
};

}  // namespace viz
}  // namespace ouster

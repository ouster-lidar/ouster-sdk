/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include "glfw.h"

#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "gltext.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace viz {

namespace {

/*
 * Callback for glfw errors
 */
void error_callback(int error, const char* description) {
    std::cerr << "GLFW error " << error << ": " << description << std::endl;
}

/*
 * Callback for keypress, runs during glfwPollEvents
 */
void handle_key_press(GLFWwindow* window, int key, int /*scancode*/, int action,
                      int mods) {
    auto ctx = static_cast<GLFWContext*>(glfwGetWindowUserPointer(window));
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        if (ctx->key_handler) ctx->key_handler(ctx->window_context, key, mods);
    }
}

/*
 * Callback for resizing the window
 */
void handle_window_resize(GLFWwindow* window, int fb_width, int fb_height) {
    auto ctx = static_cast<GLFWContext*>(glfwGetWindowUserPointer(window));
    ctx->window_context.viewport_width = fb_width;
    ctx->window_context.viewport_height = fb_height;
    glViewport(0, 0, fb_width, fb_height);
    gltViewport(fb_width, fb_height);
    if (ctx->resize_handler) ctx->resize_handler();
}

/*
 * Callback for mouse press. Polled after each drawing.
 *
 * Keeps track of whether mouse is held down with lbutton_down member
 * variable
 *
 * shift + left click is the same as middle click to support people with
 * very few mouse buttons
 */
void handle_mouse_button(GLFWwindow* window, int button, int action, int mods) {
    auto ctx = static_cast<GLFWContext*>(glfwGetWindowUserPointer(window));
    if (action == GLFW_PRESS) {
        ctx->window_context.lbutton_down =
            (button == GLFW_MOUSE_BUTTON_LEFT && mods == 0);
        ctx->window_context.mbutton_down =
            (button == GLFW_MOUSE_BUTTON_MIDDLE ||
             (button == GLFW_MOUSE_BUTTON_LEFT && mods == GLFW_MOD_SHIFT));

        // TODO right order wrt updating context?
        // run custom button handlers from users
        if (ctx->mouse_button_handler)
            ctx->mouse_button_handler(ctx->window_context, button, mods);
    } else if (action == GLFW_RELEASE) {
        ctx->window_context.lbutton_down = false;
        ctx->window_context.mbutton_down = false;
    }
}

/*
 * Callback for cursor movement
 *
 * If mouse is held down, this is used for dragging, and updates camera.
 */
void handle_cursor_pos(GLFWwindow* window, double xpos, double ypos) {
    auto ctx = static_cast<GLFWContext*>(glfwGetWindowUserPointer(window));

    // run custom button handlers from users
    if (ctx->mouse_pos_handler)
        ctx->mouse_pos_handler(ctx->window_context, xpos, ypos);

    // update context position only after passing new values to handlers
    ctx->window_context.mouse_x = xpos;
    ctx->window_context.mouse_y = ypos;
}

/*
 * Callback for mouse scroll
 *
 * Used for dollying the camera
 */
void handle_scroll(GLFWwindow* window, double xoff, double yoff) {
    auto ctx = static_cast<GLFWContext*>(glfwGetWindowUserPointer(window));
    if (ctx->scroll_handler)
        ctx->scroll_handler(ctx->window_context, xoff, yoff);
}

/*
 * Callback for mouse entering or leaving the window
 *
 * Used to avoid the "sticky" situation where one clicks and drags but
 * releases the mouse button outside of the window, and then the next time
 * when the mouse re-enters the window, it still thinks the mouse is pressed
 * causing the camera to freak out when the user moves the mouse around.
 */
void handle_cursor_enter(GLFWwindow* window, int entered) {
    auto ctx = static_cast<GLFWContext*>(glfwGetWindowUserPointer(window));
    if (!entered) {
        ctx->window_context.lbutton_down = false;
        ctx->window_context.mbutton_down = false;
    }
}

}  // namespace

/*
 * Initialize GLFW window
 */
GLFWContext::GLFWContext(const std::string& name, bool fix_aspect,
                         int window_width, int window_height) {
    glfwSetErrorCallback(error_callback);

    // avoid chdir to resources dir on macos
#ifdef __APPLE__
    glfwInitHint(GLFW_COCOA_CHDIR_RESOURCES, false);
#endif
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }
#ifdef __APPLE__
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, true);
#endif
    glfwWindowHint(GLFW_SAMPLES, GLFW_DONT_CARE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, 1);
    glfwWindowHint(GLFW_VISIBLE, false);

    // open a window and create its OpenGL context
    window =
        glfwCreateWindow(window_width, window_height, name.c_str(), NULL, NULL);

    if (window == nullptr) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(window);

#ifdef OUSTER_VIZ_GLEW
    if (glewInit() != GLEW_OK) {
        glfwTerminate();
        throw std::runtime_error("Failed to initialize GLEW");
    }
#else
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwTerminate();
        throw std::runtime_error("Failed to initialize GLAD");
    }
#endif

    std::cerr << "GL Renderer: " << glGetString(GL_RENDERER) << std::endl;
    std::cerr << "GL Version: " << glGetString(GL_VERSION)
              << " (GLSL: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << ")"
              << std::endl;

    // initialize text rendering
    if (gltInit() == GL_FALSE) {
        std::cerr << "Error initializing GLT" << std::endl;
        glfwTerminate();
        throw std::runtime_error("Error initializing GLT");
    }

    // set up callbacks (run by glfwPollEvents)
    glfwSetFramebufferSizeCallback(window, handle_window_resize);
    glfwSetKeyCallback(window, handle_key_press);
    glfwSetMouseButtonCallback(window, handle_mouse_button);
    glfwSetCursorPosCallback(window, handle_cursor_pos);
    glfwSetCursorEnterCallback(window, handle_cursor_enter);
    glfwSetScrollCallback(window, handle_scroll);

    // context for glfw callbacks
    glfwSetWindowUserPointer(window, this);

#if GLFW_VERSION_MAJOR >= 3 && GLFW_VERSION_MINOR >= 2
    // prevent window aspect from changing
    if (fix_aspect)
        glfwSetWindowAspectRatio(window, window_width, window_height);
#else
    (void)fix_aspect;
#endif

    // initialize viewport size. Note: this is conceptually different than the
    // window size, and actually different on retina displays. See: glfw docs
    int viewport_width, viewport_height;
    glfwGetFramebufferSize(window, &viewport_width, &viewport_height);

    gltViewport(viewport_width, viewport_height);
    window_context.viewport_width = viewport_width;
    window_context.viewport_height = viewport_height;
}

GLFWContext::~GLFWContext() { glfwDestroyWindow(window); }

void GLFWContext::terminate() {
    // TODO: can't terminate if we allow multiple instances
    gltTerminate();
    glfwTerminate();
}

bool GLFWContext::running() { return !glfwWindowShouldClose(window); }

void GLFWContext::running(bool state) {
    glfwSetWindowShouldClose(window, !state);
}

void GLFWContext::visible(bool state) {
    if (state)
        glfwShowWindow(window);
    else
        glfwHideWindow(window);
}

}  // namespace viz
}  // namespace ouster

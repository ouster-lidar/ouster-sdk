/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#import <AppKit/AppKit.h>

#include <iostream>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3native.h>

namespace ouster {
namespace sdk {
namespace viz {

void set_window_color_space(GLFWwindow* window) {
    @autoreleasepool {
        NSWindow* nswindow = glfwGetCocoaWindow(window);
        if (nswindow == nil) {
            std::cerr << "Warning: glfwGetCocoaWindow returned nil; viz "
                         "window color space not set"
                      << std::endl;
            return;
        }
        [nswindow setColorSpace:[NSColorSpace sRGBColorSpace]];
    }
}

}  // namespace viz
}  // namespace sdk
}  // namespace ouster

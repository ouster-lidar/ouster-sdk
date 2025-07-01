/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved
 */
#pragma once

#include <vector>

#include "glfw.h"

namespace ouster {
namespace viz {
namespace impl {

/**
 * @brief A class that represents a non default framebuffer, that is,
 * a framebuffer that is different from the window framebuffer. It is meant
 * to be used by PointViz to render to a secondary framebuffer for resolution
 * independent screenshots.
 */
class Framebuffer {
   public:
    Framebuffer(int width, int height);
    ~Framebuffer();

    void bind();
    void read_pixels_into(std::vector<uint8_t>& buffer);
    void unbind();

    bool is_complete() const;
    int width() const;
    int height() const;

   private:
    GLuint depth_rbo_handle_;
    GLuint fbo_handle_;
    GLuint texture_handle_;
    int width_;
    int height_;
    bool bound_{false};
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster

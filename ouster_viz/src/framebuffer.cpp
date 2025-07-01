/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved
 */
#include "framebuffer.h"

#include <stdexcept>
#include <vector>

namespace ouster {
namespace viz {
namespace impl {

Framebuffer::Framebuffer(int width, int height)
    : width_(width), height_(height) {
    // Create the FBO
    glGenFramebuffers(1, &fbo_handle_);

    // Bind it
    bind();

    // Create the texture
    glGenTextures(1, &texture_handle_);
    glBindTexture(GL_TEXTURE_2D, texture_handle_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Attach the texture to the FBO
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           texture_handle_, 0);

    // Create the depth renderbuffer
    glGenRenderbuffers(1, &depth_rbo_handle_);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo_handle_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_,
                          height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, depth_rbo_handle_);

    // Unbind the FBO (return to the default framebuffer)
    unbind();
}

void Framebuffer::read_pixels_into(std::vector<uint8_t>& buffer) {
    if (!bound_) {
        throw std::runtime_error("Read framebuffer while not bound");
    }
    if (buffer.size() < static_cast<std::size_t>(width_ * height_ * 3)) {
        throw std::runtime_error(
            "Buffer not large enough for reaading from framebuffer");
    }

    glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE,
                 buffer.data());
}

int Framebuffer::width() const { return width_; }

int Framebuffer::height() const { return height_; }

void Framebuffer::bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_handle_);
    bound_ = true;
}

bool Framebuffer::is_complete() const {
    return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
}

void Framebuffer::unbind() {
    if (bound_) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        bound_ = false;
    }
}

Framebuffer::~Framebuffer() {
    if (bound_) {
        unbind();
    }
    glDeleteRenderbuffers(1, &depth_rbo_handle_);
    depth_rbo_handle_ = 0;

    glDeleteTextures(1, &texture_handle_);
    texture_handle_ = 0;

    glDeleteFramebuffers(1, &fbo_handle_);
    fbo_handle_ = 0;
}

}  // namespace impl
}  // namespace viz
}  // namespace ouster

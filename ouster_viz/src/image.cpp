/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 */

#include "image.h"

#include <stdexcept>
#include <vector>

#include "camera.h"
#include "common.h"
#include "glfw.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

bool GLImage::initialized = false;
GLuint GLImage::vao;
GLuint GLImage::program_id;
GLuint GLImage::vertex_id;
GLuint GLImage::uv_id;
GLuint GLImage::mono_id;
GLuint GLImage::image_id;
GLuint GLImage::mask_id;
GLuint GLImage::palette_id;
GLuint GLImage::use_palette_id;

GLImage::GLImage() {
    if (!GLImage::initialized) {
        throw std::logic_error("GLCloud not initialized");
    }

    glGenBuffers(2, vertexbuffers_.data());

    // initialize index buffer
    GLubyte indices[] = {0, 1, 2, 0, 2, 3};
    glGenBuffers(1, &image_index_id_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, image_index_id_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(GLubyte), indices,
                 GL_STATIC_DRAW);

    GLuint textures[3];
    glGenTextures(3, textures);
    image_texture_id_ = textures[0];
    mask_texture_id_ = textures[1];
    palette_texture_id_ = textures[2];

    // initialize textures
    GLfloat init[4] = {0, 0, 0, 0};
    load_texture(init, 1, 1, image_texture_id_, GL_RED, GL_RED);
    load_texture(init, 1, 1, mask_texture_id_, GL_RGBA, GL_RGBA);
    load_texture(init, 1, 1, palette_texture_id_, GL_RGBA, GL_RGBA);
}

GLImage::GLImage(const Image& /*image*/) : GLImage{} {}

GLImage::~GLImage() {
    glDeleteBuffers(2, vertexbuffers_.data());
    glDeleteTextures(1, &image_texture_id_);
    glDeleteTextures(1, &mask_texture_id_);
    glDeleteTextures(1, &palette_texture_id_);
}

void GLImage::draw(const WindowCtx& ctx, const CameraData& /*unused*/,
                   Image& image) {
    glBindVertexArray(GLImage::vao);
    // update state
    if (image.position_changed_) {
        x0_ = image.position_[0];
        x1_ = image.position_[1];
        y0_ = image.position_[2];
        y1_ = image.position_[3];
        hshift_ = image.hshift_;
        image.position_changed_ = false;
    }

    glUniform1i(image_id, 0);
    glUniform1i(mask_id, 1);
    glUniform1i(palette_id, 2);

    glActiveTexture(GL_TEXTURE0);
    if (image.image_changed_) {
        load_texture(image.image_data_.data(), image.image_width_,
                     image.image_height_, image_texture_id_, GL_RGBA, GL_RGBA,
                     GL_FLOAT);
        image.image_changed_ = false;
    }
    glBindTexture(GL_TEXTURE_2D, image_texture_id_);

    // put the shader into mono or rgb mode
    glUniform1i(mono_id, image.mono_ ? 1 : 0);
    glUniform1i(use_palette_id, image.use_palette_ ? 1 : 0);

    glActiveTexture(GL_TEXTURE1);
    if (image.mask_changed_) {
        if (!image.mask_data_.empty()) {
            load_texture(image.mask_data_.data(), image.mask_width_,
                         image.mask_height_, mask_texture_id_, GL_RGBA,
                         GL_RGBA);
        }
        image.mask_changed_ = false;
    }
    glBindTexture(GL_TEXTURE_2D, mask_texture_id_);

    glActiveTexture(GL_TEXTURE2);
    if (image.palette_changed_) {
        if (!image.palette_data_.empty()) {
            load_texture(image.palette_data_.data(),
                         image.palette_data_.size() / 3, 1,
                         palette_texture_id_);
        }
        image.palette_changed_ = false;
    }
    glBindTexture(GL_TEXTURE_2D, palette_texture_id_);

    // draw
    double aspect = impl::window_aspect(ctx);
    GLfloat x0_scaled = (x0_ / aspect) + hshift_;
    GLfloat x1_scaled = (x1_ / aspect) + hshift_;

    const GLfloat vertices[] = {x0_scaled, y0_, x0_scaled, y1_,
                                x1_scaled, y1_, x1_scaled, y0_};
    const GLfloat texcoords[] = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0};

    glEnableVertexAttribArray(vertex_id);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers_[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * 2, vertices,
                 GL_DYNAMIC_DRAW);
    glVertexAttribPointer(vertex_id,
                          2,              // size
                          GL_FLOAT,       // type
                          GL_FALSE,       // normalized
                          0,              // stride
                          (void*)nullptr  // array buffer offset
    );
    glEnableVertexAttribArray(uv_id);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers_[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * 2, texcoords,
                 GL_DYNAMIC_DRAW);
    glVertexAttribPointer(uv_id,
                          2,              // size
                          GL_FLOAT,       // type
                          GL_FALSE,       // normalized
                          0,              // stride
                          (void*)nullptr  // array buffer offset
    );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, image_index_id_);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, (void*)nullptr);
    glBindVertexArray(0);
}

void GLImage::initialize() {
    glGenVertexArrays(1, &GLImage::vao);
    GLImage::program_id =
        load_shaders(IMAGE_VERTEX_SHADER_CODE, IMAGE_FRAGMENT_SHADER_CODE);
    // TODO: handled differently than cloud ids...
    GLImage::vertex_id = glGetAttribLocation(GLImage::program_id, "vertex");
    GLImage::uv_id = glGetAttribLocation(GLImage::program_id, "vertex_uv");
    GLImage::mono_id = glGetUniformLocation(GLImage::program_id, "mono");
    GLImage::image_id = glGetUniformLocation(GLImage::program_id, "image");
    GLImage::mask_id = glGetUniformLocation(GLImage::program_id, "mask");
    GLImage::palette_id = glGetUniformLocation(GLImage::program_id, "palette");
    GLImage::use_palette_id =
        glGetUniformLocation(GLImage::program_id, "use_palette");
    GLImage::initialized = true;
}

void GLImage::uninitialize() {
    glDeleteProgram(GLImage::program_id);
    glDeleteVertexArrays(1, &GLImage::vao);
}

void GLImage::beginDraw() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(GLImage::program_id);
}

void GLImage::endDraw() {}

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

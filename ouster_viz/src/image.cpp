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
#include "ouster/viz/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

GLImage::GLImage() {
    glGenBuffers(2, vertexbuffers_.data());

    // initialize index buffer
    GLubyte indices[] = {0, 1, 2, 0, 2, 3};
    glGenBuffers(1, &image_index_id_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, image_index_id_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(GLubyte), indices, GL_STATIC_DRAW);

    GLuint texture = 0;
    glGenTextures(1, &texture);
    palette_texture_id_ = texture;

    // initialize textures
    GLfloat init[4] = {0, 0, 0, 0};
    load_texture(init, 1, 1, palette_texture_id_, GL_RGBA, GL_RGBA);
}

GLImage::GLImage(const Image& /*image*/) : GLImage{} {}

GLImage::~GLImage() {
    glDeleteBuffers(2, vertexbuffers_.data());
    glDeleteBuffers(1, &image_index_id_);
    glDeleteTextures(1, &palette_texture_id_);
}

void GLImage::draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& /*unused*/,
                   Image& image) {
    glBindVertexArray(state.vao);
    // update state
    if (image.position_changed_) {
        x0_ = image.position_[0];
        x1_ = image.position_[1];
        y0_ = image.position_[2];
        y1_ = image.position_[3];
        hshift_ = image.hshift_;
        image.position_changed_ = false;
    }

    glUniform1i(state.image_id, 0);
    glUniform1i(state.mask_id, 1);
    glUniform1i(state.palette_id, 2);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, image.image_data_->texture(image.viz_instance_));

    // put the shader into mono or rgb mode
    glUniform1i(state.mono_id, image.mono_ ? 1 : 0);
    glUniform1i(state.use_palette_id, image.use_palette_ ? 1 : 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, image.mask_data_->texture(image.viz_instance_));

    glActiveTexture(GL_TEXTURE2);
    if (image.palette_changed_) {
        if (image.palette_data_) {
            load_texture(image.palette_data_->data(), image.palette_data_->size() / 3, 1,
                         palette_texture_id_);
        }
        image.palette_changed_ = false;
    }
    glBindTexture(GL_TEXTURE_2D, palette_texture_id_);

    // draw
    double aspect = impl::window_aspect(ctx);
    GLfloat x0_scaled = static_cast<GLfloat>((x0_ / aspect) + hshift_);
    GLfloat x1_scaled = static_cast<GLfloat>((x1_ / aspect) + hshift_);

    const GLfloat vertices[] = {x0_scaled, y0_, x0_scaled, y1_, x1_scaled, y1_, x1_scaled, y0_};
    const GLfloat texcoords[] = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0};

    glEnableVertexAttribArray(state.vertex_id);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers_[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * 2, vertices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(state.vertex_id,
                          2,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized
                          0,         // stride
                          nullptr    // array buffer offset
    );
    glEnableVertexAttribArray(state.uv_id);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers_[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * 2, texcoords, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(state.uv_id,
                          2,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized
                          0,         // stride
                          nullptr    // array buffer offset
    );

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, image_index_id_);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, nullptr);
    glBindVertexArray(0);
}

GLImage::GlobalState::GlobalState() {
    glGenVertexArrays(1, &vao);
    // NOLINTBEGIN(cppcoreguidelines-prefer-member-initializer)
    program_id = load_shaders(IMAGE_VERTEX_SHADER_CODE, IMAGE_FRAGMENT_SHADER_CODE);
    // TODO: handled differently than cloud ids...
    vertex_id = glGetAttribLocation(program_id, "vertex");
    uv_id = glGetAttribLocation(program_id, "vertex_uv");
    mono_id = glGetUniformLocation(program_id, "mono");
    image_id = glGetUniformLocation(program_id, "image");
    mask_id = glGetUniformLocation(program_id, "mask");
    palette_id = glGetUniformLocation(program_id, "palette");
    use_palette_id = glGetUniformLocation(program_id, "use_palette");
    // NOLINTEND(cppcoreguidelines-prefer-member-initializer)
}

GLImage::GlobalState::~GlobalState() {
    glDeleteProgram(program_id);
    glDeleteVertexArrays(1, &vao);
}

void GLImage::beginDraw(const GlobalState& state) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(state.program_id);
}

void GLImage::endDraw() {}

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

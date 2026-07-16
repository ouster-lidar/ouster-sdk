/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include "misc.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>

#include "camera.h"
#include "common.h"
#include "glfw.h"
#include "ouster/core/pose_conversion.h"
#include "ouster/viz/point_viz.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

/*
 * Rings
 */
GLRings::GLRings() : ring_size_(1.0), ring_line_width_(1), rings_enabled_(true) {}

void GLRings::update(const TargetDisplay& target) {
    rings_enabled_ = target.rings_enabled_;
    ring_size_ = target.get_ring_size_m();
    ring_line_width_ = target.ring_line_width_;
}

void GLRings::draw(const GlobalState& state, const WindowCtx& /*unused*/,
                   const CameraData& camera) const {
    if (!rings_enabled_) {
        return;
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    glBindVertexArray(state.ring_vao);
    glUseProgram(state.ring_program_id);
    // rings are displayed at the camera target, so model is inverse of target
    Matrix4fR mvp = (camera.proj * camera.view).cast<float>();

    glUniformMatrix4fv(state.ring_proj_view_id, 1, GL_TRUE, mvp.data());
    glUniform1f(state.ring_range_id, static_cast<float>(ring_size_));
    glUniform1f(state.ring_thickness_id, static_cast<float>(ring_line_width_));

    glDrawArrays(GL_TRIANGLES, 0, 6);

    glBindVertexArray(0);

    glDisable(GL_BLEND);
}

GLRings::GlobalState::GlobalState()
    : ring_vao(0),
      ring_program_id(load_shaders(RING_VERTEX_SHADER_CODE, RING_FRAGMENT_SHADER_CODE)),
      ring_xyz_id(glGetAttribLocation(ring_program_id, "ring_xyz")),
      ring_proj_view_id(glGetUniformLocation(ring_program_id, "proj_view")),
      ring_range_id(glGetUniformLocation(ring_program_id, "ring_range")),
      ring_thickness_id(glGetUniformLocation(ring_program_id, "ring_thickness")),
      xyz_buffer(0) {
    glGenVertexArrays(1, &ring_vao);
    glBindVertexArray(ring_vao);

    // Make a quad thats a bit larger than our maximum range
    std::vector<GLfloat> xyz(static_cast<size_t>(3 * 6), 0.0f);
    const float max_range = 1000.0f;
    // Point 0
    xyz[0] = -1.1f;
    xyz[1] = -1.1f;
    xyz[2] = 0.0f;
    // Point 1
    xyz[3] = 1.1f;
    xyz[4] = -1.1f;
    xyz[5] = 0.0f;
    // Point 2
    xyz[6] = 1.1f;
    xyz[7] = 1.1f;
    xyz[8] = 0.0f;
    // Point 3
    xyz[9] = -1.1f;
    xyz[10] = -1.1f;
    xyz[11] = 0.0f;
    // Point 4
    xyz[12] = -1.1f;
    xyz[13] = 1.1f;
    xyz[14] = 0.0f;
    // Point 5
    xyz[15] = 1.1f;
    xyz[16] = 1.1f;
    xyz[17] = 0.0f;

    // scale to expected size
    for (auto& val : xyz) {
        val = val * max_range;
    }
    glGenBuffers(1, &xyz_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeof(GLfloat) * xyz.size()), xyz.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);

    glEnableVertexAttribArray(ring_xyz_id);
    glVertexAttribPointer(ring_xyz_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized
                          0,         // stride
                          nullptr    // array buffer offset
    );

    glBindVertexArray(0);
}

GLRings::GlobalState::~GlobalState() {
    glDeleteProgram(ring_program_id);
    glDeleteVertexArrays(1, &ring_vao);
    glDeleteBuffers(1, &xyz_buffer);
}

/*
 * Cuboids
 */
GLCuboid::GLCuboid() : transform_{core::Matrix4dR::Identity()}, rgba_{0, 0, 0, 0} {}

// for Indexed<T, U>, arg ignored
GLCuboid::GLCuboid(const Cuboid& /*unused*/) : GLCuboid{} {}

GLCuboid::~GLCuboid() = default;

/*
 * Draws the cuboids from the point of view of the camera.
 */
void GLCuboid::draw(const GlobalState& state, const WindowCtx& /*unused*/, const CameraData& camera,
                    Cuboid& cuboid) {
    if (cuboid.transform_changed_) {
        transform_ = cuboid.transform_;
        cuboid.transform_changed_ = false;
    }

    if (cuboid.rgba_changed_) {
        rgba_ = cuboid.rgba_;
        cuboid.rgba_changed_ = false;
    }

    glUniform4fv(state.cuboid_rgba_id, 1, rgba_.data());

    // cuboid pose (model matrix) is separate for some reason
    const Matrix4fR mvp = (camera.proj * camera.view * camera.target * transform_).cast<float>();
    glUniformMatrix4fv(state.cuboid_proj_view_id, 1, GL_TRUE, mvp.data());

    // draw cube faces only if they should be visible to avoid alpha sort issues
    if (rgba_[3] > 0) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, state.indices_buffer);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, nullptr);
    }

    auto rgba = cuboid.rgba_;
    rgba[3] = 1;
    glUniform4fv(state.cuboid_rgba_id, 1, rgba.data());

    // draw cube frame
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, state.edge_indices_buffer);
    glDrawElements(GL_LINES, 24, GL_UNSIGNED_BYTE, nullptr);
}

void GLCuboid::draw_select(const GlobalState& state, const WindowCtx& /*unused*/,
                           const CameraData& camera, Cuboid& cuboid, uint32_t& start_index) {
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glCullFace(GL_BACK);

    if (cuboid.transform_changed_) {
        transform_ = cuboid.transform_;
        cuboid.transform_changed_ = false;
    }

    uint32_t identifier = start_index++;
    float r = static_cast<float>(identifier & 0xFF) / 255.0f;
    float g = static_cast<float>((identifier >> 8) & 0xFF) / 255.0f;
    float b = static_cast<float>((identifier >> 16) & 0xFF) / 255.0f;
    float alpha = 1.0f;

    vec4f id_rgba = {r, g, b, alpha};

    glUniform4fv(state.cuboid_rgba_id, 1, id_rgba.data());

    // cuboid pose (model matrix) is separate for some reason
    const Matrix4fR mvp = (camera.proj * camera.view * camera.target * transform_).cast<float>();
    glUniformMatrix4fv(state.cuboid_proj_view_id, 1, GL_TRUE, mvp.data());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, state.indices_buffer);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, nullptr);
}

/**
 * initializes shader program and handles
 */
GLCuboid::GlobalState::GlobalState()
    : cuboid_vao(0),
      cuboid_program_id(load_shaders(CUBOID_VERTEX_SHADER_CODE, CUBOID_FRAGMENT_SHADER_CODE)),
      cuboid_xyz_id(glGetAttribLocation(cuboid_program_id, "cuboid_xyz")),
      cuboid_proj_view_id(glGetUniformLocation(cuboid_program_id, "proj_view")),
      cuboid_rgba_id(glGetUniformLocation(cuboid_program_id, "cuboid_rgba")),
      xyz{0.5f,  0.5f, 0.5f, 0.5f,  0.5f, -0.5f, 0.5f,  -0.5f, 0.5f, 0.5f,  -0.5f, -0.5f,
          -0.5f, 0.5f, 0.5f, -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, 0.5f, -0.5f, -0.5f, -0.5f},
      indices{0, 1, 2, 2, 1, 3, 6, 7, 4, 4, 7, 5, 2, 3, 6, 6, 3, 7,
              4, 5, 0, 0, 5, 1, 0, 2, 4, 4, 2, 6, 5, 7, 1, 1, 7, 3},
      edge_indices{0, 1, 1, 3, 3, 2, 2, 0, 4, 5, 5, 7, 7, 6, 6, 4, 0, 4, 1, 5, 2, 6, 3, 7} {
    glGenVertexArrays(1, &cuboid_vao);
    glGenBuffers(1, &xyz_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeof(GLfloat) * 24), xyz.data(),
                 GL_STATIC_DRAW);

    glGenBuffers(1, &indices_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(GLubyte)),
                 indices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &edge_indices_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_indices_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(edge_indices.size() * sizeof(GLubyte)),
                 edge_indices.data(), GL_STATIC_DRAW);
}

GLCuboid::GlobalState::~GlobalState() {
    glDeleteProgram(cuboid_program_id);
    glDeleteVertexArrays(1, &cuboid_vao);
    glDeleteBuffers(1, &xyz_buffer);
    glDeleteBuffers(1, &indices_buffer);
    glDeleteBuffers(1, &edge_indices_buffer);
}

void GLCuboid::beginDraw(const GlobalState& state) {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    // enable culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    glUseProgram(state.cuboid_program_id);
    glBindVertexArray(state.cuboid_vao);
    glEnableVertexAttribArray(state.cuboid_xyz_id);
    glBindBuffer(GL_ARRAY_BUFFER, state.xyz_buffer);
    glVertexAttribPointer(state.cuboid_xyz_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized
                          0,         // stride
                          nullptr    // array buffer offset
    );
}

void GLCuboid::endDraw() {
    glBindVertexArray(0);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

/*
 * Lines
 */
GLLines::GLLines() : rgba_{0, 0, 0, 0} {
    glGenBuffers(1, &xyz_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer_);

    transform_ = core::Matrix4dR::Identity();
}

// for Indexed<T, U>, arg ignored
GLLines::GLLines(const Lines& /*unused*/) : GLLines{} {}

GLLines::~GLLines() {
    glDeleteBuffers(1, &xyz_buffer_);
}

/*
 * Draws the lines from the point of view of the camera.
 */
void GLLines::draw(const GlobalState& state, const WindowCtx& /*unused*/, const CameraData& camera,
                   Lines& lines) {
    if (lines.transform_changed_) {
        transform_ = lines.transform_;
        lines.transform_changed_ = false;
    }

    if (lines.rgba_changed_) {
        rgba_ = lines.rgba_;
        lines.rgba_changed_ = false;
    }

    if (!lines.point_data_) {
        return;  // nothing to draw
    }

    if (lines.points_changed_) {
        glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(sizeof(GLfloat) * lines.point_data_->size()),
                     lines.point_data_->data(), GL_STATIC_DRAW);
        lines.points_changed_ = false;
    }

    glBindVertexArray(state.lines_vao);
    glUniform4fv(state.lines_rgba_id, 1, rgba_.data());

    // cuboid pose (model matrix) is separate for some reason
    const Matrix4fR mvp = (camera.proj * camera.view * camera.target * transform_).cast<float>();
    glUniformMatrix4fv(state.lines_proj_view_id, 1, GL_TRUE, mvp.data());
    glEnableVertexAttribArray(state.lines_xyz_id);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer_);
    glVertexAttribPointer(state.lines_xyz_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized
                          0,         // stride
                          nullptr    // array buffer offset
    );

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.point_data_->size() / 3));

    glBindVertexArray(0);
}

/**
 * initializes shader program and handles
 */
GLLines::GlobalState::GlobalState()
    : lines_vao(0),
      lines_program_id(load_shaders(LINES_VERTEX_SHADER_CODE, LINES_FRAGMENT_SHADER_CODE)),
      lines_xyz_id(glGetAttribLocation(lines_program_id, "lines_xyz")),
      lines_proj_view_id(glGetUniformLocation(lines_program_id, "proj_view")),
      lines_rgba_id(glGetUniformLocation(lines_program_id, "lines_rgba")) {
    glGenVertexArrays(1, &lines_vao);
}

GLLines::GlobalState::~GlobalState() {
    glDeleteProgram(lines_program_id);
    glDeleteVertexArrays(1, &lines_vao);
}

void GLLines::beginDraw(const GlobalState& state) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    glLineWidth(1.0f);

    glUseProgram(state.lines_program_id);
}

void GLLines::endDraw() {
    glDisable(GL_BLEND);
}

/*
 * Label3d
 */
GLLabel::GLLabel()
    : gltext_{gltCreateText()},
      text_position_{0, 0, 0},
      is_3d_(false),
      scale_(1.0f),
      halign_(GLT_LEFT),
      valign_(GLT_BOTTOM),
      rgba_{0, 0, 0, 0} {
    assert(gltext_ != GLT_NULL);
}

// for Indexed<T, U>
GLLabel::GLLabel(const Label& /*unused*/) : GLLabel{} {}

GLLabel::~GLLabel() {
    gltDeleteText(gltext_);
}

void GLLabel::draw(const GlobalState& /*state*/, const WindowCtx& ctx, const CameraData& camera,
                   Label& label) {
    if (label.text_changed_) {
        gltSetText(gltext_, label.text_.c_str());
        label.text_changed_ = false;
    }

    if (label.pos_changed_) {
        text_position_ = Eigen::Map<const Eigen::Vector3d>{label.position_.data()};
        is_3d_ = label.is_3d_;
        halign_ = label.align_right_ ? GLT_RIGHT : GLT_LEFT;
        valign_ = label.align_top_ ? GLT_TOP : GLT_BOTTOM;
        label.pos_changed_ = false;
    }

    if (label.scale_changed_) {
        scale_ = label.scale_;
        label.scale_changed_ = false;
    }

    if (label.rgba_changed_) {
        rgba_ = label.rgba_;
        label.rgba_changed_ = false;
    }

    gltColor(rgba_[0], rgba_[1], rgba_[2], rgba_[3]);

    if (is_3d_) {
        auto view_target = camera.view * camera.target;
        core::Matrix4dR model = (Eigen::Translation3d{text_position_.cast<double>()} *
                                 // make text face the camera
                                 Eigen::Affine3d{view_target.block<3, 3>(0, 0).inverse()} *
                                 // text rendered +z direction, needs to be flipped
                                 Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                                 // scale text, could make this configurable
                                 Eigen::Scaling(0.02 * static_cast<double>(scale_)))
                                    .matrix();

        Matrix4fR mvp = (camera.proj * camera.view * camera.target * model).cast<float>();

        Eigen::Matrix4f glt_mvp = mvp;
        gltDrawText(gltext_, glt_mvp.data());
    } else {
        float x = static_cast<float>(text_position_.x()) * static_cast<float>(ctx.viewport_width);
        float y = static_cast<float>(text_position_.y()) * static_cast<float>(ctx.viewport_height);
        float scale2d = scale_ * GLFWContext::ui_scale();
        gltDrawText2DAligned(gltext_, x, y, scale2d, halign_, valign_);
    }
}

void GLLabel::beginDraw(const GlobalState& /*state*/) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);
    gltBeginDraw();
}

void GLLabel::endDraw() {
    gltEndDraw();
    glDisable(GL_BLEND);
}

float GLLabel::get_text_height(const Label& label) {
    if (label.text_.empty()) {
        return 0.0f;
    }
    float scale = label.scale_ * GLFWContext::ui_scale();
    return static_cast<float>(gltCountNewLines(label.text_.c_str()) + 1) * gltGetLineHeight(scale);
}

core::Matrix4dR object_to_cuboid_transform(const ouster::sdk::core::Object& obj) {
    core::Matrix4dR model = core::Matrix4dR::Identity();
    // Scaling
    model(0, 0) = obj.dimensions.x();
    model(1, 1) = obj.dimensions.y();
    model(2, 2) = obj.dimensions.z();

    return (obj.body_to_world * obj.object_to_body).to_matrix() * model;
}

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

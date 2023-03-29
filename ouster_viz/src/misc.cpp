/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include "misc.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "camera.h"
#include "common.h"
#include "glfw.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

/*
 * Rings
 */
bool GLRings::initialized = false;
GLuint GLRings::ring_program_id;
GLuint GLRings::ring_xyz_id;
GLuint GLRings::ring_proj_view_id;
GLuint GLRings::ring_range_id;

GLRings::GLRings(const size_t points_per_ring_)
    : points_per_ring(points_per_ring_),
      ring_size_(1),
      ring_line_width_(1),
      rings_enabled(true) {
    std::vector<GLfloat> xyz(points_per_ring_ * 3, 0);
    for (size_t i = 0; i < points_per_ring; i++) {
        const GLfloat theta = i * 2.0 * M_PI / points_per_ring;
        xyz[3 * i] = std::sin(theta);
        xyz[3 * i + 1] = std::cos(theta);
        xyz[3 * i + 2] = 0.0;
    }
    glGenBuffers(1, &xyz_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * points_per_ring * 3,
                 xyz.data(), GL_STATIC_DRAW);
}

void GLRings::update(const TargetDisplay& target) {
    rings_enabled = target.rings_enabled_;
    ring_size_ = target.ring_size_;
    ring_line_width_ = target.ring_line_width_;
}

void GLRings::draw(const WindowCtx&, const CameraData& camera) {
    if (!GLRings::initialized)
        throw std::logic_error("GLRings not initialized");

    if (!rings_enabled) return;

    glUseProgram(GLRings::ring_program_id);
    glLineWidth(1);
    const float radius = std::pow(10.0f, ring_size_);
    // rings are displayed at the camera target, so model is inverse of target
    Eigen::Matrix4f mvp = (camera.proj * camera.view).cast<float>();
    glUniformMatrix4fv(GLRings::ring_proj_view_id, 1, GL_FALSE, mvp.data());
    glEnableVertexAttribArray(GLRings::ring_xyz_id);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
    glVertexAttribPointer(GLRings::ring_xyz_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );
    const GLfloat max_radius = 1000;
    const GLfloat max_rings = 2000;  // for performance
    for (GLfloat r = radius, rr = 0; r < max_radius && rr < max_rings;
         r += radius, rr += 1) {
        glUniform1f(GLRings::ring_range_id, r);
        glDrawArrays(GL_LINE_LOOP, 0, points_per_ring);
        // Making more paths to thicken the line
        // TODO[pb]: Need to find a better way to draw a thick lines, this
        //           method is too gross (slow and rugged) :(
        for (int lw = 1; lw < ring_line_width_; ++lw) {
            glUniform1f(GLRings::ring_range_id, r + lw * 0.02);
            glDrawArrays(GL_LINE_LOOP, 0, points_per_ring);
            glUniform1f(GLRings::ring_range_id, r - lw * 0.02);
            glDrawArrays(GL_LINE_LOOP, 0, points_per_ring);
        }
    }
    glDisableVertexAttribArray(GLRings::ring_xyz_id);
}

void GLRings::initialize() {
    GLRings::ring_program_id =
        load_shaders(ring_vertex_shader_code, ring_fragment_shader_code);
    GLRings::ring_xyz_id = glGetAttribLocation(ring_program_id, "ring_xyz");
    GLRings::ring_proj_view_id =
        glGetUniformLocation(ring_program_id, "proj_view");
    GLRings::ring_range_id =
        glGetUniformLocation(ring_program_id, "ring_range");
    GLRings::initialized = true;
}

void GLRings::uninitialize() {
    GLRings::initialized = false;
    glDeleteProgram(ring_program_id);
}

/*
 * Cuboids
 */
bool GLCuboid::initialized = false;
GLuint GLCuboid::cuboid_program_id;
GLuint GLCuboid::cuboid_xyz_id;
GLuint GLCuboid::cuboid_proj_view_id;
GLuint GLCuboid::cuboid_rgba_id;

GLCuboid::GLCuboid()
    : xyz{+0.5, +0.5, +0.5, +0.5, +0.5, -0.5, +0.5, -0.5,
          +0.5, +0.5, -0.5, -0.5, -0.5, +0.5, +0.5, -0.5,
          +0.5, -0.5, -0.5, -0.5, +0.5, -0.5, -0.5, -0.5},
      indices{0, 1, 2, 2, 1, 3, 6, 7, 4, 4, 7, 5, 2, 3, 6, 6, 3, 7,
              4, 5, 0, 0, 5, 1, 0, 2, 4, 4, 2, 6, 5, 7, 1, 1, 7, 3},
      edge_indices{0, 1, 1, 3, 3, 2, 2, 0, 4, 5, 5, 7,
                   7, 6, 6, 4, 0, 4, 1, 5, 2, 6, 3, 7} {
    glGenBuffers(1, &xyz_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 24, xyz.data(),
                 GL_STATIC_DRAW);

    glGenBuffers(1, &indices_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLubyte),
                 indices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &edge_indices_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_indices_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, edge_indices.size() * sizeof(GLubyte),
                 edge_indices.data(), GL_STATIC_DRAW);

    transform = Eigen::Matrix4d::Identity();
}

// for Indexed<T, U>, arg ignored
GLCuboid::GLCuboid(const Cuboid&) : GLCuboid{} {}

GLCuboid::~GLCuboid() { glDeleteBuffers(1, &xyz_buffer); }

/*
 * Draws the cuboids from the point of view of the camera.
 */
void GLCuboid::draw(const WindowCtx&, const CameraData& camera,
                    Cuboid& cuboid) {
    if (!GLCuboid::initialized)
        throw std::logic_error("GLCuboid not initialized");

    if (cuboid.transform_changed_) {
        transform = Eigen::Map<const Eigen::Matrix4d>{cuboid.transform_.data()};
        cuboid.transform_changed_ = false;
    }

    if (cuboid.rgba_changed_) {
        rgba = cuboid.rgba_;
        cuboid.rgba_changed_ = false;
    }

    glUniform4fv(GLCuboid::cuboid_rgba_id, 1, rgba.data());

    // cuboid pose (model matrix) is separate for some reason
    const Eigen::Matrix4f mvp =
        (camera.proj * camera.view * camera.target * transform).cast<float>();
    glUniformMatrix4fv(GLCuboid::cuboid_proj_view_id, 1, GL_FALSE, mvp.data());
    glEnableVertexAttribArray(GLCuboid::cuboid_xyz_id);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
    glVertexAttribPointer(GLCuboid::cuboid_xyz_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    // draw cube faces
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices_buffer);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, (void*)0);

    auto rgba = cuboid.rgba_;
    rgba[3] = 1;
    glUniform4fv(GLCuboid::cuboid_rgba_id, 1, rgba.data());

    // draw cube frame
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_indices_buffer);
    glDrawElements(GL_LINES, 24, GL_UNSIGNED_BYTE, (void*)0);

    glDisableVertexAttribArray(GLCuboid::cuboid_xyz_id);
}

/**
 * initializes shader program and handles
 */
void GLCuboid::initialize() {
    GLCuboid::cuboid_program_id =
        load_shaders(cuboid_vertex_shader_code, cuboid_fragment_shader_code);
    GLCuboid::cuboid_xyz_id =
        glGetAttribLocation(cuboid_program_id, "cuboid_xyz");
    GLCuboid::cuboid_proj_view_id =
        glGetUniformLocation(cuboid_program_id, "proj_view");
    GLCuboid::cuboid_rgba_id =
        glGetUniformLocation(cuboid_program_id, "cuboid_rgba");
    GLCuboid::initialized = true;
}

void GLCuboid::uninitialize() {
    GLCuboid::initialized = false;
    glDeleteProgram(GLCuboid::cuboid_program_id);
}

void GLCuboid::beginDraw() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    // enable culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    glUseProgram(cuboid_program_id);
}

void GLCuboid::endDraw() {
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
}

/*
 * Label3d
 */
GLLabel::GLLabel() : gltext{gltCreateText()}, text_position{0, 0, 0} {
    assert(gltext != GLT_NULL);
}

// for Indexed<T, U>
GLLabel::GLLabel(const Label&) : GLLabel{} {}

GLLabel::~GLLabel() { gltDeleteText(gltext); }

void GLLabel::draw(const WindowCtx& ctx, const CameraData& camera,
                   Label& label) {
    if (label.text_changed_) {
        gltSetText(gltext, label.text_.c_str());
        label.text_changed_ = false;
    }

    if (label.pos_changed_) {
        text_position =
            Eigen::Map<const Eigen::Vector3d>{label.position_.data()};
        is_3d = label.is_3d_;
        halign = label.align_right_ ? GLT_RIGHT : GLT_LEFT;
        valign = label.align_top_ ? GLT_TOP : GLT_BOTTOM;
        label.pos_changed_ = false;
    }

    if (label.scale_changed_) {
        scale = label.scale_;
        label.scale_changed_ = false;
    }

    if (label.rgba_changed_) {
        rgba = label.rgba_;
        label.rgba_changed_ = false;
    }

    gltColor(rgba[0], rgba[1], rgba[2], rgba[3]);

    if (is_3d) {
        Eigen::Matrix4d model =
            (Eigen::Translation3d{text_position.cast<double>()} *
             // make text face the camera
             Eigen::Affine3d{camera.view.block<3, 3>(0, 0).inverse()} *
             // text rendered +z direction, needs to be flipped
             Eigen::AngleAxisd{M_PI, Eigen::Vector3d::UnitX()} *
             // scale text, could make this configurable
             Eigen::Scaling(0.02 * scale))
                .matrix();

        Eigen::Matrix4f mvp =
            (camera.proj * camera.view * camera.target * model).cast<float>();

        gltDrawText(gltext, mvp.data());
    } else {
        float x = text_position.x() * ctx.viewport_width;
        float y = text_position.y() * ctx.viewport_height;
        float scale2d = scale;
#ifdef __APPLE__
        // TODO: maybe try turning GLFW_COCOA_RETINA_FRAMEBUFFER off
        // TODO[pb]: Also we can start using GLFW window_content_scale for this
        scale2d *= 2.0;
#endif
        gltDrawText2DAligned(gltext, x, y, scale2d, halign, valign);
    }
}

void GLLabel::beginDraw() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);
    gltBeginDraw();
}

void GLLabel::endDraw() {
    gltEndDraw();
    glDisable(GL_BLEND);
}

}  // namespace impl
}  // namespace viz
}  // namespace ouster

/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>

#include "camera.h"
#include "glfw.h"
#include "gltext.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

/*
 * Render a set of rings on the ground as markers to helpvisualize lidar range.
 */
class GLRings {
    static bool initialized;
    static GLuint ring_program_id;
    static GLuint ring_xyz_id;
    static GLuint ring_proj_view_id;
    static GLuint ring_range_id;

    const size_t points_per_ring;
    GLuint xyz_buffer;
    int ring_size_;
    bool rings_enabled;

   public:
    /*
     * Parameter etermines number of points used to draw ring, the more the
     * rounder
     */
    GLRings(const size_t points_per_ring_ = 512);

    void update(const TargetDisplay& target);

    /*
     * Draws the rings from the point of view of the camera. The rings are
     * always centered on the camera's target.
     */
    void draw(const WindowCtx& ctx, const CameraData& camera);

    /*
     * Initializes shader program, vertex buffers and handles after OpenGL
     * context has been created
     */
    static void initialize();

    static void uninitialize();
};

/*
 * Manages opengl state for drawing a cuboid
 */
class GLCuboid {
    static bool initialized;
    static GLuint cuboid_program_id;
    static GLuint cuboid_xyz_id;
    static GLuint cuboid_proj_view_id;
    static GLuint cuboid_rgba_id;

    const std::array<GLfloat, 24> xyz;
    const std::array<GLubyte, 36> indices;
    const std::array<GLubyte, 24> edge_indices;

    GLuint xyz_buffer{0};
    GLuint indices_buffer{0};
    GLuint edge_indices_buffer{0};
    Eigen::Matrix4d transform;
    std::array<float, 4> rgba;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GLCuboid();

    /*
     * For Indexed<T, U>, arg ignored
     */
    GLCuboid(const Cuboid&);

    ~GLCuboid();

    /*
     * Draws the cuboids from the point of view of the camera
     */
    void draw(const WindowCtx& ctx, const CameraData& camera, Cuboid& cuboid);

    /*
     * Initializes shader program and handles
     */
    static void initialize();

    static void uninitialize();

    static void beginDraw();

    static void endDraw();
};

class GLLabel {
    GLTtext* gltext;
    Eigen::Vector3d text_position;
    bool is_3d;
    float scale;
    int halign;
    int valign;

    std::array<float, 4> rgba;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GLLabel();

    // for Indexed<T, U>
    GLLabel(const Label&);

    GLLabel(const GLLabel&) = delete;

    ~GLLabel();

    GLLabel& operator=(const GLLabel&) = delete;

    void draw(const WindowCtx& ctx, const CameraData& camera, Label& label);

    static void beginDraw();

    static void endDraw();
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster

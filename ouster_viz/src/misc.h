/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <array>

#include "camera.h"
#include "glfw.h"
#include "gltext.h"
#include "ouster/viz/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

/*
 * Render a set of rings on the ground as markers to helpvisualize lidar range.
 */
class GLRings {
    double ring_size_;
    int ring_line_width_;
    bool rings_enabled_;

   public:
    struct GlobalState {
        GLuint ring_vao;
        GLuint ring_program_id;
        GLint ring_xyz_id;
        GLint ring_proj_view_id;
        GLint ring_range_id;
        GLint ring_thickness_id;
        GLuint xyz_buffer;

        GlobalState();
        ~GlobalState();

        // Make sure it cant be copied
        GlobalState(const GlobalState&) = delete;
        GlobalState& operator=(const GlobalState&) = delete;
    };

    /*
     * Instantiate the rings
     */
    GLRings();

    void update(const TargetDisplay& target);

    /*
     * Draws the rings from the point of view of the camera. The rings are
     * always centered on the camera's target.
     */
    void draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera) const;
};

/*
 * Manages opengl state for drawing a cuboid
 */
class GLCuboid {
    core::Matrix4dR transform_;
    vec4f rgba_;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct GlobalState {
        GLuint cuboid_vao;
        GLuint cuboid_program_id;
        GLint cuboid_xyz_id;
        GLint cuboid_proj_view_id;
        GLint cuboid_rgba_id;
        const std::array<GLfloat, 24> xyz;
        const std::array<GLubyte, 36> indices;
        const std::array<GLubyte, 24> edge_indices;

        GLuint xyz_buffer{0};
        GLuint indices_buffer{0};
        GLuint edge_indices_buffer{0};

        GlobalState();
        ~GlobalState();

        // Make sure it cant be copied
        GlobalState(const GlobalState&) = delete;
        GlobalState& operator=(const GlobalState&) = delete;
    };

    GLCuboid();

    /*
     * For Indexed<T, U>, arg ignored
     */
    GLCuboid(const Cuboid&);

    ~GLCuboid();

    /*
     * Draws the cuboids from the point of view of the camera
     */
    void draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera,
              Cuboid& cuboid);

    /*
     * Draws the cuboid for selection
     */
    void draw_select(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera,
                     Cuboid& cuboid, uint32_t& start_index);

    static void beginDraw(const GlobalState& state);

    static void endDraw();
};

/*
 * Manages opengl state for drawing lines
 */
class GLLines {
    GLuint xyz_buffer_{0};
    core::Matrix4dR transform_;
    vec4f rgba_;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct GlobalState {
        GLuint lines_vao;
        GLuint lines_program_id;
        GLint lines_xyz_id;
        GLint lines_proj_view_id;
        GLint lines_rgba_id;

        GlobalState();
        ~GlobalState();

        // Make sure it cant be copied
        GlobalState(const GlobalState&) = delete;
        GlobalState& operator=(const GlobalState&) = delete;
    };

    GLLines();

    /*
     * For Indexed<T, U>, arg ignored
     */
    GLLines(const Lines&);

    ~GLLines();

    /*
     * Draws the cuboids from the point of view of the camera
     */
    void draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera,
              Lines& lines);

    static void beginDraw(const GlobalState& state);

    static void endDraw();
};

class GLLabel {
    GLTtext* gltext_;
    Eigen::Vector3d text_position_;
    bool is_3d_;
    float scale_;
    int halign_;
    int valign_;

    vec4f rgba_;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct GlobalState {
        GlobalState() = default;
        ~GlobalState() = default;

        // Make sure it cant be copied
        GlobalState(const GlobalState&) = delete;
        GlobalState& operator=(const GlobalState&) = delete;
    };

    GLLabel();

    // for Indexed<T, U>
    GLLabel(const Label&);

    GLLabel(const GLLabel&) = delete;

    ~GLLabel();

    GLLabel& operator=(const GLLabel&) = delete;

    void draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera,
              Label& label);

    static void beginDraw(const GlobalState& state);

    static void endDraw();

    static float get_text_height(const Label& label);
};

/*
 * Convert an Object to a 4x4 transform for a unit cuboid
 */
core::Matrix4dR object_to_cuboid_transform(const ouster::sdk::core::Object& obj);

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

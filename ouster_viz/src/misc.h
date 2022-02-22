#pragma once

#include <GL/glew.h>

#include <Eigen/Core>
#include <array>
#include <cstddef>

#include "camera.h"
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
    void draw(const CameraData& camera);

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
    static GLuint cuboid_pose_id;
    static GLuint cuboid_rgba_id;

    const std::array<GLfloat, 24> xyz;
    const std::array<GLubyte, 36> indices;
    const std::array<GLubyte, 24> edge_indices;

    GLuint xyz_buffer{0};
    GLuint indices_buffer{0};
    GLuint edge_indices_buffer{0};
    Eigen::Matrix4f pose;
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
    void draw(const CameraData& camera, Cuboid& cuboid);

    /*
     * Initializes shader program and handles
     */
    static void initialize();

    static void uninitialize();

    static void beginDraw();

    static void endDraw();
};

class GLLabel3d {
    GLTtext* gltext;
    Eigen::Vector3d text_position;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GLLabel3d();

    // for Indexed<T, U>
    GLLabel3d(const Label3d&);

    GLLabel3d(const GLLabel3d&) = delete;

    ~GLLabel3d();

    GLLabel3d& operator=(const GLLabel3d&) = delete;

    void draw(const CameraData& camera, Label3d& label);

    static void beginDraw();

    static void endDraw();
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster

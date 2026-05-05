/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Core>

#include "camera.h"
#include "glfw.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

/*
 * Contains handles to variables in GLSL shader program compiled from
 * point_vertex_shader_code and point_fragment_shader_code
 */
struct CloudIds;

/*
 * Manages opengl state for drawing a point cloud
 */
class GLCloud {
    // global gl state
    static bool initialized;
    static GLuint program_id;
    static CloudIds cloud_ids;

   private:
    /// @brief For initializing the VAO during construction
    void initialize_vao();

    // per-object gl state
    GLuint vao_;
    GLuint xyz_buffer_;
    GLuint off_buffer_;
    GLuint range_buffer_;
    GLuint key_buffer_;
    GLuint mask_buffer_;
    GLuint trans_index_buffer_;
    GLuint transform_texture_;
    GLuint palette_texture_;
    GLfloat point_size_;
    bool mono_;

    Eigen::Matrix4d map_pose_;
    Eigen::Matrix4f extrinsic_;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*
     * Set up the Cloud. Most of these arguments should correspond to CloudSetup
     */
    GLCloud(const Cloud& cloud);

    ~GLCloud();

    /*
     * Render the point cloud with the point of view of the Camera
     */
    void draw(const WindowCtx& ctx, const CameraData& camera, Cloud& cloud);

    static void initialize();

    static void uninitialize();

    static void beginDraw();

    static void endDraw();
};

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

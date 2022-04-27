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
    static GLfloat program_id;
    static CloudIds cloud_ids;

   private:
    // per-object gl state
    GLuint xyz_buffer;
    GLuint off_buffer;
    GLuint range_buffer;
    GLuint key_buffer;
    GLuint mask_buffer;
    GLuint trans_index_buffer;
    GLuint transform_texture;
    GLuint palette_texture;
    GLfloat point_size;

    Eigen::Matrix4d map_pose;
    Eigen::Matrix4f extrinsic;

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
}  // namespace ouster

/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Core>

#include "camera.h"
#include "glfw.h"
#include "ouster/viz/point_viz.h"

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
   private:
    struct ProgramIds {
        GLuint program_id;

        // Ordered only ids
        GLint off_id, range_id, transformation_id, trans_index_id, model_id, proj_view_id;

        // Unordered only ids
        GLint proj_view_model_id;

        // Common ids for all
        GLint xyz_id, key_id, mono_id, palette_id;

        // Other optional ids
        GLint mask_id;

        ProgramIds(){};

        ProgramIds(const std::string& vertexs, const std::string& frags);

        ~ProgramIds();
    };

    // per-object gl state
    GLuint ordered_vao_, unordered_vao_, maskless_unordered_vao_;
    GLfloat point_size_;
    bool mono_;

    size_t last_trans_index_key_ = 0;
    std::shared_ptr<BufferReference> trans_index_buffer_;
    std::shared_ptr<BufferReference> cached_mask_;

    core::Matrix4dR map_pose_;
    Matrix4fR extrinsic_;
    const ProgramIds* last_program_ = 0;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct GlobalState {
        ProgramIds ordered, unordered, unordered_maskless;

        GlobalState();
        ~GlobalState();

        // Make sure it can't be copied
        GlobalState(const GlobalState&) = delete;
        GlobalState& operator=(const GlobalState&) = delete;
    };

    /*
     * Set up the Cloud. Most of these arguments should correspond to CloudSetup
     */
    GLCloud(const Cloud& cloud);

    ~GLCloud();

    /*
     * Render the point cloud with the point of view of the Camera
     */
    void draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera,
              Cloud& cloud);

    void draw_select(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera,
                     Cloud& cloud, uint32_t& start_index);

    static void beginDraw(const GlobalState& state);

    static void endDraw();

   private:
    void update_buffers(const GlobalState& state, const CameraData& camera, Cloud& cloud,
                        const ProgramIds& program);
};

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <array>

#include "camera.h"
#include "glfw.h"
#include "ouster/viz/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

/*
 * Manages opengl state for drawing a point cloud
 *
 * The image is auto-scaled to be positioned either at the top (for wide images)
 * or the left (for tall images)
 */
class GLImage {
    constexpr static int SIZE_FRACTION_MAX = 20;

    // per-image gl state
    std::array<GLuint, 2> vertexbuffers_;
    GLuint palette_texture_id_{0};
    GLuint image_index_id_{0};

    float x0_{-1}, x1_{0}, y0_{0}, y1_{-1}, hshift_{0};

   public:
    struct GlobalState {
        GLuint vao;
        GLuint program_id;
        GLuint vertex_id;
        GLuint uv_id;
        GLint mono_id;
        GLint image_id;
        GLint mask_id;
        GLint palette_id;
        GLint use_palette_id;

        GlobalState();
        ~GlobalState();

        // Make sure it cant be copied
        GlobalState(const GlobalState&) = delete;
        GlobalState& operator=(const GlobalState&) = delete;
    };

    GLImage();

    GLImage(const Image& image);

    ~GLImage();

    /*
     * Render the monochrome image.
     *
     * Modifies the camera to offset it so that it is centered on the region not
     * covered by image.
     */
    void draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera,
              Image& image);

    static void beginDraw(const GlobalState& state);

    static void endDraw();
};

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

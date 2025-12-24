/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <array>

#include "camera.h"
#include "glfw.h"
#include "ouster/point_viz.h"

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

    // global gl state
    static bool initialized;
    static GLuint vao;
    static GLuint program_id;
    static GLuint vertex_id;
    static GLuint uv_id;
    static GLuint mono_id;
    static GLuint image_id;
    static GLuint mask_id;
    static GLuint palette_id;
    static GLuint use_palette_id;

    // per-image gl state
    std::array<GLuint, 2> vertexbuffers_;
    GLuint image_texture_id_{0};
    GLuint mask_texture_id_{0};
    GLuint palette_texture_id_{0};
    GLuint image_index_id_{0};

    float x0_{-1}, x1_{0}, y0_{0}, y1_{-1}, hshift_{0};

   public:
    GLImage();

    GLImage(const Image& image);

    ~GLImage();

    /*
     * Render the monochrome image.
     *
     * Modifies the camera to offset it so that it is centered on the region not
     * covered by image.
     */
    void draw(const WindowCtx& ctx, const CameraData& camera, Image& image);

    static void initialize();

    static void uninitialize();

    static void beginDraw();

    static void endDraw();
};

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

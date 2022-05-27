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
namespace viz {
namespace impl {

/*
 * Manages opengl state for drawing a point cloud
 *
 * The image is auto-scaled to be positioned either at the top (for wide images)
 * or the left (for tall images)
 */
class GLImage {
    constexpr static int size_fraction_max = 20;

    // global gl state
    static bool initialized;
    static GLuint program_id;
    static GLuint vertex_id;
    static GLuint uv_id;
    static GLuint image_id;
    static GLuint mask_id;

    // per-image gl state
    std::array<GLuint, 2> vertexbuffers;
    GLuint image_texture_id{0};
    GLuint mask_texture_id{0};
    GLuint image_index_id{0};

    float x0{-1}, x1{0}, y0{0}, y1{-1}, hshift{0};

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
}  // namespace ouster

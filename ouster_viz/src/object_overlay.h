/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <array>
#include <vector>

#include "camera.h"
#include "glfw.h"
#include "ouster/viz/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

/*
 * Manages opengl state for drawing object overlays in panorama
 */
class GLObjectOverlay {
    size_t vertex_count_{0};

    size_t face_vertex_count_{0};

    float x0_{-1}, x1_{0}, y0_{0}, y1_{-1}, hshift_{0};
    std::vector<float> altitude_angles_;
    float v_min_{0}, v_max_{0};

    GLuint v_lookup_tex;
    GLuint vbo;
    GLuint face_vbo;

   public:
    struct GlobalState {
        GLuint vao;
        GLuint face_vao;
        GLuint program_id{};
        GLint pos_id{};
        GLint center_id{};
        GLint color_attr_id{};
        GLint model_id{};
        GLint view_id{};
        GLint fov_h_id{};
        GLint v_lookup_id{};
        GLint v_min_id{};
        GLint v_max_id{};
        GLint x0_id{};
        GLint x1_id{};
        GLint y0_id{};
        GLint y1_id{};
        GLint hshift_id{};
        GLint aspect_id{};
        GLint color_id{};

        GLuint face_program_id{};
        GLint face_pos_id{};
        GLint face_center_id{};
        GLint face_color_attr_id{};
        GLint face_model_id{};
        GLint face_view_id{};
        GLint face_fov_h_id{};
        GLint face_v_lookup_id{};
        GLint face_v_min_id{};
        GLint face_v_max_id{};
        GLint face_x0_id{};
        GLint face_x1_id{};
        GLint face_y0_id{};
        GLint face_y1_id{};
        GLint face_hshift_id{};
        GLint face_aspect_id{};

        GlobalState();
        ~GlobalState();

        // Make sure it cant be copied
        GlobalState(const GlobalState&) = delete;
        GlobalState& operator=(const GlobalState&) = delete;
    };

    // TODO delete
    GLObjectOverlay(const ObjectOverlay& overlay);

    ~GLObjectOverlay();

    /*
     * Render the objects in panorama.
     */
    void draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera,
              ObjectOverlay& overlay);

    static void beginDraw(const GlobalState& state);

    static void endDraw();
};

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

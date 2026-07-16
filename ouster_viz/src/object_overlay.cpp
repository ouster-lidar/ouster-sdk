/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include "object_overlay.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#include "common.h"
#include "ouster/core/pose_conversion.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
using ouster::sdk::core::Matrix4fR;

namespace ouster {
namespace sdk {
namespace viz {

namespace impl {

GLObjectOverlay::GLObjectOverlay(const ObjectOverlay& /*overlay*/)
    : v_lookup_tex(0), vbo(0), face_vbo(0) {
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &face_vbo);

    glGenTextures(1, &v_lookup_tex);
    glBindTexture(GL_TEXTURE_2D, v_lookup_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

GLObjectOverlay::~GLObjectOverlay() {
    glDeleteTextures(1, &v_lookup_tex);

    glDeleteBuffers(1, &face_vbo);
    glDeleteBuffers(1, &vbo);
}

void GLObjectOverlay::draw(const GlobalState& state, const WindowCtx& ctx,
                           const CameraData& /*camera*/, ObjectOverlay& overlay) {
    if (overlay.objects_changed_ || overlay.view_matrix_changed_) {
        constexpr std::array<float, 24> unit_cube_vertices = {
            -0.5f, -0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, -0.5f,
            -0.5f, -0.5f, 0.5f,  0.5f, -0.5f, 0.5f,  0.5f, 0.5f, 0.5f,  -0.5f, 0.5f, 0.5f};
        constexpr std::array<int, 24> cube_edge_indices = {
            0, 1, 1, 2, 2, 3, 3, 0,  // back
            4, 5, 5, 6, 6, 7, 7, 4,  // front
            0, 4, 1, 5, 2, 6, 3, 7   // connectors
        };
        constexpr std::array<int, 36> cube_triangle_indices = {
            0, 1, 2, 0, 2, 3,  // back
            4, 7, 6, 4, 6, 5,  // front
            0, 3, 7, 0, 7, 4,  // left
            1, 5, 6, 1, 6, 2,  // right
            0, 4, 5, 0, 5, 1,  // bottom
            2, 6, 7, 2, 7, 3   // top
        };

        std::vector<float> vertices;
        vertices.reserve(10 * cube_edge_indices.size() * overlay.cuboids_.size());
        std::vector<float> face_vertices;
        face_vertices.reserve(10 * overlay.cuboids_.size() * cube_triangle_indices.size() *
                              16);  // 16 subdivisions per triangle

        for (const auto& cuboid : overlay.cuboids_) {
            core::Matrix4dR model = cuboid.get_transform();
            Eigen::Vector3d center = {model(0, 3), model(1, 3), model(2, 3)};
            auto color = cuboid.get_rgba();

            // Edges
            for (int edge_idx : cube_edge_indices) {
                const size_t e_idx = static_cast<size_t>(edge_idx);
                Eigen::Vector4d point_vec(
                    static_cast<double>(unit_cube_vertices.at(e_idx * 3)),
                    static_cast<double>(unit_cube_vertices.at((e_idx * 3) + 1)),
                    static_cast<double>(unit_cube_vertices.at((e_idx * 3) + 2)), 1.0);
                point_vec = model * point_vec;
                // Position
                vertices.push_back(static_cast<float>(point_vec.x()));
                vertices.push_back(static_cast<float>(point_vec.y()));
                vertices.push_back(static_cast<float>(point_vec.z()));
                // Center
                vertices.push_back(static_cast<float>(center.x()));
                vertices.push_back(static_cast<float>(center.y()));
                vertices.push_back(static_cast<float>(center.z()));
                // Color (alpha 1 for edges)
                vertices.push_back(color[0]);
                vertices.push_back(color[1]);
                vertices.push_back(color[2]);
                vertices.push_back(1.0f);
            }

            // Faces
            auto push_face_vertex = [&](const Eigen::Vector3d& p_local) {
                Eigen::Vector4d point_vec(p_local.x(), p_local.y(), p_local.z(), 1.0);
                point_vec = model * point_vec;
                // Position
                face_vertices.push_back(static_cast<float>(point_vec.x()));
                face_vertices.push_back(static_cast<float>(point_vec.y()));
                face_vertices.push_back(static_cast<float>(point_vec.z()));
                // Center
                face_vertices.push_back(static_cast<float>(center.x()));
                face_vertices.push_back(static_cast<float>(center.y()));
                face_vertices.push_back(static_cast<float>(center.z()));
                // Color
                face_vertices.push_back(color[0]);
                face_vertices.push_back(color[1]);
                face_vertices.push_back(color[2]);
                face_vertices.push_back(0.3f);
            };

            const int subdivisions = 4;
            for (size_t tri_idx = 0; tri_idx < cube_triangle_indices.size(); tri_idx += 3) {
                const size_t t0_idx = static_cast<size_t>(cube_triangle_indices.at(tri_idx));
                const size_t t1_idx = static_cast<size_t>(cube_triangle_indices.at(tri_idx + 1));
                const size_t t2_idx = static_cast<size_t>(cube_triangle_indices.at(tri_idx + 2));

                Eigen::Vector3d point0(
                    static_cast<double>(unit_cube_vertices.at(t0_idx * 3)),
                    static_cast<double>(unit_cube_vertices.at((t0_idx * 3) + 1)),
                    static_cast<double>(unit_cube_vertices.at((t0_idx * 3) + 2)));
                Eigen::Vector3d point1(
                    static_cast<double>(unit_cube_vertices.at(t1_idx * 3)),
                    static_cast<double>(unit_cube_vertices.at((t1_idx * 3) + 1)),
                    static_cast<double>(unit_cube_vertices.at((t1_idx * 3) + 2)));
                Eigen::Vector3d point2(
                    static_cast<double>(unit_cube_vertices.at(t2_idx * 3)),
                    static_cast<double>(unit_cube_vertices.at((t2_idx * 3) + 1)),
                    static_cast<double>(unit_cube_vertices.at((t2_idx * 3) + 2)));

                for (int sub_m = 0; sub_m < subdivisions; sub_m++) {
                    for (int sub_n = 0; sub_n < subdivisions - sub_m; sub_n++) {
                        double t_val1 = static_cast<double>(sub_m) / subdivisions;
                        double t_val2 = static_cast<double>(sub_n) / subdivisions;
                        double t_val3 = static_cast<double>(sub_n + 1) / subdivisions;
                        double t_val4 = static_cast<double>(sub_m + 1) / subdivisions;

                        // Triangle 1
                        Eigen::Vector3d sp0 =
                            point0 * (1.0 - t_val1 - t_val2) + point1 * t_val1 + point2 * t_val2;
                        Eigen::Vector3d sp1 =
                            point0 * (1.0 - t_val4 - t_val2) + point1 * t_val4 + point2 * t_val2;
                        Eigen::Vector3d sp2 =
                            point0 * (1.0 - t_val1 - t_val3) + point1 * t_val1 + point2 * t_val3;

                        push_face_vertex(sp0);
                        push_face_vertex(sp1);
                        push_face_vertex(sp2);

                        // Triangle 2 (if not at the edge)
                        if (sub_n < subdivisions - sub_m - 1) {
                            Eigen::Vector3d sp3 = point0 * (1.0 - t_val4 - t_val3) +
                                                  point1 * t_val4 + point2 * t_val3;
                            push_face_vertex(sp1);
                            push_face_vertex(sp3);
                            push_face_vertex(sp2);
                        }
                    }
                }
            }
        }
        vertex_count_ = vertices.size() / 10;
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                     vertices.data(), GL_DYNAMIC_DRAW);

        face_vertex_count_ = face_vertices.size() / 10;
        glBindBuffer(GL_ARRAY_BUFFER, face_vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(face_vertices.size() * sizeof(float)),
                     face_vertices.data(), GL_DYNAMIC_DRAW);

        overlay.objects_changed_ = false;
    }

    if (overlay.sensor_info_changed_) {
        altitude_angles_.clear();
        for (double angle : overlay.sensor_info_.beam_altitude_angles) {
            altitude_angles_.push_back(static_cast<float>(angle * M_PI / 180.0));
        }
        std::sort(altitude_angles_.begin(), altitude_angles_.end());

        if (!altitude_angles_.empty()) {
            v_min_ = altitude_angles_.front();
            v_max_ = altitude_angles_.back();

            constexpr int lut_size = 1024;
            std::vector<float> lut(lut_size);
            for (int i = 0; i < lut_size; ++i) {
                float phi = v_min_ + ((v_max_ - v_min_) * static_cast<float>(i) / (lut_size - 1));
                auto it = std::lower_bound(altitude_angles_.begin(), altitude_angles_.end(), phi);
                if (it == altitude_angles_.begin()) {
                    lut[i] = 0.0f;
                } else if (it == altitude_angles_.end()) {
                    lut[i] = 1.0f;
                } else {
                    float phi1 = *(it - 1);
                    float phi2 = *it;
                    float interpolated = (phi - phi1) / (phi2 - phi1);
                    size_t idx1 = std::distance(altitude_angles_.begin(), it - 1);
                    lut[i] = (static_cast<float>(idx1) + interpolated) /
                             (static_cast<float>(altitude_angles_.size() - 1));
                }
            }
            glBindTexture(GL_TEXTURE_2D, v_lookup_tex);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, lut_size, 1, 0, GL_RED, GL_FLOAT, lut.data());
        }
        overlay.sensor_info_changed_ = false;
    }

    if (overlay.position_changed_) {
        x0_ = overlay.position_[0];
        x1_ = overlay.position_[1];
        y0_ = overlay.position_[2];
        y1_ = overlay.position_[3];
        hshift_ = overlay.hshift_;
        overlay.position_changed_ = false;
    }

    if (altitude_angles_.empty()) {
        return;
    }

    float aspect = static_cast<float>(window_aspect(ctx));
    float sc_x0 = (x0_ / aspect + hshift_ + 1.0f) / 2.0f * static_cast<float>(ctx.viewport_width);
    float sc_x1 = (x1_ / aspect + hshift_ + 1.0f) / 2.0f * static_cast<float>(ctx.viewport_width);
    float sc_y0 = (y0_ + 1.0f) / 2.0f * static_cast<float>(ctx.viewport_height);
    float sc_y1 = (y1_ + 1.0f) / 2.0f * static_cast<float>(ctx.viewport_height);

    int ix = static_cast<int>(std::round(std::min(sc_x0, sc_x1)));
    int iy = static_cast<int>(std::round(std::min(sc_y0, sc_y1)));
    int iw = static_cast<int>(std::round(std::abs(sc_x1 - sc_x0)));
    int ih = static_cast<int>(std::round(std::abs(sc_y1 - sc_y0)));

    glEnable(GL_SCISSOR_TEST);
    glScissor(ix, iy, iw, ih);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, v_lookup_tex);

    if (face_vertex_count_ > 0) {
        glBindVertexArray(state.face_vao);
        glUseProgram(state.face_program_id);

        // Uniforms
        Matrix4fR view = overlay.view_matrix_.cast<float>();
        glUniformMatrix4fv(state.face_view_id, 1, GL_TRUE, view.data());
        Matrix4fR identity = Matrix4fR::Identity();
        glUniformMatrix4fv(state.face_model_id, 1, GL_TRUE, identity.data());

        glUniform1f(state.face_fov_h_id,
                    2.0f * static_cast<float>(M_PI));  // 360 degrees
        glUniform1i(state.face_v_lookup_id, 1);
        glUniform1f(state.face_v_min_id, v_min_);
        glUniform1f(state.face_v_max_id, v_max_);
        glUniform1f(state.face_x0_id, x0_);
        glUniform1f(state.face_x1_id, x1_);
        glUniform1f(state.face_y0_id, y0_);
        glUniform1f(state.face_y1_id, y1_);
        glUniform1f(state.face_hshift_id, hshift_);
        glUniform1f(state.face_aspect_id, static_cast<float>(window_aspect(ctx)));
        glBindBuffer(GL_ARRAY_BUFFER, face_vbo);
        // NOLINTBEGIN(performance-no-int-to-ptr)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 10 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 10 * sizeof(float),
                              reinterpret_cast<void*>(static_cast<size_t>(3 * sizeof(float))));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 10 * sizeof(float),
                              reinterpret_cast<void*>(static_cast<size_t>(6 * sizeof(float))));
        // NOLINTEND(performance-no-int-to-ptr)
        glEnableVertexAttribArray(2);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(face_vertex_count_));
        glBindVertexArray(0);
    }

    if (vertex_count_ > 0) {
        glBindVertexArray(state.vao);
        glUseProgram(state.program_id);

        // Uniforms
        Matrix4fR view = overlay.view_matrix_.cast<float>();
        glUniformMatrix4fv(state.view_id, 1, GL_TRUE, view.data());
        Matrix4fR identity = Matrix4fR::Identity();
        glUniformMatrix4fv(state.model_id, 1, GL_TRUE, identity.data());

        glUniform1f(state.fov_h_id,
                    2.0f * static_cast<float>(M_PI));  // 360 degrees
        glUniform1i(state.v_lookup_id, 1);
        glUniform1f(state.v_min_id, v_min_);
        glUniform1f(state.v_max_id, v_max_);
        glUniform1f(state.x0_id, x0_);
        glUniform1f(state.x1_id, x1_);
        glUniform1f(state.y0_id, y0_);
        glUniform1f(state.y1_id, y1_);
        glUniform1f(state.hshift_id, hshift_);
        glUniform1f(state.aspect_id, static_cast<float>(window_aspect(ctx)));

        glEnableVertexAttribArray(state.pos_id);
        glEnableVertexAttribArray(state.center_id);
        glEnableVertexAttribArray(state.color_attr_id);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        // NOLINTBEGIN(performance-no-int-to-ptr)
        glVertexAttribPointer(state.pos_id, 3, GL_FLOAT, GL_FALSE, 10 * sizeof(float), nullptr);
        glVertexAttribPointer(state.center_id, 3, GL_FLOAT, GL_FALSE, 10 * sizeof(float),
                              reinterpret_cast<void*>(static_cast<uintptr_t>(3 * sizeof(float))));
        glVertexAttribPointer(state.color_attr_id, 4, GL_FLOAT, GL_FALSE, 10 * sizeof(float),
                              reinterpret_cast<void*>(static_cast<uintptr_t>(6 * sizeof(float))));
        // NOLINTEND(performance-no-int-to-ptr)

        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertex_count_));
    }

    glBindVertexArray(0);
    glDisable(GL_SCISSOR_TEST);
}

const std::string BORDER_VERTEX_SHADER_CODE = R"SHADER(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        uniform float u_x0;
        uniform float u_x1;
        uniform float u_y0;
        uniform float u_y1;
        uniform float u_hshift;
        uniform float u_aspect;
        uniform vec4 u_color;
        out vec4 fColor;
        out vec2 fPt;
        void main() {
            float screen_x = ((aPos.x + 1.0) / 2.0 * (u_x1 - u_x0) + u_x0) / u_aspect + u_hshift;
            float screen_y = (aPos.y + 1.0) / 2.0 * (u_y0 - u_y1) + u_y1;
            gl_Position = vec4(screen_x, screen_y, 0.0, 1.0);
            fColor = u_color;
            fPt = aPos;
        }
    )SHADER";

GLObjectOverlay::GlobalState::GlobalState()
    : vao{},
      face_vao{},
      program_id(load_shaders(PANORAMA_VERTEX_SHADER_CODE, PANORAMA_FRAGMENT_SHADER_CODE,
                              PANORAMA_GEOMETRY_SHADER_CODE)),
      pos_id(glGetAttribLocation(program_id, "aPos")),
      center_id(glGetAttribLocation(program_id, "aCenter")),
      color_attr_id(glGetAttribLocation(program_id, "aColor")),
      model_id(glGetUniformLocation(program_id, "u_model")),
      view_id(glGetUniformLocation(program_id, "u_view")),
      fov_h_id(glGetUniformLocation(program_id, "u_fov_h")),
      v_lookup_id(glGetUniformLocation(program_id, "u_v_lookup")),
      v_min_id(glGetUniformLocation(program_id, "u_v_min")),
      v_max_id(glGetUniformLocation(program_id, "u_v_max")),
      x0_id(glGetUniformLocation(program_id, "u_x0")),
      x1_id(glGetUniformLocation(program_id, "u_x1")),
      y0_id(glGetUniformLocation(program_id, "u_y0")),
      y1_id(glGetUniformLocation(program_id, "u_y1")),
      hshift_id(glGetUniformLocation(program_id, "u_hshift")),
      aspect_id(glGetUniformLocation(program_id, "u_aspect")),
      color_id(glGetUniformLocation(program_id, "u_color")),
      face_program_id(load_shaders(PANORAMA_VERTEX_SHADER_CODE, PANORAMA_FRAGMENT_SHADER_CODE,
                                   PANORAMA_FACE_GEOMETRY_SHADER_CODE)),
      face_pos_id(glGetAttribLocation(face_program_id, "aPos")),
      face_center_id(glGetAttribLocation(face_program_id, "aCenter")),
      face_color_attr_id(glGetAttribLocation(face_program_id, "aColor")),
      face_model_id(glGetUniformLocation(face_program_id, "u_model")),
      face_view_id(glGetUniformLocation(face_program_id, "u_view")),
      face_fov_h_id(glGetUniformLocation(face_program_id, "u_fov_h")),
      face_v_lookup_id(glGetUniformLocation(face_program_id, "u_v_lookup")),
      face_v_min_id(glGetUniformLocation(face_program_id, "u_v_min")),
      face_v_max_id(glGetUniformLocation(face_program_id, "u_v_max")),
      face_x0_id(glGetUniformLocation(face_program_id, "u_x0")),
      face_x1_id(glGetUniformLocation(face_program_id, "u_x1")),
      face_y0_id(glGetUniformLocation(face_program_id, "u_y0")),
      face_y1_id(glGetUniformLocation(face_program_id, "u_y1")),
      face_hshift_id(glGetUniformLocation(face_program_id, "u_hshift")),
      face_aspect_id(glGetUniformLocation(face_program_id, "u_aspect")) {
    glGenVertexArrays(1, &vao);

    glGenVertexArrays(1, &face_vao);
}

GLObjectOverlay::GlobalState::~GlobalState() {
    glDeleteProgram(program_id);
    glDeleteProgram(face_program_id);
    glDeleteVertexArrays(1, &vao);
    glDeleteVertexArrays(1, &face_vao);
}

void GLObjectOverlay::beginDraw(const GlobalState& state) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    glUseProgram(state.program_id);
}

void GLObjectOverlay::endDraw() {
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
}

}  // namespace impl

ObjectOverlay::ObjectOverlay() = default;

void ObjectOverlay::update_from(const ObjectOverlay& other) {
    bool position_changed = other.position_changed_ || position_changed_;
    bool objects_changed = other.objects_changed_ || objects_changed_;
    bool sensor_info_changed = other.sensor_info_changed_ || sensor_info_changed_;
    bool view_matrix_changed = other.view_matrix_changed_ || view_matrix_changed_;
    *this = other;
    this->position_changed_ = position_changed;
    this->objects_changed_ = objects_changed;
    this->sensor_info_changed_ = sensor_info_changed;
    this->view_matrix_changed_ = view_matrix_changed;
}

void ObjectOverlay::clear() {
    position_changed_ = false;
    objects_changed_ = false;
    sensor_info_changed_ = false;
    view_matrix_changed_ = false;
}

void ObjectOverlay::dirty() {
    position_changed_ = true;
    objects_changed_ = true;
    sensor_info_changed_ = true;
    view_matrix_changed_ = true;
}

void ObjectOverlay::set_cuboids(const std::vector<Cuboid>& cuboids) {
    cuboids_ = cuboids;
    objects_changed_ = true;
}

void ObjectOverlay::set_sensor_info(const ouster::sdk::core::SensorInfo& info) {
    sensor_info_ = info;
    sensor_info_changed_ = true;
}

void ObjectOverlay::set_view_matrix(const mat4d& mat) {
    view_matrix_ = mat;
    view_matrix_changed_ = true;
}

void ObjectOverlay::set_position(float x_min, float x_max, float y_min, float y_max) {
    position_ = {x_min, x_max, y_max, y_min};
    position_changed_ = true;
}

void ObjectOverlay::set_hshift(float hshift) {
    hshift_ = hshift;
    position_changed_ = true;
}

}  // namespace viz
}  // namespace sdk
}  // namespace ouster

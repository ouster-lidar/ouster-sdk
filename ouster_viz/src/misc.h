#pragma once

#include <GL/glew.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

#include "camera.h"
#include "common.h"
#include "gltext.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

/**
 * Class to deal with showing a set of rings on the ground as markers to help
 * visualize lidar range.
 */
class Rings {
    const size_t points_per_ring;
    std::vector<GLfloat> xyz;

    GLuint ring_program_id;
    GLuint ring_xyz_id;
    GLuint ring_proj_view_id;
    GLuint ring_range_id;
    GLuint xyz_buffer;

   public:
    int ring_size;
    bool enabled;

    /**
     * constructor
     *
     * @param points_per_ring_ number of points per ring, the more the rounder
     */
    Rings(const size_t points_per_ring_ = 512)
        : points_per_ring(points_per_ring_),
          xyz(points_per_ring * 3, 0),
          ring_size(1),
          enabled(true) {
        for (size_t i = 0; i < points_per_ring; i++) {
            const GLfloat theta = i * 2.0 * M_PI / points_per_ring;
            xyz[3 * i] = std::sin(theta);
            xyz[3 * i + 1] = std::cos(theta);
            xyz[3 * i + 2] = 0.0;
        }
    }

    /**
     * initializes shader program, vertex buffers and handles after OpenGL
     * context has been created
     */
    void initialize() {
        ring_program_id =
            load_shaders(ring_vertex_shader_code, ring_fragment_shader_code);
        ring_xyz_id = glGetAttribLocation(ring_program_id, "ring_xyz");
        ring_proj_view_id = glGetUniformLocation(ring_program_id, "proj_view");
        ring_range_id = glGetUniformLocation(ring_program_id, "ring_range");

        glGenBuffers(1, &xyz_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * points_per_ring * 3,
                     xyz.data(), GL_STATIC_DRAW);
    }

    /**
     * draws the rings from the point of view of the camera.
     * The rings are always centered on the camera's target.
     *
     * @param camera The camera
     */
    void draw(const Camera& camera) {
        glUseProgram(ring_program_id);
        const float radius = std::pow(10.0f, static_cast<GLfloat>(ring_size));
        mat4f camera_data = camera.proj_view().cast<GLfloat>();
        glUniformMatrix4fv(ring_proj_view_id, 1, GL_FALSE, camera_data.data());
        glEnableVertexAttribArray(ring_xyz_id);
        glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
        glVertexAttribPointer(ring_xyz_id,
                              3,         // size
                              GL_FLOAT,  // type
                              GL_FALSE,  // normalized?
                              0,         // stride
                              (void*)0   // array buffer offset
        );
        const GLfloat max_radius = 1000;
        const GLfloat max_rings = 2000;  // for performance
        for (GLfloat r = radius, rr = 0; r < max_radius && rr < max_rings;
             r += radius, rr += 1) {
            glUniform1f(ring_range_id, r);
            glDrawArrays(GL_LINE_LOOP, 0, points_per_ring);
        }
        glDisableVertexAttribArray(ring_xyz_id);
    };

    /**
     * destructor
     */
    ~Rings() { glDeleteProgram(ring_program_id); }
};

/**
 * Class to deal with showing cuboids
 */
class Cuboids {
    const std::array<GLfloat, 24> xyz;
    const std::array<GLubyte, 24> indices;
    const std::array<GLubyte, 24> edge_indices;
    std::vector<Cuboid> cuboids;

    GLuint cuboid_program_id;
    GLuint cuboid_xyz_id;
    GLuint cuboid_proj_view_id;
    GLuint cuboid_pose_id;
    GLuint cuboid_rgba_id;
    GLuint xyz_buffer;

    // We use two different coordinate system conventions in point viz. The
    // lidar points come in a "robotics" convention where the z-axis is pointing
    // up. The text loaded from GLText is in a standard opengl convention which
    // is a right-handed convention with x-right, y-down, and z pointing into
    // the screen. This transformation takes text loaded through GLText and puts
    // it in a reference frame using the robotics convention
    mat4f T_robotics_opengl;

   public:
    bool enabled;

    /**
     * constructor
     */
    Cuboids()
        : xyz{+0.5, +0.5, +0.5, +0.5, +0.5, -0.5, +0.5, -0.5,
              +0.5, +0.5, -0.5, -0.5, -0.5, +0.5, +0.5, -0.5,
              +0.5, -0.5, -0.5, -0.5, +0.5, -0.5, -0.5, -0.5},
          indices{0, 1, 3, 2, 6, 7, 5, 4, 2, 3, 7, 6,
                  4, 5, 1, 0, 0, 2, 6, 4, 5, 7, 3, 1},
          edge_indices{0, 1, 1, 3, 3, 2, 2, 0, 4, 5, 5, 7,
                       7, 6, 6, 4, 0, 4, 1, 5, 2, 6, 3, 7},
          enabled{true} {
        T_robotics_opengl << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    }

    /**
     * initializes shader program, vertex buffers and handles after OpenGL
     * context has been created
     */
    void initialize() {
        cuboid_program_id = load_shaders(cuboid_vertex_shader_code,
                                         cuboid_fragment_shader_code);
        cuboid_xyz_id = glGetAttribLocation(cuboid_program_id, "cuboid_xyz");
        cuboid_proj_view_id =
            glGetUniformLocation(cuboid_program_id, "proj_view");
        cuboid_pose_id = glGetUniformLocation(cuboid_program_id, "pose");
        cuboid_rgba_id = glGetUniformLocation(cuboid_program_id, "cuboid_rgba");

        glGenBuffers(1, &xyz_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 24, xyz.data(),
                     GL_STATIC_DRAW);
    }

    void clear() { cuboids.clear(); }
    void push(Cuboid&& c) { cuboids.push_back(std::move(c)); }

    /**
     * draws the cuboids from the point of view of the camera.
     * The cuboids are always centered on the camera's target.
     *
     * @param camera The camera
     */
    void draw(const Camera& camera) {
        glUseProgram(cuboid_program_id);
        const mat4f camera_data = camera.proj_view_target().cast<GLfloat>();
        glUniformMatrix4fv(cuboid_proj_view_id, 1, GL_FALSE,
                           camera_data.data());
        glEnableVertexAttribArray(cuboid_xyz_id);
        glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
        glVertexAttribPointer(cuboid_xyz_id,
                              3,         // size
                              GL_FLOAT,  // type
                              GL_FALSE,  // normalized?
                              0,         // stride
                              (void*)0   // array buffer offset
        );

        for (const auto& cuboid : cuboids) {
            const mat4f pose = cuboid.pose;
            glUniformMatrix4fv(cuboid_pose_id, 1, GL_FALSE, pose.data());
            glUniform4fv(cuboid_rgba_id, 1, cuboid.rgba.data());

            glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, indices.data());
            auto rgba = cuboid.rgba;
            rgba[3] = 1;
            glUniform4fv(cuboid_rgba_id, 1, rgba.data());
            glDrawElements(GL_LINES, 24, GL_UNSIGNED_BYTE, edge_indices.data());
        }

        gltBeginDraw();
        gltColor(1.0f, 1.0f, 1.0f, 1.0f);

        // draw text
        for (const auto& cuboid : cuboids) {
            if (cuboid.text.empty()) {
                continue;
            }
            auto text = gltCreateText();
            auto err = gltSetText(text, cuboid.text.c_str());
            if (err == GL_FALSE) {
                std::cerr << "error setting text" << std::endl;
            }

            // Contruct Transform to be applied to the model of the text such
            // that it's pointing at the camera to display information to the
            // user
            mat4f text_pose = cuboid.pose;

            // Text loaded from glText is in what we're calling the "OpenGL"
            // coordinate frame (x-right, y-down, z-forward). We need to convert
            // this into the format used by the rest of the visualizer so we can
            // apply the transformations in the correct coordinate frame
            mat4f T_text_world = text_pose * T_robotics_opengl.transpose();

            // Retrieve the offset and view from the current camera position
            mat4f offset = camera.get_offset_mat().cast<float>();
            mat4f view = camera.get_view().cast<float>();

            // Text shader interprets coordinate system in x-right, y-down,
            // z-forward (into the screen) coordinate system. This is different
            // than the one the point viz is using (x-right, y-up, z-backwards).
            // Since the view calculating is expecting coordinates wrt the
            // ouster format, convert back to that and apply the transformation
            // taking the points into the view reference frame.
            mat4f T_text_view = view * offset * T_text_world;

            // We need to remove the rotation component of the transform taking
            // points in the text frame and representing them in the view frame.
            // However since the viewport and text are in different coordinate
            // frames, this isn't simply setting the rotation as the identity.
            Eigen::Matrix3f R_no_rotation;

            // clang-format off
            R_no_rotation << 1,  0,  0,
                             0, -1,  0,
                             0,  0, -1;
            // clang-format on

            // Apply scaling to the text
            constexpr float SCALING_FACTOR = 0.01F;

            T_text_view.block<3, 3>(0, 0) = (SCALING_FACTOR * R_no_rotation);

            // We want the text slightly above the cuboid so we can read it. Up
            // in this coordinate frame is in the y-dimension
            T_text_view(1, 3) += 1;

            // Calculate the concatenation of the following transforms:
            // * Transform taking points in the model (text) into the world
            // frame
            // * Transform taking points in the world frame into the view frame
            // * Transform taking points in the view frame and projecting them
            //   between 0 and 1 in the z-dimension for clipping (projection
            //   frame)
            mat4f mvp = camera.get_proj().cast<float>() * T_text_view;
            gltDrawText(text, mvp.data());
            gltDeleteText(text);
        }

        gltEndDraw();

        glDisableVertexAttribArray(cuboid_xyz_id);
    };

    /**
     * destructor
     */
    ~Cuboids() { glDeleteProgram(cuboid_program_id); }
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster

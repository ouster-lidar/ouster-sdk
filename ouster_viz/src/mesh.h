#pragma once

#include <array>
#include <vector>

#include "camera.h"
#include "glfw.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

class GLMesh {
    static bool initialized;
    static GLuint mesh_vao;
    static GLuint mesh_program_id;
    static GLint mesh_xyz_id;
    static GLint mesh_normal_id;
    static GLint mesh_proj_view_id;
    static GLint mesh_face_rgba_id;
    static GLint mesh_edge_rgba_id;
    static GLint mesh_draw_edge_color_id;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit GLMesh(const Mesh& mesh);
    GLMesh(const std::vector<Vertex3f>& vertices,
           const std::vector<GLuint>& indices,
           const std::vector<GLuint>& edge_indices);
    GLMesh(const GLMesh&) = delete;
    GLMesh& operator=(const GLMesh&) = delete;
    GLMesh(GLMesh&&) = delete;
    GLMesh& operator=(GLMesh&&) = delete;
    ~GLMesh();

    /*
     * Draws the cuboids from the point of view of the camera
     */
    void draw(const WindowCtx& ctx, const CameraData& camera, Mesh& mesh);

    /*
     * Initializes shader program and handles
     */
    static void initialize();

    static void uninitialize();

    static void beginDraw();

    static void endDraw();

   protected:
    const std::vector<Vertex3f> vertices_{};
    const std::vector<GLuint> indices_{};
    const std::vector<GLuint> edge_indices_{};
    const int culling_{GL_BACK};
    Eigen::Matrix4d transform_;
    vec4f face_rgba_;
    vec4f edge_rgba_;

    GLuint xyz_buffer_{0};
    GLuint indices_buffer_{0};
    GLuint edge_indices_buffer_{0};
};

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

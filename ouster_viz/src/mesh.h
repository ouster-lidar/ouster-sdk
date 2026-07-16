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

class GLMesh {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit GLMesh(const Mesh& mesh);
    GLMesh(const std::shared_ptr<std::vector<Vertex3f>>& vertices,
           const std::shared_ptr<std::vector<GLuint>>& indices,
           const std::shared_ptr<std::vector<GLuint>>& edge_indices);
    GLMesh(const GLMesh&) = delete;
    GLMesh& operator=(const GLMesh&) = delete;
    GLMesh(GLMesh&&) = delete;
    GLMesh& operator=(GLMesh&&) = delete;
    ~GLMesh();

    struct GlobalState {
        GLuint mesh_vao;
        GLuint mesh_program_id;
        GLint mesh_xyz_id;
        GLint mesh_normal_id;
        GLint mesh_proj_view_id;
        GLint mesh_face_rgba_id;
        GLint mesh_edge_rgba_id;
        GLint mesh_draw_edge_color_id;

        GlobalState();
        ~GlobalState();

        // Make sure it cant be copied
        GlobalState(const GlobalState&) = delete;
        GlobalState& operator=(const GlobalState&) = delete;
    };

    /*
     * Draws the cuboids from the point of view of the camera
     */
    void draw(const GlobalState& state, const WindowCtx& ctx, const CameraData& camera, Mesh& mesh);

    static void beginDraw(const GlobalState& state);

    static void endDraw();

   protected:
    const std::shared_ptr<std::vector<Vertex3f>> vertices_{};
    const std::shared_ptr<std::vector<GLuint>> indices_{};
    const std::shared_ptr<std::vector<GLuint>> edge_indices_{};
    const int culling_{GL_BACK};
    core::Matrix4dR transform_;
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

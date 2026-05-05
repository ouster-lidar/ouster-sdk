#include "mesh.h"

#include <vector>

#include "common.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

bool GLMesh::initialized = false;
GLuint GLMesh::mesh_vao;
GLuint GLMesh::mesh_program_id;
GLint GLMesh::mesh_xyz_id;
GLint GLMesh::mesh_normal_id;
GLint GLMesh::mesh_proj_view_id;
GLint GLMesh::mesh_face_rgba_id;
GLint GLMesh::mesh_edge_rgba_id;
GLint GLMesh::mesh_draw_edge_color_id;

/*
 * Draws the mesh from the point of view of the camera.
 */
void GLMesh::draw(const WindowCtx& ctx, const CameraData& camera, Mesh& mesh) {
    (void)ctx;

    if (!GLMesh::initialized) {
        throw std::logic_error("GLMesh not initialized");
    }

    if (mesh.transform_changed_) {
        transform_ = Eigen::Map<const Eigen::Matrix4d>{mesh.transform_.data()};
        mesh.transform_changed_ = false;
    }
    glBindVertexArray(GLMesh::mesh_vao);

    face_rgba_ = mesh.face_rgba_;
    edge_rgba_ = mesh.edge_rgba_;

    glUniform1f(GLMesh::mesh_draw_edge_color_id, 0);
    glUniform4fv(GLMesh::mesh_face_rgba_id, 1, face_rgba_.data());

    // mesh pose (model matrix) is separate for some reason
    const Eigen::Matrix4f mvp =
        (camera.proj * camera.view * camera.target * transform_).cast<float>();
    glUniformMatrix4fv(GLMesh::mesh_proj_view_id, 1, GL_FALSE, mvp.data());
    glEnableVertexAttribArray(GLMesh::mesh_xyz_id);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer_);

    // TWS 20250724 - it's possible to use glVertexAttribFormat +
    // glVertexAttribBinding potentially avoiding a C-style or reinterpret cast
    // from int to GLvoid* but these are OpenGL 4.3 features and are probably
    // not present on macOS.
    glVertexAttribPointer(GLMesh::mesh_xyz_id,
                          3,                 // size
                          GL_FLOAT,          // type
                          GL_FALSE,          // normalized
                          sizeof(Vertex3f),  // stride
                          nullptr            // array buffer offset
    );
    glEnableVertexAttribArray(GLMesh::mesh_normal_id);

    // TWS 20250724 - it's possible to use glVertexAttribFormat +
    // glVertexAttribBinding potentially avoiding a C-style or reinterpret cast
    // from int to GLvoid* but these are OpenGL 4.3 features and are probably
    // not present on macOS.
    glVertexAttribPointer(
        GLMesh::mesh_normal_id,
        3,                        // size
        GL_FLOAT,                 // type
        GL_FALSE,                 // normalized
        sizeof(Vertex3f),         // stride
        reinterpret_cast<void*>(  // NOLINT(performance-no-int-to-ptr)
            sizeof(viz::Vertex3f::position)));

    // TODO[tws] support drawing triangles, not just edges
    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices_buffer_);
    // glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, (void*)0);

    // FIXME
    glUniform1f(GLMesh::mesh_draw_edge_color_id, 1);
    glUniform4fv(GLMesh::mesh_edge_rgba_id, 1, edge_rgba_.data());

    // Draw edges
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_indices_buffer_);
    glDrawElements(GL_LINES, static_cast<GLsizei>(edge_indices_.size()),
                   GL_UNSIGNED_INT, nullptr);

    glDisableVertexAttribArray(GLMesh::mesh_xyz_id);
    glDisableVertexAttribArray(GLMesh::mesh_normal_id);
    glBindVertexArray(0);
}

/**
 * initializes shader program and handles
 */
void GLMesh::initialize() {
    glGenVertexArrays(1, &GLMesh::mesh_vao);
    GLMesh::mesh_program_id =
        load_shaders(mesh_vertex_shader_code, mesh_fragment_shader_code);
    GLMesh::mesh_xyz_id = glGetAttribLocation(mesh_program_id, "mesh_xyz");
    GLMesh::mesh_normal_id =
        glGetAttribLocation(mesh_program_id, "mesh_normal");
    GLMesh::mesh_proj_view_id =
        glGetUniformLocation(mesh_program_id, "proj_view");
    GLMesh::mesh_face_rgba_id =
        glGetUniformLocation(mesh_program_id, "mesh_face_rgba");
    GLMesh::mesh_edge_rgba_id =
        glGetUniformLocation(mesh_program_id, "mesh_edge_rgba");
    GLMesh::mesh_draw_edge_color_id =
        glGetUniformLocation(mesh_program_id, "mesh_draw_edge_color");
    GLMesh::initialized = true;
}

void GLMesh::uninitialize() {
    GLMesh::initialized = false;
    glDeleteProgram(GLMesh::mesh_program_id);
    glDeleteVertexArrays(1, &GLMesh::mesh_vao);
}

void GLMesh::beginDraw() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);
    glUseProgram(mesh_program_id);
}

void GLMesh::endDraw() { glDisable(GL_BLEND); }

GLMesh::GLMesh(const Mesh& mesh)
    : GLMesh(mesh.vertices_, mesh.face_indices_, mesh.edge_indices_) {}

GLMesh::GLMesh(const std::vector<Vertex3f>& vertices,
               const std::vector<GLuint>& indices,
               const std::vector<GLuint>& edge_indices)
    : vertices_(vertices),
      indices_(indices),
      edge_indices_(edge_indices),
      transform_(Eigen::Matrix4d::Identity()),
      face_rgba_{1.0f, 0.0f, 0.0f, 0.1f},
      edge_rgba_{1.0f, 0.0f, 0.0f, 1.0f} {
    glGenBuffers(1, &xyz_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizei>(sizeof(Vertex3f) * vertices_.size()),
                 vertices_.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &indices_buffer_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizei>(indices_.size() * sizeof(GLuint)),
                 indices_.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &edge_indices_buffer_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_indices_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizei>(edge_indices_.size() * sizeof(GLuint)),
                 edge_indices_.data(), GL_STATIC_DRAW);
}

GLMesh::~GLMesh() {
    glDeleteBuffers(1, &xyz_buffer_);
    glDeleteBuffers(1, &indices_buffer_);
    glDeleteBuffers(1, &edge_indices_buffer_);
}

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster


#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

void Image::draw(Camera& cam) {
    // calling this before image has been resized causes flickering
    if (data.size() <= 0) return;

    glUseProgram(image_program_id);
    const GLuint vertex_id = glGetAttribLocation(image_program_id, "vertex");
    const GLuint uv_id = glGetAttribLocation(image_program_id, "vertex_uv");
    const GLuint image_id = glGetUniformLocation(image_program_id, "image");
    const GLuint mask_id = glGetUniformLocation(image_program_id, "mask");

    glUniform1i(image_id, 0);
    glUniform1i(mask_id, 1);

    glActiveTexture(GL_TEXTURE0);
    if (texture_changed) {
        load_texture(data.data(), width, height, image_texture_id, GL_RED,
                     GL_RED);
        texture_changed = false;
    }
    glBindTexture(GL_TEXTURE_2D, image_texture_id);

    glActiveTexture(GL_TEXTURE1);
    if (mask_changed) {
        load_texture(mask_data.data(), width, height, mask_texture_id, GL_RGBA,
                     GL_RGBA);
        mask_changed = false;
    }
    glBindTexture(GL_TEXTURE_2D, mask_texture_id);

    const GLfloat window_aspect_ratio =
        window_height / static_cast<GLfloat>(window_width);

    const GLfloat size =
        size_fraction / static_cast<GLfloat>(size_fraction_max);
    GLfloat image_w = size;
    GLfloat image_h = size * aspect_ratio / window_aspect_ratio;

    GLfloat x0, x1, y0, y1;

    if (window_aspect_ratio < aspect_ratio) {
        x0 = -1.0;
        y0 = image_h;
        x1 = -1.0 + image_w * 2;
        y1 = -image_h;
        cam.setOffset(-image_w, 0);
    } else {
        image_w = size * window_aspect_ratio / aspect_ratio;
        image_h = size;
        x0 = -image_w;
        y0 = 1.0;
        x1 = image_w;
        y1 = 1.0 - 2 * image_h;
        cam.setOffset(0, image_h);
    }

    const GLfloat vertices[] = {x0, y0, x0, y1, x1, y1, x1, y0};
    const GLfloat texcoords[] = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0};

    glEnableVertexAttribArray(vertex_id);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * 2, vertices,
                 GL_DYNAMIC_DRAW);
    glVertexAttribPointer(vertex_id,
                          2,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );
    glEnableVertexAttribArray(uv_id);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * 2, texcoords,
                 GL_DYNAMIC_DRAW);
    glVertexAttribPointer(uv_id,
                          2,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    GLubyte indices[] = {0, 1, 2, 0, 2, 3};
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE,
                   indices);  // 6 indices because 2 tris
    glDisableVertexAttribArray(vertex_id);
    glDisableVertexAttribArray(uv_id);
}
}  // namespace impl
}  // namespace viz
}  // namespace ouster

#include "cloud.h"

#include <GL/glew.h>

#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

void Cloud::setMapPose(const mat4d& mat) { map_pose = mat; }

/**
 * render the point cloud with the point of view of the Camera
 */
void Cloud::draw(const Camera& camera, const impl::CloudIds& ids,
                 const GLuint palette_texture_id) {
    const Eigen::Matrix<GLfloat, 4, 4> proj_view =
        (camera.proj_view_target() * map_pose).cast<GLfloat>();
    glUniformMatrix4fv(ids.proj_view_id, 1, GL_FALSE, proj_view.data());
    glUniformMatrix4fv(ids.model_id, 1, GL_FALSE, extrinsic_data.data());

    glUniform1i(ids.palette_id, 0);
    glUniform1i(ids.transformation_id, 1);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, palette_texture_id);

    glActiveTexture(GL_TEXTURE1);

    if (texture_changed) {
        impl::load_texture(transformation.data(), w, 4,
                           transformation_texture_id, GL_RGB32F);
        texture_changed = false;
    }
    if (mask_changed) {
        glBindBuffer(GL_ARRAY_BUFFER, buffers.mask_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * n * 4, mask_data.data(),
                     GL_STATIC_DRAW);
        mask_changed = false;
    }
    if (xyz_changed) {
        glBindBuffer(GL_ARRAY_BUFFER, buffers.xyz_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * n * 3, xyz_data.data(),
                     GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, buffers.off_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * n * 3, off_data.data(),
                     GL_STATIC_DRAW);
        xyz_changed = false;
    }
    glBindTexture(GL_TEXTURE_2D, transformation_texture_id);

    glEnableVertexAttribArray(ids.mask_id);
    glBindBuffer(GL_ARRAY_BUFFER, buffers.mask_buffer);

    glVertexAttribPointer(ids.mask_id,
                          4,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glEnableVertexAttribArray(ids.xyz_id);
    glBindBuffer(GL_ARRAY_BUFFER, buffers.xyz_buffer);

    glVertexAttribPointer(ids.xyz_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glEnableVertexAttribArray(ids.off_id);
    glBindBuffer(GL_ARRAY_BUFFER, buffers.off_buffer);
    glVertexAttribPointer(ids.off_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glEnableVertexAttribArray(ids.trans_index_id);
    glBindBuffer(GL_ARRAY_BUFFER, buffers.trans_index_buffer);
    glVertexAttribPointer(ids.trans_index_id,
                          1,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glEnableVertexAttribArray(ids.range_id);
    glBindBuffer(GL_ARRAY_BUFFER, buffers.range_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * n, range_data.data(),
                 GL_DYNAMIC_DRAW);
    glVertexAttribPointer(ids.range_id,
                          1,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );
    glEnableVertexAttribArray(ids.key_id);
    glBindBuffer(GL_ARRAY_BUFFER, buffers.key_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * n, key_data.data(),
                 GL_DYNAMIC_DRAW);
    glVertexAttribPointer(ids.key_id,
                          1,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glDrawArrays(GL_POINTS, 0, n);
    glDisableVertexAttribArray(ids.mask_id);
    glDisableVertexAttribArray(ids.xyz_id);
    glDisableVertexAttribArray(ids.off_id);
    glDisableVertexAttribArray(ids.trans_index_id);
    glDisableVertexAttribArray(ids.range_id);
    glDisableVertexAttribArray(ids.key_id);
}
}  // namespace impl
}  // namespace viz
}  // namespace ouster

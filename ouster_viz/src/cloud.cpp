/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 */

#include "cloud.h"

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "camera.h"
#include "common.h"
#include "glfw.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

struct CloudIds {
    GLuint xyz_id, off_id, range_id, key_id, mask_id, model_id, proj_view_id,
        palette_id, transformation_id, trans_index_id;
    CloudIds() {}

    /**
     * constructor
     * @param point_program_id handle to GLSL shader program compiled from
     * point_vertex_shader_code and point_fragment_shader_code
     */
    explicit CloudIds(GLuint point_program_id)
        : xyz_id(glGetAttribLocation(point_program_id, "xyz")),
          off_id(glGetAttribLocation(point_program_id, "offset")),
          range_id(glGetAttribLocation(point_program_id, "range")),
          key_id(glGetAttribLocation(point_program_id, "key")),
          mask_id(glGetAttribLocation(point_program_id, "mask")),
          model_id(glGetUniformLocation(point_program_id, "model")),
          proj_view_id(glGetUniformLocation(point_program_id, "proj_view")),
          palette_id(glGetUniformLocation(point_program_id, "palette")),
          transformation_id(
              glGetUniformLocation(point_program_id, "transformation")),
          trans_index_id(glGetAttribLocation(point_program_id, "trans_index")) {
    }
};

bool GLCloud::initialized = false;
GLfloat GLCloud::program_id;
CloudIds GLCloud::cloud_ids;

GLCloud::GLCloud(const Cloud& cloud) : point_size{cloud.point_size_} {
    if (!GLCloud::initialized)
        throw std::logic_error("GLCloud not initialized");

    // allocate gl object names
    glGenBuffers(1, &xyz_buffer);
    glGenBuffers(1, &off_buffer);
    glGenBuffers(1, &range_buffer);
    glGenBuffers(1, &key_buffer);
    glGenBuffers(1, &mask_buffer);
    glGenBuffers(1, &trans_index_buffer);
    glGenTextures(1, &transform_texture);
    glGenTextures(1, &palette_texture);

    // initialize trans_index_buffer
    std::vector<GLfloat> trans_index_buffer_data(cloud.n_);
    for (size_t i = 0; i < cloud.n_; i++) {
        trans_index_buffer_data[i] = ((i % cloud.w_) + 0.5) / (GLfloat)cloud.w_;
    }

    // initialize GL state
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.xyz_data_.size(),
                 cloud.xyz_data_.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, off_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.off_data_.size(),
                 cloud.off_data_.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, range_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.range_data_.size(),
                 cloud.range_data_.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, key_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.key_data_.size(),
                 cloud.key_data_.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mask_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.mask_data_.size(),
                 cloud.mask_data_.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, trans_index_buffer);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(GLfloat) * cloud.transform_data_.size(),
                 trans_index_buffer_data.data(), GL_STATIC_DRAW);

    load_texture(cloud.transform_data_.data(), cloud.w_, 4, transform_texture,
                 GL_RGB32F);

    load_texture(cloud.palette_data_.data(), cloud.palette_data_.size() / 3, 1,
                 palette_texture);
}

GLCloud::~GLCloud() {
    glDeleteBuffers(1, &xyz_buffer);
    glDeleteBuffers(1, &off_buffer);
    glDeleteBuffers(1, &range_buffer);
    glDeleteBuffers(1, &key_buffer);
    glDeleteBuffers(1, &mask_buffer);
    glDeleteBuffers(1, &trans_index_buffer);
    glDeleteTextures(1, &transform_texture);
    glDeleteTextures(1, &palette_texture);
}

/*
 * Render the point cloud with the point of view of the Camera
 */
void GLCloud::draw(const WindowCtx&, const CameraData& camera, Cloud& cloud) {
    if (cloud.point_size_changed_) {
        point_size = cloud.point_size_;
        cloud.point_size_changed_ = false;
    }
    glPointSize(point_size);

    if (cloud.pose_changed_) {
        map_pose = Eigen::Map<const Eigen::Matrix4d>{cloud.pose_.data()};
        cloud.pose_changed_ = false;
    }
    extrinsic = Eigen::Map<const Eigen::Matrix4d>{cloud.extrinsic_.data()}
                    .cast<float>();

    const Eigen::Matrix4f mvp =
        (camera.proj * camera.view * camera.target * map_pose).cast<float>();

    glUniformMatrix4fv(GLCloud::cloud_ids.model_id, 1, GL_FALSE,
                       extrinsic.data());
    glUniformMatrix4fv(GLCloud::cloud_ids.proj_view_id, 1, GL_FALSE,
                       mvp.data());

    glUniform1i(GLCloud::cloud_ids.palette_id, 0);
    glActiveTexture(GL_TEXTURE0);
    if (cloud.palette_changed_) {
        load_texture(cloud.palette_data_.data(), cloud.palette_data_.size() / 3,
                     1, palette_texture);
        cloud.palette_changed_ = false;
    }
    glBindTexture(GL_TEXTURE_2D, palette_texture);

    glUniform1i(GLCloud::cloud_ids.transformation_id, 1);
    glActiveTexture(GL_TEXTURE1);
    if (cloud.transform_changed_) {
        load_texture(cloud.transform_data_.data(), cloud.w_, 4,
                     transform_texture, GL_RGB32F);
        cloud.transform_changed_ = false;
    }
    glBindTexture(GL_TEXTURE_2D, transform_texture);

    if (cloud.mask_changed_) {
        glBindBuffer(GL_ARRAY_BUFFER, mask_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.mask_data_.size(),
                     cloud.mask_data_.data(), GL_STATIC_DRAW);
        cloud.mask_changed_ = false;
    }

    if (cloud.xyz_changed_) {
        glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.xyz_data_.size(),
                     cloud.xyz_data_.data(), GL_STATIC_DRAW);
        cloud.xyz_changed_ = false;
    }

    if (cloud.offset_changed_) {
        glBindBuffer(GL_ARRAY_BUFFER, off_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.off_data_.size(),
                     cloud.off_data_.data(), GL_STATIC_DRAW);
        cloud.offset_changed_ = false;
    }

    if (cloud.range_changed_) {
        glBindBuffer(GL_ARRAY_BUFFER, range_buffer);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * cloud.range_data_.size(),
                     cloud.range_data_.data(), GL_DYNAMIC_DRAW);
        cloud.range_changed_ = false;
    }

    if (cloud.key_changed_) {
        glBindBuffer(GL_ARRAY_BUFFER, key_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * cloud.key_data_.size(),
                     cloud.key_data_.data(), GL_DYNAMIC_DRAW);
        cloud.key_changed_ = false;
    }

    glEnableVertexAttribArray(GLCloud::cloud_ids.mask_id);
    glBindBuffer(GL_ARRAY_BUFFER, mask_buffer);
    glVertexAttribPointer(GLCloud::cloud_ids.mask_id,
                          4,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glEnableVertexAttribArray(GLCloud::cloud_ids.xyz_id);
    glBindBuffer(GL_ARRAY_BUFFER, xyz_buffer);
    glVertexAttribPointer(GLCloud::cloud_ids.xyz_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glEnableVertexAttribArray(GLCloud::cloud_ids.off_id);
    glBindBuffer(GL_ARRAY_BUFFER, off_buffer);
    glVertexAttribPointer(GLCloud::cloud_ids.off_id,
                          3,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glEnableVertexAttribArray(GLCloud::cloud_ids.trans_index_id);
    glBindBuffer(GL_ARRAY_BUFFER, trans_index_buffer);
    glVertexAttribPointer(GLCloud::cloud_ids.trans_index_id,
                          1,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glEnableVertexAttribArray(GLCloud::cloud_ids.range_id);
    glBindBuffer(GL_ARRAY_BUFFER, range_buffer);
    glVertexAttribPointer(GLCloud::cloud_ids.range_id,
                          1,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );
    glEnableVertexAttribArray(GLCloud::cloud_ids.key_id);
    glBindBuffer(GL_ARRAY_BUFFER, key_buffer);
    glVertexAttribPointer(GLCloud::cloud_ids.key_id,
                          1,         // size
                          GL_FLOAT,  // type
                          GL_FALSE,  // normalized?
                          0,         // stride
                          (void*)0   // array buffer offset
    );

    glDrawArrays(GL_POINTS, 0, cloud.n_);
    glDisableVertexAttribArray(GLCloud::cloud_ids.mask_id);
    glDisableVertexAttribArray(GLCloud::cloud_ids.xyz_id);
    glDisableVertexAttribArray(GLCloud::cloud_ids.off_id);
    glDisableVertexAttribArray(GLCloud::cloud_ids.trans_index_id);
    glDisableVertexAttribArray(GLCloud::cloud_ids.range_id);
    glDisableVertexAttribArray(GLCloud::cloud_ids.key_id);
}

void GLCloud::initialize() {
    GLCloud::program_id =
        load_shaders(point_vertex_shader_code, point_fragment_shader_code);
    GLCloud::cloud_ids = CloudIds(GLCloud::program_id);
    GLCloud::initialized = true;
}

void GLCloud::uninitialize() {
    GLCloud::initialized = false;
    glDeleteProgram(GLCloud::program_id);
}

void GLCloud::beginDraw() { glUseProgram(GLCloud::program_id); }

void GLCloud::endDraw() {}

}  // namespace impl
}  // namespace viz
}  // namespace ouster

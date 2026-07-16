/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 */

#include "cloud.h"

#include <Eigen/Core>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "camera.h"
#include "common.h"
#include "glfw.h"
#include "ouster/viz/point_viz.h"

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

GLCloud::GLCloud(const Cloud& cloud) : point_size_{cloud.point_size_} {
    // allocate gl object names
    glGenVertexArrays(1, &vao_);
}

GLCloud::~GLCloud() {
    glDeleteVertexArrays(1, &vao_);
}

namespace {

inline size_t ti_key(size_t cloud_size, size_t w) {
    return w + (cloud_size << sizeof(size_t) * 8 / 2);
}

}  // namespace

// update buffers common to all shaders
// currently this includes everything but mask and mono
void GLCloud::update_buffers(const GlobalState& state, const CameraData& camera, Cloud& cloud) {
    // transformation indices buffers cache by dimensions
    static std::mutex trans_indexes_mutex;
    static std::map<std::pair<uint32_t, size_t>, std::weak_ptr<BufferReference>> trans_indexes;

    auto trans_index_key = ti_key(cloud.n_, cloud.w_);
    if (!trans_index_buffer_ || trans_index_key != last_trans_index_key_) {
        //  check do we have a transformation indices in cache and if not
        //  generate
        std::shared_ptr<BufferReference> buffer;
        {
            std::unique_lock<std::mutex> lock(trans_indexes_mutex);
            auto it = trans_indexes.find({cloud.viz_instance_, trans_index_key});
            if (it != trans_indexes.end()) {
                buffer = it->second.lock();
            }
        }
        if (!buffer) {
            std::vector<float> trans_index_buffer_data(cloud.n_);
            for (size_t i = 0; i < cloud.n_; i++) {
                trans_index_buffer_data[i] =
                    static_cast<float>((static_cast<double>(i % cloud.w_) + 0.5) / cloud.w_);
            }
            buffer = std::make_shared<BufferReference>(std::move(trans_index_buffer_data));
            std::unique_lock<std::mutex> lock(trans_indexes_mutex);
            trans_indexes[{cloud.viz_instance_, trans_index_key}] = {buffer};
        }
        trans_index_buffer_ = buffer;
        last_trans_index_key_ = trans_index_key;

        glEnableVertexAttribArray(state.trans_index_id);
        glBindBuffer(GL_ARRAY_BUFFER, trans_index_buffer_->vbo(cloud.viz_instance_));
        glVertexAttribPointer(state.trans_index_id,
                              1,         // size
                              GL_FLOAT,  // type
                              GL_FALSE,  // normalized
                              0,         // stride
                              nullptr    // array buffer offset
        );
    }

    if (!cloud.mask_data_) {
        // load an empty mask from cache
        // TODO dont even render with a mask if no mask present
        if (!cached_mask_) {
            // deduplicate zero initialized masks by size (most frames do not
            // use a mask)
            static std::mutex mask_mutex;
            static std::map<std::pair<uint32_t, size_t>, std::weak_ptr<impl::BufferReference>>
                mask_cache;

            std::shared_ptr<impl::BufferReference> buf;
            {
                std::unique_lock<std::mutex> lock(mask_mutex);
                auto iter = mask_cache.find({cloud.viz_instance_, cloud.n_});
                if (iter != mask_cache.end()) {
                    buf = iter->second.lock();
                }
            }
            if (!buf) {
                // generate and cache
                std::vector<uint8_t> mask(4 * cloud.n_, 0);
                buf = std::make_shared<impl::BufferReference>(std::move(mask));
                std::unique_lock<std::mutex> lock(mask_mutex);
                mask_cache[{cloud.viz_instance_, cloud.n_}] = {buf};
            }
            cached_mask_ = buf;
        }
        cloud.mask_data_ = cached_mask_;
    }

    if (cloud.xyz_changed_) {
        glEnableVertexAttribArray(state.xyz_id);
        glBindBuffer(GL_ARRAY_BUFFER, cloud.xyz_data_->vbo(cloud.viz_instance_));
        glVertexAttribPointer(state.xyz_id,
                              3,         // size
                              GL_FLOAT,  // type
                              GL_FALSE,  // normalized
                              0,         // stride
                              nullptr    // array buffer offset
        );
        cloud.xyz_changed_ = false;
    }

    if (cloud.offset_changed_) {
        glEnableVertexAttribArray(state.off_id);
        glBindBuffer(GL_ARRAY_BUFFER, cloud.off_data_->vbo(cloud.viz_instance_));
        glVertexAttribPointer(state.off_id,
                              3,         // size
                              GL_FLOAT,  // type
                              GL_FALSE,  // normalized
                              0,         // stride
                              nullptr    // array buffer offset
        );
        cloud.offset_changed_ = false;
    }

    if (cloud.range_changed_) {
        glEnableVertexAttribArray(state.range_id);
        glBindBuffer(GL_ARRAY_BUFFER, cloud.range_data_->vbo(cloud.viz_instance_));
        glVertexAttribPointer(state.range_id,
                              1,         // size
                              GL_FLOAT,  // type
                              GL_FALSE,  // normalized
                              0,         // stride
                              nullptr    // array buffer offset
        );
        cloud.range_changed_ = false;
    }

    if (cloud.key_changed_) {
        mono_ = cloud.mono_;
        glEnableVertexAttribArray(state.key_id);
        glBindBuffer(GL_ARRAY_BUFFER, cloud.key_data_->vbo(cloud.viz_instance_));
        glVertexAttribPointer(state.key_id,
                              4,                 // size
                              GL_UNSIGNED_BYTE,  // type
                              GL_TRUE,           // normalized
                              0,                 // stride
                              nullptr            // array buffer offset
        );
        cloud.key_changed_ = false;
    }

    glUniform1i(state.palette_id, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, cloud.palette_data_->texture(cloud.viz_instance_));

    glUniform1i(state.transformation_id, 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, cloud.transform_data_->texture(cloud.viz_instance_));

    if (cloud.point_size_changed_) {
        point_size_ = cloud.point_size_;
        cloud.point_size_changed_ = false;
    }
    glPointSize(point_size_);

    if (cloud.pose_changed_) {
        map_pose_ = cloud.pose_;
        cloud.pose_changed_ = false;
    }
    extrinsic_ = cloud.extrinsic_.cast<float>();

    const Matrix4fR mvp = (camera.proj * camera.view * camera.target * map_pose_).cast<float>();

    glUniformMatrix4fv(state.model_id, 1, GL_TRUE, extrinsic_.data());
    glUniformMatrix4fv(state.proj_view_id, 1, GL_TRUE, mvp.data());
}

/*
 * Render the point cloud with the point of view of the Camera
 */
void GLCloud::draw(const GlobalState& state, const WindowCtx& /*unused*/, const CameraData& camera,
                   Cloud& cloud) {
    glBindVertexArray(vao_);

    update_buffers(state, camera, cloud);

    if (cloud.mask_changed_) {
        glEnableVertexAttribArray(state.mask_id);
        glBindBuffer(GL_ARRAY_BUFFER, cloud.mask_data_->vbo(cloud.viz_instance_));
        glVertexAttribPointer(state.mask_id,
                              4,                 // size
                              GL_UNSIGNED_BYTE,  // type
                              GL_TRUE,           // normalized
                              0,                 // stride
                              nullptr            // array buffer offset
        );
        cloud.mask_changed_ = false;
    }

    // put the shader into mono or rgb mode
    glUniform1i(state.mono_id, mono_ ? 1 : 0);

    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(cloud.n_));
    glBindVertexArray(0);
}

void GLCloud::draw_select(const GlobalState& state, const WindowCtx& /*unused*/,
                          const CameraData& camera, Cloud& cloud, uint32_t& start_index) {
    // can't select if there are no points
    if (cloud.n_ == 0) {
        return;
    }

    glBindVertexArray(vao_);

    update_buffers(state, camera, cloud);

    // make a mask that colors each point uniquely
    uint32_t n_pts = cloud.n_;

    std::vector<uint8_t> mask;
    mask.resize(static_cast<size_t>(n_pts) * 4);
    for (uint32_t i = 0; i < n_pts; i++) {
        uint32_t identifier = start_index + i;

        uint8_t r = identifier & 0xFF;
        uint8_t g = (identifier >> 8) & 0xFF;
        uint8_t b = (identifier >> 16) & 0xFF;
        size_t base = static_cast<size_t>(i) * 4;
        mask[base] = r;
        mask[base + 1] = g;
        mask[base + 2] = b;
        mask[base + 3] = 255;
    }
    start_index += n_pts;

    BufferReference mask_buffer(std::move(mask), true);

    glEnableVertexAttribArray(state.mask_id);
    glBindBuffer(GL_ARRAY_BUFFER, mask_buffer.vbo(cloud.viz_instance_));
    glVertexAttribPointer(state.mask_id,
                          4,                 // size
                          GL_UNSIGNED_BYTE,  // type
                          GL_TRUE,           // normalized
                          0,                 // stride
                          nullptr            // array buffer offset
    );
    // flag that we need to reconfigure the VAO next draw
    cloud.mask_changed_ = true;

    // put the shader into mono mode
    bool mono = false;
    glUniform1i(state.mono_id, mono ? 1 : 0);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(cloud.n_));
    glBindVertexArray(0);
}

GLCloud::GlobalState::GlobalState() {
    // NOLINTBEGIN(cppcoreguidelines-prefer-member-initializer)
    program_id = load_shaders(POINT_VERTEX_SHADER_CODE, POINT_FRAGMENT_SHADER_CODE);

    xyz_id = glGetAttribLocation(program_id, "xyz");
    off_id = glGetAttribLocation(program_id, "offset");
    range_id = glGetAttribLocation(program_id, "range");
    key_id = glGetAttribLocation(program_id, "vkey");
    mask_id = glGetAttribLocation(program_id, "vmask");
    model_id = glGetUniformLocation(program_id, "model");
    proj_view_id = glGetUniformLocation(program_id, "proj_view");
    mono_id = glGetUniformLocation(program_id, "mono");
    palette_id = glGetUniformLocation(program_id, "palette");
    transformation_id = glGetUniformLocation(program_id, "transformation");
    trans_index_id = glGetAttribLocation(program_id, "trans_index");
    // NOLINTEND(cppcoreguidelines-prefer-member-initializer)
}

GLCloud::GlobalState::~GlobalState() {
    glDeleteProgram(program_id);
}

void GLCloud::beginDraw(const GlobalState& state) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ZERO);
    glUseProgram(state.program_id);
}

void GLCloud::endDraw() {
    glDisable(GL_BLEND);
}

}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster

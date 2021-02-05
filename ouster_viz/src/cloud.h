#pragma once

#include <GL/glew.h>

#include <algorithm>
#include <array>
#include <memory>
#include <vector>

#include "camera.h"
#include "common.h"
#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

/**
 * Class to deal with point clouds.
 *
 * Each point cloud consists of n points with w poses. The ith point will be
 * transformed by the (i % w)th pose.
 *
 * For example for 2048 x 64 Ouster lidar point cloud, we may have w = 2048
 * poses and n = 2048 * 64 = 131072 points.
 *
 * We also keep track of the map pose and the extrinsic matrix as mentioned in
 * the comment in the point_vertex_shader_code (see common.h).
 *
 * The map_pose is used to efficiently transform the whole point cloud without
 * having to update all ~2048 poses.
 */
class Cloud {
    const size_t n;
    const size_t w;
    struct CloudBuffers {
        GLuint xyz_buffer;
        GLuint off_buffer;
        GLuint range_buffer;
        GLuint key_buffer;
        GLuint mask_buffer;
        GLuint trans_index_buffer;
    };
    CloudBuffers buffers;
    std::vector<GLfloat> range_data;
    std::vector<GLfloat> key_data;
    std::vector<GLfloat> mask_data;
    std::vector<GLfloat> xyz_data;
    std::vector<GLfloat> off_data;
    std::vector<GLfloat> transformation;  // set automatically by setColumnPoses
    mat4d map_pose;
    std::array<GLfloat, 16> extrinsic_data;  // row major
    GLuint transformation_texture_id;
    bool texture_changed;
    bool mask_changed;
    bool xyz_changed;

   public:
    /**
     * Set up the Cloud. Most of these arguments should correspond to CloudSetup
     *
     * @param xyz        Cartesian point clouds in the same format as
     *                   impl::Cloud::xyz, compatible with output of
     *                   make_xyz_lut.
     * @param off        Cartesian point clouds in the same format as
     *                   impl::Cloud::xyz, compatible with output of
     *                   make_xyz_lut.
     * @param n          Number of points, e.g. 64 * 2048 = 131072
     * @param w          Number of poses, e.g. 2048
     * @param extrinsic  Extrinsic calibration of sensor, col major
     */
    Cloud(const double* xyz, const double* off, const size_t n, const size_t w,
          const std::array<double, 16>& extrinsic)
        : n(n),
          w(w),
          range_data(n),
          key_data(n),
          mask_data(4 * n, 0),
          xyz_data(3 * n),
          off_data(3 * n),
          transformation(12 * w, 0),
          texture_changed(true),
          mask_changed(true),
          xyz_changed(true) {
        std::array<GLuint, 6> vertexbuffers;
        glGenBuffers(6, vertexbuffers.data());
        buffers =
            CloudBuffers{vertexbuffers[0], vertexbuffers[1], vertexbuffers[2],
                         vertexbuffers[3], vertexbuffers[4], vertexbuffers[5]};

        map_pose.setIdentity();
        std::vector<GLfloat> trans_index_buffer_data(n);
        for (size_t i = 0; i < n; i++) {
            trans_index_buffer_data[i] = ((i % w) + 0.5) / (GLfloat)w;
        }

        for (size_t v = 0; v < w; v++) {
            transformation[3 * v] = 1;
            transformation[3 * (v + w) + 1] = 1;
            transformation[3 * (v + 2 * w) + 2] = 1;
        }

        glBindBuffer(GL_ARRAY_BUFFER, buffers.trans_index_buffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * n,
                     trans_index_buffer_data.data(), GL_STATIC_DRAW);

        glGenTextures(1, &transformation_texture_id);

        setXYZ(xyz);
        setOffset(off);
        std::copy(extrinsic.begin(), extrinsic.end(), extrinsic_data.begin());
    }
    /**
     * set the range values
     *
     * @param x pointer to array of at least as many elements as there are
     *          points, representing the range of the points
     */
    void setRange(const uint32_t* x) {
        std::copy(x, x + n, range_data.begin());
    }

    /**
     * set the key values, used for colouring.
     *
     * @param x        pointer to array of at least as many elements as there
     *                 are points, preferably normalized between 0 and 1
     */
    void setKey(const double* x) { std::copy(x, x + n, key_data.begin()); }

    /**
     * set the RGBA mask values, used as an overlay on top of the key
     *
     * @param x        pointer to array of at least 4x as many elements as there
     *                 are points, preferably normalized between 0 and 1
     */
    void setMask(const GLfloat* x) {
        std::copy(x, x + 4 * n, mask_data.begin());
        mask_changed = true;
    }

    /**
     * set the XYZ values
     *
     * @param x        pointer to array of exactly 3n where n is number of
     *                 points, so that the xyz position of the ith point is
     *                 i, i + n, i + 2n
     */
    void setXYZ(const double* xyz) {
        for (size_t i = 0; i < n; i++) {
            for (size_t k = 0; k < 3; k++) {
                xyz_data[3 * i + k] = static_cast<GLfloat>(xyz[i + n * k]);
            }
        }
        xyz_changed = true;
    }

    /**
     * set the offset values
     *
     * @param x        pointer to array of exactly 3n where n is number of
     *                 points, so that the xyz position of the ith point is
     *                 i, i + n, i + 2n
     */
    void setOffset(const double* off) {
        for (size_t i = 0; i < n; i++) {
            for (size_t k = 0; k < 3; k++) {
                off_data[3 * i + k] = static_cast<GLfloat>(off[i + n * k]);
            }
        }
    }

    /**
     * set the ith point cloud map pose
     *
     * @param map_pose homogeneous transformation matrix of the pose
     */
    void setMapPose(const mat4d& mat);

    /**
     * Set the per-column poses, so that the point corresponding to the pixel
     * at row u, column v in the staggered lidar scan is transformed by the vth
     * pose, given as a homogeneous transformation matrix.
     *
     * @param rotation    array of rotation matrices, total size 9 * w, where
     *                    the vth rotation matrix is:
     *                    r[v],         r[w + v],     r[2 * w + v],
     *                    r[3 * w + v], r[4 * w + v], r[5 * w + v],
     *                    r[6 * w + v], r[7 * w + v], r[8 * w + v]
     * @param translation translation vector array, column major, where each row
     *                    is a translation vector. That is, the vth translation
     *                    is t[v], t[w + v], t[2 * w + v]
     */
    void setColumnPoses(const double* rotation, const double* translation) {
        for (size_t v = 0; v < w; v++) {
            for (size_t u = 0; u < 3; u++) {
                for (size_t rgb = 0; rgb < 3; rgb++) {
                    transformation[(u * w + v) * 3 + rgb] =
                        static_cast<GLfloat>(rotation[v + u * w + 3 * rgb * w]);
                }
            }
            for (size_t rgb = 0; rgb < 3; rgb++) {
                transformation[9 * w + 3 * v + rgb] =
                    static_cast<GLfloat>(translation[v + rgb * w]);
            }
        }
        texture_changed = true;
    }

    /**
     * render the point cloud with the point of view of the Camera
     *
     * @param camera   camera to view the point cloud
     * @param ids      handles to shader program attributes
     * @param palette_texture_id handle to colour map for visualization
     */
    void draw(const Camera& camera, const CloudIds& ids,
              const GLuint palette_texture_id);

    ~Cloud() {
        // todo(dllu) reference count and delete when the last copy of
        // Cloud is deleted glDeleteBuffers(1, &buffers.xyz_buffer);
    }
};

/**
 * Class that wraps around Cloud to store several double-buffered clouds.
 * This is used for visualizing accumulated point clouds for SLAM visualization
 *
 * Essentially this is a circular buffer of double buffered clouds.
 */
struct MultiCloud {
    std::vector<std::unique_ptr<DoubleBuffer<Cloud>>> clouds;
    size_t index = 0;  // which is the latest cloud to be written to
    bool enabled = true;

    /**
     * Constructor
     * generates a vector of several clouds and initializes them with the same
     * parameters
     *
     * @param setup the CloudSetup object with necessary parameters to
     *              initialize clouds
     */
    explicit MultiCloud(size_t accumulation, const double* xyz,
                        const double* off, const size_t n, const size_t w,
                        const std::array<double, 16>& extrinsic)
        : clouds(accumulation) {
        std::generate(clouds.begin(), clouds.end(), [&]() {
            return std::unique_ptr<DoubleBuffer<Cloud>>(
                new DoubleBuffer<Cloud>(xyz, off, n, w, extrinsic));
        });
    }

    /**
     * gets the latest writable cloud
     * @return unique ptr to writable cloud
     */
    std::unique_ptr<Cloud>& write() { return clouds[index]->write; }

    /**
     * calls draw on each of the clouds
     * @param Args arguments for Cloud::draw(...)
     */
    void draw(const Camera& camera, const impl::CloudIds& ids,
              const GLuint palette_texture_id) {
        for (auto& c : clouds) {
            c->read->draw(camera, ids, palette_texture_id);
        }
    }

    /**
     * swaps the double buffer for the latest cloud.
     */
    void swap() {
        clouds[index]->swap();
        index = (index + 1) % clouds.size();
    }
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster

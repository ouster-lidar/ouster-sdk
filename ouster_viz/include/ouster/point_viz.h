/**
 * @file
 * @brief Point cloud and image visualizer for Ouster Lidar using OpenGL
 */
#pragma once

#include <GL/glew.h>

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace ouster {
namespace viz {

using mat4f = Eigen::Matrix<GLfloat, 4, 4, Eigen::DontAlign>;
using mat4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;

/**
 * Struct to deal with a single cuboid.
 */
struct Cuboid {
    mat4f pose;
    std::array<GLfloat, 4> rgba;
};

/**
 * Parameters to help set up the visualizer
 */
struct CloudSetup {
    /**
     * Xyz is a non-owning raw pointer.
     *
     * It should be compatible with raw data from Eigen objects such as
     * ouster::LidarScan::Points or ouster::Points
     */
    const double* xyz;
    const double* off;
    size_t n;
    size_t w;
    std::array<double, 16> extrinsic;
    size_t accumulation;

    /**
     * Set up the input to the PointViz
     *
     * @param xyz cartesian point clouds in the same format as impl::Cloud::xyz,
     *        compatible with output of make_xyz_lut. Non-owning raw pointer
     * @param n number of points, e.g. 64 * 2048 = 131072
     * @param w number of poses, e.g. 2048
     * @param pextrinsic pointer or iterator to array of 16 elements containing
     *        extrinsic calibration of sensor, column major
     * @param accumulation number of point clouds to show, for the slam
     *        visualization
     *
     */
    CloudSetup(const double* xyz, const double* off, size_t n, size_t w,
               const double* pextrinsic, size_t accumulation = 1)
        : xyz(xyz), off(off), n(n), w(w), accumulation(accumulation) {
        std::copy(pextrinsic, pextrinsic + 16, extrinsic.begin());
    }
};

class PointViz {
    std::vector<CloudSetup> viz_setups;
    std::string name;

   public:
    struct impl;
    std::atomic_bool quit;
    std::unique_ptr<impl> pimpl;

    using idx = std::ptrdiff_t;

    /**
     * Constructor
     *
     * @param viz_setups for setting up point clouds, typically one per sensor
     * @param name name of the visualizer, shown in the title bar
     * @param fork whether or not to run the draw loop in a separate thread.
     *        PLEASE NOTE: the fork will not work on macOS because mac only
     *        supports running graphical stuff on the main thread.
     */
    PointViz(const std::vector<CloudSetup>& viz_setups, const std::string& name,
             const bool fork = true);

    /**
     * Main drawing loop, keeps drawing things while quit is false
     */
    void drawLoop();

    /**
     * Set the ith point cloud with range values
     *
     * @param cloud_id index of which cloud to update
     * @param x pointer to array of at least as many elements as there are
     *        points, representing the range of the points
     */
    void setRange(const idx cloud_id, const uint32_t* x);

    /**
     * Set the ith point cloud with key values, used for colouring.
     *
     * @param cloud_id index of which cloud to update
     * @param x pointer to array of at least as many elements as thered are
     *        points, preferably normalized between 0 and 1
     */
    void setKey(const idx cloud_id, const double* x);

    /**
     * Convenience function equivalent to setRange and setKey
     *
     * @param cloud_id index of which cloud to update
     * @param r range
     * @param k key
     */
    void setRangeAndKey(const idx cloud_id, const uint32_t* r,
                        const double* k) {
        setRange(cloud_id, r);
        setKey(cloud_id, k);
    }

    /**
     * Set the ith point cloud with new XYZ values
     *
     * @param cloud_id index of which cloud to update
     * @param x pointer to array of exactly 3n where n is number of points, so
     *        that the xyz position of the ith point is i, i + n, i + 2n
     *
     */
    void setXYZ(const idx cloud_id, const double* x);

    /**
     * Set the ith point cloud with new offset values
     *
     * @param cloud_id index of which cloud to update
     *
     * @param x pointer to array of exactly 3n where n is number of points, so
     *        that the xyz position of the ith point is i, i + n, i + 2n
     */
    void setOffset(const idx cloud_id, const double* x);

    /**
     * Set the ith point cloud map pose
     *
     * @param cloud_id index of which cloud to update
     * @param map_pose homogeneous transformation matrix of the pose
     */
    void setMapPose(const idx cloud_id, const mat4d& map_pose);

    /**
     * Set the poses for the ith point cloud
     *
     * @param cloud_id index of which cloud to update
     * @param rotation rotation matrix 9 by w column major
     * @param translation translation vector array column major
     */
    void setColumnPoses(const idx cloud_id, const double* rotation,
                        const double* translation);

    /**
     * Set the mask for the cloud
     *
     * @param cloud_id index of which cloud to update
     * @param mask pointer to array of at least 4x as many elements as there are
     *        points, in rgba format
     *
     */
    void setCloudMask(const idx cloud_id, const GLfloat* msk_data);

    /**
     * Swap double buffered cloud after writing all necessary data
     *
     * @param i index of which cloud to swap
     */
    void cloudSwap(size_t i);

    /**
     * Set the image
     *
     * @param image_data pointer to array of at least as many elements as there
     *        are pixels in the image, in row-major format
     */
    void setImage(const GLfloat* image_data);

    /**
     * Set the mask for the image
     *
     * @param mask pointer to array of at least 4x as many elements as there are
     *        pixels, in rgba format
     */
    void setImageMask(const GLfloat* msk_data);
    /**
     * Set the pixel dimensions of the image. disables the image if either is 0
     *
     * @param w pixel height of image
     * @param h pixel width of image
     */
    void resizeImage(size_t w, size_t h);

    /**
     * Set the display aspect ratio of the image
     *
     * @param a the aspect ratio, i.e. height divided by width
     */
    void setImageAspectRatio(GLfloat a);

    /**
     * Disable showing the image
     */
    void disableImage();

    /**
     * Swap the double buffered image after writing all necessary data
     */
    void imageSwap();

    /**
     * Add a cuboid
     *
     * @param a cuboid
     */
    void addCuboid(Cuboid&& cuboid);

    /**
     * Swap double buffered cuboids
     */
    void cuboidSwap();

    /**
     * Set camera target. the camera will smoothly move towards it
     *
     * @param target the new camera target
     */
    void setCameraTarget(const mat4d& target);

    /**
     * Set point cloud color palette. 
     *
     * @param palette the new palette to use
     * @param palette_size the number of colors in the new palette
     */
    void setPointCloudPalette(const float palette[][3], size_t palette_size);

    /**
     * Add custom key handler to the key. Multiple bindings can exist
     * for the same key.
     *
     * @param key which key to bind to, e.g. GLFW_KEY_A
     * @param f function to execute after key is pressed
     */
    void attachKeyHandler(int key, std::function<void()>&& f);

    /**
     * Execute custom key handlers for the key
     *
     * @param key which key to press, e.g. GLFW_KEY_A
     */
    void callKeyHandlers(const int key);

    /**
     * Destructor, joins the window thread
     */
    ~PointViz();

   private:
    /**
     * Sets up the visualizer, creating OpenGL context, compiling shaders,
     * allocating buffers, etc.
     */
    bool initialize();
};

}  // namespace viz
}  // namespace ouster

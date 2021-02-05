#pragma once

#include <GL/glew.h>

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>

#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
namespace impl {

/**
 * Note that most rotations are done in terms of integral decidegrees
 * (tenths of a degree). This is so that the camera will be able to
 * deterministically return to its original orientation if needed, without
 * accumulating floating point error, but at the same time having fine
 * enough subdivisions to have smooth user experience.
 *
 * For your convenience the user-defined literal _deg is defined so that
 * you can convert from degree literals to decidegree easily.
 */
using decidegree = int;
/**
 * convert integral degrees to decidegrees
 */
constexpr decidegree operator""_deg(unsigned long long int angle) {
    return angle * 10;
}
/**
 * convert and round floating point degrees to decidegrees
 */
constexpr decidegree operator""_deg(long double angle) {
    return static_cast<decidegree>(angle * 10);
}
/**
 * convert decidegrees to radians
 */
template <class T>
double decidegree2radian(T angle) {
    return static_cast<double>(M_PI * angle / 180.0_deg);
}

/**
 * camera class
 *
 * The camera class contains the folowing matrices:
 *
 * * Projection matrix proj, determined by view angle
 * * View matrix, relative to the target matrix, controlled by the user
 * * Offset matrix, controlled by the user, a pure translation
 * * Target matrix, usually close to car pose (identity if not using SLAM viz)
 *
 * The final camera matrix is proj * view * offset_mat * target.
 *
 * The camera class performs computations in double because for the SLAM viz,
 * the camera's position may have moved very far from the origin, so extra
 * precision is needed to prevent floating point error.
 */
class Camera {
    /*
     * ViewParameters holds the parameters relevant to the view matrix such as
     * field of view, dolly (distance from target).
     *
     * Likewise, some parameters like focal length and dolly distance are
     * integral.
     */
    struct ViewParameters {
        decidegree yaw;
        double auto_rotate_yaw;
        decidegree pitch;
        int log_focal_length = 0;  // zero means the focal length the same as
                                   // the diagonal size of the window
        int log_distance = 0;      // zero means 50 m
        static constexpr double log_distance_0 = 50.0;
    };

    ViewParameters vp;

    // Here we store time information. Simulation updates to the camera include:
    // 1) auto rotate
    // 2) smooth camera follow
    //
    // These must be simulated in a way that is decoupled from the framerate,
    // so we use real time. steady_clock is used because it is always smoothly
    // and monotonically increasing with the smallest tick interval.
    using viz_clock = std::chrono::steady_clock;
    using viz_time_point = std::chrono::time_point<viz_clock>;
    using viz_duration = std::chrono::nanoseconds;

    viz_time_point last_updated_time;
    viz_time_point rotation_start_time;
    const viz_duration simulation_period =
        std::chrono::duration_cast<viz_duration>(
            std::chrono::duration<double>(1.0 / 144));  // 144 Hz
    const viz_duration auto_rotate_period =
        std::chrono::duration_cast<viz_duration>(
            std::chrono::duration<double>(60.0));

    bool auto_rotate;
    bool orthographic;
    bool target_initialized;
    const double camera_smoothing = 0.005;

    GLfloat x_offset, y_offset;
    std::array<double, 3> offset_3d;

    mat4d view;
    mat4d proj;
    mat4d offset_mat;
    mat4d current_target;
    mat4d desired_target;

    std::mutex desired_target_mutex;

    void setView();

    void setProj();

    void tick();

    void simulate();

   public:
    Camera();

    void reset();

    void setFov(double diagonal_angle_rad);
    void setTarget(const mat4d& mat);

    void left(decidegree amount = 5_deg);
    void right(decidegree amount = 5_deg);
    void up(decidegree amount = 5_deg);
    void down(decidegree amount = 5_deg);

    void zoomIn(int amount = 5);
    void zoomOut(int amount = 5);

    void dollyIn(int amount = 5);
    void dollyOut(int amount = 5);

    void setOffset(GLfloat x, GLfloat y);

    /**
     * set the 3D offset of the point cloud when one middle clicks and drags/
     * @param x_amount pixel amount of dragging horizontally
     * @param y_amount pixel amount of dragging vertically
     */
    void changeOffset3d(double x_amount, double y_amount);

    void update();

    void setAutoRotateOn();
    void setAutoRotateOff();
    void toggleAutoRotate();
    void setOrthographicOn();
    void setOrthographicOff();
    void toggleOrthographic();
    mat4d proj_view() const;
    mat4d proj_view_target() const;
};

}  // namespace impl
}  // namespace viz
}  // namespace ouster

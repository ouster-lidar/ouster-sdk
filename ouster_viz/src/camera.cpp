#include "ouster/point_viz.h"
#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DONT_ALIGN

namespace ouster {
namespace viz {
namespace impl {

Camera::Camera()
    : last_updated_time(viz_clock::now()),
      rotation_start_time(viz_clock::now()),
      auto_rotate(false),
      orthographic(false),
      target_initialized(false),
      x_offset(0),
      y_offset(0),
      offset_3d{0, 0, 0},
      view(mat4d::Identity()),
      proj(mat4d::Identity()),
      offset_mat(mat4d::Identity()),
      current_target(mat4d::Identity()),
      desired_target(mat4d::Identity()) {
    vp.yaw = 0;
    vp.auto_rotate_yaw = 0;
    vp.pitch = -45_deg;
    vp.log_focal_length = 0;
    vp.log_distance = 0;
    update();
}

void Camera::reset() {
    offset_3d = {0, 0, 0};
    vp.yaw = 0;
    vp.auto_rotate_yaw = 0;
    vp.pitch = -45_deg;
    vp.log_focal_length = 0;
    vp.log_distance = 0;
    update();
}

void Camera::setFov(double diagonal_angle_rad) {
    diagonal_angle_rad = std::min(diagonal_angle_rad, M_PI);
    diagonal_angle_rad = std::max(diagonal_angle_rad, 0.0);
    vp.log_focal_length =
        std::log(std::round(std::tan(diagonal_angle_rad / 2.0)));
    update();
}

void Camera::setTarget(const mat4d& mat) {
    std::lock_guard<std::mutex> guard(desired_target_mutex);
    const mat4d mat_inv = mat.inverse();
    desired_target = mat_inv;
    if (!target_initialized) {
        current_target = mat_inv;
        target_initialized = true;
    }
}

void Camera::left(decidegree amount) {
    vp.yaw = (vp.yaw + amount) % 360_deg;
    update();
}

void Camera::right(decidegree amount) {
    vp.yaw = (vp.yaw + 360_deg - amount) % 360_deg;
    update();
}

void Camera::up(decidegree amount) {
    vp.pitch = std::min(0, vp.pitch + amount);
    update();
}
void Camera::down(decidegree amount) {
    vp.pitch = std::max(-180_deg, vp.pitch - amount);
    update();
}
void Camera::zoomIn(int amount) {
    vp.log_focal_length = std::min(500, vp.log_focal_length + amount);
    update();
}
void Camera::zoomOut(int amount) {
    vp.log_focal_length = std::max(-500, vp.log_focal_length - amount);
    update();
}
void Camera::dollyIn(int amount) {
    vp.log_distance = std::max(-500, vp.log_distance - amount);
    update();
}
void Camera::dollyOut(int amount) {
    vp.log_distance = std::min(500, vp.log_distance + amount);
    update();
}

void Camera::setOffset(GLfloat x, GLfloat y) {
    x_offset = x;
    y_offset = y;
}

void Camera::changeOffset3d(double x, double y) {
    Eigen::Matrix<double, 2, 1, Eigen::DontAlign> v;
    v(0) = x;
    v(1) = -y;  // OpenGL y goes from top to bottom

    // convert from pixels to fractions of window size
    const double window_diagonal =
        std::sqrt(window_width * window_width + window_height * window_height);
    v *= 2.0 / window_diagonal;

    // convert from window coordinates to actual size
    const double view_distance =
        ViewParameters::log_distance_0 * std::exp(vp.log_distance * 0.01);
    const double focal_length = std::exp(vp.log_focal_length * 0.01);
    v *= view_distance * focal_length;

    // calculate 3d offset in the direction of the camera view
    Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>(
        offset_3d.data()) += view.block<2, 3>(0, 0).transpose() * v;
}

void Camera::update() {
    offset_mat(0, 3) = offset_3d[0];
    offset_mat(1, 3) = offset_3d[1];
    offset_mat(2, 3) = offset_3d[2];

    simulate();
    setView();
    setProj();
}

void Camera::setAutoRotateOn() {
    auto_rotate = true;
    rotation_start_time = viz_clock::now();
}

void Camera::setAutoRotateOff() {
    auto_rotate = false;
    vp.auto_rotate_yaw = 0;
}

void Camera::toggleAutoRotate() {
    if (!auto_rotate)
        setAutoRotateOn();
    else
        setAutoRotateOff();
}

void Camera::setOrthographicOn() { orthographic = true; }

void Camera::setOrthographicOff() { orthographic = false; }

void Camera::toggleOrthographic() {
    if (!orthographic)
        setOrthographicOn();
    else
        setOrthographicOff();
}

mat4d Camera::proj_view() const { return proj * view * offset_mat; }

mat4d Camera::proj_view_target() const {
    return proj * view * offset_mat * current_target;
}

void Camera::setView() {
    view.block<3, 3>(0, 0) =
        (Eigen::AngleAxis<double>(
             decidegree2radian(vp.pitch),
             Eigen::Matrix<double, 3, 1, Eigen::DontAlign>::UnitX()) *
         Eigen::AngleAxis<double>(
             decidegree2radian(vp.auto_rotate_yaw + vp.yaw),
             Eigen::Matrix<double, 3, 1, Eigen::DontAlign>::UnitZ()))
            .toRotationMatrix();
    view(2, 3) =
        -ViewParameters::log_distance_0 * std::exp(vp.log_distance * 0.01);
}

void Camera::setProj() {
    const double window_diagonal =
        std::sqrt(window_width * window_width + window_height * window_height);
    const double view_distance =
        100 * ViewParameters::log_distance_0 * std::exp(vp.log_distance * 0.01);
    const double focal_length = std::exp(vp.log_focal_length * 0.01);
    const double focal_length_multiplier = focal_length * window_diagonal;
    if (!orthographic) {
        // from http://www.songho.ca/opengl/gl_projectionmatrix.html
        // please note that Windows has defined far and near to be nothing
        // hence I had to append dist to these variables.
        const double far_dist = std::min(1000.0, view_distance);
        const double near_dist = 0.1;
        proj.setZero();
        proj(0, 0) = focal_length_multiplier / window_width;
        proj(0, 2) = x_offset;
        proj(1, 1) = focal_length_multiplier / window_height;
        proj(1, 2) = y_offset;
        proj(2, 2) = -(far_dist + near_dist) / (far_dist - near_dist);
        proj(2, 3) = -2 * far_dist * near_dist / (far_dist - near_dist);
        proj(3, 2) = -1;
    } else {
        proj.setZero();
        proj(0, 0) =
            focal_length_multiplier / window_width / (view_distance / 100);
        proj(1, 1) =
            focal_length_multiplier / window_height / (view_distance / 100);
        proj(0, 3) = -x_offset;
        proj(1, 3) = -y_offset;
        proj(2, 2) = -1e-3;
        proj(3, 3) = 1;
    }
}

void Camera::tick() {
    /*
     * current_target * pow(delta, -10) = desired_target;
     */
    if (!target_initialized) return;
    mat4d delta = current_target.inverse() * desired_target;
    Eigen::AngleAxis<double> aa;
    aa.fromRotationMatrix(delta.block<3, 3>(0, 0));
    aa.angle() *= camera_smoothing;
    delta.block<3, 3>(0, 0) = aa.toRotationMatrix();
    delta.block<3, 1>(0, 3) *= camera_smoothing;
    current_target = current_target * delta;
}

void Camera::simulate() {
    std::lock_guard<std::mutex> guard(desired_target_mutex);
    const viz_time_point curr_time = viz_clock::now();
    if (auto_rotate) {
        const viz_duration time_yaw = std::chrono::duration_cast<viz_duration>(
            curr_time - rotation_start_time);
        vp.auto_rotate_yaw = (time_yaw.count() % auto_rotate_period.count()) *
                             360.0_deg / auto_rotate_period.count();
    }

    while (last_updated_time + simulation_period < curr_time) {
        tick();
        last_updated_time += simulation_period;
    }
}
}  // namespace impl
}  // namespace viz
}  // namespace ouster

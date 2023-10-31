/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 */

#include "camera.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "ouster/point_viz.h"

namespace ouster {
namespace viz {

using Eigen::Vector3d;
using Translation3d = Eigen::Translation<double, 3>;

namespace {

/* initial distance 50 m */
constexpr double log_distance_0 = 50.0;

/* distance min/max bounds */
constexpr double log_distance_min = -500.0;
constexpr double log_distance_max = 500.0;

/*
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

/*
 * Convert integral degrees to decidegrees
 */
constexpr decidegree operator""_deg(unsigned long long int angle) {
    return angle * 10;
}

/*
 * Convert and round floating point degrees to decidegrees
 */
constexpr decidegree operator""_deg(long double angle) {
    return static_cast<decidegree>(angle * 10);
}

/*
 * Convert decidegrees to radians
 */
template <class T>
double decidegree2radian(T angle) {
    return static_cast<double>(M_PI * angle / 180.0_deg);
}

int dd(float degrees) { return std::lround(degrees * 10); }

}  // namespace

/*
 * The camera class computes the folowing matrices:
 *
 * - Projection matrix proj, determined by view angle
 * - View matrix, relative to the target matrix, controlled by the user
 * - Target matrix, usually close to car pose (identity if not using SLAM viz)
 *
 * The final camera matrix is proj * view * target.
 *
 * The camera class performs computations in double because for the SLAM viz,
 * the camera's position may have moved very far from the origin, so extra
 * precision is needed to prevent floating point error.
 */
Camera::Camera()
    : target_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
      view_offset_{0, 0, 0},
      yaw_{0},
      pitch_{-45_deg},
      log_distance_{0},
      orthographic_{false},
      fov_{90_deg},
      proj_offset_x_{0},
      proj_offset_y_{0} {}

double Camera::view_distance() const {
    return log_distance_0 * std::exp(log_distance_ * 0.01);
}

void Camera::reset() {
    view_offset_ = {0, 0, 0};
    yaw_ = 0;
    pitch_ = -45_deg;
    log_distance_ = 0;
    fov_ = 90_deg;
}

void Camera::birds_eye_view() {
    view_offset_ = {0, 0, 0};
    yaw_ = 0;
    pitch_ = 0_deg;
    log_distance_ = 200.0;
    fov_ = 90_deg;
}

// left positive, right negative
void Camera::yaw(float degrees) {
    yaw_ = (yaw_ + 360_deg + dd(degrees)) % 360_deg;
}

void Camera::set_yaw(float degrees) {
    yaw_ = (360_deg + dd(degrees)) % 360_deg;
}

float Camera::get_yaw() const { return static_cast<float>(yaw_) / 10.0; }

// down positive, up negative
void Camera::pitch(float degrees) {
    pitch_ = std::max(-180_deg, std::min(0, pitch_ + dd(degrees)));
}

void Camera::set_pitch(float degrees) {
    pitch_ = std::max(-180_deg, std::min(0, dd(degrees)));
}

float Camera::get_pitch() const { return static_cast<float>(pitch_) / 10.0; }

// in is positive, out is negative
void Camera::dolly(double amount) {
    log_distance_ = std::max(
        log_distance_min, std::min(log_distance_max, log_distance_ - amount));
}

void Camera::set_dolly(double log_distance) {
    log_distance_ =
        std::max(log_distance_min, std::min(log_distance_max, log_distance));
}

double Camera::get_dolly() const { return log_distance_; }

void Camera::dolly_xy(double x, double y) {
    // OpenGL y goes from top to bottom
    Vector3d v{x, -y, 0};
    // convert from window coordinates to actual size
    double scale = std::tan(M_PI / 3600.0 * fov_);
    v *= view_distance() * scale;
    // apply 3d offset in the direction of the camera view
    auto rot =
        (Eigen::AngleAxisd{decidegree2radian(pitch_), Vector3d::UnitX()} *
         Eigen::AngleAxisd{decidegree2radian(yaw_), Vector3d::UnitZ()})
            .matrix();
    Eigen::Map<Eigen::Vector3d>{view_offset_.data()} += rot.transpose() * v;
}

void Camera::set_view_offset(const vec3d& view_offset) {
    view_offset_ = view_offset;
}

vec3d Camera::get_view_offset() const { return view_offset_; }

void Camera::set_fov(float degrees) {
    fov_ = std::max(0.0_deg, std::min(360.0_deg, dd(degrees)));
}

float Camera::get_fov() const { return static_cast<float>(fov_ / 10.0); }

void Camera::set_orthographic(bool b) { orthographic_ = b; }
bool Camera::is_orthographic() const { return orthographic_; }

void Camera::set_proj_offset(float x, float y) {
    proj_offset_x_ = x, proj_offset_y_ = y;
}

vec2d Camera::get_proj_offset() const {
    return {proj_offset_x_, proj_offset_y_};
}

void Camera::set_target(const mat4d& target) { target_ = target; }
mat4d Camera::get_target() const { return target_; }

/*
 * Calculate camera matrices.
 *
 * View is relative to target 'look at' point. Current parametrization is:
 * - view_offset: translation to support dollying left/right
 * - yaw/pitch: camera rotation
 * - view_distance: z offset for dollying in/out
 *
 * view_offset is separate from target so we can dolly the camera while still
 * drawing some things (like distance rings) relative to target. It's applied
 * before yaw/pitch so that the camera orbits around view_offset * target.
 *
 * Unlike the usual opengl projection matrix function, this uses a diagonal fov.
 * For the orthographic projection, the fov and camera distance are used to
 * scale the viewing volume to make zoom and dolly work more or less
 * intuitively.
 *
 * reference:
 * https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix
 */
impl::CameraData Camera::matrices(double aspect) const {
    // calculate view matrix
    Eigen::Matrix4d view =
        (Translation3d{Vector3d::UnitZ() * -view_distance()} *
         Eigen::AngleAxisd{decidegree2radian(pitch_), Vector3d::UnitX()} *
         Eigen::AngleAxisd{decidegree2radian(yaw_), Vector3d::UnitZ()} *
         Translation3d{Eigen::Map<const Eigen::Vector3d>{view_offset_.data()}})
            .matrix();

    // calculate projection matrix
    const double scale = std::tan(M_PI / 3600.0 * fov_);
    const double view_dist = view_distance();
    const double far_dist = std::min(10000.0, 100 * view_dist);
    const double near_dist = 0.1;

    // for diagonal fov, use ratio of each dimension to diagonal
    double a = std::atan(aspect);
    double aspect_w = std::sin(a);
    double aspect_h = std::cos(a);

    Eigen::Matrix4d proj = Eigen::Matrix4d::Zero();
    if (orthographic_) {
        proj(0, 0) = 1 / (aspect_w * scale * view_dist);
        proj(0, 3) = -proj_offset_x_;
        proj(1, 1) = 1 / (aspect_h * scale * view_dist);
        proj(1, 3) = -proj_offset_y_;
        proj(2, 2) = -2 / (far_dist - near_dist);
        proj(2, 3) = -(far_dist + near_dist) / (far_dist - near_dist);
        proj(3, 3) = 1;
    } else {
        proj(0, 0) = 1 / (aspect_w * scale);
        proj(0, 2) = proj_offset_x_;
        proj(1, 1) = 1 / (aspect_h * scale);
        proj(1, 2) = proj_offset_y_;
        proj(2, 2) = -(far_dist + near_dist) / (far_dist - near_dist);
        proj(2, 3) = -2 * far_dist * near_dist / (far_dist - near_dist);
        proj(3, 2) = -1;
    }

    return {proj, view, Eigen::Map<const Eigen::Matrix4d>{target_.data()}};
}

}  // namespace viz
}  // namespace ouster

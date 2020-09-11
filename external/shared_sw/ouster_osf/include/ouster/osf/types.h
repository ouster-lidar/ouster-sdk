#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"

namespace ouster {
namespace OSF {

namespace os = ouster::sensor;

static const std::vector<double> DEFAULT_EXTRINSICS = {1, 0, 0, 0, 0, 1, 0, 0,
                                                       0, 0, 1, 0, 0, 0, 0, 1};

struct sensor {
    uint32_t id;  // only first 8 bit used in OSF files
    os::sensor_info meta;

    sensor() : id(0), meta(os::default_sensor_info(os::MODE_1024x10)) {}

    explicit sensor(const uint32_t sensor_id)
        : id(sensor_id), meta(os::default_sensor_info(os::MODE_1024x10)) {}

    sensor(const uint32_t sensor_id, const os::sensor_info& info)
        : id(sensor_id), meta(info) {}
};

struct pose {
    Eigen::Quaternionf orientation;
    Eigen::Translation<double, 3> position;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    pose(double R_w, double R_x, double R_y, double R_z, double T_x, double T_y,
         double T_z)
        : orientation(Eigen::Quaternionf(R_w, R_x, R_y, R_z)),
          position(Eigen::Translation<double, 3>(T_x, T_y, T_z)) {}
    pose() : pose(1, 0, 0, 0, 0, 0, 0) {}

    std::string to_string() const {
        std::stringstream ss;
        ss << "pos: [" << position.vector().transpose() << "]";
        ss << ", rot: [" << orientation.coeffs().transpose() << "]";
        return ss.str();
    }
};

// TODO[pb]: Remove and substiture with Trajectory usage in code.
using osf_poses = std::vector<pose, Eigen::aligned_allocator<pose>>;

// Trajectory holder, used to reconstruct data back from OsfFile.
using Trajectory = osf_poses;

// Gps message store
struct Gps {
    Gps(double latitude, double epy, double longitude, double epx,
        double altitude, double epv, double ept, double speed, double eps,
        double track, double epd, double climb, double epc)
        : latitude(latitude),
          epy(epy),
          longitude(longitude),
          epx(epx),
          altitude(altitude),
          epv(epv),
          ept(ept),
          speed(speed),
          eps(eps),
          track(track),
          epd(epd),
          climb(climb),
          epc(epc) {}

    Gps()
        : latitude(0),
          epy(0),
          longitude(0),
          epx(0),
          altitude(0),
          epv(0),
          ept(0),
          speed(0),
          eps(0),
          track(0),
          epd(0),
          climb(0),
          epc(0) {}

    double latitude;
    double epy;
    double longitude;
    double epx;
    double altitude;
    double epv;
    double ept;
    double speed;
    double eps;
    double track;
    double epd;
    double climb;
    double epc;
};

// Imu message data store
struct Imu {
    std::vector<double> angular_vel;
    std::vector<double> linear_accel;
    std::vector<std::chrono::nanoseconds> ts;
};

// Common timestamp for all time in ouster::OSF
using ts_t = std::chrono::nanoseconds;

using sensors_map = std::map<uint32_t, std::shared_ptr<sensor>>;

}  // namespace OSF
}  // namespace ouster

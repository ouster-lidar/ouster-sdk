#pragma once

#include <ouster/lidar_scan.h>
#include <ouster/lidar_scan_set.h>
#include <ouster/types.h>
#include <ouster/visibility.h>
#include <ouster/xyzlut.h>

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <nonstd/optional.hpp>
#include <string>
#include <vector>

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class DeskewMethod
 * @brief Abstract interface for LiDAR scan deskewing strategies.
 *
 * A DeskewMethod encapsulates the logic required to correct (deskew) the scan
 * poses for each LidarScan within the LidarScanSet.
 *
 * Typical usage pattern:
 *  - Construct a concrete deskew method passing SensorInfo per-sensor.
 *  - Update set per-sensor timestamp offsets.
 *  - Invoke update() with a LidarScanSet to apply deskewing in-place.
 */
class OUSTER_API_CLASS DeskewMethod {
   public:
    /**
     * @brief Construct a deskew method from sensor info structures.
     *
     * @param[in] infos A vector of shared pointers to SensorInfo objects, one
     * per sensor.
     */
    OUSTER_API_FUNCTION
    DeskewMethod(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
            infos);

    /**
     * @brief Virtual destructor.
     */
    OUSTER_API_FUNCTION
    virtual ~DeskewMethod() = default;

    /**
     * @brief Perform in-place deskewing of all scans in the set.
     *
     * @param[in,out] scan_set A LidarScanSet containing one or more
     * temporally adjacent LiDAR scans. Implementations adjust the LidarScan
     * poses based on an assumed motion model to reduce distortion caused by
     *        sensor movement during LidarScan acquisition.
     */
    OUSTER_API_FUNCTION
    virtual void update(ouster::sdk::core::LidarScanSet& scan_set) = 0;

    /**
     * @brief Set the last pose computed from SLAM.
     *
     * @param[in] ts A timestamp corresponding to the last pose.
     * @param[in] pose A Matrix4dR object corresponding to the last pose.
     */
    OUSTER_API_FUNCTION
    virtual void set_last_pose(int64_t ts,
                               const ouster::sdk::core::Matrix4dR& pose) {
        if (ts_list_.size() >= 2) {
            ts_list_.pop_front();
            pose_list_.pop_front();
        }
        ts_list_.push_back(ts * 1e-9);
        pose_list_.push_back(pose);
    }

   private:
    DeskewMethod(const DeskewMethod&) = delete;
    DeskewMethod& operator=(const DeskewMethod&) = delete;

   protected:
    std::vector<ouster::sdk::core::XYZLut> xyzluts_;

    // TODO[UN]: change to a circular-buffer like behavior instead of deque
    std::deque<double> ts_list_;
    std::deque<ouster::sdk::core::Matrix4dR> pose_list_;
};

/**
 * @class ConstantVelocityDeskewMethod
 * @brief Deskew method assuming constant linear and angular velocity between
 * subsequent pose registrations.
 *
 * Implements a simple motion model that linearly interpolates pose over the
 * duration of a scan. Each point's acquisition time is used to interpolate an
 * SE3 transform, which is then applied to correct its position.
 *
 * Suitable for platforms with relatively smooth and predictable motion
 */
class OUSTER_API_CLASS ConstantVelocityDeskewMethod : public DeskewMethod {
   public:
    /**
     * @brief Construct from sensor info structures.
     *
     * @param[in] infos Per-sensor SensorInfo shared pointers.
     *
     * @see DeskewMethod::DeskewMethod(const
     * std::vector<std::shared_ptr<sensor::SensorInfo>>&)
     */
    OUSTER_API_FUNCTION
    ConstantVelocityDeskewMethod(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
            infos)
        : DeskewMethod(infos) {}

    /** @copydoc DeskewMethod::update(ouster::sdk::core::LidarScanSet&) */
    OUSTER_API_FUNCTION
    void update(ouster::sdk::core::LidarScanSet& scan_set) override;
};

/**
 * @class InertialIntegrationImuDeskewMethod
 * @brief Deskew method using inertial measurement unit (IMU) data to perform
 * motion compensation.
 *
 * Implements a motion model that integrates IMU angular velocity and linear
 * acceleration data to compute the sensor's pose over the duration of a scan.
 * Each imu data point is used to interpolate the scan poses to correct for
 * motion distortion.
 */
class OUSTER_API_CLASS InertialIntegrationImuDeskewMethod
    : public DeskewMethod {
    static constexpr double GRAVITY_MPERSEC2 = 9.80665;

   public:
    /**
     * @brief Construct from sensor info structures.
     *
     * @param[in] infos Per-sensor SensorInfo shared pointers.
     *
     * @see DeskewMethod::DeskewMethod(const
     * std::vector<std::shared_ptr<sensor::SensorInfo>>&)
     */
    OUSTER_API_FUNCTION
    InertialIntegrationImuDeskewMethod(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
            infos);

    /** @copydoc DeskewMethod::update(ouster::sdk::core::LidarScanSet&) */
    OUSTER_API_FUNCTION
    void update(ouster::sdk::core::LidarScanSet& scan_set);

    /** @copydoc DeskewMethod::set_last_pose */
    OUSTER_API_FUNCTION
    void set_last_pose(int64_t ts,
                       const ouster::sdk::core::Matrix4dR& pose) override;

   private:
    static void transform_imu_data_to_body_frame(
        Eigen::Ref<const ouster::sdk::core::Matrix4dR> imu_to_body_transform,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> gyro_imu_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> accel_imu_frame,
        ouster::sdk::core::MatrixX3dR& gyro_body_frame,
        ouster::sdk::core::MatrixX3dR& accel_body_frame);

    static double angle_between_poses(
        Eigen::Ref<const ouster::sdk::core::Matrix4dR> pose_a,
        Eigen::Ref<const ouster::sdk::core::Matrix4dR> pose_b);

    void estimate_gravity_vector(ouster::sdk::core::LidarScanSet& scans);

    std::vector<ouster::sdk::core::Matrix4dR> calc_poses_with_motion_model(
        double last_timestamp,
        Eigen::Ref<const ouster::sdk::core::Matrix4dR> last_body_to_world_pose,
        const std::vector<double>& timestamps,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR>
            angular_velocity_body_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR>
            linear_accel_body_frame);

    static bool pick_last_valid_imu_pose(
        const ouster::sdk::core::LidarScanSet& scan_set, double& last_ts,
        ouster::sdk::core::Matrix4dR& last_pose);

    std::vector<ouster::sdk::core::Matrix4dR> imu_to_body_transform_;
    ouster::sdk::core::LidarScanSet last_scan_set_;
    nonstd::optional<double> last_scan_set_last_ts_{};
    nonstd::optional<ouster::sdk::core::Matrix4dR> last_scan_set_last_pose_{};
    nonstd::optional<Eigen::Vector3d> gravity_vector_world_frame_xyz_;
    std::vector<Eigen::Vector3d> accel_bias_imu_frame_;
    std::vector<Eigen::Vector3d> gyro_bias_imu_frame_;
};

/**
 * @class DeskewMethodFactory
 * @brief Factory for constructing deskew method instances by name.
 *
 * Provides static convenience functions to create concrete DeskewMethod
 * implementations using the deskew method name.
 */
class OUSTER_API_CLASS DeskewMethodFactory {
   public:
    /**
     * @brief Create a deskew method by name using sensor info objects.
     *
     * @param[in] method The string identifier of the desired method (e.g.,
     *        "constant_velocity", "imu_deskew").
     * @param[in] infos Per-sensor SensorInfo shared pointers used to internally
     *        build XYZLuts for the selected method.
     *
     * @return Unique pointer to a DeskewMethod instance, or nullptr if "none"
     * was passed as the method name
     *
     * @throws if the method name is unrecognized.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<DeskewMethod> create(
        const std::string& method,
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
            infos);
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

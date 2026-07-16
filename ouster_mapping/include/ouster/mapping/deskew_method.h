#pragma once

#include <ouster/core/frame_set.h>
#include <ouster/core/lidar_frame.h>
#include <ouster/core/types.h>
#include <ouster/core/visibility.h>
#include <ouster/core/xyzlut.h>

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
 * @brief Abstract interface for LiDAR frame deskewing strategies.
 *
 * A DeskewMethod encapsulates the logic required to correct (deskew) the frame
 * poses for each LidarFrame within the FrameSet.
 *
 * Typical usage pattern:
 *  - Construct a concrete deskew method passing SensorInfo per-sensor.
 *  - Update set per-sensor timestamp offsets.
 *  - Invoke update() with a FrameSet to apply deskewing in-place.
 */
class OUSTER_API_CLASS DeskewMethod {
   public:
    /**
     * @brief Construct a deskew method from sensor info structures.
     *
     * @param[in] infos A vector of shared pointers to SensorInfo objects, one
     * per sensor.
     * @param[in] initial_pose Pose used to anchor the first frame before a
     *        motion model has been established. Defaults to identity.
     */
    OUSTER_API_FUNCTION
    DeskewMethod(const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& infos,
                 const core::Matrix4dR& initial_pose = core::Matrix4dR::Identity());

    /**
     * @brief Virtual destructor.
     */
    OUSTER_API_FUNCTION
    virtual ~DeskewMethod() = default;

    /**
     * @brief Perform in-place deskewing of all frames in the set.
     *
     * @param[in,out] frame_set A FrameSet containing one or more
     * temporally adjacent LiDAR frames. Implementations adjust the LidarFrame
     * poses based on an assumed motion model to reduce distortion caused by
     *        sensor movement during LidarFrame acquisition.
     */
    OUSTER_API_FUNCTION
    virtual void update(ouster::sdk::core::FrameSet& frame_set) = 0;

    /**
     * @brief Set the last pose computed from SLAM.
     *
     * @param[in] timestamp_ns A timestamp corresponding to the last pose.
     * @param[in] pose A Matrix4dR object corresponding to the last pose.
     */
    OUSTER_API_FUNCTION
    virtual void set_last_pose(int64_t timestamp_ns, const core::Matrix4dR& pose) {
        if (ts_list_.size() >= 2) {
            ts_list_.pop_front();
            pose_list_.pop_front();
        }
        ts_list_.push_back(timestamp_ns * 1e-9);
        pose_list_.push_back(pose);
    }

    /**
     * @brief Finalize deskewing after registration has corrected the frame.
     *
     * Implementations may use the post-registration anchor to refine current
     * frame poses before saving state for the next update. The default behavior
     * preserves the previous feedback path.
     *
     * @param[in,out] frame_set The lidar frame set to finalize.
     * @param[in] anchor_timestamp_ns Timestamp of the anchor point in
     *        nanoseconds.
     * @param[in] corrected_anchor_pose Post-registration corrected pose of the
     *        anchor.
     */
    OUSTER_API_FUNCTION
    virtual void finalize_after_registration(ouster::sdk::core::FrameSet& frame_set,
                                             int64_t anchor_timestamp_ns,
                                             const core::Matrix4dR& corrected_anchor_pose) {
        (void)frame_set;
        set_last_pose(anchor_timestamp_ns, corrected_anchor_pose);
    }

   private:
    DeskewMethod(const DeskewMethod&) = delete;
    DeskewMethod& operator=(const DeskewMethod&) = delete;

   protected:
    std::vector<ouster::sdk::core::XYZLut> xyzluts_;

    // TODO[UN]: change to a circular-buffer like behavior instead of deque
    std::deque<double> ts_list_;
    std::deque<core::Matrix4dR> pose_list_;

    /// Pose used to anchor the first frame before a motion model has been
    /// established. Defaults to identity (the map origin).
    core::Matrix4dR initial_pose_ = core::Matrix4dR::Identity();
};

/**
 * @class ConstantVelocityDeskewMethod
 * @brief Deskew method assuming constant linear and angular velocity between
 * subsequent pose registrations.
 *
 * Implements a simple motion model that linearly interpolates pose over the
 * duration of a frame. Each point's acquisition time is used to interpolate a
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
     * @param[in] initial_pose Pose used to anchor the first frame before a
     *        motion model has been established. Defaults to identity.
     *
     * @see DeskewMethod::DeskewMethod(const
     * std::vector<std::shared_ptr<sensor::SensorInfo>>&)
     */
    OUSTER_API_FUNCTION
    ConstantVelocityDeskewMethod(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& infos,
        const core::Matrix4dR& initial_pose = core::Matrix4dR::Identity())
        : DeskewMethod(infos, initial_pose) {}

    /** @copydoc DeskewMethod::update(ouster::sdk::core::FrameSet&) */
    OUSTER_API_FUNCTION
    void update(ouster::sdk::core::FrameSet& frame_set) override;
};

/**
 * @class InertialIntegrationImuDeskewMethod
 * @brief Deskew method using inertial measurement unit (IMU) data to perform
 * motion compensation.
 *
 * Implements a motion model that integrates IMU angular velocity and linear
 * acceleration data to compute the sensor's pose over the duration of a frame.
 * Each imu data point is used to interpolate the frame poses to correct for
 * motion distortion.
 */
class OUSTER_API_CLASS InertialIntegrationImuDeskewMethod : public DeskewMethod {
    static constexpr double GRAVITY_MPERSEC2 = 9.80665;

   public:
    /**
     * @brief Construct from sensor info structures.
     *
     * @param[in] infos Per-sensor SensorInfo shared pointers.
     * @param[in] initial_pose Pose used to anchor the first frame before a
     *        motion model has been established. Defaults to identity.
     *
     * @see DeskewMethod::DeskewMethod(const
     * std::vector<std::shared_ptr<sensor::SensorInfo>>&)
     */
    OUSTER_API_FUNCTION
    InertialIntegrationImuDeskewMethod(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& infos,
        const core::Matrix4dR& initial_pose = core::Matrix4dR::Identity());

    /** @copydoc DeskewMethod::update(ouster::sdk::core::FrameSet&) */
    OUSTER_API_FUNCTION
    void update(ouster::sdk::core::FrameSet& frame_set) override;

    /** @copydoc DeskewMethod::set_last_pose */
    OUSTER_API_FUNCTION
    void set_last_pose(int64_t timestamp_ns, const core::Matrix4dR& pose) override;

    /** @brief Finalize deskewing after registration has corrected the frame.
     *  @param[in,out] frame_set The lidar frame set to finalize.
     *  @param[in] anchor_timestamp_ns Timestamp of the anchor point in
     *         nanoseconds.
     *  @param[in] corrected_anchor_pose Post-registration corrected pose of
     *         the anchor.
     */
    OUSTER_API_FUNCTION
    void finalize_after_registration(ouster::sdk::core::FrameSet& frame_set,
                                     int64_t anchor_timestamp_ns,
                                     const core::Matrix4dR& corrected_anchor_pose) override;

   private:
    struct VelocitySolveResult {
        Eigen::Vector3d initial_velocity_world_frame;
        Eigen::Vector3d ending_velocity_world_frame;
    };

    static void transform_imu_data_to_body_frame(
        Eigen::Ref<const core::Matrix4dR> imu_to_body_transform,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> gyro_imu_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> accel_imu_frame,
        ouster::sdk::core::MatrixX3dR& gyro_body_frame,
        ouster::sdk::core::MatrixX3dR& accel_body_frame);

    static double angle_between_poses(Eigen::Ref<const core::Matrix4dR> pose_a,
                                      Eigen::Ref<const core::Matrix4dR> pose_b);

    void estimate_gravity_vector(ouster::sdk::core::FrameSet& frames);

    std::vector<core::Matrix4dR> calc_poses_with_motion_model(
        double last_timestamp, Eigen::Ref<const core::Matrix4dR> last_body_to_world_pose,
        const Eigen::Vector3d& initial_linear_velocity_world_frame,
        const std::vector<double>& timestamps,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> angular_velocity_body_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> linear_accel_body_frame);

    std::vector<core::Matrix4dR> calc_corrected_poses_with_motion_model(
        double last_timestamp, Eigen::Ref<const core::Matrix4dR> last_body_to_world_pose,
        double target_timestamp, Eigen::Ref<const core::Matrix4dR> target_pose,
        const Eigen::Vector3d& initial_linear_velocity_world_frame,
        const std::vector<double>& timestamps,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> angular_velocity_body_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> linear_accel_body_frame,
        Eigen::Vector3d* ending_velocity_world_frame) const;

    nonstd::optional<VelocitySolveResult> solve_initial_velocity(
        double last_timestamp, Eigen::Ref<const core::Matrix4dR> last_body_to_world_pose,
        double target_timestamp, Eigen::Ref<const core::Matrix4dR> target_pose,
        const std::vector<double>& timestamps,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> angular_velocity_body_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> linear_accel_body_frame) const;

    static void interpolate_imu_poses_to_frame_set(ouster::sdk::core::FrameSet& frame_set,
                                                   const std::vector<double>& imu_timestamps,
                                                   const std::vector<core::Matrix4dR>& imu_poses);

    static bool pick_last_valid_imu_pose(const ouster::sdk::core::FrameSet& frame_set,
                                         double& last_ts, core::Matrix4dR& last_pose);

    std::vector<core::Matrix4dR> imu_to_body_transform_;
    ouster::sdk::core::FrameSet last_frame_set_;
    std::vector<double> rolling_imu_timestamps_;
    ouster::sdk::core::MatrixX3dR rolling_gyro_body_frame_;
    ouster::sdk::core::MatrixX3dR rolling_accel_body_frame_;
    nonstd::optional<double> imu_integration_anchor_ts_{};
    nonstd::optional<core::Matrix4dR> imu_integration_anchor_pose_{};
    nonstd::optional<Eigen::Vector3d> last_linear_velocity_world_frame_;
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
     * @param[in] initial_pose Pose used to anchor the first frame before a
     *        motion model has been established. Defaults to identity.
     *
     * @return Unique pointer to a DeskewMethod instance, or nullptr if "none"
     * was passed as the method name
     *
     * @throws if the method name is unrecognized.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<DeskewMethod> create(
        const std::string& method,
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& infos,
        const core::Matrix4dR& initial_pose = core::Matrix4dR::Identity());
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

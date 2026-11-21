#pragma once

#include <ouster/core/frame_set.h>
#include <ouster/core/lidar_frame.h>
#include <ouster/core/sensor_info.h>
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
 *  - Construct a concrete deskew method passing LidarInfo per-sensor.
 *  - Update set per-sensor timestamp offsets.
 *  - Invoke update() with a FrameSet to apply deskewing in-place.
 */
class OUSTER_API_CLASS DeskewMethod {
   public:
    /**
     * @brief Construct a deskew method from sensor info structures.
     *
     * @param[in] sensor_infos A unified vector of SensorInfo, one per sensor,
     * sharing the FrameSet index space (sensor i describes frame i) in any
     * order. Per-sensor state is sized to this list and indexed by frameset
     * position, so lidar/camera ordering does not matter.
     * @param[in] initial_pose Pose used to anchor the first frame before a
     *        motion model has been established. Unset anchors it at identity.
     */
    OUSTER_API_FUNCTION
    explicit DeskewMethod(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos,
        const nonstd::optional<core::Matrix4dR>& initial_pose = nonstd::nullopt);

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
    nonstd::optional<core::Matrix4dR> initial_pose_;
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
     * @param[in] sensor_infos Per-sensor SensorInfo shared pointers, in
     *        FrameSet order.
     * @param[in] initial_pose Pose used to anchor the first frame before a
     *        motion model has been established. Unset anchors it at identity.
     *
     * @see DeskewMethod::DeskewMethod(const
     * std::vector<std::shared_ptr<core::SensorInfo>>&)
     */
    OUSTER_API_FUNCTION
    ConstantVelocityDeskewMethod(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos,
        const nonstd::optional<core::Matrix4dR>& initial_pose = nonstd::nullopt)
        : DeskewMethod(sensor_infos, initial_pose) {}

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
   public:
    /**
     * @brief Construct from sensor info structures.
     *
     * @param[in] sensor_infos Per-sensor SensorInfo shared pointers, in
     *        FrameSet order.
     * @param[in] initial_pose Body-to-world pose, used directly as the first
     *        anchor; unset means identity. @p gravity_from_initial_pose picks how
     *        the anchor and the initial gravity are set; near-static IMU
     *        windows refine the gravity later either way.
     * @throws std::invalid_argument if @p plumb_method is not one of the three
     *         names.
     * @param[in] plumb_method How the first registered frame set is plumbed:
     *        "imu", "walls", or "all" (default) for the IMU estimate refined
     *        by wall geometry. If the IMU plumb fails there, later frame sets
     *        retry it until it succeeds. Ignored when gravity comes from
     *        @p initial_pose.
     * @param[in] gravity_from_initial_pose True (SLAM): a supplied
     *        @p initial_pose's world frame is assumed level, so the initial
     *        gravity is world -Z and only the IMU plumb runs, to check it,
     *        warning when its gravity is more than 10 degrees off; without one,
     *        plumbing levels the first anchor. False (localization) uses the
     *        anchor exactly as given and takes the initial gravity from
     *        plumbing, rotated into the world frame by it.
     *
     * @see DeskewMethod::DeskewMethod(const
     * std::vector<std::shared_ptr<core::SensorInfo>>&)
     */
    OUSTER_API_FUNCTION
    InertialIntegrationImuDeskewMethod(
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos,
        const nonstd::optional<core::Matrix4dR>& initial_pose = nonstd::nullopt,
        const std::string& plumb_method = "all", bool gravity_from_initial_pose = true);

    /** @copydoc DeskewMethod::update(ouster::sdk::core::FrameSet&) */
    OUSTER_API_FUNCTION
    void update(ouster::sdk::core::FrameSet& frame_set) override;

    /** @copydoc DeskewMethod::set_last_pose */
    OUSTER_API_FUNCTION
    void set_last_pose(int64_t timestamp_ns, const core::Matrix4dR& pose) override;

    /** @brief Finalize deskewing after registration has corrected the frame.
     *  Validates a supplied initial pose once, on the first completed registration.
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
    struct OUSTER_API_IGNORE VelocitySolveResult {
        Eigen::Vector3d initial_velocity_world_frame;
        Eigen::Vector3d ending_velocity_world_frame;
        // IMU sample times from the previous anchor on, plus a node at the registration anchor.
        std::vector<double> nodes_ts;
        // World-frame velocity at each of nodes_ts, given the solved initial velocity.
        std::vector<Eigen::Vector3d> nodes_velocities_world_frame;
    };

    static void transform_imu_data_to_body_frame(
        Eigen::Ref<const core::Matrix4dR> imu_to_body_transform,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> gyro_imu_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> accel_imu_frame,
        ouster::sdk::core::MatrixX3dR& gyro_body_frame,
        ouster::sdk::core::MatrixX3dR& accel_body_frame);

    static double angle_between_poses(Eigen::Ref<const core::Matrix4dR> pose_a,
                                      Eigen::Ref<const core::Matrix4dR> pose_b);

    /**
     * Derive the unit gravity direction in body coordinates from a pose.
     *
     * @param[in] body_to_world_pose Pose mapping body coordinates into a world
     *        frame where gravity points down along world -Z.
     * @return Normalized R^T * [0, 0, -1], where R is the pose rotation.
     *         Translation does not affect the result. Rotate this direction
     *         by R before using it with world-frame acceleration.
     */
    static Eigen::Vector3d gravity_direction_from_pose(
        Eigen::Ref<const core::Matrix4dR> body_to_world_pose);

    void estimate_gravity_vector(ouster::sdk::core::FrameSet& frames);

    // Warn when the plumbed gravity is off world -Z beyond
    // INITIAL_POSE_LEVEL_TOL_DEG. SLAM with a supplied initial_pose only; never
    // changes the pose.
    void validate_initial_pose_gravity();

    // anchor_pose: body-to-world pose the IMU gravity sample belongs to; with
    // level_anchor its rotation is replaced so gravity points down along world -Z.
    void plumb_gravity_from_imu(ouster::sdk::core::FrameSet& frame_set,
                                core::Matrix4dR& anchor_pose, bool level_anchor);
    // With level_poses the frame set's poses are rotated so its walls stand plumb.
    void plumb_gravity_from_walls(ouster::sdk::core::FrameSet& frame_set, bool level_poses);
    void set_plumbed_up(const Eigen::Vector3d& up_world_frame_xyz);

    std::vector<core::Matrix4dR> calc_poses_with_motion_model(
        double last_timestamp, Eigen::Ref<const core::Matrix4dR> last_body_to_world_pose,
        const Eigen::Vector3d& initial_linear_velocity_world_frame,
        const std::vector<double>& timestamps,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> angular_velocity_body_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> linear_accel_body_frame,
        Eigen::Ref<ouster::sdk::core::MatrixX3dR> nodes_velocities_world_frame);

    std::vector<core::Matrix4dR> calc_corrected_poses_with_motion_model(
        double last_timestamp, Eigen::Ref<const core::Matrix4dR> last_body_to_world_pose,
        size_t target_idx, Eigen::Ref<const core::Matrix4dR> target_pose,
        const Eigen::Vector3d& initial_linear_velocity_world_frame,
        const std::vector<double>& timestamps,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> angular_velocity_body_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> linear_accel_body_frame,
        Eigen::Ref<Eigen::Vector3d> ending_velocity_world_frame,
        Eigen::Ref<ouster::sdk::core::MatrixX3dR> nodes_velocities_world_frame) const;

    nonstd::optional<VelocitySolveResult> solve_initial_velocity(
        double last_timestamp, Eigen::Ref<const core::Matrix4dR> last_body_to_world_pose,
        double target_timestamp, Eigen::Ref<const core::Matrix4dR> target_pose,
        const std::vector<double>& timestamps,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> angular_velocity_body_frame,
        Eigen::Ref<const ouster::sdk::core::MatrixX3dR> linear_accel_body_frame) const;

    static void interpolate_imu_poses_to_frame_set(ouster::sdk::core::FrameSet& frame_set,
                                                   const std::vector<double>& imu_timestamps,
                                                   const std::vector<core::Matrix4dR>& imu_poses);

    // Every valid lidar IMU sample of the frame set: its timestamp and column pose, by time.
    static void collect_imu_poses(const ouster::sdk::core::FrameSet& frame_set,
                                  std::vector<double>& timestamps,
                                  std::vector<core::Matrix4dR>& poses);

    std::vector<core::Matrix4dR> imu_to_body_transform_;
    ouster::sdk::core::FrameSet last_frame_set_;
    std::vector<double> rolling_imu_timestamps_;
    ouster::sdk::core::MatrixX3dR rolling_gyro_body_frame_;
    ouster::sdk::core::MatrixX3dR rolling_accel_body_frame_;
    // IMU samples of the previous frame set; the next update anchors at the latest one at or
    // before its first column.
    std::vector<double> imu_anchor_ts_;
    std::vector<core::Matrix4dR> imu_anchor_poses_;
    // Velocity at the last velocity solve's IMU nodes, for anchors before its last node.
    std::vector<double> velocity_nodes_ts_;
    std::vector<Eigen::Vector3d> velocity_nodes_world_frame_;
    /// Start velocity of the next integration: open-loop IMU estimate, replaced by post-ICP solve.
    nonstd::optional<Eigen::Vector3d> last_linear_velocity_world_frame_;
    nonstd::optional<Eigen::Vector3d> gravity_vector_world_frame_;
    /// World frame up direction measured by plumbing. It becomes the deskew
    /// gravity unless gravity_from_initial_pose_, where it only checks the pose.
    nonstd::optional<Eigen::Vector3d> plumbed_up_world_frame_vector_;
    /// A supplied initial_pose sets the initial gravity to world -Z.
    bool gravity_from_initial_pose_ = false;
    /// Plumbing levels the world frame: SLAM without an initial_pose.
    bool level_world_frame_ = false;
    std::vector<Eigen::Vector3d> accel_bias_imu_frame_;
    std::vector<Eigen::Vector3d> gyro_bias_imu_frame_;
    /// Consecutive frames whose rotation stayed under the stationary threshold.
    int no_rotation_frame_count_ = 0;
    /// Consecutive frames whose rotation and translation stayed under the stationary thresholds.
    int stationary_frame_count_ = 0;

    /// Which plumb stages initialize gravity or validate a supplied initial pose.
    bool plumb_imu_ = true;
    bool plumb_walls_ = false;

    /// Set by the first set_last_pose, after registration or a skipped ICP:
    /// later frame sets integrate from that pose, so plumbing can no longer
    /// level the world frame. Frame sets dropped before then are plumbed again.
    bool world_frame_determined_ = false;
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
     * @param[in] sensor_infos Per-sensor SensorInfo shared pointers (in
     *        FrameSet order) used to select and internally configure the
     *        method. Non-lidar entries are ignored.
     * @param[in] initial_pose Pose used to anchor the first frame before a
     *        motion model has been established; unset means identity. See
     *        InertialIntegrationImuDeskewMethod for how IMU deskewing uses it.
     * @param[in] plumb_method How the IMU method plumbs the first frame set:
     *        "imu", "walls" or "all" (default).
     * @param[in] gravity_from_initial_pose True for SLAM, false for
     *        localization; see InertialIntegrationImuDeskewMethod.
     *
     * @return Unique pointer to a DeskewMethod instance, or nullptr if "none"
     * was passed as the method name
     *
     * @throws if the method name is unrecognized.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<DeskewMethod> create(
        const std::string& method,
        const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& sensor_infos,
        const nonstd::optional<core::Matrix4dR>& initial_pose = nonstd::nullopt,
        const std::string& plumb_method = "all", bool gravity_from_initial_pose = true);
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

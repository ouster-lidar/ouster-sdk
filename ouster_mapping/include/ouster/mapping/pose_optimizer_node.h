#pragma once

#include <cstdint>
#include <memory>

#include "ouster/core/impl/transformation.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @brief Represents a trajectory node, including its pose and downsampled point
 * cloud.
 */
class OUSTER_API_CLASS Node {
   public:
#ifndef OUSTER_CHECK_EXPORTS
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

    /** Timestamp of the node (nanoseconds). */
    uint64_t ts;

    /** Downsampled point cloud for pose-to-pose constraints. */
    core::ArrayX3dR downsampled_pts;

    /** ICP cloud points sampled from the lidar frame geometry. */
    core::ArrayX3dR icp_pts;

    /** Per-point normals for icp_pts. */
    core::ArrayX3dR icp_normals;

    /** Selected point for point-to-point constraints (size 1 or empty). */
    Eigen::ArrayX3d ptp_constraint_pt;

    /** Row index of the selected point for point-to-point constraints. */
    int ptp_row = -1;

    /** Column index of the selected point for point-to-point constraints. */
    int ptp_col = -1;

    /** Return index of the selected point for point-to-point constraints. */
    int ptp_return = -1;

    /** Selected point for absolute point constraints (size 1 or empty). */
    Eigen::ArrayX3d ap_constraint_pt;

    /**  Row index of the selected point for absolute point constraints. */
    int ap_row = -1;

    /**  Column index of the selected point for absolute point constraints. */
    int ap_col = -1;

    /**  Return index of the selected point for absolute point constraints. */
    int ap_return = -1;

    /**
     * @brief Constructor for a trajectory node.
     *
     * @param[in] timestamp Timestamp of the node.
     * @param[in] pose Homogeneous pose of the node.
     */
    OUSTER_API_FUNCTION
    Node(uint64_t timestamp, const core::Matrix4dR& pose);

    /**
     * @brief Constructor for a trajectory node.
     *
     * @param[in] timestamp Timestamp of the node.
     * @param[in] rotation Rotational component of the pose.
     * @param[in] position Translational component of the pose.
     */
    OUSTER_API_FUNCTION
    Node(uint64_t timestamp, Eigen::Quaterniond rotation, Eigen::Vector3d position);

    /**
     * @brief Retrieves the rotational component of the node pose.
     *
     * @return The node orientation as a quaternion.
     */
    OUSTER_API_FUNCTION
    const Eigen::Quaterniond& rotation() const;

    /**
     * @brief Retrieves the translational component of the node pose.
     *
     * @return The node position as an XYZ vector.
     */
    OUSTER_API_FUNCTION
    const Eigen::Vector3d& position() const;

    /**
     * @brief Replace the node pose components and invalidate the cached 4x4
     * pose.
     *
     * @param[in] rotation New rotational component of the pose.
     * @param[in] position New translational component of the pose.
     */
    OUSTER_API_FUNCTION
    void set_pose_components(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& position);

    /**
     * @brief Returns mutable quaternion coeff storage for solver updates.
     *
     * Requesting mutable access marks the cached 4x4 pose dirty.
     *
     * @return Pointer to the quaternion coefficient storage.
     */
    OUSTER_API_FUNCTION
    double* mutable_rotation_coeffs_data();

    /**
     * @brief Returns read-only quaternion coeff storage.
     *
     * @return Pointer to the quaternion coefficient storage.
     */
    OUSTER_API_FUNCTION
    const double* rotation_coeffs_data() const;

    /**
     * @brief Returns mutable translation storage for solver updates.
     *
     * Requesting mutable access marks the cached 4x4 pose dirty.
     *
     * @return Pointer to the XYZ translation storage.
     */
    OUSTER_API_FUNCTION
    double* mutable_position_data();

    /**
     * @brief Returns read-only translation storage.
     *
     * @return Pointer to the XYZ translation storage.
     */
    OUSTER_API_FUNCTION
    const double* position_data() const;

    /**
     * @brief Marks the cached 4x4 pose as stale.
     *
     * Call this when quaternion/translation are externally modified and a
     * fresh homogeneous matrix is needed on the next get_pose().
     */
    OUSTER_API_FUNCTION
    void mark_pose_dirty();

    /**
     * @brief Retrieves the pose of the node.
     *
     * @return The homogeneous pose of the node as 4x4 matrix.
     */
    OUSTER_API_FUNCTION
    const core::Matrix4dR& get_pose() const;

    /**
     * @brief Compares nodes by their timestamps.
     *
     * @param[in] other The other node to compare with.
     * @return True if this node's timestamp is less than the other node's
     * timestamp, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator<(const Node& other) const;

   private:
    friend class Trajectory;

    /**
     * @brief Builds the homogeneous pose matrix from the current pose
     * components.
     *
     * @return The node pose as a 4x4 homogeneous matrix.
     */
    core::Matrix4dR compute_transformation() const;

    /**
     * @brief Refreshes the cached homogeneous pose when it is marked dirty.
     */
    void update_pose() const;
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d position_;
    mutable core::Matrix4dR pose_;
    mutable bool pose_dirty_ = true;
};

/**
 * @brief Comparator for shared pointers to Node.
 */
struct OUSTER_API_CLASS NodePtrComparator {
    OUSTER_API_FUNCTION
    bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
        return lhs->ts < rhs->ts;
    }
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

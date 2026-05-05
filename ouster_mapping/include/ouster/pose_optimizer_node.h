#pragma once

#include <cstdint>
#include <memory>

#include "ouster/impl/transformation.h"
#include "ouster/visibility.h"

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
    Eigen::ArrayX3d downsampled_pts;

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

    /** Rotational component of the node's pose (quaternion). */
    Eigen::Quaterniond rotation;

    /** Translational component of the node's pose (x,y,z). */
    Eigen::Vector3d position;

    /**
     * @brief Constructor for a trajectory node.
     *
     * @param[in] timestamp Timestamp of the node.
     * @param[in] pose Homogeneous pose of the node.
     */
    OUSTER_API_FUNCTION
    Node(uint64_t timestamp, const Eigen::Matrix4d& pose);

    /**
     * @brief Constructor for a trajectory node.
     *
     * @param[in] timestamp Timestamp of the node.
     * @param[in] rotation Rotational component of the pose.
     * @param[in] position Translational component of the pose.
     */
    OUSTER_API_FUNCTION
    Node(uint64_t timestamp, const Eigen::Quaterniond& rotation,
         const Eigen::Vector3d& position);

    /**
     * @brief Updates the pose of the node.
     *
     * This function updates the homogeneous pose matrix of the node based on
     * the quaternion and position vector, which are refined by the pose
     * optimization iteration.
     */
    OUSTER_API_FUNCTION
    void update_pose();

    /**
     * @brief Retrieves the pose of the node.
     *
     * @return The homogeneous pose of the node as 4x4 matrix.
     */
    OUSTER_API_FUNCTION
    const Eigen::Matrix4d& get_pose() const;

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
    Eigen::Matrix4d compute_transformation() const;
    Eigen::Matrix4d pose_;
};

/**
 * @brief Comparator for shared pointers to Node.
 */
struct OUSTER_API_CLASS NodePtrComparator {
    OUSTER_API_FUNCTION
    bool operator()(const std::shared_ptr<Node>& lhs,
                    const std::shared_ptr<Node>& rhs) const {
        return lhs->ts < rhs->ts;
    }
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

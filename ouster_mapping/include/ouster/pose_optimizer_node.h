#pragma once

#include <memory>

#include "ouster/impl/transformation.h"

namespace ouster {
namespace mapping {

/**
 * @brief Represents a trajectory node, including its pose and downsampled point
 * cloud.
 */
class Node {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t ts;                                  ///< Timestamp of the node.
    Eigen::Array<double, Eigen::Dynamic, 3> pts;  ///< Downsampled point cloud.
    Eigen::Quaterniond rotation;  ///< Rotational component of the pose.
    Eigen::Vector3d position;     ///< Translational component of the pose.

    /**
     * @brief Constructor for a trajectory node.
     *
     * @param[in] ts Timestamp of the node.
     * @param[in] pose Homogeneous pose of the node.
     * @param[in] pts Downsampled point cloud.
     */
    Node(uint64_t ts, const ouster::impl::PoseH& pose,
         const Eigen::Array<double, Eigen::Dynamic, 3>& pts =
             Eigen::Array<double, 0, 3, 0>());

    /**
     * @brief Constructor for a trajectory node.
     *
     * @param[in] ts Timestamp of the node.
     * @param[in] rotation Rotational component of the pose.
     * @param[in] position Translational component of the pose.
     * @param[in] pts Downsampled point cloud.
     */
    Node(uint64_t ts, const Eigen::Quaterniond& rotation,
         const Eigen::Vector3d& position,
         const Eigen::Array<double, Eigen::Dynamic, 3>& pts =
             Eigen::Array<double, 0, 3, 0>());

    /**
     * @brief Updates the pose of the node.
     *
     * This function updates the homogeneous pose matrix of the node based on
     * the quaternion and position vector, which are refined by the pose
     * optimization iteration.
     */
    void update_pose();

    /**
     * @brief Retrieves the pose of the node.
     *
     * @return The homogeneous pose of the node.
     */
    const ouster::impl::PoseH& get_pose() const;

    /**
     * @brief Compares nodes by their timestamps.
     *
     * @param[in] other The other node to compare with.
     * @return True if this node's timestamp is less than the other node's
     * timestamp, false otherwise.
     */
    bool operator<(const Node& other) const;

   private:
    ouster::impl::PoseH compute_transformation() const;
    ouster::impl::PoseH pose_;
};

/**
 * @brief Comparator for shared pointers to Node.
 */
struct NodePtrComparator {
    bool operator()(const std::shared_ptr<Node>& lhs,
                    const std::shared_ptr<Node>& rhs) const {
        return lhs->ts < rhs->ts;
    }
};

}  // namespace mapping
}  // namespace ouster

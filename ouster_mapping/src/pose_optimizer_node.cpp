#include "ouster/pose_optimizer_node.h"

#include <cstdint>

namespace ouster {
namespace sdk {
namespace mapping {

Node::Node(uint64_t timestamp, const Eigen::Matrix4d& pose)
    : ts(timestamp),
      downsampled_pts(Eigen::Array<double, 0, 3, 0>()),
      ptp_constraint_pt(Eigen::Array<double, 0, 3, 0>()),
      ap_constraint_pt(Eigen::Array<double, 0, 3, 0>()),
      rotation(Eigen::Quaterniond(pose.block<3, 3>(0, 0))),
      position(pose.block<3, 1>(0, 3)),
      pose_(pose) {}

Node::Node(uint64_t timestamp, const Eigen::Quaterniond& rotation,
           const Eigen::Vector3d& position)
    : ts(timestamp),
      downsampled_pts(Eigen::Array<double, 0, 3, 0>()),
      ptp_constraint_pt(Eigen::Array<double, 0, 3, 0>()),
      ap_constraint_pt(Eigen::Array<double, 0, 3, 0>()),
      rotation(rotation),
      position(position) {
    update_pose();
}

Eigen::Matrix4d Node::compute_transformation() const {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    m.block<3, 1>(0, 3) = position;
    return m;
}

const Eigen::Matrix4d& Node::get_pose() const { return pose_; }

void Node::update_pose() { pose_ = compute_transformation(); }

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

#include "ouster/mapping/pose_optimizer_node.h"

#include <cstdint>
#include <utility>

namespace ouster {
namespace sdk {
namespace mapping {

Node::Node(uint64_t timestamp, const core::Matrix4dR& pose)
    : ts(timestamp),
      downsampled_pts(core::ArrayX3dR()),
      icp_pts(core::ArrayX3dR()),
      icp_normals(core::ArrayX3dR()),
      ptp_constraint_pt(Eigen::Array<double, 0, 3, 0>()),
      ap_constraint_pt(Eigen::Array<double, 0, 3, 0>()),
      rotation_(Eigen::Quaterniond(pose.block<3, 3>(0, 0))),
      position_(pose.block<3, 1>(0, 3)),
      pose_(pose),
      pose_dirty_(false) {}

Node::Node(uint64_t timestamp, Eigen::Quaterniond rotation, Eigen::Vector3d position)
    : ts(timestamp),
      downsampled_pts(core::ArrayX3dR()),
      icp_pts(core::ArrayX3dR()),
      icp_normals(core::ArrayX3dR()),
      ptp_constraint_pt(Eigen::Array<double, 0, 3, 0>()),
      ap_constraint_pt(Eigen::Array<double, 0, 3, 0>()),
      rotation_(std::move(rotation)),
      position_(std::move(position)),
      pose_(compute_transformation()),
      pose_dirty_(false) {}

core::Matrix4dR Node::compute_transformation() const {
    core::Matrix4dR mat = core::Matrix4dR::Identity();
    mat.block<3, 3>(0, 0) = rotation_.toRotationMatrix();
    mat.block<3, 1>(0, 3) = position_;
    return mat;
}

const Eigen::Quaterniond& Node::rotation() const {
    return rotation_;
}

const Eigen::Vector3d& Node::position() const {
    return position_;
}

void Node::set_pose_components(const Eigen::Quaterniond& rotation,
                               const Eigen::Vector3d& position) {
    rotation_ = rotation;
    position_ = position;
    pose_dirty_ = true;
}

double* Node::mutable_rotation_coeffs_data() {
    pose_dirty_ = true;
    return rotation_.coeffs().data();
}

const double* Node::rotation_coeffs_data() const {
    return rotation_.coeffs().data();
}

double* Node::mutable_position_data() {
    pose_dirty_ = true;
    return position_.data();
}

const double* Node::position_data() const {
    return position_.data();
}

const core::Matrix4dR& Node::get_pose() const {
    update_pose();
    return pose_;
}

void Node::update_pose() const {
    if (!pose_dirty_) {
        return;
    }
    pose_ = compute_transformation();
    pose_dirty_ = false;
}

void Node::mark_pose_dirty() {
    pose_dirty_ = true;
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

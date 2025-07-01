#include "ouster/pose_optimizer_node.h"

namespace ouster {
namespace mapping {

Node::Node(uint64_t ts, const ouster::impl::PoseH& pose,
           const Eigen::Array<double, Eigen::Dynamic, 3>& pts)
    : ts(ts), pts(pts), pose_(pose) {
    ouster::impl::PoseQ poseq = pose.log().q();
    rotation = poseq.r();
    position = poseq.t();
}

Node::Node(uint64_t ts, const Eigen::Quaterniond& rotation,
           const Eigen::Vector3d& position,
           const Eigen::Array<double, Eigen::Dynamic, 3>& pts)
    : ts(ts), pts(pts), rotation(rotation), position(position) {
    update_pose();
}

ouster::impl::PoseH Node::compute_transformation() const {
    ouster::impl::PoseQ poseq;
    poseq.set_rot(rotation);
    poseq.set_trans(position);
    ouster::impl::PoseH poseh = poseq.v().exp();
    poseh.reorthogonalize();
    return poseh;
}

const ouster::impl::PoseH& Node::get_pose() const { return pose_; }

void Node::update_pose() { pose_ = compute_transformation(); }

}  // namespace mapping
}  // namespace ouster

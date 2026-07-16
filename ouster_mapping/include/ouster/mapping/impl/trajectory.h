#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/xyzlut.h"
#include "ouster/mapping/pose_optimizer_constraint.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {
class PoseH;
}  // namespace impl
}  // namespace core

namespace mapping {
class Node;

struct TimestampsFrameIndex {
    uint64_t first_col_ts;  // Timestamp of the first column in the frame
    uint64_t last_col_ts;   // Timestamp of the last column in the frame
    uint32_t frame_index;   // Frame index of the OSF file
};

class Trajectory {
   public:
    /**
     * @brief Construct a new Trajectory object.
     *
     * @param[in] osf_file Path to the input OSF file.
     * @param[in] key_frame_distance Minimum gap/distance between consecutive
     * nodes in meters.
     */
    Trajectory(const std::string& osf_file, double key_frame_distance);

    Trajectory() = default;

    /**
     * @brief Retrieves the node corresponding to the given timestamp.
     *
     * @param[in] ts Timestamp associated with the node.
     * @return std::shared_ptr<Node> Pointer to the node if found, otherwise
     * nullptr. The returned node has its cached 4x4 pose refreshed from its
     * current quaternion/translation state.
     */
    std::shared_ptr<Node> get_node_by_ts(uint64_t timestamp) const;

    /**
     * @brief Creates a new node with the specified timestamp.
     *
     * @param[in] ts Timestamp for the new node.
     * @param[in] generate_point_cloud Flag indicating whether a point cloud
     * should be generated.
     * @return std::shared_ptr<Node> Pointer to the newly created node.
     */
    std::shared_ptr<Node> create_node_by_ts(uint64_t timestamp, bool generate_point_cloud = false,
                                            double downsample_voxel_size = 0.05);

    /**
     * @brief Saves the trajectory data to an output OSF file.
     *
     * @param[in] out_osf File path for saving the trajectory data.
     */
    void save(const std::string& out_osf);

    /**
     * @brief Inserts a new node into the trajectory based on a LidarFrame.
     *
     * @param[in] ls LidarFrame to insert as a new node.
     * @param[in] generate_point_cloud Flag indicating whether a point cloud
     * should be generated.
     * @return std::shared_ptr<Node> Pointer to the newly inserted or existing
     * node.
     */
    std::shared_ptr<Node> insert_node(const ouster::sdk::core::LidarFrame& lidar_frame,
                                      bool generate_point_cloud = false,
                                      double downsample_voxel_size = 0.05);

    /**
     * @brief Retrieve nodes whose timestamps exist in the source.
     *
     * - SamplingMode::KEY_FRAMES returns trajectory keyframe nodes whose
     *   timestamps match frame column timestamps in the OSF.
     * - SamplingMode::COLUMNS returns nodes for every frame column timestamp
     *   using the stored poses (not inserted into the trajectory map).
     *
     * @param[in] type Sampling mode used to select the node set.
     * @return A vector of valid nodes in timestamp order.
     */
    std::vector<std::shared_ptr<Node>> get_valid_nodes(
        SamplingMode type = SamplingMode::KEY_FRAMES);

    std::vector<uint64_t> get_timestamps(SamplingMode type) const;

    std::vector<core::Matrix4dR> get_poses(SamplingMode type);

    std::string input_osf_file;
    ouster::sdk::core::SensorInfo info;
    std::shared_ptr<ouster::sdk::core::XYZLut> xyz_lut;

    // timestamps to key node lookup map
    std::map<uint64_t, std::shared_ptr<Node>> timestamp_node_map;
    std::vector<TimestampsFrameIndex> timestamps_index_vec;
    std::vector<uint64_t> all_timestamps;
    std::vector<ouster::sdk::core::impl::PoseH> all_poses;

   private:
    /**
     * @brief Prepares trajectory data by processing an OSF file.
     *
     * @param[in] osf_file Path to the input OSF file.
     * @param[in] key_frame_distance Minimum distance between consecutive nodes.
     */
    void prepare_data(const std::string& osf_file, double node_gap);

    /**
     * @brief Evaluates interpolated poses for a sequence of timestamps.
     *
     * @param[in] ts Sequence of timestamps to evaluate.
     * @return std::vector<ouster::impl::PoseH> Array of interpolated poses.
     *
     * @throws std::runtime_error if Not enough nodes available for evaluation.
     */
    std::vector<std::shared_ptr<ouster::sdk::core::impl::PoseH>> evaluate(
        Eigen::Ref<const ouster::sdk::core::LidarFrame::Header<uint64_t>> timestamps) const;

    /**
     * @brief Updates the poses of all nodes in the trajectory.
     */
    void update_pose();
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

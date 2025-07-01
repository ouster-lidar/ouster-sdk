#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/pose_optimizer_enums.h"

namespace ouster {
namespace impl {
class PoseH;
}

namespace mapping {
class Node;

struct TimestampsScanIndex {
    uint64_t first_col_ts;  // Timestamp of the first column in the scan
    uint64_t last_col_ts;   // Timestamp of the last column in the scan
    uint32_t scan_index;    // Scan index of the OSF file
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
     * nullptr.
     */
    std::shared_ptr<Node> get_node_ts(uint64_t ts) const;

    /**
     * @brief Creates a new node with the specified timestamp.
     *
     * @param[in] ts Timestamp for the new node.
     * @param[in] generate_point_cloud Flag indicating whether a point cloud
     * should be generated.
     * @return std::shared_ptr<Node> Pointer to the newly created node.
     */
    std::shared_ptr<Node> create_node_ts(uint64_t ts,
                                         bool generate_point_cloud = false);

    /**
     * @brief Saves the trajectory data to an output OSF file.
     *
     * @param[in] out_osf File path for saving the trajectory data.
     */
    void save(const std::string& out_osf);

    /**
     * @brief Finds the first node whose timestamp is greater than the given
     * timestamp.
     *
     * @param[in] ts Timestamp to compare against.
     * @return std::map<uint64_t, std::shared_ptr<Node>>::iterator Iterator to
     * the node that satisfies the condition.
     *
     * @throws std::runtime_error if No node with a timestamp greater than
     * the input timestamp
     */
    std::map<uint64_t, std::shared_ptr<Node>>::iterator find_first_greater(
        uint64_t ts);

    /**
     * @brief Inserts a new node into the trajectory based on a LidarScan.
     *
     * @param[in] ls LidarScan to insert as a new node.
     * @param[in] generate_point_cloud Flag indicating whether a point cloud
     * should be generated.
     * @return std::shared_ptr<Node> Pointer to the newly inserted or existing
     * node.
     */
    std::shared_ptr<Node> insert_node(const ouster::LidarScan& ls,
                                      bool generate_point_cloud = false);

    std::vector<uint64_t> get_timestamps(SamplingMode type) const;

    std::vector<Eigen::Matrix<double, 4, 4>> get_poses(SamplingMode type);

    std::string input_osf_file;
    ouster::sensor::sensor_info info;
    ouster::XYZLut xyz_lut;

    // timestamps to key node lookup map
    std::map<uint64_t, std::shared_ptr<Node>> timestamp_node_map;
    std::vector<TimestampsScanIndex> timestamps_index_vec;
    std::vector<uint64_t> all_timestamps;
    std::vector<ouster::impl::PoseH> all_poses;

   private:
    /**
     * @brief Prepares trajectory data by processing an OSF file.
     *
     * @param[in] osf_file Path to the input OSF file.
     * @param[in] key_frame_distance Minimum distance between consecutive nodes.
     */
    void prepare_data(const std::string& osf_file, double key_frame_distance);

    /**
     * @brief Evaluates interpolated poses for a sequence of timestamps.
     *
     * @param[in] ts Sequence of timestamps to evaluate.
     * @return std::vector<ouster::impl::PoseH> Array of interpolated poses.
     *
     * @throws std::runtime_error if Not enough nodes available for evaluation.
     */
    std::vector<std::shared_ptr<ouster::impl::PoseH>> evaluate(
        Eigen::Ref<const ouster::LidarScan::Header<uint64_t>> ts) const;

    /**
     * @brief Updates the poses of all nodes in the trajectory.
     */
    void update_pose();
};

}  // namespace mapping
}  // namespace ouster

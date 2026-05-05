#define _ENABLE_EXTENDED_ALIGNED_STORAGE

#include "ouster/impl/trajectory.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "nonstd/optional.hpp"
#include "ouster/impl/logging.h"
#include "ouster/impl/transformation.h"
#include "ouster/impl/utils.h"
#include "ouster/open_source.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/osf/writer.h"
#include "ouster/pose_optimizer_node.h"
#include "ouster/pose_util.h"

using namespace ouster::sdk::core;
using ouster::sdk::core::impl::PoseH;
using ouster::sdk::core::impl::PoseV;

namespace ouster {
namespace sdk {
namespace mapping {

using ouster::sdk::core::logger;

Trajectory::Trajectory(const std::string& osf_file, double key_frame_distance)
    : input_osf_file(osf_file) {
    ouster::sdk::osf::Reader reader(osf_file);
    const auto lidar_sensors =
        reader.meta_store().find<ouster::sdk::osf::LidarSensor>();
    if (lidar_sensors.size() > 1) {
        throw std::runtime_error(
            "Multi sensor recording is not supported for now.");
    }
    for (const auto& kv : lidar_sensors) {
        const auto& value = kv.second;
        info = value->info();
        xyz_lut = ouster::sdk::core::make_xyz_lut(info, true);
        break;
    }
    logger().info("Preparing data ...");
    prepare_data(osf_file, key_frame_distance);
}

void Trajectory::prepare_data(const std::string& osf_file, double node_gap) {
    // open the source indexed and decode no fields since we arent using any
    auto source = ouster::sdk::open_source(osf_file, [](auto& req) {
        req.index = true;
        req.field_names = std::vector<std::string>();
    });

    uint32_t column_count = 0;
    uint32_t i = 0;
    for (const auto& sensor : source.sensor_info()) {
        column_count += source.individual_index()[i].size() * sensor->w();
        i++;
    }

    all_poses.reserve(column_count);
    all_timestamps.reserve(column_count);

    size_t scan_num = source.size();
    uint32_t scan_index = 0;
    for (const auto& scans : source) {
        for (const auto& ls : scans) {
            if (!ls) {
                continue;
            }

            auto scan_ts = ls->timestamp();
            int first_col = ls->get_first_valid_column();
            int last_col = ls->get_last_valid_column();
            const int num_cols = static_cast<int>(scan_ts.size());
            timestamps_index_vec.push_back(
                {scan_ts[first_col], scan_ts[last_col], scan_index});

            for (int col = 0; col < num_cols; ++col) {
                uint64_t col_timestamp = scan_ts[col];
                // Skip columns with zero timestamp (invalid data)
                if (col_timestamp == 0) {
                    continue;
                }
                Matrix4dR mat_row_major = ls->get_column_pose(col);
                all_timestamps.emplace_back(col_timestamp);
                all_poses.emplace_back(mat_row_major);
            }

            ++scan_index;

            uint64_t key_ts;
            Matrix4dR key_mat;
            if (scan_index == scan_num) {
                // For last scan, use last valid column with non-zero timestamp
                int key_col = last_col;
                while (key_col >= first_col && scan_ts[key_col] == 0) {
                    key_col--;
                }
                if (key_col >= first_col) {
                    key_ts = scan_ts[key_col];
                    key_mat = ls->get_column_pose(key_col);
                } else {
                    // No valid timestamps in this scan, skip keyframe creation
                    continue;
                }
            } else {
                // For other scans, use first valid column with non-zero
                // timestamp
                int key_col = first_col;
                while (key_col <= last_col && scan_ts[key_col] == 0) {
                    key_col++;
                }
                if (key_col <= last_col) {
                    key_ts = scan_ts[key_col];
                    key_mat = ls->get_column_pose(key_col);
                } else {
                    // No valid timestamps in this scan, skip keyframe creation
                    continue;
                }
            }
            PoseH key_pose(key_mat);

            bool is_last = (scan_index == scan_num);
            bool far_enough =
                timestamp_node_map.empty() ||
                pose_dist(timestamp_node_map.rbegin()->second->position,
                          key_pose.t()) >= node_gap;

            if (is_last || far_enough) {
                auto node = std::make_shared<Node>(key_ts, key_pose);
                timestamp_node_map.emplace(key_ts, std::move(node));
                if (is_last) {
                    logger().debug("Added final scan node at ts {}", key_ts);
                }
            }
        }
    }
}

void Trajectory::update_pose() {
    // Update each keyframe’s own pose
    for (auto& each : timestamp_node_map) {
        each.second->update_pose();
    }

    // Walk through each adjacent pair of valid keyframes
    auto it_before = timestamp_node_map.begin();
    while (std::next(it_before) != timestamp_node_map.end()) {
        // --- find next valid "before" keyframe timestamp ---
        bool found_before = false;
        uint64_t ts_before = 0;
        for (; it_before != timestamp_node_map.end(); ++it_before) {
            ts_before = it_before->second->ts;
            auto lower_bound_it = std::lower_bound(
                all_timestamps.begin(), all_timestamps.end(), ts_before);
            if (lower_bound_it != all_timestamps.end() &&
                *lower_bound_it == ts_before) {
                found_before = true;
                break;
            }
        }
        if (!found_before) {
            logger().warn(
                "Keyframe ts={} not in all_timestamps; skipping remaining "
                "segments",
                ts_before);
            break;
        }

        // --- find next valid "after" keyframe timestamp ---
        auto it_after = std::next(it_before);
        bool found_after = false;
        uint64_t ts_after = 0;
        for (; it_after != timestamp_node_map.end(); ++it_after) {
            ts_after = it_after->second->ts;
            auto lower_bound_it = std::lower_bound(
                all_timestamps.begin(), all_timestamps.end(), ts_after);
            if (lower_bound_it != all_timestamps.end() &&
                *lower_bound_it == ts_after) {
                found_after = true;
                break;
            }
        }
        if (!found_after) {
            logger().warn(
                "Keyframe ts={} not in all_timestamps; skipping this segment",
                ts_after);
            it_before = it_after;
            continue;
        }

        // --- compute indices via binary search results ---
        auto lb_before = std::lower_bound(all_timestamps.begin(),
                                          all_timestamps.end(), ts_before);
        auto lb_after = std::lower_bound(all_timestamps.begin(),
                                         all_timestamps.end(), ts_after);
        size_t idx_before = std::distance(all_timestamps.begin(), lb_before);
        size_t idx_after = std::distance(all_timestamps.begin(), lb_after);

        // if the "after" keyframe is the very last timestamp, include idx_after
        // in the segment
        bool is_last_keyframe = (std::next(lb_after) == all_timestamps.end());
        size_t idx_end = is_last_keyframe ? (idx_after + 1) : idx_after;

        // --- extract, deform, and write back the segment ---
        std::vector<PoseH> segment_poses;
        std::vector<uint64_t> segment_ts;
        segment_poses.reserve(idx_end - idx_before);
        segment_ts.reserve(idx_end - idx_before);

        for (size_t j = idx_before; j < idx_end; ++j) {
            segment_poses.push_back(all_poses[j]);
            segment_ts.push_back(all_timestamps[j]);
        }

        // Skip deformation if segment has <= 1 poses
        if (segment_poses.size() <= 1) {
            it_before = it_after;
            continue;
        }

        auto new_segment = deform_trajectory_relative_poses(
            segment_poses, segment_ts, PoseH(it_before->second->get_pose()),
            PoseH(it_after->second->get_pose()));

        for (size_t j = idx_before; j < idx_end; ++j) {
            all_poses[j] = std::move(new_segment[j - idx_before]);
        }

        // advance to the next pair
        it_before = it_after;
    }
}

std::shared_ptr<Node> Trajectory::get_node_ts(uint64_t timestamp) const {
    // If Node already exist, return the existed Node
    auto it = timestamp_node_map.find(timestamp);
    if (it != timestamp_node_map.end()) {
        return it->second;
    } else {
        return nullptr;
    }
}

std::shared_ptr<Node> Trajectory::create_node_ts(uint64_t timestamp,
                                                 bool generate_point_cloud,
                                                 double downsample_voxel_size) {
    auto source = ouster::sdk::open_source(
        input_osf_file,
        [generate_point_cloud](auto& config) {
            config.index = true;
            if (generate_point_cloud) {
                config.field_names = std::vector<std::string>({"RANGE"});
            } else {
                config.field_names = std::vector<std::string>({});
            }
        },
        /* collate = false */ false);

    nonstd::optional<uint64_t> start_index_opt;
    nonstd::optional<uint64_t> end_index_opt;

    for (const auto& each : timestamps_index_vec) {
        const uint64_t first_ts = each.first_col_ts;
        const uint64_t last_ts = each.last_col_ts;
        const uint32_t idx = each.scan_index;

        if (timestamp >= first_ts) {
            start_index_opt = idx;
        }
        if (timestamp <= last_ts) {
            end_index_opt = idx;
            break;
        }
    }

    const uint64_t source_size = source.size();
    if (source_size == 0u) {
        logger().error(
            "Trajectory::create_node_ts cannot create node: OSF source is "
            "empty");
        return nullptr;
    }

    uint64_t start_index = start_index_opt.value_or(0u);
    uint64_t end_index = end_index_opt.value_or(source_size - 1u);

    if (start_index >= source_size) {
        start_index = source_size - 1u;
    }
    if (end_index >= source_size) {
        end_index = source_size - 1u;
    }
    if (start_index > end_index) {
        start_index = end_index;
    }

    auto part_osf = source[{start_index, end_index + 1u}];

    for (const auto& scans : part_osf) {
        for (auto& ls : scans) {
            if (!ls) {
                continue;
            }

            uint64_t ls_ts = ls->get_first_valid_column_timestamp();
            if (ls_ts == timestamp) {
                auto node = this->insert_node(*ls, generate_point_cloud,
                                              downsample_voxel_size);
                return node;
            }
        }
    }

    auto interpolate_from_all =
        [&](uint64_t query_ts) -> std::shared_ptr<Node> {
        auto lb = std::lower_bound(all_timestamps.begin(), all_timestamps.end(),
                                   query_ts);
        if (lb == all_timestamps.end()) {
            logger().warn(
                "Trajectory::create_node_ts unable to interpolate ts {}: no "
                "timestamp >= query",
                query_ts);
            return nullptr;
        }

        size_t idx_after =
            static_cast<size_t>(std::distance(all_timestamps.begin(), lb));

        if (*lb == query_ts) {
            const PoseH& exact_pose = all_poses[idx_after];
            auto node = std::make_shared<Node>(query_ts, exact_pose);
            timestamp_node_map.insert({query_ts, node});
            return node;
        }

        if (idx_after == 0) {
            logger().warn(
                "Trajectory::create_node_ts unable to interpolate ts {}: no "
                "timestamp before query",
                query_ts);
            return nullptr;
        }

        size_t idx_before = idx_after - 1;
        uint64_t ts_before = all_timestamps[idx_before];
        uint64_t ts_after = all_timestamps[idx_after];
        if (ts_after == ts_before) {
            logger().warn(
                "Trajectory::create_node_ts unable to interpolate ts {}: "
                "duplicate timestamps ({})",
                query_ts, ts_after);
            return nullptr;
        }

        double alpha = static_cast<double>(query_ts - ts_before) /
                       static_cast<double>(ts_after - ts_before);
        const PoseH pose_before = all_poses[idx_before];
        const PoseH pose_after = all_poses[idx_after];

        PoseH relative = PoseH(pose_before.inverse()) * pose_after;
        PoseV delta = relative.log();
        PoseV scaled_delta = delta * alpha;
        PoseH interpolated_pose = pose_before * scaled_delta.exp();

        auto node = std::make_shared<Node>(query_ts, interpolated_pose);
        timestamp_node_map.insert({query_ts, node});
        return node;
    };

    if (!generate_point_cloud) {
        auto node = interpolate_from_all(timestamp);
        return node;
    }

    std::string error_str = "Cannot generate point cloud based on " +
                            std::to_string(timestamp) +
                            " as no LidarScan message found";
    throw std::runtime_error(error_str);
}

void Trajectory::save(const std::string& out_osf) {
    ouster::sdk::osf::Writer writer(out_osf);
    update_pose();

    ouster::sdk::osf::Reader reader(input_osf_file);
    int id = writer.add_sensor(info);

    for (const auto& m : reader.messages()) {
        if (m.is<ouster::sdk::osf::LidarScanStream>()) {
            std::unique_ptr<ouster::sdk::core::LidarScan> ls =
                m.decode_msg<ouster::sdk::osf::LidarScanStream>();

            int first_col = ls->get_first_valid_column();

            // Find first column with valid (non-zero) timestamp
            int first_valid_ts_col = first_col;
            while (first_valid_ts_col <= ls->get_last_valid_column() &&
                   ls->timestamp()(first_valid_ts_col) == 0) {
                first_valid_ts_col++;
            }

            // Skip this scan if no valid timestamps found
            if (first_valid_ts_col > ls->get_last_valid_column()) {
                continue;
            }

            if (ls->timestamp()(first_valid_ts_col) <
                    (*timestamp_node_map.begin()).first ||
                ls->timestamp()(first_valid_ts_col) >
                    (*timestamp_node_map.rbegin()).first) {
                continue;
            }

            int last_col = ls->get_last_valid_column();
            auto poses = this->evaluate(ls->timestamp());
            LidarScan ls_copy = *ls;

            for (int col = first_col; col <= last_col; ++col) {
                if (ls->timestamp()(col) == 0) {
                    ls_copy.set_column_pose(col, Matrix4dR::Identity());
                    continue;
                }
                if (poses.at(col) != nullptr) {
                    Matrix4dR pose(*(poses.at(col)));
                    ls_copy.set_column_pose(col, pose);
                }
            }

            writer.save(
                id, ls_copy,
                ouster::sdk::osf::ts_t(ls->timestamp()(first_valid_ts_col)));
        }
    }
}

std::vector<std::shared_ptr<PoseH>> Trajectory::evaluate(
    Eigen::Ref<const ouster::sdk::core::LidarScan::Header<uint64_t>> timestamps)
    const {
    const int num_timestamps = timestamps.size();
    std::vector<std::shared_ptr<PoseH>> results(num_timestamps);

    if (timestamp_node_map.size() < 2) {
        throw std::runtime_error("Not enough nodes available for evaluation.");
    }

    uint64_t first_ts = timestamps[0];
    uint64_t last_ts = timestamps[num_timestamps - 1];
    if (first_ts < all_timestamps.front() || last_ts > all_timestamps.back()) {
        logger().error("Timestamp(s) out of range: {} - {}", first_ts, last_ts);
        // fall back to identity for everything
        for (int i = 0; i < num_timestamps; ++i) {
            results[i] = std::make_shared<PoseH>(Eigen::Matrix4d::Identity());
            continue;
        }

        return results;
    }

    // find the first iter
    auto it_ts = std::lower_bound(all_timestamps.begin(), all_timestamps.end(),
                                  first_ts);
    // now for each scan timestamp, forward that iterator by i
    for (int i = 0; i < num_timestamps; ++i) {
        if (timestamps[i] == 0) {
            results[i] = std::make_shared<PoseH>(Eigen::Matrix4d::Identity());
            continue;
        }
        auto it = it_ts + i;
        // safety clamp in case of off‐by‐one
        if (it >= all_timestamps.end()) {
            it = all_timestamps.end() - 1;
        }
        auto idx = std::distance(all_timestamps.begin(), it);
        results[i] = std::make_shared<PoseH>(all_poses[idx]);
    }

    return results;
}

std::shared_ptr<Node> Trajectory::insert_node(const LidarScan& lidar_scan,
                                              bool generate_point_cloud,
                                              double downsample_voxel_size) {
    uint64_t lidar_scan_ts = lidar_scan.get_first_valid_column_timestamp();
    int first_col = lidar_scan.get_first_valid_column();

    // Prepare downsampled point cloud by dewarping and downsampling
    auto prepare_downsampled_pts = [&]() -> Eigen::ArrayX3d {
        const auto range =
            lidar_scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
        const int height = static_cast<int>(range.rows());
        const int width = static_cast<int>(range.cols());

        const auto status = lidar_scan.status();
        const int start_col = lidar_scan.get_first_valid_column();
        const int stop_col = lidar_scan.get_last_valid_column();
        if (start_col < 0 || stop_col < 0 || stop_col < start_col) {
            return Eigen::ArrayX3d();
        }

        size_t valid_count = 0;
        for (int col = start_col; col <= stop_col; ++col) {
            if (status[col] == 0) {
                continue;
            }
            for (int row = 0; row < height; ++row) {
                if (range(row, col) != 0u) {
                    ++valid_count;
                }
            }
        }

        if (valid_count == 0) {
            return Eigen::ArrayX3d();
        }

        const PointCloudXYZd pts = cartesian(lidar_scan, xyz_lut);

        const PoseH first_pose(lidar_scan.get_column_pose(first_col));
        const PoseH first_pose_inv(first_pose.inverse());
        MatrixX16dR rel_poses(width, 16);
        for (int col = 0; col < width; ++col) {
            const PoseH pose_c(
                lidar_scan.get_column_pose(static_cast<uint32_t>(col)));
            const PoseH rel(first_pose_inv * pose_c);
            Eigen::Map<Matrix4dR>(rel_poses.row(col).data()) = Matrix4dR(rel);
        }

        PointCloudXYZd dewarped_pts(pts.rows(), pts.cols());
        ouster::sdk::core::dewarp<double>(dewarped_pts, pts, rel_poses);

        // change to icp downsample input shape and type. We just store
        // points in ArrayX3d. This will be used for viz displace and icp
        // matching. Remove invalid col which means remove zeros pts
        Eigen::ArrayX3d out(static_cast<Eigen::Index>(valid_count), 3);
        Eigen::Index k = 0;
        for (int col = start_col; col <= stop_col; ++col) {
            if (status[col] == 0) {
                continue;
            }
            for (int row = 0; row < height; ++row) {
                if (range(row, col) == 0u) {
                    continue;
                }
                const Eigen::Index idx =
                    static_cast<Eigen::Index>(row) * width + col;
                out.row(k++) = dewarped_pts.row(idx).array();
            }
        }

        out = run_kiss_icp_downsample(out, downsample_voxel_size);
        return out;
    };

    auto it = timestamp_node_map.find(lidar_scan_ts);
    if (it != timestamp_node_map.end()) {
        if (generate_point_cloud) {
            it->second->downsampled_pts = prepare_downsampled_pts();
        }
        return it->second;
    }

    Eigen::ArrayX3d pts = Eigen::ArrayX3d();
    if (generate_point_cloud) {
        pts = prepare_downsampled_pts();
    }
    Matrix4dR mat = lidar_scan.get_column_pose(first_col);
    std::shared_ptr<Node> new_node =
        std::make_shared<Node>(lidar_scan_ts, Eigen::Matrix4d(mat));
    if (generate_point_cloud) {
        new_node->downsampled_pts = pts;
    }

    timestamp_node_map.insert({lidar_scan_ts, new_node});

    return new_node;
}

std::vector<uint64_t> Trajectory::get_timestamps(SamplingMode type) const {
    std::vector<uint64_t> res_ts;
    if (type == SamplingMode::KEY_FRAMES) {
        res_ts.reserve(timestamp_node_map.size());
        for (const auto& kv : timestamp_node_map) {
            res_ts.push_back(kv.first);
        }
    } else if (type == SamplingMode::COLUMNS) {
        res_ts.reserve(all_timestamps.size());
        for (const auto& timestamp : all_timestamps) {
            res_ts.push_back(timestamp);
        }
    } else {
        logger().error(
            "Invalid SamplingMode. Use SamplingMode::KEY_FRAMES or "
            "SamplingMode::COLUMNS.");
        throw std::invalid_argument("Invalid SamplingMode: ");
    }
    return res_ts;
}

std::vector<Eigen::Matrix<double, 4, 4>> Trajectory::get_poses(
    SamplingMode type) {
    update_pose();
    std::vector<Eigen::Matrix<double, 4, 4>> re_poses;
    if (type == SamplingMode::KEY_FRAMES) {
        re_poses.reserve(timestamp_node_map.size());
        for (const auto& kv : timestamp_node_map) {
            re_poses.push_back(kv.second->get_pose());
        }
    } else if (type == SamplingMode::COLUMNS) {
        re_poses.reserve(all_poses.size());
        for (const auto& pose : all_poses) {
            re_poses.push_back(pose);
        }
    } else {
        logger().error(
            "Invalid SamplingMode. Use SamplingMode::KEY_FRAMES or "
            "SamplingMode::COLUMNS.");
        throw std::invalid_argument("Invalid SamplingMode: ");
    }
    return re_poses;
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

#define _ENABLE_EXTENDED_ALIGNED_STORAGE

#include "ouster/impl/trajectory.h"

#include <chrono>

#include "ouster/impl/logging.h"
#include "ouster/impl/transformation.h"
#include "ouster/impl/utils.h"
#include "ouster/open_source.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/osf/writer.h"
#include "ouster/pose_optimizer_node.h"
#include "ouster/pose_util.h"

namespace ouster {
namespace mapping {

using sensor::logger;

Trajectory::Trajectory(const std::string& osf_file, double key_frame_distance)
    : input_osf_file(osf_file) {
    ouster::osf::Reader reader(osf_file);
    for (const auto& [key, value] :
         reader.meta_store().find<ouster::osf::LidarSensor>()) {
        info = value->info();
        xyz_lut = ouster::make_xyz_lut(info, true);
        break;
    }
    if (reader.meta_store().find<ouster::osf::LidarSensor>().size() > 1) {
        logger().warn(
            "Multiple LidarSensor messages found in OSF file. "
            "Using the first one to process.");
    }
    logger().info("Preparing data ...");
    prepare_data(osf_file, key_frame_distance);
}

void Trajectory::prepare_data(const std::string& osf_file, double node_gap) {
    // open the source indexed and decode no fields since we arent using any
    auto source = ouster::open_source(osf_file, [](auto& r) {
        r.index = true;
        r.field_names = std::vector<std::string>();
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
    for (auto& scans : source) {
        for (auto& ls : scans) {
            if (!ls) {
                continue;
            }

            auto scan_ts = ls->timestamp();
            int first_col = first_valid_col(*ls);
            int last_col = last_valid_col(*ls);
            const int num_cols = static_cast<int>(scan_ts.size());
            timestamps_index_vec.push_back(
                {scan_ts[first_col], scan_ts[last_col], scan_index});

            auto pose_ptr = ls->pose().get<double>();
            for (int col = 0; col < num_cols; ++col) {
                Eigen::Matrix4d mat;
                std::memcpy(mat.data(), pose_ptr + 4 * 4 * col, sizeof(mat));
                mat.transposeInPlace();

                all_timestamps.emplace_back(scan_ts[col]);
                all_poses.emplace_back(mat);
            }

            ++scan_index;

            uint64_t key_ts;
            Eigen::Matrix4d key_mat;
            if (scan_index == scan_num) {
                key_ts = scan_ts[last_col];
                std::memcpy(key_mat.data(), ls->pose().subview(last_col).get(),
                            sizeof(key_mat));
            } else {
                key_ts = scan_ts[first_col];
                std::memcpy(key_mat.data(), ls->pose().subview(first_col).get(),
                            sizeof(key_mat));
            }
            key_mat.transposeInPlace();
            ouster::impl::PoseH key_pose(key_mat);

            bool is_last = (scan_index == scan_num);
            bool far_enough =
                timestamp_node_map.empty() ||
                pose_dist(timestamp_node_map.rbegin()->second->position,
                          key_pose.t()) >= node_gap;

            if (is_last || far_enough) {
                Eigen::Array<double, Eigen::Dynamic, 3> dummy_pts(0, 3);
                auto node = std::make_shared<Node>(key_ts, key_pose, dummy_pts);
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
            auto lb = std::lower_bound(all_timestamps.begin(),
                                       all_timestamps.end(), ts_before);
            if (lb != all_timestamps.end() && *lb == ts_before) {
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
            auto lb = std::lower_bound(all_timestamps.begin(),
                                       all_timestamps.end(), ts_after);
            if (lb != all_timestamps.end() && *lb == ts_after) {
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
        std::vector<ouster::impl::PoseH> segment_poses;
        std::vector<uint64_t> segment_ts;
        segment_poses.reserve(idx_end - idx_before);
        segment_ts.reserve(idx_end - idx_before);

        for (size_t j = idx_before; j < idx_end; ++j) {
            segment_poses.push_back(all_poses[j]);
            segment_ts.push_back(all_timestamps[j]);
        }

        auto new_segment = deform_trajectory_relative_poses(
            segment_poses, segment_ts, it_before->second->get_pose(),
            it_after->second->get_pose());

        for (size_t j = idx_before; j < idx_end; ++j) {
            all_poses[j] = std::move(new_segment[j - idx_before]);
        }

        // advance to the next pair
        it_before = it_after;
    }
}

std::shared_ptr<Node> Trajectory::get_node_ts(uint64_t ts) const {
    // If Node already exist, return the existed Node
    auto it = timestamp_node_map.find(ts);
    if (it != timestamp_node_map.end())
        return it->second;
    else
        return nullptr;
}

std::shared_ptr<Node> Trajectory::create_node_ts(uint64_t ts,
                                                 bool generate_point_cloud) {
    auto source = ouster::open_source(
        input_osf_file,
        [generate_point_cloud](auto& r) {
            r.index = true;
            if (generate_point_cloud) {
                r.field_names = std::vector<std::string>({"RANGE"});
            } else {
                r.field_names = std::vector<std::string>({});
            }
        },
        /* collate = false */ false);

    uint64_t start_index = 0;
    uint64_t end_index = 0;

    for (const auto& each : timestamps_index_vec) {
        const uint64_t first_ts = each.first_col_ts;
        const uint64_t last_ts = each.last_col_ts;
        const uint32_t idx = each.scan_index;

        if (ts >= first_ts) {
            start_index = idx;
        }
        if (ts <= last_ts) {
            end_index = idx;
            break;
        }
    }

    if (!start_index) {
        throw std::runtime_error(std::string("Can't find any scan before ts ") +
                                 std::to_string(ts) +
                                 ". Fail to create new Node");
    }

    if (!end_index) {
        throw std::runtime_error(std::string("Can't find any scan after ts ") +
                                 std::to_string(ts) +
                                 ". Fail to create new Node");
    }

    auto part_osf = source[{start_index, end_index + 1}];

    for (const auto& scans : part_osf) {
        for (auto& ls : scans) {
            if (!ls) {
                continue;
            }

            uint64_t ls_ts = ls->get_first_valid_column_timestamp();
            if (ls_ts == ts) {
                auto node = this->insert_node(*ls, generate_point_cloud);
                return node;
            }
        }
    }

    if (generate_point_cloud) {
        std::string error_str = "Cannot generate point cloud based on " +
                                std::to_string(ts) +
                                " as no LidarScan message found";
        throw std::runtime_error(error_str);
    }

    ouster::impl::PoseH prev_pose = ouster::impl::PoseH{};
    uint64_t prev_ts = 0;
    for (const auto& scans : part_osf) {
        for (auto& ls : scans) {
            if (!ls) {
                continue;
            }

            uint64_t curr_ts = ls->get_first_valid_column_timestamp();
            uint64_t curr_last_ts = ls->get_last_valid_column_timestamp();

            if (curr_ts < ts) {
                int first_col = first_valid_col(*ls);
                Eigen::Matrix4d mat;
                std::memcpy(mat.data(), ls->pose().subview(first_col).get(),
                            sizeof(double) * 16);
                Eigen::Matrix4d mat_t = mat.transpose();
                prev_pose = ouster::impl::PoseH(mat_t);
                prev_ts = curr_ts;
            }
            if (curr_last_ts >= ts) {
                int last_col = last_valid_col(*ls);
                Eigen::Matrix4d mat;
                std::memcpy(mat.data(), ls->pose().subview(last_col).get(),
                            sizeof(double) * 16);
                Eigen::Matrix4d mat_t = mat.transpose();
                ouster::impl::PoseH curr_pose = ouster::impl::PoseH(mat_t);

                std::vector<Eigen::Matrix<double, 4, 4>> poses = {prev_pose,
                                                                  curr_pose};
                std::vector<uint64_t> timestamps = {prev_ts, curr_last_ts};
                std::vector<uint64_t> inquiry_timestamps = {ts};
                auto perturb_poses = ouster::core::interp_pose<uint64_t>(
                    inquiry_timestamps, timestamps, poses);
                auto empty_pts = Eigen::Array<double, Eigen::Dynamic, 3>();
                auto node = std::make_shared<Node>(
                    ts, impl::PoseH(perturb_poses[0]), empty_pts);
                timestamp_node_map.insert({ts, node});

                return node;
            }
        }
    }

    return nullptr;
}

void Trajectory::save(const std::string& out_osf) {
    ouster::osf::Writer writer(out_osf);
    update_pose();

    ouster::osf::Reader reader(input_osf_file);
    int id = writer.add_sensor(info);

    for (const auto& m : reader.messages()) {
        if (m.is<ouster::osf::LidarScanStream>()) {
            std::unique_ptr<ouster::LidarScan> ls =
                m.decode_msg<ouster::osf::LidarScanStream>();

            int first_col = first_valid_col(*ls);
            if (ls->timestamp()(first_col) <
                    (*timestamp_node_map.begin()).first ||
                ls->timestamp()(first_col) >
                    (*timestamp_node_map.rbegin()).first) {
                continue;
            }
            int last_col = last_valid_col(*ls);

            auto poses = this->evaluate(ls->timestamp());
            ouster::LidarScan ls_copy = *ls;

            for (int col = first_col; col <= last_col; ++col) {
                if (poses.at(col) != nullptr) {
                    ouster::impl::PoseH pose = poses.at(col)->transpose();
                    std::memcpy(ls_copy.pose().subview(col).get(), pose.data(),
                                sizeof(double) * 16);
                }
            }

            writer.save(id, ls_copy,
                        ouster::osf::ts_t(ls->timestamp()(first_col)));
        }
    }
}

std::map<uint64_t, std::shared_ptr<Node>>::iterator
Trajectory::find_first_greater(uint64_t ts) {
    auto iter = timestamp_node_map.upper_bound(ts);
    if (iter == timestamp_node_map.end()) {
        throw std::runtime_error(
            "No node with a timestamp greater than the input timestamp.");
    }
    return iter;
}

std::vector<std::shared_ptr<ouster::impl::PoseH>> Trajectory::evaluate(
    Eigen::Ref<const ouster::LidarScan::Header<uint64_t>> ts) const {
    const int n = ts.size();
    std::vector<std::shared_ptr<ouster::impl::PoseH>> results(n);

    if (timestamp_node_map.size() < 2) {
        throw std::runtime_error("Not enough nodes available for evaluation.");
    }

    uint64_t first_ts = ts[0];
    uint64_t last_ts = ts[n - 1];
    if (first_ts < all_timestamps.front() || last_ts > all_timestamps.back()) {
        logger().error("Timestamp(s) out of range: {} - {}", first_ts, last_ts);
        // fall back to identity for everything
        for (int i = 0; i < n; ++i) {
            results[i] = std::make_shared<ouster::impl::PoseH>(
                Eigen::Matrix4d::Identity());
            continue;
        }

        return results;
    }

    // find the first iter
    auto it_ts = std::lower_bound(all_timestamps.begin(), all_timestamps.end(),
                                  first_ts);
    // now for each scan timestamp, forward that iterator by i
    for (int i = 0; i < n; ++i) {
        auto it = it_ts + i;
        // safety clamp in case of off‐by‐one
        if (it >= all_timestamps.end()) {
            it = all_timestamps.end() - 1;
        }
        auto idx = std::distance(all_timestamps.begin(), it);
        results[i] = std::make_shared<ouster::impl::PoseH>(all_poses[idx]);
    }

    return results;
}

std::shared_ptr<Node> Trajectory::insert_node(const ouster::LidarScan& ls,
                                              bool generate_point_cloud) {
    uint64_t ls_ts = ls.get_first_valid_column_timestamp();
    int first_col = first_valid_col(ls);

    auto it = timestamp_node_map.find(ls_ts);
    if (it != timestamp_node_map.end()) {
        if (generate_point_cloud) {
            auto pts = cartesian(ls, xyz_lut);
            it->second->pts = run_KISS_ICP_downsample(pts);
        }
        return it->second;
    }

    Eigen::Matrix4d mat;
    std::memcpy(mat.data(), ls.pose().subview(first_col).get(),
                sizeof(double) * 16);
    Eigen::Matrix4d mat_t = mat.transpose();
    ouster::impl::PoseH pose = ouster::impl::PoseH(mat_t);

    Eigen::Array<double, Eigen::Dynamic, 3> pts =
        Eigen::Array<double, 0, 3, 0>();
    if (generate_point_cloud) {
        pts = cartesian(ls, xyz_lut);
        pts = run_KISS_ICP_downsample(pts);
    }
    std::shared_ptr<Node> new_node = std::make_shared<Node>(ls_ts, pose, pts);

    timestamp_node_map.insert({ls_ts, new_node});

    return new_node;
}

std::vector<uint64_t> Trajectory::get_timestamps(SamplingMode type) const {
    std::vector<uint64_t> res_ts;
    if (type == SamplingMode::KEY_FRAMES) {
        res_ts.reserve(timestamp_node_map.size());
        for (const auto& [timestamp, node] : timestamp_node_map) {
            res_ts.push_back(timestamp);
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
        for (const auto& [timestamp, node] : timestamp_node_map) {
            re_poses.push_back(node->get_pose());
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
}  // namespace ouster

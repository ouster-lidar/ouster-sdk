#define _ENABLE_EXTENDED_ALIGNED_STORAGE

#include "ouster/pose_optimizer.h"

#include <ceres/ceres.h>

#include <chrono>

#include "nonstd/optional.hpp"
#include "ouster/impl/absolute_pose_constraint.h"
#include "ouster/impl/abstract_constraint.h"
#include "ouster/impl/logging.h"
#include "ouster/impl/point_to_point_constraint.h"
#include "ouster/impl/pose_to_pose_constraint.h"
#include "ouster/impl/trajectory.h"
#include "ouster/impl/transformation.h"
#include "ouster/impl/utils.h"
#include "ouster/lidar_scan.h"
#include "ouster/open_source.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_scan.h"
#include "ouster/osf/writer.h"
#include "ouster/pose_conversion.h"
#include "ouster/pose_optimizer_node.h"

namespace ouster {
namespace mapping {
namespace {

ouster::impl::PoseH run_icp(std::shared_ptr<Node> node_first,
                            std::shared_ptr<Node> node_second) {
    // TODO use dewarp
    ouster::impl::PoseH pose1 = node_first->get_pose();
    Points transformed_points1 = transform_points(node_first->pts, pose1);

    ouster::impl::PoseH pose2 = node_second->get_pose();
    Points transformed_points2 = transform_points(node_second->pts, pose2);

    Eigen::Matrix4d icp_posed =
        run_KISS_ICP_matching(transformed_points1, transformed_points2);

    ouster::impl::PoseH icp_p = ouster::impl::PoseH(icp_posed);
    ouster::impl::PoseH new_first_pose = pose1 * ouster::impl::PoseH(icp_p);
    ouster::impl::PoseH diff =
        ouster::impl::PoseH(new_first_pose.inverse()) * pose2;
    return diff;
}

ceres::LossFunction* create_loss_function(LossFunction loss_func,
                                          double scale) {
    switch (loss_func) {
        case LossFunction::HuberLoss:
            return new ceres::HuberLoss(scale);
        case LossFunction::CauchyLoss:
            return new ceres::CauchyLoss(scale);
        case LossFunction::SoftLOneLoss:
            return new ceres::SoftLOneLoss(scale);
        case LossFunction::ArctanLoss:
            return new ceres::ArctanLoss(scale);
        case LossFunction::TrivialLoss:
            return nullptr;
        default:
            throw std::invalid_argument(
                "Unknown loss function. Available options are: "
                "HuberLoss, CauchyLoss, SoftLOneLoss, ArctanLoss, "
                "TrivialLoss.");
    }
}

const Eigen::IOFormat EigenMatrixPrintFmt(
    /* precision    */ 9,
    /* flags        */ 0,  // align columns
    /* coeff_sep    */ ", ",
    /* row_sep      */ "\n",
    /* mat_prefix   */ "  [",
    /* mat_suffix   */ "]",
    /* row_prefix   */ "",
    /* row_suffix   */ "");

}  // namespace

using sensor::logger;

class PoseOptimizer::Impl {
   public:
    ceres::Problem problem;
    ceres::Solver::Options options;
    double traj_rotation_weight;
    double traj_translation_weight;
    uint64_t default_max_num_iterations;
    uint32_t constraint_count_;
    Trajectory traj;
    ceres::LossFunction* loss_function = nullptr;

    Impl(const SolverConfig& solver_options, const std::string& osf_filename) {
        options.max_num_iterations = solver_options.max_num_iterations;
        options.function_tolerance = solver_options.function_tolerance;
        options.gradient_tolerance = solver_options.gradient_tolerance;
        options.parameter_tolerance = solver_options.parameter_tolerance;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = solver_options.process_printout;
        traj_rotation_weight = solver_options.traj_rotation_weight;
        traj_translation_weight = solver_options.traj_translation_weight;
        default_max_num_iterations = solver_options.max_num_iterations;
        constraint_count_ = 0;

        logger().info("Initializing Pose Optimizer ...");

        try {
            if (solver_options.loss_function == LossFunction::TrivialLoss) {
                // TrivialLoss ignores the scale entirely
                loss_function =
                    create_loss_function(LossFunction::TrivialLoss, 0.0);
                logger().info("Using TrivialLoss function");
            } else {
                // All other losses do use the scale
                loss_function = create_loss_function(
                    solver_options.loss_function, solver_options.loss_scale);
                logger().info("Using loss function: {} with scale: {}",
                              to_string(solver_options.loss_function),
                              solver_options.loss_scale);
            }

        } catch (const std::exception& e) {
            std::cerr << "Error creating loss function: " << e.what()
                      << std::endl;
            throw;
        }

        traj = Trajectory(osf_filename, solver_options.key_frame_distance);
        for (auto node : traj.timestamp_node_map) {
            add_node_to_problem(node.second);
        }
    }

    void add_node_to_problem(std::shared_ptr<Node> node) {
        if (!node) {
            return;
        }
#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1
        ceres::Manifold* quaternion_parameterization =
            new ceres::EigenQuaternionManifold();
#else
        ceres::LocalParameterization* quaternion_parameterization =
            new ceres::QuaternionParameterization();
#endif
        if (!problem.HasParameterBlock(node->rotation.coeffs().data())) {
            problem.AddParameterBlock(node->rotation.coeffs().data(), 4,
                                      quaternion_parameterization);
        } else {
            delete quaternion_parameterization;
        }

        if (!problem.HasParameterBlock(node->position.data())) {
            problem.AddParameterBlock(node->position.data(), 3);
        }
    }

    void add_constraint(impl::AbstractConstraint& constraint) {
        constraint.add_to_problem(problem, loss_function);
    }

    void add_node_neighbours_constraints(std::shared_ptr<Node> node) {
        auto node_iter = traj.find_first_greater(node->ts);
        if (node_iter == traj.timestamp_node_map.end()) {
            logger().error("Error : Can't create a node for timestamp {}",
                           node->ts);
            return;
        } else {
            // Add pose to pose constraint of the new pose and the next node in
            // traj
            ouster::impl::PoseH diff =
                node->get_pose().inverse() * (node_iter->second)->get_pose();
            ouster::impl::PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraint constraint(
                node, node_iter->second, diff_q.r(), diff_q.t(),
                traj_rotation_weight, traj_translation_weight);

            add_constraint(constraint);
        }
        if (std::distance(traj.timestamp_node_map.begin(), node_iter) > 2) {
            auto node_prev = *(std::prev(node_iter, 2));

            ouster::impl::PoseH diff =
                node_prev.second->get_pose().inverse() * node->get_pose();
            ouster::impl::PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraint constraint(
                node_prev.second, node, diff_q.r(), diff_q.t(),
                traj_rotation_weight, traj_translation_weight);

            add_constraint(constraint);
        }
    }

    std::shared_ptr<Node> get_or_create_node_ts(
        uint64_t ts, bool generate_point_cloud = false) {
        std::shared_ptr<Node> node = traj.get_node_ts(ts);
        if (node) {
            // Find the existed node and update the point cloud
            if (generate_point_cloud) {
                auto new_node = traj.create_node_ts(ts, generate_point_cloud);
                node->pts = new_node->pts;
            }
            return node;
        } else {
            // Create new node to the trajectory
            node = traj.create_node_ts(ts, generate_point_cloud);

            if (node) {
                // Add its neighbour to traj
                add_node_to_problem(node);
                add_node_neighbours_constraints(node);
            } else {
                std::string msg = "Failed to create the node from timestamp " +
                                  std::to_string(ts) +
                                  ". The timestamp may be invalid or not "
                                  "correspond to a scan's first valid column.";

                uint64_t min_ts = traj.all_timestamps.front();
                uint64_t max_ts = traj.all_timestamps.back();

                if (ts < min_ts || ts > max_ts) {
                    msg +=
                        " The timestamp is outside "
                        "the range of the OSF file [" +
                        std::to_string(min_ts) + ", " + std::to_string(max_ts) +
                        "].";
                }

                throw std::runtime_error(msg);
            }
        }
        return node;
    }

    bool add_pose_to_pose_constraint(uint64_t ts1, uint64_t ts2,
                                     const ouster::impl::PoseH& diff,
                                     double rotation_weight,
                                     double translation_weight) {
        auto node1 = get_or_create_node_ts(ts1);
        auto node2 = get_or_create_node_ts(ts2);

        if (!node1) {
            throw std::runtime_error(
                "Failed to create the first node from timestamp " +
                std::to_string(ts1) +
                ". The timestamp may be invalid or not correspond to a scan's "
                "first valid column.");
        }

        if (!node2) {
            throw std::runtime_error(
                "Failed to create the second node from timestamp " +
                std::to_string(ts2) +
                ". The timestamp may be invalid or not correspond to a scan's "
                "first valid column.");
        }

        impl::PoseToPoseConstraint constraint(
            node1, node2, diff, rotation_weight, translation_weight);
        add_constraint(constraint);

        std::ostringstream oss;
        oss << diff.format(EigenMatrixPrintFmt);
        auto diff_str = oss.str();

        ++constraint_count_;
        if (this->options.minimizer_progress_to_stdout) {
            logger().info(
                "\n\n  Added Constraint {}\n  Type: RELATIVE_POSE_TO_POSE\n  "
                "The user input pose diff matrix: \n{}\n",
                constraint_count_, diff_str);
        }
        return true;
    }

    bool add_absolute_pose_constraint(
        uint64_t ts, const ouster::impl::PoseH& target_pose,
        const ouster::impl::PoseH& diff,
        const std::array<double, 3>& rotation_weights,
        const std::array<double, 3>& translation_weights) {
        auto node1 = get_or_create_node_ts(ts);
        if (!node1) {
            throw std::runtime_error(
                "Failed to create the node from timestamp " +
                std::to_string(ts) +
                ". The timestamp may be invalid or its outside the range of "
                "OSF file.");
        }
        // This target node only for constraint usage. Won't be added to the
        // trajectory
        auto node_target = std::make_shared<Node>(ts, target_pose);
        // add_node_to_problem(node_target);
        if (diff.isIdentity()) {
            impl::AbsolutePoseConstraint constraint(
                node1, node_target, rotation_weights, translation_weights);
            add_constraint(constraint);
        } else {
            ouster::impl::PoseQ pose_q = diff.log().q();
            impl::AbsolutePoseConstraint constraint(
                node1, node_target, pose_q.r(), pose_q.t(), rotation_weights,
                translation_weights);
            add_constraint(constraint);
        }

        if (this->options.minimizer_progress_to_stdout) {
            logger().info("\n\n  Added Constraint {}\n  Type: ABSOLUTE_POSE\n",
                          ++constraint_count_);
        }
        return true;
    }

    void add_traj_constraint(bool fix_first_node) {
        auto nodes = traj.timestamp_node_map;

        if (nodes.size() < 2) {
            logger().error(
                "No constraints to add if there are fewer than 2 nodes");
            return;
        }

        auto it = nodes.begin();
        auto it_next = std::next(it);

        if (fix_first_node) {
            const auto& first_node = it->second;
            problem.SetParameterBlockConstant(
                first_node->rotation.coeffs().data());
            problem.SetParameterBlockConstant(first_node->position.data());
        }

        for (; it_next != nodes.end(); ++it, ++it_next) {
            const auto& node_before = it->second;
            const auto& node_after = it_next->second;

            ouster::impl::PoseH diff =
                node_before->get_pose().inverse() * node_after->get_pose();
            ouster::impl::PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraint constraint(
                node_before, node_after, diff_q.r(), diff_q.t(),
                traj_rotation_weight, traj_translation_weight);

            add_constraint(constraint);
        }
    }

    std::shared_ptr<Node> create_node_from_point(uint64_t ts, uint32_t row,
                                                 uint32_t col,
                                                 uint32_t return_idx) {
        if (return_idx != 1 && return_idx != 2) {
            throw std::invalid_argument(
                "Fail to create a Node. return_idx can only be 1 or 2 but "
                "received " +
                std::to_string(return_idx));
        }

        auto source = ouster::open_source(
            traj.input_osf_file,
            [return_idx](auto& r) {
                r.index = true;
                if (return_idx == 1) {
                    r.field_names = std::vector<std::string>({"RANGE"});
                } else {
                    r.field_names = std::vector<std::string>({"RANGE2"});
                }
            },
            /* collate = false */ false);

        nonstd::optional<uint64_t> start_index;
        uint64_t end_index = 0;

        for (const auto& each : traj.timestamps_index_vec) {
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

        if (!start_index.has_value()) {
            throw std::runtime_error{"Can't find any scan before ts " +
                                     std::to_string(ts) +
                                     ". Fail to create new Node"};
        }

        if (!end_index) {
            throw std::runtime_error(
                std::string("Can't find any scan after ts ") +
                std::to_string(ts) + ". Fail to create new Node");
        }

        auto part_osf = source[{start_index, end_index + 1}];

        for (const auto& scans : part_osf) {
            for (auto& ls : scans) {
                if (!ls) {
                    continue;
                }

                uint64_t ls_ts = ls->get_first_valid_column_timestamp();
                if (ls && ls_ts == ts) {
                    uint64_t col_ts = ls->timestamp()[col];
                    Eigen::Matrix4d mat;
                    std::memcpy(mat.data(), ls->pose().subview(col).get(),
                                sizeof(double) * 16);
                    Eigen::Matrix4d mat_t = mat.transpose();
                    ouster::impl::PoseH pose = ouster::impl::PoseH(mat_t);

                    const auto range =
                        (return_idx == 1)
                            ? ls->field<uint32_t>(sensor::ChanField::RANGE)
                            : ls->field<uint32_t>(sensor::ChanField::RANGE2);

                    auto d_range = ouster::destagger<uint32_t>(
                        range, traj.info.format.pixel_shift_by_row);

                    Eigen::Array<double, Eigen::Dynamic, 3> pts =
                        cartesian(d_range, traj.xyz_lut);
                    int key_pts_index = row * ls->w + col;
                    Eigen::Array<double, 1, 3> key_pts = pts.row(key_pts_index);

                    /*
                     * TODO Debug printout
                    auto transformed_points = transform_points(pts, pose);
                    save_pts_and_color(transformed_points,
                                       "pts_" + std::to_string(ts) + ".ply");
                    */

                    auto node = std::make_shared<Node>(col_ts, pose, key_pts);

                    if (node) {
                        add_node_to_problem(node);
                        add_node_neighbours_constraints(node);
                        traj.timestamp_node_map.insert({col_ts, node});
                    }

                    return node;
                }
            }
        }

        return nullptr;
    }
};

PoseOptimizer::PoseOptimizer(const std::string& osf_filename,
                             const SolverConfig& config, bool fix_first_node)
    : pimpl_(std::make_unique<Impl>(config, osf_filename)) {
    pimpl_->add_traj_constraint(fix_first_node);
}

PoseOptimizer::PoseOptimizer(const std::string& osf_filename,
                             double key_frame_distance, bool fix_first_node) {
    SolverConfig config;
    config.key_frame_distance = key_frame_distance;
    pimpl_ = std::make_unique<PoseOptimizer::Impl>(config, osf_filename);
    pimpl_->add_traj_constraint(fix_first_node);
}

PoseOptimizer::~PoseOptimizer() = default;

bool PoseOptimizer::add_absolute_pose_constraint(
    uint64_t ts, const Eigen::Matrix<double, 6, 1>& target_pose,
    double rotation_weight, double translation_weight,
    const Eigen::Matrix<double, 6, 1>& diff) {
    ouster::impl::PoseH pose_h =
        ouster::impl::PoseH(ouster::core::euler_pose_to_matrix(target_pose));
    ouster::impl::PoseH diff_h =
        ouster::impl::PoseH(ouster::core::euler_pose_to_matrix(diff));
    std::array<double, 3> rotation_weights;
    rotation_weights.fill(rotation_weight);
    std::array<double, 3> translation_weights;
    translation_weights.fill(translation_weight);
    return pimpl_->add_absolute_pose_constraint(
        ts, pose_h, diff_h, rotation_weights, translation_weights);
}

bool PoseOptimizer::add_absolute_pose_constraint(
    uint64_t ts, const Eigen::Matrix<double, 4, 4>& target_pose,
    double rotation_weight, double translation_weight,
    const Eigen::Matrix<double, 4, 4>& diff) {
    ouster::impl::PoseH pose_h(target_pose);
    ouster::impl::PoseH diff_h(diff);
    std::array<double, 3> rotation_weights;
    rotation_weights.fill(rotation_weight);
    std::array<double, 3> translation_weights;
    translation_weights.fill(translation_weight);
    return pimpl_->add_absolute_pose_constraint(
        ts, pose_h, diff_h, rotation_weights, translation_weights);
}

bool PoseOptimizer::add_absolute_pose_constraint(
    uint64_t ts, const Eigen::Matrix<double, 6, 1>& target_pose,
    const std::array<double, 3>& rotation_weights,
    const std::array<double, 3>& translation_weights,
    const Eigen::Matrix<double, 6, 1>& diff) {
    ouster::impl::PoseH pose_h =
        ouster::impl::PoseH(ouster::core::euler_pose_to_matrix(target_pose));
    ouster::impl::PoseH diff_h =
        ouster::impl::PoseH(ouster::core::euler_pose_to_matrix(diff));
    return pimpl_->add_absolute_pose_constraint(
        ts, pose_h, diff_h, rotation_weights, translation_weights);
}

bool PoseOptimizer::add_absolute_pose_constraint(
    uint64_t ts, const Eigen::Matrix<double, 4, 4>& target_pose,
    const std::array<double, 3>& rotation_weights,
    const std::array<double, 3>& translation_weights,
    const Eigen::Matrix<double, 4, 4>& diff) {
    ouster::impl::PoseH pose_h(target_pose);
    ouster::impl::PoseH diff_h(diff);
    return pimpl_->add_absolute_pose_constraint(
        ts, pose_h, diff_h, rotation_weights, translation_weights);
}

bool PoseOptimizer::add_pose_to_pose_constraint(
    uint64_t ts1, uint64_t ts2, const Eigen::Matrix<double, 6, 1>& diff,
    double rotation_weight, double translation_weight) {
    ouster::impl::PoseH diff_h =
        ouster::impl::PoseH(ouster::core::euler_pose_to_matrix(diff));
    return pimpl_->add_pose_to_pose_constraint(
        ts1, ts2, diff_h, rotation_weight, translation_weight);
}

bool PoseOptimizer::add_pose_to_pose_constraint(
    uint64_t ts1, uint64_t ts2, const Eigen::Matrix<double, 4, 4>& diff,
    double rotation_weight, double translation_weight) {
    ouster::impl::PoseH diff_h(diff);
    return pimpl_->add_pose_to_pose_constraint(
        ts1, ts2, diff_h, rotation_weight, translation_weight);
}

bool PoseOptimizer::add_pose_to_pose_constraint(uint64_t ts1, uint64_t ts2,
                                                double rotation_weight,
                                                double translation_weight) {
    auto node1 = pimpl_->get_or_create_node_ts(ts1, true);
    auto node2 = pimpl_->get_or_create_node_ts(ts2, true);

    if (!node1) {
        throw std::runtime_error(
            "Failed to create the first node from timestamp " +
            std::to_string(ts1) +
            ". The timestamp may be invalid or not correspond to a scan's "
            "first valid column.");
    }

    if (!node2) {
        throw std::runtime_error(
            "Failed to create the second node from timestamp " +
            std::to_string(ts2) +
            ". The timestamp may be invalid or not correspond to a scan's "
            "first valid column.");
    }

    auto diff = run_icp(node1, node2);

    std::ostringstream oss;
    oss << diff.format(EigenMatrixPrintFmt);
    auto diff_str = oss.str();

    ++pimpl_->constraint_count_;
    if (pimpl_->options.minimizer_progress_to_stdout) {
        logger().info(
            "\n\n  Added Constraint {}\n  Type: RELATIVE_POSE_TO_POSE\n  Use "
            "ICP to auto align two poses. The pose diff:\n{}\n",
            pimpl_->constraint_count_, diff_str);
    }

    impl::PoseToPoseConstraint constraint(node1, node2, diff, rotation_weight,
                                          translation_weight);
    pimpl_->add_constraint(constraint);

    return true;
}

bool PoseOptimizer::add_point_to_point_constraint(
    uint64_t ts1, uint32_t row1, uint32_t col1, uint32_t return_idx1,
    uint64_t ts2, uint32_t row2, uint32_t col2, uint32_t return_idx2,
    double translation_weight) {
    if (return_idx1 != 1 && return_idx1 != 2) {
        throw std::invalid_argument(
            "Constraint creation failed: return_idx1 can only be 1 or 2, but "
            "received " +
            std::to_string(return_idx1));
    }

    if (return_idx2 != 1 && return_idx2 != 2) {
        throw std::invalid_argument(
            "Constraint creation failed: return_idx2 can only be 1 or 2, but "
            "received " +
            std::to_string(return_idx2));
    }

    // Attempt to create nodes
    auto node1 = pimpl_->create_node_from_point(ts1, row1, col1, return_idx1);
    if (!node1) {
        throw std::runtime_error(
            "Failed to create the first node from timestamp " +
            std::to_string(ts1) +
            ". The timestamp may be invalid or not correspond to a scan's "
            "first valid column.");
    }

    auto node2 = pimpl_->create_node_from_point(ts2, row2, col2, return_idx2);
    if (!node2) {
        throw std::runtime_error(
            "Failed to create the second node from timestamp " +
            std::to_string(ts2) +
            ". The timestamp may be invalid or not correspond to a scan's "
            "first valid column.");
    }

    // Add the constraint if nodes were created successfully
    impl::PointToPointConstraint constraint(node1, node2, translation_weight);
    pimpl_->add_constraint(constraint);

    ++pimpl_->constraint_count_;
    if (pimpl_->options.minimizer_progress_to_stdout) {
        logger().info(
            "\n\n  Added Constraint {}\n  Type: RELATIVE_POINT_TO_POINT\n",
            pimpl_->constraint_count_);
    }
    return true;
}

void PoseOptimizer::solve(uint32_t steps) {
    if (steps > 0) {
        pimpl_->options.max_num_iterations = steps;
        logger().info("Incremental optimize with {} iterations.", steps);
    } else {
        logger().info(
            "Running full optimization with default max iterations ({}).",
            pimpl_->options.max_num_iterations);
        pimpl_->options.max_num_iterations = pimpl_->default_max_num_iterations;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(pimpl_->options, &(pimpl_->problem), &summary);

    logger().info("Initial Cost: {}", summary.initial_cost);
    logger().info("Final   Cost: {}", summary.final_cost);
}

void PoseOptimizer::save(const std::string& osf_name) {
    logger().info("Saving the results into {}", osf_name);
    pimpl_->traj.save(osf_name);
}

std::vector<uint64_t> PoseOptimizer::get_timestamps(SamplingMode type) const {
    if (type == SamplingMode::KEY_FRAMES) {
        return pimpl_->traj.get_timestamps(type);
    } else if (type == SamplingMode::COLUMNS) {
        return pimpl_->traj.get_timestamps(type);
    } else {
        logger().error(
            "Invalid SamplingMode. Use SamplingMode::KEY_FRAMES or "
            "SamplingMode::COLUMNS.");
        throw std::invalid_argument("Invalid SamplingMode: ");
    }
}

std::vector<Eigen::Matrix<double, 4, 4>> PoseOptimizer::get_poses(
    SamplingMode type) {
    if (type == SamplingMode::KEY_FRAMES) {
        return pimpl_->traj.get_poses(type);
    } else if (type == SamplingMode::COLUMNS) {
        return pimpl_->traj.get_poses(type);
    } else {
        logger().error(
            "Invalid SamplingMode: {}. Use SamplingMode::KEY_FRAMES or "
            "SamplingMode::COLUMNS.");
        throw std::invalid_argument("Invalid SamplingMode: ");
    }
}

bool save_trajectory(const std::string& filename,
                     const std::vector<uint64_t>& timestamps,
                     const std::vector<Eigen::Matrix<double, 4, 4>>& poses,
                     const std::string& file_type) {
    if (timestamps.size() != poses.size()) {
        logger().error("Timestamps and poses size mismatch: {} vs {}",
                       timestamps.size(), poses.size());
        return false;
    }

    std::ofstream file(filename, std::ios::out);
    if (!file) {
        logger().error("Unable to open file: {}", filename);
        return false;
    }

    const size_t n = timestamps.size();
    if (file_type == "csv") {
        file << "timestamp,tx,ty,tz,qx,qy,qz,qw\n";
        for (size_t i = 0; i < n; ++i) {
            uint64_t ts = timestamps[i];
            const auto& M = poses[i];
            ouster::impl::PoseH poseh(M);
            const ouster::impl::PoseQ& poseq = poseh.log().q();
            const Eigen::Vector3d& t = poseh.t();
            const Eigen::Quaterniond& q = poseq.r();

            file << ts << ',' << t.x() << ',' << t.y() << ',' << t.z() << ','
                 << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w()
                 << '\n';
        }

    } else if (file_type == "tum") {
        for (size_t i = 0; i < n; ++i) {
            uint64_t ts = timestamps[i];
            const auto& M = poses[i];
            ouster::impl::PoseH poseh(M);
            const ouster::impl::PoseQ& poseq = poseh.log().q();
            const Eigen::Vector3d& t = poseh.t();
            const Eigen::Quaterniond& q = poseq.r();

            file << ts << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
                 << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
                 << '\n';
        }

    } else {
        logger().error(
            "Unsupported file type: {}. Currently support 'csv' or 'tum'.",
            file_type);
        return false;
    }

    logger().info("Trajectory successfully saved to {}", filename);
    return true;
}

}  // namespace mapping
}  // namespace ouster

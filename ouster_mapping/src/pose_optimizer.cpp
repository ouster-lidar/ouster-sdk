#include "ouster/mapping/pose_optimizer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/algorithm/align_clouds.h"
#include "ouster/core/chanfield.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/impl/transformation.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/metadata.h"
#include "ouster/core/open_source.h"
#include "ouster/core/pose_util.h"
#include "ouster/core/voxel_hash_map.h"
#include "ouster/mapping/constraint_config.h"
#include "ouster/mapping/grid_cell_loop_detector.h"
#include "ouster/mapping/impl/absolute_point_constraint_impl.h"
#include "ouster/mapping/impl/absolute_pose_constraint_impl.h"
#include "ouster/mapping/impl/constraint_impl.h"
#include "ouster/mapping/impl/point_to_point_constraint_impl.h"
#include "ouster/mapping/impl/pose_to_pose_constraint_impl.h"
#include "ouster/mapping/impl/trajectory.h"
#include "ouster/mapping/impl/utils.h"
#include "ouster/mapping/pose_optimizer_constraint.h"
#include "ouster/mapping/pose_optimizer_node.h"

// Third-party includes
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/version.h>

#if CERES_VERSION_MAJOR < 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR < 1)
#include <ceres/local_parameterization.h>
#else
#include <ceres/manifold.h>
#endif
using ouster::sdk::core::impl::PoseH;
using ouster::sdk::core::impl::PoseQ;

using ouster::sdk::core::logger;
namespace ouster {
namespace sdk {
namespace mapping {

// It starts at 1 because 0 is reserved for traj constraints
OUSTER_API_VAR std::atomic<uint32_t> Constraint::next_constraint_id_{1};

namespace {

void apply_transform_to_trajectory(Trajectory& traj, const PoseH& transform) {
    for (auto& timestamp_node_entry : traj.timestamp_node_map) {
        auto& node_ptr = timestamp_node_entry.second;
        if (!node_ptr) {
            continue;
        }
        PoseH updated_pose = transform * PoseH(node_ptr->get_pose());
        node_ptr->set_pose_components(Eigen::Quaterniond(updated_pose.r()).normalized(),
                                      updated_pose.t());
    }
    for (auto& pose_matrix : traj.all_poses) {
        pose_matrix = transform * pose_matrix;
    }
}

// Ceres updates quaternion/translation parameter blocks directly, bypassing
// Node::set_pose_components(). Invalidate the cached 4x4 pose after each
// solver state update before any code reads node.get_pose().
void mark_cached_node_poses_dirty(Trajectory& traj) {
    for (auto& timestamp_node_entry : traj.timestamp_node_map) {
        auto& node_ptr = timestamp_node_entry.second;
        if (!node_ptr) {
            continue;
        }
        node_ptr->mark_pose_dirty();
    }
}

std::shared_ptr<ouster::sdk::core::LidarFrame> load_frame_for_timestamp(
    ouster::sdk::core::FrameSetSource& source, const Trajectory& traj, uint64_t timestamp) {
    nonstd::optional<uint64_t> start_index_opt;
    nonstd::optional<uint64_t> end_index_opt;
    for (const auto& each : traj.timestamps_index_vec) {
        if (timestamp >= each.first_col_ts) {
            start_index_opt = each.frame_index;
        }
        if (timestamp <= each.last_col_ts) {
            end_index_opt = each.frame_index;
            break;
        }
    }

    const uint64_t source_size = source.size();
    if (source_size == 0u) {
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
    for (const auto& frame_set : part_osf) {
        for (const auto& frame_ptr : frame_set) {
            if (!frame_ptr) {
                continue;
            }
            try {
                const uint64_t first_ts =
                    frame_ptr->timestamp()[frame_ptr->get_first_valid_column()];
                const uint64_t last_ts = frame_ptr->timestamp()[frame_ptr->get_last_valid_column()];
                if (timestamp >= first_ts && timestamp <= last_ts) {
                    return std::make_shared<ouster::sdk::core::LidarFrame>(*frame_ptr);
                }
            } catch (const std::runtime_error& /*e*/) {
                // this may be redundant, should we just let it throw?
                continue;
            }
        }
    }
    return nullptr;
}

/// Compute the IMU-derived gravity plumb rotation for a frame. Returns
/// Identity if IMU data is absent or invalid.
Eigen::Matrix3d compute_imu_plumb(const ouster::sdk::core::LidarFrame& frame) {
    if (!frame.sensor_info) {
        return Eigen::Matrix3d::Identity();
    }
    if (!frame.has_field(ouster::sdk::core::ChanField::IMU_STATUS) ||
        !frame.has_field(ouster::sdk::core::ChanField::IMU_ACC)) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Ref<const Eigen::ArrayX<uint16_t>> imu_status =
        frame.field(ouster::sdk::core::ChanField::IMU_STATUS);
    Eigen::Ref<const ouster::sdk::core::ArrayX3fR> imu_acc =
        frame.field(ouster::sdk::core::ChanField::IMU_ACC);
    if (imu_acc.rows() != imu_status.size()) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    int valid_count = 0;
    for (Eigen::Index i = 0; i < imu_status.size(); ++i) {
        if ((imu_status(i) & 0x01u) != 0u) {
            const Eigen::Vector3d acc = imu_acc.row(i).matrix().transpose().template cast<double>();
            if (acc.allFinite()) {
                acc_sum += acc;
                ++valid_count;
            }
        }
    }
    if (valid_count == 0) {
        return Eigen::Matrix3d::Identity();
    }
    const auto& info = *frame.sensor_info;
    const Eigen::Vector3d acc_avg = info.sensor_to_body.block<3, 3>(0, 0) *
                                    info.imu_to_sensor_transform.block<3, 3>(0, 0) *
                                    (acc_sum / static_cast<double>(valid_count));
    if (!acc_avg.allFinite() || acc_avg.norm() <= 1e-6) {
        return Eigen::Matrix3d::Identity();
    }
    return ouster::sdk::core::get_rot_matrix_to_align_to_gravity(acc_avg.x(), acc_avg.y(),
                                                                 acc_avg.z(), false);
}

// Return a copy of frame with R_plumb applied to its extrinsic.
ouster::sdk::core::LidarFrame apply_imu_plumb(const ouster::sdk::core::LidarFrame& frame,
                                              const Eigen::Matrix3d& plumb) {
    ouster::sdk::core::LidarFrame plumbed = frame;
    if (plumb.isIdentity(1e-10) || !frame.sensor_info) {
        return plumbed;
    }
    core::Matrix4dR plumb_pose = core::Matrix4dR::Identity();
    plumb_pose.block<3, 3>(0, 0) = plumb;
    auto updated_info = std::make_shared<ouster::sdk::core::SensorInfo>(*frame.sensor_info);
    updated_info->sensor_to_body = plumb_pose * frame.sensor_info->sensor_to_body;
    plumbed.sensor_info = updated_info;
    return plumbed;
}

std::pair<PoseH, double> run_icp(const Trajectory& traj, ouster::sdk::core::FrameSetSource& source,
                                 const std::shared_ptr<Node>& node_first,
                                 const std::shared_ptr<Node>& node_second) {
    PoseH pose1(node_first->get_pose());
    PoseH pose2(node_second->get_pose());
    PoseH initial_guess = PoseH(pose1.inverse() * pose2);

    auto frame_first = load_frame_for_timestamp(source, traj, node_first->ts);
    auto frame_second = load_frame_for_timestamp(source, traj, node_second->ts);
    if (frame_first && frame_second) {
        try {
            // Compute per-frame IMU plumb rotations and apply to frame copies
            // so the matcher operates in a gravity-aligned frame.
            const Eigen::Matrix3d plumb_src = compute_imu_plumb(*frame_first);
            const Eigen::Matrix3d plumb_tgt = compute_imu_plumb(*frame_second);
            const ouster::sdk::core::LidarFrame src_plumbed =
                apply_imu_plumb(*frame_first, plumb_src);
            const ouster::sdk::core::LidarFrame tgt_plumbed =
                apply_imu_plumb(*frame_second, plumb_tgt);

            // Convert T_first_from_second to the plumbed frames:
            //   T_first_plumb_from_second_plumb =
            //       R_plumb_first * T_first_from_second * R_plumb_second^-1
            core::Matrix4dR plumb_src_pose = core::Matrix4dR::Identity();
            plumb_src_pose.block<3, 3>(0, 0) = plumb_src;
            core::Matrix4dR plumb_tgt_pose = core::Matrix4dR::Identity();
            plumb_tgt_pose.block<3, 3>(0, 0) = plumb_tgt;
            const core::Matrix4dR initial_guess_plumbed =
                plumb_src_pose * initial_guess.matrix() * plumb_tgt_pose.inverse();

            double score = 0.0;
            // The constraint expects T_first_from_second. align_clouds returns
            // source_to_target_transform, so pass the second frame as the
            // source.
            const core::Matrix4dR result_plumbed =
                algorithm::align_clouds(tgt_plumbed, src_plumbed, initial_guess_plumbed, score);

            // Convert T_first_plumb_from_second_plumb back:
            //   T_first_from_second =
            //       R_plumb_first^-1 * T_plumbed * R_plumb_second
            const core::Matrix4dR result =
                plumb_src_pose.inverse() * result_plumbed * plumb_tgt_pose;
            logger().info("Auto ICP score between {} and {} : {}", node_first->ts, node_second->ts,
                          score);
            return {PoseH(result), score};
        } catch (const std::exception& e) {
            logger().warn(
                "Coarse-to-fine frame matcher failed for ts {} and {}: {}. "
                "Falling back to initial guess.",
                node_first->ts, node_second->ts, e.what());
        }
    }

    logger().info("Auto ICP score between {} and {} : {}", node_first->ts, node_second->ts, 0);
    return {initial_guess, 0.0};
}

ceres::LossFunction* create_loss_function(LossFunction loss_func, double scale) {
    switch (loss_func) {
        case LossFunction::HUBER_LOSS:
            return new ceres::HuberLoss(scale);
        case LossFunction::CAUCHY_LOSS:
            return new ceres::CauchyLoss(scale);
        case LossFunction::SOFT_L_ONE_LOSS:
            return new ceres::SoftLOneLoss(scale);
        case LossFunction::ARCTAN_LOSS:
            return new ceres::ArctanLoss(scale);
        case LossFunction::TRIVIAL_LOSS:
            return nullptr;
        default:
            throw std::invalid_argument(
                "Unknown loss function. Available options are: "
                "HUBER_LOSS, CAUCHY_LOSS, SOFT_L_ONE_LOSS, ARCTAN_LOSS, "
                "TRIVIAL_LOSS.");
    }
}

const Eigen::IOFormat EIGEN_MATRIX_PRINT_FMT(
    /* precision    */ 9,
    /* flags        */ 0,  // align columns
    /* coeff_sep    */ ", ",
    /* row_sep      */ "\n",
    /* mat_prefix   */ "  [",
    /* mat_suffix   */ "]",
    /* row_prefix   */ "",
    /* row_suffix   */ "");

}  // namespace

class PoseOptimizer::Impl {
   public:
    ceres::Problem problem;
    ceres::Solver::Options options;
    Trajectory traj;
    ceres::LossFunction* loss_function = nullptr;
    SolverConfig config;
    bool fix_first_node = false;
    // Optional functor invoked at each solver iteration
    std::function<void()> solver_step_functor_{};  // NOLINT(readability-identifier-naming)
    struct StepCallback : public ceres::IterationCallback {
        Impl* self;
        explicit StepCallback(Impl* impl_ptr) : self(impl_ptr) {}
        ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
            (void)summary;
            if (self != nullptr) {
                mark_cached_node_poses_dirty(self->traj);
                if (self->solver_step_functor_) {
                    self->solver_step_functor_();
                }
            }
            return ceres::SOLVER_CONTINUE;
        }
    };
    std::unique_ptr<StepCallback> step_cb_holder_;  // NOLINT(readability-identifier-naming)
    // Store last final cost from Ceres summary
    double cost_number_{-1};  // NOLINT(readability-identifier-naming)

    // Total number of iterations executed across all solve() calls
    uint64_t total_iterations_{0};  // NOLINT(readability-identifier-naming)

    // Track only user constraints (from config.constraints) for selective
    // removal
    std::vector<ceres::ResidualBlockId> user_constraint_residual_blocks;

    // Map constraint ID to residual block ID for individual constraint removal
    std::unordered_map<uint32_t, ceres::ResidualBlockId> constraint_id_to_residual_map;

    // Downsample voxel size for point clouds used in ICP
    double downsample_voxel_size = 0.05;
    std::shared_ptr<ouster::sdk::core::FrameSetSource>
        icp_frame_set_source_;  // NOLINT(readability-identifier-naming)

    Impl(const SolverConfig& solver_options, const std::string& osf_filename)
        : config(solver_options) {
        options.max_num_iterations = static_cast<int>(config.max_num_iterations);
        options.function_tolerance = config.function_tolerance;
        options.gradient_tolerance = config.gradient_tolerance;
        options.parameter_tolerance = config.parameter_tolerance;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = config.process_printout;
        options.update_state_every_iteration = true;
        step_cb_holder_ = std::make_unique<StepCallback>(this);
        options.callbacks.push_back(step_cb_holder_.get());

        logger().info("Initializing Pose Optimizer ...");

        try {
            if (solver_options.loss_function == LossFunction::TRIVIAL_LOSS) {
                // TRIVIAL_LOSS ignores the scale entirely
                loss_function = create_loss_function(LossFunction::TRIVIAL_LOSS, 0.0);
                logger().info("Using TRIVIAL_LOSS function");
            } else {
                // All other losses do use the scale
                loss_function =
                    create_loss_function(solver_options.loss_function, solver_options.loss_scale);
                logger().info("Using loss function: {} with scale: {}",
                              to_string(solver_options.loss_function), solver_options.loss_scale);
            }

        } catch (const std::exception& e) {
            logger().error("Error creating loss function: {}", e.what());
            throw;
        }

        traj = Trajectory(osf_filename, solver_options.key_frame_distance);
        for (const auto& node_pair : traj.timestamp_node_map) {
            add_node_to_problem(node_pair.second);
        }

        // Process constraints from config if any
        process_config_constraints();
    }

    ouster::sdk::core::FrameSetSource& get_icp_frame_set_source() {
        if (!icp_frame_set_source_) {
            icp_frame_set_source_ = ouster::sdk::open_source(
                                        traj.input_osf_file,
                                        [](auto& request) {
                                            request.index = true;
                                            // Do not filter fields here. We
                                            // always need RANGE, and NORMALS
                                            // may or may not exist in the OSF.
                                        },
                                        /* collate = false */ false)
                                        .child();
        }
        return *icp_frame_set_source_;
    }

    /**
     * @brief Process and assign IDs to constraints loaded from configuration.
     *
     * This method assigns sequential constraint IDs (starting from 1) to all
     * constraints that were loaded from the configuration file. It then adds
     * each constraint to the optimization problem. This ensures that config
     * constraints have proper IDs before any manually added constraints.
     */
    void process_config_constraints() {
        if (config.constraints.empty()) {
            return;
        }

        logger().info("Processing {} constraint(s) from config", config.constraints.size());

        for (const auto& constraint : config.constraints) {
            add_base_constraint(constraint.get());
        }
    }

    /**
     * One-time "rough alignment" pre-pass for absolute constraints.
     * Solves for a single SE(3) transform that best maps trajectory points
     * into the absolute (target) frame in a weighted least-squares sense,
     * then rigid-transform the entire trajectory by that transform.
     *
     * It uses SVD Kabsch algorithm
     * https://en.wikipedia.org/wiki/Kabsch_algorithm
     * Notes:
     * - Builds a set of 3D point correspondences from absolute constraints:
     * - Estimates a single rigid transform (R, t) using a weighted Kabsch-style
     *   SVD solve on the correspondences, minimizing:
     *       sum_i w_i * || (R * p_i + t) - q_i ||^2
     *   A reflection fix is applied to keep det(R) = +1.
     * - The weight w_i is derived from translation_weights.
     *   Note: rotation_weight and target pose orientation are not used here;
     * - Applies the transform to every node:
     *       T_new = [R|t] * T_old
     * - Skips if there are no usable pairs, total weight is zero, or the
     *   resulting motion is below the (0.5m, 1°) threshold.
     */
    core::Matrix4dR initialize_trajectory_alignment() {
        struct AlignmentPair {
            Eigen::Vector3d trajectory_point;
            Eigen::Vector3d target_point;
            double weight;
        };

        core::Matrix4dR identity = core::Matrix4dR::Identity();

        // Build correspondence pairs used by the weighted SVD solve.
        std::vector<AlignmentPair> pairs;
        pairs.reserve(config.constraints.size());

        auto weight_from_translation = [](const Eigen::Array3d& weights) {
            return weights.maxCoeff();
        };

        auto add_pose_pair = [&](const PoseH& node_pose, const PoseH& target,
                                 const Eigen::Array3d& translation_weights) {
            const double weight_value = weight_from_translation(translation_weights);
            if (weight_value <= 0.0) {
                return;
            }
            pairs.push_back({node_pose.t(), target.t(), weight_value});
        };

        auto add_point_pair = [&](const PoseH& node_pose, const Eigen::Vector3d& source_point,
                                  const Eigen::Vector3d& target_point,
                                  const Eigen::Array3d& translation_weights) {
            const double weight_value = weight_from_translation(translation_weights);
            if (weight_value <= 0.0) {
                return;
            }
            Eigen::Vector3d world_point = node_pose.r() * source_point + node_pose.t();
            pairs.push_back({world_point, target_point, weight_value});
        };

        for (const auto& constraint : config.constraints) {
            if (!constraint) {
                continue;
            }
            const auto type = constraint->get_type();

            uint64_t timestamp = 0;
            if (type == ConstraintType::ABSOLUTE_POSE) {
                timestamp =
                    dynamic_cast<const AbsolutePoseConstraint*>(constraint.get())->timestamp;
            } else if (type == ConstraintType::ABSOLUTE_POINT) {
                timestamp =
                    dynamic_cast<const AbsolutePointConstraint*>(constraint.get())->timestamp;
            } else {
                continue;
            }

            auto node = traj.get_node_by_ts(timestamp);
            if (!node) {
                continue;
            }
            PoseH node_pose(node->get_pose());

            if (type == ConstraintType::ABSOLUTE_POSE) {
                auto* abs_pose_constraint =
                    dynamic_cast<const AbsolutePoseConstraint*>(constraint.get());
                PoseH target_pose(abs_pose_constraint->pose);
                add_pose_pair(node_pose, target_pose, abs_pose_constraint->translation_weights);
            } else if (type == ConstraintType::ABSOLUTE_POINT) {
                auto* abs_point_constraint =
                    dynamic_cast<const AbsolutePointConstraint*>(constraint.get());
                if (node->ap_constraint_pt.rows() == 0) {
                    continue;
                }
                Eigen::Vector3d source_point = node->ap_constraint_pt.row(0).matrix().transpose();
                add_point_pair(node_pose, source_point, abs_point_constraint->absolute_position,
                               abs_point_constraint->translation_weights);
            }
        }

        if (pairs.empty()) {
            logger().warn(
                "initialize_trajectory_alignment(): no usable absolute "
                "constraints");
            return identity;
        }

        // Weighted centroids of source (trajectory) and destination (targets).
        double total_weight = 0.0;
        Eigen::Vector3d src_centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d dst_centroid = Eigen::Vector3d::Zero();
        for (const auto& pair_data : pairs) {
            total_weight += pair_data.weight;
            src_centroid += pair_data.weight * pair_data.trajectory_point;
            dst_centroid += pair_data.weight * pair_data.target_point;
        }

        if (total_weight <= 0.0) {
            logger().warn(
                "initialize_trajectory_alignment(): zero total weight from "
                "absolute constraints");
            return identity;
        }

        src_centroid /= total_weight;
        dst_centroid /= total_weight;

        // Weighted cross-covariance used by the SVD rotation estimate.
        Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
        for (const auto& pair_data : pairs) {
            const Eigen::Vector3d src_centered = pair_data.trajectory_point - src_centroid;
            const Eigen::Vector3d dst_centered = pair_data.target_point - dst_centroid;
            covariance_matrix += pair_data.weight * src_centered * dst_centered.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance_matrix,
                                              Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d u_matrix = svd.matrixU();
        const auto& v_matrix = svd.matrixV();
        Eigen::Matrix3d reflection_fix = Eigen::Matrix3d::Identity();
        if ((v_matrix * u_matrix.transpose()).determinant() < 0.0) {
            reflection_fix(2, 2) = -1.0;
        }
        // Kabsch rotation: R = V * S * U^T, with S fixing reflections.
        Eigen::Matrix3d rotation_matrix = v_matrix * reflection_fix * u_matrix.transpose();
        Eigen::Vector3d translation_vector = dst_centroid - rotation_matrix * src_centroid;

        const double rotation_angle = std::abs(Eigen::AngleAxisd(rotation_matrix).angle());
        const double translation_norm = translation_vector.norm();
        const double translation_threshold = 0.5;            // meters
        const double rotation_threshold = 1.0 * M_PI / 180;  // 1 degree
        if (translation_norm < translation_threshold && rotation_angle < rotation_threshold) {
            logger().info("initialize_trajectory_alignment skipped tiny alignment");
            return identity;
        }

        // Apply the alignment transform to every node/pose as a left-multiply.
        PoseH transform;
        transform.set_rot(rotation_matrix);
        transform.set_trans(translation_vector);

        apply_transform_to_trajectory(traj, transform);

        logger().info("initialize_trajectory_alignment applied");
        return transform.matrix();
    }

    // =============================================================================
    // CONSTRAINT ADDITION METHODS
    // =============================================================================

    /**
     * @brief Adds an absolute pose constraint to fix a node at a specific
     * global pose
     * @param constraint The absolute pose constraint to add
     */
    void add_absolute_pose_constraint(const AbsolutePoseConstraint& abs_constraint) {
        try {
            // For absolute pose constraints, find an existing node in the
            // trajectory
            auto node = traj.get_node_by_ts(abs_constraint.timestamp);
            if (!node) {
                // Absolute pose does not require point clouds; allow
                // interpolation without generating one
                node = get_or_create_node_by_ts(abs_constraint.timestamp, false);
                if (!node) {
                    logger().error("Failed to create node from timestamp {}",
                                   abs_constraint.timestamp);
                    return;
                }
            }
            // Use weights
            const double rotation_weight = abs_constraint.rotation_weight;
            const auto& translation_weights = abs_constraint.translation_weights;

            // Unfix the node if it's already fixed so it can be optimized by
            // the constraint
            if (problem.IsParameterBlockConstant(node->rotation_coeffs_data()) ||
                problem.IsParameterBlockConstant(node->position_data())) {
                problem.SetParameterBlockVariable(node->mutable_rotation_coeffs_data());
                problem.SetParameterBlockVariable(node->mutable_position_data());
            }

            // Create target node (NOT added to trajectory - it's a fixed
            // reference)
            auto target_node =
                std::make_shared<Node>(abs_constraint.timestamp, abs_constraint.pose);

            impl::AbsolutePoseConstraintImpl constraint_impl(node, target_node, rotation_weight,
                                                             translation_weights);
            add_constraint_with_id(constraint_impl, abs_constraint.get_constraint_id());

            release_first_node_anchor();

            logger().info(
                "Successfully processed absolute pose constraint id {} for "
                "timestamp {}",
                abs_constraint.get_constraint_id(), abs_constraint.timestamp);
        } catch (const std::exception& e) {
            logger().error("Error creating absolute pose constraint for timestamp {}: {}",
                           abs_constraint.timestamp, e.what());
        }
    }

    void release_first_node_anchor() {
        if (config.fix_first_node || !fix_first_node) {
            return;
        }

        if (traj.timestamp_node_map.empty()) {
            return;
        }

        const auto& first_entry = *traj.timestamp_node_map.begin();
        const auto& first_node = first_entry.second;
        if (!first_node) {
            return;
        }

        if (problem.IsParameterBlockConstant(first_node->rotation_coeffs_data())) {
            problem.SetParameterBlockVariable(first_node->mutable_rotation_coeffs_data());
        }
        if (problem.IsParameterBlockConstant(first_node->position_data())) {
            problem.SetParameterBlockVariable(first_node->mutable_position_data());
        }

        fix_first_node = false;
        logger().info("Released first node anchor after adding an absolute constraint.");
    }

    /**
     * @brief Adds a pose-to-pose constraint between two trajectory nodes
     * @param constraint The pose-to-pose constraint to add (can be ICP-based or
     * fixed transform)
     */
    void add_pose_to_pose_constraint(const PoseToPoseConstraint& pose_constraint) {
        // Use weights directly (no conversion needed)
        const double rotation_weight = pose_constraint.rotation_weight;
        const auto& translation_weights = pose_constraint.translation_weights;

        try {
            // Check if relative_pose is identity matrix - this indicates
            // ICP-based constraint
            PoseH relative_pose(pose_constraint.relative_pose);

            if (relative_pose.isIdentity()) {
                // ICP-based constraint: need point clouds for both nodes
                logger().info(
                    "Creating pose to pose constraint between {} and {}. Align "
                    "by ICP",
                    pose_constraint.timestamp1, pose_constraint.timestamp2);

                std::shared_ptr<Node> node1;
                std::shared_ptr<Node> node2;
                try {
                    node1 = get_or_create_node_by_ts(pose_constraint.timestamp1, true);
                } catch (const std::exception& e) {
                    logger().error(
                        "Failed to create node1 with point cloud for timestamp "
                        "{}: {}",
                        pose_constraint.timestamp1, e.what());
                    return;
                }

                try {
                    node2 = get_or_create_node_by_ts(pose_constraint.timestamp2, true);
                } catch (const std::exception& e) {
                    logger().error(
                        "Failed to create node2 with point cloud for timestamp "
                        "{}: {}",
                        pose_constraint.timestamp2, e.what());
                    return;
                }

                if (node1 && node2) {
                    auto icp_result = run_icp(traj, get_icp_frame_set_source(), node1, node2);
                    const PoseH& diff = icp_result.first;
                    std::stringstream string_stream;
                    string_stream << diff.matrix().format(EIGEN_MATRIX_PRINT_FMT);
                    logger().info(
                        "Run ICP between frames at {} and {}. The ICP "
                        "transformation matrix:\n{}",
                        node1->ts, node2->ts, string_stream.str());
                    impl::PoseToPoseConstraintImpl constraint_impl(
                        node1, node2, diff, rotation_weight, translation_weights);
                    add_constraint_with_id(constraint_impl, pose_constraint.get_constraint_id());
                    logger().info(
                        "Successfully added pose to pose constraint id {} "
                        "between {} and {}",
                        pose_constraint.get_constraint_id(), pose_constraint.timestamp1,
                        pose_constraint.timestamp2);
                } else {
                    logger().error(
                        "Failed to create nodes for pose to pose constraint. "
                        "Check that timestamps {} and {} correspond to valid "
                        "frames in the OSF file.",
                        pose_constraint.timestamp1, pose_constraint.timestamp2);
                    return;
                }
            } else {
                // Stored pose constraint: generate point clouds for these nodes
                logger().info(
                    "Creating pose to pose constraint between {} and {} using "
                    "the given transformation matrix",
                    pose_constraint.timestamp1, pose_constraint.timestamp2);

                auto node1 = get_or_create_node_by_ts(pose_constraint.timestamp1, true);
                auto node2 = get_or_create_node_by_ts(pose_constraint.timestamp2, true);

                if (node1 && node2) {
                    impl::PoseToPoseConstraintImpl constraint_impl(
                        node1, node2, relative_pose, rotation_weight, translation_weights);
                    add_constraint_with_id(constraint_impl, pose_constraint.get_constraint_id());
                    logger().info(
                        "Successfully added stored pose constraint id {} "
                        "between {} and {}",
                        pose_constraint.get_constraint_id(), pose_constraint.timestamp1,
                        pose_constraint.timestamp2);
                } else {
                    logger().error(
                        "Failed to create nodes for stored pose constraint "
                        "between {} and {}",
                        pose_constraint.timestamp1, pose_constraint.timestamp2);
                    return;
                }
            }
        } catch (const std::exception& e) {
            logger().error(
                "Error creating pose to pose constraint between "
                "timestamps {} and {}: {}",
                pose_constraint.timestamp1, pose_constraint.timestamp2, e.what());
        }
    }

    /**
     * @brief Helper function to get or create a node for point-to-point
     * constraints
     * @param timestamp The timestamp of the node
     * @param row Pixel row coordinate
     * @param col Pixel column coordinate
     * @param return_idx Return index
     * @param node_name Human-readable node identifier for logging (e.g.,
     * "node1", "node2")
     * @return Shared pointer to the node, or nullptr if creation failed
     */
    std::shared_ptr<Node> get_or_create_ptp_node(uint64_t timestamp, int row, int col,
                                                 int return_idx, const std::string& node_name) {
        std::shared_ptr<Node> node = traj.get_node_by_ts(timestamp);

        if (node) {
            if (!ensure_node_has_ptp_point(node, timestamp, row, col, return_idx)) {
                logger().warn(
                    "ensure_node_has_ptp_point failed for {}; "
                    "attempting to create a dedicated PTP node",
                    node_name);
                auto created = create_node_for_ptp(timestamp, row, col, return_idx);
                if (created) {
                    node = created;
                } else {
                    logger().error("Failed to ensure or create {} with ptp point data", node_name);
                    return nullptr;
                }
            }
        } else {
            node = create_node_for_ptp(timestamp, row, col, return_idx);
            if (!node) {
                logger().error("Failed to create {} for point-to-point constraint", node_name);
                return nullptr;
            }
        }

        return node;
    }

    /**
     * @brief Adds a point-to-point constraint between specific 3D points in two
     * frames
     * @param constraint The point-to-point constraint specifying pixel
     * coordinates and return indices
     */
    void add_point_to_point_constraint(const PointToPointConstraint& pt_constraint) {
        try {
            logger().info("Creating point to point constraint between {} and {} ",
                          pt_constraint.timestamp1, pt_constraint.timestamp2);

            // Handle node1
            std::shared_ptr<Node> node1 = get_or_create_ptp_node(
                pt_constraint.timestamp1, static_cast<int>(pt_constraint.row1),
                static_cast<int>(pt_constraint.col1), static_cast<int>(pt_constraint.return_idx1),
                "node1");
            if (!node1) {
                return;
            }

            // Handle node2
            std::shared_ptr<Node> node2 = get_or_create_ptp_node(
                pt_constraint.timestamp2, static_cast<int>(pt_constraint.row2),
                static_cast<int>(pt_constraint.col2), static_cast<int>(pt_constraint.return_idx2),
                "node2");
            if (!node2) {
                return;
            }

            // Create the constraint
            impl::PointToPointConstraintImpl constraint_impl(node1, node2,
                                                             pt_constraint.translation_weights);
            add_constraint_with_id(constraint_impl, pt_constraint.get_constraint_id());

            logger().info(
                "Successfully added point to point constraint id {} between "
                "{} and {}",
                pt_constraint.get_constraint_id(), pt_constraint.timestamp1,
                pt_constraint.timestamp2);
        } catch (const std::exception& e) {
            logger().error(
                "Error creating point to point constraint between timestamps "
                "{} and {}: {}",
                pt_constraint.timestamp1, pt_constraint.timestamp2, e.what());
        }
    }

    /**
     * @brief Adds an absolute point constraint to fix a specific 3D point at a
     * global position
     * @param constraint The absolute point constraint specifying pixel
     * coordinates and global position
     */
    void add_absolute_point_constraint(const AbsolutePointConstraint& abs_pt_constraint) {
        try {
            logger().info(
                "Creating absolute point constraint for timestamp {} at "
                "position ({}, {}, {})",
                abs_pt_constraint.timestamp, abs_pt_constraint.absolute_position.x(),
                abs_pt_constraint.absolute_position.y(), abs_pt_constraint.absolute_position.z());

            std::shared_ptr<Node> node = traj.get_node_by_ts(abs_pt_constraint.timestamp);

            if (node) {
                // Node exists, ensure it has the required point data
                if (!ensure_node_has_absolute_point(
                        node, abs_pt_constraint.timestamp, static_cast<int>(abs_pt_constraint.row),
                        static_cast<int>(abs_pt_constraint.col),
                        static_cast<int>(abs_pt_constraint.return_idx))) {
                    logger().error("Failed to ensure node has absolute point data");
                    return;
                }
            } else {
                // Node doesn't exist, create new one
                node = create_node_for_absolute_point(
                    abs_pt_constraint.timestamp, static_cast<int>(abs_pt_constraint.row),
                    static_cast<int>(abs_pt_constraint.col),
                    static_cast<int>(abs_pt_constraint.return_idx));
                if (!node) {
                    logger().error(
                        "Failed to create new node for absolute point "
                        "constraint");
                    return;
                }
            }

            // Create the constraint
            impl::AbsolutePointConstraintImpl constraint_impl(
                node, abs_pt_constraint.absolute_position, abs_pt_constraint.translation_weights);
            add_constraint_with_id(constraint_impl, abs_pt_constraint.get_constraint_id());

            release_first_node_anchor();

            logger().info(
                "Successfully processed absolute point constraint id {} for "
                "timestamp {}",
                abs_pt_constraint.get_constraint_id(), abs_pt_constraint.timestamp);
        } catch (const std::exception& e) {
            logger().error("Error creating absolute point constraint for timestamp {}: {}",
                           abs_pt_constraint.timestamp, e.what());
        }
    }

    // =============================================================================
    // HELPER FUNCTIONS FOR POINT CONSTRAINT MANAGEMENT
    // =============================================================================

    // Helper function to ensure a node has the required point data for absolute
    // point constraints
    bool ensure_node_has_absolute_point(std::shared_ptr<Node>& node, uint64_t timestamp, int row,
                                        int col, int return_idx) {
        if (!node) {
            return false;
        }

        if (node->ap_constraint_pt.rows() > 0) {
            if (node->ap_row < 0 || node->ap_col < 0 || node->ap_return < 0) {
                node->ap_row = row;
                node->ap_col = col;
                node->ap_return = return_idx;
                return true;
            }
            if (node->ap_row == row && node->ap_col == col && node->ap_return == return_idx) {
                if (node->downsampled_pts.rows() == 0) {
                    try {
                        auto tmp = create_node_from_point(timestamp, row, col, return_idx);
                        if (tmp && tmp->downsampled_pts.rows() > 0) {
                            node->downsampled_pts = tmp->downsampled_pts;
                        }
                    } catch (...) {
                    }
                }
                return true;
            }
            logger().error(
                "Absolute point selection for timestamp {} already exists with "
                "row={} col={} return={} (requested row={} col={} return={}).",
                timestamp, node->ap_row, node->ap_col, node->ap_return, row, col, return_idx);
            return false;
        }

        try {
            auto temp_node = create_node_from_point(timestamp, row, col, return_idx);
            if (temp_node) {
                if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ap_constraint_pt = temp_node->ap_constraint_pt.row(0);
                } else {
                    return false;
                }
                node->ap_row = row;
                node->ap_col = col;
                node->ap_return = return_idx;
                if (temp_node->downsampled_pts.rows() > 0 && node->downsampled_pts.rows() == 0) {
                    // Populate cloud only if not already set to avoid
                    // inconsistencies across constraints
                    node->downsampled_pts = temp_node->downsampled_pts;
                }
                logger().info("Added ap_constraint_pt to existing node for timestamp {}",
                              timestamp);
                return true;
            }
        } catch (const std::exception& e) {
            logger().error("Failed to get selected point for existing node: {}", e.what());
        }
        return false;
    }

    // Helper function to ensure a node has the required point data for
    // point-to-point constraints
    bool ensure_node_has_ptp_point(std::shared_ptr<Node>& node, uint64_t timestamp, int row,
                                   int col, int return_idx) {
        if (!node) {
            return false;
        }

        if (node->ptp_constraint_pt.rows() > 0) {
            if (node->ptp_row < 0 || node->ptp_col < 0 || node->ptp_return < 0) {
                node->ptp_row = row;
                node->ptp_col = col;
                node->ptp_return = return_idx;
                if (node->ap_constraint_pt.rows() == 0) {
                    node->ap_row = row;
                    node->ap_col = col;
                    node->ap_return = return_idx;
                }
                return true;
            }
            if (node->ptp_row == row && node->ptp_col == col && node->ptp_return == return_idx) {
                return true;
            }
            logger().error(
                "Point-to-point selection for timestamp {} already exists with "
                "row={} col={} return={} (requested row={} col={} return={}).",
                timestamp, node->ptp_row, node->ptp_col, node->ptp_return, row, col, return_idx);
            return false;
        }

        // Always fetch the requested point so we honor the row/col selection
        try {
            auto temp_node = create_node_from_point(timestamp, row, col, return_idx);
            if (temp_node) {
                if (temp_node->ptp_constraint_pt.rows() > 0) {
                    node->ptp_constraint_pt = temp_node->ptp_constraint_pt.row(0);
                } else if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ptp_constraint_pt = temp_node->ap_constraint_pt.row(0);
                } else if (temp_node->downsampled_pts.rows() > 0) {
                    node->ptp_constraint_pt = temp_node->downsampled_pts.row(0);
                } else {
                    return false;
                }

                if (node->ap_constraint_pt.rows() == 0 && temp_node->ap_constraint_pt.rows() > 0) {
                    node->ap_constraint_pt = temp_node->ap_constraint_pt.row(0);
                }
                if (temp_node->downsampled_pts.rows() > 0 && node->downsampled_pts.rows() == 0) {
                    node->downsampled_pts = temp_node->downsampled_pts;
                }
                logger().info("Added ptp_constraint_pt to existing node for timestamp {}",
                              timestamp);
                return true;
            }
        } catch (const std::exception& e) {
            logger().error("Failed to get selected point for existing node: {}", e.what());
        }
        return false;
    }

    // Helper function to create a new node for absolute point constraints
    std::shared_ptr<Node> create_node_for_absolute_point(uint64_t timestamp, int row, int col,
                                                         int return_idx) {
        try {
            auto temp_node = create_node_from_point(timestamp, row, col, return_idx);
            if (temp_node) {
                auto node = std::make_shared<Node>(temp_node->ts, temp_node->get_pose());
                // Prefer explicit AP selected point; fallback to first cloud
                // point
                if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ap_constraint_pt = temp_node->ap_constraint_pt.row(0);
                } else if (temp_node->downsampled_pts.rows() > 0) {
                    node->ap_constraint_pt = temp_node->downsampled_pts.row(0);
                } else {
                    logger().error("Temp node for ABS point has no selected/cloud points");
                    return nullptr;
                }
                node->ap_row = row;
                node->ap_col = col;
                node->ap_return = return_idx;
                node->ptp_constraint_pt = node->ap_constraint_pt;
                node->ptp_row = row;
                node->ptp_col = col;
                node->ptp_return = return_idx;
                if (temp_node->downsampled_pts.rows() > 0 && node->downsampled_pts.rows() == 0) {
                    // Only populate if we don't already have it
                    node->downsampled_pts = temp_node->downsampled_pts;
                }

                add_node_to_problem(node);
                add_node_neighbours_constraints(node);
                traj.timestamp_node_map[timestamp] = node;

                logger().info("Created new node with ap_constraint_pt for timestamp {}", timestamp);
                return node;
            }
        } catch (const std::exception& e) {
            logger().error("Failed to create node from point for timestamp {}: {}", timestamp,
                           e.what());
        }
        return nullptr;
    }

    // Helper function to create a new node for point-to-point constraints
    std::shared_ptr<Node> create_node_for_ptp(uint64_t timestamp, int row, int col,
                                              int return_idx) {
        try {
            auto temp_node = create_node_from_point(timestamp, row, col, return_idx);
            if (temp_node) {
                auto node = std::make_shared<Node>(temp_node->ts, temp_node->get_pose());
                // Prefer explicit ptp_constraint_pt, then ap_constraint_pt,
                // then fallback to first downsampled point
                if (temp_node->ptp_constraint_pt.rows() > 0) {
                    node->ptp_constraint_pt = temp_node->ptp_constraint_pt.row(0);
                } else if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ptp_constraint_pt = temp_node->ap_constraint_pt.row(0);
                } else if (temp_node->downsampled_pts.rows() > 0) {
                    node->ptp_constraint_pt = temp_node->downsampled_pts.row(0);
                } else {
                    logger().error("Temp node for PTP selection has no available point");
                    return nullptr;
                }
                node->ptp_row = row;
                node->ptp_col = col;
                node->ptp_return = return_idx;
                if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ap_constraint_pt = temp_node->ap_constraint_pt.row(0);
                    node->ap_row = row;
                    node->ap_col = col;
                    node->ap_return = return_idx;
                }
                if (temp_node->downsampled_pts.rows() > 0) {
                    node->downsampled_pts = temp_node->downsampled_pts;
                }

                add_node_to_problem(node);
                add_node_neighbours_constraints(node);
                traj.timestamp_node_map[timestamp] = node;

                logger().info("Created new node with ptp_constraint_pt for timestamp {}",
                              timestamp);
                return node;
            }
        } catch (const std::exception& e) {
            logger().error("Failed to create node from point for timestamp {}: {}", timestamp,
                           e.what());
        }
        return nullptr;
    }

    /**
     * @brief Adds a node to the Ceres optimization problem
     * @param node The node to add (handles quaternion parameterization and
     * parameter blocks)
     */
    void add_node_to_problem(const std::shared_ptr<Node>& node) {
        if (!node) {
            return;
        }
#if CERES_VERSION_MAJOR < 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR < 1)
        ceres::LocalParameterization* quaternion_parameterization =
            new ceres::QuaternionParameterization();
#else
        ceres::Manifold* quaternion_parameterization = new ceres::EigenQuaternionManifold();
#endif
        if (!problem.HasParameterBlock(node->rotation_coeffs_data())) {
            problem.AddParameterBlock(node->mutable_rotation_coeffs_data(), 4,
                                      quaternion_parameterization);
        } else {
            delete quaternion_parameterization;
        }

        if (!problem.HasParameterBlock(node->position_data())) {
            problem.AddParameterBlock(node->mutable_position_data(), 3);
        }
    }

    void add_constraint(impl::ConstraintImpl& constraint, bool is_user_constraint = false) {
        ceres::ResidualBlockId residual_id = constraint.add_to_problem(problem, loss_function);

        // Track user constraints for selective removal
        if (is_user_constraint) {
            user_constraint_residual_blocks.push_back(residual_id);
        }
    }

    // Overloaded version for adding constraints with ID tracking
    void add_constraint_with_id(impl::ConstraintImpl& constraint, uint32_t constraint_id) {
        ceres::ResidualBlockId residual_id = constraint.add_to_problem(problem, loss_function);

        // Track user constraints for selective removal
        user_constraint_residual_blocks.push_back(residual_id);

        // Map constraint ID to residual ID
        constraint_id_to_residual_map[constraint_id] = residual_id;
    }

    // Create constraint implementation from Constraint and add nodes with
    // neighbors if needed
    uint32_t add_base_constraint(Constraint* base_constraint) {
        if (base_constraint == nullptr) {
            throw std::invalid_argument("Cannot add null constraint");
        }

        // Checks if an ID is already taken by an existing user constraint.
        // We look at both the active residual map and stored config
        // constraints (id 0 is reserved for internal/trajectory constraints).
        auto constraint_id_in_use = [&](uint32_t constraint_id) {
            if (constraint_id == 0) {
                return false;
            }
            if (constraint_id_to_residual_map.find(constraint_id) !=
                constraint_id_to_residual_map.end()) {
                return true;
            }
            for (const auto& constraint : config.constraints) {
                if (!constraint || constraint.get() == base_constraint) {
                    continue;
                }
                if (constraint->get_constraint_id() == constraint_id) {
                    return true;
                }
            }
            return false;
        };

        uint32_t constraint_id = base_constraint->get_constraint_id();
        if (constraint_id_in_use(constraint_id)) {
            throw std::runtime_error("Constraint ID already in use: " +
                                     std::to_string(constraint_id));
        }
        // Dispatch to appropriate handler based on constraint type
        switch (base_constraint->get_type()) {
            case ConstraintType::ABSOLUTE_POSE:
                add_absolute_pose_constraint(
                    dynamic_cast<const AbsolutePoseConstraint&>(*base_constraint));
                break;
            case ConstraintType::POSE_TO_POSE:
                add_pose_to_pose_constraint(
                    dynamic_cast<const PoseToPoseConstraint&>(*base_constraint));
                break;
            case ConstraintType::POINT_TO_POINT:
                add_point_to_point_constraint(
                    dynamic_cast<const PointToPointConstraint&>(*base_constraint));
                break;
            case ConstraintType::ABSOLUTE_POINT:
                add_absolute_point_constraint(
                    dynamic_cast<const AbsolutePointConstraint&>(*base_constraint));
                break;
            default:
                logger().error("Unknown constraint type: {}",
                               static_cast<int>(base_constraint->get_type()));
                break;
        }

        return constraint_id;
    }

    void add_node_neighbours_constraints(const std::shared_ptr<Node>& node) {
        auto node_iter = traj.timestamp_node_map.upper_bound(node->ts);
        if (node_iter == traj.timestamp_node_map.end()) {
            logger().error("Error : Can't create a node for timestamp {}", node->ts);
            return;
        } else {
            // Add pose to pose constraint of the new pose and the next node in
            // traj
            PoseH diff = PoseH(node->get_pose()).inverse() * PoseH((node_iter->second)->get_pose());
            ouster::sdk::core::impl::PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraintImpl constraint(
                node, node_iter->second, diff_q.r(), diff_q.t(), config.traj_rotation_weight,
                {config.traj_translation_weight, config.traj_translation_weight,
                 config.traj_translation_weight});

            add_constraint(constraint);
        }
        if (std::distance(traj.timestamp_node_map.begin(), node_iter) > 2) {
            auto node_prev = *(std::prev(node_iter, 2));

            PoseH diff = PoseH(node_prev.second->get_pose()).inverse() * PoseH(node->get_pose());
            ouster::sdk::core::impl::PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraintImpl constraint(
                node_prev.second, node, diff_q.r(), diff_q.t(), config.traj_rotation_weight,
                {config.traj_translation_weight, config.traj_translation_weight,
                 config.traj_translation_weight});

            add_constraint(constraint);
        }
    }

    std::shared_ptr<Node> get_or_create_node_by_ts(uint64_t timestamp,
                                                   bool generate_point_cloud = false) {
        std::shared_ptr<Node> node = traj.get_node_by_ts(timestamp);
        if (node) {
            if (generate_point_cloud &&
                (node->downsampled_pts.rows() == 0 || node->icp_pts.rows() == 0 ||
                 node->icp_normals.rows() == 0)) {
                auto new_node =
                    traj.create_node_by_ts(timestamp, generate_point_cloud, downsample_voxel_size);
                if (new_node) {
                    if (new_node->downsampled_pts.rows() > 0) {
                        new_node->downsampled_pts = core::voxel_downsample_3d(
                            new_node->downsampled_pts, downsample_voxel_size, 1, 1,
                            core::VoxelDownsampleStrategy::AVERAGE_POINT);
                    }
                    node->downsampled_pts = new_node->downsampled_pts;
                    node->icp_pts = new_node->icp_pts;
                    node->icp_normals = new_node->icp_normals;
                }
            }
            return node;
        }

        node = traj.create_node_by_ts(timestamp, generate_point_cloud, downsample_voxel_size);
        if (node) {
            if (generate_point_cloud && node->downsampled_pts.rows() > 0) {
                node->downsampled_pts =
                    core::voxel_downsample_3d(node->downsampled_pts, downsample_voxel_size, 1, 1,
                                              core::VoxelDownsampleStrategy::AVERAGE_POINT);
            }
            add_node_to_problem(node);
            add_node_neighbours_constraints(node);
        } else {
            std::string msg =
                "Failed to create the node from timestamp " + std::to_string(timestamp) + ".";
            if (generate_point_cloud) {
                msg +=
                    " The timestamp may be invalid or not correspond to a "
                    "frame's first valid column.";
            }

            if (!traj.all_timestamps.empty()) {
                uint64_t min_ts = traj.all_timestamps.front();
                uint64_t max_ts = traj.all_timestamps.back();

                if (timestamp < min_ts || timestamp > max_ts) {
                    msg +=
                        " The timestamp may be outside the range of the OSF "
                        "file "
                        "[" +
                        std::to_string(min_ts) + ", " + std::to_string(max_ts) + "].";
                }
            }

            throw std::runtime_error(msg);
        }

        return node;
    }

    void add_traj_constraint(bool fix_first_node) {
        auto nodes = traj.timestamp_node_map;

        if (nodes.size() < 2) {
            logger().error("No constraints to add if there are fewer than 2 nodes");
            return;
        }

        logger().info(
            "Trajectory constraint translation weight {} and rotation weight "
            "{}",
            config.traj_translation_weight, config.traj_rotation_weight);

        auto it = nodes.begin();
        auto it_next = std::next(it);

        // Check if there are any absolute pose and absolute point constraints
        bool has_absolute_constraints = false;
        for (const auto& constraint : config.constraints) {
            if (constraint && (constraint->get_type() == ConstraintType::ABSOLUTE_POSE ||
                               constraint->get_type() == ConstraintType::ABSOLUTE_POINT)) {
                has_absolute_constraints = true;
                break;
            }
        }

        // Fix first node if explicitly requested OR there are no absolute
        // constraints
        bool should_fix_first_node = fix_first_node || (!has_absolute_constraints);

        if (should_fix_first_node) {
            const auto& first_node = it->second;
            problem.SetParameterBlockConstant(first_node->mutable_rotation_coeffs_data());
            problem.SetParameterBlockConstant(first_node->mutable_position_data());
            logger().info("Fixed first node as trajectory anchor");
            this->fix_first_node = true;
        } else {
            this->fix_first_node = false;
        }

        for (; it_next != nodes.end(); ++it, ++it_next) {
            const auto& node_before = it->second;
            const auto& node_after = it_next->second;

            PoseH diff = PoseH(node_before->get_pose()).inverse() * PoseH(node_after->get_pose());
            PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraintImpl constraint(
                node_before, node_after, diff_q.r(), diff_q.t(), config.traj_rotation_weight,
                {config.traj_translation_weight, config.traj_translation_weight,
                 config.traj_translation_weight});

            add_constraint(constraint);
        }
    }

    std::shared_ptr<Node> create_node_from_point(uint64_t timestamp, uint32_t row, uint32_t col,
                                                 uint32_t return_idx) {
        if (return_idx != 1 && return_idx != 2) {
            throw std::invalid_argument(
                "Fail to create a Node. return_idx can only be 1 or 2 but "
                "received " +
                std::to_string(return_idx));
        }

        const std::string field_name = return_idx == 1 ? "RANGE" : "RANGE2";
        const auto chan_field = return_idx == 1 ? ouster::sdk::core::ChanField::RANGE
                                                : ouster::sdk::core::ChanField::RANGE2;

        auto source = ouster::sdk::open_source(
            traj.input_osf_file,
            [field_name](auto& r) {
                r.index = true;
                r.field_names = std::vector<std::string>{field_name};
            },
            /* collate = false */ false);

        nonstd::optional<uint64_t> start_index_opt;
        nonstd::optional<uint64_t> end_index_opt;

        for (const auto& each : traj.timestamps_index_vec) {
            const uint64_t first_ts = each.first_col_ts;
            const uint64_t last_ts = each.last_col_ts;
            const uint32_t idx = each.frame_index;

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
            logger().error("OSF source is empty; cannot create node for ts {}", timestamp);
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

        for (const auto& frame_set : part_osf) {
            for (auto& frame_ptr : frame_set) {
                if (!frame_ptr) {
                    continue;
                }

                uint64_t ls_ts = 0;
                try {
                    ls_ts = frame_ptr->timestamp()[frame_ptr->get_first_valid_column()];
                } catch (const std::runtime_error& /*e*/) {
                    continue;
                }

                if (ls_ts == timestamp) {
                    // Validate row/col bounds against sensor metadata
                    const uint32_t height =
                        static_cast<uint32_t>(traj.info.format.pixels_per_column);
                    const uint32_t width = static_cast<uint32_t>(frame_ptr->w);
                    if (row >= height || col >= width) {
                        logger().error(
                            "Selected row/col out of bounds: row={} col={} "
                            "(h={} w={})",
                            row, col, height, width);
                        return nullptr;
                    }
                    // Use staggered column id from LidarFrame.
                    uint64_t col_ts = frame_ptr->timestamp()[col];
                    core::Matrix4dR mat = frame_ptr->get_column_pose(static_cast<int>(col));

                    // Use the selected return's range for cloud generation and
                    // selection (consistent with trajectory's LidarFrame-based
                    // cartesian)
                    const auto range = frame_ptr->field<uint32_t>(chan_field);

                    // Staggered range validity check will follow below
                    // Build cloud like trajectory path: cartesian on
                    // LidarFrame using the selected return's range for
                    // consistency across constraints
                    core::ArrayX3dR cloud_pts = (*traj.xyz_lut)(range);

                    const int first_col = frame_ptr->get_first_valid_column();
                    const PoseH first_pose(frame_ptr->get_column_pose(first_col));
                    const PoseH first_pose_inv(first_pose.inverse());

                    const int rows = static_cast<int>(frame_ptr->h);
                    const int cols = static_cast<int>(frame_ptr->w);
                    for (int col = 0; col < cols; ++col) {
                        const PoseH pose_c(frame_ptr->get_column_pose(col));
                        const PoseH rel(first_pose_inv * pose_c);
                        for (int row = 0; row < rows; ++row) {
                            const Eigen::Index idx = static_cast<Eigen::Index>(row) * cols + col;
                            if (idx >= cloud_pts.rows()) {
                                break;
                            }
                            Eigen::Vector3d point = cloud_pts.row(idx).matrix().transpose();
                            if (point.isZero(0.0)) {
                                continue;
                            }
                            point = rel * point;
                            cloud_pts.row(idx) = point.transpose().array();
                        }
                    }

                    int key_pts_index = static_cast<int>(static_cast<size_t>(row) * frame_ptr->w +
                                                         static_cast<size_t>(col));
                    // Validate the selected pixel has non-zero range
                    // (staggered). If zero, treat as error and stop.
                    if (range(row, col) == 0u) {
                        logger().error(
                            "Selected point has zero/invalid range at row={} "
                            "col={} ({})",
                            row, col, field_name);
                        return nullptr;
                    }
                    // Selected 3D point from the same cloud mapping
                    Eigen::Array<double, 1, 3> key_pts = cloud_pts.row(key_pts_index);

                    auto node = std::make_shared<Node>(col_ts, core::Matrix4dR(mat));
                    // Set both AP and PTP selected point so either constraint
                    // can use it
                    node->ap_constraint_pt = key_pts;
                    node->ap_row = static_cast<int>(row);
                    node->ap_col = static_cast<int>(col);
                    node->ap_return = static_cast<int>(return_idx);
                    node->ptp_constraint_pt = key_pts;
                    node->ptp_row = static_cast<int>(row);
                    node->ptp_col = static_cast<int>(col);
                    node->ptp_return = static_cast<int>(return_idx);
                    // Attach cloud consistent with trajectory (RANGE-based)
                    if (cloud_pts.rows() > 0) {
                        node->downsampled_pts =
                            core::voxel_downsample_3d(cloud_pts, downsample_voxel_size, 1, 1,
                                                      core::VoxelDownsampleStrategy::AVERAGE_POINT);
                    } else {
                        node->downsampled_pts = cloud_pts;
                    }

                    if (node) {
                        add_node_to_problem(node);
                        add_node_neighbours_constraints(node);
                        traj.timestamp_node_map[col_ts] = node;
                    }

                    return node;
                }
            }
        }

        return nullptr;
    }

    void clear_user_constraints_from_problem() {
        // Remove only user constraints, preserving trajectory constraints
        for (const auto& residual_id : user_constraint_residual_blocks) {
            problem.RemoveResidualBlock(residual_id);
        }

        user_constraint_residual_blocks.clear();
        constraint_id_to_residual_map.clear();
    }

    // Improved: Remove constraint by ID, updating all bookkeeping in one place
    bool remove_constraint_by_id(uint32_t constraint_id) {
        // Remove residual block from Ceres
        auto residual_it = constraint_id_to_residual_map.find(constraint_id);
        if (residual_it != constraint_id_to_residual_map.end()) {
            const auto residual_id = residual_it->second;
            problem.RemoveResidualBlock(residual_id);
            constraint_id_to_residual_map.erase(residual_it);
            user_constraint_residual_blocks.erase(
                std::remove(user_constraint_residual_blocks.begin(),
                            user_constraint_residual_blocks.end(), residual_id),
                user_constraint_residual_blocks.end());
        } else {
            logger().error("Constraint ID {} not found in residual map", constraint_id);
            return false;
        }

        // Remove from config.constraints
        auto it =
            std::find_if(config.constraints.begin(), config.constraints.end(),
                         [constraint_id](const std::unique_ptr<Constraint>& constraint) {
                             return constraint && constraint->get_constraint_id() == constraint_id;
                         });
        if (it != config.constraints.end()) {
            config.constraints.erase(it);
        } else {
            logger().warn("Constraint ID {} not found in config.constraints", constraint_id);
        }

        return true;
    }

    std::vector<std::shared_ptr<Node>> get_sampled_nodes(size_t count) {
        std::vector<std::shared_ptr<Node>> result;
        const size_t total_frames = traj.timestamps_index_vec.size();
        if (count == 0 || total_frames == 0) {
            return result;
        }

        const size_t samples = std::min(count, total_frames);
        result.reserve(samples);

        auto append_node = [&](size_t idx) {
            if (idx >= total_frames) {
                return;
            }
            const auto& info = traj.timestamps_index_vec[idx];
            const uint64_t frame_ts = info.first_col_ts;
            try {
                auto node = get_or_create_node_by_ts(frame_ts, true);
                if (!node) {
                    return;
                }
                if (node->downsampled_pts.rows() == 0) {
                    logger().warn("get_sampled_nodes: node {} has no downsampled points", frame_ts);
                    return;
                }
                result.push_back(std::move(node));
            } catch (const std::exception& e) {
                logger().warn("get_sampled_nodes: failed to prepare node for ts {}: {}", frame_ts,
                              e.what());
            }
        };

        if (samples == total_frames) {
            for (size_t idx = 0; idx < total_frames; ++idx) {
                append_node(idx);
            }
            return result;
        }

        if (samples == 1) {
            append_node(0);
            return result;
        }

        const double stride =
            static_cast<double>(total_frames - 1) / static_cast<double>(samples - 1);

        size_t previous_idx = 0;
        bool have_previous = false;
        for (size_t i = 0; i < samples; ++i) {
            const double position = stride * static_cast<double>(i);
            size_t idx = static_cast<size_t>(std::round(position));
            if (have_previous && idx <= previous_idx) {
                idx = previous_idx + 1;
            }
            if (idx >= total_frames) {
                idx = total_frames - 1;
            }
            append_node(idx);
            previous_idx = idx;
            have_previous = true;
            if (previous_idx == total_frames - 1) {
                break;
            }
        }

        return result;
    }
};

PoseOptimizer::PoseOptimizer(const std::string& osf_filename, const SolverConfig& config)
    : pimpl_(std::make_unique<Impl>(config, expand_home_path(osf_filename))) {
    pimpl_->add_traj_constraint(config.fix_first_node);
}

PoseOptimizer::PoseOptimizer(const std::string& osf_filename, double key_frame_distance) {
    SolverConfig config;
    config.key_frame_distance = key_frame_distance;
    pimpl_ = std::make_unique<PoseOptimizer::Impl>(config, expand_home_path(osf_filename));
    pimpl_->add_traj_constraint(config.fix_first_node);
}

PoseOptimizer::PoseOptimizer(const std::string& osf_filename, const std::string& config_filename) {
    SolverConfig config;
    ouster::sdk::core::ValidatorIssues issues;

    std::ifstream config_stream(config_filename);
    if (!config_stream.is_open()) {
        throw std::runtime_error("Could not open config file: " + config_filename);
    }
    std::string json_data((std::istreambuf_iterator<char>(config_stream)),
                          std::istreambuf_iterator<char>());

    bool ok = mapping::parse_and_validate_constraints(json_data, config, issues);
    if (!ok) {
        throw std::runtime_error("Error parsing config file: " + issues.to_string());
    }
    // Move parsed config into Impl
    pimpl_ = std::make_unique<PoseOptimizer::Impl>(config, expand_home_path(osf_filename));
    pimpl_->add_traj_constraint(config.fix_first_node);
}

PoseOptimizer::~PoseOptimizer() = default;

core::Matrix4dR PoseOptimizer::initialize_trajectory_alignment() {
    if (!pimpl_) {
        return core::Matrix4dR::Identity();
    }
    return pimpl_->initialize_trajectory_alignment();
}

double PoseOptimizer::solve(uint32_t steps) {
    if (steps > 0) {
        pimpl_->options.max_num_iterations = static_cast<int>(steps);
        logger().info("Incremental optimize with {} iterations.", steps);
    } else {
        logger().info("Running full optimization with default max iterations ({}).",
                      pimpl_->options.max_num_iterations);
        pimpl_->options.max_num_iterations = static_cast<int>(pimpl_->config.max_num_iterations);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(pimpl_->options, &(pimpl_->problem), &summary);
    mark_cached_node_poses_dirty(pimpl_->traj);

    logger().info("Initial Cost: {}", summary.initial_cost);
    logger().info("Final   Cost: {}", summary.final_cost);
    pimpl_->cost_number_ = summary.final_cost;
    pimpl_->total_iterations_ += static_cast<uint64_t>(summary.iterations.size());
    return summary.final_cost;
}

void PoseOptimizer::save(const std::string& osf_filename) {
    logger().info("Saving the results into {}", osf_filename);
    pimpl_->traj.save(osf_filename);
}

std::shared_ptr<Node> PoseOptimizer::get_node(uint64_t timestamp) const {
    return pimpl_->traj.get_node_by_ts(timestamp);
}

double PoseOptimizer::get_cost_value() const {
    return pimpl_->cost_number_;
}

uint64_t PoseOptimizer::get_total_iterations() const {
    if (!pimpl_) {
        return 0;
    }
    return pimpl_->total_iterations_;
}

std::vector<std::shared_ptr<Node>> PoseOptimizer::get_sampled_nodes(size_t count) const {
    if (!pimpl_) {
        return {};
    }
    return pimpl_->get_sampled_nodes(count);
}

size_t PoseOptimizer::add_relative_loop_constraints(double min_distance_m, double cell_size_m,
                                                    double icp_score_threshold) {
    if (!pimpl_) {
        return 0;
    }
    if (min_distance_m <= 0.0) {
        throw std::invalid_argument("min_distance_m must be > 0");
    }
    if (cell_size_m <= 0.0) {
        throw std::invalid_argument("cell_size_m must be > 0");
    }
    if (icp_score_threshold < 0.0 || icp_score_threshold > 1.0) {
        throw std::invalid_argument("icp_score_threshold must be within [0, 1]");
    }

    auto nodes = pimpl_->traj.get_valid_nodes(SamplingMode::KEY_FRAMES);
    if (nodes.size() < 2) {
        return 0;
    }

    GridCellLoopDetector detector(min_distance_m, cell_size_m);
    double dist_travelled = 0.0;
    bool have_prev = false;
    Eigen::Vector3d prev_pos = Eigen::Vector3d::Zero();
    size_t added = 0;
    // Enforce a minimum time separation between loop-pair endpoints (10
    // seconds).
    const uint64_t min_ts_sep_ns = 10 * static_cast<uint64_t>(1000000000);

    auto ensure_node_cloud = [&](const std::shared_ptr<Node>& candidate) {
        if (!candidate) {
            return false;
        }
        if (candidate->downsampled_pts.rows() > 0) {
            return true;
        }
        try {
            auto filled = pimpl_->traj.create_node_by_ts(candidate->ts, true);
            return filled && filled->downsampled_pts.rows() > 0;
        } catch (const std::exception& e) {
            logger().warn("Skipping loop constraint at timestamp {}: {}", candidate->ts, e.what());
            return false;
        }
    };

    for (size_t idx = 0; idx < nodes.size(); ++idx) {
        const auto& node = nodes[idx];
        const Eigen::Vector3d pos = node->get_pose().block<3, 1>(0, 3);
        if (have_prev) {
            dist_travelled += (pos - prev_pos).norm();
        }
        auto loop_indices = detector.add_pose(pos, dist_travelled);
        if (!loop_indices.empty()) {
            using CellDescriptor = GridCellLoopDetector::CellDescriptor;
            auto cell_of = [&](const Eigen::Vector3d& pos) -> CellDescriptor {
                return GridCellLoopDetector::compute_cell(pos, cell_size_m);
            };
            const CellDescriptor curr_cell = cell_of(pos);
            const uint64_t ts_curr = node->ts;

            // `loop_indices` are already sorted nearest-first by the detector's
            // spatial-hash search, so choose the first candidate that passes
            // cell/time gates instead of re-ranking by distance.
            auto pick_first_valid = [&](bool require_same_cell, size_t& out_idx) -> bool {
                for (size_t old_idx : loop_indices) {
                    const auto& old_node = nodes[old_idx];
                    if (!old_node) {
                        continue;
                    }
                    const Eigen::Vector3d old_pos = old_node->get_pose().block<3, 1>(0, 3);
                    if (require_same_cell && !(curr_cell == cell_of(old_pos))) {
                        continue;
                    }
                    const uint64_t ts_old = old_node->ts;
                    // Skip candidates too close in time to avoid adjacent
                    // pairs.
                    const uint64_t ts_diff =
                        (ts_curr > ts_old) ? (ts_curr - ts_old) : (ts_old - ts_curr);
                    if (ts_diff <= min_ts_sep_ns) {
                        continue;
                    }
                    out_idx = old_idx;
                    return true;
                }
                return false;
            };

            size_t best_idx = 0;
            bool found = pick_first_valid(true, best_idx);
            if (!found) {
                found = pick_first_valid(false, best_idx);
            }

            if (found) {
                if (!ensure_node_cloud(nodes[best_idx]) || !ensure_node_cloud(node)) {
                    prev_pos = pos;
                    have_prev = true;
                    continue;
                }
                auto icp_result = run_icp(pimpl_->traj, pimpl_->get_icp_frame_set_source(),
                                          nodes[best_idx], node);
                const PoseH& diff = icp_result.first;
                const double icp_score = icp_result.second;
                if (icp_score < icp_score_threshold) {
                    logger().info(
                        "Skipping auto loop pair {}-{}: ICP score {} below "
                        "threshold {}",
                        nodes[best_idx]->ts, node->ts, icp_score, icp_score_threshold);
                    prev_pos = pos;
                    have_prev = true;
                    continue;
                }
                auto constraint = std::make_unique<PoseToPoseConstraint>(nodes[best_idx]->ts,
                                                                         node->ts, diff.matrix());
                add_constraint(std::move(constraint));
                ++added;
            }
        }
        prev_pos = pos;
        have_prev = true;
    }

    return added;
}

std::vector<uint64_t> PoseOptimizer::get_timestamps(SamplingMode type) const {
    if (type == SamplingMode::KEY_FRAMES || type == SamplingMode::COLUMNS) {
        return pimpl_->traj.get_timestamps(type);
    } else {
        logger().error(
            "Invalid SamplingMode: {}. Use SamplingMode::KEY_FRAMES or "
            "SamplingMode::COLUMNS.",
            static_cast<int>(type));
        throw std::invalid_argument("Invalid SamplingMode: " +
                                    std::to_string(static_cast<int>(type)));
    }
}

std::vector<core::Matrix4dR> PoseOptimizer::get_poses(SamplingMode type) {
    if (type == SamplingMode::KEY_FRAMES || type == SamplingMode::COLUMNS) {
        return pimpl_->traj.get_poses(type);
    } else {
        logger().error(
            "Invalid SamplingMode: {}. Use SamplingMode::KEY_FRAMES or "
            "SamplingMode::COLUMNS.",
            static_cast<int>(type));
        throw std::invalid_argument("Invalid SamplingMode: " +
                                    std::to_string(static_cast<int>(type)));
    }
}

double PoseOptimizer::get_key_frame_distance() const {
    return pimpl_->config.key_frame_distance;
}

std::vector<std::unique_ptr<Constraint>> PoseOptimizer::get_constraints() const {
    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.reserve(pimpl_->config.constraints.size());

    for (const auto& constraint : pimpl_->config.constraints) {
        if (constraint) {
            constraints.push_back(constraint->clone());
        }
    }

    return constraints;
}

void PoseOptimizer::set_constraints(std::vector<std::unique_ptr<Constraint>> constraints) {
    try {
        clear_constraints();

        pimpl_->config.constraints = std::move(constraints);

        if (pimpl_) {
            pimpl_->process_config_constraints();
        }

    } catch (const std::exception& e) {
        logger().error("Failed to set constraints: {}", e.what());
        throw;
    }
}

void PoseOptimizer::clear_constraints() {
    // Log how many constraints are being removed
    size_t constraint_count = pimpl_->config.constraints.size();
    if (constraint_count > 0) {
        logger().info("Clearing {} constraint(s) from PoseOptimizer", constraint_count);
    }

    // First clear constraints from the Ceres problem so any residuals that
    // reference constraint implementations are removed while those
    // implementations are still valid.
    pimpl_->clear_user_constraints_from_problem();

    // Then clear constraints from the stored config (release ownership).
    pimpl_->config.constraints.clear();
}

size_t PoseOptimizer::add_absolute_gps_constraints(double min_space_m,
                                                   const Eigen::Array3d& translation_weights) {
    if (min_space_m <= 0.0) {
        throw std::invalid_argument("min_space_m must be > 0");
    }

    if ((translation_weights < 0.0).any()) {
        throw std::invalid_argument("translation_weights must be non-negative");
    }

    logger().info(
        "Applying auto GPS constraint translation weights (WX, WY, WZ) = "
        "({}, {}, {})",
        translation_weights[0], translation_weights[1], translation_weights[2]);

    auto source = ouster::sdk::open_source(
        pimpl_->traj.input_osf_file,
        [](auto& req) {
            req.index = true;
            // Do not filter fields here. GPS fields are optional in OSF, and
            // auto-GPS should simply skip constraint generation when they are
            // absent.
        },
        /* collate = */ true, /* sensor_idx = */ 0);

    size_t added_constraints = 0;
    bool has_gps_fields = false;
    bool have_origin = false;
    double lat0 = 0.0;
    double lon0 = 0.0;

    double distance_since_last_constraint_m = min_space_m;
    bool have_prev_pos = false;
    Eigen::Vector2d prev_pos_xy = Eigen::Vector2d::Zero();

    size_t frame_index = 0;
    for (const auto& frame_set : source) {
        if (frame_index++ == 0) {
            continue;
        }
        if (frame_set.size() == 0) {
            continue;
        }
        const auto& frame_ptr = frame_set[0];
        if (!frame_ptr) {
            continue;
        }

        const bool frame_has_gps =
            frame_ptr->has_field(ouster::sdk::core::ChanField::POSITION_LAT_LONG) &&
            frame_ptr->has_field(ouster::sdk::core::ChanField::POSITION_TIMESTAMP);
        if (frame_has_gps) {
            has_gps_fields = true;
        }

        PoseH frame_pose;
        try {
            const int first_col = frame_ptr->get_first_valid_column();
            frame_pose = frame_ptr->get_column_pose(first_col);
        } catch (const std::exception&) {
        }

        const bool is_identity = frame_pose.isIdentity(1e-6);
        if (!is_identity) {
            Eigen::Vector2d pos_xy(frame_pose(0, 3), frame_pose(1, 3));
            if (have_prev_pos) {
                distance_since_last_constraint_m += (pos_xy - prev_pos_xy).norm();
            }
            prev_pos_xy = pos_xy;
            have_prev_pos = true;
        }

        if (added_constraints > 0 && distance_since_last_constraint_m < min_space_m) {
            continue;
        }

        if (!frame_has_gps) {
            continue;
        }

        ouster::sdk::core::ConstArrayView2<double> lat_long(
            frame_ptr->field(ouster::sdk::core::ChanField::POSITION_LAT_LONG));
        ouster::sdk::core::ConstArrayView1<uint64_t> gps_ts(
            frame_ptr->field(ouster::sdk::core::ChanField::POSITION_TIMESTAMP));

        if (lat_long.shape[0] == 0 || gps_ts.shape[0] == 0) {
            continue;
        }

        if (lat_long.shape[0] != gps_ts.shape[0]) {
            throw std::runtime_error("GPS field size mismatch: POSITION_LAT_LONG rows " +
                                     std::to_string(lat_long.shape[0]) +
                                     " vs POSITION_TIMESTAMP rows " +
                                     std::to_string(gps_ts.shape[0]));
        }

        const uint32_t last_idx = lat_long.shape[0] - 1;
        const double lat = lat_long(last_idx, 0);
        const double lon = lat_long(last_idx, 1);
        if (!std::isfinite(lat) || !std::isfinite(lon)) {
            continue;
        }

        const uint64_t gps_stamp = gps_ts(last_idx);

        if (!have_origin) {
            lat0 = lat;
            lon0 = lon;
            have_origin = true;
        }

        core::Matrix4dR pose = core::Matrix4dR::Identity();
        const Eigen::Vector2d xy = relative_xy_from_wgs84(lat, lon, lat0, lon0);
        pose(0, 3) = xy.x();
        pose(1, 3) = xy.y();
        if (!is_identity) {
            pose(2, 3) = frame_pose(2, 3);
        }

        const double wz = is_identity ? 0.0 : translation_weights[2];
        const Eigen::Array3d per_constraint_weights(translation_weights[0], translation_weights[1],
                                                    wz);

        auto constraint =
            std::make_unique<AbsolutePoseConstraint>(gps_stamp, pose, 0.0, per_constraint_weights);
        add_constraint(std::move(constraint));
        ++added_constraints;
        distance_since_last_constraint_m = 0.0;
    }

    if (frame_index < 2) {
        throw std::runtime_error(frame_index == 0 ? "No frames found in the source"
                                                  : "Not enough frames to generate GPS constraints "
                                                    "(need at least 2)");
    }

    if (added_constraints == 0 && !has_gps_fields) {
        logger().warn(
            "No GPS fields POSITION_LAT_LONG/POSITION_TIMESTAMP found in "
            "source {}; skipping auto GPS constraints.",
            pimpl_->traj.input_osf_file);
    }

    return added_constraints;
}

void save_trajectory(const std::string& filename, const std::vector<uint64_t>& timestamps,
                     const std::vector<core::Matrix4dR>& poses, const std::string& file_type) {
    if (timestamps.size() != poses.size()) {
        logger().error("Timestamps and poses size mismatch: {} vs {}", timestamps.size(),
                       poses.size());
        throw std::runtime_error("Timestamps and poses size mismatch");
    }

    std::ofstream file(filename, std::ios::out);
    if (!file) {
        logger().error("Unable to open file: {}", filename);
        throw std::runtime_error("Unable to open file: " + filename);
    }

    const size_t num_poses = timestamps.size();
    if (file_type == "csv") {
        file << "timestamp,tx,ty,tz,qx,qy,qz,qw\n";
        for (size_t i = 0; i < num_poses; ++i) {
            uint64_t timestamp = timestamps[i];
            const auto& pose_matrix = poses[i];
            ouster::sdk::core::impl::PoseH poseh(pose_matrix);
            const ouster::sdk::core::impl::PoseQ poseq = poseh.log().q();
            const Eigen::Vector3d translation = poseh.t();
            const Eigen::Quaterniond quaternion = poseq.r();

            file << timestamp << ',' << translation.x() << ',' << translation.y() << ','
                 << translation.z() << ',' << quaternion.x() << ',' << quaternion.y() << ','
                 << quaternion.z() << ',' << quaternion.w() << '\n';
        }

    } else if (file_type == "tum") {
        for (size_t i = 0; i < num_poses; ++i) {
            uint64_t timestamp = timestamps[i];
            const auto& pose_matrix = poses[i];
            ouster::sdk::core::impl::PoseH poseh(pose_matrix);
            const ouster::sdk::core::impl::PoseQ poseq = poseh.log().q();
            const Eigen::Vector3d translation = poseh.t();
            const Eigen::Quaterniond quaternion = poseq.r();

            file << timestamp << ' ' << translation.x() << ' ' << translation.y() << ' '
                 << translation.z() << ' ' << quaternion.x() << ' ' << quaternion.y() << ' '
                 << quaternion.z() << ' ' << quaternion.w() << '\n';
        }

    } else {
        logger().error("Unsupported file type: {}. Currently support 'csv' or 'tum'.", file_type);
        throw std::runtime_error("Unsupported file type: " + file_type);
    }

    logger().info("Trajectory successfully saved to {}", filename);
}

uint32_t PoseOptimizer::add_constraint(std::unique_ptr<Constraint> constraint) {
    if (!constraint) {
        throw std::invalid_argument("Cannot add null constraint");
    }

    try {
        Constraint* constraint_ptr = constraint.get();
        const uint32_t constraint_id = pimpl_->add_base_constraint(constraint_ptr);

        // Store the Constraint in Impl's config for saving
        pimpl_->config.constraints.push_back(std::move(constraint));
        return constraint_id;

    } catch (const std::exception& e) {
        logger().error("Failed to add constraint: {}", e.what());
        throw;
    }
}

void PoseOptimizer::remove_constraint(uint32_t constraint_id) {
    if (constraint_id == 0) {
        throw std::invalid_argument("Cannot remove constraint with ID 0 (not a user constraint)");
    }

    try {
        bool removed = pimpl_->remove_constraint_by_id(constraint_id);
        if (removed) {
            logger().info("Successfully removed constraint with ID {}", constraint_id);
        } else {
            throw std::runtime_error("Failed to remove constraint with ID " +
                                     std::to_string(constraint_id) + " (not found)");
        }
    } catch (const std::exception& e) {
        logger().error("Exception while removing constraint with ID {}: {}", constraint_id,
                       e.what());
        throw;
    }
}

void PoseOptimizer::save_config(const std::string& config_filename) {
    try {
        std::string json_string = serialize_constraints_to_json(pimpl_->config);

        std::ofstream outfile(config_filename);
        if (!outfile.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + config_filename);
        }

        outfile << json_string;
        outfile.close();

        if (outfile.fail()) {
            throw std::runtime_error("Failed to write constraints to file: " + config_filename);
        }

        logger().info("Successfully saved {} constraint(s) to: {}",
                      pimpl_->config.constraints.size(), config_filename);

    } catch (const std::exception& e) {
        logger().error("Error saving constraints to file {}: {}", config_filename, e.what());
        throw;
    }
}

void PoseOptimizer::set_solver_step_callback(std::function<void()> func) {
    pimpl_->solver_step_functor_ = std::move(func);
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

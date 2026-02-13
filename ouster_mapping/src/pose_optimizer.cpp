#define _ENABLE_EXTENDED_ALIGNED_STORAGE

#include "ouster/pose_optimizer.h"

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

#include "nonstd/optional.hpp"
#include "ouster/constraint_config.h"
#include "ouster/impl/absolute_point_constraint_impl.h"
#include "ouster/impl/absolute_pose_constraint_impl.h"
#include "ouster/impl/constraint_impl.h"
#include "ouster/impl/logging.h"
#include "ouster/impl/point_to_point_constraint_impl.h"
#include "ouster/impl/pose_to_pose_constraint_impl.h"
#include "ouster/impl/trajectory.h"
#include "ouster/impl/transformation.h"
#include "ouster/impl/utils.h"
#include "ouster/lidar_scan.h"
#include "ouster/metadata.h"
#include "ouster/open_source.h"
#include "ouster/pose_optimizer_constraint.h"
#include "ouster/pose_optimizer_node.h"

// Third-party includes
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/version.h>

#if CERES_VERSION_MAJOR < 2 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR < 1)
#include <ceres/local_parameterization.h>
#else
#include <ceres/manifold.h>
#endif

using ouster::sdk::core::Matrix4dR;
using ouster::sdk::core::impl::PoseH;
using ouster::sdk::core::impl::PoseQ;

using ouster::sdk::core::logger;

namespace ouster {
namespace sdk {
namespace mapping {

// It starts at 1 because 0 is reserved for traj constraints
std::atomic<uint32_t> Constraint::next_constraint_id_{1};

namespace {

PoseH run_icp(std::shared_ptr<Node> node_first,
              std::shared_ptr<Node> node_second) {
    PoseH pose1(node_first->get_pose());
    PoseH pose2(node_second->get_pose());
    PoseH initial_guess = PoseH(pose1.inverse() * pose2);

    Eigen::Matrix4d icp_relative_pose =
        run_kiss_icp_matching(node_second->downsampled_pts,
                              node_first->downsampled_pts, initial_guess);

    return PoseH(icp_relative_pose);
}

ceres::LossFunction* create_loss_function(LossFunction loss_func,
                                          double scale) {
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
    std::function<void()> solver_step_functor_{};
    struct StepCallback : public ceres::IterationCallback {
        Impl* self;
        explicit StepCallback(Impl* s) : self(s) {}
        ceres::CallbackReturnType operator()(
            const ceres::IterationSummary&) override {
            if (self && self->solver_step_functor_) {
                self->solver_step_functor_();
            }
            return ceres::SOLVER_CONTINUE;
        }
    };
    std::unique_ptr<StepCallback> step_cb_holder_;
    bool step_cb_registered_{false};
    // Store last final cost from Ceres summary
    double cost_number_{-1};

    // Total number of iterations executed across all solve() calls
    uint64_t total_iterations_{0};

    // Track only user constraints (from config.constraints) for selective
    // removal
    std::vector<ceres::ResidualBlockId> user_constraint_residual_blocks;

    // Map constraint ID to residual block ID for individual constraint removal
    std::unordered_map<uint32_t, ceres::ResidualBlockId>
        constraint_id_to_residual_map;

    // Downsample voxel size for point clouds used in ICP
    const double downsample_voxel_size = 0.05;

    Impl(const SolverConfig& solver_options, const std::string& osf_filename)
        : config(solver_options) {
        options.max_num_iterations = config.max_num_iterations;
        options.function_tolerance = config.function_tolerance;
        options.gradient_tolerance = config.gradient_tolerance;
        options.parameter_tolerance = config.parameter_tolerance;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = config.process_printout;

        logger().info("Initializing Pose Optimizer ...");

        try {
            if (solver_options.loss_function == LossFunction::TRIVIAL_LOSS) {
                // TRIVIAL_LOSS ignores the scale entirely
                loss_function =
                    create_loss_function(LossFunction::TRIVIAL_LOSS, 0.0);
                logger().info("Using TRIVIAL_LOSS function");
            } else {
                // All other losses do use the scale
                loss_function = create_loss_function(
                    solver_options.loss_function, solver_options.loss_scale);
                logger().info("Using loss function: {} with scale: {}",
                              to_string(solver_options.loss_function),
                              solver_options.loss_scale);
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

        logger().info("Processing {} constraint(s) from config",
                      config.constraints.size());

        for (size_t i = 0; i < config.constraints.size(); ++i) {
            const auto& constraint = config.constraints[i];
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
    bool initialize_trajectory_alignment() {
        struct AlignmentPair {
            Eigen::Vector3d trajectory_point;
            Eigen::Vector3d target_point;
            double weight;
        };

        // Build correspondence pairs used by the weighted SVD solve.
        std::vector<AlignmentPair> pairs;
        pairs.reserve(config.constraints.size());

        auto weight_from_translation = [](const Eigen::Array3d& weights) {
            return weights.maxCoeff();
        };

        auto add_pose_pair = [&](const PoseH& node_pose, const PoseH& target,
                                 const Eigen::Array3d& translation_weights) {
            const double weight_value =
                weight_from_translation(translation_weights);
            if (weight_value <= 0.0) return;
            pairs.push_back({node_pose.t(), target.t(), weight_value});
        };

        auto add_point_pair = [&](const PoseH& node_pose,
                                  const Eigen::Vector3d& source_point,
                                  const Eigen::Vector3d& target_point,
                                  const Eigen::Array3d& translation_weights) {
            const double weight_value =
                weight_from_translation(translation_weights);
            if (weight_value <= 0.0) return;
            Eigen::Vector3d world_point =
                node_pose.r() * source_point + node_pose.t();
            pairs.push_back({world_point, target_point, weight_value});
        };

        for (const auto& constraint : config.constraints) {
            if (!constraint) continue;
            const auto type = constraint->get_type();

            uint64_t timestamp = 0;
            if (type == ConstraintType::ABSOLUTE_POSE) {
                timestamp =
                    static_cast<const AbsolutePoseConstraint*>(constraint.get())
                        ->timestamp;
            } else if (type == ConstraintType::ABSOLUTE_POINT) {
                timestamp = static_cast<const AbsolutePointConstraint*>(
                                constraint.get())
                                ->timestamp;
            } else {
                continue;
            }

            auto node = traj.get_node_ts(timestamp);
            if (!node) continue;
            node->update_pose();
            PoseH node_pose(node->get_pose());

            if (type == ConstraintType::ABSOLUTE_POSE) {
                auto* abs_pose_constraint =
                    static_cast<const AbsolutePoseConstraint*>(
                        constraint.get());
                PoseH target_pose(abs_pose_constraint->pose);
                add_pose_pair(node_pose, target_pose,
                              abs_pose_constraint->translation_weights);
            } else if (type == ConstraintType::ABSOLUTE_POINT) {
                auto* abs_point_constraint =
                    static_cast<const AbsolutePointConstraint*>(
                        constraint.get());
                if (node->ap_constraint_pt.rows() == 0) continue;
                Eigen::Vector3d source_point =
                    node->ap_constraint_pt.row(0).matrix().transpose();
                add_point_pair(node_pose, source_point,
                               abs_point_constraint->absolute_position,
                               abs_point_constraint->translation_weights);
            }
        }

        if (pairs.empty()) {
            logger().warn(
                "initialize_trajectory_alignment(): no usable absolute "
                "constraints");
            return false;
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
            return false;
        }

        src_centroid /= total_weight;
        dst_centroid /= total_weight;

        // Weighted cross-covariance used by the SVD rotation estimate.
        Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
        for (const auto& pair_data : pairs) {
            const Eigen::Vector3d src_centered =
                pair_data.trajectory_point - src_centroid;
            const Eigen::Vector3d dst_centered =
                pair_data.target_point - dst_centroid;
            covariance_matrix +=
                pair_data.weight * src_centered * dst_centered.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d u_matrix = svd.matrixU();
        Eigen::Matrix3d v_matrix = svd.matrixV();
        Eigen::Matrix3d reflection_fix = Eigen::Matrix3d::Identity();
        if ((v_matrix * u_matrix.transpose()).determinant() < 0.0)
            reflection_fix(2, 2) = -1.0;
        // Kabsch rotation: R = V * S * U^T, with S fixing reflections.
        Eigen::Matrix3d rotation_matrix =
            v_matrix * reflection_fix * u_matrix.transpose();
        Eigen::Vector3d translation_vector =
            dst_centroid - rotation_matrix * src_centroid;

        const double rotation_angle =
            std::abs(Eigen::AngleAxisd(rotation_matrix).angle());
        const double translation_norm = translation_vector.norm();
        const double translation_threshold = 0.5;            // meters
        const double rotation_threshold = 1.0 * M_PI / 180;  // 1 degree
        if (translation_norm < translation_threshold &&
            rotation_angle < rotation_threshold) {
            logger().info(
                "initialize_trajectory_alignment skipped tiny alignment");
            return false;
        }

        // Apply the alignment transform to every node/pose as a left-multiply.
        PoseH transform;
        transform.set_rot(rotation_matrix);
        transform.set_trans(translation_vector);

        for (auto& timestamp_node_entry : traj.timestamp_node_map) {
            auto& node_ptr = timestamp_node_entry.second;
            if (!node_ptr) continue;
            PoseH updated_pose = transform * PoseH(node_ptr->get_pose());
            node_ptr->rotation =
                Eigen::Quaterniond(updated_pose.r()).normalized();
            node_ptr->position = updated_pose.t();
            node_ptr->update_pose();
        }
        for (auto& pose_matrix : traj.all_poses)
            pose_matrix = transform * pose_matrix;

        logger().info("initialize_trajectory_alignment applied");
        return true;
    }

    // =============================================================================
    // CONSTRAINT ADDITION METHODS
    // =============================================================================

    /**
     * @brief Adds an absolute pose constraint to fix a node at a specific
     * global pose
     * @param constraint The absolute pose constraint to add
     */
    void add_absolute_pose_constraint(const Constraint* constraint) {
        auto abs_constraint =
            static_cast<const AbsolutePoseConstraint*>(constraint);

        try {
            // For absolute pose constraints, find an existing node in the
            // trajectory
            auto node = traj.get_node_ts(abs_constraint->timestamp);
            if (!node) {
                // Absolute pose does not require point clouds; allow
                // interpolation without generating one
                node = get_or_create_node_ts(abs_constraint->timestamp, false);
                if (!node) {
                    logger().error("Failed to create node from timestamp {}",
                                   abs_constraint->timestamp);
                    return;
                }
            }
            // Use weights
            const double rotation_weight = abs_constraint->rotation_weight;
            const auto& translation_weights =
                abs_constraint->translation_weights;

            // Unfix the node if it's already fixed so it can be optimized by
            // the constraint
            if (problem.IsParameterBlockConstant(
                    node->rotation.coeffs().data()) ||
                problem.IsParameterBlockConstant(node->position.data())) {
                problem.SetParameterBlockVariable(
                    node->rotation.coeffs().data());
                problem.SetParameterBlockVariable(node->position.data());
            }

            // Create target node (NOT added to trajectory - it's a fixed
            // reference)
            auto target_node = std::make_shared<Node>(abs_constraint->timestamp,
                                                      abs_constraint->pose);

            impl::AbsolutePoseConstraintImpl constraint_impl(
                node, target_node, rotation_weight, translation_weights);
            add_constraint_with_id(constraint_impl,
                                   abs_constraint->get_constraint_id());

            release_first_node_anchor();

            logger().info(
                "Successfully processed absolute pose constraint id {} for "
                "timestamp {}",
                abs_constraint->get_constraint_id(), abs_constraint->timestamp);
        } catch (const std::exception& e) {
            logger().error(
                "Error creating absolute pose constraint for timestamp {}: {}",
                abs_constraint->timestamp, e.what());
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

        if (problem.IsParameterBlockConstant(
                first_node->rotation.coeffs().data())) {
            problem.SetParameterBlockVariable(
                first_node->rotation.coeffs().data());
        }
        if (problem.IsParameterBlockConstant(first_node->position.data())) {
            problem.SetParameterBlockVariable(first_node->position.data());
        }

        fix_first_node = false;
        logger().info(
            "Released first node anchor after adding an absolute constraint.");
    }

    /**
     * @brief Adds a pose-to-pose constraint between two trajectory nodes
     * @param constraint The pose-to-pose constraint to add (can be ICP-based or
     * fixed transform)
     */
    void add_pose_to_pose_constraint(const Constraint* constraint) {
        auto pose_constraint =
            static_cast<const PoseToPoseConstraint*>(constraint);

        // Use weights directly (no conversion needed)
        const double rotation_weight = pose_constraint->rotation_weight;
        const auto& translation_weights = pose_constraint->translation_weights;

        try {
            // Check if relative_pose is identity matrix - this indicates
            // ICP-based constraint
            PoseH relative_pose(pose_constraint->relative_pose);

            if (relative_pose.isIdentity()) {
                // ICP-based constraint: need point clouds for both nodes
                logger().info(
                    "Creating pose to pose constraint between {} and {}. Align "
                    "by ICP",
                    pose_constraint->timestamp1, pose_constraint->timestamp2);

                std::shared_ptr<Node> node1, node2;
                try {
                    node1 = get_or_create_node_ts(pose_constraint->timestamp1,
                                                  true);
                } catch (const std::exception& e) {
                    logger().error(
                        "Failed to create node1 with point cloud for timestamp "
                        "{}: {}",
                        pose_constraint->timestamp1, e.what());
                    return;
                }

                try {
                    node2 = get_or_create_node_ts(pose_constraint->timestamp2,
                                                  true);
                } catch (const std::exception& e) {
                    logger().error(
                        "Failed to create node2 with point cloud for timestamp "
                        "{}: {}",
                        pose_constraint->timestamp2, e.what());
                    return;
                }

                if (node1 && node2) {
                    auto diff = run_icp(node1, node2);
                    std::stringstream string_stream;
                    string_stream
                        << diff.matrix().format(EIGEN_MATRIX_PRINT_FMT);
                    logger().info(
                        "Run ICP between scans at {} and {}. The ICP "
                        "transformation matrix:\n{}",
                        node1->ts, node2->ts, string_stream.str());
                    impl::PoseToPoseConstraintImpl constraint_impl(
                        node1, node2, diff, rotation_weight,
                        translation_weights);
                    add_constraint_with_id(
                        constraint_impl, pose_constraint->get_constraint_id());
                    logger().info(
                        "Successfully added pose to pose constraint id {} "
                        "between {} and {}",
                        pose_constraint->get_constraint_id(),
                        pose_constraint->timestamp1,
                        pose_constraint->timestamp2);
                } else {
                    logger().error(
                        "Failed to create nodes for pose to pose constraint. "
                        "Check that timestamps {} and {} correspond to valid "
                        "scans in the OSF file.",
                        pose_constraint->timestamp1,
                        pose_constraint->timestamp2);
                    return;
                }
            } else {
                // Stored pose constraint: generate point clouds for these nodes
                logger().info(
                    "Creating pose to pose constraint between {} and {} using "
                    "the given transformation matrix",
                    pose_constraint->timestamp1, pose_constraint->timestamp2);

                auto node1 =
                    get_or_create_node_ts(pose_constraint->timestamp1, true);
                auto node2 =
                    get_or_create_node_ts(pose_constraint->timestamp2, true);

                if (node1 && node2) {
                    impl::PoseToPoseConstraintImpl constraint_impl(
                        node1, node2, relative_pose, rotation_weight,
                        translation_weights);
                    add_constraint_with_id(
                        constraint_impl, pose_constraint->get_constraint_id());
                    logger().info(
                        "Successfully added stored pose constraint id {} "
                        "between {} and {}",
                        pose_constraint->get_constraint_id(),
                        pose_constraint->timestamp1,
                        pose_constraint->timestamp2);
                } else {
                    logger().error(
                        "Failed to create nodes for stored pose constraint "
                        "between {} and {}",
                        pose_constraint->timestamp1,
                        pose_constraint->timestamp2);
                    return;
                }
            }
        } catch (const std::exception& e) {
            logger().error(
                "Error creating pose to pose constraint between "
                "timestamps {} and {}: {}",
                pose_constraint->timestamp1, pose_constraint->timestamp2,
                e.what());
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
    std::shared_ptr<Node> get_or_create_ptp_node(uint64_t timestamp, int row,
                                                 int col, int return_idx,
                                                 const std::string& node_name) {
        std::shared_ptr<Node> node = traj.get_node_ts(timestamp);

        if (node) {
            if (!ensure_node_has_ptp_point(node, timestamp, row, col,
                                           return_idx)) {
                logger().warn(
                    "ensure_node_has_ptp_point failed for {}; "
                    "attempting to create a dedicated PTP node",
                    node_name);
                auto created =
                    create_node_for_ptp(timestamp, row, col, return_idx);
                if (created) {
                    node = created;
                } else {
                    logger().error(
                        "Failed to ensure or create {} with ptp point data",
                        node_name);
                    return nullptr;
                }
            }
        } else {
            node = create_node_for_ptp(timestamp, row, col, return_idx);
            if (!node) {
                logger().error(
                    "Failed to create {} for point-to-point constraint",
                    node_name);
                return nullptr;
            }
        }

        return node;
    }

    /**
     * @brief Adds a point-to-point constraint between specific 3D points in two
     * scans
     * @param constraint The point-to-point constraint specifying pixel
     * coordinates and return indices
     */
    void add_point_to_point_constraint(const Constraint* constraint) {
        auto pt_constraint =
            static_cast<const PointToPointConstraint*>(constraint);

        try {
            logger().info(
                "Creating point to point constraint between {} and {} ",
                pt_constraint->timestamp1, pt_constraint->timestamp2);

            // Handle node1
            std::shared_ptr<Node> node1 = get_or_create_ptp_node(
                pt_constraint->timestamp1, pt_constraint->row1,
                pt_constraint->col1, pt_constraint->return_idx1, "node1");
            if (!node1) {
                return;
            }

            // Handle node2
            std::shared_ptr<Node> node2 = get_or_create_ptp_node(
                pt_constraint->timestamp2, pt_constraint->row2,
                pt_constraint->col2, pt_constraint->return_idx2, "node2");
            if (!node2) {
                return;
            }

            // Create the constraint
            impl::PointToPointConstraintImpl constraint_impl(
                node1, node2, pt_constraint->translation_weights);
            add_constraint_with_id(constraint_impl,
                                   pt_constraint->get_constraint_id());

            logger().info(
                "Successfully added point to point constraint id {} between "
                "{} and {}",
                pt_constraint->get_constraint_id(), pt_constraint->timestamp1,
                pt_constraint->timestamp2);
        } catch (const std::exception& e) {
            logger().error(
                "Error creating point to point constraint between timestamps "
                "{} and {}: {}",
                pt_constraint->timestamp1, pt_constraint->timestamp2, e.what());
        }
    }

    /**
     * @brief Adds an absolute point constraint to fix a specific 3D point at a
     * global position
     * @param constraint The absolute point constraint specifying pixel
     * coordinates and global position
     */
    void add_absolute_point_constraint(const Constraint* constraint) {
        auto abs_pt_constraint =
            static_cast<const AbsolutePointConstraint*>(constraint);

        try {
            logger().info(
                "Creating absolute point constraint for timestamp {} at "
                "position ({}, {}, {})",
                abs_pt_constraint->timestamp,
                abs_pt_constraint->absolute_position.x(),
                abs_pt_constraint->absolute_position.y(),
                abs_pt_constraint->absolute_position.z());

            std::shared_ptr<Node> node =
                traj.get_node_ts(abs_pt_constraint->timestamp);

            if (node) {
                // Node exists, ensure it has the required point data
                if (!ensure_node_has_absolute_point(
                        node, abs_pt_constraint->timestamp,
                        abs_pt_constraint->row, abs_pt_constraint->col,
                        abs_pt_constraint->return_idx)) {
                    logger().error(
                        "Failed to ensure node has absolute point data");
                    return;
                }
            } else {
                // Node doesn't exist, create new one
                node = create_node_for_absolute_point(
                    abs_pt_constraint->timestamp, abs_pt_constraint->row,
                    abs_pt_constraint->col, abs_pt_constraint->return_idx);
                if (!node) {
                    logger().error(
                        "Failed to create new node for absolute point "
                        "constraint");
                    return;
                }
            }

            // Create the constraint
            impl::AbsolutePointConstraintImpl constraint_impl(
                node, abs_pt_constraint->absolute_position,
                abs_pt_constraint->translation_weights);
            add_constraint_with_id(constraint_impl,
                                   abs_pt_constraint->get_constraint_id());

            release_first_node_anchor();

            logger().info(
                "Successfully processed absolute point constraint id {} for "
                "timestamp {}",
                abs_pt_constraint->get_constraint_id(),
                abs_pt_constraint->timestamp);
        } catch (const std::exception& e) {
            logger().error(
                "Error creating absolute point constraint for timestamp {}: {}",
                abs_pt_constraint->timestamp, e.what());
        }
    }

    // =============================================================================
    // HELPER FUNCTIONS FOR POINT CONSTRAINT MANAGEMENT
    // =============================================================================

    // Helper function to ensure a node has the required point data for absolute
    // point constraints
    bool ensure_node_has_absolute_point(std::shared_ptr<Node>& node,
                                        uint64_t timestamp, int row, int col,
                                        int return_idx) {
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
            if (node->ap_row == row && node->ap_col == col &&
                node->ap_return == return_idx) {
                if (node->downsampled_pts.rows() == 0) {
                    try {
                        auto tmp = create_node_from_point(timestamp, row, col,
                                                          return_idx);
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
                timestamp, node->ap_row, node->ap_col, node->ap_return, row,
                col, return_idx);
            return false;
        }

        try {
            auto temp_node =
                create_node_from_point(timestamp, row, col, return_idx);
            if (temp_node) {
                if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ap_constraint_pt = temp_node->ap_constraint_pt.row(0);
                } else {
                    return false;
                }
                node->ap_row = row;
                node->ap_col = col;
                node->ap_return = return_idx;
                if (temp_node->downsampled_pts.rows() > 0 &&
                    node->downsampled_pts.rows() == 0) {
                    // Populate cloud only if not already set to avoid
                    // inconsistencies across constraints
                    node->downsampled_pts = temp_node->downsampled_pts;
                }
                logger().info(
                    "Added ap_constraint_pt to existing node for timestamp {}",
                    timestamp);
                return true;
            }
        } catch (const std::exception& e) {
            logger().error("Failed to get selected point for existing node: {}",
                           e.what());
        }
        return false;
    }

    // Helper function to ensure a node has the required point data for
    // point-to-point constraints
    bool ensure_node_has_ptp_point(std::shared_ptr<Node>& node,
                                   uint64_t timestamp, int row, int col,
                                   int return_idx) {
        if (!node) {
            return false;
        }

        if (node->ptp_constraint_pt.rows() > 0) {
            if (node->ptp_row < 0 || node->ptp_col < 0 ||
                node->ptp_return < 0) {
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
            if (node->ptp_row == row && node->ptp_col == col &&
                node->ptp_return == return_idx) {
                return true;
            }
            logger().error(
                "Point-to-point selection for timestamp {} already exists with "
                "row={} col={} return={} (requested row={} col={} return={}).",
                timestamp, node->ptp_row, node->ptp_col, node->ptp_return, row,
                col, return_idx);
            return false;
        }

        // Always fetch the requested point so we honor the row/col selection
        try {
            auto temp_node =
                create_node_from_point(timestamp, row, col, return_idx);
            if (temp_node) {
                if (temp_node->ptp_constraint_pt.rows() > 0) {
                    node->ptp_constraint_pt =
                        temp_node->ptp_constraint_pt.row(0);
                } else if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ptp_constraint_pt =
                        temp_node->ap_constraint_pt.row(0);
                } else if (temp_node->downsampled_pts.rows() > 0) {
                    node->ptp_constraint_pt = temp_node->downsampled_pts.row(0);
                } else {
                    return false;
                }

                if (node->ap_constraint_pt.rows() == 0 &&
                    temp_node->ap_constraint_pt.rows() > 0) {
                    node->ap_constraint_pt = temp_node->ap_constraint_pt.row(0);
                }
                if (temp_node->downsampled_pts.rows() > 0 &&
                    node->downsampled_pts.rows() == 0) {
                    node->downsampled_pts = temp_node->downsampled_pts;
                }
                logger().info(
                    "Added ptp_constraint_pt to existing node for timestamp {}",
                    timestamp);
                return true;
            }
        } catch (const std::exception& e) {
            logger().error("Failed to get selected point for existing node: {}",
                           e.what());
        }
        return false;
    }

    // Helper function to create a new node for absolute point constraints
    std::shared_ptr<Node> create_node_for_absolute_point(uint64_t timestamp,
                                                         int row, int col,
                                                         int return_idx) {
        try {
            auto temp_node =
                create_node_from_point(timestamp, row, col, return_idx);
            if (temp_node) {
                auto node = std::make_shared<Node>(temp_node->ts,
                                                   temp_node->get_pose());
                // Prefer explicit AP selected point; fallback to first cloud
                // point
                if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ap_constraint_pt = temp_node->ap_constraint_pt.row(0);
                } else if (temp_node->downsampled_pts.rows() > 0) {
                    node->ap_constraint_pt = temp_node->downsampled_pts.row(0);
                } else {
                    logger().error(
                        "Temp node for ABS point has no selected/cloud points");
                    return nullptr;
                }
                node->ap_row = row;
                node->ap_col = col;
                node->ap_return = return_idx;
                node->ptp_constraint_pt = node->ap_constraint_pt;
                node->ptp_row = row;
                node->ptp_col = col;
                node->ptp_return = return_idx;
                if (temp_node->downsampled_pts.rows() > 0 &&
                    node->downsampled_pts.rows() == 0) {
                    // Only populate if we don't already have it
                    node->downsampled_pts = temp_node->downsampled_pts;
                }

                add_node_to_problem(node);
                add_node_neighbours_constraints(node);
                traj.timestamp_node_map[timestamp] = node;

                logger().info(
                    "Created new node with ap_constraint_pt for timestamp {}",
                    timestamp);
                return node;
            }
        } catch (const std::exception& e) {
            logger().error(
                "Failed to create node from point for timestamp {}: {}",
                timestamp, e.what());
        }
        return nullptr;
    }

    // Helper function to create a new node for point-to-point constraints
    std::shared_ptr<Node> create_node_for_ptp(uint64_t timestamp, int row,
                                              int col, int return_idx) {
        try {
            auto temp_node =
                create_node_from_point(timestamp, row, col, return_idx);
            if (temp_node) {
                auto node = std::make_shared<Node>(temp_node->ts,
                                                   temp_node->get_pose());
                // Prefer explicit ptp_constraint_pt, then ap_constraint_pt,
                // then fallback to first downsampled point
                if (temp_node->ptp_constraint_pt.rows() > 0) {
                    node->ptp_constraint_pt =
                        temp_node->ptp_constraint_pt.row(0);
                } else if (temp_node->ap_constraint_pt.rows() > 0) {
                    node->ptp_constraint_pt =
                        temp_node->ap_constraint_pt.row(0);
                } else if (temp_node->downsampled_pts.rows() > 0) {
                    node->ptp_constraint_pt = temp_node->downsampled_pts.row(0);
                } else {
                    logger().error(
                        "Temp node for PTP selection has no available point");
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

                logger().info(
                    "Created new node with ptp_constraint_pt for timestamp {}",
                    timestamp);
                return node;
            }
        } catch (const std::exception& e) {
            logger().error(
                "Failed to create node from point for timestamp {}: {}",
                timestamp, e.what());
        }
        return nullptr;
    }

    /**
     * @brief Adds a node to the Ceres optimization problem
     * @param node The node to add (handles quaternion parameterization and
     * parameter blocks)
     */
    void add_node_to_problem(std::shared_ptr<Node> node) {
        if (!node) {
            return;
        }
#if CERES_VERSION_MAJOR < 2 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR < 1)
        ceres::LocalParameterization* quaternion_parameterization =
            new ceres::QuaternionParameterization();
#else
        ceres::Manifold* quaternion_parameterization =
            new ceres::EigenQuaternionManifold();
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

    void add_constraint(impl::ConstraintImpl& constraint,
                        bool is_user_constraint = false) {
        ceres::ResidualBlockId residual_id =
            constraint.add_to_problem(problem, loss_function);

        // Track user constraints for selective removal
        if (is_user_constraint) {
            user_constraint_residual_blocks.push_back(residual_id);
        }
    }

    // Overloaded version for adding constraints with ID tracking
    void add_constraint_with_id(impl::ConstraintImpl& constraint,
                                uint32_t constraint_id) {
        ceres::ResidualBlockId residual_id =
            constraint.add_to_problem(problem, loss_function);

        // Track user constraints for selective removal
        user_constraint_residual_blocks.push_back(residual_id);

        // Map constraint ID to residual ID
        constraint_id_to_residual_map[constraint_id] = residual_id;
    }

    // Create constraint implementation from Constraint and add nodes with
    // neighbors if needed
    uint32_t add_base_constraint(Constraint* base_constraint) {
        if (!base_constraint) {
            throw std::invalid_argument("Cannot add null constraint");
        }

        // Checks if an ID is already taken by an existing user constraint.
        // We look at both the active residual map and stored config
        // constraints (id 0 is reserved for internal/trajectory constraints).
        auto constraint_id_in_use = [&](uint32_t id) {
            if (id == 0) {
                return false;
            }
            if (constraint_id_to_residual_map.find(id) !=
                constraint_id_to_residual_map.end()) {
                return true;
            }
            for (const auto& constraint : config.constraints) {
                if (!constraint || constraint.get() == base_constraint) {
                    continue;
                }
                if (constraint->get_constraint_id() == id) {
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
                add_absolute_pose_constraint(base_constraint);
                break;
            case ConstraintType::POSE_TO_POSE:
                add_pose_to_pose_constraint(base_constraint);
                break;
            case ConstraintType::POINT_TO_POINT:
                add_point_to_point_constraint(base_constraint);
                break;
            case ConstraintType::ABSOLUTE_POINT:
                add_absolute_point_constraint(base_constraint);
                break;
            default:
                logger().error("Unknown constraint type: {}",
                               static_cast<int>(base_constraint->get_type()));
                break;
        }

        return constraint_id;
    }

    void add_node_neighbours_constraints(std::shared_ptr<Node> node) {
        auto node_iter = traj.timestamp_node_map.upper_bound(node->ts);
        if (node_iter == traj.timestamp_node_map.end()) {
            logger().error("Error : Can't create a node for timestamp {}",
                           node->ts);
            return;
        } else {
            // Add pose to pose constraint of the new pose and the next node in
            // traj
            PoseH diff = PoseH(node->get_pose()).inverse() *
                         PoseH((node_iter->second)->get_pose());
            ouster::sdk::core::impl::PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraintImpl constraint(
                node, node_iter->second, diff_q.r(), diff_q.t(),
                config.traj_rotation_weight,
                {config.traj_translation_weight, config.traj_translation_weight,
                 config.traj_translation_weight});

            add_constraint(constraint);
        }
        if (std::distance(traj.timestamp_node_map.begin(), node_iter) > 2) {
            auto node_prev = *(std::prev(node_iter, 2));

            PoseH diff = PoseH(node_prev.second->get_pose()).inverse() *
                         PoseH(node->get_pose());
            ouster::sdk::core::impl::PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraintImpl constraint(
                node_prev.second, node, diff_q.r(), diff_q.t(),
                config.traj_rotation_weight,
                {config.traj_translation_weight, config.traj_translation_weight,
                 config.traj_translation_weight});

            add_constraint(constraint);
        }
    }

    std::shared_ptr<Node> get_or_create_node_ts(
        uint64_t timestamp, bool generate_point_cloud = false) {
        std::shared_ptr<Node> node = traj.get_node_ts(timestamp);
        if (node) {
            if (generate_point_cloud && node->downsampled_pts.rows() == 0) {
                auto new_node = traj.create_node_ts(
                    timestamp, generate_point_cloud, downsample_voxel_size);
                if (new_node) {
                    if (new_node->downsampled_pts.rows() > 0) {
                        new_node->downsampled_pts = run_kiss_icp_downsample(
                            new_node->downsampled_pts, downsample_voxel_size);
                    }
                    node->downsampled_pts = new_node->downsampled_pts;
                }
            }
            return node;
        }

        node = traj.create_node_ts(timestamp, generate_point_cloud,
                                   downsample_voxel_size);
        if (node) {
            if (generate_point_cloud && node->downsampled_pts.rows() > 0) {
                node->downsampled_pts = run_kiss_icp_downsample(
                    node->downsampled_pts, downsample_voxel_size);
            }
            add_node_to_problem(node);
            add_node_neighbours_constraints(node);
        } else {
            std::string msg = "Failed to create the node from timestamp " +
                              std::to_string(timestamp) + ".";
            if (generate_point_cloud) {
                msg +=
                    " The timestamp may be invalid or not correspond to a "
                    "scan's first valid column.";
            }

            if (!traj.all_timestamps.empty()) {
                uint64_t min_ts = traj.all_timestamps.front();
                uint64_t max_ts = traj.all_timestamps.back();

                if (timestamp < min_ts || timestamp > max_ts) {
                    msg +=
                        " The timestamp may be outside the range of the OSF "
                        "file "
                        "[" +
                        std::to_string(min_ts) + ", " + std::to_string(max_ts) +
                        "].";
                }
            }

            throw std::runtime_error(msg);
        }

        return node;
    }

    void add_traj_constraint(bool fix_first_node) {
        auto nodes = traj.timestamp_node_map;

        if (nodes.size() < 2) {
            logger().error(
                "No constraints to add if there are fewer than 2 nodes");
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
            if (constraint &&
                (constraint->get_type() == ConstraintType::ABSOLUTE_POSE ||
                 constraint->get_type() == ConstraintType::ABSOLUTE_POINT)) {
                has_absolute_constraints = true;
                break;
            }
        }

        // Fix first node if explicitly requested OR there are no absolute
        // constraints
        bool should_fix_first_node =
            fix_first_node || (!has_absolute_constraints);

        if (should_fix_first_node) {
            const auto& first_node = it->second;
            problem.SetParameterBlockConstant(
                first_node->rotation.coeffs().data());
            problem.SetParameterBlockConstant(first_node->position.data());
            logger().info("Fixed first node as trajectory anchor");
            this->fix_first_node = true;
        } else {
            this->fix_first_node = false;
        }

        for (; it_next != nodes.end(); ++it, ++it_next) {
            const auto& node_before = it->second;
            const auto& node_after = it_next->second;

            PoseH diff = PoseH(node_before->get_pose()).inverse() *
                         PoseH(node_after->get_pose());
            PoseQ diff_q = diff.log().q();

            impl::PoseToPoseConstraintImpl constraint(
                node_before, node_after, diff_q.r(), diff_q.t(),
                config.traj_rotation_weight,
                {config.traj_translation_weight, config.traj_translation_weight,
                 config.traj_translation_weight});

            add_constraint(constraint);
        }
    }

    std::shared_ptr<Node> create_node_from_point(uint64_t timestamp,
                                                 uint32_t row, uint32_t col,
                                                 uint32_t return_idx) {
        if (return_idx != 1 && return_idx != 2) {
            throw std::invalid_argument(
                "Fail to create a Node. return_idx can only be 1 or 2 but "
                "received " +
                std::to_string(return_idx));
        }

        const std::string field_name = return_idx == 1 ? "RANGE" : "RANGE2";
        const auto chan_field = return_idx == 1
                                    ? ouster::sdk::core::ChanField::RANGE
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
            logger().error("OSF source is empty; cannot create node for ts {}",
                           timestamp);
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
                if (ls && ls_ts == timestamp) {
                    // Validate row/col bounds against sensor metadata
                    const uint32_t H = static_cast<uint32_t>(
                        traj.info.format.pixels_per_column);
                    const uint32_t W = static_cast<uint32_t>(ls->w);
                    if (row >= H || col >= W) {
                        logger().error(
                            "Selected row/col out of bounds: row={} col={} "
                            "(H={} W={})",
                            row, col, H, W);
                        return nullptr;
                    }
                    // Map destaggered (row, col) to raw column index used by
                    // LidarScan and cartesian().
                    const int W_i = static_cast<int>(ls->w);
                    const int shift =
                        ((traj.info.format.pixel_shift_by_row[row] % W_i) +
                         W_i) %
                        W_i;
                    const int col_raw_i =
                        (static_cast<int>(col) + W_i - shift) % W_i;
                    const uint32_t col_raw = static_cast<uint32_t>(col_raw_i);

                    uint64_t col_ts = ls->timestamp()[col_raw];
                    Matrix4dR mat = ls->get_column_pose(col_raw);

                    // Use the selected return's range for cloud generation and
                    // selection (consistent with trajectory's LidarScan-based
                    // cartesian)
                    const auto range = ls->field<uint32_t>(chan_field);

                    // Staggered range validity check will follow below
                    // Build cloud like trajectory path: cartesian on
                    // LidarScan using the selected return's range for
                    // consistency across constraints
                    Eigen::Array<double, Eigen::Dynamic, 3> cloud_pts =
                        cartesian(range, traj.xyz_lut);

                    const int first_col = ls->get_first_valid_column();
                    const PoseH first_pose(ls->get_column_pose(first_col));
                    const PoseH first_pose_inv(first_pose.inverse());

                    const int rows = static_cast<int>(ls->h);
                    const int cols = static_cast<int>(ls->w);
                    for (int c = 0; c < cols; ++c) {
                        const PoseH pose_c(
                            ls->get_column_pose(static_cast<uint32_t>(c)));
                        const PoseH rel(first_pose_inv * pose_c);
                        for (int r = 0; r < rows; ++r) {
                            const Eigen::Index idx =
                                static_cast<Eigen::Index>(r) * cols + c;
                            if (idx >= cloud_pts.rows()) {
                                break;
                            }
                            Eigen::Vector3d pt =
                                cloud_pts.row(idx).matrix().transpose();
                            if (pt.isZero(0.0)) {
                                continue;
                            }
                            pt = rel * pt;
                            cloud_pts.row(idx) = pt.transpose().array();
                        }
                    }

                    int key_pts_index = row * ls->w + col_raw;
                    // Validate the selected pixel has non-zero range
                    // (staggered). If zero, treat as error and stop.
                    if (range(row, col_raw) == 0u) {
                        logger().error(
                            "Selected point has zero/invalid range at row={} "
                            "col={} ({})",
                            row, col, field_name);
                        return nullptr;
                    }
                    // Selected 3D point from the same cloud mapping
                    Eigen::Array<double, 1, 3> key_pts =
                        cloud_pts.row(key_pts_index);

                    auto node =
                        std::make_shared<Node>(col_ts, Eigen::Matrix4d(mat));
                    // Set both AP and PTP selected point so either constraint
                    // can use it
                    node->ap_constraint_pt = key_pts;
                    node->ap_row = row;
                    node->ap_col = col;
                    node->ap_return = return_idx;
                    node->ptp_constraint_pt = key_pts;
                    node->ptp_row = row;
                    node->ptp_col = col;
                    node->ptp_return = return_idx;
                    // Attach cloud consistent with trajectory (RANGE-based)
                    if (cloud_pts.rows() > 0) {
                        node->downsampled_pts = run_kiss_icp_downsample(
                            cloud_pts, downsample_voxel_size);
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
            logger().error("Constraint ID {} not found in residual map",
                           constraint_id);
            return false;
        }

        // Remove from config.constraints
        auto it = std::find_if(
            config.constraints.begin(), config.constraints.end(),
            [constraint_id](const std::unique_ptr<Constraint>& constraint) {
                return constraint &&
                       constraint->get_constraint_id() == constraint_id;
            });
        if (it != config.constraints.end()) {
            config.constraints.erase(it);
        } else {
            logger().warn("Constraint ID {} not found in config.constraints",
                          constraint_id);
        }

        return true;
    }

    std::vector<std::shared_ptr<Node>> get_sampled_nodes(size_t count) {
        std::vector<std::shared_ptr<Node>> result;
        const size_t total_scans = traj.timestamps_index_vec.size();
        if (count == 0 || total_scans == 0) {
            return result;
        }

        const size_t samples = std::min(count, total_scans);
        result.reserve(samples);

        auto append_node = [&](size_t idx) {
            if (idx >= total_scans) {
                return;
            }
            const auto& info = traj.timestamps_index_vec[idx];
            const uint64_t ts = info.first_col_ts;
            try {
                auto node = get_or_create_node_ts(ts, true);
                if (!node) {
                    return;
                }
                if (node->downsampled_pts.rows() == 0) {
                    logger().warn(
                        "get_sampled_nodes: node {} has no downsampled points",
                        ts);
                    return;
                }
                result.push_back(std::move(node));
            } catch (const std::exception& e) {
                logger().warn(
                    "get_sampled_nodes: failed to prepare node for ts {}: {}",
                    ts, e.what());
            }
        };

        if (samples == total_scans) {
            for (size_t idx = 0; idx < total_scans; ++idx) {
                append_node(idx);
            }
            return result;
        }

        if (samples == 1) {
            append_node(0);
            return result;
        }

        const double stride = static_cast<double>(total_scans - 1) /
                              static_cast<double>(samples - 1);

        size_t previous_idx = 0;
        bool have_previous = false;
        for (size_t i = 0; i < samples; ++i) {
            const double position = stride * static_cast<double>(i);
            size_t idx = static_cast<size_t>(std::round(position));
            if (have_previous && idx <= previous_idx) {
                idx = previous_idx + 1;
            }
            if (idx >= total_scans) {
                idx = total_scans - 1;
            }
            append_node(idx);
            previous_idx = idx;
            have_previous = true;
            if (previous_idx == total_scans - 1) {
                break;
            }
        }

        return result;
    }
};

PoseOptimizer::PoseOptimizer(const std::string& osf_filename,
                             const SolverConfig& config)
    : pimpl_(std::make_unique<Impl>(config, expand_home_path(osf_filename))) {
    pimpl_->add_traj_constraint(config.fix_first_node);
}

PoseOptimizer::PoseOptimizer(const std::string& osf_filename,
                             double key_frame_distance) {
    SolverConfig config;
    config.key_frame_distance = key_frame_distance;
    pimpl_ = std::make_unique<PoseOptimizer::Impl>(
        config, expand_home_path(osf_filename));
    pimpl_->add_traj_constraint(config.fix_first_node);
}

PoseOptimizer::PoseOptimizer(const std::string& osf_filename,
                             const std::string& config_filename) {
    SolverConfig config;
    ouster::sdk::core::ValidatorIssues issues;

    std::ifstream config_stream(config_filename);
    if (!config_stream.is_open()) {
        throw std::runtime_error("Could not open config file: " +
                                 config_filename);
    }
    std::string json_data((std::istreambuf_iterator<char>(config_stream)),
                          std::istreambuf_iterator<char>());

    bool ok =
        mapping::parse_and_validate_constraints(json_data, config, issues);
    if (!ok) {
        throw std::runtime_error("Error parsing config file: " +
                                 issues.to_string());
    }
    // Move parsed config into Impl
    pimpl_ = std::make_unique<PoseOptimizer::Impl>(
        config, expand_home_path(osf_filename));
    pimpl_->add_traj_constraint(config.fix_first_node);
}

PoseOptimizer::~PoseOptimizer() = default;

bool PoseOptimizer::initialize_trajectory_alignment() {
    if (!pimpl_) {
        return false;
    }
    return pimpl_->initialize_trajectory_alignment();
}

double PoseOptimizer::solve(uint32_t steps) {
    if (steps > 0) {
        pimpl_->options.max_num_iterations = steps;
        logger().info("Incremental optimize with {} iterations.", steps);
    } else {
        logger().info(
            "Running full optimization with default max iterations ({}).",
            pimpl_->options.max_num_iterations);
        pimpl_->options.max_num_iterations = pimpl_->config.max_num_iterations;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(pimpl_->options, &(pimpl_->problem), &summary);

    logger().info("Initial Cost: {}", summary.initial_cost);
    logger().info("Final   Cost: {}", summary.final_cost);
    pimpl_->cost_number_ = summary.final_cost;
    pimpl_->total_iterations_ +=
        static_cast<uint64_t>(summary.iterations.size());
    return summary.final_cost;
}

void PoseOptimizer::save(const std::string& osf_filename) {
    logger().info("Saving the results into {}", osf_filename);
    pimpl_->traj.save(osf_filename);
}

std::shared_ptr<Node> PoseOptimizer::get_node(uint64_t timestamp) const {
    auto node = pimpl_->traj.get_node_ts(timestamp);
    if (node) {
        // Ensure pose_ reflects current rotation/position
        node->update_pose();
    }
    return node;
}

double PoseOptimizer::get_cost_value() const { return pimpl_->cost_number_; }

uint64_t PoseOptimizer::get_total_iterations() const {
    if (!pimpl_) {
        return 0;
    }
    return pimpl_->total_iterations_;
}

std::vector<std::shared_ptr<Node>> PoseOptimizer::get_sampled_nodes(
    size_t count) const {
    if (!pimpl_) {
        return {};
    }
    return pimpl_->get_sampled_nodes(count);
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

double PoseOptimizer::get_key_frame_distance() const {
    return pimpl_->config.key_frame_distance;
}

std::vector<std::unique_ptr<Constraint>> PoseOptimizer::get_constraints()
    const {
    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.reserve(pimpl_->config.constraints.size());

    for (const auto& constraint : pimpl_->config.constraints) {
        if (constraint) {
            constraints.push_back(constraint->clone());
        }
    }

    return constraints;
}

void PoseOptimizer::set_constraints(
    std::vector<std::unique_ptr<Constraint>> constraints) {
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
        logger().info("Clearing {} constraint(s) from PoseOptimizer",
                      constraint_count);
    }

    // First clear constraints from the Ceres problem so any residuals that
    // reference constraint implementations are removed while those
    // implementations are still valid.
    pimpl_->clear_user_constraints_from_problem();

    // Then clear constraints from the stored config (release ownership).
    pimpl_->config.constraints.clear();
}

void save_trajectory(const std::string& filename,
                     const std::vector<uint64_t>& timestamps,
                     const std::vector<Eigen::Matrix<double, 4, 4>>& poses,
                     const std::string& file_type) {
    if (timestamps.size() != poses.size()) {
        logger().error("Timestamps and poses size mismatch: {} vs {}",
                       timestamps.size(), poses.size());
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

            file << timestamp << ',' << translation.x() << ','
                 << translation.y() << ',' << translation.z() << ','
                 << quaternion.x() << ',' << quaternion.y() << ','
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

            file << timestamp << ' ' << translation.x() << ' '
                 << translation.y() << ' ' << translation.z() << ' '
                 << quaternion.x() << ' ' << quaternion.y() << ' '
                 << quaternion.z() << ' ' << quaternion.w() << '\n';
        }

    } else {
        logger().error(
            "Unsupported file type: {}. Currently support 'csv' or 'tum'.",
            file_type);
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
        const uint32_t constraint_id =
            pimpl_->add_base_constraint(constraint_ptr);

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
        throw std::invalid_argument(
            "Cannot remove constraint with ID 0 (not a user constraint)");
    }

    try {
        bool removed = pimpl_->remove_constraint_by_id(constraint_id);
        if (removed) {
            logger().info("Successfully removed constraint with ID {}",
                          constraint_id);
        } else {
            throw std::runtime_error("Failed to remove constraint with ID " +
                                     std::to_string(constraint_id) +
                                     " (not found)");
        }
    } catch (const std::exception& e) {
        logger().error("Exception while removing constraint with ID {}: {}",
                       constraint_id, e.what());
        throw;
    }
}

void PoseOptimizer::save_config(const std::string& config_filename) {
    try {
        std::string json_string = serialize_constraints_to_json(pimpl_->config);

        std::ofstream outfile(config_filename);
        if (!outfile.is_open()) {
            throw std::runtime_error("Could not open file for writing: " +
                                     config_filename);
        }

        outfile << json_string;
        outfile.close();

        if (outfile.fail()) {
            throw std::runtime_error("Failed to write constraints to file: " +
                                     config_filename);
        }

        logger().info("Successfully saved {} constraint(s) to: {}",
                      pimpl_->config.constraints.size(), config_filename);

    } catch (const std::exception& e) {
        logger().error("Error saving constraints to file {}: {}",
                       config_filename, e.what());
        throw;
    }
}

void PoseOptimizer::set_solver_step_callback(std::function<void()> fn) {
    pimpl_->solver_step_functor_ = std::move(fn);
    if (!pimpl_->step_cb_registered_) {
        pimpl_->step_cb_holder_ =
            std::make_unique<PoseOptimizer::Impl::StepCallback>(pimpl_.get());
        pimpl_->options.callbacks.push_back(pimpl_->step_cb_holder_.get());
        pimpl_->options.update_state_every_iteration = true;
        pimpl_->step_cb_registered_ = true;
        logger().info(
            "Registered solver step callback with PoseOptimizer solver");
    }
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

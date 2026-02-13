#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

class PoseOptimizer;

/**
 * @brief Sampling Mode for PoseOptimize get poses or timestamps.
 */
enum class SamplingMode {
    KEY_FRAMES,  // Key frames sampling mode, selects poses or timestamps of
                 // every key frame in Pose Optimize

    COLUMNS  // Selects poses or timestamps corresponding to every column in
             // each lidarscan.
};

/**
 * @brief Loss function types used in PoseOptimizer.
 */
enum class LossFunction {
    HUBER_LOSS,
    CAUCHY_LOSS,
    SOFT_L_ONE_LOSS,
    ARCTAN_LOSS,
    TRIVIAL_LOSS
};

/**
 * @brief Converts a string to the corresponding LossFunction enum value.
 *
 * This function takes a string representation of a loss function name
 * and returns the matching LossFunction enum.
 *
 * @param[in] name The string name of the loss function.
 * @return The corresponding LossFunction enum value.
 * @throws std::invalid_argument If the provided name does not match any known
 * LossFunction.
 */
OUSTER_API_FUNCTION
inline LossFunction loss_function_from_string(const std::string& name) {
    if (name == "HUBER_LOSS") {
        return LossFunction::HUBER_LOSS;
    } else if (name == "CAUCHY_LOSS") {
        return LossFunction::CAUCHY_LOSS;
    } else if (name == "SOFT_L_ONE_LOSS") {
        return LossFunction::SOFT_L_ONE_LOSS;
    } else if (name == "ARCTAN_LOSS") {
        return LossFunction::ARCTAN_LOSS;
    } else if (name == "TRIVIAL_LOSS") {
        return LossFunction::TRIVIAL_LOSS;
    }
    throw std::invalid_argument("Unknown LossFunction: " + name);
}

/**
 * @brief Converts a LossFunction enum value to its corresponding string.
 *
 * This function takes a LossFunction enum and returns a string representation
 * of that enum. Valid return values are: "HUBER_LOSS", "CAUCHY_LOSS",
 * "SOFT_L_ONE_LOSS", "ARCTAN_LOSS", and "TRIVIAL_LOSS".
 *
 * @param[in] lf The LossFunction enum to convert.
 * @return A std::string containing the name of the loss function.
 */
OUSTER_API_FUNCTION
inline std::string to_string(const LossFunction lf) {
    switch (lf) {
        case LossFunction::HUBER_LOSS:
            return "HUBER_LOSS";
        case LossFunction::CAUCHY_LOSS:
            return "CAUCHY_LOSS";
        case LossFunction::SOFT_L_ONE_LOSS:
            return "SOFT_L_ONE_LOSS";
        case LossFunction::ARCTAN_LOSS:
            return "ARCTAN_LOSS";
        case LossFunction::TRIVIAL_LOSS:
            return "TRIVIAL_LOSS";
        default:
            // If you ever extend the enum and forget to update this,
            // throwing an exception helps catch it at runtime.
            throw std::invalid_argument("Unknown LossFunction enum value");
    }
}

/**
 * @brief Types of constraints supported by the pose optimizer.
 */
enum class ConstraintType {
    ABSOLUTE_POSE,   // Absolute pose constraint: fixes a pose at specific
                     // timestamp
    ABSOLUTE_POINT,  // Absolute point constraint: constrains a 3D lidar point
                     // (selected by 2D image indices row/col/return at a
                     // timestamp) to a user-defined global 3D position
    POSE_TO_POSE,    // Pose-to-pose constraint: relative transform between two
                     // poses
    POINT_TO_POINT,  // Point-to-point constraint: correspondence between
                     // points
};

/**
 * @brief Converts a string to the corresponding ConstraintType enum value.
 *
 * @param[in] name The string name of the constraint type.
 * @return The corresponding ConstraintType enum value.
 * @throws std::invalid_argument If the provided name does not match any known
 * ConstraintType.
 */
OUSTER_API_FUNCTION
inline ConstraintType constraint_type_from_string(const std::string& name) {
    if (name == "ABSOLUTE_POSE" || name == "absolute_pose") {
        return ConstraintType::ABSOLUTE_POSE;
    } else if (name == "POSE_TO_POSE" || name == "pose_to_pose") {
        return ConstraintType::POSE_TO_POSE;
    } else if (name == "POINT_TO_POINT" || name == "point_to_point") {
        return ConstraintType::POINT_TO_POINT;
    } else if (name == "ABSOLUTE_POINT" || name == "absolute_point") {
        return ConstraintType::ABSOLUTE_POINT;
    }
    throw std::invalid_argument("Unknown ConstraintType: " + name);
}

/**
 * @brief Converts a ConstraintType enum value to its corresponding string.
 *
 * @param[in] ct The ConstraintType enum to convert.
 * @return A std::string containing the name of the constraint type.
 */
OUSTER_API_FUNCTION
inline std::string constraint_type_to_string(const ConstraintType ct) {
    switch (ct) {
        case ConstraintType::ABSOLUTE_POSE:
            return "ABSOLUTE_POSE";
        case ConstraintType::POSE_TO_POSE:
            return "POSE_TO_POSE";
        case ConstraintType::POINT_TO_POINT:
            return "POINT_TO_POINT";
        case ConstraintType::ABSOLUTE_POINT:
            return "ABSOLUTE_POINT";
        default:
            throw std::invalid_argument("Unknown ConstraintType enum value");
    }
}

/**
 * @brief Base class for all pose optimization constraints.
 */
class OUSTER_API_CLASS Constraint {
   public:
    /**
     * @brief Default Constructor
     */
    OUSTER_API_FUNCTION
    Constraint()
        : constraint_id(
              next_constraint_id_.fetch_add(1, std::memory_order_relaxed)) {}

    /** Destructor */
    OUSTER_API_FUNCTION
    virtual ~Constraint() = default;

    /**
     * @brief Construct a Constraint with custom translation weights.
     *
     * @param[in] translation_weights Weights to apply to x,y,z translation.
     */
    OUSTER_API_FUNCTION
    Constraint(const Eigen::Array3d& translation_weights)
        : translation_weights(translation_weights),
          constraint_id(
              next_constraint_id_.fetch_add(1, std::memory_order_relaxed)) {}

    /**
     * @brief Copy constructor
     *
     * @param[in] other The constraint to copy from.
     */
    OUSTER_API_FUNCTION
    Constraint(const Constraint& other)
        : translation_weights(other.translation_weights),
          constraint_id(other.constraint_id) {}

    /**
     * @brief Get the unique constraint ID.
     *
     * Returns 0 for non-user (internal) constraints. IDs > 0 are assigned when
     * constraint objects are constructed and are immutable once set internally.
     *
     * @return uint32_t The constraint identifier.
     */
    OUSTER_API_FUNCTION
    uint32_t get_constraint_id() const { return constraint_id; }

    /**
     * @brief Get the type of this constraint.
     *
     * @return ConstraintType The constraint category enum.
     */
    OUSTER_API_FUNCTION
    virtual ConstraintType get_type() const = 0;

    /**
     * @brief Clone this constraint (deep copy).
     *
     * Implementations must preserve constraint metadata such as the
     * constraint id when cloning.
     *
     * @return std::unique_ptr<Constraint> A deep-copied instance.
     */
    OUSTER_API_FUNCTION
    virtual std::unique_ptr<Constraint> clone() const = 0;

    /** @brief Translation weights for constraint optimization. */
    Eigen::Array3d translation_weights = Eigen::Array3d::Ones();

   private:
    friend class PoseOptimizer;
    static std::atomic<uint32_t> next_constraint_id_;

    /**
     * Internal constraint identifier.
     * 0 indicates an internal/trajectory constraint; >0 indicates a user-added
     * constraint assigned at construction time.
     */
    uint32_t constraint_id = 0;
};

/**
 * @brief Absolute pose constraint
 */
struct OUSTER_API_CLASS AbsolutePoseConstraint : public Constraint {
    /** Timestamp of the pose to constrain (nanoseconds). */
    uint64_t timestamp;

    /** The 4x4 transformation matrix (SE3) to constrain to. */
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

    /** Weight for the rotational component of the constraint. */
    double rotation_weight = 1.0;

    /** Default constructor. */
    OUSTER_API_FUNCTION
    AbsolutePoseConstraint() : Constraint() {}

    /**
     * @brief Construct an AbsolutePoseConstraint.
     *
     * @param[in] timestamp Timestamp of the pose to constrain (nanoseconds).
     * @param[in] pose 4x4 homogeneous transform representing the pose.
     * @param[in] rotation_weight Optional rotation weight applied to the
     * rotational residual.
     * @param[in] translation_weights Optional translation weights (x,y,z).
     */
    OUSTER_API_FUNCTION
    AbsolutePoseConstraint(
        uint64_t timestamp, const Eigen::Matrix4d& pose,
        double rotation_weight = 1.0,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones())
        : Constraint(translation_weights),
          timestamp(timestamp),
          pose(pose),
          rotation_weight(rotation_weight) {}

    /** @brief Get the constraint type.
     *  @return ConstraintType The constraint category. */
    OUSTER_API_FUNCTION
    ConstraintType get_type() const override {
        return ConstraintType::ABSOLUTE_POSE;
    }

    /**
     * @brief Clone this AbsolutePoseConstraint.
     *
     * The clone preserves the constraint id.
     *
     * @return std::unique_ptr<Constraint> A deep copy of this constraint.
     */
    OUSTER_API_FUNCTION
    std::unique_ptr<Constraint> clone() const override {
        return std::make_unique<AbsolutePoseConstraint>(*this);
    }
};

/**
 * @brief Pose-to-pose constraint - enforces relative transform between
 * two poses.
 */
struct OUSTER_API_CLASS PoseToPoseConstraint : public Constraint {
    /** Timestamp of the first pose (nanoseconds). */
    uint64_t timestamp1;

    /** Timestamp of the second pose (nanoseconds). */
    uint64_t timestamp2;
    /** Expected relative transform from pose1 to pose2 (4x4 SE3 matrix). */
    Eigen::Matrix4d relative_pose = Eigen::Matrix4d::Identity();

    /** Weight for the rotational component of the relative constraint. */
    double rotation_weight = 1.0;

    /** Default constructor. */
    OUSTER_API_FUNCTION
    PoseToPoseConstraint() : Constraint() {}

    /**
     * @brief Construct a relative pose-to-pose constraint.
     *
     * @param[in] timestamp1 Timestamp of the first pose (nanoseconds).
     * @param[in] timestamp2 Timestamp of the second pose (nanoseconds).
     * @param[in] relative_pose Expected relative transform from pose1 to pose2.
     * Use identity to let the optimizer auto-estimate it via ICP.
     * @param[in] rotation_weight Optional rotation weight.
     * @param[in] translation_weights Optional translation weights.
     */
    OUSTER_API_FUNCTION
    PoseToPoseConstraint(
        uint64_t timestamp1, uint64_t timestamp2,
        const Eigen::Matrix4d& relative_pose = Eigen::Matrix4d::Identity(),
        double rotation_weight = 1.0,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones())
        : Constraint(translation_weights),
          timestamp1(timestamp1),
          timestamp2(timestamp2),
          relative_pose(relative_pose),
          rotation_weight(rotation_weight) {}

    /** @brief Get the constraint type.
     *  @return ConstraintType The constraint category. */
    OUSTER_API_FUNCTION
    ConstraintType get_type() const override {
        return ConstraintType::POSE_TO_POSE;
    }

    /**
     * @brief Clone this PoseToPoseConstraint, preserving its constraint id.
     *
     * @return std::unique_ptr<Constraint> Deep copy of this constraint.
     */
    OUSTER_API_FUNCTION
    std::unique_ptr<Constraint> clone() const override {
        return std::make_unique<PoseToPoseConstraint>(*this);
    }
};

/**
 * @brief Point-to-point constraint - enforces correspondence between points.
 */
struct OUSTER_API_CLASS PointToPointConstraint : public Constraint {
    /** Timestamp of the first point's pose (nanoseconds). */
    uint64_t timestamp1;

    /** Timestamp of the second point's pose (nanoseconds). */
    uint64_t timestamp2;

    /** Row index of the first point. */
    uint32_t row1;

    /** Column index of the first point. */
    uint32_t col1;

    /** Return index of the first point (1 or 2). */
    uint32_t return_idx1;

    /** Row index of the second point. */
    uint32_t row2;

    /** Column index of the second point. */
    uint32_t col2;

    /** Return index of the second point (1 or 2). */
    uint32_t return_idx2;

    /**
     * @brief Construct a point-to-point correspondence constraint.
     *
     * @param[in] timestamp1 Timestamp of the first point's pose (nanoseconds).
     * @param[in] row1 Row index of the first point.
     * @param[in] col1 Column index of the first point.
     * @param[in] return_idx1 Return index of the first point.
     * @param[in] timestamp2 Timestamp of the second point's pose (nanoseconds).
     * @param[in] row2 Row index of the second point.
     * @param[in] col2 Column index of the second point.
     * @param[in] return_idx2 Return index of the second point.
     * @param[in] translation_weights Optional translation weights.
     */
    OUSTER_API_FUNCTION
    PointToPointConstraint(
        uint64_t timestamp1, uint32_t row1, uint32_t col1, uint32_t return_idx1,
        uint64_t timestamp2, uint32_t row2, uint32_t col2, uint32_t return_idx2,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones())
        : Constraint(translation_weights),
          timestamp1(timestamp1),
          timestamp2(timestamp2),
          row1(row1),
          col1(col1),
          return_idx1(return_idx1),
          row2(row2),
          col2(col2),
          return_idx2(return_idx2) {}

    /** Default constructor. */
    OUSTER_API_FUNCTION
    PointToPointConstraint() : Constraint() {}

    /** @brief Get the constraint type.
     *  @return ConstraintType The constraint category. */
    OUSTER_API_FUNCTION
    ConstraintType get_type() const override {
        return ConstraintType::POINT_TO_POINT;
    }

    /**
     * @brief Clone this PointToPointConstraint, preserving its constraint id.
     *
     * @return std::unique_ptr<Constraint> Deep copy of this constraint.
     */
    OUSTER_API_FUNCTION
    std::unique_ptr<Constraint> clone() const override {
        return std::make_unique<PointToPointConstraint>(*this);
    }
};

/**
 * @brief Absolute point constraint.
 *
 * Constrains a single 3D point from a lidar scan, identified by its 2D image
 * coordinates (row, col) and return index at a given timestamp, to match a
 * user-defined absolute 3D position in the world frame.
 */
struct OUSTER_API_CLASS AbsolutePointConstraint : public Constraint {
    /** Timestamp of the point's pose (nanoseconds). */
    uint64_t timestamp;

    /** Row index of the point in the 2D image. */
    uint32_t row;

    /** Column index of the point in the 2D image. */
    uint32_t col;

    /** Return index of the point (1 or 2). */
    uint32_t return_idx;

    /** Absolute position (x, y, z) in world coordinates to match. */
    Eigen::Vector3d absolute_position;

    /**
     * @brief Construct an absolute point constraint.
     *
     * @param[in] timestamp Timestamp of the point's pose (nanoseconds).
     * @param[in] row Row index of the 2D image point.
     * @param[in] col Column index of the 2D image point.
     * @param[in] return_idx Return index of the point (1 or 2).
     * @param[in] absolute_position Target global 3D position (x, y, z).
     * @param[in] translation_weights Optional translation weights.
     */
    OUSTER_API_FUNCTION
    AbsolutePointConstraint(
        uint64_t timestamp, uint32_t row, uint32_t col, uint32_t return_idx,
        const Eigen::Vector3d& absolute_position,
        const Eigen::Array3d& translation_weights = Eigen::Array3d::Ones())
        : Constraint(translation_weights),
          timestamp(timestamp),
          row(row),
          col(col),
          return_idx(return_idx),
          absolute_position(absolute_position) {}

    /** Default constructor. */
    OUSTER_API_FUNCTION
    AbsolutePointConstraint() : Constraint() {}

    /** @brief Get the constraint type.
     *  @return ConstraintType The constraint category. */
    OUSTER_API_FUNCTION
    ConstraintType get_type() const override {
        return ConstraintType::ABSOLUTE_POINT;
    }

    /**
     * @brief Clone this AbsolutePointConstraint, preserving its constraint id.
     *
     * @return std::unique_ptr<Constraint> Deep copy of this constraint.
     */
    OUSTER_API_FUNCTION
    std::unique_ptr<Constraint> clone() const override {
        return std::make_unique<AbsolutePointConstraint>(*this);
    }
};

/**
 * @brief Configuration options for the pose optimizer.
 *
 * Contains parameters that control solver behavior and a list of
 * constraints applied during optimization.
 */
struct OUSTER_API_CLASS SolverConfig {
    /** The distance between nodes in the trajectory (in meters). */
    double key_frame_distance = 1.0;

    /** The weight for rotational constraints during trajectory optimization. */
    double traj_rotation_weight = 10.0;

    /** The weight for translational constraints during trajectory optimization.
     */
    double traj_translation_weight = 10.0;

    /** The maximum number of iterations the solver will perform before
     * terminating. */
    uint64_t max_num_iterations = 500;

    /** The tolerance threshold for changes in the cost function. */
    double function_tolerance = 1e-18;

    /** The tolerance threshold for changes in the gradient. */
    double gradient_tolerance = 1e-20;

    /** The tolerance threshold for changes in parameters. */
    double parameter_tolerance = 1e-18;

    /** Flag to enable or disable detailed printout of the optimization process.
     */
    bool process_printout = true;

    /** Flag to fix the first node of the trajectory during optimization. */
    bool fix_first_node = false;

    /** The robust loss function to use for residuals. */
    LossFunction loss_function = LossFunction::HUBER_LOSS;

    /** Scaling parameter for the chosen loss function. */
    double loss_scale = 1.0;

    /** Vector of constraints to apply during optimization. */
    std::vector<std::unique_ptr<Constraint>> constraints;

    /**
     * @fn SolverConfig::SolverConfig(const SolverConfig&)
     * @brief Copy constructor performing a deep copy of constraints.
     *
     * @param[in] other The SolverConfig to copy from.
     */
    OUSTER_API_FUNCTION
    SolverConfig(const SolverConfig& other)
        : key_frame_distance(other.key_frame_distance),
          traj_rotation_weight(other.traj_rotation_weight),
          traj_translation_weight(other.traj_translation_weight),
          max_num_iterations(other.max_num_iterations),
          function_tolerance(other.function_tolerance),
          gradient_tolerance(other.gradient_tolerance),
          parameter_tolerance(other.parameter_tolerance),
          process_printout(other.process_printout),
          fix_first_node(other.fix_first_node),
          loss_function(other.loss_function),
          loss_scale(other.loss_scale) {
        // Deep copy constraints due to unique_ptr
        constraints.reserve(other.constraints.size());
        for (const auto& constraint : other.constraints) {
            constraints.push_back(constraint->clone());
        }
    }

    /**
     * @brief Copy assignment operator performing a deep copy of constraints.
     *
     * @param[in] other The SolverConfig to assign from.
     * @return SolverConfig& Reference to this.
     */
    OUSTER_API_FUNCTION
    SolverConfig& operator=(const SolverConfig& other) {
        if (this != &other) {
            key_frame_distance = other.key_frame_distance;
            traj_rotation_weight = other.traj_rotation_weight;
            traj_translation_weight = other.traj_translation_weight;
            max_num_iterations = other.max_num_iterations;
            function_tolerance = other.function_tolerance;
            gradient_tolerance = other.gradient_tolerance;
            parameter_tolerance = other.parameter_tolerance;
            process_printout = other.process_printout;
            fix_first_node = other.fix_first_node;
            loss_function = other.loss_function;
            loss_scale = other.loss_scale;

            // Deep copy constraints due to unique_ptr
            constraints.clear();
            constraints.reserve(other.constraints.size());
            for (const auto& constraint : other.constraints) {
                constraints.push_back(constraint->clone());
            }
        }
        return *this;
    }

    /** Default constructor. */
    OUSTER_API_FUNCTION
    SolverConfig() = default;

    /**
     * @brief Move constructor.
     * @param[in] other The SolverConfig to move from.
     */
    OUSTER_API_FUNCTION
    SolverConfig(SolverConfig&& other) = default;

    /**
     * @brief Move assignment operator.
     * @param[in] other The SolverConfig to move-assign from.
     * @return SolverConfig& Reference to this.
     */
    OUSTER_API_FUNCTION
    SolverConfig& operator=(SolverConfig&& other) = default;
};

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

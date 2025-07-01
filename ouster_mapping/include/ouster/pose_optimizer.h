#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>

#include "ouster/pose_optimizer_enums.h"
#include "ouster/visibility.h"

namespace ouster {
namespace mapping {

struct OUSTER_API_CLASS SolverConfig {
    double key_frame_distance = 1.0;
    double traj_rotation_weight = 1.0;
    double traj_translation_weight = 1.0;
    uint64_t max_num_iterations = 2000;
    double function_tolerance = 1e-18;
    double gradient_tolerance = 1e-20;
    double parameter_tolerance = 1e-18;
    bool process_printout = true;
    LossFunction loss_function = LossFunction::HuberLoss;
    double loss_scale = 1.0;
};

/**
 * @class PoseOptimizer
 * @brief Class for optimizing pose constraints in a trajectory using
 * non-linear optimization.
 *
 * PoseOptimizer manages trajectory optimization by minimizing errors
 * between various constraints such as pose-to-pose, absolute pose, and
 * point-to-point relationships. It uses the Ceres solver internally to
 * perform the optimization.
 */

class OUSTER_API_CLASS PoseOptimizer {
   public:
    /**
     * @brief Constructor that initializes PoseOptimizer with trajectory
     * data and solver parameters.
     *
     * This constructor loads trajectory data from an OSF file and
     * configures the optimization problem using the provided solver
     * config. It prepares the internal data structures needed for pose
     * optimization and constraint management.
     *
     * @param[in] osf_filename Path to the OSF file containing trajectory
     * data to be optimized.
     * @param[in] config SolverConfig object specifying optimization
     * parameters such as weights, iteration limits, convergence tolerances,
     * and output settings.
     * @param[in] fix_first_node Flag to fix the first node of the
     * trajectory during the pose optimization.
     */
    OUSTER_API_FUNCTION
    PoseOptimizer(const std::string& osf_filename, const SolverConfig& config,
                  bool fix_first_node = false);

    /**
     * @brief Constructor that initializes PoseOptimizer with an OSF file
     * and a simplified configuration.
     *
     * This constructor loads trajectory data from an OSF file and
     * configures the optimization problem using default parameters, with
     * only the node gap being customizable. The node gap controls the
     * spatial density of optimization nodes along the trajectory. For finer
     * control over other optimization parameters, use the constructor that
     * accepts SolverConfig.
     *
     * @param[in] osf_filename Path to the OSF file containing trajectory
     * data to be optimized.
     * @param[in] key_frame_distance The distance between consecutive
     * optimization nodes along the trajectory.
     * @param[in] fix_first_node Flag to fix the first node of the
     * trajectory during the pose optimization.
     */
    OUSTER_API_FUNCTION
    PoseOptimizer(const std::string& osf_filename, double key_frame_distance,
                  bool fix_first_node = false);

    /**
     * @brief Destructor for PoseOptimizer.
     */
    OUSTER_API_FUNCTION
    ~PoseOptimizer();

    /**
     * @brief Adds a pose-to-pose constraint using a predefined pose
     * difference.
     *
     * This function adds a constraint between two poses at specified
     * timestamps, using a given pose difference. The constraint is weighted
     * by the provided rotation and translation weights.
     *
     * @param[in] ts1 The first valid column timestamp of the first frame.
     * @param[in] ts2 The first valid column timestamp of the second frame.
     * @param[in] diff The pose difference between the first and second
     * frames in a 6x1 algebraic vector.
     * @param[in] rotation_weight Rotational weight for this constraint (0
     * ignores rotation difference).
     * @param[in] translation_weight Translational weight for this
     * constraint (0 ignores translation difference).
     * @return True if the constraint was successfully added, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    bool add_pose_to_pose_constraint(uint64_t ts1, uint64_t ts2,
                                     const Eigen::Matrix<double, 6, 1>& diff,
                                     double rotation_weight = 1.0,
                                     double translation_weight = 1.0);
    /**
     * @brief Adds a pose-to-pose constraint using a predefined pose
     * difference.
     *
     * This function adds a constraint between two poses at specified
     * timestamps, using a given pose difference. The constraint is weighted
     * by the provided rotation and translation weights.
     *
     * @param[in] ts1 The first valid column timestamp of the first frame.
     * @param[in] ts2 The first valid column timestamp of the second frame.
     * @param[in] diff The pose difference between the first and second
     * frames in a 4x4 transformation matrix.
     * @param[in] rotation_weight Rotational weight for this constraint (0
     * ignores rotation difference).
     * @param[in] translation_weight Translational weight for this
     * constraint (0 ignores translation difference).
     * @return True if the constraint was successfully added, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    bool add_pose_to_pose_constraint(uint64_t ts1, uint64_t ts2,
                                     const Eigen::Matrix<double, 4, 4>& diff,
                                     double rotation_weight = 1.0,
                                     double translation_weight = 1.0);

    /**
     * @brief Adds a pose-to-pose constraint using the default ICP algorithm
     * to calculate the frame difference.
     *
     * This function adds a constraint between two poses at specified
     * timestamps, using the ICP algorithm to compute the pose difference.
     * The constraint is weighted by the provided rotation and translation
     * weights.
     *
     * @param[in] ts1 The first valid column timestamp of the first frame.
     * @param[in] ts2 The first valid column timestamp of the second frame.
     * @param[in] rotation_weight Rotational weight for this constraint (0
     * ignores rotation difference).
     * @param[in] translation_weight Translational weight for this
     * constraint (0 ignores translation difference).
     * @return True if the constraint was successfully added, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    bool add_pose_to_pose_constraint(uint64_t ts1, uint64_t ts2,
                                     double rotation_weight = 1.0,
                                     double translation_weight = 1.0);

    /**
     * @brief Adds an absolute pose constraint with a predefined pose
     * difference for a given timestamp.
     *
     * This function adds a constraint to enforce a specific pose at a given
     * timestamp, using a given pose difference. The constraint is weighted
     * by the provided rotation and translation weights.
     *
     * @param[in] ts The timestamp at which the absolute pose constraint is
     * applied.
     * @param[in] target_pose The target pose to be enforced at the given
     * timestamp.
     * @param[in] rotation_weight Rotational weight for this constraint (0
     * ignores rotation difference).
     * @param[in] translation_weight Translational weight for this
     * constraint (0 ignores translation difference).
     * @param[in] diff The pose difference to be applied at the given
     * timestamp in a 6x1 algebraic vector.
     * @return True if the constraint was successfully added, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    bool add_absolute_pose_constraint(
        uint64_t ts, const Eigen::Matrix<double, 6, 1>& target_pose,
        double rotation_weight = 1.0, double translation_weight = 1.0,
        const Eigen::Matrix<double, 6, 1>& diff =
            Eigen::Matrix<double, 6, 1>::Zero());

    /**
     * @brief Adds an absolute pose constraint with a predefined pose
     * difference for a given timestamp.
     *
     * This function adds a constraint to enforce a specific pose at a given
     * timestamp, using a given pose difference. The constraint is weighted
     * by the provided rotation and translation weights.
     *
     * @param[in] ts The timestamp at which the absolute pose constraint is
     * applied.
     * @param[in] target_pose The target pose to be enforced at the given
     * timestamp.
     * @param[in] rotation_weight Rotational weight for this constraint (0
     * ignores rotation difference).
     * @param[in] translation_weight Translational weight for this
     * constraint (0 ignores translation difference).
     * @param[in] diff The pose difference to be applied at the given
     * timestamp in a 4x4 transformation matrix.
     * @return True if the constraint was successfully added, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    bool add_absolute_pose_constraint(
        uint64_t ts, const Eigen::Matrix<double, 4, 4>& target_pose,
        double rotation_weight = 1.0, double translation_weight = 1.0,
        const Eigen::Matrix<double, 4, 4>& diff =
            Eigen::Matrix<double, 4, 4>::Identity());

    /**
     * @brief Adds an absolute pose constraint with a predefined pose
     * difference for a given timestamp.
     *
     * This function adds a constraint to enforce a specific pose at a given
     * timestamp, using a given pose difference. The constraint is weighted
     * by the provided rotation and translation weights.
     *
     * @param[in] ts The timestamp at which the absolute pose constraint is
     * applied.
     * @param[in] target_pose The target pose to be enforced at the given
     * timestamp.
     * @param[in] rotation_weights Rotational weight for each axis of this
     * constraint (0 ignores rotation difference).
     * @param[in] translation_weights Translational weight for each
     * direction of this constraint (0 ignores translation difference).
     * @param[in] diff The pose difference to be applied at the given
     * timestamp in a 6x1 algebraic vector.
     * @return True if the constraint was successfully added, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    bool add_absolute_pose_constraint(
        uint64_t ts, const Eigen::Matrix<double, 6, 1>& target_pose,
        const std::array<double, 3>& rotation_weights = {1.0, 1.0, 1.0},
        const std::array<double, 3>& translation_weights = {1.0, 1.0, 1.0},
        const Eigen::Matrix<double, 6, 1>& diff =
            Eigen::Matrix<double, 6, 1>::Zero());

    /**
     * @brief Adds an absolute pose constraint with a predefined pose
     * difference for a given timestamp.
     *
     * This function adds a constraint to enforce a specific pose at a given
     * timestamp, using a given pose difference. The constraint is weighted
     * by the provided rotation and translation weights.
     *
     * @param[in] ts The timestamp at which the absolute pose constraint is
     * applied.
     * @param[in] target_pose The target pose to be enforced at the given
     * timestamp.
     * @param[in] rotation_weights Rotational weight for each axis of this
     * constraint (0 ignores rotation difference).
     * @param[in] translation_weights Translational weight for each
     * direction of this constraint (0 ignores translation difference).
     * @param[in] diff The pose difference to be applied at the given
     * timestamp in a 4x4 transformation matrix.
     * @return True if the constraint was successfully added, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    bool add_absolute_pose_constraint(
        uint64_t ts, const Eigen::Matrix<double, 4, 4>& target_pose,
        const std::array<double, 3>& rotation_weights = {1.0, 1.0, 1.0},
        const std::array<double, 3>& translation_weights = {1.0, 1.0, 1.0},
        const Eigen::Matrix<double, 4, 4>& diff =
            Eigen::Matrix<double, 4, 4>::Identity());

    /**
     * @brief Adds a point-to-point constraint between two points in
     * different frames.
     *
     * This function adds a constraint between two points at specified
     * timestamps and positions in their respective frames. The constraint
     * is weighted by the provided translation weight.
     *
     * @param[in] ts1 The first valid column timestamp of the first frame.
     * @param[in] row1 The row index of the point in the first frame.
     * @param[in] col1 The column index of the point in the first frame.
     * @param[in] return_idx1 The return index of the point in the first
     * frame. Either 1 or 2.
     * @param[in] ts2 The first valid column timestamp of the second frame.
     * @param[in] row2 The row index of the point in the second frame.
     * @param[in] col2 The column index of the point in the second frame.
     * @param[in] return_idx2 The return index of the point in the second
     * frame. Either 1 or 2.
     * @param[in] translation_weight Translational weight for this
     * constraint (0 ignores translation difference).
     * @return True if the constraint was successfully added, false
     * otherwise.
     */
    OUSTER_API_FUNCTION
    bool add_point_to_point_constraint(uint64_t ts1, uint32_t row1,
                                       uint32_t col1, uint32_t return_idx1,
                                       uint64_t ts2, uint32_t row2,
                                       uint32_t col2, uint32_t return_idx2,
                                       double translation_weight = 1.0);

    /**
     * @brief Optimizes the trajectory using Ceres.
     *
     * Runs the optimization from the current state. If a positive number of
     * iterations is specified, it overrides the internal setting for
     * max_num_iterations. If steps is 0, the optimizer uses the currently
     * configured max_num_iterations value without overriding it.
     *
     * @param[in] steps Number of iterations to run. Must be >= 0.
     *                  If 0, the default configured value is used.
     */
    OUSTER_API_FUNCTION
    void solve(uint32_t steps = 0);

    /**
     * @brief Saves the optimized trajectory to an OSF file.
     *
     * This function saves the current state of the optimized trajectory to
     * an OSF file with the specified name.
     *
     * @param[in] osf_name The name of the output OSF file.
     */
    OUSTER_API_FUNCTION
    void save(const std::string& osf_name);

    /**
     * @brief Retrieves timestamps based on the specified sampling mode.
     *
     * This function returns a sequence of timestamps (in nanoseconds)
     * extracted from the trajectory, based on the provided sampling mode.
     *
     * - SamplingMode::KEY_FRAMES: Returns timestamps associated with key-frame
     * poses.
     * - SamplingMode::COLUMNS: Returns timestamps corresponding to each column
     *   in the original LiDAR scans.
     *
     * @param[in] type Sampling mode used to determine which timestamps to
     * return.
     * @return A vector of timestamps in ascending order.
     *
     * @throws std::invalid_argument if the sampling mode is invalid.
     */
    OUSTER_API_FUNCTION
    std::vector<uint64_t> get_timestamps(SamplingMode type) const;

    /**
     * @brief Retrieves pose matrices corresponding to the selected sampling
     * mode.
     *
     * Returns a list of 4×4 homogeneous transformation matrices, each
     * representing the pose of the sensor or platform at a specific point in
     * time. The returned poses are aligned with the timestamps returned by
     * `get_timestamps(type)`.
     *
     * - SamplingMode::KEY_FRAMES: Returns poses at selected key frames, which
     * typically represent major changes in motion or position.
     * - SamplingMode::COLUMNS: Returns a pose for each column in the LiDAR
     * scan, useful for fine-grained temporal alignment.
     *
     * @param[in] type The sampling mode: SamplingMode::KEY_FRAMES or
     * SamplingMode::COLUMNS.
     * @return A vector of 4×4 transformation matrices ordered to match the
     * timestamps.
     *
     * @throws std::invalid_argument if an unsupported sampling mode is
     * provided.
     */
    OUSTER_API_FUNCTION
    std::vector<Eigen::Matrix<double, 4, 4>> get_poses(SamplingMode type);

   private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

/**
 * @brief Saves a trajectory into a file
 *
 * This function exports the given sequence of timestamps and corresponding 4×4
 * pose matrices to an output file. Supported output formats are:
 *  - "csv": comma-separated values with columns
 *             timestamp, tx, ty, tz, qx, qy, qz, qw
 *  - "tum": TUM trajectory format: timestamp tx ty tz qx qy qz qw
 *
 * @param[in] filename   Path of the file to write.
 * @param[in] timestamps A vector of uint64_t timestamps, one per pose.
 * @param[in] poses      A vector of 4×4 Eigen homogeneous transforms matrix,
 *                       each representing the pose at the matching timestamp.
 * @param[in] file_type  Output format selector: "csv" or "tum" (default =
 * "csv").
 * @return True if the file was successfully created and written; false on I/O
 * error.
 */
OUSTER_API_FUNCTION
bool save_trajectory(const std::string& filename,
                     const std::vector<uint64_t>& timestamps,
                     const std::vector<Eigen::Matrix<double, 4, 4>>& poses,
                     const std::string& file_type = "csv");

}  // namespace mapping
}  // namespace ouster

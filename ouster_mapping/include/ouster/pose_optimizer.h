#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/pose_optimizer_constraint.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace mapping {

/**
 * @class PoseOptimizer
 * @brief Class for optimizing pose constraints in a trajectory using
 * non-linear optimization.
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
     */
    OUSTER_API_FUNCTION
    PoseOptimizer(const std::string& osf_filename, const SolverConfig& config);

    /**
     * @brief Constructor with automatic constraint loading from JSON config
     * file.
     *
     * This constructor loads trajectory data from an OSF file and automatically
     * loads constraints from a JSON configuration file. The JSON file should
     * contain both solver parameters and constraint definitions.
     *
     * @param[in] osf_filename Path to the OSF file containing trajectory data.
     * @param[in] config_filename Path to the JSON configuration file.
     */
    OUSTER_API_FUNCTION
    PoseOptimizer(const std::string& osf_filename,
                  const std::string& config_filename);

    /**
     * @brief Constructor that initializes PoseOptimizer with an OSF file
     * and a simplified configuration.
     *
     * This constructor loads trajectory data from an OSF file and
     * configures the optimization problem using default parameters, with
     * only the node gap being customizable.
     *
     * @param[in] osf_filename Path to the OSF file containing trajectory
     * data to be optimized.
     * @param[in] key_frame_distance The distance between consecutive
     * optimization nodes along the trajectory.
     */
    OUSTER_API_FUNCTION
    PoseOptimizer(const std::string& osf_filename, double key_frame_distance);

    /**
     * @brief Destructor for PoseOptimizer.
     */
    OUSTER_API_FUNCTION
    ~PoseOptimizer();

    /**
     * @brief Save the current SolverConfig (including new added constraints)
     * to a JSON file.
     *
     * This function serializes the current solver configuration and all
     * constraints to a JSON file. The resulting file can be used later with
     * Pose Optimizer construction to restore the exact same optimization setup.
     *
     * @param[in] config_filename The path of the JSON configuration file.
     * @throws std::runtime_error if the file cannot be saved.
     */
    OUSTER_API_FUNCTION
    void save_config(const std::string& config_filename);

    /**
     * @brief Add a constraint to the optimization problem.
     *
     * This is the main method for adding constraints to the pose optimizer.
     * It accepts any constraint derived from Constraint. The constraint object
     * carries its own ID assigned at construction; adding will fail if the ID
     * is already in use.
     *
     * @param[in] constraint A constraint object derived from Constraint.
     * @return The unique ID of the constraint.
     * @throws std::runtime_error if the constraint cannot be added.
     */
    OUSTER_API_FUNCTION
    uint32_t add_constraint(std::unique_ptr<Constraint> constraint);

    /**
     * @brief Remove a constraint from the pose optimization problem.
     *
     * @param[in] constraint_id The unique ID of the constraint to remove.
     * @throws std::runtime_error if the constraint ID is not found.
     */
    OUSTER_API_FUNCTION
    void remove_constraint(uint32_t constraint_id);

    /**
     * @brief Initialize trajectory alignment using average absolute
     * constraints.
     *
     * Computes a weighted average transformation matrix from the currently
     * configured absolute pose and absolute point constraints (using their
     * weights and Lie algebra representation) and applies it uniformly to the
     * entire trajectory as an initial alignment step before optimization.
     *
     * @return true if an alignment transform was successfully applied, false
     * otherwise (e.g. not enough data, negligible transform).
     */
    OUSTER_API_FUNCTION
    bool initialize_trajectory_alignment();

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
     * @return the current cost value.
     */
    OUSTER_API_FUNCTION
    double solve(uint32_t steps = 0);

    /**
     * @brief Saves the optimized trajectory to an OSF file.
     *
     * This function saves the current state of the optimized trajectory to
     * an OSF file with the specified name.
     *
     * @param[in] osf_filename The name of the output OSF file.
     */
    OUSTER_API_FUNCTION
    void save(const std::string& osf_filename);

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

    /**
     * @brief Return the configured key-frame distance.
     *
     * @return the key_frame_distance which represents the spacing in meters
     * used when constructing the trajectory.
     */
    OUSTER_API_FUNCTION
    double get_key_frame_distance() const;

    /**
     * @brief Set a general callback to be invoked at each solver iteration.
     *
     * The callback is called from Ceres' iteration callback during solve().
     * Use this to hook custom logic such as visualization. The callback is
     * executed in the same thread that calls solve().
     *
     * @param[in] fn A general callable. Capture what you need.
     */
    OUSTER_API_FUNCTION
    void set_solver_step_callback(std::function<void()> fn);

    /**
     * @brief Get all constraints currently configured in the pose optimizer.
     *
     * This function returns a copy of all constraints that are currently
     * configured in the pose optimizer, including both constraints loaded
     * from JSON files during construction and constraints added later via
     * add_constraint().
     *
     * @return A vector of unique_ptr to Constraint objects representing all
     * currently configured constraints.
     */
    OUSTER_API_FUNCTION
    std::vector<std::unique_ptr<Constraint>> get_constraints() const;

    /**
     * @brief Set all constraints for the pose optimizer.
     *
     * This function replaces all existing constraints with the provided set
     * of constraints. Any constraints previously loaded from JSON files or
     * added via add_constraint() will be removed and replaced with the new
     * constraint set.
     *
     * @param[in] constraints A vector of unique_ptr to Constraint objects to
     * set as the complete constraint set.
     * @throws std::runtime_error if the constraints cannot be set.
     */
    OUSTER_API_FUNCTION
    void set_constraints(std::vector<std::unique_ptr<Constraint>> constraints);

    /**
     * @brief Get a node by its timestamp.
     *
     * @param[in] timestamp The timestamp (nanoseconds) of the node to retrieve.
     * @return A shared pointer to the Node with the specified timestamp. If no
     * node with the given timestamp exists in the trajectory, this function
     * returns nullptr.
     */
    OUSTER_API_FUNCTION
    std::shared_ptr<class Node> get_node(uint64_t timestamp) const;

    /**
     * @brief Get the last solver cost value.
     *
     * @return the final cost from the most recent call to solve(). If solve()
     * has not been called yet, the value will be -1
     */
    OUSTER_API_FUNCTION
    double get_cost_value() const;

    /**
     * @brief Get the cumulative number of solver iterations executed.
     *
     * @return the total number of iterations run across all calls to solve().
     */
    OUSTER_API_FUNCTION
    uint64_t get_total_iterations() const;

    /**
     * @brief Retrieve evenly sampled scan nodes with point clouds.
     *
     * @param[in] count Maximum number of samples to retrieve.
     * @return A vector of nodes, each guaranteed to have a downsampled cloud.
     */
    OUSTER_API_FUNCTION
    std::vector<std::shared_ptr<class Node>> get_sampled_nodes(
        size_t count = 100) const;

   private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    /**
     * @brief Clear all constraints stored in the PoseOptimizer.
     *
     * This private function removes all constraints stored in the Pose
     * Optimizer.
     */
    void clear_constraints();
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

 * @throws std::runtime_error on I/O error or unsupported file type.
 */
OUSTER_API_FUNCTION
void save_trajectory(const std::string& filename,
                     const std::vector<uint64_t>& timestamps,
                     const std::vector<Eigen::Matrix<double, 4, 4>>& poses,
                     const std::string& file_type = "csv");

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster

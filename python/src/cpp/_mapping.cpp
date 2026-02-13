#define FMT_UNICODE 0

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "ouster/active_time_correction.h"
#include "ouster/deskew_method.h"
#include "ouster/localization_engine.h"
#include "ouster/pose_optimizer.h"
#include "ouster/pose_optimizer_constraint.h"
#include "ouster/pose_optimizer_node.h"
#include "ouster/slam_engine.h"

namespace py = pybind11;
using namespace ouster::sdk::mapping;
using namespace ouster::sdk::core;

class PyDeskewMethod : public DeskewMethod {
   public:
    using DeskewMethod::DeskewMethod;

    void update(LidarScanSet& lidar_scan_set) override {
        PYBIND11_OVERRIDE_PURE(void, DeskewMethod, update, lidar_scan_set);
    }

    void set_last_pose(int64_t ts, const Matrix4dR& pose) override {
        PYBIND11_OVERRIDE_PURE(void, DeskewMethod, set_last_pose, ts, pose);
    }
};

void init_mapping(py::module& module, py::module& /*unused*/) {
    module.doc() =
        "Python bindings for PoseOptimizer, enabling trajectory optimization "
        "using C++. This module provides tools for optimizing sensor "
        "trajectories "
        "by applying various constraints between poses or points in a LiDAR "
        "mapping context.";

    //----- Wrap SolverConfig -------------------------------------------
    py::class_<SolverConfig>(module, "SolverConfig", R"pbdoc(
    Configuration options for the non-linear optimization solver used in trajectory refinement.

    This class encapsulates the configuration options for the solver used in the PoseOptimizer.

    Attributes:
        key_frame_distance (float): The distance between nodes in the trajectory (in meters). Controls trajectory discretization.
        traj_rotation_weight (float): The weight for rotational constraints during trajectory optimization. Higher values enforce stronger rotation consistency.
        traj_translation_weight (float): The weight for translational constraints during trajectory optimization. Higher values enforce stronger position consistency.
        max_num_iterations (int): The maximum number of iterations the solver will perform before terminating.
        function_tolerance (float): The tolerance threshold for changes in the cost function. Solver stops when improvements fall below this value.
        gradient_tolerance (float): The tolerance threshold for changes in the gradient. Solver stops when gradient magnitude falls below this value.
        parameter_tolerance (float): The tolerance threshold for changes in parameters. Solver stops when parameter changes fall below this value.
        process_printout (bool): Flag to enable or disable detailed printout of the optimization process.
        loss_function (str): The name of the robust loss function to use (e.g., "HUBER_LOSS", "CAUCHY_LOSS", "SOFT_L_ONE_LOSS", "ARCTAN_LOSS", "TRIVIAL_LOSS").
        loss_scale (float): The scaling parameter for the chosen loss function. Higher values make the loss less sensitive to outliers.
        fix_first_node (bool): Flag to fix the first node of the trajectory during optimization. Default is False.
    )pbdoc")
        .def(py::init<>(), "Initialize SolverConfig with default values.")
        .def_readwrite("key_frame_distance", &SolverConfig::key_frame_distance)
        .def_readwrite("traj_rotation_weight",
                       &SolverConfig::traj_rotation_weight)
        .def_readwrite("traj_translation_weight",
                       &SolverConfig::traj_translation_weight)
        .def_readwrite("max_num_iterations", &SolverConfig::max_num_iterations)
        .def_readwrite("function_tolerance", &SolverConfig::function_tolerance)
        .def_readwrite("gradient_tolerance", &SolverConfig::gradient_tolerance)
        .def_readwrite("parameter_tolerance",
                       &SolverConfig::parameter_tolerance)
        .def_readwrite("process_printout", &SolverConfig::process_printout)
        .def_readwrite("loss_function", &SolverConfig::loss_function)
        .def_readwrite("loss_scale", &SolverConfig::loss_scale)
        .def_readwrite("fix_first_node", &SolverConfig::fix_first_node);

    py::enum_<SamplingMode>(module, "SamplingMode")
        .value("KEY_FRAMES", SamplingMode::KEY_FRAMES)
        .value("COLUMNS", SamplingMode::COLUMNS);

    py::enum_<LossFunction>(module, "LossFunction")
        .value("HUBER_LOSS", LossFunction::HUBER_LOSS)
        .value("CAUCHY_LOSS", LossFunction::CAUCHY_LOSS)
        .value("SOFT_L_ONE_LOSS", LossFunction::SOFT_L_ONE_LOSS)
        .value("ARCTAN_LOSS", LossFunction::ARCTAN_LOSS)
        .value("TRIVIAL_LOSS", LossFunction::TRIVIAL_LOSS)

        .def_static("from_string", &loss_function_from_string, py::arg("name"),
                    R"pbdoc(
                Convert a string (e.g. "HUBER_LOSS") to the corresponding LossFunction enum.

                Args:
       name: one of "HUBER_LOSS", "CAUCHY_LOSS", "SOFT_L_ONE_LOSS", "ARCTAN_LOSS", "TRIVIAL_LOSS"
            )pbdoc");

    //----- Wrap PoseOptimizerNode (internal node) ---------------------------
    py::class_<Node, std::shared_ptr<Node>>(module, "PoseOptimizerNode")
        .def_property_readonly("ts", [](const Node& n) { return n.ts; })
        .def_property_readonly(
            "downsampled_pts",
            [](const Node& n) { return n.downsampled_pts.matrix(); })
        .def_property_readonly(
            "ptp_constraint_pt",
            [](const Node& n) { return n.ptp_constraint_pt.matrix(); })
        .def_property_readonly(
            "ap_constraint_pt",
            [](const Node& n) { return n.ap_constraint_pt.matrix(); })
        .def("get_pose", [](const Node& n) { return n.get_pose(); });

    //----- Wrap constraint classes -------------------------------------------
    // Base constraint class
    py::class_<Constraint>(module, "Constraint", R"pbdoc(
        Base class for all pose optimization constraints.

        This is the abstract base class for all constraints used in pose optimization.
        Use the specific constraint classes like AbsolutePoseConstraint, PoseToPoseConstraint,
        or PointToPointConstraint instead of using this class directly.
    )pbdoc")
        .def_readwrite("translation_weights", &Constraint::translation_weights)
        .def("get_constraint_id", &Constraint::get_constraint_id, R"pbdoc(
            Get the unique constraint ID. Returns 0 for non-user constraints.
            IDs are assigned when constraint objects are constructed.

            Returns:
                int: The constraint ID, or 0 if not a user-added constraint.
        )pbdoc");

    py::class_<AbsolutePoseConstraint, Constraint>(module,
                                                   "AbsolutePoseConstraint",
                                                   R"pbdoc(
        Absolute pose constraint - fixes a pose at a specific timestamp.

        This constraint type enforces that the sensor pose at a given timestamp
        matches a specific target pose.
    )pbdoc")
        .def(py::init<>(), "Default constructor")
        .def(py::init<uint64_t, const Eigen::Matrix4d&, double,
                      const Eigen::Array3d&>(),
             py::arg("timestamp"), py::arg("pose"),
             py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = Eigen::Array3d::Ones(),
             R"pbdoc(
                              Constructor for AbsolutePoseConstraint.

               Args:
                   timestamp (int): Timestamp of the pose to constrain (nanoseconds)
                   pose: The 4x4 transformation matrix (SE3) to constrain to
                   rotation_weight: Scalar weight applied to the quaternion axis-alignment residual
                   translation_weight: Weight for translation constraints (x, y, z)
               )pbdoc")
        .def_readwrite("timestamp", &AbsolutePoseConstraint::timestamp)
        .def_readwrite("pose", &AbsolutePoseConstraint::pose)
        .def_readwrite("rotation_weight",
                       &AbsolutePoseConstraint::rotation_weight)
        .def_readwrite("translation_weights",
                       &AbsolutePoseConstraint::translation_weights);

    py::class_<PoseToPoseConstraint, Constraint>(module, "PoseToPoseConstraint",
                                                 R"pbdoc(
        Relative pose-to-pose constraint - enforces relative transformation between two poses.

        This constraint type enforces a specific relative transformation between
        two poses at different timestamps.
    )pbdoc")
        .def(py::init<>(), "Default constructor")
        .def(py::init<uint64_t, uint64_t, const Eigen::Matrix4d&, double,
                      const Eigen::Array3d&>(),
             py::arg("timestamp1"), py::arg("timestamp2"),
             py::arg("relative_pose") = Eigen::Matrix4d::Identity(),
             py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = Eigen::Array3d::Ones(),
             R"pbdoc(
               Constructor for PoseToPoseConstraint.

               Args:
                   timestamp1 (int): Timestamp of the first pose (nanoseconds)
                   timestamp2 (int): Timestamp of the second pose (nanoseconds)
                   relative_pose: Expected relative transformation from pose1 to pose2.
                                 Use the identity matrix to let PoseOptimizer auto-estimate it via ICP.
                   rotation_weight: Scalar weight applied to the quaternion axis-alignment residual
                   translation_weight: Weight for translation constraints (x, y, z)
               )pbdoc")
        .def_readwrite("timestamp1", &PoseToPoseConstraint::timestamp1)
        .def_readwrite("timestamp2", &PoseToPoseConstraint::timestamp2)
        .def_readwrite("relative_pose", &PoseToPoseConstraint::relative_pose)
        .def_readwrite("rotation_weight",
                       &PoseToPoseConstraint::rotation_weight)
        .def_readwrite("translation_weights",
                       &PoseToPoseConstraint::translation_weights);

    py::class_<PointToPointConstraint, Constraint>(module,
                                                   "PointToPointConstraint",
                                                   R"pbdoc(
        Point-to-point constraint - enforces correspondence between points.

        This constraint type enforces that specific points in two different
        lidar scans correspond to the same physical location.
    )pbdoc")
        .def(py::init<>(), "Default constructor")
        .def(py::init<uint64_t, uint32_t, uint32_t, uint32_t, uint64_t,
                      uint32_t, uint32_t, uint32_t, const Eigen::Array3d&>(),
             py::arg("timestamp1"), py::arg("row1"), py::arg("col1"),
             py::arg("return_idx1"), py::arg("timestamp2"), py::arg("row2"),
             py::arg("col2"), py::arg("return_idx2"),
             py::arg("translation_weight") = Eigen::Array3d::Ones(),
             R"pbdoc(
               Constructor for PointToPointConstraint.

               Args:
                   timestamp1 (int): Timestamp of the first point's pose (nanoseconds)
                   row1 (int): Row index of the first point
                   col1 (int): Column index of the first point
                   return_idx1 (int): Return index of the first point (1 or 2)
                   timestamp2 (int): Timestamp of the second point's pose (nanoseconds)
                   row2 (int): Row index of the second point
                   col2 (int): Column index of the second point
                   return_idx2 (int): Return index of the second point (1 or 2)
                   translation_weight: Weight for translation constraints (x, y, z)
               )pbdoc")
        .def_readwrite("timestamp1", &PointToPointConstraint::timestamp1)
        .def_readwrite("timestamp2", &PointToPointConstraint::timestamp2)
        .def_readwrite("row1", &PointToPointConstraint::row1)
        .def_readwrite("col1", &PointToPointConstraint::col1)
        .def_readwrite("return_idx1", &PointToPointConstraint::return_idx1)
        .def_readwrite("row2", &PointToPointConstraint::row2)
        .def_readwrite("col2", &PointToPointConstraint::col2)
        .def_readwrite("return_idx2", &PointToPointConstraint::return_idx2)
        .def_readwrite("translation_weights",
                       &PointToPointConstraint::translation_weights);

    py::class_<AbsolutePointConstraint, Constraint>(module,
                                                    "AbsolutePointConstraint",
                                                    R"pbdoc(
        Absolute point constraint.

        Constrains a single 3D point from a LiDAR scan, identified by its 2D
        image coordinates (row, col) and return index at a given timestamp, to
        match a user-defined absolute 3D position in the world frame. The 3D
        point is computed the same way as in PointToPointConstraint (via RANGE/
        RANGE2 and the XYZ LUT), but is compared to a provided global point
        instead of another scan point.
    )pbdoc")
        .def(py::init<>(), "Default constructor")
        .def(py::init<uint64_t, uint32_t, uint32_t, uint32_t,
                      const Eigen::Vector3d&, const Eigen::Array3d&>(),
             py::arg("timestamp"), py::arg("row"), py::arg("col"),
             py::arg("return_idx"), py::arg("absolute_position"),
             py::arg("translation_weight") = Eigen::Array3d::Ones(),
             R"pbdoc(
               Constructor for AbsolutePointConstraint.

               Args:
                   timestamp (int): Timestamp of the point's pose (nanoseconds)
                   row (int): Row index of the point
                   col (int): Column index of the point
                   return_idx (int): Return index of the point (1 or 2)
                   absolute_position: Target world position (x, y, z)
                   translation_weight: Weight for translation constraints (x, y, z)
               )pbdoc")
        .def_readwrite("timestamp", &AbsolutePointConstraint::timestamp)
        .def_readwrite("row", &AbsolutePointConstraint::row)
        .def_readwrite("col", &AbsolutePointConstraint::col)
        .def_readwrite("return_idx", &AbsolutePointConstraint::return_idx)
        .def_readwrite("absolute_position",
                       &AbsolutePointConstraint::absolute_position)
        .def_readwrite("translation_weights",
                       &AbsolutePointConstraint::translation_weights);

    //----- Wrap PoseOptimizer -------------------------------------------
    py::class_<PoseOptimizer>(module, "PoseOptimizer", R"pbdoc(
        A class for optimizing LiDAR sensor trajectories using various geometric constraints.

        This class allows adding different types of constraints (pose-to-pose, absolute pose, point-to-point)
        and solving the trajectory optimization problem to generate a more accurate and consistent sensor path.
        The optimization aims to minimize the error across all defined constraints while maintaining
        a physically plausible trajectory.
    )pbdoc")
        // Constructors
        .def(py::init<const std::string&, const SolverConfig&>(),
             py::arg("osf_filename"), py::arg("options"),
             R"pbdoc(
                 Initialize PoseOptimizer with an OSF file and solver options.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     options (SolverConfig): Solver configuration options. Set options.fix_first_node to True to fix the first node.
                 )pbdoc")

        .def(py::init<const std::string&, const std::string&>(),
             py::arg("osf_filename"), py::arg("config_filename"),
             R"pbdoc(
                 Initialize PoseOptimizer with an OSF file and solver options loaded from a config file.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     config_filename: Path to the configuration file (JSON) containing solver options. Set fix_first_node in the config file to True to fix the first node.
                 )pbdoc")

        .def(py::init<const std::string&, double>(), py::arg("osf_filename"),
             py::arg("key_frame_distance"),
             R"pbdoc(
                 Initialize PoseOptimizer with an OSF file and a node gap.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     key_frame_distance (float): The gap distance between nodes in the trajectory.
                     To fix the first node, set fix_first_node in the SolverConfig after construction.
                 )pbdoc")

        // Methods for managing constraints
        .def(
            "add_constraint",
            [](PoseOptimizer& self, Constraint& constraint) -> uint32_t {
                // Create unique_ptr from the constraint object and add it to
                // the optimizer
                std::unique_ptr<Constraint> constraint_ptr;

                // Attempt to cast to specific constraint types and clone them
                if (auto* abs_constraint =
                        dynamic_cast<AbsolutePoseConstraint*>(&constraint)) {
                    constraint_ptr = std::make_unique<AbsolutePoseConstraint>(
                        *abs_constraint);
                } else if (auto* rel_pose_constraint =
                               dynamic_cast<PoseToPoseConstraint*>(
                                   &constraint)) {
                    constraint_ptr = std::make_unique<PoseToPoseConstraint>(
                        *rel_pose_constraint);
                } else if (auto* rel_point_constraint =
                               dynamic_cast<PointToPointConstraint*>(
                                   &constraint)) {
                    constraint_ptr = std::make_unique<PointToPointConstraint>(
                        *rel_point_constraint);
                } else if (auto* abs_point_constraint =
                               dynamic_cast<AbsolutePointConstraint*>(
                                   &constraint)) {
                    constraint_ptr = std::make_unique<AbsolutePointConstraint>(
                        *abs_point_constraint);
                } else {
                    throw std::invalid_argument("Unknown constraint type");
                }

                return self.add_constraint(std::move(constraint_ptr));
            },
            py::arg("constraint"),
            R"pbdoc(
               Add a constraint to the pose optimization problem.

               This is the new unified API for adding constraints. Use the constraint class
               constructors to create constraints, then pass them to this method.
               The constraint must already have a unique ID for later removal.
               Adding a constraint with a duplicate ID will fail.

               Args:
                   constraint: A constraint object created by one of the constraint constructors.

               Returns:
                   int: The unique constraint ID of the added constraint.

               Raises:
                   RuntimeError: If the constraint cannot be added.
               )pbdoc")

        .def("remove_constraint", &PoseOptimizer::remove_constraint,
             py::arg("constraint_id"),
             R"pbdoc(
               Remove a constraint from the pose optimization problem.

               Args:
                   constraint_id (int): The unique ID of the constraint to remove.

               Raises:
                   RuntimeError: If the constraint ID is not found.
               )pbdoc")

        .def(
            "get_constraints",
            [](const PoseOptimizer& self) -> py::list {
                auto constraints = self.get_constraints();
                py::list result;

                for (const auto& constraint : constraints) {
                    if (auto* abs_constraint =
                            dynamic_cast<const AbsolutePoseConstraint*>(
                                constraint.get())) {
                        result.append(*abs_constraint);
                    } else if (auto* rel_pose_constraint =
                                   dynamic_cast<const PoseToPoseConstraint*>(
                                       constraint.get())) {
                        result.append(*rel_pose_constraint);
                    } else if (auto* rel_point_constraint =
                                   dynamic_cast<const PointToPointConstraint*>(
                                       constraint.get())) {
                        result.append(*rel_point_constraint);
                    } else if (auto* abs_point_constraint =
                                   dynamic_cast<const AbsolutePointConstraint*>(
                                       constraint.get())) {
                        result.append(*abs_point_constraint);
                    }
                }

                return result;
            },
            R"pbdoc(
               Get all constraints currently added to the pose optimizer.

               Returns:
                   list: A list of constraint objects (copies) currently configured in the optimizer.
               )pbdoc")

        .def("initialize_trajectory_alignment",
             &PoseOptimizer::initialize_trajectory_alignment,
             R"pbdoc(
                Initialize trajectory alignment using average absolute constraints.

                Computes a weighted average SE(3) transform from the currently
                loaded absolute pose and absolute point constraints (using their
                weights in Lie algebra space) and left-multiplies the entire
                trajectory by that transform as an initial alignment step before
                optimization.

                Returns:
                    bool: True if an alignment transform was applied, False if
                        skipped (e.g. no absolute constraints or negligible delta).
            )pbdoc")

        .def("solve", &PoseOptimizer::solve, py::arg("steps") = 0,
             R"pbdoc(
             Incrementally optimize the trajectory.

             This method performs a fixed number of iterations of the optimization algorithm,
             continuing from the current state. It can be called repeatedly to gradually refine
             the trajectory. The number of iterations to execute is specified by 'steps'.

             Args:
                 steps (int, optional): The number of iterations to run for this incremental
                     optimization. Defaults to 0 (uses whatever max_num_iterations was already set).
	     Returns:
                 float: The cost value from the last solve call.
         )pbdoc")

        .def(
            "set_solver_step_callback",
            [](PoseOptimizer& self, py::function callback) {
                // Wrap the Python callable in a C++ functor.
                // The callback is invoked from the same thread as solve().
                self.set_solver_step_callback([cb = std::move(callback)]() {
                    py::gil_scoped_acquire acquire;
                    try {
                        cb();
                    } catch (py::error_already_set& e) {
                        // Avoid throwing through ceres callback; report error
                        PyErr_WriteUnraisable(e.value().ptr());
                    }
                });
            },
            py::arg("callback"),
            R"pbdoc(
                Register a Python callable to be invoked at each solver iteration.

                The callable runs on the same thread as ``solve()``, once per ceres
                iteration. Use this to hook visualization or logging.

                Args:
                    callback (Callable[[], None]): Function to call each iteration.
            )pbdoc")

        .def("get_cost_value", &PoseOptimizer::get_cost_value,
             R"pbdoc(
                Get the last solver cost value (final cost from the last solve()).

                Returns:
                    float: The last recorded solver cost value.
            )pbdoc")

        .def("save_config", &PoseOptimizer::save_config,
             py::arg("config_filename"),
             R"pbdoc(
        Save the current SolverConfig (including constraints) to a JSON file.

        This method serializes the current solver configuration and all constraints
        to a JSON file. The resulting file can be used later with Pose Optimizer
        construction to restore the exact same optimization setup.

        Args:
            config_filename (str): Path where the JSON file should be saved.

        Raises:
            RuntimeError: If the file cannot be saved.
        )pbdoc")

        .def("get_total_iterations", &PoseOptimizer::get_total_iterations,
             R"pbdoc(
                Get the cumulative number of solver iterations executed so far.

                Returns:
                    int: Total iterations across all calls to solve().
            )pbdoc")

        .def(
            "get_sampled_nodes",
            [](PoseOptimizer& self, size_t count) {
                auto nodes = self.get_sampled_nodes(count);
                py::list out;
                for (auto& node : nodes) {
                    out.append(node);
                }
                return out;
            },
            py::arg("count") = 100,
            R"pbdoc(
               Retrieve up to `count` scan nodes evenly sampled across the OSF.

               Each node is guaranteed to have a downsampled point cloud; nodes
               are created on-demand if necessary.
            )pbdoc")

        .def(
            "get_node",
            [](const PoseOptimizer& self, uint64_t ts) {
                auto n = self.get_node(ts);
                return n;
            },
            py::arg("timestamp"),
            R"pbdoc(
               Get the node associated with a given timestamp (first-valid-column ts).
               The node pose is updated before returning. Returns None if not found.
            )pbdoc")

        .def(
            "get_timestamps",
            [](const PoseOptimizer& self, SamplingMode type) {
                std::vector<uint64_t> vec = self.get_timestamps(type);

                py::array_t<uint64_t> arr(vec.size());
                auto buf = arr.request();
                uint64_t* ptr = static_cast<uint64_t*>(buf.ptr);

                std::memcpy(ptr, vec.data(), vec.size() * sizeof(uint64_t));
                return arr;
            },

            py::arg("type"),
            R"pbdoc(
                Retrieve timestamps corresponding to the selected sampling mode.

                Args:
                    type (SamplingMode): Sampling strategy to use.
                        - SamplingMode.KEY_FRAMES: Returns timestamps at key-frame poses.
                        - SamplingMode.COLUMNS: Returns timestamps of every lidarscan's columns.

                Returns:
                    numpy.ndarray[np.uint64]: A 1D array of timestamps (nanoseconds).
            )pbdoc")

        .def(
            "get_poses",
            [](PoseOptimizer& self, SamplingMode type) {
                auto mats = self.get_poses(type);
                py::ssize_t num_poses = static_cast<py::ssize_t>(mats.size());

                std::vector<py::ssize_t> shape = {num_poses, 4, 4};
                py::array_t<double> arr(shape);

                auto buf = arr.mutable_unchecked<3>();
                for (py::ssize_t i = 0; i < num_poses; ++i) {
                    for (int row = 0; row < 4; ++row) {
                        for (int col = 0; col < 4; ++col) {
                            buf(i, row, col) = mats[i](row, col);
                        }
                    }
                }
                return arr;
            },
            py::arg("type"),
            R"pbdoc(
                Retrieve poses as a NumPy array of 4×4 transformation matrices.

                Args:
                    type (SamplingMode): Sampling strategy to use.
                        - SamplingMode.KEY_FRAMES: Returns poses at key-frame timestamps.
                        - SamplingMode.COLUMNS: Returns poses of every lidarscan's columns.

                Returns:
                    numpy.ndarray[np.float64]: An (n, 4, 4) array of 4×4 poses.
            )pbdoc")

        .def("get_key_frame_distance", &PoseOptimizer::get_key_frame_distance,
             R"pbdoc(
                Return the configured key-frame distance (meters) used when constructing the trajectory.
             )pbdoc")

        .def("save", &PoseOptimizer::save, py::arg("osf_filename"),
             R"pbdoc(
                 Save the optimized trajectory to an OSF file.

                 This method writes the current state of the optimized trajectory to a new OSF file,
                 preserving all other data from the original file.

                 Args:
                     osf_filename (str): The name of the output OSF file.

                 Returns:
                     bool: True if the file was successfully saved, False otherwise.
             )pbdoc")

        .def(
            "get_constraints",
            [](const PoseOptimizer& self) {
                auto constraints = self.get_constraints();
                py::list result;
                for (const auto& constraint : constraints) {
                    // Clone each constraint to prevents dangling pointers
                    auto cloned = constraint->clone();
                    result.append(
                        py::cast(cloned.release(),
                                 py::return_value_policy::take_ownership));
                }
                return result;
            },
            R"pbdoc(
                 Get all constraints currently configured in the pose optimizer.

                 This method returns a copy of all constraints that are currently
                 configured in the pose optimizer, including both constraints loaded
                 from JSON files during construction and constraints added later via
                 add_constraint().

                 Returns:
                     List[Constraint]: A list of Constraint objects representing all currently
                     configured constraints.
             )pbdoc")

        .def(
            "set_constraints",
            [](PoseOptimizer& self, py::list constraints) {
                std::vector<std::unique_ptr<Constraint>> constraint_vec;
                for (auto item : constraints) {
                    auto* constraint_ptr = item.cast<Constraint*>();
                    constraint_vec.push_back(constraint_ptr->clone());
                }
                self.set_constraints(std::move(constraint_vec));
            },
            py::arg("constraints"),
            R"pbdoc(
                 Set all constraints for the pose optimizer.

                 This method replaces all existing constraints with the provided set
                 of constraints. Any constraints previously loaded from JSON files or
                 added via add_constraint() will be removed and replaced with the new
                 constraint set.

                 Args:
                     constraints (List[Constraint]): A list of Constraint objects to set as the
                     complete constraint set.

                 Raises:
                     RuntimeError: If the constraints cannot be set.
             )pbdoc");

    module.def(
        "save_trajectory",
        [](const std::string& filename,
           py::array_t<uint64_t, py::array::c_style | py::array::forcecast>
               ts_arr,
           py::array_t<double, py::array::c_style | py::array::forcecast>
               poses_arr,
           const std::string& file_type) {
            auto ts_buf = ts_arr.unchecked<1>();
            py::ssize_t n_ts = ts_buf.shape(0);
            std::vector<uint64_t> timestamps;
            timestamps.reserve(static_cast<size_t>(n_ts));
            for (py::ssize_t i = 0; i < n_ts; ++i) {
                timestamps.push_back(ts_buf(i));
            }

            if (poses_arr.ndim() != 3 || poses_arr.shape(0) != n_ts ||
                poses_arr.shape(1) != 4 || poses_arr.shape(2) != 4) {
                throw std::runtime_error(
                    "poses must be a (n,4,4) array with n == "
                    "timestamps.size()");
            }
            auto p_buf = poses_arr.unchecked<3>();
            std::vector<Eigen::Matrix<double, 4, 4>> poses;
            poses.reserve(static_cast<size_t>(n_ts));
            for (py::ssize_t i = 0; i < n_ts; ++i) {
                Eigen::Matrix<double, 4, 4> matrix;
                for (int row = 0; row < 4; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        matrix(row, col) = p_buf(i, row, col);
                    }
                }
                poses.push_back(matrix);
            }

            save_trajectory(filename, timestamps, poses, file_type);
        },
        py::arg("filename"), py::arg("timestamps"), py::arg("poses"),
        py::arg("file_type") = "csv");

    py::class_<SlamConfig>(module, "SlamConfig", R"pbdoc(
        Configuration options for the SLAM engine.
    )pbdoc")
        .def(py::init<>(), "...")
        .def_readwrite("min_range", &SlamConfig::min_range)
        .def_readwrite("max_range", &SlamConfig::max_range)
        .def_readwrite("voxel_size", &SlamConfig::voxel_size)
        .def_readwrite("initial_pose", &SlamConfig::initial_pose)
        .def_readwrite("backend", &SlamConfig::backend)
        .def_readwrite("deskew_method", &SlamConfig::deskew_method);

    py::class_<SlamEngine>(module, "SlamEngine", R"pbdoc(
        The SLAM engine for processing LiDAR scans and computing poses.
    )pbdoc")
        .def(py::init<const std::vector<std::shared_ptr<SensorInfo>>&,
                      const SlamConfig&>(),
             py::arg("infos"), py::arg("config"),
             R"pbdoc(
                SlamEngine constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
                    config (SlamConfig): Configuration options for the SLAM engine.
             )pbdoc")
        .def(
            "update",
            [](SlamEngine& self, LidarScanSet& scans) -> LidarScanSet& {
                self.update(scans);
                return scans;
            },
            py::arg("scans"),
            R"pbdoc(
                Update the pose (per_column_global_pose) variable in scan and return

                Args:
                    scans (List[LidarScan]): List of scans to update with the latest pose.

                Returns:
                    List[LidarScan]: The updated scans with per_column_global_pose set.
            )pbdoc")
        .def("get_point_cloud", &SlamEngine::get_point_cloud,
             R"pbdoc(
                Get the current point cloud from the SLAM engine.

                Returns:
                    Nx3: The point cloud generated by the SLAM engine.
            )pbdoc");

    py::class_<LocalizationConfig>(module, "LocalizationConfig", R"pbdoc(
        Configuration options for the Localization engine.
    )pbdoc")
        .def(py::init<>(), "...")
        .def_readwrite("min_range", &LocalizationConfig::min_range)
        .def_readwrite("max_range", &LocalizationConfig::max_range)
        .def_readwrite("voxel_size", &LocalizationConfig::voxel_size)
        .def_readwrite("initial_pose", &LocalizationConfig::initial_pose)
        .def_readwrite("backend", &LocalizationConfig::backend)
        .def_readwrite("deskew_method", &LocalizationConfig::deskew_method);

    py::class_<LocalizationEngine>(module, "LocalizationEngine", R"pbdoc(
        The Localization engine for processing LiDAR scans and computing poses based
        on prebuilt pointcloud map.
    )pbdoc")
        .def(py::init<const std::vector<std::shared_ptr<SensorInfo>>&,
                      const LocalizationConfig&, const std::string&>(),
             py::arg("infos"), py::arg("config"), py::arg("map"),
             R"pbdoc(
                LocalizationEngine constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
                    config (LocalizationConfig): Configuration options for the Localization engine.
                    map_path (string): Path to the point cloud map.
             )pbdoc")
        .def(
            "update",
            [](LocalizationEngine& self, LidarScanSet& scans) -> LidarScanSet& {
                self.update(scans);
                return scans;
            },
            py::arg("scans"),
            R"pbdoc(
                Update the pose (per_column_global_pose) variable in scan and return

                Args:
                    scans (List[LidarScan]): List of scans to update with the latest pose.

                Returns:
                    List[LidarScan]: The updated scans with per_column_global_pose set
            )pbdoc");

    py::class_<DeskewMethod, PyDeskewMethod>(module, "DeskewMethod", R"pbdoc(
        Base class for all deskewing methods.

        This is the abstract base class for all deskewing methods. Use specific
        derived classes like ConstantVelocityDeskewMethod instead of using this
        class directly.
    )pbdoc")
        .def(py::init<const std::vector<std::shared_ptr<SensorInfo>>&>(),
             py::arg("infos"),
             R"pbdoc(
                ConstantVelocityDeskewMethod constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             )pbdoc")
        .def(
            "update",
            [](DeskewMethod& self, LidarScanSet& scans) -> LidarScanSet& {
                self.update(scans);
                return scans;
            },
            py::arg("scans"),
            R"pbdoc(
                Update the pose (per_column_global_pose) variable in scan.

                Args:
                    scans (LidarScanSet): a LidarScanSet.

                Returns:
                    LidarScanSet: The updated lidar scans with per_column_global_pose set
            )pbdoc");

    py::class_<ConstantVelocityDeskewMethod, DeskewMethod>(
        module, "ConstantVelocityDeskewMethod", R"pbdoc(
        Deskew method that assumes constant velocity motion between poses.
    )pbdoc")
        .def(py::init<const std::vector<std::shared_ptr<SensorInfo>>&>(),
             py::arg("infos"),
             R"pbdoc(
                ConstantVelocityDeskewMethod constructor.

                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             )pbdoc")
        .def(
            "set_last_pose",
            [](ConstantVelocityDeskewMethod& self, uint64_t ts,
               const py::array_t<double, py::array::c_style |
                                             py::array::forcecast>& pose_arr) {
                if (pose_arr.ndim() != 2 || pose_arr.shape(0) != 4 ||
                    pose_arr.shape(1) != 4) {
                    throw std::runtime_error(
                        "pose must be a (4,4) array representing a "
                        "transformation matrix");
                }
                auto buf = pose_arr.unchecked<2>();
                Eigen::Matrix<double, 4, 4> pose;
                for (int row = 0; row < 4; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        pose(row, col) = buf(row, col);
                    }
                }
                self.set_last_pose(ts, pose);
            },
            py::arg("ts"), py::arg("pose"),
            R"pbdoc(
                Set the current pose to use for deskewing.

                This method allows setting the current pose that will be used
                as a reference for deskewing incoming scans. The current pose
                should represent the sensor's pose at the time of the most recent
                scan.

                Args:
                    ts (int): The timestamp (nanoseconds) associated with the pose.
                    pose (numpy.ndarray): A 4x4 transformation matrix representing
                        the current pose.
            )pbdoc");

    py::class_<DeskewMethodFactory>(module, "DeskewMethodFactory", R"pbdoc(
            Factory class for creating DeskewMethod instances.
        )pbdoc")
        .def_static(
            "create",
            [&](const std::string& method_name,
                const std::vector<std::shared_ptr<SensorInfo>>& infos)
                -> std::shared_ptr<DeskewMethod> {
                return DeskewMethodFactory::create(method_name, infos);
            },
            py::arg("method_name"), py::arg("infos"),
            R"pbdoc(
                    Create a DeskewMethod instance based on the specified method name.

                    Args:
                        method_name (str): The name of the deskew method to create.
                        infos (List[SensorInfo]): List of sensor info objects for each
                            sensor in the system.

                    Returns:
                        DeskewMethod: A new instance of the specified deskew method.
                 )pbdoc");

    py::class_<ActiveTimeCorrection>(module, "ActiveTimeCorrection", R"pbdoc(
        Class for correcting timestamps of LiDAR scans based on active time correction.
    )pbdoc")
        .def(py::init<const std::vector<std::shared_ptr<SensorInfo>>&>(),
             py::arg("infos"),
             R"pbdoc(
                ActiveTimeCorrection constructor.
                Args:
                    infos (List[SensorInfo]): List of sensor info objects for each
                        sensor in the system.
             )pbdoc")
        .def(
            "update",
            [](ActiveTimeCorrection& self,
               LidarScanSet& scans) -> LidarScanSet& {
                self.update(scans);
                return scans;
            },
            py::arg("scans"),
            R"pbdoc(
                Update the timestamps in the provided LidarScanSet using active time correction.
            )pbdoc")
        .def(
            "reset",
            [](ActiveTimeCorrection& self,
               LidarScanSet& scans) -> LidarScanSet& {
                self.reset(scans);
                return scans;
            },
            py::arg("scans"),
            R"pbdoc(
                Restore the timestamps in the provided LidarScanSet to their original values before active time correction was applied.
            )pbdoc");
}

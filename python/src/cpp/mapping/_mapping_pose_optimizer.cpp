/*
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * Bindings for PoseOptimizer.
 * Extracted from _mapping.cpp to allow parallel compilation.
 */

#include <nanobind/stl/unique_ptr.h>

#include <cstring>
#include <functional>
#include <memory>

#include "_mapping.h"
#include "ouster/mapping/pose_optimizer.h"
#include "ouster/mapping/pose_optimizer_constraint.h"
#include "ouster/mapping/pose_optimizer_node.h"

using namespace ouster::sdk::mapping;
using namespace ouster::sdk::core;
namespace {

class PyPoseOptimizer : public PoseOptimizer {
   public:
    using PoseOptimizer::PoseOptimizer;

    py::handle solver_step_cb;
};

int pose_optimizer_tp_traverse(PyObject* self, visitproc visit, void* arg) {
    Py_VISIT(Py_TYPE(self));
    if (!py::inst_ready(self)) {
        return 0;
    }
    auto w = py::inst_ptr<PyPoseOptimizer>(self);
    if (w->solver_step_cb.ptr() != nullptr) {
        Py_VISIT(w->solver_step_cb.ptr());
    }
    return 0;
}

int pose_optimizer_tp_clear(PyObject* self) {
    auto w = py::inst_ptr<PyPoseOptimizer>(self);
    if (w->solver_step_cb.ptr() != nullptr) {
        w->set_solver_step_callback(std::function<void()>());
        w->solver_step_cb = py::handle();
    }
    return 0;
}

PyType_Slot pose_optimizer_slots[] = {
    {Py_tp_traverse, reinterpret_cast<void*>(pose_optimizer_tp_traverse)},
    {Py_tp_clear, reinterpret_cast<void*>(pose_optimizer_tp_clear)},
    {0, nullptr},
};

}  // namespace

void init_mapping_pose_optimizer(py::module_& module) {
    py::class_<PyPoseOptimizer>(module, "PoseOptimizer", py::type_slots(pose_optimizer_slots),
                                R"pbdoc(
        A class for optimizing LiDAR sensor trajectories using various geometric constraints.

        This class allows adding different types of constraints (pose-to-pose, absolute pose, point-to-point)
        and solving the trajectory optimization problem to generate a more accurate and consistent sensor path.
        The optimization aims to minimize the error across all defined constraints while maintaining
        a physically plausible trajectory.
    )pbdoc")
        .def(py::init<const std::string&, const SolverConfig&>(), py::arg("osf_filename"),
             py::arg("options"),
             R"pbdoc(
                 Initialize PoseOptimizer with an OSF file and solver options.

                 Args:
                     osf_filename (str): Path to the OSF file containing trajectory data.
                     options (SolverConfig): Solver configuration options. Set options.fix_first_node to True to fix the first node.
                 )pbdoc")
        .def(py::init<const std::string&, const std::string&>(), py::arg("osf_filename"),
             py::arg("config_filename"),
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
        .def(
            "add_constraint",
            [](PyPoseOptimizer& self, const py::object& constraint_obj) -> uint32_t {
                std::unique_ptr<Constraint> constraint_ptr;
                if (py::isinstance<AbsolutePoseConstraint>(constraint_obj)) {
                    auto& constraint = py::cast<AbsolutePoseConstraint&>(constraint_obj);
                    constraint_ptr = std::make_unique<AbsolutePoseConstraint>(constraint);
                } else if (py::isinstance<PoseToPoseConstraint>(constraint_obj)) {
                    auto& constraint = py::cast<PoseToPoseConstraint&>(constraint_obj);
                    constraint_ptr = std::make_unique<PoseToPoseConstraint>(constraint);
                } else if (py::isinstance<PointToPointConstraint>(constraint_obj)) {
                    auto& constraint = py::cast<PointToPointConstraint&>(constraint_obj);
                    constraint_ptr = std::make_unique<PointToPointConstraint>(constraint);
                } else if (py::isinstance<AbsolutePointConstraint>(constraint_obj)) {
                    auto& constraint = py::cast<AbsolutePointConstraint&>(constraint_obj);
                    constraint_ptr = std::make_unique<AbsolutePointConstraint>(constraint);
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
        .def(
            "add_absolute_gps_constraints",
            [](PyPoseOptimizer& self, double min_space_m,
               const py::ndarray<py::numpy, py::ndim<1>, const double, py::c_contig>&
                   translation_weight) {
                if (translation_weight.shape(0) != 3) {
                    throw std::invalid_argument("translation_weight must be a length-3 array");
                }
                auto w = translation_weight.view();
                Eigen::Array3d weights;
                weights << w(0), w(1), w(2);
                return self.add_absolute_gps_constraints(min_space_m, weights);
            },
            py::arg("min_space_m"), py::arg("translation_weight"),
            R"pbdoc(
               Generate and add GPS-derived absolute pose constraints.

               Reads POSITION_LAT_LONG and POSITION_TIMESTAMP fields from the
               source OSF and adds AbsolutePoseConstraints roughly every
               `min_space_m` meters of travel (estimated from frame poses). XY is
               constrained in a local WGS84 linearization; Z is copied from the
               frame pose when available, otherwise unconstrained. The Z weight
               is set to 0 when the frame pose is identity.

               Args:
                   min_space_m (float): Minimum spacing between constraints in meters.
                   translation_weight (numpy.ndarray): Translation weights (x, y, z).

               Returns:
                   int: Number of constraints added.
               )pbdoc")
        .def("remove_constraint", &PyPoseOptimizer::remove_constraint, py::arg("constraint_id"),
             R"pbdoc(
               Remove a constraint from the pose optimization problem.

               Args:
                   constraint_id (int): The unique ID of the constraint to remove.

               Raises:
                   RuntimeError: If the constraint ID is not found.
               )pbdoc")
        .def("initialize_trajectory_alignment", &PyPoseOptimizer::initialize_trajectory_alignment,
             R"pbdoc(
                Initialize trajectory alignment using average absolute constraints.

                Computes a weighted average SE(3) transform from the currently
                loaded absolute pose and absolute point constraints (using their
                weights in Lie algebra space) and left-multiplies the entire
                trajectory by that transform as an initial alignment step before
                optimization.

                Returns:
                    np.ndarray: The applied 4x4 alignment transform. Returns
                        identity if skipped (e.g. no absolute constraints or a
                        negligible delta).
            )pbdoc")
        .def("solve", &PyPoseOptimizer::solve, py::arg("steps") = 0,
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
            [](PyPoseOptimizer& self, py::callable callback) {
                self.solver_step_cb = callback;
                self.set_solver_step_callback([cb_copy = std::move(callback)]() {
                    py::gil_scoped_acquire acquire;
                    try {
                        cb_copy();
                    } catch (py::python_error& e) {
                        e.discard_as_unraisable("solver_step_callback");
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
        .def("get_cost_value", &PyPoseOptimizer::get_cost_value,
             R"pbdoc(
                Get the last solver cost value (final cost from the last solve()).

                Returns:
                    float: The last recorded solver cost value.
            )pbdoc")
        .def("save_config", &PyPoseOptimizer::save_config, py::arg("config_filename"),
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
        .def("get_total_iterations", &PyPoseOptimizer::get_total_iterations,
             R"pbdoc(
                Get the cumulative number of solver iterations executed so far.

                Returns:
                    int: Total iterations across all calls to solve().
            )pbdoc")
        .def(
            "get_sampled_nodes",
            [](PyPoseOptimizer& self, size_t count) {
                auto nodes = self.get_sampled_nodes(count);
                py::list out;
                for (auto& node : nodes) {
                    out.append(node);
                }
                return out;
            },
            py::arg("count") = 100,
            R"pbdoc(
               Retrieve up to `count` frame nodes evenly sampled across the OSF.

               Each node is guaranteed to have a downsampled point cloud; nodes
               are created on-demand if necessary.
            )pbdoc")
        .def(
            "get_node",
            [](const PyPoseOptimizer& self, uint64_t timestamp) {
                return self.get_node(timestamp);
            },
            py::arg("timestamp"),
            R"pbdoc(
               Get the node associated with a given timestamp (first-valid-column ts).
               Returns None if not found.
            )pbdoc")
        .def(
            "get_timestamps",
            [](const PyPoseOptimizer& self, SamplingMode type) {
                std::vector<uint64_t> vec = self.get_timestamps(type);
                auto arr = make_array<uint64_t, 1>({static_cast<size_t>(vec.size())});
                std::memcpy(arr.data(), vec.data(), vec.size() * sizeof(uint64_t));
                return arr;
            },
            py::arg("type"),
            R"pbdoc(
                Retrieve timestamps corresponding to the selected sampling mode.

                Args:
                    type (SamplingMode): Sampling strategy to use.
                        - SamplingMode.KEY_FRAMES: Returns timestamps at key-frame poses.
                        - SamplingMode.COLUMNS: Returns timestamps of every LidarFrame's columns.

                Returns:
                    numpy.ndarray[np.uint64]: A 1D array of timestamps (nanoseconds).
            )pbdoc")
        .def(
            "get_poses",
            [](PyPoseOptimizer& self, SamplingMode type) {
                auto mats = self.get_poses(type);
                size_t num_poses = mats.size();
                auto arr = make_array<double, 3>({num_poses, 4, 4});
                auto buf = arr.view();
                for (size_t i = 0; i < num_poses; ++i) {
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
                Retrieve poses as a NumPy array of 4x4 transformation matrices.

                Args:
                    type (SamplingMode): Sampling strategy to use.
                        - SamplingMode.KEY_FRAMES: Returns poses at key-frame timestamps.
                        - SamplingMode.COLUMNS: Returns poses of every LidarFrame's columns.

                Returns:
                    numpy.ndarray[np.float64]: An (n, 4, 4) array of 4x4 poses.
            )pbdoc")
        .def("get_key_frame_distance", &PyPoseOptimizer::get_key_frame_distance,
             R"pbdoc(
                Return the configured key-frame distance (meters) used when constructing the trajectory.
             )pbdoc")
        .def("add_relative_loop_constraints", &PyPoseOptimizer::add_relative_loop_constraints,
             py::arg("min_distance_m"), py::arg("cell_size_m") = 1.0,
             py::arg("icp_score_threshold") = 0.0,
             R"pbdoc(
                Automatically add relative loop-closure constraints using a spatial hash detector.

                Args:
                    min_distance_m (float): Minimum traveled distance (meters)
                        required between successive loop-closure additions.
                    cell_size_m (float): Spatial hash grid cell size (meters).
                    icp_score_threshold (float): Minimum auto-ICP confidence in [0, 1]
                        required to keep a detected loop pair. Use 0 to disable filtering.

                Returns:
                    int: Number of loop-closure constraints added.
             )pbdoc")
        .def("save", &PyPoseOptimizer::save, py::arg("osf_filename"),
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
            [](const PyPoseOptimizer& self) {
                auto constraints = self.get_constraints();
                py::list result;
                for (const auto& constraint : constraints) {
                    auto cloned = constraint->clone();
                    result.append(py::cast(cloned.release(), py::rv_policy::take_ownership));
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
            [](PyPoseOptimizer& self, const py::list& constraints) {
                std::vector<std::unique_ptr<Constraint>> constraint_vec;
                for (auto item : constraints) {
                    auto* constraint_ptr = py::cast<Constraint*>(item);
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
}

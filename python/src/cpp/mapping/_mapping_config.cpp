/*
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * Bindings for SolverConfig, SamplingMode, LossFunction, and PoseOptimizerNode.
 * Extracted from _mapping.cpp to allow parallel compilation.
 */

#include "_mapping.h"
#include "ouster/mapping/pose_optimizer.h"
#include "ouster/mapping/pose_optimizer_node.h"

using namespace ouster::sdk::mapping;
using namespace ouster::sdk::core;
void init_mapping_config(py::module_& module) {
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
        .def_rw("key_frame_distance", &SolverConfig::key_frame_distance)
        .def_rw("traj_rotation_weight", &SolverConfig::traj_rotation_weight)
        .def_rw("traj_translation_weight", &SolverConfig::traj_translation_weight)
        .def_rw("max_num_iterations", &SolverConfig::max_num_iterations)
        .def_rw("function_tolerance", &SolverConfig::function_tolerance)
        .def_rw("gradient_tolerance", &SolverConfig::gradient_tolerance)
        .def_rw("parameter_tolerance", &SolverConfig::parameter_tolerance)
        .def_rw("process_printout", &SolverConfig::process_printout)
        .def_rw("loss_function", &SolverConfig::loss_function)
        .def_rw("loss_scale", &SolverConfig::loss_scale)
        .def_rw("fix_first_node", &SolverConfig::fix_first_node);

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

    py::class_<Node>(module, "PoseOptimizerNode")
        .def_prop_ro("ts", [](const Node& node) { return node.ts; })
        .def_prop_ro("downsampled_pts", [](const Node& node) { return node.downsampled_pts; })
        .def_prop_ro("ptp_constraint_pt", [](const Node& node) { return node.ptp_constraint_pt; })
        .def_prop_ro("ap_constraint_pt", [](const Node& node) { return node.ap_constraint_pt; })
        .def("get_pose", [](Node& node) { return node.get_pose(); });
}

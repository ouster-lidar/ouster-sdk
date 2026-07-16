/*
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * Entry point for mapping bindings. The larger binding groups live in
 * separate translation units:
 *   _mapping_config.cpp          – SolverConfig, SamplingMode, LossFunction,
 *                                  PoseOptimizerNode
 *   _mapping_constraints.cpp     – all Constraint classes
 *   _mapping_pose_optimizer.cpp  – PoseOptimizer
 *   _mapping_slam.cpp            – SlamEngine, LocalizationEngine, DeskewMethod
 *                                  variants, ActiveTimeCorrection
 *   _mapping_registration.cpp    – ICPRegistration, AdaptiveThreshold
 */

#include "_mapping.h"

#include <nanobind/stl/string.h>

#include <cstdint>
#include <string>
#include <vector>

#include "ouster/mapping/pose_optimizer.h"

using ouster::sdk::core::Matrix4dR;
using ouster::sdk::mapping::save_trajectory;

void init_mapping(py::module_& module, py::module_& /*root_m*/) {
    module.doc() =
        "Python bindings for Ouster mapping: SLAM, localization, trajectory "
        "optimization, and frame-to-map ICP registration.";

    init_mapping_config(module);
    init_mapping_constraints(module);
    init_mapping_pose_optimizer(module);
    module.def(
        "save_trajectory",
        [](const std::string& filename,
           const py::ndarray<py::numpy, py::ndim<1>, const uint64_t, py::c_contig>& ts_arr,
           const py::ndarray<py::numpy, py::ndim<3>, const double, py::c_contig>& poses_arr,
           const std::string& file_type) {
            auto ts_buf = ts_arr.view();
            const size_t n_ts = ts_buf.shape(0);
            std::vector<uint64_t> timestamps;
            timestamps.reserve(n_ts);
            for (size_t i = 0; i < n_ts; ++i) {
                timestamps.push_back(ts_buf(i));
            }
            if (poses_arr.ndim() != 3 || poses_arr.shape(0) != n_ts || poses_arr.shape(1) != 4 ||
                poses_arr.shape(2) != 4) {
                throw std::runtime_error(
                    "poses must be a (n,4,4) array with n == "
                    "timestamps.size()");
            }
            auto p_buf = poses_arr.view();
            std::vector<Matrix4dR> poses;
            poses.reserve(n_ts);
            for (size_t i = 0; i < n_ts; ++i) {
                Matrix4dR matrix;
                for (int row = 0; row < 4; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        matrix(row, col) = p_buf(i, row, col);
                    }
                }
                poses.push_back(matrix);
            }
            save_trajectory(filename, timestamps, poses, file_type);
        },
        py::arg("filename"), py::arg("timestamps"), py::arg("poses"), py::arg("file_type") = "csv");
    init_mapping_registration(module);
    init_mapping_slam(module);
}

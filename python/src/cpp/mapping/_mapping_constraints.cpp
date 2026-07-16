/*
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * Bindings for all Constraint classes.
 * Extracted from _mapping.cpp to allow parallel compilation.
 */

#include "_mapping.h"
#include "ouster/mapping/pose_optimizer_constraint.h"

using namespace ouster::sdk::mapping;
using namespace ouster::sdk::core;
void init_mapping_constraints(py::module_& module) {
    py::class_<Constraint>(module, "Constraint", R"pbdoc(
        Base class for all pose optimization constraints.

        This is the abstract base class for all constraints used in pose optimization.
        Use the specific constraint classes like AbsolutePoseConstraint, PoseToPoseConstraint,
        or PointToPointConstraint instead of using this class directly.
    )pbdoc")
        .def_rw("translation_weights", &Constraint::translation_weights)
        .def("get_constraint_id", &Constraint::get_constraint_id, R"pbdoc(
            Get the unique constraint ID. Returns 0 for non-user constraints.
            IDs are assigned when constraint objects are constructed.

            Returns:
                int: The constraint ID, or 0 if not a user-added constraint.
        )pbdoc");

    py::class_<AbsolutePoseConstraint, Constraint>(module, "AbsolutePoseConstraint",
                                                   R"pbdoc(
        Absolute pose constraint - fixes a pose at a specific timestamp.

        This constraint type enforces that the sensor pose at a given timestamp
        matches a specific target pose.
    )pbdoc")
        .def(py::init<>(), "Default constructor")
        .def(py::init<uint64_t, const Matrix4dR&, double, const Eigen::Array3d&>(),
             py::arg("timestamp"), py::arg("pose"), py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = Eigen::Array3d::Ones(),
             py::sig("def __init__(self, timestamp: numpy.uint64, pose: "
                     "numpy.ndarray, rotation_weight: float = 1.0, "
                     "translation_weight: numpy.ndarray = ...) -> None"),
             R"pbdoc(
                              Constructor for AbsolutePoseConstraint.

               Args:
                   timestamp (int): Timestamp of the pose to constrain (nanoseconds)
                   pose: The 4x4 transformation matrix (SE3) to constrain to
                   rotation_weight: Scalar weight applied to the quaternion axis-alignment residual
                   translation_weight: Weight for translation constraints (x, y, z)
               )pbdoc")
        .def_rw("timestamp", &AbsolutePoseConstraint::timestamp)
        .def_rw("pose", &AbsolutePoseConstraint::pose)
        .def_rw("rotation_weight", &AbsolutePoseConstraint::rotation_weight)
        .def_rw("translation_weights", &AbsolutePoseConstraint::translation_weights);

    py::class_<PoseToPoseConstraint, Constraint>(module, "PoseToPoseConstraint",
                                                 R"pbdoc(
        Relative pose-to-pose constraint - enforces relative transformation between two poses.

        This constraint type enforces a specific relative transformation between
        two poses at different timestamps.
    )pbdoc")
        .def(py::init<>(), "Default constructor")
        .def(py::init<uint64_t, uint64_t, const Matrix4dR&, double, const Eigen::Array3d&>(),
             py::arg("timestamp1"), py::arg("timestamp2"),
             py::arg("relative_pose") = Matrix4dR::Identity(), py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = Eigen::Array3d::Ones(),
             py::sig("def __init__(self, timestamp1: numpy.uint64, timestamp2: "
                     "numpy.uint64, relative_pose: numpy.ndarray = ..., "
                     "rotation_weight: float = 1.0, translation_weight: "
                     "numpy.ndarray = ...) -> None"),
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
        .def_rw("timestamp1", &PoseToPoseConstraint::timestamp1)
        .def_rw("timestamp2", &PoseToPoseConstraint::timestamp2)
        .def_rw("relative_pose", &PoseToPoseConstraint::relative_pose)
        .def_rw("rotation_weight", &PoseToPoseConstraint::rotation_weight)
        .def_rw("translation_weights", &PoseToPoseConstraint::translation_weights);

    py::class_<PointToPointConstraint, Constraint>(module, "PointToPointConstraint",
                                                   R"pbdoc(
        Point-to-point constraint - enforces correspondence between points.

        This constraint type enforces that specific points in two different
        lidar frames correspond to the same physical location.
    )pbdoc")
        .def(py::init<>(), "Default constructor")
        .def(py::init<uint64_t, uint32_t, uint32_t, uint32_t, uint64_t, uint32_t, uint32_t,
                      uint32_t, const Eigen::Array3d&>(),
             py::arg("timestamp1"), py::arg("row1"), py::arg("col1"), py::arg("return_idx1"),
             py::arg("timestamp2"), py::arg("row2"), py::arg("col2"), py::arg("return_idx2"),
             py::arg("translation_weight") = Eigen::Array3d::Ones(),
             py::sig("def __init__(self, timestamp1: numpy.uint64, row1: int, "
                     "col1: int, return_idx1: int, timestamp2: numpy.uint64, "
                     "row2: int, col2: int, return_idx2: int, "
                     "translation_weight: numpy.ndarray = ...) -> None"),
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
        .def_rw("timestamp1", &PointToPointConstraint::timestamp1)
        .def_rw("timestamp2", &PointToPointConstraint::timestamp2)
        .def_rw("row1", &PointToPointConstraint::row1)
        .def_rw("col1", &PointToPointConstraint::col1)
        .def_rw("return_idx1", &PointToPointConstraint::return_idx1)
        .def_rw("row2", &PointToPointConstraint::row2)
        .def_rw("col2", &PointToPointConstraint::col2)
        .def_rw("return_idx2", &PointToPointConstraint::return_idx2)
        .def_rw("translation_weights", &PointToPointConstraint::translation_weights);

    py::class_<AbsolutePointConstraint, Constraint>(module, "AbsolutePointConstraint",
                                                    R"pbdoc(
        Absolute point constraint.

        Constrains a single 3D point from a LiDAR frame, identified by its 2D
        image coordinates (row, col) and return index at a given timestamp, to
        match a user-defined absolute 3D position in the world frame. The 3D
        point is computed the same way as in PointToPointConstraint (via RANGE/
        RANGE2 and the XYZ LUT), but is compared to a provided global point
        instead of another frame point.
    )pbdoc")
        .def(py::init<>(), "Default constructor")
        .def(py::init<uint64_t, uint32_t, uint32_t, uint32_t, const Eigen::Vector3d&,
                      const Eigen::Array3d&>(),
             py::arg("timestamp"), py::arg("row"), py::arg("col"), py::arg("return_idx"),
             py::arg("absolute_position"), py::arg("translation_weight") = Eigen::Array3d::Ones(),
             py::sig("def __init__(self, timestamp: numpy.uint64, row: int, col: "
                     "int, return_idx: int, absolute_position: numpy.ndarray, "
                     "translation_weight: numpy.ndarray = ...) -> None"),
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
        .def_rw("timestamp", &AbsolutePointConstraint::timestamp)
        .def_rw("row", &AbsolutePointConstraint::row)
        .def_rw("col", &AbsolutePointConstraint::col)
        .def_rw("return_idx", &AbsolutePointConstraint::return_idx)
        .def_rw("absolute_position", &AbsolutePointConstraint::absolute_position)
        .def_rw("translation_weights", &AbsolutePointConstraint::translation_weights);
}

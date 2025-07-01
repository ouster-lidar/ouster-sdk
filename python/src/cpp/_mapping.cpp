#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/Dense>
#include <iostream>

#include "ouster/impl/preprocessing.h"
#include "ouster/pose_optimizer.h"
#include "ouster/pose_optimizer_enums.h"

namespace py = pybind11;

namespace pybind11 {

template <typename Vector, typename holder_type = std::unique_ptr<Vector>,
          typename... Args>
py::class_<Vector, holder_type> bind_vector_without_repr(
    py::module& m, std::string const& name, Args&&... args) {
    // hack function to disable __repr__ for the convenient function
    // bind_vector()
    using Class_ = py::class_<Vector, holder_type>;
    Class_ cl(m, name.c_str(), std::forward<Args>(args)...);
    cl.def(py::init<>());
    cl.def(
        "__bool__", [](const Vector& v) -> bool { return !v.empty(); },
        "Check whether the list is nonempty");
    cl.def("__len__", &Vector::size);
    return cl;
}

// - This function is used by Pybind for std::vector<SomeEigenType> constructor.
//   This optional constructor is added to avoid too many Python <-> C++ API
//   calls when the vector size is large using the default binding method.
//   Pybind matches np.float64 array to py::array_t<double> buffer.
// - Directly using templates for the py::array_t<double> and py::array_t<int>
//   and etc. doesn't work. The current solution is to explicitly implement
//   bindings for each py array types.
template <typename EigenVector>
std::vector<EigenVector> py_array_to_vectors_double(
    py::array_t<double, py::array::c_style | py::array::forcecast> array) {
    int64_t eigen_vector_size = EigenVector::SizeAtCompileTime;
    if (array.ndim() != 2 || array.shape(1) != eigen_vector_size) {
        throw py::cast_error();
    }
    std::vector<EigenVector> eigen_vectors(array.shape(0));
    auto array_unchecked = array.mutable_unchecked<2>();
    for (auto i = 0; i < array_unchecked.shape(0); ++i) {
        eigen_vectors[i] = Eigen::Map<EigenVector>(&array_unchecked(i, 0));
    }
    return eigen_vectors;
}

}  // namespace pybind11

template <typename EigenVector, typename Vector = std::vector<EigenVector>,
          typename holder_type = std::unique_ptr<Vector>, typename InitFunc>
py::class_<Vector, holder_type> pybind_eigen_vector_of_vector(
    py::module& m, const std::string& bind_name, const std::string& repr_name,
    InitFunc init_func) {
    using Scalar = typename EigenVector::Scalar;
    auto vec = py::bind_vector_without_repr<std::vector<EigenVector>>(
        m, bind_name, py::buffer_protocol(), py::module_local());
    vec.def(py::init(init_func));
    vec.def_buffer([](std::vector<EigenVector>& v) -> py::buffer_info {
        size_t rows = EigenVector::RowsAtCompileTime;
        return py::buffer_info(
            v.data(), sizeof(Scalar), py::format_descriptor<Scalar>::format(),
            2, {v.size(), rows}, {sizeof(EigenVector), sizeof(Scalar)});
    });
    vec.def("__repr__", [repr_name](const std::vector<EigenVector>& v) {
        return repr_name + std::string(" with ") + std::to_string(v.size()) +
               std::string(" elements.\n") +
               std::string("Use numpy.asarray() to access data.");
    });
    vec.def("__copy__", [](std::vector<EigenVector>& v) {
        return std::vector<EigenVector>(v);
    });
    vec.def("__deepcopy__", [](std::vector<EigenVector>& v) {
        return std::vector<EigenVector>(v);
    });

    // py::detail must be after custom constructor
    using Class_ = py::class_<Vector, std::unique_ptr<Vector>>;
    py::detail::vector_if_copy_constructible<Vector, Class_>(vec);
    py::detail::vector_if_equal_operator<Vector, Class_>(vec);
    py::detail::vector_modifiers<Vector, Class_>(vec);
    py::detail::vector_accessor<Vector, Class_>(vec);

    return vec;
}

using namespace ouster;
using namespace mapping;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

void init_mapping(py::module& m, py::module&) {
    auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
        m, "_Vector3dVector", "std::vector<Eigen::Vector3d>",
        py::py_array_to_vectors_double<Eigen::Vector3d>);

    m.doc() =
        "Python bindings for PoseOptimizer, enabling trajectory optimization "
        "using C++. This module provides tools for optimizing sensor "
        "trajectories "
        "by applying various constraints between poses or points in a LiDAR "
        "mapping context.";

    //----- Wrap SolverConfig -------------------------------------------
    py::class_<SolverConfig>(m, "SolverConfig", R"pbdoc(
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
	loss_function (str): The name of the robust loss function to use (e.g., "HuberLoss", "CauchyLoss", "SoftLOneLoss", "ArctanLoss", "TrivialLoss").
        loss_scale (float): The scaling parameter for the chosen loss function. Higher values make the loss less sensitive to outliers.
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
        .def_readwrite("loss_scale", &SolverConfig::loss_scale);

    py::enum_<SamplingMode>(m, "SamplingMode")
        .value("KEY_FRAMES", SamplingMode::KEY_FRAMES)
        .value("COLUMNS", SamplingMode::COLUMNS);

    py::enum_<LossFunction>(m, "LossFunction")
        .value("HuberLoss", LossFunction::HuberLoss)
        .value("CauchyLoss", LossFunction::CauchyLoss)
        .value("SoftLOneLoss", LossFunction::SoftLOneLoss)
        .value("ArctanLoss", LossFunction::ArctanLoss)
        .value("TrivialLoss", LossFunction::TrivialLoss)

        .def_static("from_string", &from_string, py::arg("name"),
                    R"pbdoc(
                Convert a string (e.g. "HuberLoss") to the corresponding LossFunction enum.
                Args:
		    name: one of "HuberLoss", "CauchyLoss", "SoftLOneLoss", "ArctanLoss", "TrivialLoss"
            )pbdoc");

    //----- Wrap PoseOptimizer -------------------------------------------
    py::class_<PoseOptimizer>(m, "PoseOptimizer", R"pbdoc(
        A class for optimizing LiDAR sensor trajectories using various geometric constraints.

        This class allows adding different types of constraints (pose-to-pose, absolute pose, point-to-point)
        and solving the trajectory optimization problem to generate a more accurate and consistent sensor path.
        The optimization aims to minimize the error across all defined constraints while maintaining
        a physically plausible trajectory.
    )pbdoc")
        // Constructors
        .def(py::init<const std::string&, const SolverConfig&, bool>(),
             py::arg("osf_filename"), py::arg("options"),
             py::arg("fix_first_node") = false,
             R"pbdoc(
             Initialize PoseOptimizer with an OSF file and solver options.

             Args:
                 osf_filename (str): Path to the OSF file containing trajectory data.
                 options (SolverConfig): Solver configuration options.
		 fix_first_node (bool, optional): Flag to fix the first node in the trajectory. Default is False.
             )pbdoc")

        .def(py::init<const std::string&, double, bool>(),
             py::arg("osf_filename"), py::arg("key_frame_distance"),
             py::arg("fix_first_node") = false,
             R"pbdoc(
             Initialize PoseOptimizer with an OSF file and a node gap.

             Args:
                 osf_filename (str): Path to the OSF file containing trajectory data.
                 key_frame_distance (float): The gap distance between nodes in the trajectory.
		 fix_first_node (bool, optional): Flag to fix the first node in the trajectory. Default is False.
             )pbdoc")

        .def("add_pose_to_pose_constraint",
             py::overload_cast<uint64_t, uint64_t,
                               const Eigen::Matrix<double, 6, 1>&, double,
                               double>(
                 &PoseOptimizer::add_pose_to_pose_constraint),
             py::arg("ts1"), py::arg("ts2"), py::arg("diff"),
             py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = 1.0,
             R"pbdoc(
               Add a relative pose constraint between two frames with a known transformation.
   
               This constraint specifies that the relative transformation between poses at timestamps ts1 and ts2
               should match the provided difference. Useful when you have externally computed transformations
               (e.g., from ICP or visual odometry).
   
               Args:
                   ts1 (int): Timestamp of the first frame.
                   ts2 (int): Timestamp of the second frame.
                   diff (Eigen::Matrix<double, 6, 1>): Pose difference between the two frames [rx,ry,rz,tx,ty,tz]
                   rotation_weight (float, optional): Weight for rotation constraints. Default is 1.0.
                   translation_weight (float, optional): Weight for translation constraints. Default is 1.0.
   
               Returns:
                   bool: True if the constraint was successfully added, False otherwise.
               )pbdoc")

        .def("add_pose_to_pose_constraint",
             py::overload_cast<uint64_t, uint64_t,
                               const Eigen::Matrix<double, 4, 4>&, double,
                               double>(
                 &PoseOptimizer::add_pose_to_pose_constraint),
             py::arg("ts1"), py::arg("ts2"), py::arg("diff"),
             py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = 1.0,
             R"pbdoc(
               Add a relative pose constraint between two frames with a known transformation in 4x4 matrix.
   
               This constraint specifies that the relative transformation between poses at timestamps ts1 and ts2
               should match the provided difference in 4x4 matrix form. Useful when you have externally computed transformations
               (e.g., from ICP or visual odometry).
   
               Args:
                   ts1 (int): Timestamp of the first frame.
                   ts2 (int): Timestamp of the second frame.
                   diff (Eigen::Matrix<double, 4, 4>): Pose difference between the two frames as a 4x4 transformation matrix.
                   rotation_weight (float, optional): Weight for rotation constraints. Default is 1.0.
                   translation_weight (float, optional): Weight for translation constraints. Default is 1.0.
   
               Returns:
                   bool: True if the constraint was successfully added, False otherwise.
               )pbdoc")

        .def("add_pose_to_pose_constraint",
             py::overload_cast<uint64_t, uint64_t, double, double>(
                 &PoseOptimizer::add_pose_to_pose_constraint),
             py::arg("ts1"), py::arg("ts2"), py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = 1.0,
             R"pbdoc(
               Add a relative pose constraint between two frames using automatic alignment.
   
               This overload automatically computes the transformation between frames at ts1 and ts2
               using the built-in ICP algorithm on the point clouds from these timestamps.
   
               Args:
                   ts1 (int): Timestamp of the first frame.
                   ts2 (int): Timestamp of the second frame.
                   rotation_weight (float, optional): Weight for rotation constraints. Default is 1.0.
                   translation_weight (float, optional): Weight for translation constraints. Default is 1.0.
   
               Returns:
                   bool: True if the constraint was successfully added, False otherwise.
               )pbdoc")

        .def("add_absolute_pose_constraint",
             py::overload_cast<uint64_t, const Eigen::Matrix<double, 6, 1>&,
                               double, double,
                               const Eigen::Matrix<double, 6, 1>&>(
                 &PoseOptimizer::add_absolute_pose_constraint),
             py::arg("ts"), py::arg("target_pose"),
             py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = 1.0,
             py::arg("diff") = Eigen::Matrix<double, 6, 1>::Zero(),
             R"pbdoc(
               Add a constraint fixing a pose to an absolute position and orientation.
   
               This constraint anchors a pose at a specific timestamp to a known fixed position/orientation
               in the global coordinate frame, such as from GPS or manual ground truth annotations.
   
               Args:
                   ts (int): Timestamp at which the absolute pose constraint is applied.
                   target_pose (Eigen::Matrix<double, 6, 1>): The target pose [rx,ry,rz,tx,ty,tz] in global coordinates.
                   rotation_weight (float, optional): Weight for rotation constraints. Default is 1.0.
                   translation_weight (float, optional): Weight for translation constraints. Default is 1.0.
		   diff (Eigen::Matrix<double, 6, 1>, optional): The expected difference (rotation and translation) between the source and target. Default is zero.
   
               Returns:
                   bool: True if the constraint was successfully added, False otherwise.
               )pbdoc")

        .def("add_absolute_pose_constraint",
             py::overload_cast<uint64_t, const Eigen::Matrix<double, 4, 4>&,
                               double, double,
                               const Eigen::Matrix<double, 4, 4>&>(
                 &PoseOptimizer::add_absolute_pose_constraint),
             py::arg("ts"), py::arg("target_pose"),
             py::arg("rotation_weight") = 1.0,
             py::arg("translation_weight") = 1.0,
             py::arg("diff") = Eigen::Matrix<double, 4, 4>::Identity(),
             R"pbdoc(
               Add a constraint fixing a pose to an absolute position and orientation with a 4x4 matrix.
   
               This constraint anchors a pose at a specific timestamp to a known fixed position/orientation
               in the global coordinate frame, such as from GPS or manual ground truth annotations,
               using a 4x4 transformation matrix.
   
               Args:
                   ts (int): Timestamp at which the absolute pose constraint is applied.
                   target_pose (Eigen::Matrix<double, 4, 4>): The target pose as a 4x4 transformation matrix in global coordinates.
                   rotation_weight (float, optional): Weight for rotation constraints. Default is 1.0.
                   translation_weight (float, optional): Weight for translation constraints. Default is 1.0.
		   diff (Eigen::Matrix<double, 4, 4>, optional): The expected difference (rotation and translation) between the source and target. Default is identity.
   
               Returns:
                   bool: True if the constraint was successfully added, False otherwise.
               )pbdoc")

        .def("add_absolute_pose_constraint",
             py::overload_cast<uint64_t, const Eigen::Matrix<double, 6, 1>&,
                               const std::array<double, 3>&,
                               const std::array<double, 3>&,
                               const Eigen::Matrix<double, 6, 1>&>(
                 &PoseOptimizer::add_absolute_pose_constraint),
             py::arg("ts"), py::arg("target_pose"),
             py::arg("rotation_weights") = std::array<double, 3>{1.0, 1.0, 1.0},
             py::arg("translation_weights") =
                 std::array<double, 3>{1.0, 1.0, 1.0},
             py::arg("diff") = Eigen::Matrix<double, 6, 1>::Zero(),
             R"pbdoc(
               Adds an absolute pose constraint with a predefined pose difference for a given timestamp (6x1 form).

               This function adds a constraint to enforce a specific pose (target_pose) at timestamp ts,
               weighted by per-axis rotation and translation weights. The pose difference can further
               adjust how the target pose is enforced.

               Args:
                   ts (int): Timestamp at which the absolute pose constraint is applied.
                   target_pose (Eigen::Matrix<double, 6, 1>): The target pose [rx, ry, rz, tx, ty, tz].
                   rotation_weights (List[float]): Rotational weight for each axis ([rx_weight, ry_weight, rz_weight]).
                   translation_weights (List[float]): Translational weight for each axis ([tx_weight, ty_weight, tz_weight]).
                   diff (Eigen::Matrix<double, 6, 1>, optional): The pose difference in 6x1 form. Default is zero.

               Returns:
                   bool: True if the constraint was successfully added, false otherwise.
             )pbdoc")

        .def("add_absolute_pose_constraint",
             py::overload_cast<uint64_t, const Eigen::Matrix<double, 4, 4>&,
                               const std::array<double, 3>&,
                               const std::array<double, 3>&,
                               const Eigen::Matrix<double, 4, 4>&>(
                 &PoseOptimizer::add_absolute_pose_constraint),
             py::arg("ts"), py::arg("target_pose"),
             py::arg("rotation_weights") = std::array<double, 3>{1.0, 1.0, 1.0},
             py::arg("translation_weights") =
                 std::array<double, 3>{1.0, 1.0, 1.0},
             py::arg("diff") = Eigen::Matrix<double, 4, 4>::Identity(),
             R"pbdoc(
                  Adds an absolute pose constraint with a predefined pose difference for a given timestamp (4x4 matrix form).
   
                  This function adds a constraint to enforce a specific pose (target_pose) at timestamp ts,
                  weighted by per-axis rotation and translation weights. The pose difference can further
                  adjust how the target pose is enforced using a 4x4 transformation matrix.
   
                  Args:
                      ts (int): Timestamp at which the absolute pose constraint is applied.
                      target_pose (Eigen::Matrix<double, 4, 4>): The target pose as a 4x4 transformation matrix.
                      rotation_weights (List[float]): Rotational weight for each axis ([rx_weight, ry_weight, rz_weight]).
                      translation_weights (List[float]): Translational weight for each direction ([tx_weight, ty_weight, tz_weight]).
                      diff (Eigen::Matrix<double, 4, 4>, optional): The pose difference in 4x4 form. Default is identity.
   
                  Returns:
                      bool: True if the constraint was successfully added, false otherwise.
                )pbdoc")

        // add_point_to_point_constraint
        .def("add_point_to_point_constraint",
             &PoseOptimizer::add_point_to_point_constraint, py::arg("ts1"),
             py::arg("row1"), py::arg("col1"), py::arg("return_idx1"),
             py::arg("ts2"), py::arg("row2"), py::arg("col2"),
             py::arg("return_idx2"), py::arg("translation_weight") = 1.0,
             R"pbdoc(
                Add a constraint between two specific points that should represent the same physical location.

                This constraint enforces that two LiDAR points from different frames, identified by their
                row/column coordinates, should map to the same physical location in the world after applying
                their respective pose transformations. Useful for manual feature correspondence.

                Args:
                    ts1 (int): Timestamp of the first frame.
                    row1 (int): Row index of the point in the first frame's 2D point representation.
                    col1 (int): Column index of the point in the first frame's 2D point representation.
                    return_idx1 (int): Lidar return index for multi-return sensors (0 for first return).
                    ts2 (int): Timestamp of the second frame.
                    row2 (int): Row index of the point in the second frame's 2D point representation.
                    col2 (int): Column index of the point in the second frame's 2D point representation.
                    return_idx2 (int): Lidar return index for multi-return sensors (0 for first return).
                    translation_weight (float, optional): Weight for this constraint. Default is 1.0.

                Returns:
                    bool: True if the constraint was successfully added, False otherwise.
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
         )pbdoc")

	.def("get_timestamps",
            [](const PoseOptimizer &self, SamplingMode type) {
	        std::vector<uint64_t> vec = self.get_timestamps(type);

                py::array_t<uint64_t> arr(vec.size());
                auto buf = arr.request();
                uint64_t *ptr = static_cast<uint64_t*>(buf.ptr);

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

	.def("get_poses",
             [](PoseOptimizer& self, SamplingMode type) {
	 	auto mats = self.get_poses(type);
         	py::ssize_t n = static_cast<py::ssize_t>(mats.size());

         	std::vector<py::ssize_t> shape = {n, 4, 4};
         	py::array_t<double> arr(shape);

         	auto buf = arr.mutable_unchecked<3>();
         	for (py::ssize_t i = 0; i < n; ++i) {
         	    for (int r = 0; r < 4; ++r) {
         	        for (int c = 0; c < 4; ++c) {
         	            buf(i, r, c) = mats[i](r, c);
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

        .def("save", &PoseOptimizer::save, py::arg("osf_name"),
             R"pbdoc(
                 Save the optimized trajectory to an OSF file.

                 This method writes the current state of the optimized trajectory to a new OSF file,
                 preserving all other data from the original file.

                 Args:
                     osf_name (str): The name of the output OSF file.

                 Returns:
                     bool: True if the file was successfully saved, False otherwise.
             )pbdoc");

    py::class_<Preprocessor> internal_preprocessor(m, "_Preprocessor",
                                                   "Don't use this");
    internal_preprocessor
        .def(py::init<double, double, bool, int>(), py::arg("max_range"),
             py::arg("min_range"), py::arg("deskew"),
             py::arg("max_num_threads"))
        .def(
            "_preprocess",
            [](Preprocessor& self, const std::vector<Eigen::Vector3d>& points,
               const std::vector<double>& timestamps,
               const Eigen::Matrix4d& relative_motion) {
                return self.Preprocess(points, timestamps, relative_motion);
            },
            py::arg("points"), py::arg("timestamps"),
            py::arg("relative_motion"));

    m.def(
        "save_trajectory",
        [](const std::string& filename,
           py::array_t<uint64_t, py::array::c_style | py::array::forcecast>
               ts_arr,
           py::array_t<double, py::array::c_style | py::array::forcecast>
               poses_arr,
           const std::string& file_type) -> bool {
            auto ts_buf = ts_arr.unchecked<1>();
            py::ssize_t n_ts = ts_buf.shape(0);
            std::vector<uint64_t> timestamps;
            timestamps.reserve((size_t)n_ts);
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
            poses.reserve((size_t)n_ts);
            for (py::ssize_t i = 0; i < n_ts; ++i) {
                Eigen::Matrix<double, 4, 4> M;
                for (int r = 0; r < 4; ++r) {
                    for (int c = 0; c < 4; ++c) M(r, c) = p_buf(i, r, c);
                }
                poses.push_back(M);
            }

            return save_trajectory(filename, timestamps, poses, file_type);
        },
        py::arg("filename"), py::arg("timestamps"), py::arg("poses"),
        py::arg("file_type") = "csv");
}

pose_util.h
===========

.. contents::
   :local:

Typedefs
---------
.. doxygentypedef:: ouster::core::Points
   :project: cpp_api

.. doxygentypedef:: ouster::core::Poses
   :project: cpp_api

.. doxygentypedef:: ouster::core::Pose
   :project: cpp_api

.. doxygentypedef:: ouster::core::PointsT
   :project: cpp_api

.. doxygentypedef:: ouster::core::PosesT
   :project: cpp_api

Functions
---------
.. doxygenfunction:: ouster::core::dewarp(const Points &points, const Pose &pose)
   :project: cpp_api

.. doxygenfunction:: ouster::core::dewarp(const Eigen::Ref<const PointsT<T>> &points, const Eigen::Ref<const PosesT<T>> &poses)
   :project: cpp_api

.. doxygenfunction:: ouster::core::dewarp(Eigen::Ref<PointsT<T>> dewarped, const Eigen::Ref<const PointsT<T>> points, const Eigen::Ref<const PosesT<T>> poses)
   :project: cpp_api

.. doxygenfunction:: ouster::core::transform(const Points &points, const Pose &pose)
   :project: cpp_api

.. doxygenfunction:: ouster::core::transform(const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>> points, const Eigen::Ref<const Eigen::Matrix<T, 1, 16, Eigen::RowMajor>> pose)
   :project: cpp_api

.. doxygenfunction:: ouster::core::transform(Eigen::Ref<Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>> transformed, const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>> points, const Eigen::Ref<const Eigen::Matrix<T, 1, 16, Eigen::RowMajor>> pose)
   :project: cpp_api


.. doxygenfunction:: ouster::core::interp_pose(const std::vector<double> &x_interp, const std::vector<double> &x_known, const Poses &poses_known)
   :project: cpp_api

.. doxygenfunction:: ouster::core::interp_pose(const std::vector<T> &x_interp, const std::vector<T> &x_known, const std::vector<Eigen::Matrix<double, 4, 4>> &poses_known)
   :project: cpp_api

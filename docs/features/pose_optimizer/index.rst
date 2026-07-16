.. _pose-optimizer:

==============
Pose Optimizer
==============

.. toctree::
   :hidden:
   :maxdepth: 1

   pose_optimizer_overview
   pose_optimizer_cli_sessions
   pose_optimizer_api
   pose_optimizer_visualization

The Pose Optimizer is a post processing tool for refining SLAM-generated trajectories and improving point cloud quality by reducing drift and improving alignment through constraint-based optimization.

The optimized trajectory improves alignment and reduces drift for better point cloud results, which can enhance downstream point cloud generation and localization.

.. figure:: /images/neighborhood_pose_optimize.png
   :align: center
   :alt: Neighborhood Pose Optimization

   Neighborhood-based pose optimization overview.

Key Capabilities
----------------

- **Constraint-Based Optimization**: The `PoseOptimizer` supports a variety of constraint types — including pose-to-pose and point-to-point correspondences, absolute-pose constraints, and absolute-point constraints — enabling flexible, robust trajectory refinement.

- **Loop Closure Integration**: Built-in support for manual and automatic loop closure constraints, including ``add_relative_loop_constraints()`` and the ``ouster-cli`` ``--auto-loop`` flag, reduces drift in long trajectories by linking corresponding observations across time.

- **GPS and Ground Truth Integration**: Absolute pose and point constraints, along with ``ouster-cli`` ``--auto-gps``, enable integration of GPS waypoints, surveyed landmarks, and other ground truth data for accurate global alignment.

- **Real-time Visualization**: The integrated visualization system provides real-time monitoring of optimization progress with interactive 3D displays, constraint selection and removal, deferred auto-loop generation, and gravity alignment via IMU data.

- **Store Constraints into JSON**: Export and import a constraints JSON file that records the exact set of constraints used for an optimization run. Saving and re-loading this file lets you reproduce optimization runs deterministically

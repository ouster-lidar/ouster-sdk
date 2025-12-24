=========================
Pose Optimization Example
=========================

.. contents::
   :local:
   :depth: 3

.. _pose-optimize-example:

The Pose Optimizer is a tool designed to refine the trajectory produced by a SLAM algorithm.
In this guide, you'll learn how to use the Python API to perform pose optimization.
Before getting started, ensure you have a SLAM processed OSF file that contains the initial SLAM poses.
You could get the sample OSF file from `Sample Data`_


New Direct Constraint Construction API
======================================

The Pose Optimizer now supports direct constraint construction, making the API more intuitive and object-oriented. 
Instead of using factory methods like ``make_pose_to_pose_constraint()``, you can now directly instantiate 
constraint objects and add them to the optimizer:

.. code:: python

   from ouster.sdk.mapping import PoseOptimizer, AbsolutePoseConstraint
   import numpy as np
   key_frame_distance = 1.0
   po = PoseOptimizer(source_osf_file, key_frame_distance)

   matrix = np.eye(4)
   # Adjust translation values (e.g., x = 3, y = 8)
   matrix[0, 3] = 3  # x translation
   matrix[1, 3] = 8  # y translation

   # Absolute Pose Constraint construction
   scan3_first_col_ts = 404807768600
   constraint = AbsolutePoseConstraint(timestamp=123456789, pose=matrix)
   constraint_id = po.add_constraint(constraint)  # Store ID for later removal if desired

   # You can also get and set constraints directly
   constraints = po.get_constraints()  # Returns a list of constraint objects
   po.set_constraints(constraints)     # Set all constraints at once

Available constraint types:

- ``AbsolutePoseConstraint``: Fixes a pose at a specific timestamp
- ``PoseToPoseConstraint``: Enforces relative transformation between two poses
- ``PointToPointConstraint``: Enforces correspondence between points in different scans
- ``AbsolutePointConstraint``: Enforces correspondence between a point and absolute coordinates

Each call to ``PoseOptimizer.add_constraint()`` returns the unique constraint ID assigned
by the optimizer, which is useful if you later want to remove that specific constraint.

**Important:** ``AbsolutePoseConstraint`` and ``PoseToPoseConstraint`` both use
``rotation_weight`` (a scalar) to scale the quaternion axis-alignment residual.
Set the value higher to enforce the measured orientation more strongly, or to
``0`` to ignore the rotational component altogether.


Running the Pose Optimizer PYTHON API
=====================================

.. code:: python

   from ouster.sdk.mapping import (PoseOptimizer, SamplingMode, save_trajectory,
                                  AbsolutePoseConstraint, PoseToPoseConstraint, 
                                  PointToPointConstraint, AbsolutePointConstraint)
   import numpy as np

   source_osf_file = "/PATH_TO_THE_OSF_FILE"
   output_osf_file = "/PATH_TO_THE_OUTPUT_OSF_FILE"
   # The key_frame_distance parameter is used to determine the distance in meter to
   # skip when adding the key node.
   key_frame_distance = 1.0
   po = PoseOptimizer(source_osf_file, key_frame_distance)

   scan1_first_col_ts = 411107223370
   scan2_first_col_ts = 531097181460

   # matrix can be 4 by 4 matrix or 6 by 1 in Eular angle and position vector
   scan_diff = np.array([[ 9.99526626e-01,  2.89677832e-02, -1.03630216e-02,  1.30564030e+00],
                         [-2.89636725e-02,  9.99580315e-01,  5.46561382e-04,  5.91995785e-01],
                         [ 1.03745051e-02, -2.46151491e-04,  9.99946153e-01, -1.31568585e-01],
                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])



   # Add pose to pose constraint which optimizes the relative pose between two scans to remove the drift
   pose2pose_constraint = PoseToPoseConstraint(
       scan1_first_col_ts,      # ts1 must be the frame timestamp (the first valid timestamp of the lidarscan) and must fall within the range of Lidar timestamps
       scan2_first_col_ts,      # ts2 must be the frame timestamp (the first valid timestamp of the lidarscan) and must fall within the range of Lidar timestamps
       scan_diff,               # relative transformation matrix between poses
       rotation_weight=1.0,                             # scalar weight for the quaternion axis-alignment residual (trajectory smoothness)
       translation_weight=np.array([1.0, 1.0, 1.0]),  # per-axis weights (x, y, z). Default is [1.0, 1.0, 1.0]
   )
   pose2pose_id = po.add_constraint(pose2pose_constraint)
   scan1_selected_point_row = 9
   scan1_selected_point_col = 542
   scan1_range_return_index = 1
   scan2_selected_point_row = 5
   scan2_selected_point_col = 545
   scan2_range_return_index = 1


   # Add point to point constraint which optimizes by closing the difference between two points in two scans
   pt2pt_constraint = PointToPointConstraint(
       scan1_first_col_ts,        # ts1 must be the frame timestamp (the first valid timestamp of the lidarscan) and must fall within the range of Lidar timestamps
       scan1_selected_point_row,  # row1 index
       scan1_selected_point_col,  # col1 index
       scan1_range_return_index,  # return_idx1 (should be 1 or 2)
       scan2_first_col_ts,        # ts2 must be the frame timestamp (the first valid timestamp of the lidarscan) and must fall within the range of Lidar timestamps
       scan2_selected_point_row,  # row2 index
       scan2_selected_point_col,  # col2 index
       scan2_range_return_index,  # return_idx2 (should be 1 or 2)
       translation_weight=np.array([1.0, 1.0, 1.0]),    # per-axis weights (x, y, z). Default is [1.0, 1.0, 1.0]
   )
   pt2pt_id = po.add_constraint(pt2pt_constraint)

   # Add absolute point constraint which fixes a specific point to absolute coordinates
   scan4_first_col_ts = 411107223370  # Example timestamp
   scan4_selected_point_row = 10
   scan4_selected_point_col = 100
   scan4_range_return_index = 1
   absolute_position = np.array([5.0, 2.0, 1.5])  # Known absolute position (x, y, z) in world coordinates

   abs_point_constraint = AbsolutePointConstraint(
       scan4_first_col_ts,        # ts must be the frame timestamp and fall within the range of Lidar timestamps
       scan4_selected_point_row,  # row index of the point
       scan4_selected_point_col,  # col index of the point
       scan4_range_return_index,  # return_idx (should be 1 or 2)
       absolute_position,         # absolute 3D position (x, y, z) that the point should correspond to
       translation_weight=np.array([1.0, 1.0, 1.0]),    # per-axis weights (x, y, z). Default is [1.0, 1.0, 1.0]
   )
   po.add_constraint(abs_point_constraint)

   # matrix can be 4 by 4 matrix or 6 by 1 in Eular angle and position vector
   matrix = np.eye(4)

   # Adjust translation values (e.g., x = 3, y = 8)
   matrix[0, 3] = 3  # x translation
   matrix[1, 3] = 8  # y translation

   scan3_first_col_ts = 404807768600


   # Add absolute pose constraint using the new direct constraint construction API
   abs_constraint = AbsolutePoseConstraint(
       scan3_first_col_ts,      # ts must fall within the range of the Lidar timestamps
       matrix,                  # matrix you want to apply to the pose of the chosen ts
       rotation_weight=1.0,                          # scalar weight for the quaternion axis-alignment residual
       translation_weight=np.array([1.0, 1.0, 1.0]),  # per-axis weights (x, y, z). Default is [1.0, 1.0, 1.0]
   )
   abs_pose_id = po.add_constraint(abs_constraint)

   # Optional: inspect constraint IDs and remove a specific user constraint
   # You can retrieve constraint objects and their internal IDs using
   # `get_constraints()` and `get_constraint_id()`. Removing a constraint is
   # optional and should be done only when you intentionally want to delete a
   # user-added constraint.
   constraints = po.get_constraints()
   print("Constraints count:", len(constraints))
   for c in constraints:
      # 0 indicates a non-user (internal) constraint; user-added constraints
      # have positive IDs assigned internally.
      print("constraint_id =", c.get_constraint_id())

   # Example: remove the pose-to-pose constraint added above (optional)
   # Note: remove only if you intend to delete a specific user constraint.
   po.remove_constraint(pose2pose_id)
   print(f"Removed constraint id {pose2pose_id}")
   
   # Solve completely until convergence
   po.solve()
   ts = po.get_timestamps(SamplingMode.COLUMNS)
   poses = po.get_poses(SamplingMode.COLUMNS)

   # Export the optimized trajectory to a CSV file so you can explore with the data
   save_trajectory("loop_test_traj.csv", ts, poses)

   # Save the optimized trajectory to an OSF file and you can visualize it following the steps below
   po.save(output_osf_file)


Verify your input data and adjust the constraint weights for optimal results.


Optional: Visualize During Optimization
=======================================

Use the CLI pose optimizer visualizer from Python to monitor trajectory
updates as the solver runs:

.. code:: python

   from ouster.cli.plugins.source_po_viz import PoseOptimizerViz

   viz_enabled = True

   if viz_enabled:
       viewer = PoseOptimizerViz(po, output_osf_file)
       viewer.run()


Visualizing the results
=======================
 
By following this guide, you can refine SLAM-generated point cloud using the Pose Optimizer.
This process reduces drift and improves alignment to improve the point cloud quality.

For visualizing the results, you can use the following ouster-cli commands:

- Replay the recording and visualize the refined point cloud.

.. code:: bash

   ouster-cli source <OUTPUT_OSF_FILENAME> viz --map

- Export a point cloud file and visualize the complete point cloud

.. code:: bash

   ouster-cli source <OUTPUT_OSF_FILENAME> save refined_point_cloud.ply
   ouster-cli source refined_point_cloud.ply viz


Enjoy experimenting with the Pose Optimization API!

.. _Sample Data: https://studio.ouster.com/share/QP8WDMOTC3HOI0TD

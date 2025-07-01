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


Running the Pose Optimizer PYTHON API
=====================================

.. code:: python

   from ouster.sdk.mapping import PoseOptimizer, SamplingMode, save_trajectory
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
   po.add_pose_to_pose_constraint(scan1_first_col_ts,      # ts1 must be the frame timestamp (the first valid timestamp of the lidarscan) and must fall within the range of Lidar timestamps
                                  scan2_first_col_ts,      # ts2 must be the frame timestamp (the first valid timestamp of the lidarscan) and must fall within the range of Lidar timestamps
                                  scan_diff,               # optional. if missing, the code uses ICP to calculate 
                                  rotation_weight=1.0,     # optional. Default is 1.0
                                  translation_weight=1.0,  # optional. Default is 1.0
                                  )


   scan1_selected_point_row = 9
   scan1_selected_point_col = 542
   scan1_range_return_index = 1
   scan2_selected_point_row = 5
   scan2_selected_point_col = 545
   scan2_range_return_index = 1

   # Add point to point constraint which optimizes by closing the difference between two points in two scans
   po.add_point_to_point_constraint(scan1_first_col_ts,        # ts1 must be the frame timestamp (the first valid timestamp of the lidarscan) and must fall within the range of Lidar timestamps
                                    scan1_selected_point_row,  # row1 index
                                    scan1_selected_point_col,  # col1 index
                                    scan1_range_return_index,  # return_idx1 (should be 1 or 2)
                                    scan2_first_col_ts,        # ts2 must be the frame timestamp (the first valid timestamp of the lidarscan) and must fall within the range of Lidar timestamps
                                    scan2_selected_point_row,  # row2 index
                                    scan2_selected_point_col,  # col2 index
                                    scan2_range_return_index,  # return_idx2 (should be 1 or 2)
                                    translation_weight=1.0,    # optional. Default is 1.0
                                    )

   # matrix can be 4 by 4 matrix or 6 by 1 in Eular angle and position vector
   matrix = np.eye(4)

   # Adjust translation values (e.g., x = 3, y = 8)
   matrix[0, 3] = 3  # x translation
   matrix[1, 3] = 8  # y translation

   scan3_first_col_ts = 404807768600

   po.add_absolute_pose_constraint(scan3_first_col_ts,      # ts must fall within the range of the Lidar timestamps
                                   matrix,                  # matrix
                                   rotation_weight=1.0,     # optional. Default is 1.0
                                   translation_weight=1.0,  # optional. Default is 1.0
                                   )
   
   # Solve completely until convergence
   po.solve()
   ts = po.get_timestamps(SamplingMode.COLUMNS)
   poses = po.get_poses(SamplingMode.COLUMNS)

   # Export the optimized trajectory to a CSV file so you can explore with the data
   save_trajectory("loop_test_traj.csv", ts, poses)

   # Save the optimized trajectory to an OSF file and you can visualize it following the steps below
   po.save(output_osf_file)


Verify your input data and adjust the constraint weights for optimal results.


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

.. _Sample Data: https://studio.ouster.com/drive/92996?orgId=1

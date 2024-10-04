===============
SLAM Quickstart
===============

.. contents::
   :local:
   :depth: 3

.. _slam-api-example:

This guide provides examples of using the SLAM API for development purposes.
Users can run SLAM with an OS sensor's hostname or IP for real-time processing, or with a recorded PCAP/OSF file for offline processing.


Obtain Lidar Pose and Calculate Pose Difference
===============================================
The SLAM API outputs the sensor's pose for each lidar scan, which you can use to determine the
sensor's orientation in your system. From the Lidar Poses, we can calculate the Pose difference
between consecutive scans

.. code:: python

   from ouster.sdk import open_source
   from ouster.sdk.mapping.slam import KissBackend
   import numpy as np
   import ouster.sdk.client as client
   source_file_path = "/PATH_TO_THE_FILE"
   data_source = open_source(source_file_path, sensor_idx=-1)
   slam = KissBackend(data_source.metadata, max_range=75, min_range=1, voxel_size=1.0)
   last_scan_pose = np.eye(4)

   for idx, scans in enumerate(data_source):
       # slam.update takes a list of scans as input and returns a list of scans,
       # supporting potential multi-sensor configurations
       scans_w_poses = slam.update(scans)
       if not scans_w_poses:
           continue
       # use the first scan for calculation
       col = client.first_valid_column(scans_w_poses[0])
       # scans_w_poses[0].pose is a list of poses where each pose represents a column
       # points' pose. Use the first valid scan's column pose as the scan pose
       scan_pose = scans_w_poses[0].pose[col]
       scan_ts = scans[0].timestamp[col]
       print(f"idx = {idx} at timestamp {scan_ts} has the pose {scan_pose}")

       # calculate the inverse transformation of the last scan pose
       inverse_last = np.linalg.inv(last_scan_pose)
       # calculate the pose difference by matrix multiplication
       pose_diff = np.dot(inverse_last, scan_pose)
       # extract rotation and translation
       rotation_diff = pose_diff[:3, :3]
       translation_diff = pose_diff[:3, 3]
       print(f"idx = {idx} and Rotation Difference: {rotation_diff}, "
             f"Translation Difference: {translation_diff}")


SLAM with Visualizer and Accumulated Scans
=========================================
Visualizers and Accumulated Scans are also available for monitoring the performance of the algorithm,
as well as for demonstration and feedback purposes.

.. code:: python

   from ouster.sdk import open_source
   from ouster.sdk.viz import SimpleViz
   from ouster.sdk.mapping.slam import KissBackend

   source_file_path = "/PATH_TO_THE_FILE"
   data_source = open_source(source_file_path, sensor_idx=-1)
   slam = KissBackend(data_source.metadata, max_range=75, min_range=1, voxel_size=1.0)

   scans_w_poses = map(lambda x: slam.update(x)[0], data_source)

   SimpleViz(data_source.metadata, accum_max_num=10).run(scans_w_poses)


More details about the visualizer and accumulated scans can be found at the
:ref:`Ouster Visualizer <viz-run>` and :ref:`Scans Accumulator <viz-scans-accum>`


.. note::

   The performance of the SLAM algorithm depends on your CPU's processing power and the 'voxel_size'
   parameter.
   Below is a suggestion for selecting an appropriate voxel size:

   | Outdoor: 1.4 - 2.2
   | Large indoor: 1.0 - 1.8
   | Small indoor: 0.4 - 0.8


Intro SLAM in Ouster-CLI
========================
We also offer a simpler method to run SLAM using the ``ouster-cli``. For additional details, please refer to :ref:`Ouster-CLI Mapping <ouster-cli-mapping>`.

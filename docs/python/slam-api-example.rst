===============
SLAM Quickstart
===============

.. contents::
   :local:
   :depth: 3

.. _slam-api-example:

This guide provides examples of using the SLAM API for development purposes.
Users can run SLAM with an OS sensor's hostname or IP for real-time processing, or with a recorded PCAP/OSF file for offline processing.

.. warning::
   Due to missing upstream dependency support on python3.12, slam does not work on python3.12.


Obtain Lidar Pose and Calculate Pose Difference
===============================================
The SLAM API outputs the sensor's pose for each lidar scan, which you can use to determine the
sensor's orientation in your system. From the Lidar Poses, we can calculate the Pose difference
between consecutive scans

.. code:: python

   from ouster.sdk import open_source
   from ouster.mapping.slam import KissBackend
   import numpy as np
   scans = open_source(pcap_path, sensor_idx=0)
   slam = KissBackend(scans.metadata, max_range=75, min_range=1, voxel_size=1.0)
   last_scan_pose = np.eye(4)

   for idx, scan in enumerate(scans):
        scan_w_poses = slam.update(scan)
        col = client.first_valid_column(scan_w_poses)
        # scan_w_poses.pose is a list where each pose represents a column points' pose.
        # use the first valid scan's column pose as the scan pose
        scan_pose = scan_w_poses.pose[col]
        print(f"idx = {idx} and Scan Pose {scan_pose}")

        # calculate the inverse transformation of the last scan pose
        inverse_last = np.linalg.inv(last_scan_pose)
        # calculate the pose difference by matrix multiplication
        pose_diff = np.dot(inverse_last, scan_pose)
        # extract rotation and translation
        rotation_diff = pose_diff[:3, :3]
        translation_diff = pose_diff[:3, 3]
        print(f"idx = {idx} and Rotation Difference: {rotation_diff}, "
              f"Translation Difference: {translation_diff}")


SLAM with Visulizer and Accumulated Scans
=========================================
Visualizers and Accumulated Scans are also available for monitoring the performance of the algorithm,
as well as for demonstration and feedback purposes.

.. code:: python

   from functools import partial
   from ouster.viz import SimpleViz, ScansAccumulator
   from ouster.mapping.slam import KissBackend
   scans = open_source(pcap_path, sensor_idx=0)
   slam = KissBackend(scans.metadata, max_range=75, min_range=1, voxel_size=1.0)

   scans_w_poses = map(partial(slam.update), scans)
   scans_acc = ScansAccumulator(info,
                                accum_max_num=10,
                                accum_min_dist_num=1,
                                map_enabled=True,
                                map_select_ratio=0.01)

   SimpleViz(info, scans_accum=scans_acc, rate=0.0).run(scans_w_poses)

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

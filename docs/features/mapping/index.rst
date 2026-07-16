SLAM
====

.. toctree::
   :hidden:
   :maxdepth: 1

   mapping-cli-sessions
   motion-compensation
   using-the-api
   viz-frames-accum
   point_cloud_alignment/index

SLAM (Simultaneous Localization and Mapping) is a computational technique that enables a robot to
construct a map of an unknown environment while simultaneously keeping track of its own location within that map.
Utilizing Ouster's LiDAR sensors, SLAM leverages high-resolution distance data to achieve precise localization and detailed environmental mapping.

The generated map is can be useful for subsequent tasks like path planning, obstacle avoidance, and navigation.

.. figure:: _snippets/images/localization-map-main.png
   :alt: SLAM processing pipeline
   :width: 100%

The Ouster SDK provides a robust Python and C++ API for building 3D maps from lidar sensor data.
This API enables developers to process raw sensor packets, accumulate point clouds, and construct spatial representations suitable for visualization, localization, and other applications.


Key Capabilities
----------------

- **Real-time SLAM**: The `SlamEngine` class uses the built in Lidar Inertial Odometry (LIO) backend for fast, incremental map generation and pose estimation from live or recorded data.

- **Map-Based Localization**: The ``LocalizationEngine`` enables real-time pose tracking against a pre-built point cloud map, supporting localization in known environments. See the dedicated :doc:`Localization </features/localization/index>` section for full details.

- **Motion Distortion Correction**: Built-in deskewing corrects for motion distortion within frames. On firmware >= 3.2 the default correction method for motion distortion is IMU-based deskewing for higher-fidelity correction; on older firmware it falls back to a constant-velocity model.

- **Multi-Sensor Synchronization**: The `ActiveTimeCorrection` module automatically handles unsynchronized or non-monotonic timestamps, improving multi-sensor SLAM robustness.

- **Offline Trajectory Optimization**: The `PoseOptimizer` class refines full trajectories post-processing, supporting constraints like `ABSOLUTE_POSE` (loop closures, GPS fixes) and `POINT_TO_POINT` correspondences.

- **Flexible I/O and Export**: Mapping tools ingest data from live sensors for real-time processing, PCAP, and OSF files for offline processing. Maps and trajectories can be exported in standard formats (PLY, LAS, PCD) or as OSF files.


Best Practices
--------------

- **Use Hardware Time Synchronization**: For best multi-sensor results, synchronize sensors using PTP 1588 or GPS/PPS. `ActiveTimeCorrection` is a fallback, but hardware sync is preferred.

- **Provide Accurate Extrinsics**: Supply precise 4x4 extrinsics matrices for each sensor to ensure correct point cloud fusion.

- **Tune Slam Config Parameters**: Adjust configuration settings for optimal results. These will be explained in detail in the next section.
 
Using SDK, one can configure and run SLAM using Python, C++, or via the CLI.

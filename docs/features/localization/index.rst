.. _localization:

============
Localization
============

.. toctree::
   :hidden:
   :maxdepth: 1

   using-cli
   using-the-api

Introduction
============

The Localization feature allows the user to identify the position and orientation (pose) of one or more sensors within a
pre-existing 3D point cloud map. While :doc:`SLAM </features/mapping/index>` is used to map an unknown
area, localization is used when you already have a map and need the sensor to navigate or track its
movement relative to that map 3D point cloud map.

By using the Ouster SDK, you can achieve high-precision, high-frequency pose tracking without needing to implement
complex point-cloud registration from scratch. The SDK handles motion compensation, multi-sensor
fusion, and frame-to-map alignment, exposing them through a simple CLI command and a programmatic
Python/C++ API.

Localization does not require GPS, GNSS, or other expensive secondary sensors. A single Ouster
LiDAR can be used by SDK to reliably estimate the sensor's position based entirely on relative pose changes within the environment.

Typical applications include:

* **Autonomous mobile robots (AMR):** Continuous pose tracking for navigation and path planning
  inside warehouses, factories, or other structured environments.
* **Fleet operations:** Multiple vehicles sharing a single reference map, each independently
  localizing against it.


How it Works
============

The SDK's localization engine uses an *Iterative Closest Point* (ICP) approach to match frames with the map.
The key difference from SLAM is that the internal map uses a **static map constraint**. The engine loads a
pre-built point cloud once at construction time and never modifies it. Each incoming batch of lidar frames is
aligned to this fixed reference to determine where the sensor is.

.. figure:: /images/localization_overview.svg
   :align: center
   :alt: High-level localization overview — Source feeds into Point Cloud Matching, which queries a static Reference Map, producing a Pose Estimate
   :width: 100%

   Source data is aligned against a static reference map using ICP. The map feeds into point cloud matching
   as a read-only input. The output is a per-column SE(3) pose written into each ``LidarFrame``.

At the heart of the engine is **frame-to-map registration** using Iterative Closest Point (ICP).
For each incoming frame the algorithm:

1. Finds the **nearest neighbor** in the static map for every live frame point (via a voxel hash
   map for fast lookup).
2. Computes the **rigid transformation** (rotation + translation) that minimizes the sum of squared
   distances between the matched pairs.
3. Applies the transform and **repeats** until the change falls below a convergence threshold (up to
   50 iterations).

The result is the sensor's pose in the map frame.

.. figure:: /images/icp_convergence.gif
   :align: center
   :alt: ICP frame-to-map alignment animation showing the live frame converging onto the static map
   :width: 580px

   ICP alignment: the cyan frame (source) rigidly slides into the white reference map
   (target) over successive iterations. Green lines show per-point nearest-neighbor
   correspondences used to solve each rigid transform.


SLAM vs. Localization
---------------------

Both SLAM and Localization use the same underlying ICP registration, but they serve different
purposes. SLAM explores an unknown environment and builds a map as it goes; Localization assumes
the map already exists and focuses on tracking the sensor within it.

.. figure:: /images/slam_vs_localization.svg
   :align: center
   :alt: SLAM and Localization workflows
   :width: 100%

   SLAM vs Localization workflow summary

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Aspect
     - SLAM
     - Localization
   * - Map
     - Built incrementally from live frames
     - Loaded from a pre-existing file (PLY or PCD)
   * - Map updates
     - New points are added each frame
     - Map is fixed; no points are added
   * - Use case
     - Exploring and mapping an unknown environment
     - Tracking pose in a known environment
   * - SDK class
     - ``SlamEngine``
     - ``LocalizationEngine``


Key Capabilities
================

- **Frame-to-Map Registration**: The ``LocalizationEngine`` aligns live point clouds against a
  pre-built map using Ouster SDK's iterative closest point solver, estimating a precise rigid
  transformation each frame.

- **Motion Distortion Correction**: Configurable deskewing compensates for sensor motion during a
  frame. On FW >= 3.2 the engine can use IMU data for higher-fidelity correction; on older firmware
  it falls back to a constant velocity deskew model.

- **Multi-Sensor Support**: The engine accepts frames from multiple sensors simultaneously,
  aggregating their points into a single frame before registration. This provides a wider field of
  view and improves robustness in feature-poor environments.

- **Per-Column Pose Output**: Rather than a single pose per frame, the engine computes a unique SE(3)
  transform for every measurement column, enabling sub-frame trajectory precision for high-speed
  motion.

- **Custom Solver Integration**: The SDK can serve as a data provider — use ``XYZLut`` to convert
  frames to Cartesian point clouds for your own solver, then inject poses back into the frame for
  downstream visualization and recording.

- **Flexible Map Input**: Maps in PLY or PCD format are loaded directly. A typical workflow is to
  run SLAM first, export the accumulated point cloud, and reuse it for localization indefinitely.


Getting Started
===============

See :doc:`using-cli` for the quickest way to run localization from the command line and to see visualization options.

For programmatic integration, see :doc:`using-the-api` which covers configuration, engine
construction, the processing loop, and using a custom localization solver in both Python and C++.


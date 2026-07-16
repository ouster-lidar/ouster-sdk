Using the API
=============

The Ouster SDK provides a mapping API that allows developers to integrate Simultaneous Localization and Mapping (SLAM) capabilities into their applications.
The API is built around two primary components:

* :ouster:class:`SlamConfig <py=ouster.sdk.mapping.SlamConfig|cpp=ouster::sdk::mapping::SlamConfig>`
  - A class whose instances hold configuration parameters for the SLAM process. Subclasses, such as
  :ouster:class:`LIOSlamConfig <py=ouster.sdk.mapping.LIOSlamConfig|cpp=ouster::sdk::mapping::LIOSlamConfig>`,
  provide specific configurations for different implementations. LIO refers to the Lidar Inertial Odometry.
* :ouster:class:`SlamEngine <py=ouster.sdk.mapping.SlamEngine|cpp=ouster::sdk::mapping::SlamEngine>`
  - An interface for implementations that process lidar frames, estimates sensor poses, and build an internal map.
  This interface also serves as a factory for different SLAM implementations, e.g. the built-in Lidar Inertial Odometry engine.
* The engine modifies input LidarFrame objects to include per-column pose information.

Prerequisites
-------------

Before using the mapping and SLAM features, ensure your development environment is set up and the Ouster SDK is installed.

- Follow the installation steps in the `Getting Started <../../getting-started.html>`__ guide.
- Download PCAP/OSF data from the `Sample Data <../../getting-started/download-data.html>`__ section to experiment with the API.
  
- For C++ development, make sure you have built the SDK with ``BUILD_MAPPING=ON``.
  

Configuration
-------------

Start by configuring how the SLAM engine should run.
:ouster:class:`SlamConfig <py=ouster.sdk.mapping.SlamConfig|cpp=ouster::sdk::mapping::SlamConfig>` holds
the shared parameters (minimum and maximum range, voxel resolution, deskew strategy, initial pose, etc.).
Subclasses such as
:ouster:class:`LIOSlamConfig <py=ouster.sdk.mapping.LIOSlamConfig|cpp=ouster::sdk::mapping::LIOSlamConfig>`
provide configuration for the LIO backend; pass the resulting config to
:ouster:func:`SlamEngine.create <py=ouster.sdk.mapping.SlamEngine.create|cpp=ouster::sdk::mapping::SlamEngine::create>`.

In Python, use the :ouster:func:`SlamConfig.create <py=ouster.sdk.mapping.SlamConfig.create>` factory
with ``"lio"`` to obtain a concrete config instance.
In C++, construct :ouster:class:`LIOSlamConfig <cpp=ouster::sdk::mapping::LIOSlamConfig>` directly.

A ``SlamConfig`` (or ``LIOSlamConfig``) object provides several parameters to tune the SLAM algorithm:

.. list-table::
   :header-rows: 1
   :widths: 22 28 50

   * - Option
     - Type | Values
     - Notes
   * - ``min_range`` | ``max_range``
     - float
     - Range gating (in meters) to filter points before processing.
   * - ``voxel_size``
     - float (meters)
     - Voxel size for the internal map. If set to 0.0, a suitable size will be automatically determined from the first few frames.
       Controls map resolution; Smaller values increase map fidelity but also increase memory usage and computation time.
   * - ``initial_pose``
     - 4×4 matrix
     - A 4x4 transformation matrix specifying the starting pose of the sensor in the world frame. Defaults to the identity matrix.
   * - ``max_iterations``
     - int
     - Maximum number of ICP registration iterations per frame update. Defaults to 500.
   * - ``deskew_method``
     - ``"auto"`` (default) | ``"constant_velocity"`` | ``"imu_deskew"`` | ``"none"``
     - Method to use for motion distortion correction (deskewing) within each frame.

 
``deskew_method`` specifies how to handle motion distortion within each frame. Options are:

* **"auto"**: For FW >= 3.2, the ``deskew_method`` is set to ``"imu_deskew"``, which uses IMU data from the sensor to perform more accurate deskewing.
  On FW < 3.2, it defaults to ``"constant_velocity"``.
* **"constant_velocity"**: Enables software-based deskewing assuming constant velocity motion between poses during the frame acquisition.
* **"none"**: Disables software deskewing, assuming the input frames are already deskewed or that motion distortion is negligible.


.. note::

   The performance of the SLAM algorithm depends on your CPU's processing power and the 'voxel_size'
   parameter.
   Below is a suggestion for selecting an appropriate voxel size:

   | Outdoor: 1.4 - 2.2
   | Large indoor: 1.0 - 1.8
   | Small indoor: 0.4 - 0.8


The following code snippets demonstrate how to create a SlamConfig, which ``SlamEngine.create`` needs to instantiate a ``SlamEngine``.

.. rubric:: **Imports**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slam.py
         :language: python
         :start-after: [doc-stag-slam-imports]
         :end-before: [doc-etag-slam-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/slam.py>`__
         :dedent: 0
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: ../../../examples/slam_example.cpp
         :language: cpp
         :start-after: [doc-stag-slam-imports]
         :end-before: [doc-etag-slam-imports]
         :class: doc-snippet
         :caption: `View on GitHub <https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/slam_example.cpp>`__
         :dedent: 0

.. rubric:: **Create SlamConfig/LIOSlamConfig**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slam.py
         :language: python
         :start-after: [doc-stag-slam-open]
         :end-before: [doc-etag-slam-open]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/slam.py>`__
         :dedent: 4
   
   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: ../../../examples/slam_example.cpp
         :language: cpp
         :start-after: [doc-stag-slam-open]
         :end-before: [doc-etag-slam-open]
         :class: doc-snippet
         :caption: `View on GitHub <https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/slam_example.cpp>`__
         :dedent: 4


Open a Data Source and Construct the Engine
-------------------------------------------

Next, open your lidar data source using :ouster:func:`open_source <py=ouster.sdk.open_source|cpp=ouster::sdk::open_source>`. This function handles live sensors, PCAP files, and OSF files,
returning an iterable source object and associated metadata (:ouster:class:`SensorInfo <py=ouster.sdk.core.SensorInfo|cpp=ouster::sdk::core::SensorInfo>`).

The ``SlamEngine`` is also a factory that can instantiate derived ``SlamEngine`` instances using the
:ouster:func:`create <py=ouster.sdk.mapping.SlamEngine.create|cpp=ouster::sdk::mapping::SlamEngine::create>` method,
providing the ``SensorInfo`` list (even for a single sensor) and an optional ``SlamConfig`` (defaults to
``LIOSlamConfig{}``). Currently, ``SlamEngine`` supports only the ``lio`` (Lidar Inertial Odometry) implementation, which provides fast, incremental, CPU-based mapping.
The ``SensorInfo`` is crucial as it contains calibration data (beam angles, intrinsics) needed for point cloud generation and deskewing.


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slam.py
         :language: python
         :start-after: [doc-stag-slam-engine]
         :end-before: [doc-etag-slam-engine]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/slam.py>`__
         :dedent: 3

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: ../../../examples/slam_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-slam-engine]
         :end-before: //! [doc-etag-slam-engine]
         :class: doc-snippet
         :caption: `View on GitHub <https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/slam_example.cpp>`__
         :dedent: 3

.. _slam-run-loop:

Run SLAM on Each Frame
----------------------

The core of the SLAM process is an iteration loop. You read a :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>`, ``frame_set``, from your source and pass it to the ``slam_engine.update()`` method.

:ouster:func:`SlamEngine.update() <py=ouster.sdk.mapping.SlamEngine.update|cpp=ouster::sdk::mapping::SlamEngine::update>` processes the input ``FrameSet`` and augments each frame with per-column pose information.
In Python, it returns the updated frame set; in C++, it modifies the ``FrameSet`` in place and returns ``void``.

This ``update()`` call performs several crucial steps:

- It deskews the frame(s) using the configured ``deskew_method``, writing an initial per-column trajectory into ``frame.body_to_world``.
- It aggregates points from the frame(s) in the set.
- It registers the new points against its internal map (using KISS-ICP) to estimate the sensor's new pose.
- It applies the ICP pose correction to every column pose and updates its internal map with the new points.

After ``update()`` completes, the ``FrameSet`` is populated with map-aligned pose data. You can then access the pose for any column (measurement) in the frame to get the sensor’s position at that precise moment. The list contains one entry per sensor.


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slam.py
         :language: python
         :start-after: [doc-stag-slam-loop-update]
         :end-before: [doc-etag-slam-loop-update]
         :dedent: 4
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/slam.py>`__

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: ../../../examples/slam_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-slam-loop-update]
         :end-before: //! [doc-etag-slam-loop-update]
         :dedent: 4
         :class: doc-snippet
         :caption: `View on GitHub <https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/slam_example.cpp>`__


Inspect SLAM Output
-------------------

Each :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` now carries ``frame.body_to_world`` (a per-column SE(3) transform) and ``frame.timestamp`` arrays that are aligned with the
range/reflectivity columns.  The engine also keeps the internal map up to date so subsequent calls
benefit from accumulated structure.

The examples show how to extract the pose corresponding to the last valid column in the frame and convert its rotation part into Euler angles (yaw, pitch, roll) for easier interpretation.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slam.py
         :language: python
         :start-after: [doc-stag-slam-loop-printpose]
         :end-before: [doc-etag-slam-loop-printpose]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/slam.py>`__
         :dedent: 8

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: ../../../examples/slam_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-slam-loop-printpose]
         :end-before: //! [doc-etag-slam-loop-printpose]
         :class: doc-snippet
         :caption: `View on GitHub <https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/slam_example.cpp>`__
         :dedent: 8

.. rubric:: **Full processing loop**

Combining these steps, the typical structure for running SLAM involves opening the source, creating the engine, and looping through the frames while calling update() and processing the results. The complete loop is shown below:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slam.py
         :language: python
         :start-after: [doc-stag-slam-loop]
         :end-before: [doc-etag-slam-loop]
         :class: doc-snippet
         :dedent: 4
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/slam.py>`__

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: ../../../examples/slam_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-slam-loop]
         :end-before: //! [doc-etag-slam-loop]
         :class: doc-snippet
         :dedent: 4
         :caption: `View on GitHub <https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/slam_example.cpp>`__


.. include:: tutorial_1.rst

Handling Motion Distortion: Deskew vs. Dewarp
---------------------------------------------

When a LiDAR sensor is in motion (e.g., on a vehicle or a handheld device), the resulting point cloud
suffers from distortion. Because each column of a frame is captured at a slightly different time, the
captured frame data can be distorted "skewed".

To create an accurate map, the Ouster SDK's mapping module corrects this distortion using a two-step
process: Deskewing and Dewarping.

Deskew (Motion Compensation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Deskewing is the process of estimating the required motion compensation for the frame captured while 
the sensor in motion.
   
.. rubric:: **What it does**
   
It estimates the sensor's 4x4 pose at the specific timestamp for each column in the frame.
This is done either using high-frequency data from an IMU or a constant velocity motion model.

.. rubric:: **How it's used in the SDK**

The ``deskew_method`` discussed earlier runs as the first step inside ``SlamEngine::update()``.
It estimates an initial per-column trajectory and writes it into ``frame.body_to_world`` (a ``(W, 4, 4)`` array).

.. rubric:: SDK Implementation

As described in :ref:`slam-run-loop`, ``update()`` then registers the deskewed points against the
internal map using KISS-ICP and applies the resulting pose correction to every column pose in
``frame.body_to_world``. The poses you read after ``update()`` are therefore deskewed *and* map-aligned—not
deskew-only estimates.

In short: Deskewing supplies the initial per-column trajectory; ICP registration refines it before
the poses are returned.


Dewarp (Point Cloud Transformation)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Dewarping applies the estimated motion compensation from the deskewing step to transform the 3D points
into a rigid, and geometrically correct point cloud.

.. rubric:: **What it does**
   
It takes the distorted (H, W, 3) point cloud and the (W, 4, 4) per-column poses. It then applies the
i-th pose matrix to all points in the i-th column, transforming all points from their individual
"column frames" into a single, consistent coordinate frame (the frame of the middle column).

.. rubric:: **Imports**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slam_dewarp_example.py
         :language: python
         :start-after: [doc-stag-slam-dewarp-imports]
         :end-before: [doc-etag-slam-dewarp-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/slam_dewarp_example.py>`__
         :dedent: 0

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slam_dewarp_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-slam-dewarp-imports]
         :end-before: //! [doc-etag-slam-dewarp-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/cpp/slam_dewarp_example.cpp>`__
         :dedent: 0

.. rubric:: **How it's used in the SDK**
   
Dewarping produces a new point cloud by applying a lidar frame's per-column poses to the frame's point cloud. When deskew (motion
compensation) produces the per-column poses for the frame, the result of dewarp is a point cloud with motion distortion removed, where all
points are expressed in a common frame.
Refer to :ouster:func:`dewarp <py=ouster.sdk.core.dewarp|cpp=ouster::sdk::core::dewarp>` for API details.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slam_dewarp_example.py
         :language: python
         :start-after: [doc-stag-slam-dewarp-py]
         :end-before: [doc-etag-slam-dewarp-py]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/slam_dewarp_example.py>`__
         :dedent: 0

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slam_dewarp_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-slam-dewarp-cpp]
         :end-before: //! [doc-etag-slam-dewarp-cpp]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/cpp/slam_dewarp_example.cpp>`__
         :dedent: 0

Both snippets iterate over the first batch of frames, rely on SLAM to populate per-column poses
(see :ref:`slam-run-loop`), and then call the appropriate ``dewarp`` helper to express all XYZ samples
in the global frame. To learn more about :ouster:class:`core.XYZLut <py=ouster.sdk.core.data.XYZLut|cpp=ouster::sdk::core::XYZLutT>`
refer to :doc:`/features/processing/using-the-api`.

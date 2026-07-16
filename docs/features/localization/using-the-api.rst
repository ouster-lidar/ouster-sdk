:tocdepth: 2

Using the API
=============

The Ouster SDK provides a programmatic localization API in both Python and C++. The API mirrors
the :doc:`SLAM API </features/mapping/using-the-api>` in structure, making it straightforward to
switch between mapping a new environment and localizing against an existing map.

The API is built around two components:

* :ouster:class:`LocalizationConfig <py=ouster.sdk.mapping.LocalizationConfig|cpp=ouster::sdk::mapping::LocalizationConfig>` - 
  A class whose instances hold configuration parameters (range gates, voxel size, deskew method, initial pose). Subclasses, such as 
  :ouster:class:`LIOLocalizationConfig <py=ouster.sdk.mapping.LIOLocalizationConfig|cpp=ouster::sdk::mapping::LIOLocalizationConfig>`,
  provide specific configurations for different localization engines.
* :ouster:class:`LocalizationEngine <py=ouster.sdk.mapping.LocalizationEngine|cpp=ouster::sdk::mapping::LocalizationEngine>`
  - An interface for implementations that process lidar frames and writes per-column poses to the frames using a user-provided map.
  This interface also serves as a factory for different localization implementations, e.g. the built-in LIO-based engine.
* The engine modifies input LidarFrame objects to include per-column pose information.


Prerequisites
-------------

Before using the localization API, ensure your development environment is set up and the
Ouster SDK is installed.

- Follow the installation steps in the `Getting Started <../../getting-started.html>`__ guide.
- Prepare a point cloud map file (PLY or PCD) generated from a prior SLAM run.
- For C++ development, build the SDK with ``BUILD_MAPPING=ON``.


Configuration
-------------

Start by configuring how the localization engine should run.
:ouster:class:`LocalizationConfig <py=ouster.sdk.mapping.LocalizationConfig|cpp=ouster::sdk::mapping::LocalizationConfig>` holds
the shared parameters (range gates, voxel size, deskew method, initial pose, etc.).
Subclasses such as
:ouster:class:`LIOLocalizationConfig <py=ouster.sdk.mapping.LIOLocalizationConfig|cpp=ouster::sdk::mapping::LIOLocalizationConfig>`
provide configuration for the LIO backend; pass the resulting config to
:ouster:func:`LocalizationEngine.create <py=ouster.sdk.mapping.LocalizationEngine.create|cpp=ouster::sdk::mapping::LocalizationEngine::create>`.

In Python, use the :ouster:func:`LocalizationConfig.create <py=ouster.sdk.mapping.LocalizationConfig.create>` factory
with ``"lio"`` to obtain a concrete config instance.
In C++, construct :ouster:class:`LIOLocalizationConfig <cpp=ouster::sdk::mapping::LIOLocalizationConfig>` directly.

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Parameter
     - Default
     - Description
   * - ``min_range``
     - ``0.0``
     - Minimum range in meters. Points closer than this are discarded before registration.
   * - ``max_range``
     - ``150.0``
     - Maximum range in meters. Points beyond this are discarded before registration.
   * - ``voxel_size``
     - ``0.0``
     - Voxel grid resolution in meters for downsampling. When set to ``0.0``, the engine estimates
       a suitable value automatically from the first batch of frames (falling back to ``1.0`` m).
   * - ``max_iterations``
     - ``500``
     - Maximum number of ICP registration iterations per frame update.
   * - ``initial_pose``
     - Identity (4×4)
     - A 4×4 transformation matrix specifying the sensor's starting pose in the map frame.
   * - ``deskew_method``
     - ``"auto"``
     - Motion compensation strategy: ``"auto"``, ``"constant_velocity"``, ``"imu_deskew"``, or
       ``"none"``.

The following code snippets demonstrate how to create a ``LocalizationConfig``, which the ``LocalizationEngine.create`` method needs to instantiate a ``LocalizationEngine``.

.. rubric:: **Imports**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/localization.py
         :language: python
         :start-after: [doc-stag-localization-imports]
         :end-before: [doc-etag-localization-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/python/localization.py>`__
         :dedent: 0

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/localization_example.cpp
         :language: cpp
         :start-after: [doc-stag-localization-imports]
         :end-before: [doc-etag-localization-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/cpp/localization_example.cpp>`__
         :dedent: 0

.. rubric:: **Create LocalizationConfig / LIOLocalizationConfig**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/localization.py
         :language: python
         :start-after: [doc-stag-localization-config]
         :end-before: [doc-etag-localization-config]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/python/localization.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/localization_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-localization-config]
         :end-before: //! [doc-etag-localization-config]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/cpp/localization_example.cpp>`__
         :dedent: 4


Open a Data Source and Construct the Engine
-------------------------------------------

Open your lidar data source with :ouster:func:`open_source <py=ouster.sdk.open_source|cpp=ouster::sdk::open_source>`.  This function handles live sensors, PCAP files, and OSF files,
returning an iterable source object and associated metadata (:ouster:class:`SensorInfo <py=ouster.sdk.core.SensorInfo|cpp=ouster::sdk::core::SensorInfo>`).

Use :ouster:func:`LocalizationEngine.create <py=ouster.sdk.mapping.LocalizationEngine.create|cpp=ouster::sdk::mapping::LocalizationEngine::create>` to instantiate the engine,
passing the ``SensorInfo`` list (even for a single sensor), a map (a PLY/PCD file path or an in-memory point cloud),
and an optional ``LocalizationConfig`` (defaults to ``LIOLocalizationConfig{}``).

Currently, ``LocalizationEngine`` supports only the ``lio`` (Lidar Inertial Odometry) implementation, which provides
fast, incremental, CPU-based frame-to-map registration.
The ``SensorInfo`` is crucial as it contains calibration data (beam angles, intrinsics) needed for point cloud generation and deskewing.

.. note::

   The map is not part of ``LocalizationConfig``; pass it to
   :ouster:func:`LocalizationEngine.create <py=ouster.sdk.mapping.LocalizationEngine.create|cpp=ouster::sdk::mapping::LocalizationEngine::create>`.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/localization.py
         :language: python
         :start-after: [doc-stag-localization-engine]
         :end-before: [doc-etag-localization-engine]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/python/localization.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/localization_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-localization-engine]
         :end-before: //! [doc-etag-localization-engine]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/cpp/localization_example.cpp>`__
         :dedent: 4

The create method invokes the constructor for the LIO-based ``LocalizationEngine`` implementation, which loads the map file,
indexes its points into the voxel structure, and prepares the engine for frame registration.

In C++ there is also an overload that accepts an in-memory :ouster:class:`PointCloudXYZf <cpp=ouster::sdk::core::PointCloudXYZf>` (Eigen N×3 matrix)
instead of a file path — useful when the map is already loaded or generated programmatically.

Map File Requirements
---------------------

The localization engine accepts point cloud maps in **PLY** or **PCD** format, loaded via
:ouster:func:`read_pointcloud() <py=ouster.sdk.core.read_pointcloud|cpp=ouster::sdk::read_pointcloud>`. The file must contain an N×3 array of XYZ coordinates in float precision.

A typical workflow to produce a map is:

1. Run SLAM to generate an OSF file with poses:

   .. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
      :language: bash
      :start-after: [doc-slam-to-osf-begin]
      :end-before: [doc-slam-to-osf-end]
      :dedent: 0
      :class: doc-snippet

2. Export the accumulated point cloud as a PLY file:

   .. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
      :language: bash
      :start-after: [doc-osf-to-map-ply-begin]
      :end-before: [doc-osf-to-map-ply-end]
      :dedent: 0
      :class: doc-snippet

Refer to the :ref:`SLAM CLI documentation <ouster-cli-mapping>` for detailed instructions on map
generation, including range filtering with the ``clip`` command.


Processing Pipeline Detail
--------------------------

The core entry point is the :ouster:func:`update() <py=ouster.sdk.mapping.LocalizationEngine.update|cpp=ouster::sdk::mapping::LocalizationEngine::update>` method on ``LocalizationEngine``. You pass it a
:ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` (one or more lidar frames from a single rotation cycle).
In Python, it returns the updated frame set; in C++, it modifies the ``FrameSet`` in place and returns ``void``.
The engine registers the frame against the static map and writes map-aligned per-column poses into each frame.
The result is a ``(W, 4, 4)`` array of SE(3) transforms on each frame — one 4×4 matrix per measurement
column — encoding the sensor's position and orientation at each column's timestamp relative to
the map origin.

Each call to :ouster:func:`update() <py=ouster.sdk.mapping.LocalizationEngine.update|cpp=ouster::sdk::mapping::LocalizationEngine::update>` performs the following steps:

1. **Time correction** — inter-sensor clock offsets and timestamp monotonicity are checked and
   corrected via ``ActiveTimeCorrection``.

2. **Deskewing (motion compensation)** — because each column in a lidar frame is captured at a
   slightly different time, sensor motion introduces distortion. The SDK applies a deskew method
   (configurable via ``deskew_method``) to estimate per-column poses that compensate for this
   motion:

   * ``"auto"`` — uses IMU-based deskewing when synchronous IMU data is available (FW 3.2+ with
     ``ACCEL32_GYRO32_NMEA``), otherwise falls back to constant-velocity deskewing.
   * ``"constant_velocity"`` — assumes uniform motion between poses.
   * ``"imu_deskew"`` — uses IMU accelerometer and gyroscope data for higher-fidelity correction.
   * ``"none"`` — disables deskewing.

3. **Point aggregation** — valid points from all sensors in the :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` are merged into a
   single frame, filtered by ``min_range`` and ``max_range``.

4. **Frame-to-map registration** — The aggregated frame is aligned to the static map via ICP. This process
   determines the rigid transformation that minimizes the distance between live points and their nearest map neighbors.

5. **Pose correction** — The ICP correction is applied to every deskewed column pose, writing
   map-aligned per-column SE(3) transforms back into each ``LidarFrame``. The poses you read after
   ``update()`` are deskewed *and* map-aligned—not deskew-only estimates.


.. figure:: /images/localization_data_pipeline.svg
   :align: center
   :alt: The stages executed on each call to update() in LocalizationEngine.
   :width: 100%

   The stages executed on each call to :ouster:func:`update() <py=ouster.sdk.mapping.LocalizationEngine.update|cpp=ouster::sdk::mapping::LocalizationEngine::update>`.


Run Localization on Each Frame
-----------------------------

The core loop reads :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` batches from the source and passes each to :ouster:func:`update() <py=ouster.sdk.mapping.LocalizationEngine.update|cpp=ouster::sdk::mapping::LocalizationEngine::update>`.
In Python, ``update()`` returns the updated frame set; in C++, it mutates the ``FrameSet`` in place.
Each frame's per-column ``body_to_world`` array is filled with SE(3) transforms relative to the map origin.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/localization.py
         :language: python
         :start-after: [doc-stag-localization-loop-update]
         :end-before: [doc-etag-localization-loop-update]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/python/localization.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/localization_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-localization-loop-update]
         :end-before: //! [doc-etag-localization-loop-update]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/cpp/localization_example.cpp>`__
         :dedent: 4

.. note::

   ``update()`` does **not** return a success flag. If the frame cannot be processed (e.g., all
   timestamps are invalid or no valid points remain after range filtering), the engine logs a
   warning and the frame's poses remain unchanged.


Inspect Localization Output
---------------------------

After ``update()``, each :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` carries ``frame.body_to_world`` — a ``(W, 4, 4)`` array of per-column
SE(3) transforms. You can extract translation and rotation for the last valid column:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/localization.py
         :language: python
         :start-after: [doc-stag-localization-loop-printpose]
         :end-before: [doc-etag-localization-loop-printpose]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/python/localization.py>`__
         :dedent: 8

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/localization_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-localization-loop-printpose]
         :end-before: //! [doc-etag-localization-loop-printpose]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/cpp/localization_example.cpp>`__
         :dedent: 8


Full Processing Loop
--------------------

Combining all of the above, the typical localization workflow looks like this:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/localization.py
         :language: python
         :start-after: [doc-stag-localization-loop]
         :end-before: [doc-etag-localization-loop]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/python/localization.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/localization_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-localization-loop]
         :end-before: //! [doc-etag-localization-loop]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/cpp/localization_example.cpp>`__
         :dedent: 4


Advanced Topics
---------------

Multi-Sensor Localization
^^^^^^^^^^^^^^^^^^^^^^^^^

The ``LocalizationEngine`` accepts a list of :ouster:class:`SensorInfo <py=ouster.sdk.core.SensorInfo|cpp=ouster::sdk::core::SensorInfo>` objects and processes a
:ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` containing frames from multiple sensors. Points from all sensors are aggregated
into a single frame before registration, providing a wider field of view and improving robustness
in feature-poor environments (e.g., long hallways with few geometric features).

For best results with multi-sensor setups:

* **Use hardware time synchronization** (PTP 1588 or GPS/PPS). The SDK includes software-based
  ``ActiveTimeCorrection`` as a fallback, but hardware sync yields better accuracy.
* **Provide accurate extrinsics** — each sensor's 4×4 extrinsic matrix must be correctly set so
  that points from different sensors are fused in a consistent coordinate frame.


GNSS/GPS Initialization
^^^^^^^^^^^^^^^^^^^^^^^^

Starting at the map's origin is not always practical, especially on large outdoor sites. If a
rough GPS fix is available you can compute an ``initial_pose`` matrix before constructing the
engine:

1. Convert the GPS coordinate (latitude, longitude) to a local Cartesian frame such as
   `UTM <https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system>`_
   (Universal Transverse Mercator — a projection that converts degrees of latitude/longitude into
   flat X/Y coordinates in meters).
2. Compute the offset between the current UTM position and the map origin (which was established
   during the original SLAM run).
3. Set the resulting translation on ``LocalizationConfig.initial_pose``.


.. literalinclude:: _snippets/python/localization.py
   :language: python
   :start-after: [doc-stag-localization-gnss-init]
   :end-before: [doc-etag-localization-gnss-init]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/python/localization.py>`__
   :dedent: 4

For CLI usage of the initial pose flag see :ref:`setting-the-initial-pose` in :doc:`using-cli`.


Dynamic Filtering
^^^^^^^^^^^^^^^^^

When a sensor is mounted close to the robot chassis, the lidar consistently sees the robot's own
body. These static, close-range points can confuse the frame-to-map registration and cause drift.

Increase ``min_range`` so that returns from the robot's frame are excluded before they reach the
engine:

.. literalinclude:: _snippets/python/localization.py
   :language: python
   :start-after: [doc-stag-localization-dynamic-filter]
   :end-before: [doc-etag-localization-dynamic-filter]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/localization/_snippets/python/localization.py>`__
   :dedent: 4

Similarly, setting ``max_range`` to a sensible value for your environment avoids matching against
noisy long-range returns.

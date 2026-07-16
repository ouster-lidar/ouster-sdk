Perception [BETA]
=================

.. note::

   This is a **BETA** feature.

Ouster SDK 1.0 introduces the Ouster Perception API, which includes a detection engine interface and implementation
to add objects (i.e. features derived from lidar data) to :ouster:class:`LidarFrame
<py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` or
:ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>`.

At this time, Ouster SDK includes a single detection engine implementation that performs clustering
on the lidar data present in a ``LidarFrame`` or ``FrameSet`` and adds an
:ouster:class:`Object <py=ouster.sdk.core.Object|cpp=ouster::sdk::core::Object>` for each cluster.
Future releases will add new engines, including those that do object classification.

Key capabilities
----------------

* **Cluster-based object detection:** Run the built-in ``DetectionEngine`` on live or recorded sources to produce cluster-derived :ouster:class:`Object <py=ouster.sdk.core.Object|cpp=ouster::sdk::core::Object>` instances for each frame.
* **OSF persistence for perception outputs:** Save frames, detected objects, and related metadata into :doc:`OSF <reference/osf>` for reproducible offline workflows.
* **Metadata-aware datasets:** Create and attach metadata such as, class maps, through :ouster:class:`FrameSetSourceMetadataSet <py=ouster.sdk.core.FrameSetSourceMetadataSet|cpp=ouster::sdk::core::FrameSetSourceMetadataSet>` via API, and persist it in OSF; CLI preserves existing metadata during OSF save.
* **Integrated visualization:** Inspect detections with ``ouster-cli ... detect ... viz`` or in Python using :ouster:class:`SimpleViz <py=ouster.sdk.viz.SimpleViz>`.

You can find the getting started guide for CLI at :doc:`ouster-cli </getting-started/installation>`.

Applications
------------

Common applications for Perception in the SDK include:

* **Rapid obstacle discovery and scene triage:** Accelerating early-stage development in robotics prototypes by quickly isolating critical objects.
* **Downstream stack integration:** Extracting obstacles, bounding boxes, or clusters to serve as clean inputs for tracking, localization, and path-planning modules.
* **Dataset curation and annotation:** Saving model inferences and detections directly within OSF files to streamline downstream labeling and data engineering.
* **Baseline benchmarking and regression testing:** Evaluating model performance metrics consistently across historical recordings and diverse physical sensor configurations.
* **Visualization and situational awareness:** Overlaying perception data for client demos, remote operator interfaces, and visual QA workflows.

Core concepts
-------------

.. perception-core-types-start

* :ouster:class:`Object <py=ouster.sdk.core.Object|cpp=ouster::sdk::core::Object>` — a generic object,
  represented by a bounding box, ``object_to_body`` and ``body_to_world`` poses, velocity, classification, a timestamp, and user-definable set
  of properties represented by strings.

* :ouster:class:`DetectionConfig <py=ouster.sdk.perception.DetectionConfig|cpp=ouster::sdk::perception::DetectionConfig>`
  — base configuration for detection engines. Subclasses such as
  :ouster:class:`ClassicDetectionConfig <py=ouster.sdk.perception.ClassicDetectionConfig|cpp=ouster::sdk::perception::ClassicDetectionConfig>`
  provide settings for the classic cluster-based engine.

* :ouster:class:`DetectionEngine <py=ouster.sdk.perception.DetectionEngine|cpp=ouster::sdk::perception::DetectionEngine>`
  — a base class that declares virtual methods for modifying (e.g. by adding objects to)
  :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>` and
  :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>`.

* :ouster:class:`ClassMap <py=ouster.sdk.core.ClassMap|cpp=ouster::sdk::core::ClassMap>`
  — A mapping from integers to strings, meant to map ``Object::class_id`` to a string description.

* :ouster:class:`ClassMapSet <py=ouster.sdk.core.ClassMapSet|cpp=ouster::sdk::core::ClassMapSet>` — A mapping from strings to
  ``ClassMap`` instances, allowing users to specify different descriptions for ``Object`` instances derived from different
  ``DetectionEngine``.

* :ouster:class:`FrameSetSourceMetadataSet <py=ouster.sdk.core.FrameSetSourceMetadataSet|cpp=ouster::sdk::core::FrameSetSourceMetadataSet>`
  — A container for instances of ``ClassMapSet`` and other metadata a user may want to associate with a ``FrameSetSource``.
  Currently, because OSF is the only format that supports ``LidarFrame`` and ``FrameSet``, the
  :ouster:class:`OsfFrameSetSource <py=ouster.sdk.osf.OsfFrameSetSource|cpp=ouster::sdk::osf::OsfFrameSetSource>` is the only
  implementation that supports metadata.

.. perception-core-types-end

For detailed type definitions and end-to-end examples, see :doc:`API workflow <using-api>` for application integration, custom
logic, and programmatic control.

For fast iteration, using CLI commands, to dataset generate and vizualize data refer to :doc:`CLI workflow <using-cli>`.

Coordinate Frames
-----------------

The SDK represents all spatial relationships through a chain of named coordinate frames connected by
SE(3) transforms. An *SE(3) transform* is a 4x4 matrix holding a 3D rotation and a 3D translation; it
moves a point from one frame into another without changing the object's shape or size.

Each transform is named ``a_to_b``, which means it takes a point's coordinates in frame ``a`` and gives
them back in frame ``b``. Transforms are combined by multiplying their matrices (right-to-left), so a
two-step hop ``a`` → ``b`` → ``c`` is:

.. math::

   T_{a}^{c} = T_{b}^{c} \cdot T_{a}^{b}

To turn a raw measurement into a world point, the SDK walks the chain ``lidar`` → ``sensor`` → ``body``
→ ``world``. Every transform can also be inverted, so you can travel the chain in either direction.

Every edge is either
**static** — a single time-invariant matrix that is the same for every frame in a recording —
or **dynamic** — a timestamped sequence of matrices that is interpolated (spherical-linear for
the rotation, linear for the translation) to whatever time you ask for. The four
kinds of transform are:

- **Firmware calibrated** *(static)* — the sensor's internal optics and IMU geometry. These
  come from the per-unit factory calibration shipped in the sensor metadata, so they are
  accurate to the individual sensor and never change over its lifetime.
- **Per-mounting configuration** *(static)* — how each sensor is positioned on the platform.
  Supplied once by the user or a calibration routine, this is the piece that lets several
  independently-calibrated sensors agree on a single shared ``body`` frame; it changes only if
  the hardware is physically re-mounted.
- **Per-detection snapshot** *(static)* — a detected object's placement relative to ``body``.
  Each object stores its own matrix, so the value differs from object to object, but for any
  single detection it is one fixed pose rather than a trajectory.
- **Dynamic trajectory** *(dynamic)* — the platform's ego motion through the ``world`` frame.
  This is the only quantity that evolves as the recording plays. ``LidarFrame::body_to_world``
  stores the full timestamped trajectory, one matrix per lidar column; and ``Object::body_to_world``
  is a single matrix sampled from that trajectory at the object's detection timestamp.

.. figure:: /images/sdk_pose_graph.svg
   :alt: SDK pose graph: coordinate frames and transforms
   :width: 100%

   SDK pose graph: coordinate frames and transforms

The key coordinate frames used in this pose graph are:

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Frame
     - Description
   * - ``beam``
     - Raw lidar optics frame, where each laser beam originates. An internal frame, brought into the
       ``lidar`` frame via ``beam_to_lidar_transform``.
   * - ``lidar``
     - Lidar measurement frame. A fixed, hardware-calibrated frame defined by the sensor's internal geometry.
       Transformed to the sensor housing frame via ``lidar_to_sensor_transform``.
   * - ``sensor``
     - Sensor housing frame. The external coordinate system of one physical sensor unit. Each sensor on the
       platform has its own ``sensor`` frame, transformed to the shared body frame via its own ``sensor_to_body``
       (per mounting configuration).
   * - ``imu``
     - IMU measurement frame. Related to the sensor frame by the fixed ``imu_to_sensor_transform``.
   * - ``body``
     - The single common reference frame shared by all sensors mounted on the
       platform; measurements from every sensor are fused here.
   * - ``object``
     - Object-local bounding box frame. The origin is at the center of the detected object's bounding box,
       with axes aligned to the object's principal directions. ``object_to_body`` is a per-detection snapshot:
       fixed for a given ``Object`` instance but different across frames as the object moves relative to ``body``.
   * - ``world``
     - Sequence-local world frame. A fixed reference frame for a recording session; typically the body frame
       at the start of the sequence. The dynamic ``body_to_world`` trajectory expresses the ego-vehicle pose
       in this frame over time. It stores one SE(3) per lidar column (not just one per frame) because the
       vehicle keeps moving during the lidar's sweep (one full frame, e.g. ~100 ms at 10 Hz); using the
       right per-column pose removes that motion smear — a step known as *dewarping* (see :doc:`/features/mapping/motion-compensation`).

All of these frames are right-handed. For the exact axis directions and origins of the Ouster
``sensor``, ``lidar``, and ``imu`` frames, see the `Ouster sensor coordinate frame documentation
<https://static.ouster.dev/sensor-docs/image_route1/image_route3/sensor_data/sensor-data.html#coordinate-frames-and-xyz-calculation>`_.

Where poses are stored
^^^^^^^^^^^^^^^^^^^^^^^

Transforms are owned by three data structures:

- :ouster:class:`SensorInfo <py=ouster.sdk.core.SensorInfo|cpp=ouster::sdk::core::SensorInfo>` (one per sensor): ``lidar_to_sensor_transform``, ``imu_to_sensor_transform``, and ``sensor_to_body``.
- :ouster:class:`LidarFrame <py=ouster.sdk.core.LidarFrame|cpp=ouster::sdk::core::LidarFrame>`: dynamic ``body_to_world()`` ego trajectory, one SE(3) per lidar column (shape ``(w, 4, 4)``).
- :ouster:class:`Object <py=ouster.sdk.core.Object|cpp=ouster::sdk::core::Object>`: per-detection ``object_to_body`` and ``body_to_world``.
  ``body_to_world`` is sampled from ``LidarFrame`` at ``Object.timestamp``, making each object self-contained.

**Object world pose** is computed by composing the two ``Object`` pose fields (both stored on the ``Object`` itself):

.. math::

   T_{\text{object}}^{\text{world}} = T_{\text{body}}^{\text{world}} \cdot T_{\text{object}}^{\text{body}}

In code: ``object_in_world = object.body_to_world * object.object_to_body``

The ``velocity`` field of an :ouster:class:`Object <py=ouster.sdk.core.Object|cpp=ouster::sdk::core::Object>` is expressed in the ``world`` frame.


.. toctree::
   :maxdepth: 1
   :hidden:

   using-cli
   using-api
   normals/index
   ground_seg/index

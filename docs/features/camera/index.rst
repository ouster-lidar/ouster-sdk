.. _camera-data:

Camera Data
===========

In addition to lidar, the Ouster SDK's frame-based API can represent data from
cameras. The camera API mirrors the lidar API closely, so if you are already
familiar with :doc:`LidarFrame </features/consumption/lidar_frame>` and
``FrameSet``, working with camera data will feel familiar.

There are three core types:

- ``CameraInfo`` — metadata describing a camera: its image size, extrinsics,
  and an optional ``CameraModel``. This is the camera counterpart to
  ``LidarInfo``.
- ``CameraModel`` — the camera's pinhole intrinsics (``fx``, ``fy``, ``cx``,
  ``cy``).
- ``CameraFrame`` — a single frame of camera data with named image fields (for
  example ``LEFT_RGB`` or ``RANGE``). This is the camera counterpart to
  ``LidarFrame``.

Before you fuse imagery with range data into a point cloud, read
:ref:`camera-projection-requirements` below. Two constraints there — rectified
imagery and radial range units — are not checked at runtime, and violating
either produces a wrong cloud that still looks plausible.

Similarities with the LidarFrame API
------------------------------------------

``CameraFrame`` and ``LidarFrame`` share a common ``Frame`` base class, and
``CameraInfo`` and ``LidarInfo`` share a common ``SensorInfo`` base class. As a
result:

- Frames expose the same interface: ``w`` / ``h``, named fields accessed with
  ``add_field`` / ``field``, per-frame timestamps, and ``body_to_world`` poses.
- A single ``FrameSet`` can hold both lidar and camera frames, so one source may
  combine multiple sensors of either kind.
- The same tools apply to both. You iterate frames from any source, collate
  frames across sensors, and project range images into 3D points with the same
  ``XYZLut``.

Integrating a camera
--------------------------

The SDK defines the camera data model but does not talk to any particular
camera. Reading frames from a specific camera and populating ``CameraInfo`` and
``CameraFrame`` — and, if you wish, saving them — is up to you.

.. _camera-projection-requirements:

Projecting camera data into a point cloud
-----------------------------------------

Fusing a camera's ``RANGE`` field with its imagery to produce a point cloud
requires a calibrated, rectified camera and range in the right units. The three
constraints below are what the SDK assumes; only the first is enforced.

.. warning::

   **Without a** ``CameraModel``, **imagery can be visualized but not
   projected.** ``CameraInfo.model`` is optional, and leaving it unset is a
   legitimate way to record a camera whose intrinsics you do not know: images
   render, frames collate, and timestamps and poses round-trip normally.
   An ``XYZLut`` constructed from such a ``CameraInfo`` is empty, and applying
   it to a frame raises, rather than guessing a focal length and returning a
   cloud that is wrong by an unknown scale factor.

.. warning::

   **Only rectified imagery is supported.** ``CameraModel`` carries pinhole
   intrinsics and no distortion coefficients, and the projection path applies
   no undistortion. Setting a ``CameraModel`` is therefore an assertion that
   the imagery it describes is already rectified. If your camera reports both a
   raw and a rectified calibration, use the rectified one, and rectify the
   images before handing them to the SDK. Feeding distorted imagery through
   projection produces a quietly warped cloud with no error or warning.

.. warning::

   **RANGE must be radial millimeters, not planar depth.** ``RANGE`` is the
   distance from the camera origin to the point, matching the lidar
   convention. Many depth cameras instead report *planar* depth — the distance
   along the optical axis — and the two agree only at the image center. Storing
   planar depth as ``RANGE`` produces a cloud that looks reasonable but is
   increasingly wrong toward the image edges. Convert first:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/depth_to_range.py
         :language: python
         :start-after: [doc-stag-depth-to-range]
         :end-before: [doc-etag-depth-to-range]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/camera/_snippets/python/depth_to_range.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/depth_to_range.cpp
         :language: cpp
         :start-after: //! [doc-stag-depth-to-range]
         :end-before: //! [doc-etag-depth-to-range]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/camera/_snippets/cpp/depth_to_range.cpp>`__
         :dedent: 4

Ouster Sensor Format
--------------------------

Camera data is written and replayed using the same :ref:`OSF <osf>` APIs as
lidar data. The examples below mirror the lidar OSF examples.

.. rubric:: Writing camera frames

.. code-block:: python

    import numpy as np
    import ouster.sdk.core as core
    import ouster.sdk.osf as osf

    # Describe the camera (compare with LidarInfo for lidar).
    info = core.CameraInfo()
    info.width = 640
    info.height = 480
    # Optional: omit it and the imagery is viewable but not projectable.
    info.model = core.CameraModel(fx=600.0, fy=600.0, cx=320.0, cy=240.0)

    # Build a frame of camera data (compare with LidarFrame).
    frame = core.CameraFrame()
    frame.w = info.width
    frame.h = info.height
    frame.add_field("LEFT_RGB",
                    np.zeros((info.height, info.width, 3), dtype=np.uint8))

    with osf.Writer("camera.osf") as writer:
        writer.add_sensor(info)        # register the camera stream
        writer.save(0, frame, 100)     # (stream index, frame, timestamp ns)

.. rubric:: Reading camera frames

.. code-block:: python

    import ouster.sdk.core as core
    from ouster.sdk import open_source

    source = open_source("camera.osf")
    for frame_set in source:
        for frame in frame_set.valid_camera_frames():
            # Per-frame timestamp and body_to_world pose both round-trip
            # through OSF, just like a LidarFrame. A camera frame has no
            # frame_id (that is a LidarFrame member); identify it by timestamp.
            print(f"camera frame {frame.w}x{frame.h}, ts={frame.timestamp[0]}, "
                  f"fields={list(frame.fields)}")
    source.close()

Frames of any type can also be printed directly with ``str(frame)`` (or
``ouster::sdk::core::to_string(frame)`` in C++), which prints a ``LidarFrame``
or ``CameraFrame`` according to the frame's concrete type — handy when iterating
``frame_set.valid_frames()`` over a mixed source.

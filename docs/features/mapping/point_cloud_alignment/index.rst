=====================
Point Cloud Alignment
=====================

Point Cloud Alignment estimates the rigid transform between overlapping lidar
frames or point clouds. It can also align a multi-sensor ``FrameSet`` to
sensor 0, producing an extrinsic matrix for each sensor.

Users can calibrate the extrinsics of a multi-sensor setup, register two lidar
frames, or align two point clouds before combining or comparing them. Use the
``align`` CLI command for multi-sensor sources, or call
:ouster:func:`align_clouds <py=ouster.sdk.algorithm.align_clouds|cpp=ouster::sdk::algorithm::align_clouds>`
from Python or C++ for frame-set, pairwise-frame, or point-cloud alignment.

The SDK uses a coarse-to-fine pipeline: it first searches over yaw and
translation using FFT and histogram correlation, then refines the result with
multi-pass ICP. This larger capture range makes it more robust than ICP-only
alignment when the input point clouds start farther apart or have a larger
initial yaw offset.

Using the CLI
=============

Use a multi-sensor source with the ``align`` command. It matches the point
clouds from all available sensors when they have sufficient overlapping
coverage, estimates the transformations that align them to sensor 0, and
applies those transformations as the updated sensor extrinsics.

Chain ``viz`` to inspect the aligned point clouds. The command also prints the
resulting extrinsic matrices:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-align-viz]
   :end-before: [doc-etag-cli-align-viz]
   :dedent: 0
   :class: doc-snippet

For the best results, ``align`` expects the input sensor extrinsics to be
aligned with gravity. If the source still contains IMU packets from sensors
running FW 3.2 or later, run ``plumb`` before ``align`` to calculate the
gravity-aligned extrinsics:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-plumb-align-viz]
   :end-before: [doc-etag-cli-plumb-align-viz]
   :dedent: 0
   :class: doc-snippet

Chain ``save`` to write the frames and updated extrinsics to a new OSF file:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-align-save]
   :end-before: [doc-etag-cli-align-save]
   :dedent: 0
   :class: doc-snippet

Using the API
=============

Use :ouster:func:`align_clouds <py=ouster.sdk.algorithm.align_clouds|cpp=ouster::sdk::algorithm::align_clouds>`
for pairwise frame alignment, point-cloud alignment, and multi-sensor extrinsic
calibration.

Frame inputs must have gravity-aligned ``sensor_info.sensor_to_body`` rotations.
The standard frame and frame-set alignment paths use the full existing
``sensor_info.sensor_to_body`` matrix, including translation, as the initial
extrinsic prior and estimate a correction from there. Point-cloud inputs must
already be in a gravity-aligned common frame.

Both overloads return rigid 4 x 4 transformation matrices, but the result
depends on the input:

* ``align_clouds(source_frame, target_frame)`` returns
  ``source_to_target_transform``, which aligns the source frame to the target
  frame.
  The target frame is the fixed reference. To calculate the aligned source
  extrinsic, left-multiply the source frame's existing extrinsic by the returned
  transform.
* ``align_clouds(frameset)`` returns one aligned extrinsic matrix for each frame
  in a multi-sensor ``FrameSet``. Frame 0 is the reference and must be valid.
  Every other valid frame is aligned to frame 0, and the returned transforms can
  be used as updated sensor extrinsics.

Each returned multi-sensor transform is written back to the same index as its
input frame, so ``aligned_extrinsics[i]`` corresponds to ``frames[i]``. In
Python, the result has shape ``(N, 4, 4)``; in C++, it is a vector of 4 x 4
matrices.

Use ``align_clouds(source_frame, target_frame)`` to align two ``LidarFrame``
objects:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/align_clouds.py
         :language: python
         :start-after: [doc-stag-alignment-pairwise]
         :end-before: [doc-etag-alignment-pairwise]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/point_cloud_alignment/_snippets/python/align_clouds.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/align_clouds_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-alignment-pairwise]
         :end-before: //! [doc-etag-alignment-pairwise]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/point_cloud_alignment/_snippets/cpp/align_clouds_example.cpp>`__
         :dedent: 4

Use ``align_clouds(frameset)`` to align every valid frame in a ``FrameSet`` to
frame 0, for example when calibrating sensor extrinsics:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/align_clouds.py
         :language: python
         :start-after: [doc-stag-alignment-frame-set]
         :end-before: [doc-etag-alignment-frame-set]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/point_cloud_alignment/_snippets/python/align_clouds.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/align_clouds_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-alignment-frame-set]
         :end-before: //! [doc-etag-alignment-frame-set]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/point_cloud_alignment/_snippets/cpp/align_clouds_example.cpp>`__
         :dedent: 4

.. _ground-segmentation:

===================
Ground Segmentation
===================

Ground Segmentation classifies lidar returns as ground or non-ground. The
result is stored directly on each frame as a ``uint8`` mask: ``1`` means ground
and ``0`` means non-ground. The first-return mask is named ``GROUND`` and the
optional second-return mask is named ``GROUND2``.

Users can visualize the mask, remove ground points before object detection or
mapping, or reuse it during point cloud alignment. Use the ``ground`` CLI
command to annotate a stream of frames, or use
:ouster:class:`GroundSegEngine <py=ouster.sdk.algorithm.GroundSegEngine|cpp=ouster::sdk::algorithm::GroundSegEngine>`
to add the mask inside a Python or C++ processing pipeline.

When used with ``viz``, the visualizer can highlight the segmented ground in
the point cloud while keeping the image panels available for comparison:

.. figure:: /images/ground_seg_in_viz.png
   :align: center
   :alt: A screenshot of Ouster Viz displaying ground segmentation results in the point cloud view
   :width: 100%

   Ouster Viz displaying the output of the ``ground`` command.

Using the CLI
=============

The chainable ``ground`` command segments every frame and makes the mask
available to downstream commands such as ``viz``:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-ground-viz]
   :end-before: [doc-etag-cli-ground-viz]
   :dedent: 0
   :class: doc-snippet

For the best results, ``ground`` expects the input sensor extrinsics to be
aligned with gravity. If the recording contains IMU data from sensors running
FW 3.2 or later, run ``plumb`` before ``ground`` to calculate the
gravity-aligned extrinsics:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-plumb-ground-viz]
   :end-before: [doc-etag-cli-plumb-ground-viz]
   :dedent: 0
   :class: doc-snippet

For moving recordings, especially handheld recordings, run ``slam`` before
ground segmentation. SLAM supplies a pose for each frame column so the ground
model is built in a consistent global frame:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-plumb-slam-ground-viz]
   :end-before: [doc-etag-cli-plumb-slam-ground-viz]
   :dedent: 0
   :class: doc-snippet

Using the API
=============

Create a
:ouster:class:`GroundSegEngine <py=ouster.sdk.algorithm.GroundSegEngine|cpp=ouster::sdk::algorithm::GroundSegEngine>`
and pass each ``FrameSet`` to :ouster:func:`update() <py=ouster.sdk.algorithm.GroundSegEngine.update|cpp=ouster::sdk::algorithm::GroundSegEngine::update>`.
The engine modifies the frames in place but does not calculate gravity
alignment. Before reading frames, assign each sensor a
``SensorInfo.sensor_to_body`` whose frame is aligned with gravity. In this
frame, ground surface normals should point along the positive Z axis. The
examples assume ``plumb_extrinsics`` contains these gravity-aligned
transforms.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/ground_seg.py
         :language: python
         :start-after: [doc-stag-ground-seg-api]
         :end-before: [doc-etag-ground-seg-api]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/perception/ground_seg/_snippets/python/ground_seg.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/ground_seg_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-ground-seg-api]
         :end-before: //! [doc-etag-ground-seg-api]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/perception/ground_seg/_snippets/cpp/ground_seg_example.cpp>`__
         :dedent: 4

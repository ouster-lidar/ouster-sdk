.. _normals:

========
Normals
========

Surface normals are unit vectors perpendicular to locally estimated surfaces
in a lidar point cloud. The SDK computes them from neighboring lidar returns
and estimates one normal per valid return from destaggered XYZ and range
images.

Users can add normals to frames for visualization, save them for later
processing, or pass them to algorithms such as point cloud registration,
surface analysis, and filtering. Use the ``normals`` CLI command to process a
stream of frames, or call the
:ouster:func:`normals <py=ouster.sdk.algorithm.normals|cpp=ouster::sdk::algorithm::normals>`
API when integrating normal estimation into an application.

When used with ``viz``, the visualizer can color the point cloud by the
computed normals while also showing the corresponding image view:

.. figure:: /images/normals_value_in_viz.png
   :align: center
   :alt: A screenshot of Ouster Viz displaying normals values in the point cloud and image panels
   :width: 100%

   Ouster Viz displaying normals values after running the ``normals`` command.

Using the CLI
=============

The chainable ``normals`` command computes normals for every frame. It adds a
``NORMALS`` field for the first return and a ``NORMALS2`` field when the second
return is available.

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-normals-viz]
   :end-before: [doc-etag-cli-normals-viz]
   :dedent: 0
   :class: doc-snippet

By default, the command dewarps points using the frame poses and expresses the
normals in the global coordinate frame. Use ``--sensor-coord`` to compute them
in the sensor coordinate frame:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-normals-sensor-coord-viz]
   :end-before: [doc-etag-cli-normals-sensor-coord-viz]
   :dedent: 0
   :class: doc-snippet

Using the API
=============

The
:ouster:func:`normals <py=ouster.sdk.algorithm.normals|cpp=ouster::sdk::algorithm::normals>`
function expects destaggered XYZ and range arrays. XYZ points and sensor origins
must use the same coordinate frame. The examples below use the sensor coordinate
frame, so every sensor origin is ``(0, 0, 0)``. See
:doc:`../../processing/using-the-api` for creating XYZ data and destaggering
lidar fields. ``LidarFrame`` fields are stored in staggered layout, so the
examples stagger the computed normals before writing the ``NORMALS`` field.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/normals.py
         :language: python
         :start-after: [doc-stag-normals-api]
         :end-before: [doc-etag-normals-api]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/perception/normals/_snippets/python/normals.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/normals_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-normals-api]
         :end-before: //! [doc-etag-normals-api]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/perception/normals/_snippets/cpp/normals_example.cpp>`__
         :dedent: 4

Invalid returns, or returns for which a normal cannot be estimated, contain a
zero vector.

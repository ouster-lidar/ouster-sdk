Using the CLI
==============

The ``localize`` command was introduced in SDK 0.14.0. It loads a pre-built point cloud map,
streams lidar data from a sensor or recording, and continuously estimates the sensor's pose
relative to the map.

Basic Usage
-----------

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-basic]
   :end-before: [doc-etag-localize-basic]
   :dedent: 0

Replace ``{SOURCE}`` with a sensor hostname/IP, or a path to a PCAP or OSF file.
The ``{MAP}`` argument is a PLY (or PCD) file produced by a prior SLAM run (see
:ref:`ouster-cli-mapping-save`).

When you append ``viz`` after the map path, the Ouster Visualizer loads the reference map in the
background (flattened by default), streams lidar from the source, and updates the sensor pose
relative to the map origin:

.. figure:: ../mapping/_snippets/images/localization-map-main.png
   :align: center
   :alt: Localization visualizer with reference map in the background
   :width: 100%

.. note::

   Map files must be point clouds in PLY or PCD format.

You can also chain ``save output.osf`` to record the localized stream:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-save]
   :end-before: [doc-etag-localize-save]
   :dedent: 0

``localize`` Command Reference
------------------------------

To see all available options:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-help]
   :end-before: [doc-etag-localize-help]
   :dedent: 0

.. list-table::
   :header-rows: 1
   :widths: 28 15 57

   * - Flag
     - Default
     - Description
   * - ``map_path`` (positional)
     - *(required)*
     - Path to the point cloud map file (PLY or PCD).
   * - ``--min-range``
     - ``0.0``
     - Lower limit of range measurements in meters. Points closer than this are excluded.
   * - ``--max-range``
     - ``150.0``
     - Upper limit of range measurements in meters. Points beyond this are excluded.
   * - ``-v`` / ``--voxel-size``
     - auto
     - Map voxel size in meters. When omitted (or ``0.0``), the engine estimates a suitable size.
   * - ``--deskew-method``
     - ``auto``
     - Deskewing strategy: ``auto``, ``none``, ``constant_velocity``, or ``imu_deskew``.

.. _setting-the-initial-pose:

Setting the Initial Pose
^^^^^^^^^^^^^^^^^^^^^^^^

The above commands work well when the input source begins from the same place as the origin of the map,
however, in many situations this isn't the case. In the case of wanting to start from a different starting point
than the map origin, the user could do that using the following:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-initpose]
   :end-before: [doc-etag-localize-initpose]
   :dedent: 0

This would set the initial pose of the input source with respect to the map origin,
where ``PX,PY,PZ``  represent the position and R,P,Y represent orientation in Euler angles (roll, pitch, yaw) specified in degrees.

When not specified, the initial pose defaults to the identity matrix (the map origin).

For the Python API, see :ref:`setting-the-initial-pose` in :doc:`using-the-api`.

Tuning Range and Voxel Size
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Adjusting range gates removes irrelevant points (e.g., the robot's own frame at close range, or
noisy returns at long range):

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-ranges]
   :end-before: [doc-etag-localize-ranges]
   :dedent: 0

Setting the voxel size explicitly gives you control over the map resolution used for matching:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-voxel]
   :end-before: [doc-etag-localize-voxel]
   :dedent: 0

Selecting the Deskew Method
^^^^^^^^^^^^^^^^^^^^^^^^^^^

On sensors running FW >= 3.2, ``auto`` uses IMU-based deskewing for more accurate motion
compensation. You can override this:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-deskew]
   :end-before: [doc-etag-localize-deskew]
   :dedent: 0


.. _reference-map-appearance:

Reference map appearance
------------------------

The localization ``viz`` command accepts options prefixed with ``--global-map-`` to control how
the static map is drawn. For example, show the map in 3D without flattening:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-flatmap]
   :end-before: [doc-etag-localize-flatmap]
   :dedent: 0

You can combine several ``--global-map-*`` flags—for example Z clipping, downsampling, and point
size:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-localize-globalmap-opts]
   :end-before: [doc-etag-localize-globalmap-opts]
   :dedent: 0

The full list of ``--global-map-*`` flags:

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Flag
     - Default
     - Description
   * - ``--global-map``
     - *(from localize)*
     - Override the map file displayed in the visualizer. This does **not** affect the map used for
       registration — only the visual display.
   * - ``--global-map-flatten``
     - ``True``
     - When ``True``, projects the map onto the XY plane (Z = 0). Set to ``False`` for full 3D.
   * - ``--global-map-min-z``
     - *(none)*
     - Filter out map points below this Z value (meters).
   * - ``--global-map-max-z``
     - *(none)*
     - Filter out map points above this Z value (meters).
   * - ``--global-map-voxel-size``
     - *(none)*
     - Downsample the displayed map using this voxel size (meters). Reduces visual clutter and
       improves rendering performance for large maps.
   * - ``--global-map-point-size``
     - ``1.0``
     - Point size for the rendered map cloud.

To see every flag directly, run ``ouster-cli source <SOURCE_URL> localize map.ply viz --help``.

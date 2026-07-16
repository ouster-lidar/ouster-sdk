Using the Visualizer
====================

After :ref:`installing <installation>` the ``ouster-sdk`` package, the following visualizes lidar data arriving on a udp port. Note that you may have to use
``ouster-cli source <SENSOR_HOSTNAME> config`` first to configure your sensor properly.

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-viz-sensor]
   :end-before: [doc-etag-cli-viz-sensor]
   :class: doc-snippet
   :dedent: 0

where ``<sensor hostname>`` is the hostname (os-99xxxxxxxxxx) or IP of the sensor.

Alternately, the following replays lidar data saved in a pcap file and visualizes the output. It will looks for a
metadata json file with the same name as PCAP FILE by default, but you can specify a file using ``-m
<METADATA JSON>``.

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-viz-pcap]
   :end-before: [doc-etag-cli-viz-pcap]
   :class: doc-snippet
   :dedent: 0

Use extrinsics input
--------------------

The visualizer also includes an option to control the orientation of the point cloud in space when
loaded. If you possess, say, an OS-DOME mounted upside down, you can start the visualizer with
the option ``--extrinsics`` to adjust the visualization accordingly:


.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-viz-extrinsics]
   :end-before: [doc-etag-cli-viz-extrinsics]
   :dedent: 0


The input is a row-major homogeneous matrix.

For other options, run ``ouster-cli source <sensor | pcap | osf> viz -h``

.. note::

   All basic primitives that you see as part of ``ouster-cli`` visualizer are exposed through
   :class:`.viz.PointViz` bindings. Please see the API reference for :py:class:`.viz.PointViz` for how to use it
   programmatically in Python.


.. _adjust-playback-rate-and-looping:

Adjust Playback Rate and Looping
--------------------------------

To visualize the OSF at 1.5x speed while looping back:

.. literalinclude:: ../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-viz-osf]
   :end-before: [doc-etag-cli-viz-osf]
   :dedent: 0

Advanced usage with sensor
--------------------------

The Ouster visualizer automatically configures connected sensors to send data to the appropriate UDP
destination address. If your sensor is already configured appropriately, you may find it useful to
use the argument ``--no-auto-udp-dest`` to save time by skipping the round trip to reconfigure the
sensor.


Streaming Live Data
--------------------

Instead of working with a recorded dataset or a few captured frames of data, let's see if we can get
a live feed from your :ref:`configured<ex-configure-sensor>` sensor:

.. literalinclude:: /../python/src/ouster/sdk/examples/core.py
   :start-after: [doc-stag-live-plot-reflectivity]
   :end-before: [doc-etag-live-plot-reflectivity]
   :emphasize-lines: 2-3
   :dedent:

To exit the visualization, you can use ``ESC``.

Here, we are using ``core.destagger`` to undo the azimuth pixel shifts in ``REFLECTIVITY`` channel. We will learn more about this in the  :doc:`next section <../processing/index>`.

Visualizing LidarFrame(s)
------------------------

If you want to visualize a ``LidarFrame`` or a list of ``LidarFrame`` objects using the python API, you can use
:meth:`ouster.sdk.viz.lf_show` method. Here is a code snippet that shows how it can be used to
visualize a list of ``LidarFrame``:

.. literalinclude:: _snippets/python/playback_viz.py
   :language: python
   :start-after: [doc-stag-viz-frame-show]
   :end-before: [doc-etag-viz-frame-show]
   :dedent: 4

    
This will open an interactive window displaying the ``LidarFrame`` as PointCloud.


Additionally, ``lf_show`` accepts a list or slice of ``LidarFrame(s)``.

To visualize multiple frames simultaneously:

.. literalinclude:: _snippets/python/playback_viz.py
   :language: python
   :start-after: [doc-stag-viz-frame-show-list]
   :end-before: [doc-etag-viz-frame-show-list]
   :dedent: 4

This example passes a list of frames to ``lf_show`` so they appear in the visualization window at the same time.
You can toggle each frame on/off within the visualizer using ``CTRL+1`` through ``CTRL+9`` to compare frames side by side. These
frames do not need to originate from the same source.

To visualize a range of frames from the frame source sequentially:

.. literalinclude:: _snippets/python/playback_viz.py
   :language: python
   :start-after: [doc-stag-viz-frame-show-slice]
   :end-before: [doc-etag-viz-frame-show-slice]
   :dedent: 4

Here ``stop`` controls how many frames are included in the range. When played back in visualizer you can press
the ``spacebar`` to unpause the playback or use the ``<`` and ``>`` keys to step back and forth between frames.


.. figure:: /images/viz-tutorial/lidar_frame_viz_checkers.png
    :align: center

    ``PointViz`` with ``Image``, ``Label`` and masks applied

For more advanced visualization you can explore a programmatically accessible :class:`.viz.PointViz`
API.
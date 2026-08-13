Visualize
=========

Ouster’s OpenGL-based visualizer allows for easy visualization from pcaps and sensors on all platforms the Ouster SDK supports.

Using ``ouster-cli`` is a fastest way to visualize data from a connected sensor, recorded ``pcap``
or OSF files with SLAM poses:

If you've installed the ``ouster-sdk`` (see :ref:`Python Installation <installation-python>`) then
you're all set to visualize with below command. 

The following replays lidar data saved in a pcap file and visualizes the output. It will look for a
metadata json file with the same name as the PCAP file by default, but you can specify the metadata
explicitly. Set ``SAMPLE_DATA_PCAP_PATH`` (and optionally ``SAMPLE_DATA_JSON_PATH`` using ``-m`` if your metadata
is stored separately) before running:

.. literalinclude:: ../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-viz-from-pcap]
   :end-before: [doc-etag-viz-from-pcap]

.. _using-ouster-sensor:

Using an Ouster Sensor
----------------------

The following visualizes lidar data arriving on a udp port.

.. note::

   Connecting to an Ouster sensor is covered in the `Networking Guide`_ section of the Ouster
   Sensor Documentation.

Set ``SENSOR_HOSTNAME`` to your sensor's hostname or IP before running:

.. literalinclude:: ../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-viz-from-sensor]
   :end-before: [doc-etag-viz-from-sensor]


.. note::

   To make sure everything is connected, open a separate console window and try pinging the sensor. You
   should see some output like:

   .. tab-set::
      :sync-group: os-commands

      .. tab-item:: Linux/macOS
         :sync: unix

         .. code::

            $ ping -c1 <SENSOR_HOSTNAME>
            PING <SENSOR_HOSTNAME> (192.0.2.42) 56(84) bytes of data.
            64 bytes from <SENSOR_HOSTNAME> (192.0.2.42): icmp_seq=1 ttl=64 time=0.217 ms
      
      .. tab-item:: Windows x64
         :sync: windows

         .. code::

            PS > ping /n 10 <SENSOR_HOSTNAME>
            Pinging <SENSOR_HOSTNAME> (192.0.2.42) with 32 bytes of data:
            Reply from 192.0.2.42: bytes=32 time=101ms TTL=124


Note that you may have to use ``ouster-cli source $SENSOR_HOSTNAME config`` first to configure your sensor properly.

You should get a view similar to:

.. figure:: /images/ouster-viz.png
   :align: center
   :width: 100%

   Ouster SDK CLI ``ouster-cli source <sensor | pcap | osf> viz`` visualization of OS1 128 sample data

You can control your visualizer with mouse and keyboard.

- Click and drag rotates the view.
- Middle click and drag moves the view.
- Scroll adjusts how far away the camera is from the vehicle.

The default Ouster SDK CLI ``ouster-cli source <sensor | pcap | osf> viz`` visualizer view includes two 2D range images atop which can be cycled through the available fields, and a 3D point cloud on the bottom. For dual returns sensors, both returns are displayed by default.

Visualizer options
--------------------

To get a list of all the options supported by the ``ouster-cli source … viz`` command, run:

.. literalinclude:: ../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-viz-help]
   :end-before: [doc-etag-viz-help]
   :dedent: 0


.. _simple-viz-keymap:

..
   [start-simple-viz-keymap]

Keyboard controls
^^^^^^^^^^^^^^^^^

Press ``?`` while the visualizer window is focused to print the current key bindings. The table
below summarizes the defaults:

**Camera**
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``shift``        Camera translation with mouse drag
    ``w``            Camera pitch down
    ``s``            Camera pitch up
    ``a``            Camera yaw right
    ``d``            Camera yaw left
    ``q``            Camera roll left
    ``e``            Camera roll right
    ``shift+r``      Reset camera orientation
    ``ctrl+r``       Set camera to the birds-eye view
    ``shift+1``      Top-down view
    ``shift+2``      Front-facing view
    ``shift+3``      Left-facing view
    ``u``            Cycle camera mode
    ``= / -``        Dolly in/out
    ``0``            Toggle orthographic camera
    ``ctrl+- / ctrl+=`` Increase/decrease field of view (perspective only)
    ================ ===============================================

**Playback**
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``space``        Toggle pause
    ``. / ,``         Step one frame forward/back
    ``ctrl + . / ,`` Step 10 frames forward/back
    ``> / <``        Increase/decrease playback rate (during replay)
    ``h / shift+h``  Increase/decrease subframes (during replay)
    ================ ===============================================

**2D View**
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``b / B``        Cycle top 2D image
    ``n / N``        Cycle bottom 2D image
    ``i / shift+i``  Increase/decrease size of displayed 2D images
    ``ctrl+i``       Flip or hide 2D images
    ================ ===============================================

**3D View**
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``p / P``        Increase/decrease point size
    ``m / M``        Cycle point cloud coloring mode
    ``f / F``        Cycle point cloud color palette
    ``ctrl+1`` … ``ctrl+9`` Toggle sensor 1–9 point cloud(s)
    ``1``            Toggle first return point cloud visibility
    ``2``            Toggle second return point cloud visibility
    ``6``            Toggle frames accumulation view mode (ACCUM)
    ``7``            Toggle overall map view mode (MAP)
    ``8``            Toggle track view mode
    ``9``            Toggle axis helpers at frame origin
    ``' / shift+'``  Increase/decrease spacing in range markers
    ``ctrl + '``     Cycle through thickness of range markers or hide
    ``c``            Cycle current highlight mode
    ``j / J``        Increase/decrease point size of accumulated clouds or map
    ``k / K``        Cycle point cloud coloring mode of accumulated clouds or map
    ``g / G``        Cycle point cloud color palette of accumulated clouds or map
    ================ ===============================================

**Other**
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``o``            Toggle on-screen display
    ``?``            Print keys to standard out
    ``shift+z``      Save a screenshot of the current view
    ``shift+x``      Toggle a continuous saving of screenshots
    ``ctrl+x``       Toggle continuous screenshot format
    ``v / shift+v``  Cycle screenshot resolution factor
    ``esc``          Exit
    ================ ===============================================

..
   [end-simple-viz-keymap]

Extrinsics
----------

The visualizer includes an option to control the orientation of the point cloud in space when
loaded. If you possess, say, an OS-DOME mounted upside down, you can start the visualizer with the
option ``--extrinsics``:

.. literalinclude:: ../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-viz-with-extrinsics]
   :end-before: [doc-etag-viz-with-extrinsics]
   :dedent: 0

The input is a row-major homogeneous matrix.

.. note::

   All basic primitives that you see as part of ``ouster-cli`` visualizer are exposed through
   :py:class:`.viz.PointViz` bindings. Please see the API reference for :py:class:`.viz.PointViz` for how to use it
   programmatically in Python.


Subframes
^^^^^^^^^

When replaying recorded data, pass ``--subframes`` to interpolate the camera pose between frames and
visualize smoother motion. If the value is greater than zero, the visualizer linearly interpolates
between consecutive frames to produce that many intermediate frames per frame. You can also press
``r`` or ``SHIFT+r`` while the visualizer is running to increase or decrease the subframe count.

Set ``SAMPLE_DATA_PCAP_PATH`` before running:

.. literalinclude:: ../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-viz-subframes]
   :end-before: [doc-etag-viz-subframes]
   :dedent: 0


Advanced usage with sensor
--------------------------

Running the ``viz`` command will auto-configure the udp destination of the sensor while leaving the lidar port as
previously set on the sensor. If your sensor is already configured appropriately, you may find it useful to
use the argument ``--no-auto-udp-dest`` to save time by skipping the round trip to reconfigure the
sensor.

Congratulations! You've installed and visualized with the Ouster Python SDK!

.. _Networking Guide: https://docs.ouster.com/sensor-docs/host-networking

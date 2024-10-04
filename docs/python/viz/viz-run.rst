.. _viz-run:

==============================
Running the Ouster visualizer
==============================


After :ref:`installing <installation>` the ``ouster-sdk`` package, you can run::

   $ ouster-cli source <sensor hostname> viz

where ``<sensor hostname>`` is the hostname (os-99xxxxxxxxxx) or IP of the sensor.

Alternately, to replay the existing data from ``pcap`` and ``json`` files call the visualizer as::

   $ ouster-cli source [--meta <meta_path>] <pcap_path> viz

.. figure:: /images/ouster-viz.png
    :align: center

    Ouster SDK CLI ``ouster-cli source <sensor | pcap | osf> viz`` visualization of OS1 128 sample data

The Ouster SDK CLI visualizer does not include a GUI, but can be controlled with the mouse and
keyboard:

* Click and drag rotates the view
* Middle click and drag moves the view
* Scroll adjusts how far away the camera is from the vehicle


.. _simple-viz-keymap:

..
   [start-simple-viz-keymap]

Keyboard Controls
-----------------

**Camera***
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``shift``        Camera Translation with mouse drag
    ``w``            Camera pitch down
    ``s``            Camera pitch up
    ``a``            Camera yaw right
    ``d``            Camera yaw left
    ``R``            Reset camera
    ``ctr-r``        Set camera to the birds-eye view
    ``u``            Toggle camera mode FOLLOW/FIXED
    ``= / -``        Dolly in/out
    ``0``            Toggle orthographic camera
    ================ ===============================================

**Playback**
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``space``        Toggle pause
    ``. / ,``        Step one frame forward/back
    ``ctrl + . / ,`` Step 10 frames forward/back
    ``> / <``        Increase/decrease playback rate (during replay)
    ================ ===============================================

**2D View**
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``b / B``        Cycle top 2D image
    ``n / N``        Cycle bottom 2D image
    ``e / E``        Increase/decrease size of displayed 2D images
    ================ ===============================================

**3D View**
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``p / P``        Increase/decrease point size
    ``m / M``        Cycle point cloud coloring mode
    ``f / F``        Cycle point cloud color palette
    ``ctrl + [N]``   Enable/disable the Nth sensor cloud where N is `1` to `9`
    ``1``            Toggle first return point cloud visibility
    ``2``            Toggle second return point cloud visibility
    ``6``            Toggle scans accumulation view mode (ACCUM)
    ``7``            Toggle overall map view mode (MAP)
    ``8``            Toggle poses/trajectory view mode (TRACK)
    ``2``            Toggle second return point cloud visibility
    ``9``            Show axes
    ``' / "``        Increase/decrease spacing in range markers
    ``ctrl + '``     Increase thickness of range markers
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
    ``esc``          Exit
    ================ ===============================================

..
   [end-simple-viz-keymap]

The visualizer also includes an option to control the orientation of the point cloud in space when
loaded. If you possess, say, an OS-DOME mounted an upside down, you can start the visualizer with
the option ``--extrinsics``::

    $ ouster-cli source --extrinsics -1 0 0 0 0 1 0 0 0 0 -1 0 0 0 0 1 10.0.0.13 viz

The input is a row-major homogeneous matrix.

For other options, run ``ouster-cli source <sensor | pcap | osf> viz -h``

.. note::

   All basic primitives that you see as part of ``ouster-cli`` visualizer are exposed through
   :class:`.viz.PointViz` bindings. Please see :doc:`viz-api-tutorial` for how to use it
   programmatically in Python.


Advanced usage with sensor
--------------------------

The Ouster visualizer automatically configures connected sensors to send data to the appropriate UDP
destination address. If your sensor is already configured appropriately, you may find it useful to
use the argument ``--no-auto-udp-dest`` to save time by skipping the round trip to reconfigure the
sensor.



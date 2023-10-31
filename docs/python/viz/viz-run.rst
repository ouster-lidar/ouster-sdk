.. _viz-run:

==============================
Running the Ouster visualizer
==============================


After :ref:`installing <installation>` the ``ouster-sdk`` package, you can run::

   $ ouster-cli source <sensor hostname> viz

where ``<sensor hostname>`` is the hostname (os-99xxxxxxxxxx) or IP of the sensor.

Alternately, to replay the existing data from ``pcap`` and ``json`` files call the visualizer as::

   $ ouster-cli source <pcap_path> [--meta <meta_path>] viz

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

Keyboard controls:
    ================ ===============================================
        Key          What it does
    ================ ===============================================
    ``o``            Toggle on-screen display
    ``p / P``        Increase/decrease point size
    ``m / M``        Cycle point cloud coloring mode
    ``f / F``        Cycle point cloud color palette
    ``b / B``        Cycle top 2D image
    ``n / N``        Cycle bottom 2D image
    ``R``            Reset camera
    ``ctr-r``        Set camera to the birds-eye view
    ``u``            Toggle camera mode FOLLOW/FIXED
    ``e / E``        Increase/decrease size of displayed 2D images
    ``' / "``        Increase/decrease spacing in range markers
    ``w``            Camera pitch up
    ``s``            Camera pitch down
    ``a``            Camera yaw left
    ``d``            Camera yaw right
    ``1``            Toggle first return point cloud visibility
    ``2``            Toggle second return point cloud visibility
    ``0``            Toggle orthographic camera
    ``= / -``        Dolly in/out
    ``?``            Prints key bindings
    ``space``        Toggle pause
    ``esc``          Exit visualization
    ``. / ,``        Step one frame forward/back
    ``ctrl + . / ,`` Step 10 frames forward/back
    ``> / <``        Increase/decrease playback rate (during replay)
    ``shift``        Camera Translation with mouse drag
    ``shift+z``      Save a screenshot of the current view
    ``shift+x``      Toggle a continuous saving of screenshots
    ``?``            Print keys to standard out
    ================ ===============================================

..
   [end-simple-viz-keymap]

The visualizer also includes an option to control the orientation of the point cloud in space when
loaded. If you possess, say, an OS-DOME mounted an upside down, you can start the visualizer with
the option ``--extrinsics``::

    $ ouster-cli source 10.0.0.13 viz --extrinsics -1 0 0 0 0 1 0 0 0 0 -1 0 0 0 0 1

The input is a row-major homogenous matrix.

For other options, run ``ouster-cli source <sensor | pcap | osf> viz -h``

.. note::

   All basic primitives that you see as part of ``ouster-cli`` visualizer are exposed through
   :class:`.viz.PointViz` bindings. Please see :doc:`viz-api-tutorial` for how to use it
   programmatically in Python.


Advanced usage with sensor
--------------------------

The Ouster visualizer automatically configures connected sensors to send data to the appropriate udp
destination address. If your sensor is already configured appropriately, you may find it useful to
use the argument ``--no-auto-dest`` to save time by skipping the roundtrip to reconfigure the
sensor.



=======================
Point Cloud Visualizer
=======================

The Ouster visualization toolkit is written in C++ with Python bindings for Python functionality. It consists of the following:

- ``simple-viz``: the default Python application visualizer, which can also be used as an entrypoint
  for more sophisticated custom point cloud visualizations
- ``ouster_viz``: the core C++ library 
- ``ouster.sdk.viz``: the Python module for the bindings


Running the Ouster visualizer
=============================

After :ref:`installing <installation>` the ``ouster-sdk`` package, you can run::

   $ simple-viz --sensor <sensor hostname> [--no-auto-dest] [--lidar-port PORT]

where ``<sensor hostname>`` is the hostname (os-99xxxxxxxxxx) or IP of the sensor.

``--no-auto-dest`` option skips the automatic sensor configuration step, which means sensor should
be already configured in a way that UDP packets are sending to the current machine so visualizer
will be able to read packets from a corresponding socket.

To replay the existing data from ``pcap`` and ``json`` files call the visualizer as::

   $ simple-viz --pcap <pcap_path> [--meta <meta_path>]

.. figure:: /images/simple-viz.png
    :align: center

    Ouster ``simple-viz`` visualization of OS1 128 sample data

The sample visualizer does not currently include a GUI, but can be controlled with the mouse and
keyboard:

* Click and drag rotates the view
* Middle click and drag moves the view
* Scroll adjusts how far away the camera is from the vehicle

..
   [start-simple-viz-keymap]

Keyboard controls:
    ==============  ===============================================
        Key         What it does
    ==============  ===============================================
    ``o``           Toggle on-screen display
    ``p/P``         Increase/decrease point size
    ``m``           Cycle point cloud coloring mode
    ``b``           Cycle top 2D image
    ``n``           Cycle bottom 2D image
    ``R``           Reset camera
    ``e/E``         Increase/decrease size of displayed 2D images
    ``'/"``         Increase/decrease spacing in range markers
    ``w``           Camera pitch up
    ``s``           Camera pitch down
    ``a``           Camera yaw left
    ``d``           Camera yaw right
    ``1``           Toggle first return point cloud visibility
    ``2``           Toggle second return point cloud visibility
    ``0``           Toggle orthographic camera
    ``=/-``         Dolly in/out
    ``(space)``     Toggle pause
    ``./,``         Step one frame forward/back
    ``ctrl + ./,``  Step 10 frames forward/back
    ``>/<``         Increase/decrease playback rate (during replay)
    ``shift``       Camera Translation with mouse drag
    ==============  ===============================================

..
   [end-simple-viz-keymap]

For usage and other options, run ``./simple_viz -h``


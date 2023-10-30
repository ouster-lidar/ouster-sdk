=====================
Visualizations in 3D
=====================

The Ouster Python SDK provides two visualization utilities for user convenience. These are
introduced briefly below.

.. contents::
   :local:
   :depth: 3

.. _ex-ouster-viz:


Visualization with Ouster's SDK CLI ``ouster-cli``
==================================================

Ouster's OpenGL-based visualizer allows for easy visualization from pcaps and sensors on all
platforms the Ouster SDK supports.

The default Ouster SDK CLI ``ouster-cli source <sensor | pcap | osf> viz`` visualizer view includes
two 2D range images atop which can be cycled through the available fields, and a 3D point cloud on
the bottom. For dual returns sensors, both returns are displayed by default.

.. figure:: /images/ouster-viz.png
    :align: center

    Ouster SDK CLI ``ouster-cli source <sensor | pcap | osf> viz`` visualization of OS1 128 Rev 7 sample data

The visualizer can be controlled with mouse and keyboard:

.. include:: /python/viz/viz-run.rst
    :start-after: [start-simple-viz-keymap]
    :end-before: [end-simple-viz-keymap]

To run the visualizer with a sensor::

    $ ouster-cli source $SENSOR_HOSTNAME viz

This will auto-configure the udp destination of the sensor while leaving the lidar port as
previously set on the sensor.

To run the visualizer with a pcap::

    $ ouster-cli source $SAMPLE_DATA_PCAP_PATH [--meta $SAMPLE_DATA_JSON_PATH] viz


Visualization with Ouster's :class:`.viz.PointViz`
===================================================

Please refer to :doc:`/python/viz/viz-api-tutorial` for details on extending and customizing
:class:`.viz.PointViz`.


.. _ex-open3d:


Visualization with Open3d
==========================

The `Open3d library`_ contains Python bindings for a variety of tools for working with point cloud
data. Loading data into Open3d is just a matter of reshaping the numpy representation of a point
cloud, as demonstrated in the :func:`.examples.pcap.pcap_3d_one_scan` example:

.. literalinclude:: /../python/src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-open3d-one-scan]
    :end-before: [doc-etag-open3d-one-scan]
    :emphasize-lines: 1-6
    :linenos:
    :dedent:

The :mod:`.examples.open3d` module contains a more fully-featured visualizer built using the Open3d
library, which can be used to replay pcap files or visualize a running sensor. The bulk of the
visualizer is implemented in the :func:`.examples.open3d.viewer_3d` function.

.. note::

   You'll have to install the `Open3d package`_ from PyPI to run this example.


As an example, you can view frame ``84`` from the sample data by running the following command:

.. tabs::

    .. code-tab:: console Linux/macOS

       $ python3 -m ouster.sdk.examples.open3d \
           --pcap $SAMPLE_DATA_PCAP_PATH --meta $SAMPLE_DATA_JSON_PATH --start 84 --pause

    .. code-tab:: powershell Windows x64

       PS > py -3 -m ouster.sdk.examples.open3d ^
           --pcap $SAMPLE_DATA_PCAP_PATH --meta $SAMPLE_DATA_JSON_PATH --start 84 --pause

You may also want to try the ``--sensor`` option to display the output of a running sensor. Use the
``-h`` flag to see a full list of command line options and flags.

Running the example above should open a window displaying a scene from a city intersection,
reproduced below:

.. figure:: /images/lidar_scan_xyz_84_3d.png
   :align: center

   Open3D visualization of OS1 sample data (frame 84). Points colored by ``SIGNAL`` field.

You should be able to click and drag the mouse to look around. You can zoom in and out using the
mouse wheel, and hold control or shift while dragging to pan and roll, respectively.

Hitting the spacebar will start playing back the rest of the pcap in real time. Note that reasonable
performance for realtime playback requires relatively fast hardware, since Open3d runs all rendering
and processing in a single thread.

All of the visualizer controls are listed in the table below:

.. list-table:: Open3d Visualizer Controls
   :widths: 15 30
   :header-rows: 1
   :align: center

   * - Key
     - What it does
   * - **Mouse wheel**
     - Zoom in and out
   * - **Left click + drag**
     - Tilt and rotate the camera
   * - **Ctrl + left click + drag**
     - Pan the camera laterally
   * - **Shift + left click + drag**
     - Roll the camera
   * - **"+" / "-"**
     - Increase or decrease point sizes
   * - **Spacebar**
     - Pause or resume playback
   * - **"M"**
     - Cycle through channel fields used for visualization
   * - **Right arrow key**
     - When reading a pcap, jump 10 frames forward

.. _Open3d library: http://www.open3d.org/
.. _Open3d package: https://pypi.org/project/open3d/

.. _ex-visualization-with-matplotlib:


Visualization with Matplotlib
==============================

You should have defined ``source`` using either a pcap file or UDP data streaming directly from a
sensor, please refer to :doc:`/python/quickstart` for introduction.

.. note::

    Below pictures were rendered using :doc:`OS2 128 Rev 05 Bridge </sample-data>` sample data.

Let's read from ``source`` until we get to the 50th frame of data:

.. code:: python

   from contextlib import closing
   from more_itertools import nth
   with closing(client.Scans(source)) as scans:
       scan = nth(scans, 50)

.. note::

    If you're using a sensor and it takes a few seconds, don't be alarmed! It has to get to the 50th
    frame of data, which would be 5.0 seconds for a sensor running in 1024x10 mode.

We can extract the range measurements from the frame of data stored in the :py:class:`.LidarScan`
datatype and plot a range image where each column corresponds to a single azimuth angle:

.. code:: python

   range_field = scan.field(client.ChanField.RANGE)
   range_img = client.destagger(info, range_field)

We can plot the results using standard Python tools that work with numpy datatypes. Here, we extract
a column segment of the range data and display the result:

.. code:: python

   import matplotlib.pyplot as plt
   plt.imshow(range_img[:, 640:1024], resample=False)
   plt.axis('off')
   plt.show()

.. note::

    If running ``plt.show`` gives you an error about your Matplotlib backend, you will need a `GUI
    backend`_ such as TkAgg or Qt5Agg in order to visualize your data with matplotlib.


.. figure:: /images/brooklyn_bridge_ls_50_range_image.png
    :align: center
    :figwidth: 100%

    Range image of OS2 bridge data. Data taken at Brooklyn Bridge, NYC.


In addition to viewing the data in 2D, we can also plot the results in 3D by projecting the range
measurements into Cartesian coordinates. To do this, we first create a lookup table, then use it to
produce X, Y, Z coordinates from our scan data with shape (H x W x 3):

.. code:: python

    xyzlut = client.XYZLut(info)
    xyz = xyzlut(scan)

Now we rearrange the resulting numpy array into a shape that's suitable for plotting:

.. code:: python

    import numpy as np
    [x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
    ax = plt.axes(projection='3d')
    r = 10
    ax.set_xlim3d([-r, r])
    ax.set_ylim3d([-r, r])
    ax.set_zlim3d([-r/2, r/2])
    plt.axis('off')
    z_col = np.minimum(np.absolute(z), 5)
    ax.scatter(x, y, z, c=z_col, s=0.2)
    plt.show()

.. figure:: /images/brooklyn_bridge_ls_50_xyz_cut.png
   :align: center

   Point cloud from OS2 bridge data with colormap on z. Data taken at Brooklyn Bridge, NYC.

You should be able to rotate the resulting scene to view it from different angles.

To learn more about manipulating lidar data, see:

- :ref:`ex-staggered-and-destaggered`
- :ref:`ex-xyzlut`
- :ref:`ex-correlating-2d-and-3d`


.. _GUI backend: https://matplotlib.org/stable/tutorials/introductory/usage.html#the-builtin-backends

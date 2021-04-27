===================
Examples & Concepts
===================

A loosely connected collection of examples and concepts useful for working with Ouster SDK. If you
are just starting, please see :ref:`quickstart`.

For convenience, throughout the examples and concepts we will use ``pcap_path`` and
``metadata_path`` to refer to the variables from the Quick Start Guide.


.. _ex-metadata:


Obtaining Sensor Metadata
=========================

Ouster sensors require metadata to interpret the readings of the sensor. Represented by the object
:py:class:`.SensorInfo`, its fields include configuration parameters such as 
``lidar_mode`` and and sensor intrinsics like ``beam_azimuth_angles``.

When you work with a sensor, the client will automatically fetch the metadata. Recorded
``pcaps``, however, must always be accompanied by a ``json`` file containing the metadata of the
sensor as it was running when the data was recorded. 

Since it's crucial to save the correct metadata file, let's see how we can get that from a sensor.
Try running the following example::

    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME fetch-metadata

And now let's look inside the example we just ran:

.. literalinclude:: /../src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-fetch-metadata]
    :end-before: [doc-etag-fetch-metadata]
    :dedent:

Seems simple enough!


.. _ex-packets:


Working Directly with Packets
=============================

The :py:class:`.PacketSource` is the basic interface for packets from the sensor. It can be
advantageous to work with packets directly when latency is a concern, or when you wish to examine
the data packet by packet, for example, if you wish to examine timestamps of packets.

Let's make a :py:class:`.PacketSource` from our sample data using :py:class:`.pcap.Pcap`:

.. code:: python

    metadata = client.SensorInfo(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

Now we can read packets from ``source`` with the following code:

.. literalinclude:: /../src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-read-packets]
    :end-before: [doc-etag-pcap-read-packets]
    :dedent:


.. _ex-staggered-and-destaggered:


Staggered vs Destaggered 2D Representations
===========================================

The default **staggered** representation of a :py:class:`.LidarScan` has columns which pertain to
measurements taken at a single timestamp. For a more natural 2D image, we **destagger** the field
with the :py:func:`.client.destagger` function.

Let's take a look at a typical **staggered** representation:

.. figure:: images/lidar_scan_staggered.png
   :align: center

   LidarScan ``RANGE`` field visualized with :py:func:`matplotlib.pyplot.imshow()` and simple gray
   color mapping for better look.

This **staggered** representation definitely doesn't look like a normal image, which shouldn't
surprise us since its columns pertain to timestamps instead of azimuth angles.

Let's destagger the image, changing the columns to represent the azimuth angles:

.. code::

    import matplotlib.pyplot as plt
    from more_itertools import nth

    metadata = client.SensorInfo(metadata_path)
    source = pcap.Pcap(pcap_path, metadata)

    scans = client.Scans(source)

    # iterate `scans` and get the 84th LidarScan
    scan = nth(scans, 84)
    ranges = scan.field(client.ChanField.RANGE)

    # destagger ranges, notice `metadata` use, that is needed to get
    # sensor intrinsics and correctly data transforms
    ranges_destaggered = client.destagger(source.metadata, ranges)

    plt.imshow(ranges_destaggered, cmap='gray', resample=False)

This should give the scene below, of which we have magnified two patches for better visiblity.

.. figure:: images/lidar_scan_destaggered.png
    :align: center

    **destaggered** LidarScan ``RANGE`` field

After destaggering, we can see the scene contains a man on a bicycle, a few cars, and many trees.
This image now makes visual sense, and we can easily use this data in common visual task pipelines.

.. note::

    By the way, you can view this particular scene in both 2D and 3D at Ouster's `Web Slam`_! Use
    your mouse to click and move the 3D scene, and the listed controls to rotate between different
    destaggered image views. The video at the bottom shows the registered point clouds of our
    internal SLAM algorithm. 

.. _Web Slam: https://webslam.ouster.dev/slam/1610482355.9361048.rVdW_dgws/


.. _ex-xyzlut:


Working with 3D Points and the XYZLut
=====================================

To facilitate working with 3D points, you can create a function via :py:func:`.client.XYZLut` which
will project any :py:class:`.LidarScan` into cartesian coordinates by referencing a precalculated XYZ
Look-uptable. This function can then be applied to any scan to create a numpy array of ``H x W x
3``, which represents the cartesian coordintes of the points in the sensor coordinate frame.

.. literalinclude:: /../src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-plot-xyz-points]
    :end-before: [doc-etag-plot-xyz-points]
    :emphasize-lines: 2-5
    :linenos:
    :dedent:

If you have a sensor, you can run this code with one of our examples::

    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME plot-xyz-points

That should open a 3D plot of a single scan of your location taken just now by your sensor. You
should be able to recognize the contours of the scene around you.

If you don’t have a sensor, you can run this code with our pcap examples::

    $ python -m ouster.sdk.examples.pcap OS1_128.pcap OS1_2048x10_128.json \
      plot-xyz-points --scan-num 84


.. figure:: images/lidar_scan_xyz_84.png
   :align: center

   Point cloud from sample data (scan 84). Points colored by ``SIGNAL`` value.

For details check the source code of an example :func:`.examples.pcap.pcap_display_xyz_points`

.. _ex-correlating-2d-and-3d:

Working with 2D and 3D Representations Simultaneously
=====================================================

The direct correlation between 2D and 3D representations in an Ouster sensor provides a powerful
framework for working with the data. As an easy example, you might decide you want to look at only
the 3D points within a certain range and from certain azimuth angles.

.. literalinclude:: /../src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-filter-3d]
    :end-before: [doc-etag-filter-3d]
    :emphasize-lines:  10-11, 15
    :linenos:

Since we'd like to filter on azimuth angles, first we first destagger both the 2D and 3D points, so
that our columns in the ``HxW`` representation correspond to azimuth angle, not timestamp. (See
:ref:`ex-staggered-and-destaggered` for an explanation on destaggering.)

Then we filter the 3D points ``xyz_destaggered`` by comparing the range measurement to
``min_range``, which we can do because there is a 1:1 correspondence between the columns and rows of
the destaggered representations of ``xyz_destaggered`` and ``range_staggered``. (Similarly, there
would be a 1:1 correspondence between the staggered representations ``xyz`` and ``range``, where the
columns correspond with timestamp).

Finally, we select only the azimuth columns we're interested in. In this case, we've arbitrarily
chosen the first 270 degrees of rotation.

If you have a sensor, you can run this code with an example::
    
    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME filter-3d-by-range-and-azimuth

.. _ex-streaming:


Streaming Live Data
===================

Instead of working with a recorded dataset or a few captured frames of data, let's see if we can get
a live feed from the sensor::
    
    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME live-plot-signal

This should give you a live feed from your sensor that looks like a black and white moving image.
Try waving your hand or moving around to find yourself within the image!

So how did we do that?

.. literalinclude:: /../src/ouster/sdk/examples/client.py
   :start-after: [doc-stag-live-plot-signal]
   :end-before: [doc-etag-live-plot-signal]
   :emphasize-lines: 2-3
   :linenos:
   :dedent:

Notice that instead of taking a ``sample`` as we did in previous example, we used
:py:meth:`.Scans.stream`, which allows for a continuous live data stream.  We close the ``stream``
when we are finished, hence the use of :py:func:`.closing` in the highlighted line.

To exit the visualization, you can use ``ESC``.


.. _ex-pcap-record:


Recording Sensor Data
=====================

It's easy to record data to a pcap file from a sensor programatically. Let's try it first
with the following example::

    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME record-pcap

This will capture the :class:`.client.LidarPacket`'s and :class:`.client.ImuPacket`'s data for 10
seconds and store the pcap file along with the metadata json file into the current directory.

The source code of an example below:

.. literalinclude:: /../src/ouster/sdk/examples/client.py
   :start-after: [doc-stag-pcap-record]
   :end-before: [doc-etag-pcap-record]
   :emphasize-lines: 15
   :linenos:
   :dedent:

Good! The resulting pcap and json files can be used with any examples in the :mod:`.examples.pcap`
module.


.. _ex-pcap-live-preview:


Pcap Live Data Preview
=======================

We can easily view the data that was recorded in the previous section. Based on an example from
:ref:`ex-streaming` above we have a pcap viewer with additional *stagger*/*destagger* handler on key
`D` and pause on `SPACE` key  (source code: :func:`.examples.pcap.pcap_2d_viewer`). To run it try
the following command::

    $ python -m ouster.sdk.examples.pcap OS1_128.pcap OS1_2048x10_128.json 2d-viewer

Or substitute example data with pcap and json that you just recorded.


.. _ex-pcap-to-csv:


Pcap to CSV
=======================

Sometimes we want to get a point cloud (``XYZ`` + other fields) as a ``CSV`` file for further
analysis with other tools.

To convert the first ``5`` scans of sample data from a pcap file, you can try::

    $ python -m ouster.sdk.examples.pcap OS1_128.pcap OS1_2048x10_128.json \
      pcap-to-csv --scan-num 5

The source code of an example below:

.. literalinclude:: /../src/ouster/sdk/examples/pcap.py
    :start-after: [doc-stag-pcap-to-csv]
    :end-before: [doc-etag-pcap-to-csv]
    :emphasize-lines: 37-41
    :linenos:
    :dedent:

Because we stored the scan as structured 2D images, we can easily recover it by loading it back into
a ``numpy.ndarray`` and continuing to use it as a 2D image.

.. code:: python

    import numpy as np

    # read array from CSV
    frame = np.loadtxt('my_frame_00000.csv', delimiter=',')

    # convert back to "fat" 2D image [H x W x 7] shape
    frame = frame.reshape((128, -1, frame.shape[1]))

We used ``128`` while restoring 2D image from a CSV file because it's the number of channels of our
``OS-1-128.pcap`` sample data recording.

Check :func:`.examples.pcap.pcap_to_csv` documentation for further details.

.. _ex-imu:


Working with IMU data from the Ouster sensor
============================================
IMU data from the Ouster sensor can be read as :py:class:`~.client.ImuPacket`. Let's do something
easy, like graph the acceleration in z direction over time. Let's look at some code:

.. literalinclude:: /../src/ouster/sdk/examples/client.py
    :start-after: [doc-stag-imu-z-accel]
    :end-before: [doc-etag-imu-z-accel]
    :emphasize-lines: 4-6
    :linenos:

Like other ``Packets``, we'll want to get them from a :py:class:`.PacketSource`. After getting ``imu_packet_list``, we obtain the ``sys_ts`` and ``z`` part of ``accel`` and plot them.

If you have a sensor, you can run the code above with the ``plot-imu-z-accel`` example::

    $ python -m ouster.sdk.examples.client $SENSOR_HOSTNAME plot-imu-z-accel

